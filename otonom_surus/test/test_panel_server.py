"""panel_server'ın ROS gerektirmeyen birim testleri: statik dosya, SSE ve HTTP uçları."""

import http.client
import json
import os
import threading

from otonom_surus.panel_runs import RUN_FIELDS, RunIndex
from otonom_surus.panel_server import (
    Broadcaster, make_server, resolve_static, runs_route, sse_event)
import pytest


def test_kok_ve_sorgulu_yol_index_html(tmp_path):
    (tmp_path / 'index.html').write_text('<!doctype html>')
    assert resolve_static(str(tmp_path), '/') == str(tmp_path / 'index.html')
    assert resolve_static(str(tmp_path), '/?v=2#ust') == str(tmp_path / 'index.html')
    assert resolve_static(str(tmp_path), '/index.html') == str(tmp_path / 'index.html')


def test_kok_disina_cikilamaz(tmp_path):
    web = tmp_path / 'web'
    web.mkdir()
    (tmp_path / 'gizli.txt').write_text('gizli')
    for path in ('/../gizli.txt', '/%2e%2e/gizli.txt', '/alt/../../gizli.txt',
                 '//../gizli.txt', '/%2E%2E%2Fgizli.txt', '/gizli.txt%00.js'):
        assert resolve_static(str(web), path) is None, path


def test_olmayan_dosya_ve_klasor_none(tmp_path):
    (tmp_path / 'alt').mkdir()
    assert resolve_static(str(tmp_path), '/yok.js') is None
    assert resolve_static(str(tmp_path), '/alt') is None
    assert resolve_static(str(tmp_path), '/alt/') is None  # alt/index.html yok


def test_kurulumdaki_sembolik_baglanti_sunulur(tmp_path):
    # --symlink-install: share/.../web/app.js kaynak klasöre bağlantıdır, kök dışını gösterir.
    src = tmp_path / 'src'
    src.mkdir()
    (src / 'app.js').write_text('// kaynak')
    web = tmp_path / 'install' / 'web'
    web.mkdir(parents=True)
    os.symlink(src / 'app.js', web / 'app.js')
    assert resolve_static(str(web), '/app.js') == str(web / 'app.js')


def test_sse_cercevesi_tek_satir_utf8():
    frame = sse_event('hello', {'ad': 'Çağrı', 'satır': 'a\nb'})
    assert frame == 'event: hello\ndata: {"ad":"Çağrı","satır":"a\\nb"}\n\n'.encode('utf-8')


def test_dagitici_her_istemciye_iletir_ve_kapanir():
    b = Broadcaster()
    q1, q2 = b.register(), b.register()
    assert b.count() == 2
    b.publish('ping', {'t': 1})
    assert q1.get_nowait() == q2.get_nowait() == sse_event('ping', {'t': 1})
    b.unregister(q2)
    b.close()
    assert q1.get_nowait() is None
    assert q2.empty()
    assert b.register().get_nowait() is None  # kapandıktan sonra bağlanan da hemen çıkar


def test_dolu_kuyrukta_en_eski_olay_duser():
    b = Broadcaster(maxsize=2)
    q = b.register()
    for t in range(3):
        b.publish('ping', {'t': t})
    assert [q.get_nowait() for _ in range(2)] == [sse_event('ping', {'t': t}) for t in (1, 2)]


class FakePanel:
    """PanelServer'ın HTTP tarafının kullandığı arayüz, ROS'suz; koşu dizini gerçek."""

    def __init__(self, web_dir, index):
        self.web_dir = web_dir
        self.broadcaster = Broadcaster()
        self.index = index

    def hello(self):
        return {'ok': True, 'version': 'test', 'clients': self.broadcaster.count()}

    def api(self, method, path, body):
        if method == 'GET' and path == '/api/health':
            return 200, self.hello()
        if path == '/api/echo':  # POST'ta gövde, GET'te sorgu parametreleri
            return (201 if method == 'POST' else 200), body
        if method == 'GET':
            return runs_route(self.index, path, body)
        return None

    def log_request(self, client, text):
        pass


@pytest.fixture
def server(tmp_path):
    web = tmp_path / 'web'
    web.mkdir()
    (web / 'index.html').write_text('<!doctype html><title>Panel</title>', encoding='utf-8')
    (web / 'app.js').write_text('// istemci', encoding='utf-8')
    (tmp_path / 'gizli.txt').write_text('gizli')
    runs = tmp_path / 'results' / 'alt'
    (runs / 'trajectories').mkdir(parents=True)
    (runs / 'runs.csv').write_text(
        ','.join(RUN_FIELDS) + '\n'
        '2026-10-15T14:17:29,1,r1,SUCCEEDED,1,1,28.1,28.2,5.7,5.6,0.98,0.3,0,\n'
        '2026-10-15T14:20:00,2,r2,FAILED,0,3,9.0,9.1,1.0,12.0,,0.3,2,"hedef 1: FAILED, kala"\n',
        encoding='utf-8')
    (runs / 'trajectories' / 's1_r1.csv').write_text(
        't,x,y\n0.0,0.0,0.0\n0.5,0.1,0.0\n1.0,0.2,0.0\n1.5,0.3,0.0\n')
    srv = make_server(FakePanel(str(web), RunIndex(str(tmp_path / 'results'))), '127.0.0.1', 0)
    thread = threading.Thread(target=srv.serve_forever, kwargs={'poll_interval': 0.05})
    thread.start()
    yield srv
    srv.panel.broadcaster.close()
    srv.shutdown()
    srv.server_close()
    thread.join(timeout=5)


def request(srv, method, path, body=None):
    conn = http.client.HTTPConnection('127.0.0.1', srv.server_address[1], timeout=5)
    conn.request(method, path, body=body, headers={'Content-Type': 'application/json'})
    return conn.getresponse()


def test_http_statik_dosyalar_ve_turleri(server):
    r = request(server, 'GET', '/')
    assert r.status == 200
    assert r.getheader('Content-Type') == 'text/html; charset=utf-8'
    assert b'<title>Panel</title>' in r.read()
    r = request(server, 'GET', '/app.js')
    assert r.status == 200
    assert r.getheader('Content-Type').startswith('text/javascript')


def test_http_kok_disi_ve_bilinmeyen_uc_404(server):
    for method, path in (('GET', '/../gizli.txt'), ('GET', '/%2e%2e/gizli.txt'),
                         ('GET', '/yok.css'), ('GET', '/api/yok'), ('POST', '/api/run/start')):
        r = request(server, method, path)
        assert r.status == 404, path
        assert r.getheader('Content-Type') == 'application/json; charset=utf-8'
        assert 'error' in json.loads(r.read())


def test_http_saglik(server):
    r = request(server, 'GET', '/api/health')
    assert r.status == 200
    assert json.loads(r.read())['ok'] is True


def test_http_post_govdesi(server):
    r = request(server, 'POST', '/api/echo', json.dumps({'görev': 2}).encode('utf-8'))
    assert r.status == 201
    assert json.loads(r.read()) == {'görev': 2}
    r = request(server, 'POST', '/api/echo')  # boş gövde -> {}
    assert r.status == 201 and json.loads(r.read()) == {}
    for bad in (b'{bozuk', b'[1, 2]', b'\xff\xfe'):
        r = request(server, 'POST', '/api/echo', bad)
        assert r.status == 400, bad
        assert 'error' in json.loads(r.read())


def test_http_get_sorgu_parametreleri(server):
    query = 'world=senaryo2.world&dt=0.5&dt=1&ad=%C3%87a%C4%9Fr%C4%B1'
    r = request(server, 'GET', '/api/echo?' + query)
    assert json.loads(r.read()) == {'world': 'senaryo2.world', 'dt': '1', 'ad': 'Çağrı'}


def body(response, status=200):
    assert response.status == status
    return json.loads(response.read())


def test_kosu_dizini_uclari(server):
    d = body(request(server, 'GET', '/api/runs'))
    assert d['root'].endswith('results') and d['warnings'] == []
    assert [r['id'] for r in d['runs']] == ['alt/s2_r2', 'alt/s1_r1']
    assert d['runs'][0]['failure_reason'] == 'hedef 1: FAILED, kala'
    assert body(request(server, 'GET', '/api/runs?world=yok.world'))['runs'] == []
    assert body(request(server, 'GET', '/api/runs/alt/s1_r1'))['duration_s'] == 28.1

    # Kimlikteki '/' yüzde kodlu da gelebilir.
    d = body(request(server, 'GET', '/api/runs/alt%2Fs1_r1/trajectory'))
    assert d['id'] == 'alt/s1_r1' and len(d['pts']) == 4
    d = body(request(server, 'GET', '/api/runs/alt/s1_r1/trajectory?dt=1'))
    assert [p[0] for p in d['pts']] == [0.0, 1.0, 1.5]
    r = request(server, 'GET', '/api/runs/alt/s1_r1/trajectory?dt=x')
    assert 'dt' in body(r, 400)['error']

    assert 'yörünge kaydı yok' in body(
        request(server, 'GET', '/api/runs/alt/s2_r2/trajectory'), 404)['error']
    for path in ('/api/runs/alt/s9_yok', '/api/runs/../gizli.txt/trajectory',
                 '/api/runs/alt/../../gizli.txt'):
        assert 'bulunamadı' in body(request(server, 'GET', path), 404)['error'], path


def test_sse_hello_olay_ve_kapanis(server):
    r = request(server, 'GET', '/api/stream')
    assert r.status == 200
    assert r.getheader('Content-Type') == 'text/event-stream; charset=utf-8'
    assert r.readline() == b'retry: 3000\n'
    assert r.readline() == b'\n'
    assert r.readline() == b'event: hello\n'
    hello = json.loads(r.readline()[len(b'data: '):])
    assert hello['clients'] == 1
    assert r.readline() == b'\n'

    server.panel.broadcaster.publish('durum', {'görev': 'S1'})
    assert r.readline() == b'event: durum\n'
    assert json.loads(r.readline()[len(b'data: '):]) == {'görev': 'S1'}
    assert r.readline() == b'\n'

    server.panel.broadcaster.close()
    assert r.read() == b''  # sunucu kapanınca akış biter
