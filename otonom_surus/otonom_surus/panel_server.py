"""
Web kontrol paneli sunucusu: tarayıcıya arayüzü ve canlı veriyi sunar, koşuları başlatır.

Tek süreçte hem ROS düğümü hem HTTP sunucusu çalışır. ROS ana iş parçacığında döner; HTTP
istekleri ThreadingHTTPServer'ın iş parçacıklarında karşılanır. Canlı veri tarayıcıya
Server-Sent Events (SSE) ile akar: bağlı her sekmenin bir kuyruğu vardır, Broadcaster olayları
bütün kuyruklara dağıtır, /api/stream isteği kendi kuyruğunu boşaltır. Koşuları panel_process'in
RunManager'ı alt süreç olarak başlatır ve durdurur; ROS konularını panel_live'ın LiveBridge'i
önbelleğe alıp hız sınırlı olaylar olarak gönderir; geçmiş koşuları panel_runs'ın RunIndex'i
results/ altından okur. Python standart kütüphanesi ve Pillow yeter.

Sunucu yalnızca 127.0.0.1'e bağlanır (kimlik doğrulama yok); WSL2'nin localhost yönlendirmesi
sayesinde Windows tarayıcısından http://localhost:<port> ile açılır.

Uç noktalar:
    GET  /                web/ klasöründeki arayüz (index.html, app.js, style.css)
    GET  /api/health      sunucu durumu
    GET  /api/stream      SSE: hello (durum, son günlük, canlı verinin son hali), ping, run, log,
                          run_finished, runs_changed; canlı: status, pose, trail, plan, map
    GET  /api/worlds      paketteki dünyaların geometrisi (world_sdf)
    GET  /api/scenarios   scenarios.yaml'daki senaryolar (hedef yaw'ları derece)
    GET  /api/run         koşunun durumu
    POST /api/run/start   {"scenario": 1, "gui": true, "rviz": false}
    POST /api/run/stop
    GET  /api/runs        geçmiş koşular, en yeni önce (?world=senaryo2.world)
    GET  /api/runs/<kimlik>             tek koşu (kimlik: s1_r1, parametre_fpd2/s2_r1 ...)
    GET  /api/runs/<kimlik>/trajectory  yörünge [[t, x, y], ...] (?dt=0.5: seyreltilmiş)

Kullanım:
    ros2 launch otonom_surus panel.launch.py
"""

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
import queue
import signal
import threading
import time
from urllib.parse import parse_qs, unquote, urlsplit
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from otonom_surus.panel_live import LiveBridge
from otonom_surus.panel_process import RunError, RunManager
from otonom_surus.panel_runs import RunIndex
from otonom_surus.scenarios import load_scenario, scenario_numbers
from otonom_surus.world_sdf import world_geometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

VERSION = '0.4.0'
HOST = '127.0.0.1'  # bilerek parametre değil: panelin ağa açılmaması için (PT18)
PING_PERIOD = 5.0  # s; istemci 10 s veri alamazsa bağlantıyı kopmuş sayar
SSE_RETRY_MS = 3000  # tarayıcının kopunca yeniden bağlanma aralığı
CLIENT_QUEUE = 256  # sekme başına bekleyen olay sınırı; dolunca en eski düşer
MAX_BODY = 64 * 1024  # POST gövdesi sınırı

MIME_TYPES = {
    '.html': 'text/html; charset=utf-8',
    '.js': 'text/javascript; charset=utf-8',
    '.css': 'text/css; charset=utf-8',
    '.json': 'application/json; charset=utf-8',
    '.svg': 'image/svg+xml',
    '.png': 'image/png',
    '.ico': 'image/x-icon',
}


def resolve_static(root, url_path):
    """
    URL yolunu root altındaki bir dosyaya çevirir; root dışına çıkan ya da olmayan yolda None.

    '/' ile biten yol index.html'dir. Yüzde kodlaması çözülür, sorgu ve parça atılır. Sınır
    denetimi sözcük düzeyinde (normpath) yapılır, realpath ile değil: --symlink-install
    kurulumunda share/otonom_surus/web/ altındaki dosyalar kaynak klasörüne sembolik bağlantıdır
    ve bunlar sunulabilmelidir; '..' ile root dışına ise çıkılamaz.
    """
    path = unquote(urlsplit(url_path).path)
    if '\x00' in path:
        return None
    if path.endswith('/'):
        path += 'index.html'
    root = os.path.normpath(root)
    full = os.path.normpath(os.path.join(root, path.lstrip('/')))
    if os.path.commonpath([root, full]) != root or not os.path.isfile(full):
        return None
    return full


def sse_event(event, data):
    """Bir SSE çerçevesi: 'event: <ad>', tek satır JSON 'data:' ve boş satır (UTF-8)."""
    payload = json.dumps(data, ensure_ascii=False, separators=(',', ':'))
    return f'event: {event}\ndata: {payload}\n\n'.encode('utf-8')


def runs_route(index, path, query):
    """
    Koşu dizininin uçları (RunIndex): (HTTP durum kodu, JSON verisi); bilinmeyen yolda None.

    Kimlikte '/' bulunabilir (parametre_fpd2/s2_r1); yüzde kodlaması çözülür. Kimlik yalnızca
    dizinde aranır, dosya yolu kullanıcı girdisinden kurulmaz.
    """
    if path == '/api/runs':
        return 200, {'root': index.root, 'runs': index.runs(query.get('world') or None),
                     'warnings': index.warnings()}
    if not path.startswith('/api/runs/'):
        return None
    rest = unquote(path[len('/api/runs/'):])
    if rest.endswith('/trajectory'):
        run_id = rest[:-len('/trajectory')]
        try:
            dt = float(query.get('dt') or 0)
        except ValueError:
            return 400, {'error': 'dt sayı olmalı'}
        try:
            points = index.trajectory(run_id, dt if math.isfinite(dt) and dt > 0 else None)
        except KeyError:
            return 404, {'error': f'koşu bulunamadı: {run_id}'}
        except FileNotFoundError:
            return 404, {'error': f'{run_id}: yörünge kaydı yok'}
        return 200, {'id': run_id, 'pts': points}
    record = index.get(rest)
    if record is None:
        return 404, {'error': f'koşu bulunamadı: {rest}'}
    return 200, record


class Broadcaster:
    """Olayları bağlı her SSE istemcisinin kuyruğuna dağıtır; dolan kuyrukta en eski olay düşer."""

    def __init__(self, maxsize=CLIENT_QUEUE):
        self._lock = threading.Lock()
        self._clients = set()
        self._maxsize = maxsize
        self.closed = False

    def register(self):
        q = queue.Queue(self._maxsize)
        with self._lock:
            self._clients.add(q)
            if self.closed:
                q.put_nowait(None)
        return q

    def unregister(self, q):
        with self._lock:
            self._clients.discard(q)

    def count(self):
        with self._lock:
            return len(self._clients)

    def publish(self, event, data):
        frame = sse_event(event, data)
        with self._lock:
            clients = list(self._clients)
        for q in clients:
            self._put(q, frame)

    def close(self):
        """Kapanışı bildirir: her kuyruğa None konur, akış döngüleri bağlantıyı kapatır."""
        with self._lock:
            self.closed = True
            clients = list(self._clients)
        for q in clients:
            self._put(q, None)

    @staticmethod
    def _put(q, item):
        while True:
            try:
                q.put_nowait(item)
                return
            except queue.Full:
                try:
                    q.get_nowait()
                except queue.Empty:
                    pass


class PanelHandler(BaseHTTPRequestHandler):
    """
    HTTP istekleri.

    self.server.panel; web_dir, broadcaster, hello(), api(method, path, body) ve log_request()
    sağlayan nesnedir: çalışırken PanelServer düğümü, testlerde onun yerine geçen basit bir nesne.
    api() bilinmeyen uçta None, bilinende (HTTP durum kodu, JSON verisi) döndürür. body,
    POST'ta JSON gövdesi, GET'te sorgu parametreleridir ({ad: son değer}).
    """

    server_version = 'OtonomPanel/' + VERSION

    def do_GET(self):  # noqa: N802 (http.server'ın adlandırması)
        url = urlsplit(self.path)
        if url.path == '/api/stream':
            self.stream()
        elif url.path.startswith('/api/'):
            query = {k: v[-1] for k, v in parse_qs(url.query).items()}
            self.api('GET', url.path, query)
        else:
            self.send_static()

    def do_POST(self):  # noqa: N802
        path = urlsplit(self.path).path
        body, error = self.read_json()
        if error:
            self.send_json({'error': error}, 400)
        else:
            self.api('POST', path, body)

    def api(self, method, path, body):
        result = self.server.panel.api(method, path, body)
        if result is None:
            self.send_json({'error': f'bilinmeyen uç nokta: {method} {path}'}, 404)
        else:
            self.send_json(result[1], result[0])

    def read_json(self):
        """POST gövdesi: (sözlük, None) ya da (None, hata iletisi); boş gövde {}."""
        try:
            length = int(self.headers.get('Content-Length') or 0)
        except ValueError:
            return None, 'Content-Length geçersiz'
        if length > MAX_BODY:
            return None, 'gövde çok büyük'
        raw = self.rfile.read(length) if length else b''
        if not raw.strip():
            return {}, None
        try:
            body = json.loads(raw)
        except (UnicodeDecodeError, json.JSONDecodeError):
            return None, 'gövde JSON değil'
        if not isinstance(body, dict):
            return None, 'gövde bir JSON nesnesi olmalı'
        return body, None

    def send_json(self, data, status=200):
        # send_error kullanılmıyor: durum satırına latin-1 yazar, Türkçe ileti hata verir.
        body = json.dumps(data, ensure_ascii=False).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def send_static(self):
        full = resolve_static(self.server.panel.web_dir, self.path)
        if full is None:
            self.send_json({'error': 'dosya bulunamadı'}, 404)
            return
        with open(full, 'rb') as f:
            body = f.read()
        self.send_response(200)
        self.send_header(
            'Content-Type', MIME_TYPES.get(os.path.splitext(full)[1], 'application/octet-stream'))
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-cache')  # web/ değişince yenile yeter
        self.end_headers()
        self.wfile.write(body)

    def stream(self):
        """SSE: önce retry ve hello, sonra kuyruktaki olaylar; olay yoksa PING_PERIOD'da ping."""
        panel = self.server.panel
        q = panel.broadcaster.register()
        try:
            self.send_response(200)
            self.send_header('Content-Type', 'text/event-stream; charset=utf-8')
            self.send_header('Cache-Control', 'no-cache')
            self.end_headers()
            self.wfile.write(f'retry: {SSE_RETRY_MS}\n\n'.encode('ascii'))
            self.wfile.write(sse_event('hello', panel.hello()))
            self.wfile.flush()
            while True:
                try:
                    frame = q.get(timeout=PING_PERIOD)
                except queue.Empty:
                    frame = sse_event('ping', {'t': round(time.time(), 1)})
                if frame is None:  # sunucu kapanıyor
                    break
                self.wfile.write(frame)
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            pass  # sekme kapandı ya da yenilendi
        finally:
            panel.broadcaster.unregister(q)

    def log_message(self, format, *args):  # noqa: A002 (üst sınıfın imzası)
        self.server.panel.log_request(self.address_string(), format % args)


def make_server(panel, host, port):
    """HTTP sunucusu; port 0 ise boş bir port seçilir (testler)."""
    server = ThreadingHTTPServer((host, port), PanelHandler)  # daemon_threads: True
    server.panel = panel
    return server


class PanelServer(Node):
    """Panelin ROS tarafı: parametreler, API, koşu yöneticisi, koşu dizini ve SSE dağıtıcısı."""

    def __init__(self):
        super().__init__('panel_server')
        share = get_package_share_directory('otonom_surus')
        default_scenarios = os.path.join(share, 'config', 'scenarios.yaml')
        self.port = self.declare_parameter('port', 8080).value
        self.results_dir = os.path.abspath(os.path.expanduser(
            self.declare_parameter('results_dir', 'results/panel').value))
        # Geçmiş koşuların kökü; boşsa results_dir'in üst klasörü (results/panel -> results).
        runs_root = self.declare_parameter('runs_root', '').value
        self.runs_root = (os.path.abspath(os.path.expanduser(runs_root)) if runs_root
                          else os.path.dirname(self.results_dir))
        self.web_dir = self.declare_parameter('web_dir', os.path.join(share, 'web')).value
        self.scenarios_file = self.declare_parameter('scenarios_file', default_scenarios).value
        self.worlds_dir = os.path.join(share, 'worlds')
        self.broadcaster = Broadcaster()
        self.index = RunIndex(
            self.runs_root, panel_dir=self.results_dir,
            base_params=os.path.join(share, 'config', 'nav2_params.yaml'),
            scenario_files=[self.scenarios_file,
                            os.path.join(self.results_dir, 'panel_scenarios.yaml')])
        self.announced = set()  # bittiği runs_changed ile duyurulan görevler (run_id)
        self.live = LiveBridge(self, self.broadcaster.publish, on_status=self.on_mission_status,
                               movers=self.mover_names())
        # scenarios_file varsayılansa komuta yazılmaz (launch'ın varsayılanı aynı dosya).
        custom = None if self.scenarios_file == default_scenarios else self.scenarios_file
        self.runs = RunManager(self.results_dir, self.on_run_event,
                               scenarios_file=custom, cwd=os.getcwd())
        self.started = time.monotonic()

    def mover_names(self):
        """Bütün senaryolardaki hareketli kutuların adları (canlı konumları için abone olunur)."""
        names = set()
        try:
            for number in scenario_numbers(self.scenarios_file):
                names.update(o['name'] for o in
                             load_scenario(self.scenarios_file, number)['moving_obstacles'])
        except (OSError, ValueError) as e:
            self.get_logger().warn(f'{self.scenarios_file}: {e}; hareketli kutular izlenmeyecek')
        return sorted(names)

    def on_run_event(self, event, data):
        """
        RunManager'ın olayları; yeni koşu başlarken önceki koşunun canlı verisi silinir.

        run_finished'e koşunun dizindeki kimliği (index_id) eklenir: istemci listeyi yeniler
        ve o satırı "az önce" diye işaretler.
        """
        if event == 'run' and data.get('phase') == 'starting':
            self.live.reset()
        if event == 'run_finished':
            data = dict(data, index_id=self.index.id_for(
                self.results_dir, data.get('scenario'), data.get('run_id')))
        self.broadcaster.publish(event, data)

    def on_mission_status(self, status):
        """
        /mission_status: koşu aşaması RunManager'a; biten görev runs_changed ile duyurulur.

        mission_manager 'finished'i sonuçları yazdıktan sonra yayınlar; panelin koşusu da panel
        dışından başlatılan görev de böylece launch kapanmayı beklemeden listeye düşer. Görev
        başına bir kez (bittikten sonra durum 1 Hz'de tekrarlanır).
        """
        self.runs.on_status(status)
        run_id = status.get('run_id')
        if status.get('state') == 'finished' and run_id not in self.announced:
            self.announced.add(run_id)
            self.broadcaster.publish('runs_changed', {'id': self.index.id_for(
                status.get('results_dir') or '', status.get('scenario'), run_id)})

    def health(self):
        return {
            'ok': True,
            'version': VERSION,
            'node': self.get_name(),
            'ros': rclpy.ok(),
            'sim': self.runs.state()['phase'],
            'results_dir': self.results_dir,
            'runs_root': self.runs_root,
            'clients': self.broadcaster.count(),
            'uptime_s': round(time.monotonic() - self.started, 1),
        }

    def hello(self):
        """Yeni bağlanan sekmeye: sağlık, koşunun durumu, son günlük ve canlı verinin son hali."""
        return {**self.health(), 'run': self.runs.state(), 'log': self.runs.recent_log(),
                'live': self.live.snapshot()}

    def api(self, method, path, body):
        if method == 'GET':
            handlers = {'/api/health': lambda: (200, self.health()),
                        '/api/worlds': self.worlds,
                        '/api/scenarios': self.scenarios,
                        '/api/run': lambda: (200, self.runs.state())}
            handler = handlers.get(path)
            return handler() if handler else runs_route(self.index, path, body or {})
        if method == 'POST' and path == '/api/run/start':
            return self.start_run(body)
        if method == 'POST' and path == '/api/run/stop':
            try:
                return 200, self.runs.stop()
            except RunError as e:
                return 409, {'error': str(e)}
        return None

    def worlds(self):
        worlds = []
        for name in sorted(os.listdir(self.worlds_dir)):
            if not name.endswith('.world'):
                continue
            try:
                worlds.append({'file': name,
                               'geometry': world_geometry(os.path.join(self.worlds_dir, name))})
            except (ET.ParseError, OSError, ValueError) as e:
                worlds.append({'file': name, 'error': str(e)})
        return 200, worlds

    def scenarios(self):
        try:
            result = []
            for number in scenario_numbers(self.scenarios_file):
                s = load_scenario(self.scenarios_file, number)
                result.append({
                    'id': number, 'name': s['name'], 'world': s['world'],
                    'goals': [[x, y, round(math.degrees(yaw), 1)] for x, y, yaw in s['goals']],
                    'moving_obstacles': s['moving_obstacles'],
                })
        except (OSError, ValueError) as e:
            return 500, {'error': f'{self.scenarios_file}: {e}'}
        return 200, result

    def start_run(self, body):
        try:
            scenario = int(body.get('scenario'))
        except (TypeError, ValueError):
            return 400, {'error': 'scenario tamsayı olmalı'}
        try:
            known = scenario_numbers(self.scenarios_file)
        except OSError as e:
            return 500, {'error': f'{self.scenarios_file}: {e}'}
        if scenario not in known:
            return 400, {'error': f'senaryo {scenario} tanımlı değil (tanımlı: {known})'}
        try:
            state = self.runs.start(scenario, gui=bool(body.get('gui', True)),
                                    rviz=bool(body.get('rviz', False)))
        except RunError as e:
            return 409, {'error': str(e)}
        self.get_logger().info(f'Koşu başladı: {state["command"]}')
        return 200, state

    def log_request(self, client, text):
        self.get_logger().debug(f'{client} {text}')


def main(args=None):
    """
    ROS ana iş parçacığında döner, HTTP sunucusu ayrı bir iş parçacığında.

    Tersi (ROS ayrı iş parçacığında) Ctrl+C'de düğüm, yürütücü hâlâ rcl'nin bekleme kümesindeyken
    yok edilebiliyordu: "terminate called without an active exception" ve süreç SIGKILL'e kadar
    asılı kaldı (PT21). Kapanış sırası: önce çalışan koşu durdurulur (alt süreçler kendi
    oturumlarında olduğundan terminaldeki Ctrl+C onlara ulaşmaz), sonra SSE akışları ve HTTP,
    en son düğüm.
    """
    rclpy.init(args=args)
    node = PanelServer()
    try:
        server = make_server(node, HOST, node.port)
    except OSError as e:
        node.get_logger().error(
            f'{HOST}:{node.port} açılamadı ({e.strerror}); başka port: port:=8090')
        node.destroy_node()
        rclpy.try_shutdown()
        return 1
    http_thread = threading.Thread(
        target=server.serve_forever, kwargs={'poll_interval': 0.5}, daemon=True)
    http_thread.start()
    node.get_logger().info(
        f'Panel hazır: http://localhost:{node.port} (sonuçlar: {node.results_dir})')
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Terminalde Ctrl+C bütün süreç grubuna gider, launch da SIGINT'i ayrıca iletir; koşu
        # durdurulurken gelen ikinci sinyal (ya da launch'ın SIGTERM'i) temizliği kesmesin.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        node.runs.shutdown()
        node.broadcaster.close()  # açık SSE akışları kuyruktan None alıp çıkar
        server.shutdown()
        server.server_close()
        http_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.try_shutdown()
        signal.signal(signal.SIGINT, signal.SIG_IGN)  # try_shutdown eski işleyiciyi geri koyar
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
    return 0


if __name__ == '__main__':
    main()
