"""
Web kontrol paneli sunucusu: tarayıcıya arayüzü ve canlı veriyi sunar.

Tek süreçte hem ROS düğümü hem HTTP sunucusu çalışır. ROS ana iş parçacığında döner; HTTP
istekleri ThreadingHTTPServer'ın iş parçacıklarında karşılanır. Canlı veri tarayıcıya
Server-Sent Events (SSE) ile akar: bağlı her sekmenin bir kuyruğu vardır, Broadcaster olayları
bütün kuyruklara dağıtır, /api/stream isteği kendi kuyruğunu boşaltır. Ek kütüphane gerekmez
(Python standart kütüphanesi).

Sunucu yalnızca 127.0.0.1'e bağlanır (kimlik doğrulama yok); WSL2'nin localhost yönlendirmesi
sayesinde Windows tarayıcısından http://localhost:<port> ile açılır.

Uç noktalar:
    GET /               web/ klasöründeki arayüz (index.html, app.js, style.css)
    GET /api/health     sunucu durumu (JSON)
    GET /api/stream     SSE akışı: hello (bağlanınca), ping (5 s'de bir)

Kullanım:
    ros2 launch otonom_surus panel.launch.py
"""

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
import queue
import signal
import threading
import time
from urllib.parse import unquote, urlsplit

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

VERSION = '0.1.0'
HOST = '127.0.0.1'  # bilerek parametre değil: panelin ağa açılmaması için (PT18)
PING_PERIOD = 5.0  # s; istemci 10 s veri alamazsa bağlantıyı kopmuş sayar
SSE_RETRY_MS = 3000  # tarayıcının kopunca yeniden bağlanma aralığı
CLIENT_QUEUE = 256  # sekme başına bekleyen olay sınırı; dolunca en eski düşer

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

    self.server.panel; web_dir, broadcaster, health() ve log_request() sağlayan nesnedir:
    çalışırken PanelServer düğümü, testlerde onun yerine geçen basit bir nesne.
    """

    server_version = 'OtonomPanel/' + VERSION

    def do_GET(self):  # noqa: N802 (http.server'ın adlandırması)
        path = urlsplit(self.path).path
        if path == '/api/health':
            self.send_json(self.server.panel.health())
        elif path == '/api/stream':
            self.stream()
        elif path.startswith('/api/'):
            self.send_json({'error': f'bilinmeyen uç nokta: {path}'}, 404)
        else:
            self.send_static()

    def do_POST(self):  # noqa: N802
        self.send_json({'error': f'bilinmeyen uç nokta: {urlsplit(self.path).path}'}, 404)

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
            self.wfile.write(sse_event('hello', panel.health()))
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
    """Panelin ROS tarafı: parametreler, durum ve SSE dağıtıcısı."""

    def __init__(self):
        super().__init__('panel_server')
        self.port = self.declare_parameter('port', 8080).value
        self.results_dir = os.path.abspath(os.path.expanduser(
            self.declare_parameter('results_dir', 'results/panel').value))
        self.web_dir = self.declare_parameter('web_dir', os.path.join(
            get_package_share_directory('otonom_surus'), 'web')).value
        self.broadcaster = Broadcaster()
        self.started = time.monotonic()

    def health(self):
        return {
            'ok': True,
            'version': VERSION,
            'node': self.get_name(),
            'ros': rclpy.ok(),
            'sim': 'idle',
            'results_dir': self.results_dir,
            'clients': self.broadcaster.count(),
            'uptime_s': round(time.monotonic() - self.started, 1),
        }

    def log_request(self, client, text):
        self.get_logger().debug(f'{client} {text}')


def main(args=None):
    """
    ROS ana iş parçacığında döner, HTTP sunucusu ayrı bir iş parçacığında.

    Tersi (ROS ayrı iş parçacığında) Ctrl+C'de düğüm, yürütücü hâlâ rcl'nin bekleme kümesindeyken
    yok edilebiliyordu: "terminate called without an active exception" ve süreç SIGKILL'e kadar
    asılı kaldı. Bu düzende kapanış sırası belirli: önce SSE akışları ve HTTP durur, sonra düğüm.
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
        # Terminalde Ctrl+C bütün süreç grubuna gider, launch da SIGINT'i ayrıca iletir. İkinci
        # sinyal temizlik sırasında süreci öldürmesin (launch "exit code -2" yazıyordu).
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.broadcaster.close()  # açık SSE akışları kuyruktan None alıp çıkar
        server.shutdown()
        server.server_close()
        http_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.try_shutdown()
        signal.signal(signal.SIGINT, signal.SIG_IGN)  # try_shutdown eski işleyiciyi geri koyar
    return 0


if __name__ == '__main__':
    main()
