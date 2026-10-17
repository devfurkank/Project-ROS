"""
Panelin canlı verisi: ROS konularını önbelleğe alıp tarayıcıya hız sınırlı SSE olayları gönderir.

Abonelikler panel_server düğümündedir; geri çağrılar ve zamanlayıcı ROS ana iş parçacığında,
reset() ve snapshot() HTTP iş parçacıklarında çalışır, önbellek kilitle korunur. Olaylar:
    status  /mission_status (mission_manager'ın JSON'u) olduğu gibi, her gelişinde
    pose    TF map -> base_footprint, /odom hızı, en yakın engel, algılanan küme sayısı ve
            hareketli kutuların konumu; en çok 10 Hz, yalnızca yeni TF gelince
    trail   /traveled_path (mission_manager'ın izi), değişince en çok 2 Hz
    plan    /plan (Nav2 global planı) seyreltilmiş, değişince en çok 2 Hz
    map     /map -> gri tonlu PNG (map_saver renkleri), içerik değişince en çok 1 Hz

Robot, iz, plan ve harita map çerçevesindedir (PT3). Hareketli kutular /<ad>/odom'dan, yani
dünya koordinatında gelir; aradaki fark map -> odom kayması kadardır (birkaç cm). Harita
orijininin dönüklüğü yok sayılır (slam_toolbox'ta sıfır).

Önbellek yeni koşuda boşaltılır: panel koşuyu başlatırken (reset), başka run_id'li bir görev
görülünce (ör. terminalden başlatılmış) ve sim zamanı geri gidince (simülasyon yeniden başladı;
yalnızca TF). Yoksa önceki simülasyonun ileri zaman damgalı TF'leri tamponda kalır, yenileri
"geçmişten" sayılıp atılır (TF_OLD_DATA) ve robot eski yerinde donar (PT25).
"""

import array
import base64
import io
import json
import math
import threading
import zlib

from nav_msgs.msg import OccupancyGrid, Odometry, Path
from PIL import Image
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Float32, String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

TICK = 0.1  # s, duvar saati: pose en çok 10 Hz
PATH_EVERY = 5  # tick: iz ve plan en çok 2 Hz
MAP_EVERY = 10  # tick: harita en çok 1 Hz
PLAN_STEP = 0.1  # m; Nav2 planı 0.05 m aralıklı, tarayıcıya bunun yarısı yeter
CLOCK_JUMP = 1.0  # s; /odom zaman damgası bundan çok geri giderse simülasyon yeniden başladı
# slam_toolbox haritayı, mission_manager durumu transient_local yayınlar (PT1).
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)

# Hücre değeri (işaretsiz bayt olarak) -> map_saver'ın üçlü kipindeki gri ton: boş (0–25) 254,
# dolu (65–100) 0, bilinmeyen (-1 = 255) ve aradakiler 205.
_GRAY = bytes(254 if v <= 25 else 0 if 65 <= v <= 100 else 205 for v in range(256))


def map_png(width, height, data):
    """
    Harita (OccupancyGrid verisi) -> map_saver renklerinde gri tonlu PNG baytları.

    Izgaranın 0. satırı haritanın altıdır (en küçük y), resminki üstü: satırlar ters çevrilir,
    böylece resim kuzey yukarıda olacak şekilde doğrudan çizilir. Veri boyutu tutmazsa ValueError.
    """
    raw = array.array('b', data).tobytes().translate(_GRAY)
    if len(raw) != width * height:
        raise ValueError(f'harita {width} x {height} ama {len(raw)} hücre')
    rows = [raw[i * width:(i + 1) * width] for i in range(height)]
    out = io.BytesIO()
    Image.frombytes('L', (width, height), b''.join(reversed(rows))).save(out, format='PNG')
    return out.getvalue()


def thin_path(points, min_step):
    """Ardışık iki nokta arası en az min_step olacak şekilde seyreltir; ilk ve son nokta kalır."""
    if len(points) < 3:
        return list(points)
    out = [points[0]]
    for p in points[1:-1]:
        if math.hypot(p[0] - out[-1][0], p[1] - out[-1][1]) >= min_step:
            out.append(p)
    out.append(points[-1])
    return out


def path_points(msg, min_step=0.0):
    """nav_msgs/Path -> [[x, y], ...]: cm'ye yuvarlanmış, isteğe göre seyreltilmiş."""
    points = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
    return [[round(x, 2), round(y, 2)] for x, y in thin_path(points, min_step)]


def yaw_of(q):
    """Dördeyden z ekseni etrafındaki dönüş (rad)."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def finite(value, digits):
    """Yuvarlanmış sayı; değer yoksa ya da sonsuzsa None (JSON'da Infinity geçersiz)."""
    if value is None or not math.isfinite(value):
        return None
    return round(value, digits)


class LiveBridge:
    """
    Canlı veri köprüsü: abonelikler, önbellek ve hız sınırlı yayın.

    publish(event, data) olayları tarayıcılara dağıtır; on_status(status) her /mission_status
    iletisiyle çağrılır (panelin koşu yöneticisi aşamayı buradan izler). movers hareketli
    kutuların adları; her biri için /<ad>/odom'a abone olunur.
    """

    def __init__(self, node, publish, on_status=None, movers=()):
        self.node = node
        self.publish = publish
        self.on_status_cb = on_status
        self._lock = threading.Lock()
        self._ticks = 0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)
        self.reset()
        sub = node.create_subscription
        sub(OccupancyGrid, 'map', self.on_map, LATCHED)
        sub(String, 'mission_status', self.on_status, LATCHED)
        sub(Path, 'traveled_path', self.on_trail, 10)
        sub(Path, 'plan', self.on_plan, 10)
        sub(Odometry, 'odom', self.on_odom, 10)
        sub(Float32, 'min_obstacle_distance', self.on_nearest, 10)
        sub(MarkerArray, 'detected_obstacles', self.on_markers, 10)
        for name in movers:
            sub(Odometry, f'/{name}/odom', lambda msg, n=name: self.on_mover(n, msg), 10)
        node.create_timer(TICK, self.tick)

    def reset(self, status=None):
        """
        Önceki koşunun durumu, haritası, izi, planı, konumu ve TF'leri unutulur.

        status verilirse yeni görevin durumu olarak saklanır.
        """
        with self._lock:
            self.status = status
            self.map_msg = None
            self.map_event = None
            self._map_key = None
            self.trail = None
            self.plan = None
            self._trail_new = False
            self._plan_new = False
            self.pose = None
            self.speed = None
            self.nearest = None
            self.clusters = None
            self.movers = {}
            self._odom_stamp = None
        self.tf_buffer.clear()

    def snapshot(self):
        """Yeni bağlanan sekmeye: son durum, harita, iz, plan ve konum (olmayan None)."""
        with self._lock:
            return {'status': self.status, 'map': self.map_event, 'trail': self.trail,
                    'plan': self.plan, 'pose': self.pose}

    # --- Geri çağrılar (ROS iş parçacığı) -----------------------------------------------------

    def on_map(self, msg):
        with self._lock:
            self.map_msg = msg  # kodlama tick'te, en çok 1 Hz

    def on_status(self, msg):
        try:
            status = json.loads(msg.data)
        except ValueError:
            status = None
        if not isinstance(status, dict):
            self.node.get_logger().warn(f'mission_status çözülemedi: {msg.data[:80]!r}')
            return
        with self._lock:
            other = self.status is not None and self.status.get('run_id') != status.get('run_id')
            if not other:
                self.status = status
        if other:  # başka bir görev başladı (ör. terminalden): öncekinin verisi karışmasın
            self.reset(status)
        self.publish('status', status)
        if self.on_status_cb is not None:
            self.on_status_cb(status)

    def on_trail(self, msg):
        points = path_points(msg)
        with self._lock:
            if points != self.trail:
                self.trail, self._trail_new = points, True

    def on_plan(self, msg):
        points = path_points(msg, PLAN_STEP)
        with self._lock:
            if points != self.plan:
                self.plan, self._plan_new = points, True

    def on_odom(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self._lock:
            restarted = self._odom_stamp is not None and stamp < self._odom_stamp - CLOCK_JUMP
            self._odom_stamp = stamp
            self.speed = msg.twist.twist.linear.x
            if restarted:
                self.pose = None
        if restarted:  # sim zamanı geri gitti: eski TF'ler yenilerini engellemesin
            self.tf_buffer.clear()

    def on_nearest(self, msg):
        with self._lock:
            self.nearest = msg.data

    def on_markers(self, msg):
        # obstacle_detector her iletide önce DELETEALL, sonra kümeleri silindir olarak gönderir.
        count = sum(m.action == Marker.ADD and m.type == Marker.CYLINDER for m in msg.markers)
        with self._lock:
            self.clusters = count

    def on_mover(self, name, msg):
        p = msg.pose.pose
        with self._lock:
            self.movers[name] = [round(p.position.x, 3), round(p.position.y, 3),
                                 round(yaw_of(p.orientation), 3)]

    # --- Yayın --------------------------------------------------------------------------------

    def tick(self):
        """Zamanlayıcı (TICK): yeni konum her seferinde, iz ve plan 2 Hz, harita 1 Hz."""
        self._ticks += 1
        pose = self._robot_pose()
        events = []
        with self._lock:
            if pose is not None and (self.pose is None or pose['t'] != self.pose['t']):
                pose.update(v=finite(self.speed, 3), nearest=finite(self.nearest, 3),
                            clusters=self.clusters, movers=dict(self.movers))
                self.pose = pose
                events.append(('pose', pose))
            if self._ticks % PATH_EVERY == 0:
                if self._trail_new:
                    self._trail_new = False
                    events.append(('trail', {'pts': self.trail}))
                if self._plan_new:
                    self._plan_new = False
                    events.append(('plan', {'pts': self.plan}))
            map_msg = self.map_msg if self._ticks % MAP_EVERY == 0 else None
        if map_msg is not None:
            event = self._encode_map(map_msg)
            if event is not None:
                events.append(('map', event))
        for event, data in events:
            self.publish(event, data)

    def _robot_pose(self):
        """Robotun map çerçevesindeki son konumu; t dönüşümün zaman damgası (sim zamanı)."""
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_footprint', Time())
        except TransformException:
            return None
        tr = tf.transform
        stamp = tf.header.stamp
        return {'x': round(tr.translation.x, 3), 'y': round(tr.translation.y, 3),
                'th': round(yaw_of(tr.rotation), 3),
                't': round(stamp.sec + stamp.nanosec * 1e-9, 2)}

    def _encode_map(self, msg):
        """Harita değiştiyse 'map' olayının verisi, değişmediyse None."""
        info = msg.info
        origin = info.origin.position
        raw = array.array('b', msg.data).tobytes()
        key = (info.width, info.height, info.resolution, origin.x, origin.y, zlib.crc32(raw))
        with self._lock:
            if key == self._map_key:
                return None
        try:
            png = map_png(info.width, info.height, raw)
        except ValueError as e:
            self.node.get_logger().warn(f'/map kodlanamadı: {e}')
            return None
        event = {'png': base64.b64encode(png).decode('ascii'), 'w': info.width,
                 'h': info.height, 'res': round(info.resolution, 4),
                 'ox': round(origin.x, 3), 'oy': round(origin.y, 3)}
        with self._lock:
            if self.map_msg is not msg:  # arada reset() ya da yeni harita geldi
                return None
            self._map_key, self.map_event = key, event
        return event
