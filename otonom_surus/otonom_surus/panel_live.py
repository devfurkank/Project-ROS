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
Katmanlar (P7): yalnızca o katmanı açmış sekmelere gider; hiçbir sekme istemiyorsa hesaplanmaz
(PT16). wants(katman) kimin istediğini söyler, layer_snapshot() yeni açan sekmeye son hali verir.
    scan      lidar     /scan (sensor_data QoS, PT2): 360 -> 180 ışın, cm tamsayı (geçersiz 0),
                        tarama anında sensörün map'teki pozu; yeni tarama varsa en çok 5 Hz
    clusters  clusters  /detected_obstacles'taki silindirler [x, y, r]; değişince en çok 5 Hz
    costmap   costmap   /global_costmap/costmap (0–100, −1 bilinmeyen, PT15) değerleriyle 8 bit
                        PNG (−1 = 255); içerik ya da konum değişince en çok 1 Hz. Pencere robotla
                        kayar (rolling_window), orijin her iletide gelir

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
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

TICK = 0.1  # s, duvar saati: pose en çok 10 Hz
PATH_EVERY = 5  # tick: iz ve plan en çok 2 Hz
MAP_EVERY = 10  # tick: harita ve costmap en çok 1 Hz
LAYER_EVERY = 2  # tick: tarama ve kümeler en çok 5 Hz
PLAN_STEP = 0.1  # m; Nav2 planı 0.05 m aralıklı, tarayıcıya bunun yarısı yeter
SCAN_STRIDE = 2  # 360 ışından her ikincisi (2°): tarayıcıda çizim ve trafik yarıya iner
LAYERS = ('lidar', 'costmap', 'clusters')
CLOCK_JUMP = 1.0  # s; /odom zaman damgası bundan çok geri giderse simülasyon yeniden başladı
# slam_toolbox haritayı, mission_manager durumu transient_local yayınlar (PT1).
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)

# Hücre değeri (işaretsiz bayt olarak) -> map_saver'ın üçlü kipindeki gri ton: boş (0–25) 254,
# dolu (65–100) 0, bilinmeyen (-1 = 255) ve aradakiler 205.
_GRAY = bytes(254 if v <= 25 else 0 if 65 <= v <= 100 else 205 for v in range(256))


def grid_png(width, height, raw):
    """
    Izgara baytları (satır satır, 0. satır haritanın altı) -> kuzey yukarıda 8 bit gri PNG.

    Izgaranın 0. satırı haritanın altıdır (en küçük y), resminki üstü: satırlar ters çevrilir,
    böylece resim doğrudan çizilir. Veri boyutu tutmazsa ValueError.
    """
    if len(raw) != width * height:
        raise ValueError(f'ızgara {width} x {height} ama {len(raw)} hücre')
    rows = [raw[i * width:(i + 1) * width] for i in range(height)]
    out = io.BytesIO()
    Image.frombytes('L', (width, height), b''.join(reversed(rows))).save(out, format='PNG')
    return out.getvalue()


def map_png(width, height, data):
    """Harita (OccupancyGrid verisi) -> map_saver renklerinde gri tonlu PNG baytları."""
    return grid_png(width, height, array.array('b', data).tobytes().translate(_GRAY))


def costmap_png(width, height, data):
    """
    Costmap (OccupancyGrid: 0–100, −1 bilinmeyen) -> değerlerin kendisiyle 8 bit PNG.

    Değer bayt olarak yazılır, −1 255 olur; renklendirmeyi tarayıcı yapar (0 saydam, 1–98
    maliyet, 99 robot sığmaz, 100 ölümcül).
    """
    return grid_png(width, height, array.array('b', data).tobytes())


def scan_ranges(ranges, range_min, range_max, stride=SCAN_STRIDE):
    """Her stride'ıncı ışının uzaklığı cm tamsayı; ölçüm yoksa ya da aralık dışındaysa 0."""
    return [round(r * 100) if math.isfinite(r) and range_min <= r <= range_max else 0
            for r in ranges[::stride]]


def cylinders(markers):
    """obstacle_detector işaretçilerinden engel kümeleri [x, y, r]: silindirler, duvarlar değil."""
    return [[round(m.pose.position.x, 3), round(m.pose.position.y, 3), round(m.scale.x / 2, 3)]
            for m in markers if m.action == Marker.ADD and m.type == Marker.CYLINDER]


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

    publish(event, data[, layer]) olayları tarayıcılara dağıtır (layer verilirse yalnızca o
    katmanı açmış sekmelere); on_status(status) her /mission_status iletisiyle çağrılır (panelin
    koşu yöneticisi aşamayı buradan izler). movers hareketli kutuların adları; her biri için
    /<ad>/odom'a abone olunur. wants(layer) o katmanı isteyen sekme var mı.
    """

    def __init__(self, node, publish, on_status=None, movers=(), wants=lambda layer: True):
        self.node = node
        self.publish = publish
        self.on_status_cb = on_status
        self.wants = wants
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
        sub(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        sub(OccupancyGrid, 'global_costmap/costmap', self.on_costmap, LATCHED)
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
            # Katmanlar: son ileti, son gönderilen olay (yeni açan sekmeye) ve yenilik bayrağı.
            self.scan_msg = None
            self.scan_event = None
            self.cluster_list = None
            self.clusters_event = None
            self._clusters_new = False
            self.costmap_msg = None
            self.costmap_event = None
            self._costmap_key = None
        self.tf_buffer.clear()

    def snapshot(self):
        """Yeni bağlanan sekmeye: son durum, harita, iz, plan ve konum (olmayan None)."""
        with self._lock:
            return {'status': self.status, 'map': self.map_event, 'trail': self.trail,
                    'plan': self.plan, 'pose': self.pose}

    def view(self):
        """Müdahale denetimleri için: son durum, robotun konumu ve son harita iletisi (/map)."""
        with self._lock:
            return {'status': self.status, 'pose': self.pose, 'map': self.map_msg}

    def layer_snapshot(self, layer):
        """Katmanı yeni açan sekmeye: (olay, veri) son gönderilen hali; yoksa None."""
        with self._lock:
            event = {'lidar': ('scan', self.scan_event),
                     'clusters': ('clusters', self.clusters_event),
                     'costmap': ('costmap', self.costmap_event)}.get(layer)
        return event if event and event[1] is not None else None

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
        # obstacle_detector her iletide önce DELETEALL, sonra kümeleri silindir olarak gönderir
        # (navigation.launch.py'de çerçevesi map).
        found = cylinders(msg.markers)
        with self._lock:
            self.clusters = len(found)
            if found != self.cluster_list:
                self.cluster_list, self._clusters_new = found, True

    def on_scan(self, msg):
        with self._lock:
            self.scan_msg = msg  # kodlama tick'te, en çok 5 Hz ve yalnızca istenirse

    def on_costmap(self, msg):
        with self._lock:
            self.costmap_msg = msg

    def on_mover(self, name, msg):
        p = msg.pose.pose
        with self._lock:
            self.movers[name] = [round(p.position.x, 3), round(p.position.y, 3),
                                 round(yaw_of(p.orientation), 3)]

    # --- Yayın --------------------------------------------------------------------------------

    def tick(self):
        """
        Zamanlayıcı (TICK): yeni konum her seferinde, iz ve plan 2 Hz, harita 1 Hz.

        Katmanlar yalnızca isteyen sekme varsa: tarama ve kümeler 5 Hz, costmap 1 Hz.
        """
        self._ticks += 1
        pose = self._robot_pose()
        events = []
        layer_tick = self._ticks % LAYER_EVERY == 0
        slow_tick = self._ticks % MAP_EVERY == 0
        with self._lock:
            if pose is not None and (self.pose is None or pose['t'] != self.pose['t']):
                pose.update(v=finite(self.speed, 3), nearest=finite(self.nearest, 3),
                            clusters=self.clusters, movers=dict(self.movers))
                self.pose = pose
                events.append(('pose', pose, None))
            if self._ticks % PATH_EVERY == 0:
                if self._trail_new:
                    self._trail_new = False
                    events.append(('trail', {'pts': self.trail}, None))
                if self._plan_new:
                    self._plan_new = False
                    events.append(('plan', {'pts': self.plan}, None))
            map_msg = self.map_msg if slow_tick else None
            scan_msg = None
            if layer_tick:
                scan_msg, self.scan_msg = self.scan_msg, None  # her tarama bir kez
                if self._clusters_new:
                    self._clusters_new = False
                    self.clusters_event = {'c': self.cluster_list}
                    events.append(('clusters', self.clusters_event, 'clusters'))
            costmap_msg = self.costmap_msg if slow_tick else None
        if map_msg is not None:
            event = self._encode_map(map_msg)
            if event is not None:
                events.append(('map', event, None))
        if scan_msg is not None and self.wants('lidar'):
            event = self._encode_scan(scan_msg)
            if event is not None:
                events.append(('scan', event, 'lidar'))
        if costmap_msg is not None and self.wants('costmap'):
            event = self._encode_costmap(costmap_msg)
            if event is not None:
                events.append(('costmap', event, 'costmap'))
        for event, data, layer in events:
            if layer is None:
                self.publish(event, data)
            elif self.wants(layer):
                self.publish(event, data, layer)

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

    def _encode_costmap(self, msg):
        """Costmap ya da penceresinin yeri değiştiyse 'costmap' olayının verisi, değilse None."""
        info = msg.info
        origin = info.origin.position
        raw = array.array('b', msg.data).tobytes()
        key = (info.width, info.height, info.resolution, origin.x, origin.y, zlib.crc32(raw))
        with self._lock:
            if key == self._costmap_key:
                return None
        try:
            png = grid_png(info.width, info.height, raw)
        except ValueError as e:
            self.node.get_logger().warn(f'costmap kodlanamadı: {e}')
            return None
        event = {'png': base64.b64encode(png).decode('ascii'), 'w': info.width,
                 'h': info.height, 'res': round(info.resolution, 4),
                 'ox': round(origin.x, 3), 'oy': round(origin.y, 3)}
        with self._lock:
            if self.costmap_msg is None:  # arada reset()
                return None
            self._costmap_key, self.costmap_event = key, event
        return event

    def _encode_scan(self, msg):
        """
        'scan' olayının verisi: sensörün tarama anındaki map pozu ve seyreltilmiş uzaklıklar.

        Poz tarama damgasındaki TF'ten; o an henüz tamponda yoksa (map -> odom geç gelir) en
        son TF'ten. TF hiç yoksa None.
        """
        tf = None
        for stamp in (Time.from_msg(msg.header.stamp), Time()):
            try:
                tf = self.tf_buffer.lookup_transform('map', msg.header.frame_id, stamp)
                break
            except TransformException:
                continue
        if tf is None:
            return None
        tr = tf.transform
        stamp = msg.header.stamp
        event = {'x': round(tr.translation.x, 3), 'y': round(tr.translation.y, 3),
                 'th': round(yaw_of(tr.rotation), 4),
                 't': round(stamp.sec + stamp.nanosec * 1e-9, 2),
                 'a0': round(msg.angle_min, 5), 'da': round(msg.angle_increment * SCAN_STRIDE, 6),
                 'rmax': round(msg.range_max, 2),
                 'r': scan_ranges(msg.ranges, msg.range_min, msg.range_max)}
        with self._lock:
            self.scan_event = event
        return event
