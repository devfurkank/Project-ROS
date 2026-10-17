"""
Panelin müdahaleleri: tıkla-git hedefi, engel atma ve kaldırma, serbest sürüşü bitirme.

Nav2'ye hedefi yalnızca mission_manager gönderir (PT12): panel hedefi /panel/goal'e yayınlar,
mission_manager yürütülen hedefi iptal edip kalan kuyruğu yeni hedefle değiştirir ve olay kaydına
yazar. Kutular Gazebo'nun /spawn_entity ve /delete_entity servisleriyle atılıp kaldırılır; her
biri /panel/events ile mission_manager'a bildirilir, o da koşunun olay kaydına yazar (tekrar
oynatmada doğru anda görünsünler diye). Serbest sürüşün bitişi de aynı konudan gider (finish).

Denetimler (check_goal, check_box) ROS'a bağımlı değildir. Hedef oda dışına, dünyadaki bir şeklin
ya da atılmış kutunun robot sığmayacak kadar yakınına, SLAM haritasında dolu hücrenin yanına
verilmez. Kutu oda dışına, robota çok yakına, başka bir şeklin, kutunun ya da hareketli kutunun
şeridinin üstüne atılmaz (PT11). Servisler HTTP iş parçacığından çağrılır ve yanıt zaman aşımıyla
beklenir (PT5); yanıtı ROS ana iş parçacığındaki yürütücü işler.
"""

import json
import math
import threading
import time

from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from geometry_msgs.msg import Pose, PoseStamped
from otonom_surus.world_sdf import box_sdf, half_extent
from std_msgs.msg import String

ROBOT_RADIUS = 0.125  # m; nav2_params.yaml robot_radius: hedef bundan yakına verilmez
BOX_SIZE = (0.4, 0.4, 0.5)  # m; atılan kutu: en, boy, yükseklik
MAX_BOXES = 8
ROBOT_CLEARANCE = 0.25  # m; robotun merkezinden kutunun en yakın noktasına (yarıçap 0.105 + pay)
OCCUPIED = 65  # SLAM hücresi bu değerden büyük-eşitse dolu (map_saver'ın eşiği, panel_live)
SERVICE_TIMEOUT = 5.0  # s
NAME_PREFIX = 'panel_kutu_'
# İletilerde şeklin adı: yönelme (-e) ve birliktelik (-le) halleriyle.
TO_ROLE = {'wall': 'bir duvara', 'obstacle': 'bir engele'}
WITH_ROLE = {'wall': 'bir duvarla', 'obstacle': 'bir engelle'}


class ControlError(Exception):
    """Müdahale yapılamadı; ileti kullanıcıya gösterilir, status HTTP durum kodudur."""

    def __init__(self, message, status=400):
        super().__init__(message)
        self.status = status


# --- Geometri (ROS'suz) -----------------------------------------------------------------------

def box_aabb(x, y, size=BOX_SIZE):
    """Eksenlere hizalı kutu (x0, y0, x1, y1)."""
    return x - size[0] / 2, y - size[1] / 2, x + size[0] / 2, y + size[1] / 2


def shape_aabb(shape):
    """world_geometry şeklinin eksenlere hizalı çevresi (dönük kutuda, silindirde geniş tutar)."""
    dx, dy = half_extent(shape)
    return shape['x'] - dx, shape['y'] - dy, shape['x'] + dx, shape['y'] + dy


def overlaps(a, b, tol=1e-3):
    """İki dikdörtgen üst üste biniyor mu; kenarları değen (tol içinde) binmiyor sayılır."""
    return a[0] < b[2] - tol and b[0] < a[2] - tol and a[1] < b[3] - tol and b[1] < a[3] - tol


def aabb_distance(x, y, a):
    """Noktanın dikdörtgene uzaklığı; içindeyse 0."""
    return math.hypot(max(a[0] - x, 0.0, x - a[2]), max(a[1] - y, 0.0, y - a[3]))


def shape_distance(x, y, shape):
    """Noktanın şekle (dönük kutu ya da silindir) uzaklığı; içindeyse 0."""
    if shape['kind'] == 'cylinder':
        return max(0.0, math.hypot(x - shape['x'], y - shape['y']) - shape['size'])
    c, s = math.cos(shape['yaw']), math.sin(shape['yaw'])
    dx, dy = x - shape['x'], y - shape['y']
    u, v = c * dx + s * dy, -s * dx + c * dy  # şeklin kendi ekseninde
    w, h = shape['size'][:2]
    return math.hypot(max(abs(u) - w / 2, 0.0), max(abs(v) - h / 2, 0.0))


def lane_aabbs(geometry, movers):
    """
    Hareketli kutuların gidip geldiği alanlar: doğduğu yerdeki ve şeridin ucundaki halinin çevresi.

    movers senaryonun moving_obstacles'ı; kutu kendi +x ekseni boyunca travel metre gider.
    """
    lanes = []
    for mover in movers:
        shape = next((s for s in geometry['shapes'] if s['model'] == mover['name']), None)
        if shape is None:
            continue
        dx, dy = half_extent(shape)
        ex = shape['x'] + math.cos(shape['yaw']) * mover['travel']
        ey = shape['y'] + math.sin(shape['yaw']) * mover['travel']
        lanes.append((min(shape['x'], ex) - dx, min(shape['y'], ey) - dy,
                      max(shape['x'], ex) + dx, max(shape['y'], ey) + dy))
    return lanes


def inside_bounds(x, y, bounds):
    return bounds is None or (bounds[0] <= x <= bounds[2] and bounds[1] <= y <= bounds[3])


def occupied_near(grid, x, y, radius):
    """
    SLAM haritasında (x, y)'nin radius yakınında dolu hücre var mı.

    grid: {'w', 'h', 'res', 'ox', 'oy', 'data'}; data OccupancyGrid'in hücreleri (satır satır,
    0. satır haritanın altı; -1 bilinmeyen). Harita dışı ve bilinmeyen hücre dolu sayılmaz
    (Nav2 bilinmeyende planlayabiliyor: allow_unknown).
    """
    res = grid['res']
    i0 = max(0, int(math.floor((x - radius - grid['ox']) / res)))
    i1 = min(grid['w'] - 1, int(math.floor((x + radius - grid['ox']) / res)))
    j0 = max(0, int(math.floor((y - radius - grid['oy']) / res)))
    j1 = min(grid['h'] - 1, int(math.floor((y + radius - grid['oy']) / res)))
    for j in range(j0, j1 + 1):
        cy = grid['oy'] + (j + 0.5) * res
        for i in range(i0, i1 + 1):
            cx = grid['ox'] + (i + 0.5) * res
            if (math.hypot(cx - x, cy - y) <= radius + res / 2
                    and grid['data'][j * grid['w'] + i] >= OCCUPIED):
                return True
    return False


def check_goal(x, y, geometry, boxes=(), grid=None):
    """Elle hedefin ön kontrolü: sorun varsa kullanıcıya gösterilecek ileti, yoksa None."""
    if geometry is not None:
        if not inside_bounds(x, y, geometry['bounds']):
            return 'Hedef odanın dışında.'
        for shape in geometry['shapes']:
            if shape['role'] != 'moving' and shape_distance(x, y, shape) < ROBOT_RADIUS:
                what = TO_ROLE.get(shape['role'], TO_ROLE['obstacle'])
                return f'Hedef {what} çok yakın; robot oraya sığmaz.'
    for box in boxes:
        if aabb_distance(x, y, box_aabb(box['x'], box['y'], box['size'])) < ROBOT_RADIUS:
            return 'Hedef atılmış bir kutunun üstünde ya da çok yakınında.'
    if grid is not None and occupied_near(grid, x, y, ROBOT_RADIUS):
        return 'Hedef SLAM haritasında dolu bir hücrenin yanında; robot oraya sığmaz.'
    return None


def check_box(x, y, geometry, boxes=(), robot=None, lanes=()):
    """Atılacak kutunun ön kontrolü (PT11): sorun varsa ileti, yoksa None."""
    if len(boxes) >= MAX_BOXES:
        return f'En çok {MAX_BOXES} kutu atılabilir; önce birini kaldır.'
    box = box_aabb(x, y)
    if geometry is not None:
        bounds = geometry['bounds']
        if bounds is not None and not (bounds[0] <= box[0] and box[2] <= bounds[2]
                                       and bounds[1] <= box[1] and box[3] <= bounds[3]):
            return 'Kutu odanın dışına taşıyor.'
        for shape in geometry['shapes']:
            if shape['role'] != 'moving' and overlaps(box, shape_aabb(shape)):
                what = WITH_ROLE.get(shape['role'], WITH_ROLE['obstacle'])
                return f'Kutu {what} çakışıyor.'
    for lane in lanes:
        if overlaps(box, lane):
            return 'Kutu hareketli bir kutunun yolunu kesiyor; Gazebo’da çarpışırlar.'
    for other in boxes:
        if overlaps(box, box_aabb(other['x'], other['y'], other['size'])):
            return 'Kutu atılmış başka bir kutuyla çakışıyor.'
    if robot is not None and aabb_distance(robot[0], robot[1], box) < ROBOT_CLEARANCE:
        return f'Kutu robota çok yakın (en az {ROBOT_CLEARANCE:.2f} m).'
    return None


def grid_of(msg):
    """Harita iletisi -> occupied_near'ın beklediği sözlük (veri kopyalanmaz); ileti yoksa None."""
    if msg is None:
        return None
    info = msg.info
    return {'w': info.width, 'h': info.height, 'res': info.resolution,
            'ox': info.origin.position.x, 'oy': info.origin.position.y, 'data': msg.data}


def goal_msg(x, y, yaw_deg, stamp):
    """Elle hedef: map çerçevesinde konum ve yön."""
    msg = PoseStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp = stamp
    msg.pose.position.x, msg.pose.position.y = float(x), float(y)
    yaw = math.radians(yaw_deg)
    msg.pose.orientation.z, msg.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return msg


# --- ROS tarafı -------------------------------------------------------------------------------

class Controller:
    """
    Hedef ve olay yayıncıları, Gazebo servis istemcileri ve atılan kutuların listesi.

    publish(event, data) tarayıcılara dağıtır: kutu listesi değişince 'obstacles'. Kutular yeni
    koşu başlarken unutulur (reset); koşu bitince Gazebo kapandığından onlar da kalkar.
    """

    def __init__(self, node, publish):
        self.node = node
        self.publish = publish
        self.goal_pub = node.create_publisher(PoseStamped, 'panel/goal', 10)
        self.event_pub = node.create_publisher(String, 'panel/events', 10)
        self.spawn_client = node.create_client(SpawnEntity, 'spawn_entity')
        self.delete_client = node.create_client(DeleteEntity, 'delete_entity')
        self._lock = threading.Lock()  # kutu listesi
        self._op = threading.Lock()  # aynı anda tek atma ya da kaldırma (denetim + servis)
        self.boxes = {}
        self._next = 1
        self.reset()

    def reset(self):
        with self._lock:
            self.boxes = {}
            self._next = 1
        self._announce()

    def boxes_list(self):
        with self._lock:
            return [dict(b) for b in self.boxes.values()]

    def send_goal(self, x, y, yaw_deg):
        self.goal_pub.publish(goal_msg(x, y, yaw_deg, self.node.get_clock().now().to_msg()))

    def finish(self):
        self._event({'type': 'finish'})

    def spawn_box(self, x, y, validate):
        """
        Kutuyu atar: validate(kutular) None dönerse; atılan kutunun sözlüğü.

        Denetim ve servis çağrısı tek kilit altında: aynı anda gelen iki istek ikisi de "boş
        yer" görüp üst üste kutu atmasın.
        """
        with self._op:
            error = validate(self.boxes_list())
            if error:
                raise ControlError(error, 400)
            with self._lock:
                name = f'{NAME_PREFIX}{self._next}'
                self._next += 1  # zaman aşımında da: geç gelen kutu adı yeniden kullanılmasın
            request = SpawnEntity.Request(name=name, xml=box_sdf(name, BOX_SIZE),
                                          reference_frame='world')
            request.initial_pose = Pose()
            request.initial_pose.position.x, request.initial_pose.position.y = x, y
            request.initial_pose.position.z = BOX_SIZE[2] / 2
            response = self._call(self.spawn_client, request, 'spawn_entity')
            if not response.success:
                raise ControlError(f'Gazebo kutuyu atamadı: {response.status_message}', 502)
            box = {'name': name, 'x': round(x, 3), 'y': round(y, 3), 'size': list(BOX_SIZE)}
            with self._lock:
                self.boxes[name] = box
            self._event({'type': 'spawn', **box})
        self._announce()
        return dict(box)

    def delete_boxes(self, name=None):
        """Bir kutuyu (name) ya da hepsini kaldırır; kaldırılanların adları."""
        with self._op:
            names = list(self.boxes) if name is None else [name]
            if name is not None and name not in self.boxes:
                raise ControlError(f'Böyle bir kutu yok: {name}', 404)
            removed, errors = [], []
            for n in names:
                response = self._call(self.delete_client, DeleteEntity.Request(name=n),
                                      'delete_entity')
                if not response.success:
                    errors.append(f'{n}: {response.status_message}')
                    continue
                with self._lock:
                    self.boxes.pop(n, None)
                self._event({'type': 'delete', 'name': n})
                removed.append(n)
        self._announce()
        if errors and not removed:
            raise ControlError('Gazebo kutuyu kaldıramadı: ' + '; '.join(errors), 502)
        return removed

    def _announce(self):
        self.publish('obstacles', {'boxes': self.boxes_list()})

    def _event(self, data):
        self.event_pub.publish(String(data=json.dumps(data, ensure_ascii=False)))

    def _call(self, client, request, service):
        """Servis çağrısı, SERVICE_TIMEOUT'lu (PT5); yanıt ROS ana iş parçacığında işlenir."""
        if not client.wait_for_service(timeout_sec=1.0):
            raise ControlError(f'/{service} servisi yok (Gazebo açık mı?)', 503)
        future = client.call_async(request)
        deadline = time.monotonic() + SERVICE_TIMEOUT
        while not future.done():
            if time.monotonic() > deadline:
                raise ControlError(
                    f'/{service} {SERVICE_TIMEOUT:.0f} s içinde yanıt vermedi', 504)
            time.sleep(0.02)
        if future.result() is None:
            raise ControlError(f'/{service} çağrısı başarısız: {future.exception()}', 502)
        return future.result()
