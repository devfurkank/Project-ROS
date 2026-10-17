"""
Senaryonun hedeflerini Nav2'ye sırayla gönderip koşunun metriklerini kaydeden düğüm.

config/scenarios.yaml'dan senaryonun hedef listesini okur; her hedefi NavigateToPose eylemiyle
(nav2_simple_commander.BasicNavigator.goToPose) gönderir ve sonucunu bekler. Koşu boyunca süreyi
(sim zamanı), kat edilen yolu, LiDAR'ın en yakın yüzeye mesafesini ve kurtarma davranışı
sayısını ölçer. Bitince results_dir altına yazar ve çıkar (scenario.launch.py de kapanır):
    runs.csv                     koşu başına bir satır
    goals.csv                    hedef başına bir satır
    trajectories/s<N>_<run>.csv  robotun yörüngesi
    maps/s<N>_<run>.pgm, .yaml   SLAM'ın koşu sonundaki haritası (map_saver_cli)
    meta/s<N>_<run>.json         koşunun kimliği: dünya ve geometrisi, senaryo, parametre dosyası,
                                 commit, kaynak (panel | run_experiments | terminal)
    events/s<N>_<run>.jsonl      web panelinin müdahaleleri (elle hedef, iptal, atılan ve
                                 kaldırılan kutu); yalnızca müdahale olduysa

Web paneli: /panel/goal'den gelen hedef yürütülen hedefi iptal eder ve kalan hedeflerin yerine
geçer. Nav2'ye hedefi yine yalnızca bu düğüm gönderir (PT12). İptal edilen hedef goals.csv'de
CANCELED, elle verilen sıradaki goal_index ile yer alır; CANCELED koşuyu başarısız saymaz ve
hedef sayısına girmez. mode=interactive'de (free.launch.py, serbest sürüş) senaryonun hedefleri
yüklenmez: düğüm panelden hedef bekler (durum 'idle'), /panel/events'ten {"type": "finish"} gelince
sonuçları yazar ve çıkar. Ölçüm ve kayıt iki kipte de aynı.

Konular:
    odom                   (nav_msgs/Odometry)  girdi: yol ve yörünge (Gazebo'da dünya gerçeği)
    min_obstacle_distance  (std_msgs/Float32)   girdi: obstacle_detector'dan
    panel/goal             (geometry_msgs/PoseStamped)  girdi: panelden elle hedef, map çerçevesi
    panel/events           (std_msgs/String)    girdi: panel olayları, JSON: spawn, delete, finish
    traveled_path          (nav_msgs/Path)      gidilen iz, map çerçevesinde (RViz)
    mission_status         (std_msgs/String)    görevin durumu, JSON (web paneli); her
                                                değişimde ve 1 Hz'de, transient_local
"""

import csv
import json
import math
import os
import subprocess
import time
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.srv import GetState
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry, Path
from otonom_surus.scenarios import load_scenario
from otonom_surus.world_sdf import world_geometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Float32, String
from tf2_ros import Buffer, TransformException, TransformListener

STATUS_VERSION = 1  # /mission_status ve meta şemasının sürümü
RECOVERY_HOLD = 5.0  # s (sim); kurtarma sayısı artınca durum bu kadar süre 'recovering' kalır
MODES = ('scenario', 'interactive')
CANCELED = 'CANCELED'  # panelin iptal ettiği hedefin sonucu (yeni hedef ya da bitir)
PANEL_EVENTS = ('spawn', 'delete')  # /panel/events'ten olay kaydına yazılanlar
# Son durum geç bağlanan aboneye de gitsin (panel koşu sürerken açılabilir).
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)

RUN_FIELDS = [
    'timestamp', 'scenario', 'run_id', 'result', 'goals_reached', 'goal_count',
    'duration_s', 'wall_duration_s', 'path_length_m', 'straight_line_m', 'path_efficiency',
    'min_obstacle_dist_m', 'recoveries', 'failure_reason']
GOAL_FIELDS = [
    'timestamp', 'scenario', 'run_id', 'goal_index', 'goal_x', 'goal_y', 'goal_yaw_deg',
    'result', 'duration_s', 'path_length_m', 'recoveries', 'final_error_m']


class PathRecorder:
    """Konumları en az min_step aralıkla kaydeder ve kat edilen yolu toplar."""

    def __init__(self, min_step):
        self.min_step = min_step
        self.points = []  # (t, x, y)
        self.length = 0.0

    def add(self, t, x, y):
        """
        Noktayı son kaydedilene en az min_step uzaksa ekler; eklendiyse True döner.

        Robot dururken fizik motorunun milimetre altı titreşimleri böylece yola eklenmez.
        """
        if self.points:
            _, px, py = self.points[-1]
            step = math.hypot(x - px, y - py)
            if step < self.min_step:
                return False
            self.length += step
        self.points.append((t, x, y))
        return True


def straight_line_length(start, goals):
    """Başlangıçtan hedeflere sırayla kuş uçuşu gidilen toplam mesafe."""
    total, (px, py) = 0.0, start
    for x, y in goals:
        total += math.hypot(x - px, y - py)
        px, py = x, y
    return total


def append_csv(path, fields, row):
    """Satırı CSV'nin sonuna ekler; dosya yoksa önce başlık satırını yazar."""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    new = not os.path.exists(path)
    with open(path, 'a', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        if new:
            writer.writeheader()
        writer.writerow(row)


def goal_parts(goal):
    """Hedef demeti -> (x, y, yaw[rad], kaynak); kaynak 'scenario' ya da 'panel'."""
    x, y, yaw = goal[:3]
    return x, y, yaw, goal[3] if len(goal) > 3 else 'scenario'


def mission_status(scenario, run_id, results_dir, state, goals, results, now=None, t0=None,
                   current=None, current_recoveries=0, last_recovery=None, distance=0.0,
                   min_distance=math.inf, mode='scenario'):
    """
    /mission_status iletisinin içeriği: JSON'a hazır sözlük (sürüm alanlı).

    state: waiting_nav2 | running | idle | finished; idle serbest sürüşte hedef beklenirken,
    çalışırken son RECOVERY_HOLD saniyede kurtarma sayısı arttıysa recovering. goals hedef kuyruğu
    ((x, y, yaw[rad]) ya da sonuna kaynağı eklenmiş demetler; panelin hedefi kuyruğu değiştirir),
    results biten hedeflerin navigate() satırları (iptal edilenler dahil), current yürütülen
    hedefin sırası (1'den; hedefler arasında None), current_recoveries onun şimdiye kadarki
    kurtarma sayısı. Zamanlar sim zamanı (s); t0 kaydın başladığı an, t ondan bu yana geçen süre.
    Sonsuz değer JSON'da geçersiz olduğundan ölçülmemiş en yakın engel None olur.
    """
    if (state == 'running' and last_recovery is not None and now is not None
            and now - last_recovery < RECOVERY_HOLD):
        state = 'recovering'
    parts = [goal_parts(g) for g in goals]
    status = {
        'v': STATUS_VERSION, 'mode': mode, 'state': state,
        'scenario': scenario, 'run_id': run_id, 'results_dir': results_dir,
        't0': None if t0 is None else round(t0, 3),
        't': None if t0 is None or now is None else round(now - t0, 2),
        'goal_index': current, 'goal_count': len(goals), 'goal': None,
        'goals': [[x, y, round(math.degrees(yaw), 1), source] for x, y, yaw, source in parts],
        'goal_results': [r['result'] for r in results],
        'recoveries': sum(r['recoveries'] for r in results) + current_recoveries,
        'distance': round(distance, 3),
        'min_obstacle': round(min_distance, 3) if math.isfinite(min_distance) else None,
    }
    if current is not None:
        x, y, yaw, _ = parts[current - 1]
        status['goal'] = [x, y, round(math.degrees(yaw), 1)]
    if state == 'finished':
        failed = next((r['result'] for r in results
                       if r['result'] not in ('SUCCEEDED', CANCELED)), None)
        status['result'] = failed or 'SUCCEEDED'
    return status


def replace_queue(goals, done, goal):
    """
    Panelin hedefi kuyruğa: bitmiş (iptal edilen dahil) ilk done hedef kalır, gerisi yerine goal.

    Dönen (yeni kuyruk, bırakılan hedefler); goal kuyrukta done + 1. sıradadır.
    """
    return list(goals[:done]) + [goal], list(goals[done:])


def run_outcome(goals, results):
    """
    Koşunun özeti: (ilk başarısız hedefin satırı ya da None, varılan hedef, sayılan hedefler).

    Panelin iptal ettiği hedef (CANCELED) başarısızlık sayılmaz ve sayılan hedeflere girmez.
    Sayılan hedefler kuyruğun iptal edilmeyenleridir, yürütülmeden kalanlar da (stop_on_failure)
    dahil; panel müdahale etmediyse senaryonun hedefleri, yani Faz 8'deki goal_count.
    """
    canceled = {r['goal_index'] for r in results if r['result'] == CANCELED}
    counted = [g for i, g in enumerate(goals, 1) if i not in canceled]
    failed = next((r for r in results if r['result'] not in ('SUCCEEDED', CANCELED)), None)
    reached = sum(r['result'] == 'SUCCEEDED' for r in results)
    return failed, reached, counted


def panel_event(data):
    """
    /panel/events iletisi (çözülmüş JSON) -> (tür, alanlar); geçersizse None.

    spawn: name, x, y, size ([en, boy, yükseklik]) · delete: name · finish: alan yok.
    """
    if not isinstance(data, dict):
        return None
    kind = data.get('type')
    if kind == 'finish':
        return kind, {}
    name = data.get('name')
    if kind not in PANEL_EVENTS or not isinstance(name, str) or not name:
        return None
    if kind == 'delete':
        return kind, {'name': name}
    try:
        x, y = float(data['x']), float(data['y'])
        size = [float(v) for v in data.get('size') or ()]
    except (KeyError, TypeError, ValueError):
        return None
    if not (math.isfinite(x) and math.isfinite(y)) or len(size) != 3:
        return None
    return kind, {'name': name, 'x': round(x, 3), 'y': round(y, 3), 'size': size}


def event_line(t, kind, fields):
    """Olay kaydının bir satırı (JSON): t = sim zamanı − t0 (yörüngeyle aynı köken), tür, alan."""
    return json.dumps({'t': round(t, 2), 'type': kind, **fields}, ensure_ascii=False,
                      allow_nan=False)


def git_commit(path):
    """Dosyanın (sembolik bağlantı çözülerek) bulunduğu git deposunun kısa commit'i; yoksa None."""
    try:
        done = subprocess.run(
            ['git', '-C', os.path.dirname(os.path.realpath(path)), 'rev-parse', '--short', 'HEAD'],
            capture_output=True, text=True, timeout=5.0)
    except (OSError, subprocess.TimeoutExpired):
        return None
    return done.stdout.strip() or None if done.returncode == 0 else None


def world_path(world):
    """Senaryodaki dünya: paketin worlds/ klasörüne göre ya da mutlak yol (sim.launch.py gibi)."""
    return os.path.join(get_package_share_directory('otonom_surus'), 'worlds', world)


def run_meta(scenario, run_id, source, mode, t0, scenario_def, world_file, params_file,
             commit):
    """
    meta/s<N>_<run_id>.json'un içeriği: koşunun kimliği (JSON'a hazır sözlük).

    Tekrar oynatma dünyayı buradan çizebilsin diye geometri de yazılır (dünya dosyası sonradan
    değişse ya da silinse bile, P9); okunamazsa None.
    """
    try:
        geometry = world_geometry(world_file)
    except (OSError, ET.ParseError, ValueError, AttributeError):
        geometry = None
    return {
        'v': STATUS_VERSION, 'scenario': scenario, 'run_id': run_id, 'source': source,
        'mode': mode, 't0': round(t0, 3), 'world_file': world_file, 'world_geometry': geometry,
        'scenario_def': {
            'name': scenario_def['name'], 'world': scenario_def['world'],
            'goals': [[x, y, round(math.degrees(yaw), 1)] for x, y, yaw in scenario_def['goals']],
            'moving_obstacles': scenario_def['moving_obstacles'],
        },
        'params_file': params_file, 'git_commit': commit, 'detailed': False,
    }


def goal_pose(x, y, yaw, stamp):
    """Hedef pozu: map çerçevesinde (x, y) konumu ve yaw yönü."""
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = stamp
    pose.pose.position.x, pose.pose.position.y = x, y
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


class MissionManager(BasicNavigator):
    """scenarios.yaml (ya da panel) hedefleri -> NavigateToPose; sonuçlar -> results_dir/."""

    def __init__(self):
        super().__init__(node_name='mission_manager')
        share = get_package_share_directory('otonom_surus')
        self.scenario = self.declare_parameter('scenario', 1).value
        self.run_id = (self.declare_parameter('run_id', '').value
                       or time.strftime('%Y%m%d-%H%M%S'))
        self.goal_timeout = self.declare_parameter('goal_timeout', 180.0).value
        self.stop_on_failure = self.declare_parameter('stop_on_failure', False).value
        self.results_dir = os.path.abspath(os.path.expanduser(
            self.declare_parameter('results_dir', 'results').value))
        self.scenarios_file = self.declare_parameter('scenarios_file', os.path.join(
            share, 'config', 'scenarios.yaml')).value
        # Kip ve kaynak meta'ya yazılır; params_file yalnızca kayıt için (Nav2'yi launch açar).
        self.mode = self.declare_parameter('mode', 'scenario').value
        self.source = self.declare_parameter('source', 'terminal').value
        self.params_file = self.declare_parameter('params_file', os.path.join(
            share, 'config', 'nav2_params.yaml')).value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.recording = False  # ilk hedef gönderilince başlar
        self.odom_xy = None  # son /odom konumu
        self.odom_track = PathRecorder(0.01)  # metrikler: Gazebo'nun gerçek konumu
        self.map_track = PathRecorder(0.05)  # RViz izi: robotun SLAM'daki konumu
        self.min_distance = math.inf
        self.t0 = 0.0
        # /mission_status için görevin durumu
        self.state = 'waiting_nav2'
        self.goals = []
        self.results = []  # biten hedeflerin satırları
        self.current = None  # yürütülen hedefin sırası (1'den)
        self.current_recoveries = 0
        self.last_recovery = None  # kurtarma sayısının son arttığı sim zamanı
        self.t_end = None
        # Web paneli: bekleyen elle hedef (x, y, yaw, 'panel'), bitir isteği, kayıt başlamadan
        # gelen olaylar (sim zamanı, tür, alanlar) ve elle verilen hedef sayısı.
        self.pending_goal = None
        self.finish_requested = False
        self.early_events = []
        self.manual_goals = 0

        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.create_subscription(Float32, 'min_obstacle_distance', self.on_distance, 10)
        self.create_subscription(PoseStamped, 'panel/goal', self.on_panel_goal, 10)
        self.create_subscription(String, 'panel/events', self.on_panel_event, 10)
        self.path_pub = self.create_publisher(Path, 'traveled_path', 10)
        self.status_pub = self.create_publisher(String, 'mission_status', LATCHED)
        self.create_timer(0.2, self.update_traveled_path)
        self.create_timer(1.0, self.publish_status)

    def publish_status(self):
        """Görevin durumunu /mission_status'a JSON olarak yayınlar (web paneli için)."""
        started = self.state != 'waiting_nav2'
        status = mission_status(
            self.scenario, self.run_id, self.results_dir, self.state, self.goals, self.results,
            now=self.t_end if self.t_end is not None else self.sim_time(),
            t0=self.t0 if started else None, current=self.current,
            current_recoveries=self.current_recoveries, last_recovery=self.last_recovery,
            distance=self.odom_track.length, min_distance=self.min_distance, mode=self.mode)
        self.status_pub.publish(String(data=json.dumps(status, ensure_ascii=False)))

    def record_path(self, folder, ext):
        """Koşunun kayıt dosyası: results_dir/<folder>/s<N>_<run_id><ext>."""
        return os.path.join(self.results_dir, folder, f's{self.scenario}_{self.run_id}{ext}')

    def on_panel_goal(self, msg):
        """
        Panelden elle hedef: kuyruğa ana döngüde girer (yürütülen hedefi navigate() iptal eder).

        Nav2 hazır olmadan ve görev bittikten sonra gelen hedef yok sayılır; panel de o anlarda
        hedef göndermez. Üst üste gelirse sonuncusu geçerli.
        """
        if self.state == 'waiting_nav2' or self.t_end is not None or self.finish_requested:
            self.warn('Panelin hedefi yok sayıldı: görev hedef almaya hazır değil')
            return
        if msg.header.frame_id not in ('', 'map'):
            self.warn(f'Panelin hedefi yok sayıldı: çerçeve map değil ({msg.header.frame_id})')
            return
        p, q = msg.pose.position, msg.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pending_goal = (round(p.x, 3), round(p.y, 3), yaw, 'panel')

    def on_panel_event(self, msg):
        """Panel olayı: finish görevi bitirir, spawn ve delete olay kaydına yazılır."""
        try:
            event = panel_event(json.loads(msg.data))
        except ValueError:
            event = None
        if event is None:
            self.warn(f'Panel olayı anlaşılamadı: {msg.data[:80]!r}')
            return
        kind, fields = event
        if kind == 'finish':
            if not self.finish_requested:
                self.info('Panel görevi bitirdi; sonuçlar yazılacak')
            self.finish_requested = True
            return
        self.record_event(kind, fields)

    def record_event(self, kind, fields):
        """
        Olayı events/s<N>_<run_id>.jsonl'a yazar; t = sim zamanı − t0 (yörüngeyle aynı köken).

        Kayıt başlamadan (Nav2 beklenirken) gelen olay saklanır, başlayınca negatif t ile yazılır:
        tekrar oynatmada baştan beri vardır. Koşu bittikten sonra gelen olay yazılmaz.
        """
        if self.t_end is not None:
            return
        if not self.recording:
            self.early_events.append((self.sim_time(), kind, fields))
            return
        self.write_event(self.sim_time() - self.t0, kind, fields)

    def write_event(self, t, kind, fields):
        path = self.record_path('events', '.jsonl')
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path, 'a', encoding='utf-8') as f:
                f.write(event_line(t, kind, fields) + '\n')
        except OSError as e:
            self.warn(f'Olay kaydedilemedi ({kind}): {e}')

    def write_meta(self, scenario):
        """
        meta/s<N>_<run_id>.json: sonuçlarla birlikte bir kez.

        Görev başında değil sonda yazılır: ilk hedefin gönderilmesi gecikmesin (süre ölçümü
        Faz 8'deki gibi kalsın); sonuç satırı olmayan koşunun meta'sı zaten işe yaramaz.
        """
        meta = run_meta(self.scenario, self.run_id, self.source, self.mode, self.t0, scenario,
                        world_path(scenario['world']), self.params_file,
                        git_commit(self.scenarios_file))
        path = self.record_path('meta', '.json')
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path, 'w', encoding='utf-8') as f:
                json.dump(meta, f, ensure_ascii=False, indent=1, allow_nan=False)
                f.write('\n')
        except OSError as e:
            self.warn(f'Koşunun meta kaydı yazılamadı: {e}')

    def sim_time(self):
        """Saniye cinsinden sim zamanı (use_sim_time ile /clock)."""
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.odom_xy = (p.x, p.y)
        if self.recording:
            self.odom_track.add(self.sim_time() - self.t0, p.x, p.y)

    def on_distance(self, msg):
        if self.recording:
            self.min_distance = min(self.min_distance, msg.data)

    def update_traveled_path(self):
        """
        Robotun map çerçevesindeki konumunu ize ekler ve izi yayınlar.

        RViz'in sabit çerçevesi map olduğundan iz de map'te yayınlanır; RViz'in dönüşüm
        yapması gerekmez (obstacle_detector'daki işaretçilerle aynı gerekçe).
        """
        if not self.recording:
            return
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_footprint', Time())
        except TransformException:
            return
        self.map_track.add(0.0, t.transform.translation.x, t.transform.translation.y)

        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        for _, x, y in self.map_track.points:
            pose = PoseStamped(header=path.header)
            pose.pose.position.x, pose.pose.position.y = x, y
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.path_pub.publish(path)

    def _waitForNodeToActivate(self, node_name):
        """
        Düğüm 'active' olana kadar bekler; BasicNavigator'ınkinden farkı yanıt zaman aşımı.

        BasicNavigator get_state yanıtını süresiz bekliyor. DDS keşfi tamamlanmadan gelen
        istekte yanıt ara sıra kayboluyor (bt_navigator: "failed to send response to
        /bt_navigator/get_state (timeout)") ve görev hiç başlamıyordu (Faz 7'de ~20 koşuda 1).
        Burada yanıt 2 s içinde gelmezse istek yeniden gönderilir.
        """
        client = self.create_client(GetState, f'{node_name}/get_state')
        while not client.wait_for_service(timeout_sec=1.0):
            self.info(f'{node_name}/get_state servisi bekleniyor...')
        state = 'unknown'
        while state != 'active':
            future = client.call_async(GetState.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            if future.done() and future.result() is not None:
                state = future.result().current_state.label
            elif not future.done():
                self.info(f'{node_name}/get_state yanıt vermedi, yeniden soruluyor')
            if state != 'active':
                time.sleep(1.0)
        self.destroy_client(client)

    def wait_until_localized(self):
        """
        İlk hedeften önce SLAM'ın map -> base_footprint dönüşümünü ve taze /odom'u bekler.

        Sonra kuyrukları boşaltmak için yarım saniye daha döner. Nav2 beklenirken düğüm uzun
        aralıklarla döndüğünden saat (/clock) bayat kalabiliyor; başlangıç zamanı bayat saatle
        alınınca süre ~2 s fazla ölçülüyordu (sim süresi gerçek süreyi aşıyordu).
        """
        self.odom_xy = None
        start = time.monotonic()
        while not (self.odom_xy is not None and self.tf_buffer.can_transform(
                'map', 'base_footprint', Time())):
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.monotonic() - start > 5.0:
                self.info('map -> base_footprint dönüşümü ve /odom bekleniyor...')
                start = time.monotonic()
        drain_until = time.monotonic() + 0.5
        while time.monotonic() < drain_until:
            rclpy.spin_once(self, timeout_sec=0.05)

    def cancel_and_wait(self):
        """Yürütülen hedefi iptal eder, sonucu en çok 5 s bekler; tamamlandıysa True."""
        self.cancelTask()
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            if self.isTaskComplete():
                return True
        return False

    def navigate(self, index, goal):
        """
        Tek hedefe gider; hedefin sonuç satırını döndürür.

        Panelden yeni hedef ya da bitir isteği gelirse hedef iptal edilir (CANCELED) ve olay
        kaydına 'cancel' yazılır; iptal yetişmeden varılmışsa sonuç SUCCEEDED kalır.
        """
        x, y, yaw, source = goal_parts(goal)
        origin = ' (panel)' if source == 'panel' else ''
        self.info(f'Hedef {index}/{len(self.goals)}{origin}: ({x:.2f}, {y:.2f}), '
                  f'yaw {math.degrees(yaw):.0f}°')
        start, start_length = self.sim_time(), self.odom_track.length
        self.feedback = None
        recoveries = 0
        result = None
        self.current, self.current_recoveries = index, 0
        self.publish_status()
        if not self.goToPose(goal_pose(x, y, yaw, self.get_clock().now().to_msg())):
            result = 'REJECTED'
        while result is None and not self.isTaskComplete():
            if self.feedback is not None:
                recoveries = max(recoveries, self.feedback.number_of_recoveries)
            if recoveries > self.current_recoveries:
                self.current_recoveries, self.last_recovery = recoveries, self.sim_time()
                self.publish_status()
            if self.pending_goal is not None or self.finish_requested:
                reason = 'bitir' if self.finish_requested else 'panel hedefi'
                self.info(f'Hedef {index}: iptal ediliyor ({reason})')
                done = self.cancel_and_wait()
                result = ('SUCCEEDED' if done and self.getResult() == TaskResult.SUCCEEDED
                          else CANCELED)
                if result == CANCELED:
                    self.record_event('cancel', {'index': index, 'reason': reason})
            elif self.sim_time() - start > self.goal_timeout:
                self.warn(f'Hedef {index}: {self.goal_timeout:.0f} s içinde varılamadı, '
                          'iptal ediliyor')
                self.cancel_and_wait()
                result = 'TIMEOUT'
        if result is None:
            result = self.getResult().name
        if self.feedback is not None:
            recoveries = max(recoveries, self.feedback.number_of_recoveries)

        error = math.hypot(x - self.odom_xy[0], y - self.odom_xy[1])
        self.info(f'Hedef {index}/{len(self.goals)}: {result}, hedefe {error:.2f} m, '
                  f'{recoveries} kurtarma')
        return {
            'goal_index': index, 'goal_x': x, 'goal_y': y,
            'goal_yaw_deg': round(math.degrees(yaw), 1), 'result': result,
            'duration_s': round(self.sim_time() - start, 2),
            'path_length_m': round(self.odom_track.length - start_length, 3),
            'recoveries': recoveries, 'final_error_m': round(error, 3),
        }

    def run(self):
        """Görevi baştan sona yürütür ve sonuçları yazar."""
        if self.mode not in MODES:
            raise ValueError(f'mode {MODES} biri olmalı: {self.mode!r}')
        scenario = load_scenario(self.scenarios_file, self.scenario)
        interactive = self.mode == 'interactive'
        self.goals = [] if interactive else [(*g, 'scenario') for g in scenario['goals']]
        plan = 'serbest sürüş (hedefler panelden)' if interactive else f'{len(self.goals)} hedef'
        self.info(f'Senaryo {self.scenario} ({scenario["name"]}), koşu {self.run_id}: '
                  f'{plan}; sonuçlar {self.results_dir}')
        self.publish_status()  # waiting_nav2

        # T3: varsayılan localizer='amcl' SLAM modunda sonsuza kadar bekler (AMCL yok;
        # slam_toolbox Humble'da lifecycle düğümü değil).
        self.waitUntilNav2Active(localizer='bt_navigator')
        self.wait_until_localized()

        start_xy = self.odom_xy
        self.t0, wall_start = self.sim_time(), time.monotonic()
        self.odom_track.add(0.0, *start_xy)
        self.recording = True
        self.state = 'running'
        for stamp, kind, fields in self.early_events:
            self.write_event(stamp - self.t0, kind, fields)
        self.early_events = []
        results = self.results
        while not self.finish_requested:
            if self.pending_goal is not None:
                self.take_panel_goal()
            if len(results) < len(self.goals):
                self.state = 'running'
                results.append(self.navigate(len(results) + 1, self.goals[len(results)]))
                self.current, self.current_recoveries = None, 0  # artık results'ta sayılıyor
                self.publish_status()
                if (results[-1]['result'] not in ('SUCCEEDED', CANCELED)
                        and self.stop_on_failure and self.pending_goal is None):
                    break
            elif interactive:
                if self.state != 'idle':
                    self.state = 'idle'
                    self.publish_status()
                    self.info('Panelden hedef bekleniyor')
                rclpy.spin_once(self, timeout_sec=0.1)
            else:
                break
        self.recording = False
        self.t_end = self.sim_time()
        self.write_results(self.goals, results, start_xy, time.monotonic() - wall_start)
        self.write_meta(scenario)
        self.state = 'finished'
        self.publish_status()
        self.save_map()

    def take_panel_goal(self):
        """Bekleyen elle hedef kuyruğa girer; yürütülmemiş hedeflerin yerine geçer (olay: goal)."""
        goal, self.pending_goal = self.pending_goal, None
        done = len(self.results)
        self.goals, dropped = replace_queue(self.goals, done, goal)
        self.manual_goals += 1
        x, y, yaw, _ = goal
        self.record_event('goal', {'index': done + 1, 'x': x, 'y': y,
                                   'yaw': round(math.degrees(yaw), 1), 'source': 'panel'})
        note = f'; yürütülmemiş {len(dropped)} hedef bırakıldı' if dropped else ''
        self.info(f'Panelden hedef E{self.manual_goals}: ({x:.2f}, {y:.2f}), '
                  f'yaw {math.degrees(yaw):.0f}°, sırası {done + 1}{note}')

    def write_results(self, goals, results, start_xy, wall_duration):
        """runs.csv, goals.csv ve yörünge dosyasını yazar."""
        stamp = time.strftime('%Y-%m-%dT%H:%M:%S')
        common = {'timestamp': stamp, 'scenario': self.scenario, 'run_id': self.run_id}
        failed, reached, counted = run_outcome(goals, results)
        straight = straight_line_length(start_xy, [goal_parts(g)[:2] for g in counted])
        path_length = self.odom_track.length
        run = {
            **common,
            'result': 'SUCCEEDED' if failed is None else failed['result'],
            'goals_reached': reached, 'goal_count': len(counted),
            'duration_s': round(self.sim_time() - self.t0, 2),
            'wall_duration_s': round(wall_duration, 2),
            'path_length_m': round(path_length, 3),
            'straight_line_m': round(straight, 3),
            # Yalnızca başarılı koşuda anlamlı: kuş uçuşu / gidilen yol (1 = dümdüz).
            'path_efficiency': (round(straight / path_length, 3)
                                if failed is None and counted and path_length > 0 else ''),
            'min_obstacle_dist_m': (round(self.min_distance, 3)
                                    if math.isfinite(self.min_distance) else ''),
            'recoveries': sum(r['recoveries'] for r in results),
            'failure_reason': '' if failed is None else (
                f'hedef {failed["goal_index"]} ({failed["goal_x"]:.2f}, {failed["goal_y"]:.2f}): '
                f'{failed["result"]}, hedefe {failed["final_error_m"]:.2f} m kala'),
        }

        append_csv(os.path.join(self.results_dir, 'runs.csv'), RUN_FIELDS, run)
        for r in results:
            append_csv(os.path.join(self.results_dir, 'goals.csv'), GOAL_FIELDS,
                       {**common, **r})
        trajectory = os.path.join(
            self.results_dir, 'trajectories', f's{self.scenario}_{self.run_id}.csv')
        os.makedirs(os.path.dirname(trajectory), exist_ok=True)
        with open(trajectory, 'w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow(['t', 'x', 'y'])
            for t, x, y in self.odom_track.points:
                writer.writerow([round(t, 2), round(x, 4), round(y, 4)])

        self.info(
            f'Görev bitti: {run["result"]}, {reached}/{len(counted)} hedef, '
            f'{run["duration_s"]} s (sim), {run["path_length_m"]} m yol, '
            f'en yakın yüzey {run["min_obstacle_dist_m"]} m, {run["recoveries"]} kurtarma '
            f'-> {self.results_dir}')

    def save_map(self):
        """
        SLAM haritasını maps/s<N>_<run_id>.pgm ve .yaml olarak kaydeder (modül ① kanıtı).

        nav2_map_server'ın map_saver_cli aracını çağırır; slam_toolbox bu düğüm çıkana kadar
        çalıştığından /map hâlâ yayında. Varsayılan 2 s'lik abonelik beklemesi yetmeyip
        "Failed to spin map subscription" verebildiği için 10 s verilir (T12). Kayıt
        başarısız olursa yalnızca uyarı yazılır; koşunun sonuçları zaten yazılmıştır.
        """
        prefix = os.path.join(self.results_dir, 'maps', f's{self.scenario}_{self.run_id}')
        os.makedirs(os.path.dirname(prefix), exist_ok=True)
        saver = os.path.join(
            get_package_prefix('nav2_map_server'), 'lib', 'nav2_map_server', 'map_saver_cli')
        try:
            done = subprocess.run(
                [saver, '-f', prefix, '--ros-args', '-p', 'save_map_timeout:=10.0'],
                capture_output=True, text=True, timeout=30.0)
        except (OSError, subprocess.TimeoutExpired) as e:
            self.warn(f'Harita kaydedilemedi: {e}')
            return
        if done.returncode == 0:
            self.info(f'Harita kaydedildi: {prefix}.pgm')
        else:
            last = (done.stderr.strip().splitlines() or ['?'])[-1]
            self.warn(f'Harita kaydedilemedi (map_saver_cli çıkış kodu {done.returncode}): '
                      f'{last}')


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    status = 0
    try:
        node.run()
    except ValueError as e:  # scenarios.yaml'da senaryo yok ya da biçimi bozuk
        node.get_logger().error(str(e))
        status = 1
    except (KeyboardInterrupt, ExternalShutdownException):
        node.get_logger().warn('Görev yarıda kesildi; sonuç yazılmadı.')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return status


if __name__ == '__main__':
    main()
