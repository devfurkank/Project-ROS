"""
LiDAR taramasından engel çıkaran düğüm.

/scan ölçümleri açı sırasıyla gezilir; ardışık iki nokta arasındaki mesafe uyarlamalı bir
eşikten küçükse ikisi aynı kümeye girer. Yeterince noktası olan her küme bir engeldir:
küçükse silindir, büyükse (duvar, köşe) noktalarından geçen bir çizgi olarak RViz'e gider.

Konular:
    scan                   (sensor_msgs/LaserScan)           girdi
    detected_obstacles     (visualization_msgs/MarkerArray)  işaretçiler (çerçeve: frame_id)
    min_obstacle_distance  (std_msgs/Float32)                LiDAR'dan en yakın yüzeye (m)
"""

from collections import namedtuple
import math

from geometry_msgs.msg import Point
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Header
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

ScanPoint = namedtuple('ScanPoint', 'range x y')


def scan_to_points(ranges, angle_min, angle_increment, range_min, range_max):
    """Geçerli ölçümleri (inf, nan ve aralık dışı atılarak) kartezyen noktalara çevirir."""
    points = []
    for i, r in enumerate(ranges):
        if math.isfinite(r) and range_min <= r <= range_max:
            angle = angle_min + i * angle_increment
            points.append(ScanPoint(r, r * math.cos(angle), r * math.sin(angle)))
    return points


def covers_full_circle(n, angle_increment):
    """Tarama 360° kapsıyor mu, yani son ışın ilk ışının komşusu mu."""
    return n > 1 and 2.0 * math.pi - (n - 1) * angle_increment <= 1.5 * angle_increment


def breakpoint_distance(r, angle_increment, min_gap, min_incidence, sigma):
    """
    İki ardışık noktanın aynı kümede sayılacağı en büyük mesafe.

    Uyarlamalı kırılma noktası eşiği (Borges ve Aldon, 2004): ışın uzadıkça ve yüzey ışına
    eğik durdukça komşu ışınların noktaları seyrekleşir, eşik de buna göre büyür. r önceki
    noktanın uzaklığı, min_incidence yüzeyin ışınla yapabileceği en dar açı, sigma mesafe
    ölçüm gürültüsünün standart sapmasıdır. Eşik min_gap'ten küçük olmaz.

    Arada geçersiz ışın olsa da açı farkı olarak tarama çözünürlüğü kullanılır: gerçek fark
    min_incidence'a yaklaştıkça formül sonsuza gider ve aralarından ışın geçen, farklı
    uzaklıktaki iki engeli birleştirirdi.
    """
    adaptive = (r * math.sin(angle_increment) / math.sin(min_incidence - angle_increment)
                + 3.0 * sigma)
    return max(min_gap, adaptive)


def find_clusters(points, angle_increment, full_circle, min_points=3, min_gap=0.15,
                  min_incidence=math.radians(10.0), sigma=0.01):
    """
    Noktaları açı sırasıyla kümelere ayırır; min_points'ten az noktalı kümeleri atar.

    full_circle ise son küme ilk kümeyle birleşebilir; böylece 0° doğrultusundaki bir engel
    iki parçaya bölünmez.
    """
    def same_cluster(p, q):
        gap = math.hypot(q.x - p.x, q.y - p.y)
        return gap <= breakpoint_distance(
            p.range, angle_increment, min_gap, min_incidence, sigma)

    clusters = []
    for p in points:
        if clusters and same_cluster(clusters[-1][-1], p):
            clusters[-1].append(p)
        else:
            clusters.append([p])

    if full_circle and len(clusters) > 1 and same_cluster(clusters[-1][-1], clusters[0][0]):
        clusters[0] = clusters.pop() + clusters[0]

    return [c for c in clusters if len(c) >= min_points]


def describe_cluster(cluster):
    """
    Kümenin merkezini ve yarıçapını (cx, cy, r) döndürür.

    Merkez, kümenin iki uç noktasının orta noktasıdır. LiDAR bir silindirin yalnızca robota
    bakan yarısını gördüğünden noktaların ağırlık merkezi engelin önüne kayar; uç noktaların
    ortası ise gerçek merkeze çok daha yakındır. Yarıçap, merkezden en uzak noktanın mesafesi.
    """
    first, last = cluster[0], cluster[-1]
    cx, cy = (first.x + last.x) / 2.0, (first.y + last.y) / 2.0
    radius = max(math.hypot(p.x - cx, p.y - cy) for p in cluster)
    return cx, cy, radius


def yaw_of(q):
    """Kuaterniyonun z ekseni etrafındaki dönüş açısı (rad)."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def planar_transform(x, y, tx, ty, yaw):
    """(x, y) noktasını z ekseni etrafında yaw kadar döndürüp (tx, ty) kadar öteler."""
    c, s = math.cos(yaw), math.sin(yaw)
    return tx + c * x - s * y, ty + s * x + c * y


def cylinder_marker(header, marker_id, x, y, z, radius):
    """Engeli LiDAR düzlemine (z) ortalanmış yarı saydam kırmızı bir silindirle gösterir."""
    m = Marker(header=header, ns='engeller', id=marker_id,
               type=Marker.CYLINDER, action=Marker.ADD)
    m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
    m.pose.orientation.w = 1.0
    m.scale.x = m.scale.y = 2.0 * radius
    m.scale.z = 0.4
    m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.2, 0.1, 0.6
    return m


def wall_marker(header, marker_id, points, z):
    """Büyük kümeyi (duvar, köşe) noktalarından geçen gri bir çizgiyle gösterir."""
    m = Marker(header=header, ns='duvarlar', id=marker_id,
               type=Marker.LINE_STRIP, action=Marker.ADD)
    m.pose.orientation.w = 1.0
    m.scale.x = 0.04
    m.color.r, m.color.g, m.color.b, m.color.a = 0.75, 0.75, 0.75, 0.9
    m.points = [Point(x=x, y=y, z=z) for x, y in points]
    return m


class ObstacleDetector(Node):
    """/scan -> /detected_obstacles ve /min_obstacle_distance."""

    def __init__(self):
        super().__init__('obstacle_detector')
        self.min_points = self.declare_parameter('min_points', 3).value
        self.min_gap = self.declare_parameter('cluster_tolerance', 0.15).value
        self.min_incidence = math.radians(
            self.declare_parameter('min_incidence_deg', 10.0).value)
        self.range_sigma = self.declare_parameter('range_sigma', 0.01).value
        self.max_radius = self.declare_parameter('max_obstacle_radius', 0.75).value
        # İşaretçilerin yayınlanacağı çerçeve; boşsa tarama çerçevesi (base_scan) kullanılır.
        self.frame_id = self.declare_parameter('frame_id', 'odom').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pending = None  # TF'i beklenen son tarama: (header, engeller, duvarlar)
        self.skipped = 0  # TF gelmediği için art arda atlanan tarama sayısı

        self.marker_pub = self.create_publisher(MarkerArray, 'detected_obstacles', 10)
        self.distance_pub = self.create_publisher(Float32, 'min_obstacle_distance', 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.create_timer(0.02, self.publish_pending)

    def on_scan(self, scan):
        points = scan_to_points(
            scan.ranges, scan.angle_min, scan.angle_increment, scan.range_min, scan.range_max)
        clusters = find_clusters(
            points, scan.angle_increment,
            covers_full_circle(len(scan.ranges), scan.angle_increment),
            self.min_points, self.min_gap, self.min_incidence, self.range_sigma)

        nearest = min((p.range for p in points), default=math.inf)
        self.distance_pub.publish(Float32(data=nearest))

        obstacles, walls = [], []
        for cluster in clusters:
            cx, cy, radius = describe_cluster(cluster)
            if radius <= self.max_radius:
                obstacles.append((cx, cy, radius))
            else:
                walls.append([(p.x, p.y) for p in cluster])
        self.get_logger().info(
            f'{len(obstacles)} engel, {len(walls)} duvar; en yakın yüzey {nearest:.2f} m',
            throttle_duration_sec=5.0)

        if self.pending is not None:
            self.skipped += 1
            # ~5 s; açılışta slam_toolbox'ın map -> odom'u yayınlamasını beklerken uyarma
            if self.skipped >= 25:
                self.get_logger().warn(
                    f'{self.frame_id} <- {scan.header.frame_id} dönüşümü gelmiyor; '
                    f'{self.skipped} taramanın işaretçileri atlandı', throttle_duration_sec=5.0)
        self.pending = (scan.header, obstacles, walls)
        self.publish_pending()

    def publish_pending(self):
        """
        Bekleyen taramanın işaretçilerini frame_id çerçevesinde yayınlar.

        Tarama, robotun o anki konumunu veren odom TF'inden birkaç ms önce gelir. RViz'in
        MarkerArray ekranı TF'i beklemez; işaretçiler tarama çerçevesinde yayınlansa dönüşüm
        çoğu zaman başarısız olur ve işaretçiler yanıp söner. Bu yüzden tarama anının TF'i
        gelene kadar beklenir (zamanlayıcı yeniden dener) ve noktalar burada taşınır.
        """
        if self.pending is None:
            return
        header, obstacles, walls = self.pending
        tx = ty = tz = yaw = 0.0
        if self.frame_id and self.frame_id != header.frame_id:
            try:
                t = self.tf_buffer.lookup_transform(
                    self.frame_id, header.frame_id, Time.from_msg(header.stamp))
            except TransformException:
                return
            tr = t.transform.translation
            tx, ty, tz, yaw = tr.x, tr.y, tr.z, yaw_of(t.transform.rotation)
            header = Header(stamp=header.stamp, frame_id=self.frame_id)
        self.pending = None
        self.skipped = 0

        # Önceki taramanın işaretçilerini sil; sonra bu taramanınkileri ekle.
        markers = MarkerArray()
        markers.markers.append(Marker(header=header, action=Marker.DELETEALL))
        for i, (cx, cy, radius) in enumerate(obstacles):
            x, y = planar_transform(cx, cy, tx, ty, yaw)
            markers.markers.append(cylinder_marker(header, i, x, y, tz, radius))
        for i, line in enumerate(walls):
            points = [planar_transform(x, y, tx, ty, yaw) for x, y in line]
            markers.markers.append(wall_marker(header, i, points, tz))
        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
