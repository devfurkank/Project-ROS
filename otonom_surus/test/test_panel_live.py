"""
panel_live'ın testleri: harita PNG'si, yol seyreltme ve canlı veri köprüsünün olayları.

Köprü gerçek bir rclpy düğümüyle kurulur ama döndürülmez: geri çağrılar ve tick() doğrudan
çağrılır, TF tampona elle yazılır. Simülasyon gerekmez.
"""

import base64
import io
import json
import math

from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from otonom_surus.panel_live import (
    LiveBridge, MAP_EVERY, map_png, PATH_EVERY, thin_path, yaw_of)
from PIL import Image
import pytest
import rclpy
from rclpy.context import Context
from std_msgs.msg import Float32, String
from visualization_msgs.msg import Marker, MarkerArray


def test_harita_png_map_saver_tonlari_ve_kuzey_yukari():
    data = [-1, 0, 100,   # ızgaranın 0. satırı: haritanın altı
            100, 30, 10]
    image = Image.open(io.BytesIO(map_png(3, 2, data)))
    assert image.mode == 'L' and image.size == (3, 2)
    assert list(image.getdata()) == [0, 205, 254,    # resmin üstü = ızgaranın 1. satırı
                                     205, 254, 0]
    with pytest.raises(ValueError, match='hücre'):
        map_png(3, 3, data)


def test_yol_seyreltme_ilk_ve_son_nokta_kalir():
    points = [(float(i), 0.0) for i in range(11)]
    assert thin_path(points, 2.0) == [(0.0, 0.0), (2.0, 0.0), (4.0, 0.0), (6.0, 0.0),
                                      (8.0, 0.0), (10.0, 0.0)]
    assert thin_path(points[:2], 5.0) == points[:2]
    assert thin_path([], 1.0) == []


def test_dordeyden_yaw():
    for yaw in (0.0, 1.0, -2.5, math.pi / 2):
        q = Quaternion(z=math.sin(yaw / 2), w=math.cos(yaw / 2))
        assert math.isclose(yaw_of(q), yaw, abs_tol=1e-9)


@pytest.fixture
def bridge():
    context = Context()
    rclpy.init(context=context)
    node = rclpy.create_node('test_panel_live', context=context)
    events, statuses = [], []
    live = LiveBridge(node, lambda event, data: events.append((event, data)),
                      on_status=statuses.append, movers=['engel1'])
    yield live, events, statuses
    node.destroy_node()
    rclpy.shutdown(context=context)


def of(events, kind):
    return [data for event, data in events if event == kind]


def ticks(live, count):
    for _ in range(count):
        live.tick()


def transform(x, y, yaw, stamp):
    t = TransformStamped()
    t.header.frame_id, t.child_frame_id = 'map', 'base_footprint'
    t.header.stamp.sec, t.header.stamp.nanosec = int(stamp), round(stamp % 1 * 1e9)
    t.transform.translation.x, t.transform.translation.y = x, y
    t.transform.rotation.z, t.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return t


def path(points):
    msg = Path()
    for x, y in points:
        pose = PoseStamped()
        pose.pose.position.x, pose.pose.position.y = x, y
        msg.poses.append(pose)
    return msg


def grid(values, width, height):
    msg = OccupancyGrid()
    msg.info.width, msg.info.height, msg.info.resolution = width, height, 0.05
    msg.info.origin.position.x, msg.info.origin.position.y = -1.0, -2.0
    msg.data = values
    return msg


def test_konum_yalnizca_yeni_tf_ile_ve_metriklerle(bridge):
    live, events, _ = bridge
    live.tick()
    assert of(events, 'pose') == []  # map çerçevesi yok (SLAM başlamadı)

    odom = Odometry()
    odom.twist.twist.linear.x = 0.21
    live.on_odom(odom)
    live.on_nearest(Float32(data=float('inf')))  # menzilde yüzey yok
    live.on_markers(MarkerArray(markers=[
        Marker(action=Marker.DELETEALL), Marker(type=Marker.CYLINDER, action=Marker.ADD),
        Marker(type=Marker.CYLINDER, action=Marker.ADD),
        Marker(type=Marker.LINE_STRIP, action=Marker.ADD)]))
    mover = Odometry()
    mover.pose.pose.position.x, mover.pose.pose.orientation.w = 1.5, 1.0
    live.on_mover('engel1', mover)
    live.tf_buffer.set_transform(transform(1.0, 2.0, math.pi / 2, 10.5), 'test')
    ticks(live, 2)  # aynı TF: ikinci tick olay üretmez
    [pose] = of(events, 'pose')
    assert pose == {'x': 1.0, 'y': 2.0, 'th': 1.571, 't': 10.5, 'v': 0.21, 'nearest': None,
                    'clusters': 2, 'movers': {'engel1': [1.5, 0.0, 0.0]}}
    json.dumps(pose, allow_nan=False)

    live.on_nearest(Float32(data=0.4567))
    live.tf_buffer.set_transform(transform(1.1, 2.0, math.pi / 2, 10.6), 'test')
    live.tick()
    assert [p['x'] for p in of(events, 'pose')] == [1.0, 1.1]
    assert of(events, 'pose')[-1]['nearest'] == 0.457


def test_iz_ve_plan_degisince_en_cok_2_hz(bridge):
    live, events, _ = bridge
    live.on_trail(path([(0.0, 0.0), (0.123, 0.456)]))
    live.on_plan(path([(0.05 * i, 0.0) for i in range(41)]))  # 2 m, 5 cm aralıklı
    ticks(live, PATH_EVERY - 1)
    assert of(events, 'trail') == [] and of(events, 'plan') == []
    live.tick()
    assert of(events, 'trail') == [{'pts': [[0.0, 0.0], [0.12, 0.46]]}]
    [plan] = of(events, 'plan')
    assert plan['pts'][0] == [0.0, 0.0] and plan['pts'][-1] == [2.0, 0.0]
    assert 15 <= len(plan['pts']) <= 22  # 0.1 m'ye seyreltildi

    live.on_trail(path([(0.0, 0.0), (0.123, 0.456)]))  # aynı iz yeniden geldi
    ticks(live, PATH_EVERY)
    assert len(of(events, 'trail')) == 1
    live.on_trail(path([(0.0, 0.0), (0.123, 0.456), (0.5, 0.5)]))
    ticks(live, PATH_EVERY)
    assert of(events, 'trail')[-1]['pts'][-1] == [0.5, 0.5]


def test_harita_degisince_en_cok_1_hz(bridge):
    live, events, _ = bridge
    live.on_map(grid([0, 100, -1, 0], 2, 2))
    ticks(live, MAP_EVERY - 1)
    assert of(events, 'map') == []
    live.tick()
    [event] = of(events, 'map')
    assert (event['w'], event['h'], event['res'], event['ox'], event['oy']) == (
        2, 2, 0.05, -1.0, -2.0)
    image = Image.open(io.BytesIO(base64.b64decode(event['png'])))
    assert list(image.getdata()) == [205, 254, 254, 0]

    live.on_map(grid([0, 100, -1, 0], 2, 2))  # aynı içerik, yeni ileti
    ticks(live, MAP_EVERY)
    assert len(of(events, 'map')) == 1
    live.on_map(grid([0, 100, 100, 0], 2, 2))
    ticks(live, MAP_EVERY)
    assert len(of(events, 'map')) == 2
    assert live.snapshot()['map'] == of(events, 'map')[-1]


def test_durum_iletilir_ve_sifirlama_her_seyi_unutur(bridge):
    live, events, statuses = bridge
    status = {'v': 1, 'state': 'running', 'run_id': 'p1', 'scenario': 2}
    live.on_status(String(data=json.dumps(status)))
    live.on_status(String(data='bozuk {'))
    assert of(events, 'status') == [status] and statuses == [status]

    live.on_trail(path([(0.0, 0.0), (1.0, 0.0)]))
    live.on_map(grid([0] * 4, 2, 2))
    live.tf_buffer.set_transform(transform(1.0, 0.0, 0.0, 5.0), 'test')
    ticks(live, MAP_EVERY)
    snap = live.snapshot()
    assert snap['status'] == status and snap['trail'] and snap['map']
    assert snap['pose']['x'] == 1.0

    live.reset()
    assert live.snapshot() == {'status': None, 'map': None, 'trail': None, 'plan': None,
                               'pose': None}
    count = len(events)
    ticks(live, MAP_EVERY)
    assert len(events) == count  # eski TF ve harita silindi: olay yok


def test_baska_gorev_gorulunce_onceki_veri_unutulur(bridge):
    live, events, statuses = bridge
    live.on_status(String(data=json.dumps({'run_id': 'p1', 'state': 'finished'})))
    live.on_trail(path([(0.0, 0.0), (1.0, 0.0)]))
    live.on_map(grid([0] * 4, 2, 2))
    live.tf_buffer.set_transform(transform(4.0, 4.0, 0.0, 45.0), 'test')
    ticks(live, MAP_EVERY)
    assert live.snapshot()['map'] and live.snapshot()['pose']

    live.on_status(String(data=json.dumps({'run_id': 'p1', 'state': 'finished'})))
    assert live.snapshot()['trail']  # aynı görevin tekrarı: hiçbir şey silinmez
    other = {'run_id': 'izleme1', 'state': 'waiting_nav2'}
    live.on_status(String(data=json.dumps(other)))  # terminalden başlatılmış yeni görev
    assert live.snapshot() == {'status': other, 'map': None, 'trail': None, 'plan': None,
                               'pose': None}
    assert of(events, 'status')[-1] == other and statuses[-1] == other


def test_sim_zamani_geri_gidince_tf_temizlenir(bridge):
    live, events, _ = bridge

    def odom(stamp):
        msg = Odometry()
        msg.header.stamp.sec = stamp
        return msg

    live.on_odom(odom(45))
    live.tf_buffer.set_transform(transform(4.0, 4.0, 0.0, 45.0), 'test')
    live.tick()
    live.on_odom(odom(45))  # aynı zaman: bir şey olmaz
    live.on_odom(odom(0))  # simülasyon yeniden başladı
    live.tf_buffer.set_transform(transform(0.1, 0.0, 0.0, 0.2), 'test')  # eskisi kalsa atılırdı
    live.tick()
    assert [(p['x'], p['t']) for p in of(events, 'pose')] == [(4.0, 45.0), (0.1, 0.2)]
