"""
panel_control'ün testleri: elle hedefin ve atılan kutunun ön kontrolleri, geometri yardımcıları.

Denetimler paketteki gerçek dünyalarla (senaryo2: zikzak koridor ve kapıları, senaryo3: hareketli
kutular) sınanır. Son testte Controller gerçek bir rclpy düğümüyle kurulur ama döndürülmez:
Gazebo olmadan yalnızca servise gitmeden önceki yollar denenir.
"""

import math
import os

from otonom_surus.panel_control import (
    aabb_distance, box_aabb, BOX_SIZE, check_box, check_goal, ControlError, Controller, goal_msg,
    lane_aabbs, MAX_BOXES, occupied_near, overlaps, shape_distance)
from otonom_surus.scenarios import load_scenario
from otonom_surus.world_sdf import world_geometry
import pytest
import rclpy
from rclpy.context import Context
from rclpy.time import Time

PACKAGE = os.path.join(os.path.dirname(__file__), '..')
WORLDS = os.path.join(PACKAGE, 'worlds')
SCENARIOS = os.path.join(PACKAGE, 'config', 'scenarios.yaml')
S2 = world_geometry(os.path.join(WORLDS, 'senaryo2.world'))
S3 = world_geometry(os.path.join(WORLDS, 'senaryo3.world'))


def box(name, x, y):
    return {'name': name, 'x': x, 'y': y, 'size': list(BOX_SIZE)}


def test_dikdortgen_ve_sekil_uzakliklari():
    a = box_aabb(1.0, 1.0)
    assert a == pytest.approx((0.8, 0.8, 1.2, 1.2))
    assert overlaps(a, box_aabb(1.3, 1.0)) and not overlaps(a, box_aabb(1.4, 1.0))  # değen: yok
    assert aabb_distance(1.0, 1.0, a) == 0.0 and aabb_distance(1.5, 1.6, a) == pytest.approx(0.5)
    cylinder = {'kind': 'cylinder', 'x': 2.0, 'y': -0.4, 'yaw': 0.0, 'size': 0.3}
    assert shape_distance(2.0, 0.4, cylinder) == pytest.approx(0.5)
    assert shape_distance(2.0, -0.4, cylinder) == 0.0
    # 90° dönük 1 x 0.2 kutu: kendi x ekseni dünyanın y'si.
    rotated = {'kind': 'box', 'x': 0.0, 'y': 0.0, 'yaw': math.pi / 2, 'size': [1.0, 0.2]}
    assert shape_distance(0.0, 0.6, rotated) == pytest.approx(0.1)
    assert shape_distance(0.3, 0.0, rotated) == pytest.approx(0.2)


def test_hareketli_kutularin_seritleri():
    movers = load_scenario(SCENARIOS, 3)['moving_obstacles']
    lanes = lane_aabbs(S3, movers)
    assert len(lanes) == 2
    shapes = {s['model']: s for s in S3['shapes']}
    for lane, mover in zip(lanes, movers):
        shape = shapes[mover['name']]
        width, height = lane[2] - lane[0], lane[3] - lane[1]
        # Şerit kutunun kendi +x ekseni boyunca travel kadar uzar (engel2 +y yönünde).
        along = width if abs(math.cos(shape['yaw'])) > 0.5 else height
        assert along == pytest.approx(mover['travel'] + max(shape['size']), abs=1e-3)
    assert lane_aabbs(S3, [{'name': 'yok', 'travel': 1.0, 'speed': 0.1}]) == []


def test_kutu_kapiyi_kapatabilir_ama_duvara_ve_birbirine_giremez():
    # S2'nin alt kapısı (y = 1.0, x 3.6–4.4): iki kutu kapının hemen kuzeyinde yan yana.
    assert check_box(3.8, 1.3, S2) is None
    assert check_box(4.2, 1.3, S2, [box('panel_kutu_1', 3.8, 1.3)]) is None  # kenarları değiyor
    assert check_box(4.0, 1.0, S2) is None  # kapının içi de serbest
    assert 'duvarla' in check_box(3.0, 1.0, S2)  # iç duvarın üstü
    assert 'engelle' in check_box(2.4, 1.55, S2)  # kutu_1'in üstü
    assert 'başka bir kutuyla' in check_box(3.9, 1.3, S2, [box('panel_kutu_1', 3.8, 1.3)])
    assert 'odanın dışına' in check_box(5.1, 2.0, S2)
    assert 'robota çok yakın' in check_box(1.0, 2.0, S2, robot=(1.0, 1.65))
    assert check_box(1.0, 2.0, S2, robot=(1.0, 1.4)) is None  # kutunun kenarına 0.4 m
    full = [box(f'panel_kutu_{i}', 0.0, 2.0) for i in range(MAX_BOXES)]
    assert f'En çok {MAX_BOXES}' in check_box(1.0, 2.0, S2, full)


def test_kutu_hareketli_kutunun_yolunu_kesemez():
    lanes = lane_aabbs(S3, load_scenario(SCENARIOS, 3)['moving_obstacles'])
    shapes = {s['model']: s for s in S3['shapes']}
    e1 = shapes['engel1']
    assert 'yolunu kesiyor' in check_box(e1['x'] + 0.7, e1['y'], S3, lanes=lanes)
    assert check_box(e1['x'] + 0.7, e1['y'], S3) is None  # şeritsiz denetimde serbest


def grid_with(cells, w=40, h=40, res=0.05, ox=-1.0, oy=-1.0):
    """Genişliği w, yüksekliği h hücre ızgara (bilinmeyen = -1); cells: {(i, j): değer}."""
    data = [-1] * (w * h)
    for (i, j), v in cells.items():
        data[j * w + i] = v
    return {'w': w, 'h': h, 'res': res, 'ox': ox, 'oy': oy, 'data': data}


def test_haritada_dolu_hucre_yakinligi():
    # (0, 0) noktası hücre (20, 20)'nin köşesinde.
    grid = grid_with({(22, 20): 100, (30, 30): 50, (25, 20): 0})
    assert occupied_near(grid, 0.0, 0.0, 0.125)  # 0.1 m ötede dolu hücre
    assert not occupied_near(grid, 0.0, 0.0, 0.05)
    assert not occupied_near(grid, 0.5, 0.5, 0.125)  # 50: ne dolu ne boş (map_saver eşiği 65)
    assert not occupied_near(grid, 5.0, 5.0, 0.2)  # harita dışı: dolu sayılmaz
    assert not occupied_near(grid_with({}), 0.0, 0.0, 0.3)  # bilinmeyen dolu sayılmaz


def test_hedef_on_kontrolu():
    assert check_goal(4.0, 4.0, S2) is None
    assert check_goal(0.0, 2.0, S2) is None  # orta koridor
    assert 'odanın dışında' in check_goal(6.0, 2.0, S2)
    assert 'duvara' in check_goal(1.0, 1.1, S2)  # iç duvarın 0.025 m kuzeyi
    assert 'engele' in check_goal(2.0, -0.4, S2)  # silindir_1'in içi
    assert 'kutunun' in check_goal(1.05, 2.0, S2, [box('panel_kutu_1', 1.2, 2.0)])
    # Dolu hücrenin merkezi (1.025, 2.025).
    grid = grid_with({(40, 60): 100}, w=200, h=200, ox=-1.0, oy=-1.0)
    assert 'SLAM' in check_goal(1.0, 2.0, S2, grid=grid)
    assert check_goal(1.0, 2.5, S2, grid=grid) is None
    assert check_goal(10.0, 10.0, None) is None  # dünya bilinmiyorsa yalnızca harita denetlenir


def test_hedef_iletisi_map_cercevesinde_ve_yonu_dogru():
    msg = goal_msg(2.0, 1.0, 90.0, Time(seconds=5).to_msg())
    assert msg.header.frame_id == 'map' and msg.header.stamp.sec == 5
    q = msg.pose.orientation
    assert (msg.pose.position.x, msg.pose.position.y) == (2.0, 1.0)
    assert math.isclose(2 * math.atan2(q.z, q.w), math.pi / 2)


@pytest.fixture
def controller():
    context = Context()
    rclpy.init(context=context)
    # Ad alanı: aynı ROS_DOMAIN_ID'de açık bir Gazebo olsa da onun servisleri bulunmasın.
    node = rclpy.create_node('test_panel_control', namespace='/panel_control_testi',
                             context=context)
    events = []
    ctl = Controller(node, lambda event, data: events.append((event, data)))
    yield ctl, events
    node.destroy_node()
    rclpy.shutdown(context=context)


def test_denetleyici_reddi_ve_servis_yoklugu(controller):
    ctl, events = controller
    assert events == [('obstacles', {'boxes': []})]  # kurulunca boş liste duyurulur
    with pytest.raises(ControlError) as e:
        ctl.spawn_box(1.0, 2.0, lambda boxes: 'Kutu robota çok yakın.')
    assert e.value.status == 400 and ctl.boxes_list() == []
    # Gazebo yok: servis 1 s beklenir, 503.
    with pytest.raises(ControlError) as e:
        ctl.spawn_box(1.0, 2.0, lambda boxes: None)
    assert e.value.status == 503 and 'spawn_entity' in str(e.value)
    with pytest.raises(ControlError) as e:
        ctl.delete_boxes('panel_kutu_9')
    assert e.value.status == 404
    assert ctl.delete_boxes() == []  # kaldırılacak kutu yok
