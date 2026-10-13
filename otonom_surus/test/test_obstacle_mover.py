"""obstacle_mover şerit mantığının ROS gerektirmeyen birim testleri."""

import math

from geometry_msgs.msg import Quaternion
from otonom_surus.obstacle_mover import Lane, yaw_of


def drive(lane, start, axis, speed, dt, steps):
    """Engeli lane'in verdiği yönde sürer; her adımdaki (ilerleme, yön) listesini döndürür."""
    x, y = start
    yaw = math.atan2(axis[1], axis[0])
    trace = []
    for _ in range(steps):
        direction = lane.update(x, y, yaw)
        x += direction * speed * dt * axis[0]
        y += direction * speed * dt * axis[1]
        trace.append(((x - start[0]) * axis[0] + (y - start[1]) * axis[1], direction))
    return trace


def test_serit_uclarinda_doner_ve_disina_cikmaz():
    lane = Lane(travel=1.5)
    trace = drive(lane, (0.4, 1.5), (1.0, 0.0), speed=0.15, dt=0.1, steps=600)  # 60 s
    progress = [p for p, _ in trace]
    assert min(progress) >= -0.015 - 1e-9  # bir adımlık (speed * dt) aşma payı
    assert max(progress) <= 1.5 + 0.015 + 1e-9
    turns = sum(1 for (_, a), (_, b) in zip(trace, trace[1:]) if a != b)
    assert turns == 5  # 60 s / (1.5 m / 0.15 m/s) = 6 yarım tur -> 5 dönüş


def test_serit_engelin_ilk_yonu_boyunca():
    lane = Lane(travel=1.0)
    trace = drive(lane, (3.1, 2.3), (0.0, 1.0), speed=0.1, dt=0.1, steps=150)  # yaw 90°
    assert lane.origin[:2] == (3.1, 2.3)
    assert trace[0][1] == 1.0  # önce +y yönünde
    assert trace[120][1] == -1.0  # 1 m sonra geri
    assert max(p for p, _ in trace) <= 1.0 + 0.01 + 1e-9


def test_eksene_dik_kayma_donusu_etkilemez():
    lane = Lane(travel=1.0)
    assert lane.update(0.0, 0.0, 0.0) == 1.0
    assert lane.update(0.5, 0.8, 0.0) == 1.0  # robot kutuyu 0.8 m yana itti
    assert lane.update(1.0, 0.8, 0.0) == -1.0
    assert lane.update(0.2, -0.3, 0.0) == -1.0
    assert lane.update(-0.01, 0.0, 0.0) == 1.0


def test_yaw_kuaterniyondan():
    for yaw in (0.0, 0.5, math.pi / 2, -2.0, math.pi - 0.01):
        q = Quaternion(z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))
        assert math.isclose(yaw_of(q), yaw, abs_tol=1e-9)
