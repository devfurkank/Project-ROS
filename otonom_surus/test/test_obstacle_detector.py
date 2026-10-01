"""obstacle_detector kümeleme fonksiyonlarının ROS gerektirmeyen birim testleri."""

import math
import random

from geometry_msgs.msg import Quaternion
from otonom_surus.obstacle_detector import (
    covers_full_circle, describe_cluster, find_clusters, planar_transform, scan_to_points,
    yaw_of)

# TurtleBot3 Burger LiDAR'ı: 0 .. 6.28 rad arası 360 ışın, 0.12 .. 3.5 m.
N = 360
INC = 6.28 / (N - 1)
RANGE_MIN, RANGE_MAX = 0.12, 3.5


def ray_to_circle(angle, cx, cy, r):
    """Başlangıçtan çıkan ışının (cx, cy) merkezli çembere uzaklığı; kesmiyorsa inf."""
    dx, dy = math.cos(angle), math.sin(angle)
    b = dx * cx + dy * cy
    disc = b * b - (cx * cx + cy * cy - r * r)
    if disc < 0 or b - math.sqrt(disc) <= 0:
        return math.inf
    return b - math.sqrt(disc)


def simulate(circles=(), wall_y=None, noise=0.0):
    """Çemberler ve isteğe bağlı y = wall_y duvarı için sentetik bir tarama üretir."""
    rng = random.Random(0)
    ranges = []
    for i in range(N):
        a = i * INC
        hits = [ray_to_circle(a, *c) for c in circles]
        if wall_y is not None and math.sin(a) * wall_y > 0:
            hits.append(wall_y / math.sin(a))
        r = min(hits, default=math.inf) + rng.gauss(0.0, noise)
        ranges.append(r if r <= RANGE_MAX else math.inf)
    return ranges


def clusters_of(ranges):
    """Taramayı düğümdeki gibi kümeler."""
    points = scan_to_points(ranges, 0.0, INC, RANGE_MIN, RANGE_MAX)
    return find_clusters(points, INC, covers_full_circle(len(ranges), INC))


def polar(r, deg):
    """Kutupsal koordinatı kartezyene çevirir."""
    return r * math.cos(math.radians(deg)), r * math.sin(math.radians(deg))


def test_tam_tur_algilanir():
    assert covers_full_circle(N, INC)
    assert not covers_full_circle(180, INC)


def test_gecersiz_olcumler_atilir():
    ranges = [math.inf, math.nan, 0.05, 4.0, 1.0]
    points = scan_to_points(ranges, 0.0, INC, RANGE_MIN, RANGE_MAX)
    assert [p.range for p in points] == [1.0]


def test_tek_silindirin_merkezi_ve_yaricapi():
    clusters = clusters_of(simulate(circles=[(1.5, 0.8, 0.3)], noise=0.01))
    assert len(clusters) == 1
    cx, cy, radius = describe_cluster(clusters[0])
    assert math.hypot(cx - 1.5, cy - 0.8) < 0.1
    assert 0.2 < radius < 0.4


def test_iki_ayri_engel_iki_kume():
    clusters = clusters_of(simulate(circles=[(1.5, 1.0, 0.2), (-1.0, 1.5, 0.2)]))
    assert len(clusters) == 2


def test_aradan_isin_gecen_farkli_uzakliktaki_engeller_birlesmez():
    # Yakın engel ile arkadaki engel arasında birkaç ışın boşa gidiyor (inf).
    ranges = simulate(circles=[(*polar(1.7, 20), 0.1), (*polar(3.0, 32), 0.2)])
    clusters = clusters_of(ranges)
    assert len(clusters) == 2


def test_sifir_derece_dogrultusundaki_engel_bolunmez():
    ranges = simulate(circles=[(1.0, 0.0, 0.2)])
    assert math.isfinite(ranges[0]) and math.isfinite(ranges[-1])
    clusters = clusters_of(ranges)
    assert len(clusters) == 1
    cx, cy, _ = describe_cluster(clusters[0])
    assert math.hypot(cx - 1.0, cy) < 0.1


def test_az_noktali_kume_atilir():
    ranges = [math.inf] * N
    ranges[100] = ranges[101] = 2.0
    assert clusters_of(ranges) == []


def test_duzlemsel_donusum():
    yaw = yaw_of(Quaternion(z=math.sin(math.pi / 4), w=math.cos(math.pi / 4)))
    assert math.isclose(yaw, math.pi / 2)
    x, y = planar_transform(1.0, 0.0, 2.0, 3.0, yaw)
    assert math.isclose(x, 2.0, abs_tol=1e-9) and math.isclose(y, 4.0)


def test_egik_duvar_tek_parca_kalir():
    # Uzak uçlarda noktalar 0.15 m'den seyrek; uyarlamalı eşik duvarı yine de bölmez.
    ranges = simulate(wall_y=1.0, noise=0.01)
    clusters = clusters_of(ranges)
    assert len(clusters) == 1
    assert len(clusters[0]) == sum(math.isfinite(r) for r in ranges)
    _, _, radius = describe_cluster(clusters[0])
    assert radius > 0.75
