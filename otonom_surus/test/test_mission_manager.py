"""Senaryo okuma ve mission_manager metrik fonksiyonlarının ROS gerektirmeyen birim testleri."""

import csv
import math
import os
import xml.etree.ElementTree as ET

from otonom_surus.mission_manager import append_csv, PathRecorder, straight_line_length
from otonom_surus.scenarios import load_scenario
import pytest
import yaml

PACKAGE = os.path.join(os.path.dirname(__file__), '..')
SCENARIOS = os.path.join(PACKAGE, 'config', 'scenarios.yaml')


def write_yaml(tmp_path, data):
    """data'yı geçici bir scenarios.yaml'a yazar ve yolunu döndürür."""
    path = tmp_path / 'scenarios.yaml'
    path.write_text(yaml.safe_dump(data), encoding='utf-8')
    return str(path)


def test_paketteki_her_senaryonun_dunyasi_ve_hedefleri_var():
    with open(SCENARIOS, encoding='utf-8') as f:
        numbers = list(yaml.safe_load(f))
    assert {1, 2, 3} <= set(numbers)
    for number in numbers:
        scenario = load_scenario(SCENARIOS, number)
        assert os.path.isfile(os.path.join(PACKAGE, 'worlds', scenario['world']))
        assert scenario['goals']


def test_senaryo1_hedefi():
    scenario = load_scenario(SCENARIOS, '1')  # launch argümanı metin olarak gelir
    assert scenario['world'] == 'senaryo1.world'
    [(x, y, yaw)] = scenario['goals']
    assert (x, y) == (4.0, 4.0)
    assert math.isclose(yaw, math.pi / 2)
    assert scenario['moving_obstacles'] == []


def test_senaryo2_hedefleri_sirali():
    scenario = load_scenario(SCENARIOS, 2)
    assert [(x, y) for x, y, _ in scenario['goals']] == [(4.0, 0.0), (4.0, 4.0), (0.0, 4.0)]


def test_hareketli_engeller_dunyada_planar_move_ile_tanimli():
    """scenarios.yaml'daki her hareketli engel dünyada sürülebilir ve TF yayınlamıyor (T6)."""
    scenario = load_scenario(SCENARIOS, 3)
    names = [o['name'] for o in scenario['moving_obstacles']]
    assert names == ['engel1', 'engel2']
    world = ET.parse(os.path.join(PACKAGE, 'worlds', scenario['world'])).getroot()
    for obstacle in scenario['moving_obstacles']:
        assert obstacle['speed'] <= 0.15  # LiDAR 5 Hz (T10)
        [model] = [m for m in world.iter('model') if m.get('name') == obstacle['name']]
        assert model.findtext('static', 'false') == 'false'
        [plugin] = [p for p in model.findall('plugin')
                    if p.get('filename') == 'libgazebo_ros_planar_move.so']
        assert plugin.findtext('ros/namespace') == '/' + obstacle['name']
        assert plugin.findtext('publish_odom') == 'true'  # obstacle_mover konumu buradan alır
        assert plugin.findtext('publish_odom_tf') == 'false'


def test_bozuk_hareketli_engel_hata_verir(tmp_path):
    for obstacle in [{'name': 'e', 'travel': 1.0}, {'name': 'e', 'travel': 0, 'speed': 0.1},
                     {'travel': 1.0, 'speed': 0.1}, 'engel1']:
        path = write_yaml(tmp_path, {1: {'world': 'a.world', 'goals': [[1, 2, 0]],
                                         'moving_obstacles': [obstacle]}})
        with pytest.raises(ValueError, match='hareketli engel'):
            load_scenario(path, 1)


def test_tanimsiz_senaryo_hata_verir(tmp_path):
    path = write_yaml(tmp_path, {1: {'world': 'a.world', 'goals': [[1, 2, 0]]}})
    with pytest.raises(ValueError, match='senaryo 2 yok'):
        load_scenario(path, 2)
    with pytest.raises(ValueError, match='tamsayı'):
        load_scenario(path, 'bir')


def test_bozuk_hedef_hata_verir(tmp_path):
    path = write_yaml(tmp_path, {1: {'world': 'a.world', 'goals': [[1, 2]]}})
    with pytest.raises(ValueError, match='yaw_derece'):
        load_scenario(path, 1)
    path = write_yaml(tmp_path, {1: {'world': 'a.world', 'goals': []}})
    with pytest.raises(ValueError, match='goals boş'):
        load_scenario(path, 1)


def test_yol_kaydedici_titresimi_saymaz():
    track = PathRecorder(min_step=0.01)
    assert track.add(0.0, 0.0, 0.0)
    for i in range(100):  # robot dururken milimetre altı titreşim
        assert not track.add(0.1 * i, 0.001 * (i % 2), 0.0)
    track.add(10.0, 3.0, 0.0)
    track.add(11.0, 3.0, 4.0)
    assert math.isclose(track.length, 7.0)
    assert [p[1:] for p in track.points] == [(0.0, 0.0), (3.0, 0.0), (3.0, 4.0)]


def test_kus_ucusu_mesafe_hedefleri_sirayla_dolasir():
    goals = [(4.0, 0.0), (4.0, 4.0), (0.0, 4.0)]
    assert math.isclose(straight_line_length((0.0, 0.0), goals), 12.0)
    assert math.isclose(straight_line_length((0.0, 0.0), [(3.0, 4.0)]), 5.0)


def test_csv_basligi_bir_kez_yazilir(tmp_path):
    path = str(tmp_path / 'alt' / 'runs.csv')
    append_csv(path, ['a', 'b'], {'a': 1, 'b': 'ğüş'})
    append_csv(path, ['a', 'b'], {'a': 2, 'b': ''})
    with open(path, encoding='utf-8') as f:
        rows = list(csv.DictReader(f))
    assert rows == [{'a': '1', 'b': 'ğüş'}, {'a': '2', 'b': ''}]
