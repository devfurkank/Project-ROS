"""world_sdf'in (.world ayrıştırıcısı) ROS gerektirmeyen birim testleri, paketteki dünyalarla."""

import math
import os

from otonom_surus.world_sdf import world_geometry, world_shapes

WORLDS = os.path.join(os.path.dirname(__file__), '..', 'worlds')


def by_model(shapes):
    return {s['model']: s for s in shapes}


def roles(geometry):
    counts = {}
    for shape in geometry['shapes']:
        counts[shape['role']] = counts.get(shape['role'], 0) + 1
    return counts


def test_senaryo1_sekilleri_ve_donuk_kutu():
    shapes = world_shapes(os.path.join(WORLDS, 'senaryo1.world'))
    models = by_model(shapes)
    assert {'kutu_1', 'kutu_2', 'kutu_3', 'silindir_1', 'silindir_2', 'silindir_3'} <= set(models)
    assert 'hedef' not in models  # hedef diski yalnızca görsel, çarpışması yok
    assert 'ground_plane' not in models
    kutu = models['kutu_1']
    assert kutu['kind'] == 'box' and kutu['static']
    assert (kutu['x'], kutu['y']) == (1.5, 1.5)
    assert math.isclose(kutu['yaw'], 0.35)
    assert kutu['size'] == (0.6, 0.6)
    assert models['silindir_1']['kind'] == 'cylinder'
    assert models['silindir_1']['size'] == 0.3
    assert sum(1 for s in shapes if s['model'] == 'oda') == 4  # odanın dört duvarı


def test_rollere_gore_sayilar():
    assert roles(world_geometry(os.path.join(WORLDS, 'senaryo1.world'))) == \
        {'wall': 4, 'obstacle': 6}
    assert roles(world_geometry(os.path.join(WORLDS, 'senaryo2.world'))) == \
        {'wall': 8, 'obstacle': 4}  # oda + 4 iç duvar parçası
    assert roles(world_geometry(os.path.join(WORLDS, 'senaryo3.world'))) == \
        {'wall': 4, 'obstacle': 2, 'moving': 2}


def test_hareketli_kutular_statik_degil_ve_donukluk_korunur():
    geometry = world_geometry(os.path.join(WORLDS, 'senaryo3.world'))
    moving = {s['model']: s for s in geometry['shapes'] if s['role'] == 'moving'}
    assert set(moving) == {'engel1', 'engel2'}
    assert math.isclose(moving['engel2']['yaw'], math.pi / 2, abs_tol=1e-3)  # şeridi +y


def test_sinirlar_odayi_kapsar_ve_json_turleri():
    geometry = world_geometry(os.path.join(WORLDS, 'senaryo2.world'))
    x0, y0, x1, y1 = geometry['bounds']
    assert x0 <= -1.1 and y0 <= -1.1 and x1 >= 5.1 and y1 >= 5.1
    for shape in geometry['shapes']:
        if shape['kind'] == 'box':
            assert isinstance(shape['size'], list) and len(shape['size']) == 2
        else:
            assert isinstance(shape['size'], float)


def test_bos_dunyanin_siniri_yok(tmp_path):
    path = tmp_path / 'bos.world'
    path.write_text('<sdf version="1.6"><world name="w"><model name="ground_plane">'
                    '<static>true</static><link name="l"><collision name="c"><geometry>'
                    '<plane><normal>0 0 1</normal></plane></geometry></collision></link>'
                    '</model></world></sdf>')
    assert world_geometry(str(path)) == {'shapes': [], 'bounds': None}
