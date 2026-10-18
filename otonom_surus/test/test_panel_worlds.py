"""panel_worlds'ün testleri: tasarımın denetimi, kayıt, numaralar, düzenleme ve silme (ROS'suz)."""

import copy
import os

from otonom_surus.panel_runs import RUN_FIELDS
from otonom_surus.panel_worlds import clean_design, DesignError, FIRST_ID, WorldStore
from otonom_surus.scenarios import load_scenario, scenario_numbers
from otonom_surus.world_sdf import design_geometry, world_geometry
import pytest

SAMPLE = {
    'name': 'Örnek parkur',
    'walls': [[-1.0, 1.2, 2.4, 1.2], [0.8, 2.8, 5.0, 2.8]],
    'boxes': [[3.6, 0.3, 0.5, 0.5], [1.6, 1.55, 0.5, 0.5], [3.0, 3.8, 0.5, 0.5]],
    'cylinders': [[4.2, 2.0, 0.25], [1.6, 4.4, 0.25]],
    'goals': [[4.2, 4.2, 90], [0.0, 4.3, 180]],
}


def design(**changes):
    d = copy.deepcopy(SAMPLE)
    d.update(changes)
    return d


@pytest.fixture
def store(tmp_path):
    return WorldStore(str(tmp_path / 'panel'), reserved=[1, 2, 3, 4, 5])


def test_kayit_dunya_ve_senaryo_dosyasi(store):
    entry = store.save(SAMPLE)
    assert entry['id'] == FIRST_ID and entry['file'] == 'ozel_100.world'
    assert entry['name'] == 'Örnek parkur'
    path = entry['world_file']
    assert os.path.isabs(path) and os.path.dirname(path) == store.worlds_dir
    assert world_geometry(path) == design_geometry(clean_design(SAMPLE))
    # Senaryo dosyası paketinkiyle aynı okuyucuyla okunur (sim.launch.py, mission_manager).
    assert scenario_numbers(store.scenarios_file) == [100] and store.owns(100)
    s = load_scenario(store.scenarios_file, 100)
    assert (s['name'], s['world'], s['moving_obstacles']) == ('Örnek parkur', path, [])
    assert [(x, y) for x, y, _ in s['goals']] == [(4.2, 4.2), (0.0, 4.3)]


def test_numaralar_artar_duzenleme_ustune_yazar(store):
    first = store.save(SAMPLE)
    second = store.save(design(name=''))
    assert second['id'] == 101 and second['name'] == 'Tasarlanan dünya 101'
    edited = store.save(design(id=100, name='Yeni ad', boxes=[], goals=[[4.0, 4.0, 0]]))
    assert edited['id'] == 100 and edited['world_file'] == first['world_file']
    assert store.numbers() == [100, 101]
    [e100, e101] = store.entries()
    assert e100['name'] == 'Yeni ad' and e100['design']['boxes'] == []
    assert e100['goals'] == [[4.0, 4.0, 0.0]] == e100['design']['goals']
    assert e101['design']['walls'] == SAMPLE['walls']  # düzenlemek için geri kurulur
    with pytest.raises(KeyError):
        store.save(design(id=150))  # kayıtlı olmayan dünyanın üstüne yazılmaz
    with pytest.raises(DesignError):
        store.save(design(id='100'))


def test_silme_ve_kosusu_olan_numara_yeniden_verilmez(store):
    store.save(SAMPLE)
    store.save(SAMPLE)
    removed = store.delete(101)
    assert removed['name'] == 'Örnek parkur'
    assert store.numbers() == [100]
    assert not os.path.exists(os.path.join(store.worlds_dir, 'ozel_101.world'))
    with pytest.raises(KeyError):
        store.delete(101)
    # 101'in koşusu yok: yeniden verilebilir. 100 silinir ama koşusu runs.csv'de: verilmez.
    with open(os.path.join(store.root, 'runs.csv'), 'w', encoding='utf-8') as f:
        f.write(','.join(RUN_FIELDS) + '\n')
        f.write('2026-10-18T10:00:00,100,p1,SUCCEEDED,2,2,1,1,1,1,1,1,0,\n')
    store.delete(100)
    assert store.save(SAMPLE)['id'] == 101
    assert store.save(SAMPLE)['id'] == 102


def test_paketin_numaralari_ve_bos_dosya(tmp_path):
    store = WorldStore(str(tmp_path / 'p'), reserved=[1, 2, 100, 104])
    assert store.numbers() == [] and store.entries() == []
    assert store.save(SAMPLE)['id'] == 105


def test_bozuk_dunya_dosyasi_listede_hata(store):
    entry = store.save(SAMPLE)
    with open(entry['world_file'], 'w', encoding='utf-8') as f:
        f.write('<sdf><world>')
    [e] = store.entries()
    assert 'error' in e and 'geometry' not in e


def test_yon_araliga_getirilir_sayilar_yuvarlanir():
    d = clean_design(design(goals=[[4.2, 4.2, 270], [0.0, 4.3, -180], [2.0, 4.0, 45.04]],
                            boxes=[[3.6004, 0.3, 0.5, 0.5]]))
    assert [g[2] for g in d['goals']] == [-90.0, 180.0, 45.0]
    assert d['boxes'] == [[3.6, 0.3, 0.5, 0.5]]
    assert clean_design(design(name='  Kendi   parkurum '))['name'] == 'Kendi parkurum'


@pytest.mark.parametrize('changes, message', [
    ({'walls': [[-1.0, 0.3, 2.0, 0.3]]}, '(0, 0) noktasının 0.5 m'),
    ({'boxes': [[0.5, 0.0, 0.5, 0.5]]}, 'Kutu 1 çok yakın'),
    ({'cylinders': [[0.0, -0.6, 0.25]]}, 'Silindir 1 çok yakın'),
    ({'boxes': [[4.9, 4.0, 0.5, 0.5]]}, 'Kutu 1 odanın dışına'),
    ({'cylinders': [[2.0, 5.0, 0.25]]}, 'Silindir 1 odanın dışına'),
    ({'walls': [[2.0, 2.0, 6.0, 2.0]]}, 'Duvar 1 odanın dışına'),
    ({'walls': [[0.0, 2.0, 2.0, 3.0]]}, 'eksenlere hizalı'),
    ({'walls': [[1.0, 2.0, 1.2, 2.0]]}, 'en az 0.3 m'),
    ({'boxes': [[2.0, 2.0, 0.05, 0.5]]}, 'en ve boy'),
    ({'cylinders': [[2.0, 2.0, 2.0]]}, 'yarıçap'),
    ({'goals': []}, 'En az bir hedef'),
    ({'goals': [[2.0, 2.0, 0]] * 3 + [[3.0, 2.0, 0], [2.0, 2.0, 0], [3.0, 2.0, 0]]}, 'En çok 5'),
    ({'goals': [[3.6, 0.3, 0]]}, 'Hedef 1 bir engele çok yakın'),
    ({'goals': [[2.0, 1.25, 0]]}, 'Hedef 1 bir duvara çok yakın'),
    ({'goals': [[4.95, 2.0, 0]]}, 'Hedef 1 odanın dışında'),
    ({'goals': [[0.3, 0.2, 0]]}, 'robotun başlangıcına çok yakın'),
    ({'goals': [[4.2, 4.2, 0], [4.4, 4.4, 0]]}, 'Hedef 2 bir öncekine'),
    ({'boxes': [[1.0, 1.0, 'yarım', 0.5]]}, 'Kutu 1 4 sayıdan'),
    ({'boxes': [[1.0, 1.0, True, 0.5]]}, 'Kutu 1 4 sayıdan'),
    ({'goals': [[1.0, float('nan'), 0]]}, 'sonlu'),
    ({'walls': 'duvar'}, 'liste olmalı'),
    ({'name': 'x' * 41}, 'en çok 40'),
    ({'name': 'a\nb'}, None),  # boşluklar birleşir: geçerli
    ({'boxes': [[2.0 + 0.1 * (i % 20), 2.0 + 0.5 * (i // 20), 0.1, 0.1] for i in range(41)]},
     'En çok 40 şekil'),
])
def test_kurala_uymayan_tasarim_reddedilir(changes, message):
    if message is None:
        assert clean_design(design(**changes))['name'] == 'a b'
        return
    with pytest.raises(DesignError, match=message.replace('(', r'\(').replace(')', r'\)')):
        clean_design(design(**changes))


def test_denetim_karakterli_ad_reddedilir():
    with pytest.raises(DesignError, match='denetim karakteri'):
        clean_design(design(name='a\x07b'))
