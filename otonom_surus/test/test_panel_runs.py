"""
panel_runs'ın testleri: koşu dizini, CSV okuma, varyant etiketi, önbellek ve yörünge.

Geçici klasörde sahte bir results/ ağacı kurulur (başlıksız CSV, eksik yörünge, Türkçe ve
virgüllü failure_reason, bozuk satır, taranmaması gereken klasörler). Son test depodaki gerçek
Faz 8 verisini okur (30 koşu); depo bulunamazsa atlanır.
"""

import collections
import json
import os
import pathlib

from otonom_surus.panel_runs import (
    GOAL_FIELDS, read_trajectory, RUN_FIELDS, RunIndex, variant_label)
import pytest

SCENARIOS = """
1:
  name: Dağınık statik engeller
  world: senaryo1.world
  goals:
    - [4.0, 4.0, 90.0]
2:
  name: Zikzak koridor
  world: /mutlak/yol/senaryo2.world
  goals:
    - [4.0, 0.0, 90.0]
    - [4.0, 4.0, 180.0]
"""

BASE_PARAMS = """
controller_server:
  ros__parameters:
    use_sim_time: true
    FollowPath:
      forward_prune_distance: 0.7
      max_vel_x: 0.22
"""

REASON = 'hedef 2 (4.00, 4.00): FAILED, hedefe 3.29 m kala'


def write(path, text):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text.lstrip('\n'), encoding='utf-8')


def trajectory_text(n, header=True):
    rows = [f'{0.2 * i:.1f},{0.01 * i:.4f},0.0' for i in range(n)]
    return '\n'.join((['t,x,y'] if header else []) + rows) + '\n'


@pytest.fixture
def tree(tmp_path):
    root = tmp_path / 'results'
    runs_header = ','.join(RUN_FIELDS)
    goals_header = ','.join(GOAL_FIELDS)
    write(root / 'runs.csv', f"""
{runs_header}
2026-10-15T14:17:29,1,r1,SUCCEEDED,1,1,28.1,28.22,5.756,5.657,0.983,0.372,0,
2026-10-15T14:22:16,2,r1,FAILED,1,2,217.1,217.94,12.284,12.0,,0.278,15,"{REASON}"
2026-10-15T14:30:00,,r9,SUCCEEDED,1,1,28.0,28.0,5.7,5.657,0.99,0.3,0,
""")
    write(root / 'goals.csv', f"""
{goals_header}
2026-10-15T14:17:29,1,r1,1,4.0,4.0,90.0,SUCCEEDED,28.1,5.756,0,0.233
2026-10-15T14:22:16,2,r1,2,4.0,4.0,180.0,FAILED,178.7,6.078,15,3.287
2026-10-15T14:22:16,2,r1,1,4.0,0.0,90.0,SUCCEEDED,22.2,3.848,0,0.227
""")
    write(root / 'trajectories' / 's1_r1.csv', trajectory_text(51))
    write(root / 'maps' / 's1_r1.yaml', 'image: s1_r1.pgm\n')
    # Parametre varyantı; CSV'leri başlıksız.
    write(root / 'fpd' / 'nav2_params.yaml', BASE_PARAMS.replace('0.7', '2.0'))
    write(root / 'fpd' / 'runs.csv',
          '2026-10-15T15:00:00,2,r1,SUCCEEDED,2,2,200.5,201.0,12.0,12.0,1.0,0.25,3,\n')
    write(root / 'fpd' / 'goals.csv',
          '2026-10-15T15:00:00,2,r1,1,4.0,0.0,90.0,SUCCEEDED,22.0,3.8,0,0.2\n'
          '2026-10-15T15:00:00,2,r1,2,4.0,4.0,180.0,SUCCEEDED,178.5,8.2,3,0.24\n')
    write(root / 'fpd' / 'trajectories' / 's2_r1.csv', trajectory_text(5, header=False))
    write(root / 'panel' / 'runs.csv', f"""
{runs_header}
2026-10-17T15:30:40,1,p20261017-153000,SUCCEEDED,1,1,27.8,27.7,5.77,5.657,0.98,0.35,0,
""")
    write(root / 'panel' / 'trajectories' / 's1_p20261017-153000.csv', trajectory_text(3))
    # Taranmayanlar: runs.csv'siz klasör, iki düzey derin klasör, gizli klasör.
    write(root / 'bos' / 'trajectories' / 's1_r1.csv', trajectory_text(3))
    write(root / 'fpd' / 'derin' / 'runs.csv', f'{runs_header}\n2026-10-16T00:00:00,1,d1,'
                                               'SUCCEEDED,1,1,1,1,1,1,1,1,0,\n')
    write(root / '.gizli' / 'runs.csv', f'{runs_header}\n2026-10-16T00:00:00,1,g1,'
                                        'SUCCEEDED,1,1,1,1,1,1,1,1,0,\n')
    write(tmp_path / 'scenarios.yaml', SCENARIOS)
    write(tmp_path / 'nav2_params.yaml', BASE_PARAMS)
    return RunIndex(str(root), panel_dir=str(root / 'panel'),
                    base_params=str(tmp_path / 'nav2_params.yaml'),
                    scenario_files=[str(tmp_path / 'scenarios.yaml'),
                                    str(root / 'panel' / 'panel_scenarios.yaml')])


def test_kosu_kimlikleri_en_yeni_once_ve_taranmayan_klasorler(tree):
    assert [r['id'] for r in tree.runs()] == [
        'panel/s1_p20261017-153000', 'fpd/s2_r1', 's2_r1', 's1_r1']


def test_alanlar_turkce_neden_bos_sayi_ve_hedefler(tree):
    r = tree.get('s2_r1')
    assert (r['scenario'], r['run_id'], r['result']) == (2, 'r1', 'FAILED')
    assert r['failure_reason'] == REASON
    assert r['path_efficiency'] is None and r['recoveries'] == 15
    assert (r['scenario_name'], r['world']) == ('Zikzak koridor', 'senaryo2.world')
    assert (r['label'], r['variant'], r['panel']) == ('temel', None, False)
    assert [(g['index'], g['result']) for g in r['goals']] == [(1, 'SUCCEEDED'), (2, 'FAILED')]
    assert r['goals'][1]['final_error_m'] == 3.287
    assert not r['files']['trajectory'] and not r['files']['map']
    s1 = tree.get('s1_r1')
    assert s1['files'] == {'trajectory': True, 'map': True, 'log': False, 'meta': False,
                           'events': False, 'obstacles': False, 'replay': False}
    assert tree.get('yok') is None
    json.dumps(tree.runs(), allow_nan=False)  # JSON'a hazır: NaN ve sonsuz yok


def test_varyant_basliksiz_csv_ve_panel_klasoru(tree):
    r = tree.get('fpd/s2_r1')
    assert r['variant'] == r['label'] == 'forward_prune_distance 2.0'
    assert (r['duration_s'], r['recoveries'], r['path_efficiency']) == (200.5, 3, 1.0)
    assert [g['duration_s'] for g in r['goals']] == [22.0, 178.5]
    assert r['files']['trajectory']
    p = tree.get('panel/s1_p20261017-153000')
    assert (p['label'], p['panel'], p['folder']) == ('panel', True, 'panel')


def test_bozuk_satir_uyarisi_ve_dunya_filtresi(tree):
    assert any('1 satır okunamadı' in w for w in tree.warnings())
    assert [r['id'] for r in tree.runs('senaryo1.world')] == [
        'panel/s1_p20261017-153000', 's1_r1']
    assert tree.runs('yok.world') == []


def _touch_later(path):
    """Değişim zamanını 1 s ileri alır (aynı nanosaniyeye düşme olasılığına karşı)."""
    st = os.stat(path)
    os.utime(path, ns=(st.st_atime_ns, st.st_mtime_ns + 10**9))


def test_onbellek_yalnizca_degisen_klasoru_yeniden_okur(tree):
    s1 = tree.get('s1_r1')
    fpd = tree.get('fpd/s2_r1')
    panel_runs = os.path.join(tree.panel_dir, 'runs.csv')
    with open(panel_runs, 'a', encoding='utf-8') as f:
        f.write('2026-10-17T16:00:00,2,p20261017-155800,ABORTED,0,2,5.0,5.0,0.5,12.0,,0.4,1,'
                '"hedef 1 (4.00, 0.00): ABORTED, hedefe 3.50 m kala"\n')
    _touch_later(panel_runs)
    runs = tree.runs()
    assert runs[0]['id'] == 'panel/s2_p20261017-155800' and runs[0]['result'] == 'ABORTED'
    assert runs[0]['failure_reason'].endswith('hedefe 3.50 m kala')
    assert tree.get('s1_r1') is s1 and tree.get('fpd/s2_r1') is fpd  # yeniden okunmadı

    # Yeni yörünge dosyası klasörün zamanını değiştirir -> bayrak güncellenir.
    write(pathlib.Path(tree.root) / 'trajectories' / 's2_r1.csv', trajectory_text(3))
    _touch_later(os.path.join(tree.root, 'trajectories'))
    assert tree.get('s2_r1')['files']['trajectory']
    assert tree.get('fpd/s2_r1') is fpd


def test_yorunge_okuma_seyreltme_ve_hatalar(tree, tmp_path):
    path = tmp_path / 'yorunge.csv'
    write(path, trajectory_text(51).replace('0.4,0.0200,0.0', 'bozuk,satır'))
    points = read_trajectory(str(path))
    assert len(points) == 50 and points[0] == [0.0, 0.0, 0.0] and points[-1][0] == 10.0
    thinned = read_trajectory(str(path), dt=1.0)
    assert [p[0] for p in thinned] == [float(t) for t in range(11)]  # ilk ve son kalır

    assert tree.trajectory('s1_r1') == read_trajectory(
        os.path.join(tree.root, 'trajectories', 's1_r1.csv'))
    assert len(tree.trajectory('fpd/s2_r1')) == 5  # başlıksız dosya
    with pytest.raises(FileNotFoundError):
        tree.trajectory('s2_r1')
    with pytest.raises(KeyError):
        tree.trajectory('s9_yok')


def test_varyant_etiketi(tmp_path):
    base = tmp_path / 'base.yaml'
    write(base, BASE_PARAMS)
    cases = {
        BASE_PARAMS: None,
        BASE_PARAMS.replace('0.7', '2.0'): 'forward_prune_distance 2.0',
        BASE_PARAMS.replace('      max_vel_x: 0.22\n', ''): 'max_vel_x (yok)',
        BASE_PARAMS.replace('true', 'false'): 'use_sim_time false',
        BASE_PARAMS.replace('true', 'false').replace('0.7', '2.0').replace('0.22', '0.3')
        + 'yeni: 1\n': 'use_sim_time false, forward_prune_distance 2.0, +2 parametre',
    }
    for text, label in cases.items():
        write(tmp_path / 'p.yaml', text)
        assert variant_label(str(tmp_path / 'p.yaml'), str(base)) == label, text
    assert variant_label(str(tmp_path / 'yok.yaml'), str(base)) is None


def test_sonuc_klasorunden_kimlik(tree, tmp_path):
    root = tree.root
    assert tree.id_for(root, 1, 'r1') == 's1_r1'
    assert tree.id_for(os.path.join(root, 'fpd'), 2, 'r1') == 'fpd/s2_r1'
    assert tree.id_for(tree.panel_dir, 1, 'p1') == 'panel/s1_p1'
    assert tree.id_for(str(tmp_path / 'baska'), 1, 'r1') is None
    outside = RunIndex(root, panel_dir=str(tmp_path / 'panel_disarida'))
    assert outside.id_for(str(tmp_path / 'panel_disarida'), 1, 'p1') == 'panel_disarida/s1_p1'


def test_alan_listeleri_mission_manager_ile_ayni():
    from otonom_surus import mission_manager
    assert RUN_FIELDS == mission_manager.RUN_FIELDS
    assert GOAL_FIELDS == mission_manager.GOAL_FIELDS


def test_gercek_depodaki_30_kosu():
    repo = os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
    results = os.path.join(repo, 'results')
    if not os.path.isfile(os.path.join(results, 'runs.csv')):
        pytest.skip(f'depo verisi yok: {results}')
    config = os.path.join(repo, 'otonom_surus', 'config')
    index = RunIndex(results, panel_dir=os.path.join(results, 'panel'),
                     base_params=os.path.join(config, 'nav2_params.yaml'),
                     scenario_files=[os.path.join(config, 'scenarios.yaml')])
    runs = [r for r in index.runs() if not r['panel']]
    assert collections.Counter(r['folder'] for r in runs) == {
        '': 15, 'zor_varyant': 10, 'parametre_fpd2': 5}
    assert {r['world'] for r in runs} == {'senaryo1.world', 'senaryo2.world', 'senaryo3.world'}
    assert [w for w in index.warnings() if index.panel_dir not in w] == []
    for r in runs:
        assert r['files']['trajectory'] and len(index.trajectory(r['id'])) > 100, r['id']
        assert len(r['goals']) == r['goal_count'], r['id']
    fpd = index.get('parametre_fpd2/s2_r1')
    assert fpd['label'] == 'forward_prune_distance 2.0'
    assert (fpd['result'], fpd['path_efficiency'], fpd['recoveries']) == ('FAILED', None, 15)
    assert [g['result'] for g in fpd['goals']] == ['SUCCEEDED', 'FAILED', 'SUCCEEDED']
    assert index.get('zor_varyant/s5_r3')['label'] == 'zor varyant'
    assert index.get('s1_r1')['label'] == 'temel'
