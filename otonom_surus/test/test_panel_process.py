"""
panel_process'in ROS'suz birim testleri: satır ayrıştırma, süreç grubu, koşu aşamaları.

Koşunun aşaması /mission_status'tan izlenir; testlerde bu iletiler on_status'a doğrudan verilir.
"""

import os
import threading
import time

from otonom_surus.panel_process import (
    find_run, ManagedProcess, parse_line, RunError, RunManager, scenario_command)
import pytest

MM = '[mission_manager-7] [INFO] [1792226568.000105631] [mission_manager]: '


def test_mission_manager_satiri_iletilir():
    entry = parse_line(MM + 'Hedef 1/3: (4.00, 0.00), yaw 90°')
    assert entry == {'level': 'INFO', 'source': 'mission_manager',
                     'text': 'Hedef 1/3: (4.00, 0.00), yaw 90°'}


def test_diger_dugumlerin_bilgisi_iletilmez_uyarisi_iletilir():
    assert parse_line('[controller_server-3] [INFO] [1.5] [controller_server]: Received') is None
    entry = parse_line('[bt_navigator-5] [WARN] [1.5] [bt_navigator]: Timed out')
    assert entry == {'level': 'WARN', 'source': 'bt_navigator', 'text': 'Timed out'}


def test_launch_satirlari():
    assert parse_line('[INFO] [launch]: All log files can be found below /x') is None
    entry = parse_line('[ERROR] [gzserver-1]: process has died [pid 5, exit code -9]')
    assert entry == {'level': 'ERROR', 'source': 'gzserver-1',
                     'text': 'process has died [pid 5, exit code -9]'}
    assert parse_line('[WARNING] [launch]: user interrupted with ctrl-c')['level'] == 'WARN'


def test_gurultu_ve_bicimsiz_satir_atilir():
    assert parse_line('[gzclient-2] screen size is bogus 0x0, using 1920x1080') is None
    assert parse_line('serbest metin') is None


def test_komut_terminaldekiyle_ayni():
    assert scenario_command(2, True, False, '/r/panel', 'p1') == [
        'ros2', 'launch', 'otonom_surus', 'scenario.launch.py', 'scenario:=2', 'gui:=true',
        'rviz:=false', 'results_dir:=/r/panel', 'run_id:=p1']
    assert scenario_command(1, 0, 1, '/r', 'p2', '/s.yaml')[-1] == 'scenarios_file:=/s.yaml'


def test_sonuc_satiri_bulunur(tmp_path):
    (tmp_path / 'runs.csv').write_text(
        'scenario,run_id,result,goals_reached,goal_count,failure_reason\n'
        '1,p1,SUCCEEDED,1,1,\n2,p1,FAILED,2,3,"hedef 2: FAILED, kaldı"\n', encoding='utf-8')
    row = find_run(str(tmp_path), 2, 'p1')
    assert row['result'] == 'FAILED' and row['failure_reason'] == 'hedef 2: FAILED, kaldı'
    assert find_run(str(tmp_path), 3, 'p1') is None
    assert find_run(str(tmp_path / 'yok'), 1, 'p1') is None


def test_cikti_satir_satir_ve_gunluk_dosyasi(tmp_path):
    lines, done = [], []
    log = tmp_path / 'logs' / 'k.log'
    p = ManagedProcess(['bash', '-c', 'echo bir; echo "iki ğüş"; exit 3'], on_line=lines.append,
                       on_exit=done.append, log_path=str(log), leftover_grace=0.5).start()
    assert p.wait(10)
    assert lines == ['bir', 'iki ğüş'] and done == [3] and not p.running
    assert log.read_text(encoding='utf-8') == 'bir\niki ğüş\n'


def test_sigint_lidere_gider_ve_yeterse_orada_kalir():
    p = ManagedProcess(['sleep', '30'], leftover_grace=0.5).start()
    time.sleep(0.3)
    started = time.monotonic()
    assert p.stop(int_timeout=5) == 'SIGINT'
    assert time.monotonic() - started < 3
    assert p.wait(5) and not p.group_alive()


def test_lider_olup_cocuklar_kalirsa_grup_temizlenir():
    # SIGINT yalnızca bash'e gider; arka plandaki sleep'ler (SIGINT'i yok sayar) grupta kalır,
    # SIGTERM bütün grubu boşaltır.
    p = ManagedProcess(['bash', '-c', 'sleep 30 & sleep 30 & wait'], leftover_grace=0.5).start()
    time.sleep(0.3)
    assert p.stop(int_timeout=0.5, term_timeout=3) == 'SIGTERM'
    assert p.wait(5) and not p.group_alive()


def test_sigint_yok_sayilirsa_sigterm_ve_sigkill():
    p = ManagedProcess(['bash', '-c', 'trap "" INT; sleep 30'], leftover_grace=0.5).start()
    time.sleep(0.3)
    assert p.stop(int_timeout=0.5, term_timeout=3) == 'SIGTERM'
    p = ManagedProcess(['bash', '-c', 'trap "" INT TERM; sleep 30'], leftover_grace=0.5).start()
    time.sleep(0.3)
    assert p.stop(int_timeout=0.3, term_timeout=0.3, kill_timeout=3) == 'SIGKILL'
    assert p.wait(5) and not p.group_alive()


def test_surec_bitince_grupta_kalan_oldurulur():
    # Ana süreç çıkar, arka plandaki sleep grupta kalır (T22'deki gibi); izleyici öldürmeli.
    done = []
    p = ManagedProcess(['bash', '-c', 'sleep 30 & exit 0'], on_exit=done.append,
                       leftover_grace=0.3).start()
    assert p.wait(10)
    assert done == [0] and not p.group_alive()


def fake_launch(lines, csv_row=None, tail='exit 0'):
    """Sahte launch: mission_manager satırları basar, isteğe göre runs.csv'ye yazar."""
    def command(scenario, gui, rviz, results_dir, run_id, scenarios_file):
        script = ''.join(f'echo {MM + line!r}; sleep 0.05; ' for line in lines)
        if csv_row is not None:
            row = csv_row.format(scenario=scenario, run_id=run_id)
            script += (f'mkdir -p {results_dir}; printf "scenario,run_id,result,goals_reached,'
                       f'goal_count\\n{row}\\n" > {results_dir}/runs.csv; ')
        return ['bash', '-c', script + tail]
    return command


class Events:
    """publish() olaylarını toplar; belli bir aşamayı bekler."""

    def __init__(self):
        self.items = []
        self.cond = threading.Condition()

    def __call__(self, event, data):
        with self.cond:
            self.items.append((event, data))
            self.cond.notify_all()

    def phases(self):
        return [d['phase'] for e, d in self.items if e == 'run']

    def wait_phase(self, phase, timeout=10):
        with self.cond:
            return self.cond.wait_for(lambda: phase in self.phases(), timeout)


def manager(tmp_path, events, command, gazebo=False):
    return RunManager(str(tmp_path / 'panel'), events, command=command,
                      gazebo_check=lambda: gazebo, leftover_grace=0.3)


def status(run_id, state, **fields):
    """mission_manager'ın /mission_status iletisine benzeyen sözlük."""
    return {'v': 1, 'run_id': run_id, 'state': state, 'goal_index': None, 'goal_count': 2,
            'goal_results': [], 'recoveries': 0, **fields}


def test_kosu_asamalari_ve_sonuc(tmp_path):
    events = Events()
    done = tmp_path / 'bitir'
    runs = manager(tmp_path, events, fake_launch(
        ['Senaryo 2 (Zikzak), koşu p: 2 hedef', 'Hedef 1/2: (4.00, 0.00), yaw 90°'],
        csv_row='{scenario},{run_id},FAILED,1,2',
        tail=f'while [ ! -e {done} ]; do sleep 0.05; done; exit 0'))
    state = runs.start(2, gui=False)
    rid = state['run_id']
    assert state['phase'] == 'starting' and rid.startswith('p')
    assert 'scenario:=2' not in state['command']  # sahte komut; gerçek komut testi yukarıda

    runs.on_status(status(rid, 'waiting_nav2'))  # aşama aynı, hedef sayısı öğrenildi
    runs.on_status(status(rid, 'running', goal_index=1))
    runs.on_status(status(rid, 'running', goal_index=1))  # 1 Hz tekrarı: olay yok
    runs.on_status(status('baska', 'finished', result='SUCCEEDED'))  # başka koşu: yok sayılır
    runs.on_status(status(rid, 'recovering', goal_index=1, recoveries=1))
    assert runs.state()['recovering'] and runs.state()['phase'] == 'running'
    runs.on_status(status(rid, 'running', goal_results=['SUCCEEDED'], recoveries=1))
    assert runs.state()['goal'] == 1  # hedefler arasında son hedef korunur
    runs.on_status(status(rid, 'running', goal_index=2, goal_results=['SUCCEEDED'],
                          recoveries=1))
    runs.on_status(status(rid, 'finished', goal_results=['SUCCEEDED', 'FAILED'], recoveries=3,
                          result='FAILED'))
    runs.on_status(status(rid, 'finished', goal_results=['SUCCEEDED', 'FAILED'], recoveries=3,
                          result='FAILED'))  # finishing'te artık işlenmez
    assert events.phases() == ['starting', 'starting', 'running', 'running', 'running',
                               'running', 'finishing']
    done.touch()
    assert events.wait_phase('finished')
    final = runs.state()
    assert final['goal_results'] == ['SUCCEEDED', 'FAILED'] and final['goal_count'] == 2
    assert final['recoveries'] == 3 and not final['recovering']
    assert final['result'] == 'FAILED' and final['summary']['goals_reached'] == '1'
    assert [e for e, _ in events.items].count('run_finished') == 1
    assert any(d.get('text', '').startswith('Hedef 1/2') for e, d in events.items if e == 'log')
    log_file = os.path.join(runs.results_dir, 'logs', f's2_{final["run_id"]}.log')
    assert os.path.isfile(log_file)


def test_sonuc_satiri_yoksa_hata(tmp_path):
    events = Events()
    runs = manager(tmp_path, events, fake_launch(['Hedef 1/1: (1.00, 1.00), yaw 0°']))
    runs.start(1)
    assert events.wait_phase('error')
    assert 'runs.csv' in runs.state()['message']


def test_hata_koduyla_biten_launch(tmp_path):
    events = Events()
    runs = manager(tmp_path, events, fake_launch([], tail='exit 2'))
    runs.start(1)
    assert events.wait_phase('error')
    assert runs.state()['returncode'] == 2


def test_tek_kosu_kilidi_gazebo_denetimi_ve_durdurma(tmp_path):
    events = Events()
    # SIGINT yalnızca lidere gider. Sahte launch bash olduğundan sinyal 'exec sleep' sonrasına
    # denk gelmeli: bash ön plandaki bir komutu beklerken SIGINT'i erteleyip betiğe devam eder
    # (gerçek launch sinyali hemen işler).
    runs = manager(tmp_path, events, fake_launch(['Hedef 1/1: (1.00, 1.00), yaw 0°'],
                                                 tail='exec sleep 30'))
    with pytest.raises(RunError):
        runs.stop()  # çalışan koşu yok
    rid = runs.start(1)['run_id']
    with pytest.raises(RunError, match='Zaten'):
        runs.start(1)
    runs.on_status(status(rid, 'running', goal_index=1, goal_count=1))
    assert events.wait_phase('running')
    time.sleep(0.5)
    assert runs.stop()['phase'] == 'stopping'
    assert events.wait_phase('stopped')
    assert not runs.process.group_alive()
    with events.cond:  # durdurma iletisi 'stopped' olayından hemen sonra gelebilir
        assert events.cond.wait_for(lambda: any(
            d.get('text') == 'Koşu durduruldu (SIGINT)' for e, d in events.items if e == 'log'), 5)

    busy = manager(tmp_path, Events(), fake_launch([]), gazebo=True)
    with pytest.raises(RunError, match='gzserver'):
        busy.start(1)
    assert busy.state() == {'phase': 'idle'}


def test_kapanista_calisan_kosu_durdurulur(tmp_path):
    events = Events()
    runs = manager(tmp_path, events, fake_launch([], tail='exec sleep 30'))
    runs.start(3)
    time.sleep(0.3)
    runs.shutdown(timeout=10)
    assert not runs.process.running and not runs.process.group_alive()
    assert runs.state()['phase'] == 'stopped'
