"""
Panelin alt süreçleri: launch'ı kendi süreç grubunda başlatma, çıktısını okuma, durdurma.

ROS'a bağımlı değildir; birim testleri sahte komutlarla çalışır. Yöntem run_experiments.sh'deki
gibi: launch kendi süreç grubunda (yeni oturumda) açılır; durdururken önce launch'a SIGINT,
kapanmazsa bütün gruba SIGTERM ve SIGKILL gider. Böylece yalnızca o koşudan kalan süreçler
ölür, kullanıcının başka oturumlarına dokunulmaz.

RunManager panelden başlatılan koşuyu yönetir: tek koşu kilidi, komut, aşama, günlük, sonuç.
Koşu senaryo görevi (scenario.launch.py) ya da serbest sürüştür (free.launch.py, mode
'interactive': hedefleri panel verir). Aşama süreçten ve mission_manager'ın /mission_status
iletilerinden (on_status) çıkarılır:
    idle → starting → running → finishing → finished
                    ↘ stopping → stopped          (Durdur)
                    ↘ error                       (launch hata koduyla bitti / sonuç yok)
Serbest sürüşte Durdur görevi bitirir: mission_manager'a "finish" gider (on_finish), sonuçlar
yazılır ve launch kendiliğinden kapanır (running → finishing → finished). FINISH_TIMEOUT içinde
kapanmazsa ya da Nav2 hazır olmadan durdurulursa süreç senaryo koşusundaki gibi durdurulur.
"""

import collections
import csv
import os
import re
import signal
import subprocess
import threading
import time

LOG_LINES = 200  # yeni açılan sekmeye gönderilen son günlük satırları
NOISE = ('screen size is bogus',)  # WSLg'nin zararsız uyarısı
LAUNCH_FILES = {'scenario': 'scenario.launch.py', 'interactive': 'free.launch.py'}
# Serbest sürüşte "bitir"den sonra kapanış için süre: hedefin iptali (≤ 5 s), sonuçların
# yazılması ve map_saver_cli (≤ 30 s), launch'ın düğümleri kapatması.
FINISH_TIMEOUT = 60.0

# Launch çıktısındaki iki satır biçimi:
#   [mission_manager-7] [INFO] [1792226568.000105631] [mission_manager]: Hedef 1/3: (4.00, ...
#   [ERROR] [panel_server-1]: process has died [pid 6614, exit code -9, ...]
NODE_LOG = re.compile(
    r'^\[(?P<proc>[^\]]+?)-\d+\] \[(?P<level>[A-Z]+)\] \[[\d.]+\] \[(?P<node>[^\]]+)\]: '
    r'(?P<msg>.*)$')
LAUNCH_LOG = re.compile(r'^\[(?P<level>[A-Z]+)\] \[(?P<who>[^\]]+)\]: (?P<msg>.*)$')
WARN_LEVELS = ('WARN', 'WARNING', 'ERROR', 'FATAL')

RESULT_FIELDS = ('result', 'goals_reached', 'goal_count', 'duration_s', 'path_length_m',
                 'path_efficiency', 'min_obstacle_dist_m', 'recoveries', 'failure_reason')


class RunError(Exception):
    """Koşu başlatılamadı ya da durdurulamadı; ileti kullanıcıya gösterilir."""


def parse_line(line):
    """
    Launch çıktısının bir satırı → {'level', 'source', 'text'}; olay akışına gitmeyecekse None.

    İletilen: mission_manager'ın her satırı, her düğümün uyarı ve hataları, launch'ın kendi
    uyarı ve hataları (ör. "process has died"). Geri kalan her şey yalnızca günlük dosyasına.
    """
    if any(noise in line for noise in NOISE):
        return None
    m = NODE_LOG.match(line)
    if m:
        if m['node'] == 'mission_manager' or m['level'] in WARN_LEVELS:
            return {'level': _level(m['level']), 'source': m['node'], 'text': m['msg']}
        return None
    m = LAUNCH_LOG.match(line)
    if m and m['level'] in WARN_LEVELS:
        return {'level': _level(m['level']), 'source': m['who'], 'text': m['msg']}
    return None


def _level(level):
    return 'WARN' if level == 'WARNING' else level


def gazebo_running():
    """Başka bir gzserver çalışıyor mu (T2: ikincisi robotu doğuramaz, launch takılır)."""
    return subprocess.run(['pgrep', '-x', 'gzserver'], stdout=subprocess.DEVNULL,
                          stderr=subprocess.DEVNULL).returncode == 0


def launch_env():
    """Alt süreç ortamı: çıktı satır satır gelsin, renk kodu olmasın."""
    env = dict(os.environ)
    env.update(PYTHONUNBUFFERED='1', RCUTILS_COLORIZED_OUTPUT='0')
    return env


def scenario_command(scenario, gui, rviz, results_dir, run_id, scenarios_file=None,
                     mode='scenario'):
    """Panelin çalıştırdığı komut; terminalde aynısı yazılabilir (serbest sürüşte free.launch)."""
    cmd = ['ros2', 'launch', 'otonom_surus', LAUNCH_FILES[mode], f'scenario:={scenario}',
           f'gui:={str(bool(gui)).lower()}', f'rviz:={str(bool(rviz)).lower()}',
           f'results_dir:={results_dir}', f'run_id:={run_id}', 'source:=panel']
    if scenarios_file:
        cmd.append(f'scenarios_file:={scenarios_file}')
    return cmd


def find_run(results_dir, scenario, run_id):
    """results_dir/runs.csv'de koşunun satırı (RESULT_FIELDS alanları); yoksa None."""
    path = os.path.join(results_dir, 'runs.csv')
    try:
        with open(path, newline='', encoding='utf-8') as f:
            for row in csv.DictReader(f):
                if row.get('scenario') == str(scenario) and row.get('run_id') == run_id:
                    return {k: row.get(k, '') for k in RESULT_FIELDS}
    except FileNotFoundError:
        pass
    return None


class ManagedProcess:
    """
    Kendi süreç grubunda çalışan bir alt süreç.

    Çıktı satır satır on_line'a (ve log_path'e), bitiş on_exit(returncode)'a gider. Süreç
    kendiliğinden bitince grupta kalanlar (T22: gzserver) leftover_grace sonra öldürülür.
    """

    def __init__(self, cmd, on_line=None, on_exit=None, log_path=None, env=None, cwd=None,
                 leftover_grace=5.0):
        self.cmd = cmd
        self.on_line = on_line
        self.on_exit = on_exit
        self.log_path = log_path
        self.env = env
        self.cwd = cwd
        self.leftover_grace = leftover_grace
        self.proc = None
        self.pgid = None
        self.returncode = None
        self._log = None
        self._log_lock = threading.Lock()
        self._reader = None
        self._done = threading.Event()

    def start(self):
        if self.log_path:
            os.makedirs(os.path.dirname(self.log_path), exist_ok=True)
            self._log = open(self.log_path, 'w', encoding='utf-8')
        self.proc = subprocess.Popen(
            self.cmd, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, env=self.env, cwd=self.cwd, start_new_session=True)
        self.pgid = self.proc.pid  # yeni oturumun lideri: grup kimliği = pid
        self._reader = threading.Thread(target=self._read, daemon=True)
        self._reader.start()
        threading.Thread(target=self._watch, daemon=True).start()
        return self

    @property
    def running(self):
        return self.proc is not None and not self._done.is_set()

    def wait(self, timeout=None):
        """Süreç ve grup temizliği bitene kadar bekler; bittiyse True."""
        return self._done.wait(timeout)

    def group_alive(self):
        if self.pgid is None:
            return False
        try:
            os.killpg(self.pgid, 0)
        except ProcessLookupError:
            return False
        except PermissionError:
            return True
        return True

    def stop(self, int_timeout=30.0, term_timeout=10.0, kill_timeout=5.0):
        """
        Önce yalnızca lider sürece (launch) SIGINT; grup boşalmazsa gruba SIGTERM, sonra SIGKILL.

        SIGINT bütün gruba gönderilmez: düğümler onu hem doğrudan hem launch'tan alıp
        "process has died, exit code -2" ile ölüyordu. Launch'a gönderilince düğümleri kendisi
        sırayla kapatır (run_experiments.sh'deki gibi). Dönen değer grubu boşaltan sinyalin adı;
        grup zaten boşsa None.
        """
        if not self.group_alive():
            return None
        for sig, timeout, group in ((signal.SIGINT, int_timeout, False),
                                    (signal.SIGTERM, term_timeout, True),
                                    (signal.SIGKILL, kill_timeout, True)):
            self._signal(sig, group)
            if self._wait_group(timeout):
                return sig.name
        return 'SIGKILL'

    def _signal(self, sig, group=True):
        try:
            if group:
                os.killpg(self.pgid, sig)
            else:
                os.kill(self.proc.pid, sig)
        except ProcessLookupError:
            pass

    def _wait_group(self, timeout):
        deadline = time.monotonic() + timeout
        while self.group_alive():
            if time.monotonic() >= deadline:
                return False
            time.sleep(0.1)
        return True

    def _read(self):
        for raw in self.proc.stdout:
            line = raw.decode('utf-8', 'replace').rstrip('\r\n')
            with self._log_lock:
                if self._log is not None:
                    self._log.write(line + '\n')
                    self._log.flush()
            if self.on_line is not None:
                self.on_line(line)

    def _watch(self):
        self.returncode = self.proc.wait()
        if not self._wait_group(self.leftover_grace):
            self._signal(signal.SIGKILL)
            self._wait_group(5.0)
        self._reader.join(timeout=5.0)
        with self._log_lock:
            if self._log is not None:
                self._log.close()
                self._log = None
        try:
            if self.on_exit is not None:
                self.on_exit(self.returncode)
        finally:
            self._done.set()  # wait() geri çağrı da bittikten sonra döner


class RunManager:
    """
    Panelden başlatılan koşu (aynı anda en çok bir tane): senaryo görevi ya da serbest sürüş.

    publish(event, data) olayları dağıtır: 'run' (aşama ya da hedef ilerlemesi değişti), 'log'
    (günlük satırı), 'run_finished' (koşu sonuçlarıyla bitti). on_finish() serbest sürüşü
    bitirir (panel mission_manager'a "finish" gönderir). command ve gazebo_check testlerde
    değiştirilir.
    """

    def __init__(self, results_dir, publish, scenarios_file=None, command=scenario_command,
                 gazebo_check=gazebo_running, cwd=None, leftover_grace=5.0, on_finish=None,
                 finish_timeout=FINISH_TIMEOUT):
        self.results_dir = results_dir
        self.publish = publish
        self.scenarios_file = scenarios_file
        self.command = command
        self.gazebo_check = gazebo_check
        self.cwd = cwd
        self.leftover_grace = leftover_grace
        self.on_finish = on_finish
        self.finish_timeout = finish_timeout
        self.process = None
        self.log = collections.deque(maxlen=LOG_LINES)
        self._lock = threading.Lock()
        self._stopping = False
        self._state = {'phase': 'idle'}

    def state(self):
        with self._lock:
            return dict(self._state)

    def recent_log(self):
        with self._lock:
            return list(self.log)

    def start(self, scenario, gui=True, rviz=False, mode='scenario'):
        if mode not in LAUNCH_FILES:
            raise RunError(f'Bilinmeyen kip: {mode}')
        with self._lock:
            if self.process is not None and self.process.running:
                raise RunError('Zaten çalışan bir koşu var; önce durdur.')
            if self.gazebo_check():
                raise RunError('Başka bir Gazebo (gzserver) açık; önce onu kapat.')
            run_id = 'p' + time.strftime('%Y%m%d-%H%M%S')
            cmd = self.command(scenario=scenario, gui=gui, rviz=rviz,
                               results_dir=self.results_dir, run_id=run_id,
                               scenarios_file=self.scenarios_file, mode=mode)
            self._state = {
                'phase': 'starting', 'mode': mode, 'scenario': scenario, 'run_id': run_id,
                'gui': bool(gui), 'rviz': bool(rviz), 'command': ' '.join(cmd),
                'started': time.time(), 'goal': None, 'goal_count': None, 'goal_results': [],
                'recoveries': 0, 'recovering': False, 'idle': False,
            }
            self._stopping = False
            self.log.clear()
            self.process = ManagedProcess(
                cmd, on_line=self._on_line, on_exit=self._on_exit, env=launch_env(),
                log_path=os.path.join(self.results_dir, 'logs', f's{scenario}_{run_id}.log'),
                cwd=self.cwd, leftover_grace=self.leftover_grace)
            try:
                self.process.start()
            except OSError as e:
                self._state = {'phase': 'idle'}
                raise RunError(f'Launch başlatılamadı: {e}') from None
            state = dict(self._state)
        self.publish('run', state)
        return state

    def stop(self):
        """
        Durdurmayı başlatır ve hemen döner; bitiş 'run' olayıyla bildirilir.

        Serbest sürüşte görev hedef alabilir durumdaysa (running) önce bitirilir: sonuçlar
        yazılır, bitiş 'finished' olur. Bitirilirken (finishing) ikinci Durdur süreci durdurur.
        Senaryo koşusu ve henüz hazır olmayan serbest sürüş durdurulur (stopped, sonuç yazılmaz).
        """
        with self._lock:
            process = self.process
            if process is None or not process.running:
                raise RunError('Çalışan koşu yok.')
            if self._stopping:
                return dict(self._state)
            finish = (self._state.get('mode') == 'interactive'
                      and self._state['phase'] == 'running' and self.on_finish is not None)
            if finish:
                self._state['phase'] = 'finishing'
            else:
                self._stopping = True
                self._state['phase'] = 'stopping'
            state = dict(self._state)
        self.publish('run', state)
        worker = self._finish_worker if finish else self._stop_worker
        threading.Thread(target=worker, args=(process,), daemon=True).start()
        return state

    def _finish_worker(self, process):
        self.on_finish()
        self._add_log({'level': 'INFO', 'source': 'panel',
                       'text': 'Serbest sürüş bitiriliyor; sonuçlar yazılıyor'})
        if process.wait(self.finish_timeout):
            return
        with self._lock:
            if self._stopping or self.process is not process:
                return
            self._stopping = True
            self._state['phase'] = 'stopping'
            state = dict(self._state)
        self.publish('run', state)
        self._add_log({'level': 'WARN', 'source': 'panel', 'text':
                       f'Görev {self.finish_timeout:.0f} s içinde kapanmadı; durduruluyor'})
        self._stop_worker(process)

    def shutdown(self, timeout=60.0):
        """Panel kapanırken: çalışan koşuyu durdurur ve bitmesini bekler."""
        process = self.process
        if process is not None and process.running:
            with self._lock:
                self._stopping = True
            process.stop()
            process.wait(timeout)

    def _stop_worker(self, process):
        sig = process.stop()
        if sig:
            self._add_log({'level': 'INFO', 'source': 'panel', 'text': f'Koşu durduruldu ({sig})'})

    def _add_log(self, entry):
        with self._lock:
            self.log.append(entry)
        self.publish('log', entry)

    def _on_line(self, line):
        entry = parse_line(line)
        if entry is not None:
            self._add_log(entry)

    def on_status(self, status):
        """
        mission_manager'ın /mission_status iletisi (sözlük): aşama, hedef ve kurtarma ilerlemesi.

        Yalnızca bu koşunun (run_id) iletisi işlenir. Durum değiştiyse 'run' olayı gider; 1 Hz'lik
        tekrarlar (yalnızca süre ve yol değişir) olay üretmez. Hedefler arasındaki kısa anda
        (goal_index None) son hedef korunur. Bitirilirken (finishing) hedef sonuçları işlenir
        ama aşama geri dönmez.
        """
        with self._lock:
            st = self._state
            if (st['phase'] not in ('starting', 'running', 'finishing')
                    or st.get('run_id') != status.get('run_id')):
                return
            before = dict(st, goal_results=list(st['goal_results']))
            mission = status.get('state')
            if mission in ('running', 'recovering', 'idle'):
                if st['phase'] != 'finishing':
                    st['phase'] = 'running'
            elif mission == 'finished':
                st['phase'], st['result'] = 'finishing', status.get('result')
            if status.get('goal_index') is not None:
                st['goal'] = status['goal_index']
            st['goal_count'] = status.get('goal_count')
            st['goal_results'] = list(status.get('goal_results') or [])
            st['recoveries'] = status.get('recoveries', 0)
            st['recovering'] = mission == 'recovering'
            st['idle'] = mission == 'idle'
            if st == before:
                return
            state = dict(st)
        self.publish('run', state)

    def _on_exit(self, returncode):
        with self._lock:
            st = self._state
            st['returncode'] = returncode
            st['ended'] = time.time()
            row = None
            if self._stopping:
                st['phase'], st['message'] = 'stopped', 'Koşu durduruldu.'
            elif returncode == 0:
                row = find_run(self.results_dir, st['scenario'], st['run_id'])
                if row is not None:
                    st['phase'], st['summary'] = 'finished', row
                    st['result'] = row['result']
                else:
                    st['phase'] = 'error'
                    st['message'] = 'Launch bitti ama runs.csv’de bu koşunun satırı yok.'
            else:
                st['phase'], st['message'] = 'error', f'Launch {returncode} koduyla bitti.'
            self._stopping = False
            state = dict(st)
        self.publish('run', state)
        if row is not None:
            self.publish('run_finished', state)
