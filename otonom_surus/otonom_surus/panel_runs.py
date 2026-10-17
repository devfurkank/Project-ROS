"""
Koşu dizini: results/ altındaki koşular, hedef sonuçları ve kayıt dosyaları (web paneli).

ROS'a bağımlı değildir. Kök klasör (results/) ve onun bir alt düzeyindeki klasörler
(zor_varyant/, parametre_fpd2/, panel/ ...) runs.csv içeriyorsa taranır; daha derine inilmez.
Panelin kendi sonuç klasörü kökün dışında olsa da taranır. Koşu kimliği köke göre klasör ile
s<N>_<run_id>'dir; kökteki koşularda klasör yazılmaz:
    s1_r1    parametre_fpd2/s2_r1    panel/s1_p20261017-153000

Her koşuya goals.csv'den hedef sonuçları ve hangi kayıt dosyalarının bulunduğu eklenir (yörünge,
harita, günlük; P6–P10'da meta, olay, engel ve ayrıntılı kayıt). Eksik dosya hata değildir,
"kayıt yok" demektir. Varyant etiketi klasördeki nav2_params.yaml'ın paketinkinden farkıdır
(ör. "forward_prune_distance 2.0"); fark yoksa klasörün adı.

Dizin önbelleklidir: her istekte yalnızca dosyaların değişim zamanlarına bakılır, yalnızca değişen
klasör yeniden okunur.
"""

import csv
import math
import os
import threading

from otonom_surus.scenarios import load_scenario, scenario_numbers
import yaml

# mission_manager.RUN_FIELDS ve GOAL_FIELDS'in kopyası (o modül rclpy'ye bağımlı); eşitliklerini
# test_panel_runs denetler. Yalnızca başlık satırı olmayan CSV'yi okurken gerekir.
RUN_FIELDS = [
    'timestamp', 'scenario', 'run_id', 'result', 'goals_reached', 'goal_count',
    'duration_s', 'wall_duration_s', 'path_length_m', 'straight_line_m', 'path_efficiency',
    'min_obstacle_dist_m', 'recoveries', 'failure_reason']
GOAL_FIELDS = [
    'timestamp', 'scenario', 'run_id', 'goal_index', 'goal_x', 'goal_y', 'goal_yaw_deg',
    'result', 'duration_s', 'path_length_m', 'recoveries', 'final_error_m']

# Koşu başına kayıt dosyaları: tür -> (klasör, uzantı); dosya adı s<N>_<run_id><uzantı>.
RECORDS = {
    'trajectory': ('trajectories', '.csv'),
    'map': ('maps', '.yaml'),
    'log': ('logs', '.log'),
    'meta': ('meta', '.json'),  # P6
    'events': ('events', '.jsonl'),  # P6
    'obstacles': ('obstacles', '.csv'),  # P8
    'replay': ('replay', '.json.gz'),  # P10
}
ROOT_LABEL = 'temel'  # kökteki koşuların varyant etiketi (paketin parametreleri)
MAX_VARIANT_KEYS = 3  # etikette en çok bu kadar değişen parametre adı yazılır
_MISSING = object()


def _number(text):
    """CSV hücresi -> float; boş, sayı olmayan ve sonlu olmayan değerde None (JSON'a hazır)."""
    try:
        value = float(text)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def _integer(text):
    value = _number(text)
    return None if value is None else int(value)


def read_rows(path, fields):
    """
    Bir CSV dosyasının satırları (sözlük listesi); dosya yoksa boş liste.

    İlk satırda run_id sütunu yoksa dosyanın başlığı yok sayılır ve sütunlar fields sırasıyla
    okunur. Kısa satırda eksik sütunlar sözlükte yer almaz.
    """
    try:
        with open(path, newline='', encoding='utf-8-sig') as f:
            rows = [row for row in csv.reader(f) if row]
    except FileNotFoundError:
        return []
    if not rows:
        return []
    header, body = (rows[0], rows[1:]) if 'run_id' in rows[0] else (fields, rows)
    return [dict(zip(header, row)) for row in body]


def read_trajectory(path, dt=None):
    """
    Yörünge dosyası (t, x, y) -> [[t, x, y], ...]; dt verilirse en çok dt saniyede bir nokta.

    Başlık satırı ve bozuk satırlar atlanır; seyreltmede ilk ve son nokta her zaman kalır.
    Dosya yoksa FileNotFoundError.
    """
    points = []
    with open(path, newline='', encoding='utf-8-sig') as f:
        for row in csv.reader(f):
            values = [_number(v) for v in row[:3]]
            if len(values) == 3 and None not in values:
                points.append(values)
    if dt and len(points) > 2:
        thinned = [points[0]]
        for point in points[1:-1]:
            if point[0] - thinned[-1][0] >= dt:
                thinned.append(point)
        thinned.append(points[-1])
        points = thinned
    return points


def _flatten(data, prefix=()):
    """İç içe sözlüğün yaprakları: {anahtar yolu (demet): değer}."""
    if not isinstance(data, dict):
        return {prefix: data}
    leaves = {}
    for key, value in data.items():
        leaves.update(_flatten(value, prefix + (str(key),)))
    return leaves


def _yaml_text(value):
    if isinstance(value, bool):
        return str(value).lower()
    return str(value)


def variant_label(params_path, base_path):
    """
    Klasörün Nav2 parametrelerinin paketinkinden farkı, ör. "forward_prune_distance 2.0".

    Değişen her yaprak için son anahtar ve klasördeki değer (dosya sırasıyla); paketinde olup
    klasörde olmayan anahtar "(yok)". MAX_VARIANT_KEYS'ten fazla fark varsa ilkleri ve kalanların
    sayısı. Fark yoksa ya da dosyalardan biri okunamazsa None.
    """
    try:
        with open(params_path, encoding='utf-8') as f:
            params = _flatten(yaml.safe_load(f))
        with open(base_path, encoding='utf-8') as f:
            base = _flatten(yaml.safe_load(f))
    except (OSError, yaml.YAMLError):
        return None
    changed = []
    for key in list(params) + [k for k in base if k not in params]:
        value = params.get(key, _MISSING)
        if value == base.get(key, _MISSING):
            continue
        name = key[-1] if key else '(kök)'
        changed.append(f'{name} (yok)' if value is _MISSING else f'{name} {_yaml_text(value)}')
    if not changed:
        return None
    if len(changed) > MAX_VARIANT_KEYS:
        rest = len(changed) - (MAX_VARIANT_KEYS - 1)
        changed = changed[:MAX_VARIANT_KEYS - 1] + [f'+{rest} parametre']
    return ', '.join(changed)


def folder_label(key):
    """Parametre farkı olmayan klasörün etiketi: kök 'temel', diğerleri adı ('_' boşluk olur)."""
    if not key:
        return ROOT_LABEL
    return key.rsplit('/', 1)[-1].replace('_', ' ')


def _stat(path):
    """Dosyanın ya da klasörün değişim imzası; yoksa None."""
    try:
        st = os.stat(path)
    except OSError:
        return None
    return st.st_mtime_ns, st.st_size


class RunIndex:
    """
    results/ altındaki koşuların dizini (iş parçacığı güvenli).

    root: tarama kökü (results/). panel_dir: panelin sonuç klasörü; kökün dışında olsa da taranır,
    koşuları 'panel': True ile işaretlenir. base_params: paketin nav2_params.yaml'ı (varyant
    etiketi için). scenario_files: senaryo dosyaları (paketinki ve P9'da panelinki); senaryonun
    adı ve dünyası ilk tanımlandığı dosyadan alınır.
    """

    def __init__(self, root, panel_dir=None, base_params=None, scenario_files=()):
        self.root = os.path.abspath(root)
        self.panel_dir = os.path.abspath(panel_dir) if panel_dir else None
        self.base_params = base_params
        self.scenario_files = list(scenario_files)
        self._lock = threading.Lock()
        self._folders = {}  # anahtar -> {'path', 'sig', 'records', 'warnings'}
        self._scenarios = {'sig': None, 'defs': {}, 'warnings': []}

    def folder_key(self, path):
        """Klasörün köke göre anahtarı ('' kök); kökün dışındaysa klasörün adı."""
        rel = os.path.relpath(os.path.abspath(path), self.root)
        if rel == '.':
            return ''
        if rel.startswith('..'):
            return os.path.basename(os.path.abspath(path))
        return rel.replace(os.sep, '/')

    def id_for(self, results_dir, scenario, run_id):
        """results_dir'e yazılmış koşunun dizindeki kimliği; o klasör taranmıyorsa None."""
        path = os.path.abspath(results_dir)
        if path not in (self.root, self.panel_dir) and os.path.dirname(path) != self.root:
            return None
        key = self.folder_key(path)
        name = f's{scenario}_{run_id}'
        return f'{key}/{name}' if key else name

    def runs(self, world=None):
        """Koşular, en yeni önce; world (dosya adı) verilirse yalnızca o dünyanın koşuları."""
        with self._lock:
            self._refresh()
            runs = [r for f in self._folders.values() for r in f['records'].values()]
        if world:
            runs = [r for r in runs if r['world'] == world]
        return sorted(runs, key=lambda r: (r['timestamp'] or '', r['id']), reverse=True)

    def get(self, run_id):
        """Tek koşunun kaydı; yoksa None."""
        with self._lock:
            self._refresh()
            for folder in self._folders.values():
                if run_id in folder['records']:
                    return folder['records'][run_id]
        return None

    def trajectory(self, run_id, dt=None):
        """Yörünge (read_trajectory); koşu yoksa KeyError, dosya yoksa FileNotFoundError."""
        with self._lock:
            self._refresh()
            for folder in self._folders.values():
                record = folder['records'].get(run_id)
                if record is not None:
                    path = os.path.join(folder['path'], 'trajectories',
                                        f's{record["scenario"]}_{record["run_id"]}.csv')
                    break
            else:
                raise KeyError(run_id)
        return read_trajectory(path, dt)

    def warnings(self):
        """Okuma sırasında atlanan dosya ve satırlar için kullanıcıya gösterilecek notlar."""
        with self._lock:
            self._refresh()
            notes = [] if os.path.isdir(self.root) else [f'{self.root} bulunamadı']
            notes += self._scenarios['warnings']
            for folder in self._folders.values():
                notes += folder['warnings']
        return notes

    def _dirs(self):
        """Taranacak klasörler {anahtar: yol}: runs.csv'si olan kök, alt klasörler ve panel_dir."""
        dirs = {'': self.root}
        try:
            names = sorted(e.name for e in os.scandir(self.root)
                           if e.is_dir() and not e.name.startswith('.'))
        except OSError:
            names = []
        for name in names:
            dirs.setdefault(name, os.path.join(self.root, name))
        if self.panel_dir:
            dirs.setdefault(self.folder_key(self.panel_dir), self.panel_dir)
        return {k: p for k, p in dirs.items() if os.path.isfile(os.path.join(p, 'runs.csv'))}

    def _refresh(self):
        """Değişen klasörleri yeniden okur (kilit tutulurken çağrılır)."""
        scenario_sig = tuple(_stat(p) for p in self.scenario_files)
        if scenario_sig != self._scenarios['sig']:
            self._scenarios = self._read_scenarios(scenario_sig)
        common = (scenario_sig, _stat(self.base_params) if self.base_params else None)
        folders = {}
        for key, path in self._dirs().items():
            names = ['runs.csv', 'goals.csv', 'nav2_params.yaml']
            names += sorted({sub for sub, _ in RECORDS.values()})
            sig = (path, common, tuple(_stat(os.path.join(path, n)) for n in names))
            old = self._folders.get(key)
            folders[key] = old if old and old['sig'] == sig else self._read_folder(key, path, sig)
        self._folders = folders

    def _read_scenarios(self, sig):
        defs, warnings = {}, []
        for path in self.scenario_files:
            try:
                numbers = scenario_numbers(path)
            except FileNotFoundError:
                continue  # panel senaryo dosyası P9'a kadar yok
            except (OSError, yaml.YAMLError) as e:
                warnings.append(f'{path}: {e}')
                continue
            for number in numbers:
                if number in defs:
                    continue
                try:
                    s = load_scenario(path, number)
                except (ValueError, yaml.YAMLError) as e:
                    warnings.append(f'{path}: {e}')
                    continue
                defs[number] = {'name': s['name'], 'world': os.path.basename(s['world'])}
        return {'sig': sig, 'defs': defs, 'warnings': warnings}

    def _read_folder(self, key, path, sig):
        """Klasörün runs.csv ve goals.csv'si -> {kimlik: kayıt}."""
        goals = {}
        for row in read_rows(os.path.join(path, 'goals.csv'), GOAL_FIELDS):
            goals.setdefault((_integer(row.get('scenario')), row.get('run_id')), []).append(row)
        present = {}
        for kind, (sub, _) in RECORDS.items():
            try:
                present[kind] = set(os.listdir(os.path.join(path, sub)))
            except OSError:
                present[kind] = set()
        params = os.path.join(path, 'nav2_params.yaml')
        variant = None
        if self.base_params and os.path.isfile(params):
            variant = variant_label(params, self.base_params)
        label = variant or folder_label(key)
        defs = self._scenarios['defs']

        records, skipped = {}, 0
        for row in read_rows(os.path.join(path, 'runs.csv'), RUN_FIELDS):
            scenario = _integer(row.get('scenario'))
            run_id = (row.get('run_id') or '').strip()
            if scenario is None or not run_id:
                skipped += 1
                continue
            name = f's{scenario}_{run_id}'
            rid = f'{key}/{name}' if key else name
            # Aynı koşu kimliği iki kez yazıldıysa hedefleri zaman damgasından ayırt edilir.
            candidates = goals.get((scenario, run_id), [])
            same = [g for g in candidates if g.get('timestamp') == row.get('timestamp')]
            scenario_def = defs.get(scenario, {})
            records[rid] = {  # aynı kimlik ikinci kez geçerse sonraki satır geçerli
                'id': rid, 'folder': key, 'label': label, 'variant': variant,
                'panel': path == self.panel_dir,
                'scenario': scenario, 'run_id': run_id,
                'scenario_name': scenario_def.get('name'), 'world': scenario_def.get('world'),
                'timestamp': row.get('timestamp') or None,
                'result': row.get('result') or None,
                'goals_reached': _integer(row.get('goals_reached')),
                'goal_count': _integer(row.get('goal_count')),
                'duration_s': _number(row.get('duration_s')),
                'wall_duration_s': _number(row.get('wall_duration_s')),
                'path_length_m': _number(row.get('path_length_m')),
                'straight_line_m': _number(row.get('straight_line_m')),
                'path_efficiency': _number(row.get('path_efficiency')),
                'min_obstacle_dist_m': _number(row.get('min_obstacle_dist_m')),
                'recoveries': _integer(row.get('recoveries')),
                'failure_reason': row.get('failure_reason') or '',
                'goals': sorted((self._goal(g) for g in same or candidates),
                                key=lambda g: g['index'] or 0),
                'files': {kind: name + ext in present[kind]
                          for kind, (_, ext) in RECORDS.items()},
            }
        warnings = []
        if skipped:
            runs_csv = os.path.join(path, 'runs.csv')
            warnings.append(f'{runs_csv}: {skipped} satır okunamadı, atlandı')
        return {'path': path, 'sig': sig, 'records': records, 'warnings': warnings}

    @staticmethod
    def _goal(row):
        return {
            'index': _integer(row.get('goal_index')),
            'x': _number(row.get('goal_x')), 'y': _number(row.get('goal_y')),
            'yaw': _number(row.get('goal_yaw_deg')),
            'result': row.get('result') or None,
            'duration_s': _number(row.get('duration_s')),
            'path_length_m': _number(row.get('path_length_m')),
            'recoveries': _integer(row.get('recoveries')),
            'final_error_m': _number(row.get('final_error_m')),
        }
