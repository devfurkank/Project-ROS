"""
Panelde tasarlanan dünyalar (P9): tasarımın denetimi, .world üretimi ve panelin senaryo dosyası.

ROS'a bağımlı değildir. Tasarım (world_sdf'teki biçim) paketteki dünyalarla aynı odaya yerleşir.
Kaydedilen her dünya bir senaryodur: dünya dosyası <results_dir>/worlds/ozel_<N>.world, senaryo
tanımı <results_dir>/panel_scenarios.yaml (biçimi config/scenarios.yaml'ınkiyle aynı; dünya mutlak
yolla, sim.launch.py onu olduğu gibi kullanır, PT13). Panel koşuyu bu dosyayla başlatır
(scenarios_file:=...). Numaralar FIRST_ID'den başlar, paketin senaryolarıyla (1-5) çakışmaz ve
silinen dünyanın numarası, koşusu varsa yeniden verilmez (eski koşuların adı karışmasın).

Denetimler sunucuda (kayıt reddedilir) ve aynılarıyla istemcide (yerleştirirken) yapılır: şekiller
odanın içinde, robotun doğduğu (0, 0)'ın START_CLEAR çevresi boş, 1-MAX_GOALS hedef, hedef bir
şeklin GOAL_CLEAR yakınında değil ve bir öncekine (ilk hedefte başlangıca) GOAL_STEP'ten yakın
değil. Yol ön kontrolü (A*) yalnızca istemcide: ulaşılamaz hedef kaydı engellemez, uyarıdır.
"""

import math
import os
import re
import threading
import xml.etree.ElementTree as ET

from otonom_surus.panel_runs import read_rows, RUN_FIELDS
from otonom_surus.scenarios import load_scenario, scenario_numbers
from otonom_surus.world_sdf import (
    design_geometry, design_of, design_world, ROOM, world_geometry)
import yaml

FIRST_ID = 100
SCENARIOS_NAME = 'panel_scenarios.yaml'
WORLDS_DIR = 'worlds'
WORLD_FILE = re.compile(r'^ozel_(\d+)\.world$')
MAX_GOALS = 5
MAX_SHAPES = 40  # duvar + kutu + silindir
MAX_NAME = 40
START_CLEAR = 0.5  # m; robotun doğduğu (0, 0) noktasının çevresi boş kalır
GOAL_CLEAR = 0.2  # m; hedeften en yakın şekle (robot yarıçapı 0.125 + pay)
GOAL_STEP = 0.5  # m; hedeften bir öncekine (ilk hedefte başlangıca)
MIN_WALL = 0.3  # m; en kısa iç duvar
BOX_SIZES = (0.1, 3.0)  # m; kutunun eni ve boyu
CYLINDER_RADII = (0.05, 1.0)  # m
EPS = 1e-6
HEADER = ('# Panelin "Dünya tasarla" moduyla kaydedilen senaryolar (P9). Dosyayı panel yazar;\n'
          '# biçimi config/scenarios.yaml ile aynı. Dünyalar worlds/ozel_<N>.world (mutlak yol).\n'
          '\n')
TO_ROLE = {'wall': 'bir duvara', 'obstacle': 'bir engele'}  # yönelme hali


class DesignError(ValueError):
    """Tasarım kaydedilemez; ileti kullanıcıya gösterilir (HTTP 400)."""


def _numbers(item, count, what):
    """Tasarımın bir öğesi: count sayılık liste -> yuvarlanmış float'lar; değilse DesignError."""
    if (not isinstance(item, (list, tuple)) or len(item) != count
            or any(isinstance(v, bool) or not isinstance(v, (int, float)) for v in item)):
        raise DesignError(f'{what} {count} sayıdan oluşmalı: {item!r}')
    if not all(math.isfinite(v) for v in item):
        raise DesignError(f'{what} sonlu sayılardan oluşmalı')
    return [round(float(v), 3) for v in item]


def _inside(x0, y0, x1, y1, margin=0.0):
    rx0, ry0, rx1, ry1 = ROOM
    return (rx0 + margin - EPS <= x0 and x1 <= rx1 - margin + EPS
            and ry0 + margin - EPS <= y0 and y1 <= ry1 - margin + EPS)


def _name(value):
    if value is None:
        return ''
    if not isinstance(value, str):
        raise DesignError('Ad metin olmalı.')
    name = ' '.join(value.split())
    if len(name) > MAX_NAME:
        raise DesignError(f'Ad en çok {MAX_NAME} karakter olabilir.')
    if any(ord(c) < 32 for c in name):
        raise DesignError('Adda denetim karakteri olamaz.')
    return name


def _items(body, key):
    items = body.get(key)
    if items is None:
        return []
    if not isinstance(items, list):
        raise DesignError(f'{key} bir liste olmalı.')
    return items


def clean_design(body):
    """
    İstek gövdesi -> denetlenmiş tasarım {name, walls, boxes, cylinders, goals}.

    Sayılar 3 basamağa yuvarlanır, hedef yönü (-180, 180] aralığına getirilir. Kurallara uymayan
    tasarımda Türkçe iletili DesignError.
    """
    design = {'name': _name(body.get('name'))}
    walls = []
    for i, item in enumerate(_items(body, 'walls'), 1):
        x0, y0, x1, y1 = _numbers(item, 4, f'Duvar {i}')
        if abs(x1 - x0) > EPS and abs(y1 - y0) > EPS:
            raise DesignError(f'Duvar {i} eksenlere hizalı olmalı (yatay ya da dikey).')
        if math.hypot(x1 - x0, y1 - y0) < MIN_WALL - EPS:
            raise DesignError(f'Duvar {i} en az {MIN_WALL:g} m uzunlukta olmalı.')
        if not _inside(min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1)):
            raise DesignError(f'Duvar {i} odanın dışına taşıyor.')
        walls.append([x0, y0, x1, y1])
    boxes = []
    for i, item in enumerate(_items(body, 'boxes'), 1):
        x, y, w, h = _numbers(item, 4, f'Kutu {i}')
        if not (BOX_SIZES[0] <= w <= BOX_SIZES[1] and BOX_SIZES[0] <= h <= BOX_SIZES[1]):
            raise DesignError(f'Kutu {i}: en ve boy {BOX_SIZES[0]:g}-{BOX_SIZES[1]:g} m olmalı.')
        if not _inside(x - w / 2, y - h / 2, x + w / 2, y + h / 2):
            raise DesignError(f'Kutu {i} odanın dışına taşıyor.')
        boxes.append([x, y, w, h])
    cylinders = []
    for i, item in enumerate(_items(body, 'cylinders'), 1):
        x, y, r = _numbers(item, 3, f'Silindir {i}')
        if not CYLINDER_RADII[0] <= r <= CYLINDER_RADII[1]:
            raise DesignError(
                f'Silindir {i}: yarıçap {CYLINDER_RADII[0]:g}-{CYLINDER_RADII[1]:g} m olmalı.')
        if not _inside(x - r, y - r, x + r, y + r):
            raise DesignError(f'Silindir {i} odanın dışına taşıyor.')
        cylinders.append([x, y, r])
    if len(walls) + len(boxes) + len(cylinders) > MAX_SHAPES:
        raise DesignError(f'En çok {MAX_SHAPES} şekil (duvar, kutu, silindir) konabilir.')
    goals = []
    for i, item in enumerate(_items(body, 'goals'), 1):
        x, y, yaw = _numbers(item, 3, f'Hedef {i}')
        yaw = round(math.degrees(math.atan2(math.sin(math.radians(yaw)),
                                            math.cos(math.radians(yaw)))), 1)
        goals.append([x, y, 180.0 if yaw == -180.0 else yaw])
    if not goals:
        raise DesignError('En az bir hedef koy.')
    if len(goals) > MAX_GOALS:
        raise DesignError(f'En çok {MAX_GOALS} hedef konabilir.')
    design.update(walls=walls, boxes=boxes, cylinders=cylinders, goals=goals)
    _check_layout(design)
    return design


def _distance(x, y, shape):
    """Noktanın eksenlere hizalı kutuya ya da silindire uzaklığı (tasarımda dönük şekil yok)."""
    if shape['kind'] == 'cylinder':
        return max(0.0, math.hypot(x - shape['x'], y - shape['y']) - shape['size'])
    w, h = shape['size']
    return math.hypot(max(abs(x - shape['x']) - w / 2, 0.0), max(abs(y - shape['y']) - h / 2, 0.0))


def _label(shape):
    kind = shape['model'].split('_')[0]
    number = shape['model'].rsplit('_', 1)[-1]
    names = {'duvar': 'Duvar', 'kutu': 'Kutu', 'silindir': 'Silindir'}
    return f'{names.get(kind, "Şekil")} {number}'


def _check_layout(design):
    """Başlangıcın boşluğu ve hedeflerin yerleşimi (şekiller odanın içinde denetlendi)."""
    shapes = design_geometry(design)['shapes']
    for shape in shapes:
        if shape['model'] != 'oda' and _distance(0.0, 0.0, shape) < START_CLEAR - EPS:
            raise DesignError(f'Robotun doğduğu (0, 0) noktasının {START_CLEAR:g} m çevresi boş '
                              f'kalmalı: {_label(shape)} çok yakın.')
    previous = (0.0, 0.0)
    for i, (x, y, _) in enumerate(design['goals'], 1):
        if not _inside(x, y, x, y, GOAL_CLEAR):
            raise DesignError(f'Hedef {i} odanın dışında ya da duvarına çok yakın.')
        for shape in shapes:
            if _distance(x, y, shape) < GOAL_CLEAR - EPS:
                what = TO_ROLE.get(shape['role'], TO_ROLE['obstacle'])
                raise DesignError(f'Hedef {i} {what} çok yakın; robot oraya sığmaz.')
        if math.hypot(x - previous[0], y - previous[1]) < GOAL_STEP - EPS:
            before = 'robotun başlangıcına' if i == 1 else 'bir öncekine'
            raise DesignError(f'Hedef {i} {before} çok yakın (en az {GOAL_STEP:g} m).')
        previous = (x, y)


def _write_atomic(path, text):
    tmp = f'{path}.tmp'
    with open(tmp, 'w', encoding='utf-8') as f:
        f.write(text)
    os.replace(tmp, path)


class WorldStore:
    """
    Tasarlanan dünyalar ve onların senaryo dosyası (iş parçacığı güvenli).

    root: panelin sonuç klasörü (results/panel). reserved: paketin senaryo numaraları (bunlar
    verilmez).
    """

    def __init__(self, root, reserved=()):
        self.root = os.path.abspath(root)
        self.worlds_dir = os.path.join(self.root, WORLDS_DIR)
        self.scenarios_file = os.path.join(self.root, SCENARIOS_NAME)
        self.reserved = set(reserved)
        self._lock = threading.Lock()

    def _read(self):
        """Senaryo dosyası -> {numara: tanım}; dosya yoksa boş. Bozuksa yaml.YAMLError/OSError."""
        try:
            with open(self.scenarios_file, encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
        except FileNotFoundError:
            return {}
        if not isinstance(data, dict):
            raise OSError(f'{self.scenarios_file}: senaryo sözlüğü değil')
        return {k: v for k, v in data.items() if isinstance(k, int) and isinstance(v, dict)}

    def _write(self, defs):
        parts = [yaml.safe_dump({number: defs[number]}, allow_unicode=True, sort_keys=False,
                                default_flow_style=None, width=99) for number in sorted(defs)]
        os.makedirs(self.root, exist_ok=True)
        _write_atomic(self.scenarios_file, HEADER + '\n'.join(parts))

    def numbers(self):
        """Tasarlanmış dünyaların senaryo numaraları, sıralı."""
        with self._lock:
            try:
                return scenario_numbers(self.scenarios_file)
            except FileNotFoundError:
                return []

    def owns(self, number):
        return number in self.numbers()

    def _used(self, defs):
        """Verilmiş numaralar: dosyadakiler, paketinkiler, dünya dosyaları ve koşusu olanlar."""
        used = set(defs) | self.reserved
        try:
            used |= {int(m[1]) for m in map(WORLD_FILE.match, os.listdir(self.worlds_dir)) if m}
        except OSError:
            pass
        for row in read_rows(os.path.join(self.root, 'runs.csv'), RUN_FIELDS):
            try:
                used.add(int(row.get('scenario')))
            except (TypeError, ValueError):
                pass
        return used

    def entries(self):
        """
        Kayıtlı dünyalar, numara sırasıyla: id, name, file, world_file, goals; geometry ve design.

        Dünya dosyası okunamazsa geometry ve design yerine error. design: tasarlayıcının
        düzenleyebileceği biçim (ad ve hedefler dahil).
        """
        with self._lock:
            defs = self._read()
        entries = []
        for number in sorted(defs):
            try:
                s = load_scenario(self.scenarios_file, number)
            except (ValueError, OSError, yaml.YAMLError) as e:
                entries.append({'id': number, 'name': str(number), 'file': None, 'error': str(e)})
                continue
            goals = [[x, y, round(math.degrees(yaw), 1)] for x, y, yaw in s['goals']]
            entry = {'id': number, 'name': s['name'], 'file': os.path.basename(s['world']),
                     'world_file': s['world'], 'goals': goals}
            try:
                geometry = world_geometry(s['world'])
            except (OSError, ET.ParseError, ValueError, AttributeError, TypeError) as e:
                entry['error'] = f'{entry["file"]} okunamadı: {e}'
            else:
                entry['geometry'] = geometry
                entry['design'] = dict(design_of(geometry), name=s['name'], goals=goals)
            entries.append(entry)
        return entries

    def save(self, body):
        """
        Tasarımı kaydeder: body'de id varsa o dünyanın üstüne, yoksa yeni numarayla.

        Dönen: {id, name, file, world_file}. Tasarım kurallara uymuyorsa DesignError, id kayıtlı
        değilse KeyError. Önce dünya dosyası yazılır, sonra senaryo: senaryo hiçbir zaman olmayan
        bir dosyayı göstermez.
        """
        design = clean_design(body)
        number = body.get('id')
        if number is not None and (isinstance(number, bool) or not isinstance(number, int)):
            raise DesignError('id tamsayı olmalı.')
        with self._lock:
            defs = self._read()
            if number is None:
                number = max([FIRST_ID - 1] + [n for n in self._used(defs) if n >= FIRST_ID]) + 1
            elif number not in defs:
                raise KeyError(number)
            name = design['name'] or f'Tasarlanan dünya {number}'
            file = f'ozel_{number}.world'
            path = os.path.join(self.worlds_dir, file)
            os.makedirs(self.worlds_dir, exist_ok=True)
            _write_atomic(path, design_world(design, f'ozel_{number}'))
            defs[number] = {'name': name, 'world': path, 'goals': design['goals']}
            self._write(defs)
        return {'id': number, 'name': name, 'file': file, 'world_file': path}

    def delete(self, number):
        """
        Dünyayı siler: önce senaryosu, sonra dosyası (results/panel/worlds altındaysa).

        Koşuları kalır; tekrar oynatma geometriyi koşunun meta kaydından çizer. Kayıtlı değilse
        KeyError. Dönen: silinen tanım.
        """
        with self._lock:
            defs = self._read()
            if number not in defs:
                raise KeyError(number)
            removed = defs.pop(number)
            self._write(defs)
        path = str(removed.get('world') or '')
        if (os.path.dirname(os.path.abspath(path)) == self.worlds_dir
                and WORLD_FILE.match(os.path.basename(path))):
            try:
                os.remove(path)
            except FileNotFoundError:
                pass
        return removed
