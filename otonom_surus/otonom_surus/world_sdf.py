"""
Gazebo .world (SDF) dosyalarından iki boyutlu geometri ve üretilen SDF'ler.

Üretilenler: panelin attığı kutu ve panelde tasarlanan dünya. scripts/analyze_results.py
(yörünge ve harita çizimleri) ile web paneli (dünya kartları, harita) aynı ayrıştırıcıyı
kullanır. Tasarlanan dünya (P9) paketteki dünyalarla aynı odadır; üretilen dosya yine bu
ayrıştırıcıyla okunur, düzenlemek için tasarım geometriden geri kurulur (design_of). ROS'a
bağımlı değildir.
"""

import math
import xml.etree.ElementTree as ET

# Tasarlanan dünyalar (P9): paketteki dünyalarla aynı 6 x 6 m oda ve ölçüler.
ROOM = (-1.0, -1.0, 5.0, 5.0)  # odanın iç yüzeyleri: x0, y0, x1, y1 (m)
WALL_T = 0.15  # m; oda ve iç duvarların kalınlığı
HEIGHT = 0.5  # m; duvar, kutu ve silindir yüksekliği
GOAL_DISC = 0.25  # m; hedef işaretinin yarıçapı (yalnızca görsel, çarpışması yok)
COLORS = {'wall': (0.55, 0.55, 0.6), 'box': (0.9, 0.45, 0.1), 'cylinder': (0.15, 0.4, 0.8),
          'goal': (0.1, 0.75, 0.2)}
# Tasarımdaki şekillerin model adları (shape_role duvarı adından tanır: duvar_*).
PREFIXES = {'walls': 'duvar_', 'boxes': 'kutu_', 'cylinders': 'silindir_'}


def _pose(element):
    """SDF öğesinin <pose>'u: (x, y, yaw); yoksa sıfır."""
    text = element.findtext('pose')
    values = [float(v) for v in text.split()] if text else [0.0] * 6
    return values[0], values[1], values[5]


def _compose(a, b):
    """Pozları birleştirir: a pozundaki çerçevede verilen b pozunu dış çerçeveye taşır."""
    ax, ay, at = a
    bx, by, bt = b
    return (ax + bx * math.cos(at) - by * math.sin(at),
            ay + bx * math.sin(at) + by * math.cos(at), at + bt)


def world_shapes(path):
    """
    Dünyadaki modellerin çarpışma şekilleri (yalnızca <collision>; hedef diski görseldir).

    Her şekil: model, static, kind ('box' | 'cylinder'), x, y, yaw ve size (kutuda (en, boy),
    silindirde yarıçap).
    """
    shapes = []
    for model in ET.parse(path).getroot().iter('model'):
        static = model.findtext('static', 'false').strip() == 'true'
        model_pose = _pose(model)
        for link in model.findall('link'):
            link_pose = _compose(model_pose, _pose(link))
            for collision in link.findall('collision'):
                x, y, yaw = _compose(link_pose, _pose(collision))
                geometry = collision.find('geometry')
                if geometry.find('box') is not None:
                    sx, sy, _ = (float(v) for v in geometry.findtext('box/size').split())
                    kind, size = 'box', (sx, sy)
                elif geometry.find('cylinder') is not None:
                    kind, size = 'cylinder', float(geometry.findtext('cylinder/radius'))
                else:
                    continue  # ground_plane
                shapes.append({'model': model.get('name'), 'static': static, 'kind': kind,
                               'x': x, 'y': y, 'yaw': yaw, 'size': size})
    return shapes


def shape_role(shape):
    """Panelde çizim rolü: 'wall' (oda, iç duvar), 'moving' (statik değil) ya da 'obstacle'."""
    if not shape['static']:
        return 'moving'
    name = shape['model'] or ''
    if name == 'oda' or name.startswith(('ic_duvar', 'duvar')):
        return 'wall'
    return 'obstacle'


def half_extent(shape):
    """Şeklin eksenlere hizalı yarı genişliği (dx, dy); dönük kutuda çevreleyen dikdörtgen."""
    if shape['kind'] == 'cylinder':
        return shape['size'], shape['size']
    c, s = abs(math.cos(shape['yaw'])), abs(math.sin(shape['yaw']))
    w, h = shape['size']
    return (c * w + s * h) / 2, (s * w + c * h) / 2


def shape_distance(x, y, shape):
    """Noktanın şekle (dönük kutu ya da silindir) uzaklığı; içindeyse 0."""
    if shape['kind'] == 'cylinder':
        return max(0.0, math.hypot(x - shape['x'], y - shape['y']) - shape['size'])
    c, s = math.cos(shape['yaw']), math.sin(shape['yaw'])
    dx, dy = x - shape['x'], y - shape['y']
    u, v = c * dx + s * dy, -s * dx + c * dy  # şeklin kendi ekseninde
    w, h = shape['size'][:2]
    return math.hypot(max(abs(u) - w / 2, 0.0), max(abs(v) - h / 2, 0.0))


def _geometry(shapes):
    """world_shapes biçimindeki şekiller -> JSON'a hazır geometri (world_geometry)."""
    result, xs, ys = [], [], []
    for shape in shapes:
        dx, dy = half_extent(shape)
        xs += [shape['x'] - dx, shape['x'] + dx]
        ys += [shape['y'] - dy, shape['y'] + dy]
        size = shape['size']
        result.append({
            'model': shape['model'], 'role': shape_role(shape), 'kind': shape['kind'],
            'x': round(shape['x'], 3), 'y': round(shape['y'], 3), 'yaw': round(shape['yaw'], 4),
            'size': [round(v, 3) for v in size] if shape['kind'] == 'box' else round(size, 3),
        })
    bounds = None
    if result:
        bounds = [round(min(xs), 3), round(min(ys), 3), round(max(xs), 3), round(max(ys), 3)]
    return {'shapes': result, 'bounds': bounds}


def world_geometry(path):
    """
    Panel için JSON'a hazır geometri: şekiller ve sınır kutusu.

    Şekil: model, role, kind, x, y, yaw, size (kutuda [en, boy], silindirde yarıçap); bounds:
    [x_min, y_min, x_max, y_max]. Boş dünyada bounds None.
    """
    return _geometry(world_shapes(path))


# --- Tasarlanan dünya (P9) ---------------------------------------------------------------------
# Tasarım: {'walls': [[x0, y0, x1, y1]], 'boxes': [[x, y, en, boy]], 'cylinders': [[x, y, r]],
# 'goals': [[x, y, yaw_derece]]}. Duvarlar eksenlere hizalıdır; uç noktalarından yarım kalınlık
# taşar (odanın köşeleri gibi): oda yüzeyinden çizilen duvar odaya değer, iki duvar köşede
# birleşir.

def wall_rect(wall):
    """İç duvar [x0, y0, x1, y1] -> kutusu (merkez x, merkez y, en, boy)."""
    x0, y0, x1, y1 = wall
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    if abs(x1 - x0) >= abs(y1 - y0):
        return cx, cy, abs(x1 - x0) + WALL_T, WALL_T
    return cx, cy, WALL_T, abs(y1 - y0) + WALL_T


def _box(model, x, y, w, h, static=True):
    return {'model': model, 'static': static, 'kind': 'box', 'x': x, 'y': y, 'yaw': 0.0,
            'size': (w, h)}


def room_shapes():
    """Odanın dört duvarı (world_shapes biçiminde; paketteki dünyalardaki 'oda' modeliyle aynı)."""
    x0, y0, x1, y1 = ROOM
    cx, cy, t = (x0 + x1) / 2, (y0 + y1) / 2, WALL_T
    return [_box('oda', cx, y0 - t / 2, x1 - x0 + 2 * t, t),
            _box('oda', cx, y1 + t / 2, x1 - x0 + 2 * t, t),
            _box('oda', x0 - t / 2, cy, t, y1 - y0),
            _box('oda', x1 + t / 2, cy, t, y1 - y0)]


def design_shapes(design):
    """Tasarımın şekilleri (odası hariç, world_shapes biçiminde); adları duvar_1, kutu_1 ..."""
    shapes = []
    for i, wall in enumerate(design.get('walls') or [], 1):
        shapes.append(_box(f'{PREFIXES["walls"]}{i}', *wall_rect(wall)))
    for i, (x, y, w, h) in enumerate(design.get('boxes') or [], 1):
        shapes.append(_box(f'{PREFIXES["boxes"]}{i}', x, y, w, h))
    for i, (x, y, r) in enumerate(design.get('cylinders') or [], 1):
        shapes.append({'model': f'{PREFIXES["cylinders"]}{i}', 'static': True, 'kind': 'cylinder',
                       'x': x, 'y': y, 'yaw': 0.0, 'size': r})
    return shapes


def design_geometry(design):
    """Tasarımın geometrisi (oda dahil): üretilen .world'ün world_geometry'si ile aynı."""
    return _geometry(room_shapes() + design_shapes(design))


def _vec(*values):
    # + 0.0: -0.0 '-0' yazılmasın
    return ' '.join(f'{round(v, 4) + 0.0:g}' for v in values)


def _text(parent, tag, text):
    element = ET.SubElement(parent, tag)
    element.text = text
    return element


def _part(link, tag, name, shape, z, pose=None, color=None, length=HEIGHT):
    """Bağlantıya collision ya da visual: kutu (size en, boy) ya da silindir (size yarıçap)."""
    part = ET.SubElement(link, tag, name=name)
    if pose is not None:
        _text(part, 'pose', _vec(*pose[:2], z, 0, 0, 0))
    geometry = ET.SubElement(part, 'geometry')
    if shape['kind'] == 'box':
        _text(ET.SubElement(geometry, 'box'), 'size', _vec(*shape['size'], length))
    else:
        cylinder = ET.SubElement(geometry, 'cylinder')
        _text(cylinder, 'radius', _vec(shape['size']))
        _text(cylinder, 'length', _vec(length))
    if color is not None:
        material = ET.SubElement(part, 'material')
        _text(material, 'ambient', _vec(*color, 1))
        _text(material, 'diffuse', _vec(*color, 1))
    return part


def _model(world, name, pose=None):
    model = ET.SubElement(world, 'model', name=name)
    _text(model, 'static', 'true')
    if pose is not None:
        _text(model, 'pose', _vec(*pose, 0, 0, 0))
    return model


def _world_header(world):
    """Fizik, sahne, kamera, güneş ve zemin: paketteki dünyalarla aynı."""
    physics = ET.SubElement(world, 'physics', type='ode')
    _text(physics, 'real_time_update_rate', '1000.0')
    _text(physics, 'max_step_size', '0.001')
    _text(physics, 'real_time_factor', '1')
    ode = ET.SubElement(physics, 'ode')
    solver = ET.SubElement(ode, 'solver')
    for tag, text in (('type', 'quick'), ('iters', '150'), ('precon_iters', '0'),
                      ('sor', '1.400000'), ('use_dynamic_moi_rescaling', '1')):
        _text(solver, tag, text)
    constraints = ET.SubElement(ode, 'constraints')
    for tag, text in (('cfm', '0.00001'), ('erp', '0.2'),
                      ('contact_max_correcting_vel', '2000.000000'),
                      ('contact_surface_layer', '0.01000')):
        _text(constraints, tag, text)
    scene = ET.SubElement(world, 'scene')
    _text(scene, 'ambient', '0.4 0.4 0.4 1')
    _text(scene, 'background', '0.7 0.7 0.7 1')
    _text(scene, 'shadows', 'false')
    camera = ET.SubElement(ET.SubElement(world, 'gui', fullscreen='0'), 'camera',
                           name='user_camera')
    _text(camera, 'pose', '2.0 -2.6 6.2 0 0.9 1.5708')
    _text(camera, 'view_controller', 'orbit')
    _text(camera, 'projection_type', 'perspective')
    sun = ET.SubElement(world, 'light', name='sun', type='directional')
    for tag, text in (('cast_shadows', 'false'), ('pose', '0 0 10 0 0 0'),
                      ('diffuse', '0.8 0.8 0.8 1'), ('specular', '0.2 0.2 0.2 1')):
        _text(sun, tag, text)
    attenuation = ET.SubElement(sun, 'attenuation')
    for tag, text in (('range', '1000'), ('constant', '0.9'), ('linear', '0.01'),
                      ('quadratic', '0.001')):
        _text(attenuation, tag, text)
    _text(sun, 'direction', '-0.5 0.1 -0.9')

    ground = ET.SubElement(world, 'model', name='ground_plane')
    _text(ground, 'static', 'true')
    link = ET.SubElement(ground, 'link', name='link')
    for tag in ('collision', 'visual'):
        part = ET.SubElement(link, tag, name=tag)
        if tag == 'visual':
            _text(part, 'cast_shadows', 'false')
        plane = ET.SubElement(ET.SubElement(part, 'geometry'), 'plane')
        _text(plane, 'normal', '0 0 1')
        _text(plane, 'size', '100 100')
        if tag == 'collision':
            friction = ET.SubElement(ET.SubElement(ET.SubElement(part, 'surface'), 'friction'),
                                     'ode')
            _text(friction, 'mu', '100')
            _text(friction, 'mu2', '50')
        else:
            material = ET.SubElement(part, 'material')
            _text(material, 'ambient', '0.85 0.85 0.85 1')
            _text(material, 'diffuse', '0.85 0.85 0.85 1')


def design_world(design, world_name):
    """
    Tasarımdan Gazebo dünyası (.world metni).

    Paketteki dünyalar gibi: aynı fizik ayarları, 'oda' modeli (dört duvar), bütün modeller statik
    ve yalnızca ilkel şekiller (T7: çevrimiçi model:// yok). İç duvarlar duvar_<i>, kutular
    kutu_<i>, silindirler silindir_<i>; hedef işaretleri hedef_<i> (zemine 4 mm'lik yeşil disk,
    çarpışması yok: LiDAR görmez, robot üstünden geçer).
    """
    sdf = ET.Element('sdf', version='1.6')
    world = ET.SubElement(sdf, 'world', name=world_name)
    counts = {k: len(design.get(k) or []) for k in ('walls', 'boxes', 'cylinders', 'goals')}
    world.append(ET.Comment(
        f' Panelin dünya tasarlayıcısıyla üretildi (P9): {counts["walls"]} duvar, '
        f'{counts["boxes"]} kutu, {counts["cylinders"]} silindir, {counts["goals"]} hedef. '
        f'Oda içi x, y ∈ [{ROOM[0]:g}, {ROOM[2]:g}]; robot (0, 0)\'da doğar. '))
    _world_header(world)

    room = _model(world, 'oda')
    link = ET.SubElement(room, 'link', name='duvarlar')
    for side, shape in zip(('guney', 'kuzey', 'bati', 'dogu'), room_shapes()):
        pose = (shape['x'], shape['y'])
        _part(link, 'collision', f'{side}_c', shape, HEIGHT / 2, pose)
        _part(link, 'visual', f'{side}_v', shape, HEIGHT / 2, pose, COLORS['wall'])

    colors = {'duvar_': COLORS['wall'], 'kutu_': COLORS['box'], 'silindir_': COLORS['cylinder']}
    for shape in design_shapes(design):
        model = _model(world, shape['model'], (shape['x'], shape['y'], HEIGHT / 2))
        link = ET.SubElement(model, 'link', name='link')
        color = next(c for prefix, c in colors.items() if shape['model'].startswith(prefix))
        _part(link, 'collision', 'collision', shape, 0)
        _part(link, 'visual', 'visual', shape, 0, color=color)

    disc = {'kind': 'cylinder', 'size': GOAL_DISC}
    for i, (x, y, _) in enumerate(design.get('goals') or [], 1):
        model = _model(world, f'hedef_{i}', (x, y, 0.002))
        link = ET.SubElement(model, 'link', name='link')
        _part(link, 'visual', 'visual', disc, 0, color=COLORS['goal'], length=0.004)

    ET.indent(sdf, '  ')
    return '<?xml version="1.0"?>\n' + ET.tostring(sdf, encoding='unicode') + '\n'


def _suffix(model):
    tail = model.rsplit('_', 1)[-1]
    return int(tail) if tail.isdigit() else 0


def design_of(geometry):
    """
    Geometriden (world_geometry) tasarımın şekilleri: düzenlemek için design_world'ün tersi.

    Yalnızca tasarlayıcının adlandırdığı modeller okunur (duvar_*, kutu_*, silindir_*; sıraları
    adlarındaki numarayla); oda ve diğerleri atlanır. Hedefler senaryodadır, burada yoktur.
    """
    design = {'walls': [], 'boxes': [], 'cylinders': []}
    for shape in sorted(geometry['shapes'], key=lambda s: _suffix(s['model'] or '')):
        model, x, y = shape['model'] or '', shape['x'], shape['y']
        if model.startswith(PREFIXES['walls']) and shape['kind'] == 'box':
            w, h = shape['size']
            if w >= h:
                half = (w - WALL_T) / 2
                wall = [x - half, y, x + half, y]
            else:
                half = (h - WALL_T) / 2
                wall = [x, y - half, x, y + half]
            design['walls'].append([round(v, 3) for v in wall])
        elif model.startswith(PREFIXES['boxes']) and shape['kind'] == 'box':
            design['boxes'].append([round(v, 3) for v in (x, y, *shape['size'])])
        elif model.startswith(PREFIXES['cylinders']) and shape['kind'] == 'cylinder':
            design['cylinders'].append([round(v, 3) for v in (x, y, shape['size'])])
    return design


def box_sdf(name, size, color=(0.85, 0.2, 0.55)):
    """
    Panelin Gazebo'ya attığı kutunun SDF'i (spawn_entity'nin xml alanı).

    Kutu statiktir (PT11): robot çarpınca yerinden oynamaz, başka modeli itmez. Konumu ve
    yüksekliği spawn_entity'nin initial_pose'u verir (merkezi zeminden size[2] / 2 yukarıda).
    Rengi dünyadaki engellerden ayrışsın diye eflatun.
    """
    sdf = ET.Element('sdf', version='1.6')
    model = ET.SubElement(sdf, 'model', name=name)
    ET.SubElement(model, 'static').text = 'true'
    link = ET.SubElement(model, 'link', name='link')
    box = ' '.join(f'{v:g}' for v in size)
    rgba = ' '.join(f'{v:g}' for v in (*color, 1))
    for tag in ('collision', 'visual'):
        part = ET.SubElement(link, tag, name=tag)
        ET.SubElement(ET.SubElement(ET.SubElement(part, 'geometry'), 'box'), 'size').text = box
        if tag == 'visual':
            material = ET.SubElement(part, 'material')
            ET.SubElement(material, 'ambient').text = rgba
            ET.SubElement(material, 'diffuse').text = rgba
    return ET.tostring(sdf, encoding='unicode')
