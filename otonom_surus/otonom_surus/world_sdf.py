"""
Gazebo .world (SDF) dosyalarından iki boyutlu geometri ve panelin attığı kutunun SDF'i.

scripts/analyze_results.py (yörünge ve harita çizimleri) ile web paneli (dünya kartları, harita)
aynı ayrıştırıcıyı kullanır. ROS'a bağımlı değildir.
"""

import math
import xml.etree.ElementTree as ET


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


def world_geometry(path):
    """
    Panel için JSON'a hazır geometri: şekiller ve sınır kutusu.

    Şekil: model, role, kind, x, y, yaw, size (kutuda [en, boy], silindirde yarıçap); bounds:
    [x_min, y_min, x_max, y_max]. Boş dünyada bounds None.
    """
    shapes, xs, ys = [], [], []
    for shape in world_shapes(path):
        dx, dy = half_extent(shape)
        xs += [shape['x'] - dx, shape['x'] + dx]
        ys += [shape['y'] - dy, shape['y'] + dy]
        size = shape['size']
        shapes.append({
            'model': shape['model'], 'role': shape_role(shape), 'kind': shape['kind'],
            'x': round(shape['x'], 3), 'y': round(shape['y'], 3), 'yaw': round(shape['yaw'], 4),
            'size': [round(v, 3) for v in size] if shape['kind'] == 'box' else round(size, 3),
        })
    bounds = None
    if shapes:
        bounds = [round(min(xs), 3), round(min(ys), 3), round(max(xs), 3), round(max(ys), 3)]
    return {'shapes': shapes, 'bounds': bounds}


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
