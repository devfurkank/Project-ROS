"""
Senaryo tanımlarını (config/scenarios.yaml) okur.

sim.launch.py dünya dosyasını, mission_manager hedef listesini, obstacle_mover hareketli
engelleri buradan alır; böylece bir senaryonun tanımı tek yerde durur. ROS'a bağımlı değildir.
"""

import math

import yaml


def load_scenario(path, number):
    """
    scenarios.yaml'dan bir senaryoyu okur.

    Dönen sözlük: name, world, goals ve moving_obstacles. goals, [(x, y, yaw_rad), ...]
    listesidir (dosyada yaw derece). moving_obstacles, [{'name', 'travel', 'speed'}, ...]
    listesidir; tanımlı değilse boş. Senaryo yoksa ya da biçimi bozuksa ValueError verir.
    """
    with open(path, encoding='utf-8') as f:
        scenarios = yaml.safe_load(f) or {}
    try:
        number = int(number)
    except ValueError:
        raise ValueError(f'senaryo numarası tamsayı olmalı: {number!r}') from None
    if number not in scenarios:
        raise ValueError(
            f'{path} içinde senaryo {number} yok (tanımlı olanlar: {sorted(scenarios)})')

    scenario = scenarios[number]
    if not scenario.get('world'):
        raise ValueError(f'senaryo {number}: world tanımlı değil')
    goals = []
    for goal in scenario.get('goals') or []:
        if not isinstance(goal, list) or len(goal) != 3:
            raise ValueError(f'senaryo {number}: hedef [x, y, yaw_derece] olmalı: {goal!r}')
        x, y, yaw_deg = (float(v) for v in goal)
        goals.append((x, y, math.radians(yaw_deg)))
    if not goals:
        raise ValueError(f'senaryo {number}: goals boş')

    obstacles = []
    for obstacle in scenario.get('moving_obstacles') or []:
        try:
            name = obstacle['name']
            travel, speed = float(obstacle['travel']), float(obstacle['speed'])
        except (TypeError, KeyError, ValueError):
            name, travel, speed = None, 0.0, 0.0
        if not name or not isinstance(name, str) or travel <= 0.0 or speed <= 0.0:
            raise ValueError(f'senaryo {number}: hareketli engel {{name, travel > 0, '
                             f'speed > 0}} olmalı: {obstacle!r}')
        obstacles.append({'name': name, 'travel': travel, 'speed': speed})

    return {
        'name': scenario.get('name', f'Senaryo {number}'),
        'world': scenario['world'],
        'goals': goals,
        'moving_obstacles': obstacles,
    }
