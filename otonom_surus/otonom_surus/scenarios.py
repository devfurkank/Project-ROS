"""
Senaryo tanımlarını (config/scenarios.yaml) okur.

sim.launch.py dünya dosyasını, mission_manager hedef listesini buradan alır; böylece bir
senaryonun tanımı tek yerde durur. ROS'a bağımlı değildir.
"""

import math

import yaml


def load_scenario(path, number):
    """
    scenarios.yaml'dan bir senaryoyu okur.

    Dönen sözlük: name, world ve goals. goals, [(x, y, yaw_rad), ...] listesidir (dosyada yaw
    derece). Senaryo yoksa ya da biçimi bozuksa ValueError verir.
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

    return {
        'name': scenario.get('name', f'Senaryo {number}'),
        'world': scenario['world'],
        'goals': goals,
    }
