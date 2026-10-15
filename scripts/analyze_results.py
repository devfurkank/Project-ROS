#!/usr/bin/env python3
"""
Deney sonuçlarını özetler: senaryo başına tablo, çubuk grafikler, yörünge ve harita çizimleri.

Kullanım:
    python3 scripts/analyze_results.py [sonuç_klasörü] [--scenarios dosya]

Sonuç klasörü (varsayılan <depo>/results) run_experiments.sh ve mission_manager'ın
çıktılarını içerir: runs.csv, goals.csv, experiments.csv (varsa), trajectories/, maps/.
Aynı klasöre yazılanlar:
    summary.csv                  senaryo başına özet (sayısal)
    summary.md                   rapor için tablolar, başarısız koşuların listesi
    figures/metrics.png          başarı oranı ve ortalama ± std çubuk grafikleri
    figures/trajectory_s<N>.png  senaryonun dünyası ve bütün koşuların yörüngeleri
    figures/map_s<N>.png         bir koşunun SLAM haritası, tasarlanan geometri ve yörünge

Süre, yol, verim ve hedefte kalan hata yalnızca başarılı koşulardan (hedeflerden) hesaplanır;
en küçük engel mesafesi ve kurtarma sayısı sonucu kaydedilen bütün koşulardan.
experiments.csv'de olup runs.csv'de satırı olmayan koşu (gerçek zaman sınırına takılmış)
başarısız sayılır.

Senaryoların dünyası ve hedefleri deponun otonom_surus/config/scenarios.yaml'ından okunur.
ROS gerekmez: pandas, matplotlib, Pillow ve PyYAML yeterli.
"""

import argparse
import math
import os
import sys
import time
import xml.etree.ElementTree as ET

from matplotlib.lines import Line2D
from matplotlib.patches import Circle, Patch, Polygon
import matplotlib.pyplot as plt
import pandas as pd
from PIL import Image
import yaml

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PACKAGE = os.path.join(REPO, 'otonom_surus')
sys.path.insert(0, PACKAGE)
from otonom_surus.scenarios import load_scenario  # noqa: E402, I100

NO_RECORD = 'KAYIT_YOK'  # experiments.csv'de var, runs.csv'de yok

# Renkler: tek seri tek renk; başarısızlık durum rengiyle (kırmızı) ve etiketle gösterilir.
INK, INK2, MUTED = '#0b0b0b', '#52514e', '#898781'
GRID, AXIS, SURFACE = '#e1e0d9', '#c3c2b7', '#fcfcfb'
BLUE, ORANGE, YELLOW = '#2a78d6', '#eb6834', '#eda100'
FAILURE = '#d03b3b'
OBSTACLE = '#bdbcb4'

plt.switch_backend('Agg')
plt.rcParams.update({
    'font.size': 9, 'axes.titlesize': 10, 'axes.titlecolor': INK, 'axes.labelcolor': INK2,
    'axes.edgecolor': AXIS, 'xtick.color': AXIS, 'ytick.color': AXIS,
    'xtick.labelcolor': INK2, 'ytick.labelcolor': INK2, 'legend.fontsize': 8,
    'figure.facecolor': 'white', 'axes.facecolor': SURFACE, 'savefig.dpi': 150,
})


# --- Veriler ----------------------------------------------------------------------------------

def read_results(results_dir, definitions):
    """
    runs.csv ve goals.csv'yi okur; kaydı olmayan denemeleri runs'a başarısız satır olarak ekler.

    Dönen: (runs, goals, kayıtsız deneme sayısı). Aynı (senaryo, run_id) iki kez yazılmışsa
    sonuncusu kullanılır.
    """
    path = os.path.join(results_dir, 'runs.csv')
    if not os.path.exists(path):
        sys.exit(f'analyze_results: {path} yok; önce scripts/run_experiments.sh')
    runs = pd.read_csv(path, dtype={'run_id': str})
    goals_path = os.path.join(results_dir, 'goals.csv')
    goals = (pd.read_csv(goals_path, dtype={'run_id': str}) if os.path.exists(goals_path)
             else pd.DataFrame(columns=['scenario', 'run_id', 'goal_index', 'result']))
    for name, frame in (('runs.csv', runs), ('goals.csv', goals)):
        keys = ['scenario', 'run_id'] + (['goal_index'] if name == 'goals.csv' else [])
        repeated = frame.duplicated(keys, keep='last')
        if repeated.any():
            print(f'uyarı: {name} içinde {repeated.sum()} tekrarlanan satır; sonuncular alındı')
    runs = runs.drop_duplicates(['scenario', 'run_id'], keep='last')
    goals = goals.drop_duplicates(['scenario', 'run_id', 'goal_index'], keep='last')

    unrecorded_attempts = 0
    experiments = os.path.join(results_dir, 'experiments.csv')
    if os.path.exists(experiments):
        attempts = pd.read_csv(experiments, dtype={'run_id': str, 'note': str})
        failed = attempts[attempts['recorded'] == 0]
        unrecorded_attempts = len(failed)
        recorded = set(zip(runs['scenario'], runs['run_id']))
        missing = failed[[(s, r) not in recorded
                          for s, r in zip(failed['scenario'], failed['run_id'])]]
        missing = missing.drop_duplicates(['scenario', 'run_id'], keep='last')
        rows = [{
            'timestamp': m.timestamp, 'scenario': m.scenario, 'run_id': m.run_id,
            'result': NO_RECORD, 'goals_reached': 0,
            'goal_count': len(definitions[m.scenario]['goals']) if m.scenario in definitions
            else math.nan,
            'wall_duration_s': math.nan, 'recoveries': math.nan,
            'failure_reason': f'sonuç yazılmadı: {m.note}',
        } for m in missing.itertuples()]
        if rows:
            runs = pd.concat([runs, pd.DataFrame(rows)], ignore_index=True)
    return runs.sort_values('timestamp', ignore_index=True), goals, unrecorded_attempts


def summarize(runs, goals, definitions):
    """Senaryo başına özet satırları (summary.csv)."""
    rows = []
    for scenario, group in runs.groupby('scenario'):
        ok = group[group['result'] == 'SUCCEEDED']
        recorded = group[group['result'] != NO_RECORD]
        scenario_goals = goals[(goals['scenario'] == scenario)
                               & goals['run_id'].isin(group['run_id'])]
        goals_ok = scenario_goals[scenario_goals['result'] == 'SUCCEEDED']
        rtf = recorded['duration_s'] / recorded['wall_duration_s']
        rows.append({
            'scenario': scenario,
            'name': definitions.get(scenario, {}).get('name', ''),
            'runs': len(group), 'succeeded': len(ok),
            'success_rate': len(ok) / len(group),
            'goals_reached': int(group['goals_reached'].sum()),
            'goals_total': int(group['goal_count'].sum()),
            'duration_mean': ok['duration_s'].mean(), 'duration_std': ok['duration_s'].std(),
            'wall_mean': ok['wall_duration_s'].mean(), 'wall_std': ok['wall_duration_s'].std(),
            'path_mean': ok['path_length_m'].mean(), 'path_std': ok['path_length_m'].std(),
            'efficiency_mean': ok['path_efficiency'].mean(),
            'efficiency_std': ok['path_efficiency'].std(),
            'min_dist_mean': recorded['min_obstacle_dist_m'].mean(),
            'min_dist_std': recorded['min_obstacle_dist_m'].std(),
            'min_dist_min': recorded['min_obstacle_dist_m'].min(),
            'recoveries_total': recorded['recoveries'].sum(),
            'final_error_mean': goals_ok['final_error_m'].mean(),
            'final_error_std': goals_ok['final_error_m'].std(),
            'final_error_max': goals_ok['final_error_m'].max(),
            'real_time_factor': rtf.mean(),
        })
    return pd.DataFrame(rows)


# --- summary.md -------------------------------------------------------------------------------

def fmt(mean, std=math.nan, digits=2):
    """'ortalama ± std'; değer yoksa '—', std yoksa (tek koşu) yalnızca ortalama."""
    if pd.isna(mean):
        return '—'
    if pd.isna(std):
        return f'{mean:.{digits}f}'
    return f'{mean:.{digits}f} ± {std:.{digits}f}'


def table(header, rows):
    lines = ['| ' + ' | '.join(header) + ' |', '|' + ' --- |' * len(header)]
    lines += ['| ' + ' | '.join(str(c) for c in row) + ' |' for row in rows]
    return '\n'.join(lines)


def write_markdown(path, results_dir, runs, goals, summary, unrecorded_attempts, figures):
    source = os.path.relpath(results_dir, REPO)
    if source.startswith('..'):  # depo dışındaki bir klasör
        source = results_dir
    out = [
        '# Deney sonuçları',
        '',
        f'`scripts/analyze_results.py` ile {time.strftime("%Y-%m-%d %H:%M")} tarihinde '
        f'`{source}` klasöründen üretildi: {len(runs)} koşu '
        f'({", ".join(f"S{s}: {n}" for s, n in runs.groupby("scenario").size().items())}).',
        '',
        '## Senaryo başına özet',
        '',
        table(
            ['Senaryo', 'Koşu', 'Başarılı', 'Ulaşılan hedef', 'Süre (s, sim)', 'Yol (m)',
             'Verim', 'En küçük engel mesafesi (m)', 'Kurtarma', 'Hedefte kalan hata (m)',
             'Gerçek zaman çarpanı'],
            [[f'S{r.scenario} — {r.name}', r.runs, f'{r.succeeded}/{r.runs}',
              f'{r.goals_reached}/{r.goals_total}', fmt(r.duration_mean, r.duration_std, 1),
              fmt(r.path_mean, r.path_std), fmt(r.efficiency_mean, r.efficiency_std, 3),
              f'{fmt(r.min_dist_mean, r.min_dist_std)} (en az {fmt(r.min_dist_min)})',
              fmt(r.recoveries_total, digits=0),
              f'{fmt(r.final_error_mean, r.final_error_std)} (en çok {fmt(r.final_error_max)})',
              fmt(r.real_time_factor)]
             for r in summary.itertuples()]),
        '',
        'Ortalama ± standart sapma (örneklem, n − 1). Süre, yol ve verim yalnızca başarılı '
        'koşulardan; hedefte kalan hata başarılı hedeflerden (durulan nokta → hedef; Nav2 '
        'toleransı 0.25 m); en küçük engel mesafesi (LiDAR → en yakın yüzey, engel ya da duvar) '
        've kurtarma sayısı (toplam) sonucu kaydedilen bütün koşulardan. Verim = kuş uçuşu / '
        'gidilen yol. Gerçek zaman çarpanı = sim süresi / gerçek süre.',
        '',
        '## Hedef başına',
        '',
    ]
    rows = []
    for (scenario, index), group in goals[goals['run_id'].isin(runs['run_id'])].groupby(
            ['scenario', 'goal_index']):
        group = group[group['run_id'].isin(runs.loc[runs['scenario'] == scenario, 'run_id'])]
        if group.empty:
            continue
        ok = group[group['result'] == 'SUCCEEDED']
        first = group.iloc[0]
        rows.append([
            f'S{scenario}', index,
            f'({first.goal_x:g}, {first.goal_y:g}, {first.goal_yaw_deg:g}°)',
            f'{len(ok)}/{len(group)}', fmt(ok['duration_s'].mean(), ok['duration_s'].std(), 1),
            fmt(ok['path_length_m'].mean(), ok['path_length_m'].std()),
            fmt(group['recoveries'].sum(), digits=0),
            fmt(ok['final_error_m'].mean(), ok['final_error_m'].std())])
    out += [table(['Senaryo', 'Hedef', 'Konum (x, y, yaw)', 'Başarılı', 'Süre (s, sim)',
                   'Yol (m)', 'Kurtarma', 'Kalan hata (m)'], rows), '']

    out += ['## Başarısız ya da kaydı olmayan koşular', '']
    failed = runs[runs['result'] != 'SUCCEEDED']
    if failed.empty:
        out.append(f'Yok: {len(runs)} koşunun hepsi başarılı.')
    else:
        out.append(table(
            ['Senaryo', 'Koşu', 'Sonuç', 'Ulaşılan hedef', 'Neden'],
            [[f'S{r.scenario}', r.run_id, r.result, f'{r.goals_reached:g}/{r.goal_count:g}',
              r.failure_reason] for r in failed.itertuples()]))
    if unrecorded_attempts:
        out += ['', f'experiments.csv: {unrecorded_attempts} deneme sonuç yazamadan kesildi '
                '(sonradan yeniden koşulup kaydedilenler dahil).']
    out += ['', '## Bütün koşular', '', table(
        ['Senaryo', 'Koşu', 'Sonuç', 'Hedef', 'Süre (s, sim)', 'Gerçek süre (s)', 'Yol (m)',
         'Verim', 'En küçük engel mesafesi (m)', 'Kurtarma'],
        [[f'S{r.scenario}', r.run_id, r.result, f'{r.goals_reached:g}/{r.goal_count:g}',
          fmt(r.duration_s, digits=1), fmt(r.wall_duration_s, digits=1),
          fmt(r.path_length_m), fmt(r.path_efficiency, digits=3),
          fmt(r.min_obstacle_dist_m), fmt(r.recoveries, digits=0)]
         for r in runs.itertuples()])]
    out += ['', '## Şekiller', ''] + [f'- `{os.path.relpath(f, results_dir)}`' for f in figures]
    with open(path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(out) + '\n')


# --- Dünya geometrisi ------------------------------------------------------------------------

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


def _rectangle(x, y, yaw, length, width):
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + c * dx - s * dy, y + s * dx + c * dy)
            for dx, dy in ((-length / 2, -width / 2), (length / 2, -width / 2),
                           (length / 2, width / 2), (-length / 2, width / 2))]


def _extent(shape):
    """Şeklin (x, y) genişliği: kutuda (en, boy), silindirde çap."""
    if shape['kind'] == 'box':
        return shape['size']
    return 2 * shape['size'], 2 * shape['size']


def draw_world(ax, shapes, moving, outline=False):
    """
    Statik engelleri ve hareketli engellerin şeritlerini çizer.

    moving: {model adı: şerit uzunluğu}. Hareketli engel doğduğu yerden kendi +x ekseni boyunca
    gider (obstacle_mover); şerit, engelin taradığı alandır. outline=True ise yalnızca
    çerçeve çizilir (harita üstüne).
    """
    for shape in shapes:
        x, y, yaw = shape['x'], shape['y'], shape['yaw']
        if shape['model'] in moving:
            length, width = _extent(shape)
            travel = moving[shape['model']]
            ax.add_patch(Polygon(
                _rectangle(x + travel / 2 * math.cos(yaw), y + travel / 2 * math.sin(yaw), yaw,
                           travel + length, width),
                facecolor=YELLOW, alpha=0.18 if not outline else 0.12, edgecolor='none'))
            ax.add_patch(Polygon(_rectangle(x, y, yaw, length, width), fill=False,
                                 edgecolor=YELLOW, linewidth=1.2))
            continue
        style = ({'fill': False, 'edgecolor': BLUE, 'linewidth': 0.8} if outline
                 else {'facecolor': OBSTACLE, 'edgecolor': 'none'})
        if shape['kind'] == 'box':
            ax.add_patch(Polygon(_rectangle(x, y, yaw, *shape['size']), **style))
        else:
            ax.add_patch(Circle((x, y), shape['size'], **style))


def world_bounds(shapes, margin=0.15):
    points = []
    for shape in shapes:
        if shape['kind'] == 'box':
            points += _rectangle(shape['x'], shape['y'], shape['yaw'], *shape['size'])
        else:
            r = shape['size']
            points += [(shape['x'] - r, shape['y'] - r), (shape['x'] + r, shape['y'] + r)]
    xs, ys = zip(*points)
    return min(xs) - margin, max(xs) + margin, min(ys) - margin, max(ys) + margin


def draw_goals(ax, definition, tolerance):
    for i, (x, y, yaw) in enumerate(definition['goals'], 1):
        ax.add_patch(Circle((x, y), tolerance, fill=False, edgecolor=INK2, linewidth=0.8))
        ax.annotate('', xy=(x + 0.35 * math.cos(yaw), y + 0.35 * math.sin(yaw)), xytext=(x, y),
                    arrowprops={'arrowstyle': '->', 'color': INK, 'linewidth': 1})
        ax.plot(x, y, 'o', markersize=11, color=INK, markeredgecolor=SURFACE, zorder=5)
        ax.text(x, y, str(i), color='white', fontsize=7, ha='center', va='center', zorder=6)
    ax.plot(0, 0, '>', markersize=9, color=INK, markeredgecolor=SURFACE, zorder=5)


def frame(ax, bounds, title):
    ax.set_xlim(bounds[0], bounds[1])
    ax.set_ylim(bounds[2], bounds[3])
    ax.set_aspect('equal')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title(title, loc='left')
    ax.grid(color=GRID, linewidth=0.6)
    ax.set_axisbelow(True)


def legend_below(ax, handles, ncol=3):
    ax.legend(handles=handles, loc='upper center', bbox_to_anchor=(0.5, -0.1), ncol=ncol,
              frameon=False, labelcolor=INK2)


def world_handles(moving, tolerance, outline=False):
    handles = [
        Line2D([], [], marker='>', linestyle='none', color=INK, label='Başlangıç (0, 0)'),
        Line2D([], [], marker='o', linestyle='none', color=INK,
               label=f'Hedef (sıra no; daire: {tolerance:g} m tolerans)'),
        (Line2D([], [], color=BLUE, linewidth=0.8, label='Tasarlanan engeller ve duvarlar')
         if outline else Patch(facecolor=OBSTACLE, label='Statik engel / duvar'))]
    if moving:
        handles.append(Patch(facecolor=YELLOW, alpha=0.4, edgecolor=YELLOW,
                             label='Hareketli engel şeridi (çerçeve: başlangıç konumu)'))
    return handles


# --- Şekiller --------------------------------------------------------------------------------

def plot_metrics(path, runs, goals):
    """Başarı oranı ve beş metrik için senaryo başına ortalama ± std (noktalar: koşular)."""
    scenarios = sorted(runs['scenario'].unique())
    ok = runs[runs['result'] == 'SUCCEEDED']
    recorded = runs[runs['result'] != NO_RECORD]
    goals_ok = goals[(goals['result'] == 'SUCCEEDED') & goals['run_id'].isin(runs['run_id'])]
    panels = [
        ('Süre (s, sim)', ok, 'duration_s', 1),
        ('Yol uzunluğu (m)', ok, 'path_length_m', 2),
        ('Yol verimliliği (kuş uçuşu / yol)', ok, 'path_efficiency', 3),
        ('En küçük engel mesafesi (m)', recorded, 'min_obstacle_dist_m', 2),
        ('Hedefte kalan hata (m)', goals_ok, 'final_error_m', 2),
    ]
    fig, axes = plt.subplots(2, 3, figsize=(10, 6.4))
    axes = axes.ravel()
    labels = [f'S{s}' for s in scenarios]
    positions = range(len(scenarios))

    ax = axes[0]
    counts = [(runs['scenario'] == s).sum() for s in scenarios]
    wins = [(ok['scenario'] == s).sum() for s in scenarios]
    rates = [100 * w / n for w, n in zip(wins, counts)]
    ax.bar(positions, rates, width=0.34, color=BLUE)
    for i, (w, n, r) in enumerate(zip(wins, counts, rates)):
        ax.annotate(f'{w}/{n}', (i, r), xytext=(0, 3), textcoords='offset points',
                    ha='center', va='bottom', color=INK, fontsize=8)
    ax.set_ylim(0, 115)
    ax.set_yticks([0, 25, 50, 75, 100])
    ax.set_title('Başarı oranı (%)', loc='left')

    for ax, (title, frame_, column, digits) in zip(axes[1:], panels):
        top = 0.0
        for i, s in enumerate(scenarios):
            values = frame_.loc[frame_['scenario'] == s, column].dropna()
            if values.empty:
                ax.text(i, 0, '—', ha='center', va='bottom', color=MUTED)
                continue
            mean, std = values.mean(), values.std()
            ax.bar(i, mean, width=0.34, color=BLUE)
            if len(values) > 1:
                ax.errorbar(i, mean, yerr=std, color=INK2, capsize=3, linewidth=1)
            spread = [i + 0.16 * (k / max(len(values) - 1, 1) - 0.5)
                      for k in range(len(values))]
            ax.scatter(spread, values, s=12, color=INK2, edgecolor=SURFACE, linewidth=0.8,
                       zorder=3)
            peak = max(values.max(), mean + (std if len(values) > 1 else 0))
            ax.annotate(f'{mean:.{digits}f}', (i, peak), xytext=(0, 4),
                        textcoords='offset points', ha='center', va='bottom', color=INK,
                        fontsize=8)
            top = max(top, peak)
        ax.set_ylim(0, top * 1.22 if top > 0 else 1)
        ax.set_title(title, loc='left')

    for ax in axes:
        ax.set_xticks(list(positions))
        ax.set_xticklabels(labels)
        ax.set_xlim(-0.6, len(scenarios) - 0.4)
        ax.grid(axis='y', color=GRID, linewidth=0.6)
        ax.set_axisbelow(True)
        for side in ('top', 'right'):
            ax.spines[side].set_visible(False)
    fig.text(0.01, 0.005,
             'Çubuk: ortalama · çizgi: ± standart sapma · nokta: tek koşu (kalan hatada tek '
             'hedef). Süre, yol, verim: başarılı koşular; kalan hata: başarılı hedefler;\nengel '
             'mesafesi: LiDAR → en yakın yüzey (engel ya da duvar), sonucu kaydedilen bütün '
             'koşular.', color=MUTED, fontsize=8)
    fig.tight_layout(rect=(0, 0.05, 1, 1))
    fig.savefig(path)
    plt.close(fig)


def failure_time(goals, scenario, run_id):
    """
    İlk başarısız hedefin bittiği an (s, görevin başından); hedef satırı yoksa None.

    Hedef süreleri art arda toplanır; hedefler arasındaki geçiş milisaniyeler sürer.
    """
    elapsed = 0.0
    run_goals = goals[(goals['scenario'] == scenario) & (goals['run_id'] == run_id)]
    for goal in run_goals.sort_values('goal_index').itertuples():
        elapsed += goal.duration_s
        if goal.result != 'SUCCEEDED':
            return elapsed
    return None


def plot_trajectories(path, scenario, definition, shapes, moving, runs, goals, results_dir,
                      tolerance):
    """
    Senaryonun bütün koşularının yörüngeleri; başarısız koşular kırmızı.

    Başarısız koşuda X, ilk başarısız hedefin bittiği yerdir (robot sonra sıradaki hedeflere
    geçmiş olabilir; yörüngenin sonu değil).
    """
    fig, ax = plt.subplots(figsize=(6.4, 7.2))
    draw_world(ax, shapes, moving)
    group = runs[runs['scenario'] == scenario]
    ok_count, failed, missing = 0, [], []
    for run in group.itertuples():
        trajectory = os.path.join(results_dir, 'trajectories', f's{scenario}_{run.run_id}.csv')
        if not os.path.exists(trajectory):
            missing.append(f'{run.run_id} {run.result}')
            continue
        points = pd.read_csv(trajectory)
        if run.result == 'SUCCEEDED':
            ok_count += 1
            ax.plot(points['x'], points['y'], color=BLUE, linewidth=1.4, alpha=0.75,
                    solid_capstyle='round')
            continue
        failed.append(f'{run.run_id} {run.result}')
        ax.plot(points['x'], points['y'], color=FAILURE, linewidth=1.6, alpha=0.8,
                solid_capstyle='round', zorder=4)
        end = failure_time(goals, scenario, run.run_id)
        stop = (points.iloc[-1] if end is None
                else points.loc[(points['t'] - end).abs().idxmin()])
        ax.plot(stop['x'], stop['y'], 'X', color=FAILURE, markersize=9,
                markeredgecolor=SURFACE, zorder=5)
    draw_goals(ax, definition, tolerance)
    handles = []
    if ok_count:
        handles.append(Line2D([], [], color=BLUE, linewidth=1.4,
                              label=f'Başarılı koşu ({ok_count})'))
    if failed:
        handles.append(Line2D([], [], color=FAILURE, linewidth=1.6, marker='X',
                              label=f'Başarısız koşu ({len(failed)}; X: başarısız hedefin '
                                    f'bittiği yer)'))
    if missing:
        handles.append(Line2D([], [], color=FAILURE, linestyle='none', marker='X',
                              label=f'Yörüngesi yok: {", ".join(missing)}'))
    frame(ax, world_bounds(shapes),
          f'S{scenario} — {definition["name"]}: {len(group)} koşunun yörüngesi')
    legend_below(ax, handles + world_handles(moving, tolerance), ncol=2)
    fig.savefig(path, bbox_inches='tight')
    plt.close(fig)


def plot_map(path, scenario, definition, shapes, moving, runs, results_dir, tolerance):
    """Bir koşunun SLAM haritası; tercihen ilk başarılı koşu. Harita yoksa False."""
    group = runs[runs['scenario'] == scenario]
    group = pd.concat([group[group['result'] == 'SUCCEEDED'],
                       group[group['result'] != 'SUCCEEDED']])
    for run in group.itertuples():
        prefix = os.path.join(results_dir, 'maps', f's{scenario}_{run.run_id}')
        if os.path.exists(prefix + '.pgm') and os.path.exists(prefix + '.yaml'):
            break
    else:
        return False
    with open(prefix + '.yaml', encoding='utf-8') as f:
        meta = yaml.safe_load(f)
    image = Image.open(prefix + '.pgm')
    width, height = image.size
    res, (ox, oy) = meta['resolution'], meta['origin'][:2]

    fig, ax = plt.subplots(figsize=(6.4, 7.2))
    ax.imshow(image, cmap='gray', vmin=0, vmax=255, interpolation='nearest',
              extent=(ox, ox + width * res, oy, oy + height * res))
    draw_world(ax, shapes, moving, outline=True)
    trajectory = os.path.join(results_dir, 'trajectories', f's{scenario}_{run.run_id}.csv')
    handles = []
    if os.path.exists(trajectory):
        points = pd.read_csv(trajectory)
        ax.plot(points['x'], points['y'], color=ORANGE, linewidth=1.4)
        handles.append(Line2D([], [], color=ORANGE, linewidth=1.4,
                              label=f'Yörünge ({run.run_id}, {run.result})'))
    draw_goals(ax, definition, tolerance)
    frame(ax, world_bounds(shapes),
          f'S{scenario} — SLAM haritası ({run.run_id}, {width}×{height} hücre, {res:g} m)')
    ax.grid(False)
    handles.append(Patch(facecolor='black', label='Dolu'))
    handles.append(Patch(facecolor='white', edgecolor=AXIS, label='Boş'))
    handles.append(Patch(facecolor='#cdcdcd', label='Bilinmeyen'))
    legend_below(ax, handles + world_handles(moving, tolerance, outline=True), ncol=2)
    fig.savefig(path, bbox_inches='tight')
    plt.close(fig)
    return True


# --- Ana akış --------------------------------------------------------------------------------

def goal_tolerance():
    """Nav2 hedef denetleyicisinin xy toleransı (nav2_params.yaml), bulunamazsa 0.25 m."""
    try:
        with open(os.path.join(PACKAGE, 'config', 'nav2_params.yaml'), encoding='utf-8') as f:
            params = yaml.safe_load(f)
        return float(params['controller_server']['ros__parameters']['general_goal_checker']
                     ['xy_goal_tolerance'])
    except (OSError, KeyError, TypeError, ValueError):
        return 0.25


def main():
    parser = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    parser.add_argument('results_dir', nargs='?', default=os.path.join(REPO, 'results'),
                        help='sonuç klasörü (varsayılan <depo>/results)')
    parser.add_argument('--scenarios', default=os.path.join(PACKAGE, 'config', 'scenarios.yaml'),
                        help='senaryo tanımları (varsayılan deponun scenarios.yaml\'ı)')
    args = parser.parse_args()
    results_dir = os.path.abspath(args.results_dir)

    with open(args.scenarios, encoding='utf-8') as f:
        numbers = list(yaml.safe_load(f) or {})
    definitions = {int(n): load_scenario(args.scenarios, n) for n in numbers}
    runs, goals, unrecorded_attempts = read_results(results_dir, definitions)
    summary = summarize(runs, goals, definitions)
    summary.round(4).to_csv(os.path.join(results_dir, 'summary.csv'), index=False)

    figure_dir = os.path.join(results_dir, 'figures')
    os.makedirs(figure_dir, exist_ok=True)
    figures = [os.path.join(figure_dir, 'metrics.png')]
    plot_metrics(figures[0], runs, goals)
    tolerance = goal_tolerance()
    for scenario in sorted(runs['scenario'].unique()):
        definition = definitions.get(scenario)
        if definition is None:
            print(f'uyarı: S{scenario} {args.scenarios} içinde yok; çizilmedi')
            continue
        world = os.path.join(PACKAGE, 'worlds', definition['world'])
        shapes = world_shapes(world)
        moving = {o['name']: o['travel'] for o in definition['moving_obstacles']}
        figures.append(os.path.join(figure_dir, f'trajectory_s{scenario}.png'))
        plot_trajectories(figures[-1], scenario, definition, shapes, moving, runs, goals,
                          results_dir, tolerance)
        map_figure = os.path.join(figure_dir, f'map_s{scenario}.png')
        if plot_map(map_figure, scenario, definition, shapes, moving, runs, results_dir,
                    tolerance):
            figures.append(map_figure)

    write_markdown(os.path.join(results_dir, 'summary.md'), results_dir, runs, goals, summary,
                   unrecorded_attempts, figures)
    for r in summary.itertuples():
        duration = fmt(r.duration_mean, r.duration_std, 1)
        print(f'S{r.scenario}: {r.succeeded}/{r.runs} başarılı, süre {duration} s, '
              f'yol {fmt(r.path_mean, r.path_std)} m, en küçük mesafe {fmt(r.min_dist_min)} m, '
              f'{fmt(r.recoveries_total, digits=0)} kurtarma')
    print(f'Yazıldı: {os.path.join(results_dir, "summary.md")}, summary.csv, '
          f'{len(figures)} şekil ({figure_dir})')


if __name__ == '__main__':
    main()
