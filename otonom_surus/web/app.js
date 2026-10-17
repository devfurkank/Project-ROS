// Otonom sürüş paneli istemcisi.
// P1: bağlantı göstergesi, mod seçici. P2: dünya kartları, görev seçimi, başlat/durdur, koşu kartı.
// Sunucuyla iki yol: /api/stream (SSE, sunucu -> tarayıcı) ve /api/... (JSON istekleri).
'use strict';

const $ = (id) => document.getElementById(id);

const MODES = {
  live: {title: 'Harita', empty: 'Dünya seçilmedi',
    hint: 'Önizleme: seçili dünya, görevin hedefleri ve robotun doğduğu nokta. Canlı harita P3’te.'},
  replay: {title: 'Tekrar oynatma', empty: 'Koşu seçilmedi',
    hint: 'Koşu seçici ve hayalet robotlar P4–P5’te gelecek.'},
  edit: {title: 'Dünya tasarla', empty: 'Tasarlayıcı yakında',
    hint: 'Dünya tasarlayıcı P9’da gelecek.'},
};
// Koşu aşamaları: [rozet metni, rozet rengi]
const PHASES = {
  idle: ['Hazır', 'neutral'],
  starting: ['Başlatılıyor', 'warn'],
  running: ['Görevde', 'accent'],
  finishing: ['Tamamlanıyor', 'accent'],
  stopping: ['Durduruluyor', 'warn'],
  finished: ['Tamamlandı', 'ok'],
  stopped: ['Durduruldu', 'bad'],
  error: ['Hata', 'bad'],
};
const ACTIVE = new Set(['starting', 'running', 'finishing', 'stopping']);
const STALE_MS = 10000;  // bu kadar süre veri gelmezse bağlantı kopmuş sayılır
const HEALTH_MS = 10000;  // sunucu bilgisinin yenilenme aralığı
const LOG_LINES = 80;

const state = {
  mode: 'live',
  connected: null,  // null: henüz bilinmiyor (ilk açılış)
  lastSeen: 0,
  health: null,
  healthAt: 0,
  source: null,
  worlds: {},  // dosya adı -> geometri
  scenarios: [],
  scenario: null,  // seçili senaryo numarası
  cards: [],  // dünya kartları: {canvas, geometry, scenario}
  run: {phase: 'idle'},
  busy: false,  // başlat/durdur isteği yolda
};

// --- Yardımcılar ---------------------------------------------------------------------------

function store(key, value) {
  try {
    localStorage.setItem(key, value);
  } catch (e) {
    // Gizli pencere ya da engellenmiş depolama: yalnızca bu oturumda hatırlanır.
  }
}

function restore(key) {
  try {
    return localStorage.getItem(key);
  } catch (e) {
    return null;
  }
}

function cssVar(name) {
  return getComputedStyle(document.documentElement).getPropertyValue(name).trim();
}

function baseName(path) {
  return String(path).split('/').pop();
}

function el(tag, cls, text) {
  const e = document.createElement(tag);
  if (cls) e.className = cls;
  if (text !== undefined) e.textContent = text;
  return e;
}

function clock(t) {
  return new Date(t).toLocaleTimeString('tr-TR', {hour12: false});
}

function duration(s) {
  s = Math.max(0, Math.round(s));
  if (s < 60) return `${s} s`;
  if (s < 3600) return `${Math.floor(s / 60)} dk ${s % 60} s`;
  return `${Math.floor(s / 3600)} sa ${Math.floor((s % 3600) / 60)} dk`;
}

function scenarioById(id) {
  return state.scenarios.find((s) => s.id === id) || null;
}

// --- Olay akışı ----------------------------------------------------------------------------

function addLog(entry, time) {
  const box = $('log');
  const line = el('div', entry.level || 'INFO');
  const source = entry.source && entry.source !== 'panel' ? `${entry.source}: ` : '';
  line.textContent = `${clock(time || Date.now())}  ${source}${entry.text}`;
  const atBottom = box.scrollTop + box.clientHeight >= box.scrollHeight - 4;
  box.appendChild(line);
  while (box.childElementCount > LOG_LINES) box.firstElementChild.remove();
  if (atBottom) box.scrollTop = box.scrollHeight;
}

function note(text, level) {
  addLog({level: level || 'INFO', source: 'panel', text});
}

// --- Bağlantı ------------------------------------------------------------------------------

function setConnection(ok) {
  if (state.connected === ok) return;
  const first = state.connected === null;
  state.connected = ok;
  $('conn').className = 'conn ' + (ok ? 'on' : 'off');
  $('connText').textContent = ok ? 'ROS 2 bağlı' : 'Bağlantı yok';
  if (ok) note('Sunucuya bağlandı');
  else note(first ? 'Sunucuya ulaşılamıyor, yeniden deneniyor' : 'Bağlantı koptu, yeniden deneniyor',
    'WARN');
  renderControls();
}

function seen() {
  state.lastSeen = Date.now();
  setConnection(true);
}

function setHealth(h) {
  state.health = h;
  state.healthAt = Date.now();
  renderHealth();
}

function renderHealth() {
  const h = state.health;
  if (!h) return;
  const live = state.connected;
  $('hVersion').textContent = h.version;
  $('hClients').textContent = live ? String(h.clients) : '–';
  $('hUptime').textContent = live ? duration(h.uptime_s + (Date.now() - state.healthAt) / 1000) : '–';
  $('hSeen').textContent = state.lastSeen ? `${duration((Date.now() - state.lastSeen) / 1000)} önce` : '–';
  $('hResults').textContent = h.results_dir;
  renderCommand();
}

async function refreshHealth() {
  if (!state.connected) return;
  try {
    const r = await fetch('/api/health', {cache: 'no-store'});
    if (r.ok) setHealth(await r.json());
  } catch (e) {
    // Bağlantı durumu SSE'den izleniyor; burada ayrıca bildirmeye gerek yok.
  }
}

function connect() {
  const es = new EventSource('/api/stream');
  state.source = es;
  es.addEventListener('hello', (e) => {
    const hello = JSON.parse(e.data);
    seen();
    setHealth(hello);
    $('log').textContent = '';
    (hello.log || []).forEach((entry) => addLog(entry));
    applyRun(hello.run || {phase: 'idle'}, true);
    if (!state.scenarios.length) loadCatalog();
  });
  es.addEventListener('ping', seen);
  es.addEventListener('run', (e) => applyRun(JSON.parse(e.data)));
  es.addEventListener('log', (e) => addLog(JSON.parse(e.data)));
  es.onerror = () => {
    setConnection(false);
    // CONNECTING iken tarayıcı kendisi yeniden dener (sunucunun retry süresiyle);
    // CLOSED ise (ör. sunucu hata yanıtı verdi) bağlantıyı biz yeniden kurarız.
    if (es.readyState === EventSource.CLOSED) {
      es.close();
      setTimeout(connect, 3000);
    }
  };
}

// --- Dünyalar ve senaryolar ----------------------------------------------------------------

async function getJson(url) {
  const r = await fetch(url, {cache: 'no-store'});
  const data = await r.json();
  if (!r.ok) throw new Error(data.error || `${url}: HTTP ${r.status}`);
  return data;
}

async function loadCatalog() {
  try {
    const [worlds, scenarios] = await Promise.all([getJson('/api/worlds'), getJson('/api/scenarios')]);
    state.worlds = {};
    worlds.forEach((w) => {
      if (w.geometry) state.worlds[w.file] = w.geometry;
      else note(`${w.file} okunamadı: ${w.error}`, 'ERROR');
    });
    state.scenarios = scenarios;
    const saved = Number(restore('panel.scenario'));
    const running = ACTIVE.has(state.run.phase) ? state.run.scenario : null;
    state.scenario = scenarioById(running) ? running
      : scenarioById(saved) ? saved : (scenarios[0] ? scenarios[0].id : null);
    renderWorlds();
    renderTasks();
    renderCommand();
    renderControls();
    drawMap();
  } catch (e) {
    $('err').textContent = `Dünyalar ya da senaryolar yüklenemedi: ${e.message}`;
  }
}

// Senaryoları dünyalarına göre gruplar: [{file, title, scenarios: [...]}], ilk senaryo kartın adı.
function worldGroups() {
  const groups = new Map();
  state.scenarios.forEach((s) => {
    const file = baseName(s.world);
    if (!groups.has(file)) groups.set(file, {file, title: s.name, scenarios: []});
    groups.get(file).scenarios.push(s);
  });
  return [...groups.values()];
}

function taskLabel(s, group) {
  if (s.name.startsWith(group.title + ', ')) return s.name.slice(group.title.length + 2);
  if (group.scenarios.length > 1) return 'Temel görev';
  if (s.goals.length === 1) return `Tek hedef (${s.goals[0][0]}, ${s.goals[0][1]})`;
  return `Sıralı ${s.goals.length} hedef`;
}

// Kartlar katalog yüklenince bir kez kurulur; boyut, tema ya da mod değişince yalnızca
// önizlemeler yeniden çizilir (drawPreviews), böylece odak ve tıklama hedefleri korunur.
function renderWorlds() {
  const box = $('worlds');
  box.textContent = '';
  state.cards = [];
  const current = scenarioById(state.scenario);
  worldGroups().forEach((g) => {
    const base = g.scenarios[0];
    const card = el('button', 'wcard');
    card.type = 'button';
    card.dataset.file = g.file;
    const on = current && baseName(current.world) === g.file;
    card.classList.toggle('on', Boolean(on));
    card.setAttribute('aria-pressed', String(Boolean(on)));
    const cv = el('canvas');
    card.appendChild(cv);
    card.appendChild(el('p', 'wname', g.title));
    const movers = base.moving_obstacles.length;
    const sub = movers ? `${movers} hareketli kutu` : `${base.goals.length} hedef`;
    card.appendChild(el('p', 'wsub', `${g.file} · ${sub}`));
    card.addEventListener('click', () => selectScenario(base.id));
    box.appendChild(card);
    state.cards.push({canvas: cv, geometry: state.worlds[g.file], scenario: base});
  });
  requestAnimationFrame(drawPreviews);
}

function drawPreviews() {
  (state.cards || []).forEach((c) => drawWorld(c.canvas, c.geometry, c.scenario, false));
}

function renderTasks() {
  const box = $('tasks');
  box.textContent = '';
  const current = scenarioById(state.scenario);
  if (!current) return;
  const group = worldGroups().find((g) => g.file === baseName(current.world));
  group.scenarios.forEach((s) => {
    const chip = el('button', 'chip' + (s.id === current.id ? ' on' : ''));
    chip.type = 'button';
    chip.setAttribute('aria-pressed', String(s.id === current.id));
    chip.appendChild(document.createTextNode(taskLabel(s, group)));
    chip.appendChild(el('span', 'tag', `S${s.id}`));
    chip.addEventListener('click', () => selectScenario(s.id));
    box.appendChild(chip);
  });
}

function selectScenario(id) {
  if (ACTIVE.has(state.run.phase)) {
    note('Koşu sürerken görev değiştirilemez; önce durdur', 'WARN');
    return;
  }
  state.scenario = id;
  store('panel.scenario', String(id));
  $('err').textContent = '';
  document.querySelectorAll('.wcard').forEach((c) => {
    const s = scenarioById(id);
    const on = s && baseName(s.world) === c.dataset.file;
    c.classList.toggle('on', Boolean(on));
    c.setAttribute('aria-pressed', String(Boolean(on)));
  });
  renderTasks();
  renderCommand();
  renderControls();
  drawMap();
}

// --- Komut ve düğmeler ---------------------------------------------------------------------

function previewCommand() {
  if (ACTIVE.has(state.run.phase) && state.run.command) return state.run.command;
  if (state.scenario === null) return '';
  const dir = state.health ? state.health.results_dir : '<sonuç klasörü>';
  return `ros2 launch otonom_surus scenario.launch.py scenario:=${state.scenario}` +
    ` gui:=${$('gz').checked} rviz:=${$('rv').checked} results_dir:=${dir} run_id:=p<tarih-saat>`;
}

function renderCommand() {
  const cmd = previewCommand();
  $('cmd').textContent = cmd;
  $('cmd').title = cmd;
}

function renderControls() {
  const active = ACTIVE.has(state.run.phase);
  const go = $('go');
  const stop = $('stop');
  go.disabled = !state.connected || active || state.busy || state.scenario === null;
  stop.disabled = !state.connected || !active || state.run.phase === 'stopping' || state.busy;
  go.title = active ? 'Bir koşu sürüyor' : !state.connected ? 'Sunucuya bağlı değil' : '';
  $('gz').disabled = active;
  $('rv').disabled = active;
}

async function post(url, body) {
  const r = await fetch(url, {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify(body || {}),
  });
  const data = await r.json().catch(() => ({}));
  if (!r.ok) throw new Error(data.error || `HTTP ${r.status}`);
  return data;
}

async function startRun() {
  $('err').textContent = '';
  state.busy = true;
  renderControls();
  try {
    const run = await post('/api/run/start',
      {scenario: state.scenario, gui: $('gz').checked, rviz: $('rv').checked});
    applyRun(run);
    note(`Koşu başlatıldı: S${run.scenario}, ${run.run_id}`);
  } catch (e) {
    $('err').textContent = e.message;
    note(`Başlatılamadı: ${e.message}`, 'ERROR');
  } finally {
    state.busy = false;
    renderControls();
  }
}

async function stopRun() {
  $('err').textContent = '';
  state.busy = true;
  renderControls();
  try {
    applyRun(await post('/api/run/stop'));
  } catch (e) {
    $('err').textContent = e.message;
  } finally {
    state.busy = false;
    renderControls();
  }
}

// --- Koşu kartı ----------------------------------------------------------------------------

function applyRun(run, quiet) {
  const before = state.run.phase;
  state.run = run;
  if (!quiet && before !== run.phase) {
    if (run.phase === 'finished') note(`Koşu bitti: ${run.result} (${run.run_id})`);
    if (run.phase === 'error') note(run.message || 'Koşu hatayla bitti', 'ERROR');
  }
  renderRun();
  renderControls();
  renderCommand();
  if (state.health) state.health.sim = run.phase;
}

function renderRun() {
  const run = state.run;
  let [label, tone] = PHASES[run.phase] || [run.phase, 'neutral'];
  if (run.phase === 'finished' && run.result !== 'SUCCEEDED') {
    label = `Bitti: ${run.result}`;
    tone = 'bad';
  }
  const badge = $('badge');
  badge.textContent = label;
  badge.className = 'badge ' + tone;

  const s = scenarioById(run.scenario);
  $('runScn').textContent = run.scenario ? `S${run.scenario}${s ? ' · ' + s.name : ''}` : '';

  const count = run.goal_count || (s ? s.goals.length : 0);
  let line = 'Koşu yok';
  if (run.phase === 'starting') line = 'Gazebo, SLAM ve Nav2 açılıyor…';
  else if (run.phase === 'running' && run.goal) {
    const g = s && s.goals[run.goal - 1];
    line = `Hedef ${run.goal} / ${count}` + (g ? `  →  (${g[0]}, ${g[1]})` : '');
  } else if (run.phase === 'finishing') line = 'Sonuçlar ve harita yazılıyor…';
  else if (run.phase === 'stopping') line = 'Süreçler kapatılıyor…';
  else if (run.phase === 'finished' && run.summary) {
    line = `${run.summary.goals_reached}/${run.summary.goal_count} hedef · ${run.result}`;
  } else if (run.phase === 'stopped') line = 'Koşu durduruldu';
  else if (run.phase === 'error') line = run.message || 'Hata';
  $('runGoal').textContent = line;

  const bars = $('bars');
  bars.textContent = '';
  if (run.phase !== 'idle') {
    const results = run.goal_results || [];
    for (let i = 0; i < count; i++) {
      const r = results[i];
      let cls = '';
      if (r === 'SUCCEEDED') cls = 'ok';
      else if (r) cls = 'bad';
      else if (run.phase === 'running' && run.goal === i + 1) cls = 'cur';
      bars.appendChild(el('span', cls));
    }
  }

  const sum = $('runSummary');
  sum.textContent = '';
  if (run.phase === 'finished' && run.summary) {
    const m = run.summary;
    const rows = [
      ['Süre (sim)', `${m.duration_s} s`], ['Yol', `${m.path_length_m} m`],
      ['Verim', m.path_efficiency || '–'], ['En yakın engel', `${m.min_obstacle_dist_m} m`],
      ['Kurtarma', m.recoveries],
    ];
    if (m.failure_reason) rows.push(['Neden', m.failure_reason]);
    rows.forEach(([k, v]) => {
      sum.appendChild(el('dt', '', k));
      sum.appendChild(el('dd', '', v));
    });
  }
  renderElapsed();
}

function renderElapsed() {
  const run = state.run;
  let text = '';
  if (run.started) {
    const end = run.ended ? run.ended * 1000 : Date.now();
    text = `Başladı ${clock(run.started * 1000)} · ${run.ended ? 'sürdü' : 'geçen'} ` +
      duration((end - run.started * 1000) / 1000);
  }
  $('runMsg').textContent = text;
}

// --- Harita --------------------------------------------------------------------------------

// Dünya geometrisini, görevin hedeflerini ve robotun doğduğu noktayı tuvale çizer.
function drawWorld(cv, geometry, scenario, detailed) {
  const dpr = window.devicePixelRatio || 1;
  const w = cv.clientWidth;
  const h = cv.clientHeight;
  if (!w || !h) return null;
  cv.width = Math.round(w * dpr);
  cv.height = Math.round(h * dpr);
  const x = cv.getContext('2d');
  x.setTransform(dpr, 0, 0, dpr, 0, 0);
  x.fillStyle = cssVar(detailed ? '--surface-1' : '--surface-2');
  x.fillRect(0, 0, w, h);
  if (!geometry || !geometry.bounds) return null;

  const pad = 0.25;
  const [x0, y0, x1, y1] = geometry.bounds;
  const k = Math.min(w / (x1 - x0 + 2 * pad), h / (y1 - y0 + 2 * pad));
  const ox = (w - (x1 - x0) * k) / 2 - x0 * k;
  const oy = (h + (y1 - y0) * k) / 2 + y0 * k;
  const X = (v) => ox + v * k;
  const Y = (v) => oy - v * k;

  if (detailed) {
    x.strokeStyle = cssVar('--border');
    x.lineWidth = 1;
    x.beginPath();
    for (let i = Math.ceil(x0); i <= Math.floor(x1); i++) {
      x.moveTo(X(i), Y(y0));
      x.lineTo(X(i), Y(y1));
    }
    for (let j = Math.ceil(y0); j <= Math.floor(y1); j++) {
      x.moveTo(X(x0), Y(j));
      x.lineTo(X(x1), Y(j));
    }
    x.stroke();
  }

  const colors = {wall: cssVar('--map-wall'), obstacle: cssVar('--map-obstacle'),
    moving: cssVar('--map-moving')};
  geometry.shapes.forEach((sh) => {
    x.fillStyle = colors[sh.role] || colors.obstacle;
    if (sh.kind === 'cylinder') {
      x.beginPath();
      x.arc(X(sh.x), Y(sh.y), sh.size * k, 0, 2 * Math.PI);
      x.fill();
    } else {
      x.save();
      x.translate(X(sh.x), Y(sh.y));
      x.rotate(-sh.yaw);
      x.fillRect(-sh.size[0] / 2 * k, -sh.size[1] / 2 * k, sh.size[0] * k, sh.size[1] * k);
      x.restore();
    }
  });

  if (scenario && detailed) {
    // Hareketli kutuların şeritleri: doğdukları yerden kendi +x eksenleri boyunca travel metre.
    x.strokeStyle = colors.moving;
    x.setLineDash([4, 4]);
    x.lineWidth = 1.5;
    scenario.moving_obstacles.forEach((m) => {
      const sh = geometry.shapes.find((g) => g.model === m.name);
      if (!sh) return;
      x.beginPath();
      x.moveTo(X(sh.x), Y(sh.y));
      x.lineTo(X(sh.x + Math.cos(sh.yaw) * m.travel), Y(sh.y + Math.sin(sh.yaw) * m.travel));
      x.stroke();
    });
    x.setLineDash([]);
  }

  if (scenario) {
    const goal = cssVar('--map-goal');
    x.font = `500 11px ${getComputedStyle(document.body).fontFamily}`;
    scenario.goals.forEach((g, i) => {
      const r = 0.25 * k;
      const cx = X(g[0]);
      const cy = Y(g[1]);
      x.strokeStyle = goal;
      x.lineWidth = detailed ? 2 : 1.5;
      x.beginPath();
      x.arc(cx, cy, r, 0, 2 * Math.PI);
      x.stroke();
      if (detailed) {
        const a = g[2] * Math.PI / 180;
        x.beginPath();
        x.moveTo(cx, cy);
        x.lineTo(cx + Math.cos(a) * r, cy - Math.sin(a) * r);
        x.stroke();
        x.fillStyle = goal;
        x.fillText(String(i + 1), cx + r + 2, cy - r + 4);
      }
    });
  }

  const rr = Math.max(0.105 * k, detailed ? 5 : 3);
  x.fillStyle = cssVar('--map-robot');
  x.beginPath();
  x.arc(X(0), Y(0), rr, 0, 2 * Math.PI);
  x.fill();
  return {X, Y, k};
}

function drawMap() {
  const cv = $('map');
  const s = scenarioById(state.scenario);
  const geometry = state.mode === 'live' && s ? state.worlds[baseName(s.world)] : null;
  const view = drawWorld(cv, geometry, s, true);
  if (!view) {
    const x = cv.getContext('2d');
    x.fillStyle = cssVar('--text-secondary');
    x.font = `14px ${getComputedStyle(document.body).fontFamily}`;
    x.textAlign = 'center';
    x.textBaseline = 'middle';
    x.fillText(MODES[state.mode].empty, cv.clientWidth / 2, cv.clientHeight / 2);
    x.textAlign = 'start';
  }
  $('legend').style.display = view ? '' : 'none';
}

function renderLegend() {
  const items = [['--map-robot', 'Robot (doğduğu yer)', '50%'], ['--map-goal', 'Hedef', '50%'],
    ['--map-wall', 'Duvar', '2px'], ['--map-obstacle', 'Engel', '2px'],
    ['--map-moving', 'Hareketli kutu ve şeridi', '2px']];
  const box = $('legend');
  box.textContent = '';
  items.forEach(([color, text, radius]) => {
    const item = el('span');
    const sw = el('i');
    sw.style.background = `var(${color})`;
    sw.style.borderRadius = radius;
    item.appendChild(sw);
    item.appendChild(document.createTextNode(text));
    box.appendChild(item);
  });
}

// --- Mod -----------------------------------------------------------------------------------

function setMode(mode) {
  state.mode = mode;
  document.querySelectorAll('#modes button').forEach((b) => {
    const on = b.dataset.m === mode;
    b.classList.toggle('on', on);
    b.setAttribute('aria-pressed', String(on));
  });
  $('liveTop').style.display = mode === 'live' ? '' : 'none';
  $('mapTitle').textContent = MODES[mode].title;
  $('hint').textContent = MODES[mode].hint;
  store('panel.mode', mode);
  drawMap();
  if (mode === 'live') drawPreviews();  // gizliyken çizilen önizlemeler boş kalır
}

// --- Başlangıç -----------------------------------------------------------------------------

document.querySelectorAll('#modes button').forEach((b) => {
  b.addEventListener('click', () => setMode(b.dataset.m));
});
$('go').addEventListener('click', startRun);
$('stop').addEventListener('click', stopRun);
$('gz').addEventListener('change', renderCommand);
$('rv').addEventListener('change', renderCommand);
new ResizeObserver(() => {
  drawMap();
  drawPreviews();
}).observe($('map'));
window.matchMedia('(prefers-color-scheme: dark)').addEventListener('change', () => {
  drawMap();
  drawPreviews();
});

// Bağlantı açık görünse de veri kesildiyse (asılı bağlantı) yeniden bağlan.
setInterval(() => {
  if (state.connected && Date.now() - state.lastSeen > STALE_MS) {
    setConnection(false);
    state.source.close();
    connect();
  }
  renderHealth();
  if (ACTIVE.has(state.run.phase)) renderElapsed();
}, 1000);
setInterval(refreshHealth, HEALTH_MS);

renderLegend();
renderRun();
renderControls();
const savedMode = restore('panel.mode');
setMode(MODES[savedMode] ? savedMode : 'live');
connect();
