// Otonom sürüş paneli istemcisi.
// P1: bağlantı göstergesi, mod seçici. P2: dünya kartları, görev seçimi, başlat/durdur, koşu kartı.
// P3: canlı harita (SLAM, robot, iz, Nav2 planı, hareketli kutular), görev durumu ve metrikler.
// Sunucuyla iki yol: /api/stream (SSE, sunucu -> tarayıcı) ve /api/... (JSON istekleri).
'use strict';

const $ = (id) => document.getElementById(id);

const MODES = {
  live: {title: 'Harita', empty: 'Dünya seçilmedi',
    hint: 'Önizleme: seçili dünya, görevin hedefleri ve robotun doğduğu nokta. Başlatınca SLAM ' +
      'haritası, robot, izlenen yol ve Nav2 planı canlı çizilir.'},
  replay: {title: 'Tekrar oynatma', empty: 'Koşu seçilmedi',
    hint: 'Koşu seçici ve hayalet robotlar P4–P5’te gelecek.'},
  edit: {title: 'Dünya tasarla', empty: 'Tasarlayıcı yakında',
    hint: 'Dünya tasarlayıcı P9’da gelecek.'},
};
const LIVE_HINT = 'Canlı: SLAM haritası (/map; sisli alan henüz görülmedi), robot (TF map → ' +
  'base_footprint), izlenen yol ve kesikli Nav2 planı. İnce çizgiler dünyanın gerçek geometrisi; ' +
  'SLAM haritasıyla arasındaki fark haritalama hatasıdır.';
// Koşu aşamaları: [rozet metni, rozet rengi]. watching/watched: panel dışından başlatılmış görev.
const PHASES = {
  idle: ['Hazır', 'neutral'],
  starting: ['Başlatılıyor', 'warn'],
  running: ['Görevde', 'accent'],
  finishing: ['Tamamlanıyor', 'accent'],
  stopping: ['Durduruluyor', 'warn'],
  finished: ['Tamamlandı', 'ok'],
  stopped: ['Durduruldu', 'bad'],
  error: ['Hata', 'bad'],
  watching: ['İzleniyor', 'accent'],
  watched: ['İzlendi', 'neutral'],
};
const ACTIVE = new Set(['starting', 'running', 'finishing', 'stopping']);
const MOVING = new Set(['running', 'recovering']);  // /mission_status: robot hedefe gidiyor
const STALE_MS = 10000;  // bu kadar süre veri gelmezse bağlantı kopmuş sayılır
const HEALTH_MS = 10000;  // sunucu bilgisinin yenilenme aralığı
const POSE_STALE_MS = 3000;  // bu kadar süre konum gelmezse anlık metrikler '–'
const EXTERNAL_STALE_MS = 5000;  // panel dışı görevin durumu bu kadar gelmezse izleme biter
const LOG_LINES = 80;
const LEGENDS = {
  preview: [['dot', '--map-robot', 'Robot (doğduğu yer)'], ['ring', '--map-goal', 'Hedef'],
    ['box', '--map-wall', 'Duvar'], ['box', '--map-obstacle', 'Engel'],
    ['box', '--map-moving', 'Hareketli kutu ve şeridi']],
  live: [['dot', '--map-robot', 'Robot'], ['line', '--map-path', 'İzlenen yol'],
    ['dash', '--map-robot', 'Nav2 planı'], ['ring', '--map-goal', 'Hedef'],
    ['ring', '--map-ok', 'Varıldı'], ['ring', '--map-bad', 'Başarısız'],
    ['box', '--map-slam', 'SLAM: dolu'], ['box', '--map-fog', 'Henüz görülmedi'],
    ['outline', '--map-wall', 'Gerçek geometri'], ['box', '--map-moving', 'Hareketli kutu']],
};

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
  live: emptyLive(),
  legendKey: '',
};

// Canlı verinin önbelleği; yeni koşu başlarken boşaltılır. scenario: verinin ait olduğu senaryo.
function emptyLive() {
  return {scenario: null, status: null, statusAt: 0, pose: null, poseAt: 0, trail: [], plan: [],
    map: null, mapImg: null, mapCanvas: null, pendingMap: null};
}

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

// Herhangi bir CSS rengini [r, g, b, a]'ya çevirir (1 piksellik tuvalde boyayarak).
const swatchCtx = document.createElement('canvas').getContext('2d', {willReadFrequently: true});
function colorRgb(color) {
  swatchCtx.clearRect(0, 0, 1, 1);
  swatchCtx.fillStyle = color;
  swatchCtx.fillRect(0, 0, 1, 1);
  return swatchCtx.getImageData(0, 0, 1, 1).data;
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
  // Her olay bağlantının canlı olduğunu gösterir: veri akarken sunucu ping göndermez.
  const on = (type, handler) => es.addEventListener(type, (e) => {
    seen();
    handler(JSON.parse(e.data));
  });
  on('hello', (hello) => {
    setHealth(hello);
    $('log').textContent = '';
    (hello.log || []).forEach((entry) => addLog(entry));
    applyRun(hello.run || {phase: 'idle'}, true);
    applyLive(hello.live);
    if (!state.scenarios.length) loadCatalog();
  });
  on('ping', () => {});
  on('run', (run) => applyRun(run));
  on('log', (entry) => addLog(entry));
  on('status', setStatus);
  on('pose', setPose);
  on('map', setMap);
  on('trail', (d) => {
    state.live.trail = d.pts || [];
    redraw();
  });
  on('plan', (d) => {
    state.live.plan = d.pts || [];
    redraw();
  });
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
    const ext = externalStatus();
    const running = ACTIVE.has(state.run.phase) ? state.run.scenario : ext ? ext.scenario : null;
    state.scenario = scenarioById(running) ? running
      : scenarioById(saved) ? saved : (scenarios[0] ? scenarios[0].id : null);
    renderWorlds();
    renderTasks();
    renderCommand();
    renderControls();
    renderRun();
    renderMetrics();
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
  (state.cards || []).forEach((c) => drawWorld(c.canvas, c.geometry, c.scenario, false, null));
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

// remember: false ise seçim tarayıcıda saklanmaz (panel dışı görevin dünyasına geçerken).
function selectScenario(id, remember = true) {
  if (ACTIVE.has(state.run.phase)) {
    note('Koşu sürerken görev değiştirilemez; önce durdur', 'WARN');
    return;
  }
  state.scenario = id;
  if (remember) store('panel.scenario', String(id));
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
  renderMetrics();
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
  const ext = externalStatus();
  const watching = Boolean(ext) && ext.state !== 'finished';
  const go = $('go');
  const stop = $('stop');
  go.disabled = !state.connected || active || watching || state.busy || state.scenario === null;
  stop.disabled = !state.connected || !active || state.run.phase === 'stopping' || state.busy;
  go.title = active ? 'Bir koşu sürüyor' : !state.connected ? 'Sunucuya bağlı değil'
    : watching ? 'Simülasyon panel dışından çalışıyor; yalnızca izleniyor' : '';
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
    // SSE'deki 'run' olayı yanıttan önce gelmiş olabilir; eski yanıt yeni durumu ezmesin.
    if (run.run_id !== state.run.run_id) applyRun(run);
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
  if (run.run_id && run.run_id !== state.run.run_id) {
    // Yeni koşu: önceki koşunun haritası, izi ve konumu silinir (sunucu da siler).
    state.live = emptyLive();
    state.live.scenario = run.scenario;
  }
  state.run = run;
  if (!quiet && before !== run.phase) {
    if (run.phase === 'finished') note(`Koşu bitti: ${run.result} (${run.run_id})`);
    if (run.phase === 'error') note(run.message || 'Koşu hatayla bitti', 'ERROR');
  }
  renderRun();
  renderControls();
  renderCommand();
  renderMetrics();
  redraw();
  if (state.health) state.health.sim = run.phase;
}

// Panel dışından (terminalden) başlatılmış görevin son durumu; panelin kendi koşusuysa null.
function externalStatus() {
  const st = state.live.status;
  if (!st || ACTIVE.has(state.run.phase) || st.run_id === state.run.run_id) return null;
  if (st.state !== 'finished' && Date.now() - state.live.statusAt > EXTERNAL_STALE_MS) return null;
  return st;
}

// Koşu kartında gösterilen koşu: panelinki ya da panel dışından izlenen görev.
function runView() {
  const ext = externalStatus();
  if (!ext) return state.run;
  return {phase: ext.state === 'finished' ? 'watched' : 'watching', external: true,
    scenario: ext.scenario, run_id: ext.run_id, goal: ext.goal_index, goal_count: ext.goal_count,
    goal_results: ext.goal_results, result: ext.result, recovering: ext.state === 'recovering'};
}

function renderRun() {
  const run = runView();
  const going = run.phase === 'running' || run.phase === 'watching';
  let [label, tone] = PHASES[run.phase] || [run.phase, 'neutral'];
  if (run.phase === 'finished' && run.result !== 'SUCCEEDED') {
    label = `Bitti: ${run.result}`;
    tone = 'bad';
  } else if (run.phase === 'running' && run.recovering) {
    label = 'Kurtarma davranışı';
    tone = 'warn';
  }
  const badge = $('badge');
  badge.textContent = label;
  badge.className = 'badge ' + tone;

  const s = scenarioById(run.scenario);
  $('runScn').textContent = run.scenario
    ? `S${run.scenario}${s ? ' · ' + s.name : ''}${run.external ? ' · panel dışından' : ''}` : '';

  const count = run.goal_count || (s ? s.goals.length : 0);
  const results = run.goal_results || [];
  let line = 'Koşu yok';
  if (run.phase === 'starting') line = 'Gazebo, SLAM ve Nav2 açılıyor…';
  else if (going && run.goal) {
    const g = s && s.goals[run.goal - 1];
    line = `Hedef ${run.goal} / ${count}` + (g ? `  →  (${g[0]}, ${g[1]})` : '');
    if (run.external && run.recovering) line += ' · kurtarma';
  } else if (going) line = 'Nav2 bekleniyor…';
  else if (run.phase === 'finishing') line = 'Sonuçlar ve harita yazılıyor…';
  else if (run.phase === 'stopping') line = 'Süreçler kapatılıyor…';
  else if (run.phase === 'finished' && run.summary) {
    line = `${run.summary.goals_reached}/${run.summary.goal_count} hedef · ${run.result}`;
  } else if (run.phase === 'watched') {
    line = `${results.filter((r) => r === 'SUCCEEDED').length}/${count} hedef · ${run.result}`;
  } else if (run.phase === 'stopped') line = 'Koşu durduruldu';
  else if (run.phase === 'error') line = run.message || 'Hata';
  $('runGoal').textContent = line;

  const bars = $('bars');
  bars.textContent = '';
  if (run.phase !== 'idle') {
    for (let i = 0; i < count; i++) {
      const r = results[i];
      let cls = '';
      if (r === 'SUCCEEDED') cls = 'ok';
      else if (r) cls = 'bad';
      else if (going && run.goal === i + 1) cls = 'cur';
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
  renderElapsed(run);
}

function renderElapsed(run) {
  let text = '';
  if (run.started) {
    const end = run.ended ? run.ended * 1000 : Date.now();
    text = `Başladı ${clock(run.started * 1000)} · ${run.ended ? 'sürdü' : 'geçen'} ` +
      duration((end - run.started * 1000) / 1000);
  } else if (run.external) {
    text = `${run.run_id} · terminalden başlatılmış; panel yalnızca izliyor`;
  }
  $('runMsg').textContent = text;
}

// --- Canlı veri ----------------------------------------------------------------------------

// Canlı veri haritada ve metriklerde gösterilsin mi: seçili senaryonun verisi varsa ya da
// panelin koşusu yeni başladıysa (henüz veri yokken sis ve doğum noktası).
function liveShown() {
  const L = state.live;
  if (state.mode !== 'live' || L.scenario === null || L.scenario !== state.scenario) return false;
  return Boolean(L.status || L.pose || L.map) || ACTIVE.has(state.run.phase);
}

function applyLive(snap) {
  snap = snap || {};
  state.live = emptyLive();
  state.live.scenario = snap.status ? snap.status.scenario
    : state.run.scenario !== undefined ? state.run.scenario : null;
  state.live.trail = snap.trail || [];
  state.live.plan = snap.plan || [];
  state.live.pose = snap.pose || null;  // poseAt 0: yeni konum gelene kadar bayat sayılır
  if (snap.status) setStatus(snap.status);
  if (snap.map) setMap(snap.map);
  renderMetrics();
  redraw();
}

function setStatus(s) {
  let L = state.live;
  const fresh = !L.status || L.status.run_id !== s.run_id;
  if (L.status && fresh) {
    // Başka bir görev başladı (ör. terminalden): öncekinin haritası, izi ve konumu karışmasın
    // (sunucu da siler).
    state.live = L = emptyLive();
  }
  L.status = s;
  L.statusAt = Date.now();
  L.scenario = s.scenario;
  if (fresh && s.run_id !== state.run.run_id && !ACTIVE.has(state.run.phase)) {
    note(`Panel dışından başlatılmış görev izleniyor: S${s.scenario}, ${s.run_id}`);
    if (scenarioById(s.scenario) && s.scenario !== state.scenario) selectScenario(s.scenario, false);
  }
  renderRun();
  renderControls();
  renderMetrics();
  redraw();
}

function setPose(p) {
  state.live.pose = p;
  state.live.poseAt = Date.now();
  renderMetrics();
  redraw();
}

// Harita PNG'si (map_saver tonları) çözülüp temanın renkleriyle yeniden boyanır.
function setMap(m) {
  const L = state.live;
  L.pendingMap = m;
  const img = new Image();
  img.onload = () => {
    if (state.live !== L || L.pendingMap !== m) return;  // arada sıfırlandı ya da yenisi geldi
    L.map = m;
    L.mapImg = img;
    L.mapCanvas = recolorMap(img);
    redraw();
  };
  img.src = `data:image/png;base64,${m.png}`;
}

function recolorMap(img) {
  const c = document.createElement('canvas');
  c.width = img.naturalWidth;
  c.height = img.naturalHeight;
  const x = c.getContext('2d');
  x.drawImage(img, 0, 0);
  const d = x.getImageData(0, 0, c.width, c.height);
  const free = colorRgb(cssVar('--map-free'));
  const occupied = colorRgb(cssVar('--map-slam'));
  const p = d.data;
  for (let i = 0; i < p.length; i += 4) {
    // Tonlar: 0 dolu, 254 boş, 205 bilinmeyen (saydam kalır, altındaki sis görünür).
    const color = p[i] < 100 ? occupied : p[i] > 230 ? free : null;
    if (color) {
      p[i] = color[0];
      p[i + 1] = color[1];
      p[i + 2] = color[2];
      p[i + 3] = 255;
    } else {
      p[i + 3] = 0;
    }
  }
  x.putImageData(d, 0, 0);
  return c;
}

function setMetric(id, text, extra, cls) {
  const e = $(id);
  e.textContent = text;
  e.className = 'mv' + (cls ? ' ' + cls : '');
  if (extra) {
    e.appendChild(document.createTextNode(' '));
    e.appendChild(el('small', '', extra));
  }
}

function renderMetrics() {
  const shown = liveShown();
  const L = state.live;
  const st = shown ? L.status : null;
  const p = shown && L.pose && Date.now() - L.poseAt < POSE_STALE_MS ? L.pose : null;
  const started = Boolean(st && st.t0 !== null && st.t !== null);
  let t = started ? st.t : null;
  // Süre: durum 1 Hz gelir; robot giderken konumun zaman damgasıyla akıcı ilerler.
  if (started && p && MOVING.has(st.state)) t = Math.max(t, p.t - st.t0);
  setMetric('mTime', t === null ? '–' : `${t.toFixed(1)} s`);
  setMetric('mDist', started ? `${st.distance.toFixed(2)} m` : '–');
  const min = started && st.min_obstacle !== null ? `en az ${st.min_obstacle.toFixed(2)}` : '';
  setMetric('mNear', p && p.nearest !== null ? `${p.nearest.toFixed(2)} m` : '–', min);
  setMetric('mSpeed', p && p.v !== null ? `${p.v.toFixed(2)} m/s` : '–');
  setMetric('mClusters', p && p.clusters !== null ? String(p.clusters) : '–');
  setMetric('mRec', st ? String(st.recoveries) : '–', '', st && st.state === 'recovering' ? 'warn' : '');
}

// --- Harita --------------------------------------------------------------------------------

// Tuvali boyutlandırır (yalnızca değiştiyse), zemini boyar ve dünya -> piksel dönüşümünü kurar.
function setupCanvas(cv, bounds, bg) {
  const dpr = window.devicePixelRatio || 1;
  const w = cv.clientWidth;
  const h = cv.clientHeight;
  if (!w || !h) return null;
  const W = Math.round(w * dpr);
  const H = Math.round(h * dpr);
  if (cv.width !== W || cv.height !== H) {
    cv.width = W;
    cv.height = H;
  }
  const x = cv.getContext('2d');
  x.setTransform(dpr, 0, 0, dpr, 0, 0);
  x.fillStyle = bg;
  x.fillRect(0, 0, w, h);
  if (!bounds) return null;
  const pad = 0.25;
  const [x0, y0, x1, y1] = bounds;
  const k = Math.min(w / (x1 - x0 + 2 * pad), h / (y1 - y0 + 2 * pad));
  const ox = (w - (x1 - x0) * k) / 2 - x0 * k;
  const oy = (h + (y1 - y0) * k) / 2 + y0 * k;
  return {x, w, h, k, X: (v) => ox + v * k, Y: (v) => oy - v * k};
}

// Dünyayı, görevin hedeflerini ve robotu çizer. live verilirse (canlı önbellek) sis, SLAM
// haritası, iz, plan, hareketli kutuların konumu ve robotun gerçek konumu da çizilir.
function drawWorld(cv, geometry, scenario, detailed, live) {
  const T = setupCanvas(cv, geometry ? geometry.bounds : null,
    cssVar(detailed ? '--surface-1' : '--surface-2'));
  if (!T) return null;
  if (live) drawSlam(T, live);
  if (detailed) drawGrid(T, geometry.bounds);
  // SLAM haritası varken gerçek geometri yalnızca çizgiyle: aradaki fark görünsün.
  drawShapes(T, geometry, Boolean(live && live.mapCanvas), live && live.pose ? live.pose.movers : null);
  if (scenario && detailed) drawLanes(T, geometry, scenario);
  const st = live ? live.status : null;
  if (live) {
    const going = st && MOVING.has(st.state);
    const trail = live.trail.slice();
    if (going && live.pose && trail.length) trail.push([live.pose.x, live.pose.y]);
    drawLine(T, trail, cssVar('--map-path'), 2.5);
    if (going) drawLine(T, live.plan, cssVar('--map-robot'), 1.5, [5, 4]);
  }
  if (scenario) {
    drawGoals(T, scenario.goals, st ? st.goal_results : [], st ? st.goal_index : null, detailed);
  }
  const p = live ? live.pose : null;
  drawRobot(T, p ? [p.x, p.y, p.th] : [0, 0, null], detailed);
  return T;
}

function drawSlam(T, live) {
  const x = T.x;
  x.fillStyle = cssVar('--map-fog');  // haritanın dışı da bilinmiyor
  x.fillRect(0, 0, T.w, T.h);
  const m = live.map;
  if (!m || !live.mapCanvas) return;
  x.imageSmoothingEnabled = false;  // hücreler keskin kalsın
  x.drawImage(live.mapCanvas, T.X(m.ox), T.Y(m.oy + m.h * m.res), m.w * m.res * T.k,
    m.h * m.res * T.k);
  x.imageSmoothingEnabled = true;
}

function drawGrid(T, [x0, y0, x1, y1]) {
  const x = T.x;
  x.strokeStyle = cssVar('--border');
  x.lineWidth = 1;
  x.beginPath();
  for (let i = Math.ceil(x0); i <= Math.floor(x1); i++) {
    x.moveTo(T.X(i), T.Y(y0));
    x.lineTo(T.X(i), T.Y(y1));
  }
  for (let j = Math.ceil(y0); j <= Math.floor(y1); j++) {
    x.moveTo(T.X(x0), T.Y(j));
    x.lineTo(T.X(x1), T.Y(j));
  }
  x.stroke();
}

// outline: şekiller yalnızca çizgiyle. movers: hareketli kutuların canlı konumu {ad: [x, y, yaw]};
// konumu bilinen kutu her zaman dolu çizilir.
function drawShapes(T, geometry, outline, movers) {
  const x = T.x;
  const colors = {wall: cssVar('--map-wall'), obstacle: cssVar('--map-obstacle'),
    moving: cssVar('--map-moving')};
  geometry.shapes.forEach((sh) => {
    const color = colors[sh.role] || colors.obstacle;
    const pose = sh.role === 'moving' && movers ? movers[sh.model] : null;
    const [px, py, yaw] = pose || [sh.x, sh.y, sh.yaw];
    x.beginPath();
    if (sh.kind === 'cylinder') {
      x.arc(T.X(px), T.Y(py), sh.size * T.k, 0, 2 * Math.PI);
    } else {
      // Dönük dikdörtgenin köşeleri dünyada hesaplanır (tuval döndürülmez).
      const c = Math.cos(yaw);
      const s = Math.sin(yaw);
      const hx = sh.size[0] / 2;
      const hy = sh.size[1] / 2;
      [[-hx, -hy], [hx, -hy], [hx, hy], [-hx, hy]].forEach(([u, v], i) => {
        const X = T.X(px + u * c - v * s);
        const Y = T.Y(py + u * s + v * c);
        if (i) x.lineTo(X, Y);
        else x.moveTo(X, Y);
      });
      x.closePath();
    }
    if (outline && !pose) {
      x.strokeStyle = color;
      x.lineWidth = 1.25;
      x.stroke();
    } else {
      x.fillStyle = color;
      x.fill();
    }
  });
}

// Hareketli kutuların şeritleri: doğdukları yerden kendi +x eksenleri boyunca travel metre.
function drawLanes(T, geometry, scenario) {
  const x = T.x;
  x.save();
  x.strokeStyle = cssVar('--map-moving');
  x.setLineDash([4, 4]);
  x.lineWidth = 1.5;
  scenario.moving_obstacles.forEach((m) => {
    const sh = geometry.shapes.find((g) => g.model === m.name);
    if (!sh) return;
    x.beginPath();
    x.moveTo(T.X(sh.x), T.Y(sh.y));
    x.lineTo(T.X(sh.x + Math.cos(sh.yaw) * m.travel), T.Y(sh.y + Math.sin(sh.yaw) * m.travel));
    x.stroke();
  });
  x.restore();
}

function drawLine(T, pts, color, width, dash) {
  if (!pts || pts.length < 2) return;
  const x = T.x;
  x.save();
  x.strokeStyle = color;
  x.lineWidth = width;
  x.lineJoin = 'round';
  x.lineCap = 'round';
  x.setLineDash(dash || []);
  x.beginPath();
  pts.forEach((p, i) => {
    if (i) x.lineTo(T.X(p[0]), T.Y(p[1]));
    else x.moveTo(T.X(p[0]), T.Y(p[1]));
  });
  x.stroke();
  x.restore();
}

// Hedefler: varılan yeşil, başarısız kırmızı, yürütülen kalın; ayrıntılıda yön çizgisi ve numara.
function drawGoals(T, goals, results, current, detailed) {
  const x = T.x;
  const colors = {goal: cssVar('--map-goal'), ok: cssVar('--map-ok'), bad: cssVar('--map-bad')};
  x.font = `500 11px ${getComputedStyle(document.body).fontFamily}`;
  goals.forEach((g, i) => {
    const r = 0.25 * T.k;
    const cx = T.X(g[0]);
    const cy = T.Y(g[1]);
    const res = results[i];
    const color = res === 'SUCCEEDED' ? colors.ok : res ? colors.bad : colors.goal;
    x.strokeStyle = color;
    x.lineWidth = !detailed ? 1.5 : current === i + 1 ? 3 : 2;
    x.beginPath();
    x.arc(cx, cy, r, 0, 2 * Math.PI);
    x.stroke();
    if (detailed) {
      const a = g[2] * Math.PI / 180;
      x.beginPath();
      x.moveTo(cx, cy);
      x.lineTo(cx + Math.cos(a) * r, cy - Math.sin(a) * r);
      x.stroke();
      x.fillStyle = color;
      x.fillText(String(i + 1), cx + r + 2, cy - r + 4);
    }
  });
}

// Robot: th null ise yalnızca doğduğu nokta, değilse yönünü gösteren çizgiyle canlı konum.
function drawRobot(T, [px, py, th], detailed) {
  const x = T.x;
  const live = th !== null;
  const rr = Math.max(0.105 * T.k, detailed ? (live ? 6 : 5) : 3);
  x.fillStyle = cssVar('--map-robot');
  x.beginPath();
  x.arc(T.X(px), T.Y(py), rr, 0, 2 * Math.PI);
  x.fill();
  if (live) {
    x.save();
    x.strokeStyle = cssVar('--surface-2');
    x.lineWidth = 2;
    x.lineCap = 'round';
    x.beginPath();
    x.moveTo(T.X(px), T.Y(py));
    x.lineTo(T.X(px) + Math.cos(th) * rr, T.Y(py) - Math.sin(th) * rr);
    x.stroke();
    x.restore();
  }
}

function drawMap() {
  const cv = $('map');
  const s = scenarioById(state.scenario);
  const geometry = state.mode === 'live' && s ? state.worlds[baseName(s.world)] : null;
  const live = geometry && liveShown() ? state.live : null;
  const view = drawWorld(cv, geometry, s, true, live);
  if (!view) {
    const x = cv.getContext('2d');
    x.fillStyle = cssVar('--text-secondary');
    x.font = `14px ${getComputedStyle(document.body).fontFamily}`;
    x.textAlign = 'center';
    x.textBaseline = 'middle';
    x.fillText(MODES[state.mode].empty, cv.clientWidth / 2, cv.clientHeight / 2);
    x.textAlign = 'start';
    x.textBaseline = 'alphabetic';
  }
  renderMapInfo(view ? (live ? 'live' : 'preview') : 'none');
  const p = live && live.pose;
  $('coord').textContent = p
    ? `x ${p.x.toFixed(2)} · y ${p.y.toFixed(2)} · θ ${Math.round(p.th * 180 / Math.PI)}°` : '';
}

// Çizim isteklerini ekran tazelemesine toplar (konum 10 Hz gelir, harita en çok bir kez çizilir).
let drawQueued = false;
function redraw() {
  if (drawQueued) return;
  drawQueued = true;
  requestAnimationFrame(() => {
    drawQueued = false;
    drawMap();
  });
}

// Lejant ve açıklama haritanın türüne göre (önizleme / canlı); yalnızca tür değişince kurulur.
function renderMapInfo(key) {
  if (key === state.legendKey) return;
  state.legendKey = key;
  $('hint').textContent = key === 'live' ? LIVE_HINT : MODES[state.mode].hint;
  const box = $('legend');
  box.textContent = '';
  box.style.display = key === 'none' ? 'none' : '';
  (LEGENDS[key] || []).forEach(([kind, v, text]) => {
    const item = el('span');
    const sw = el('i');
    const color = `var(${v})`;
    if (kind === 'ring' || kind === 'outline') sw.style.border = `${kind === 'ring' ? 2 : 1.5}px solid ${color}`;
    else if (kind === 'dash') sw.style.borderTop = `2px dashed ${color}`;
    else sw.style.background = color;
    sw.style.borderRadius = kind === 'dot' || kind === 'ring' ? '50%' : '2px';
    if (kind === 'line' || kind === 'dash') {
      sw.style.width = '14px';
      sw.style.height = kind === 'line' ? '3px' : '0';
    }
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
  state.legendKey = '';  // açıklama yeni moda göre yeniden yazılsın
  store('panel.mode', mode);
  drawMap();
  renderMetrics();
  if (mode === 'live') drawPreviews();  // gizliyken çizilen önizlemeler boş kalır
}

function onTheme() {
  if (state.live.mapImg) state.live.mapCanvas = recolorMap(state.live.mapImg);
  drawMap();
  drawPreviews();
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
window.matchMedia('(prefers-color-scheme: dark)').addEventListener('change', onTheme);

// Bağlantı açık görünse de veri kesildiyse (asılı bağlantı) yeniden bağlan. Geçen süre, bayat
// konum ve panel dışı görevin izlenmesi saniyede bir yeniden değerlendirilir.
setInterval(() => {
  if (state.connected && Date.now() - state.lastSeen > STALE_MS) {
    setConnection(false);
    state.source.close();
    connect();
  }
  renderHealth();
  renderRun();
  renderControls();
  renderMetrics();
}, 1000);
setInterval(refreshHealth, HEALTH_MS);

renderRun();
renderControls();
renderMetrics();
const savedMode = restore('panel.mode');
setMode(MODES[savedMode] ? savedMode : 'live');
connect();
