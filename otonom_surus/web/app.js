// Otonom sürüş paneli istemcisi.
// P1: bağlantı göstergesi, mod seçici. P2: dünya kartları, görev seçimi, başlat/durdur, koşu kartı.
// P3: canlı harita (SLAM, robot, iz, Nav2 planı, hareketli kutular), görev durumu ve metrikler.
// P4: geçmiş koşular tablosu (koşu dizini). P5: tekrar oynatma: dünya ve en çok 6 koşu seçilir,
// hayalet robotlar kaydedilmiş yörüngelerinde birlikte oynar; zaman çizelgesi, koşu kartları, fark.
// P6: müdahale: haritaya tıkla-git (elle hedef, E1, E2…), Gazebo'ya kutu at / kaldır, serbest sürüş;
// tekrar oynatmada olay kaydı (kutular ve elle hedefler kendi anlarında).
// Sunucuyla iki yol: /api/stream (SSE, sunucu -> tarayıcı) ve /api/... (JSON istekleri).
'use strict';

const $ = (id) => document.getElementById(id);

const MODES = {
  live: {title: 'Harita', empty: 'Dünya seçilmedi',
    hint: 'Önizleme: seçili dünya, görevin hedefleri ve robotun doğduğu nokta. Başlatınca SLAM ' +
      'haritası, robot, izlenen yol ve Nav2 planı canlı çizilir.'},
  replay: {title: 'Tekrar oynatma', empty: 'Koşu seçilmedi',
    hint: 'Yukarıdan bir dünya ve en çok 6 koşu seç; Oynat ile robotlar kaydedilmiş ' +
      'yörüngelerinde birlikte yürür.'},
  edit: {title: 'Dünya tasarla', empty: 'Tasarlayıcı yakında',
    hint: 'Dünya tasarlayıcı P9’da gelecek.'},
};
const LIVE_HINT = 'Canlı: SLAM haritası (/map; sisli alan henüz görülmedi), robot (TF map → ' +
  'base_footprint), izlenen yol ve kesikli Nav2 planı. İnce çizgiler dünyanın gerçek geometrisi; ' +
  'SLAM haritasıyla arasındaki fark haritalama hatasıdır.';
const REPLAY_HINT = 'Kaydedilmiş yörüngeler (/odom, dünya koordinatı) zaman ekseninde oynar; ' +
  'robotların numaraları koşu kartlarıyla eşleşir. Zaman çizelgesinde soluk bant robotun ' +
  'hareketsiz kaldığı aralıklar (iki kayıt arası > 1.5 s), dikey çizgiler hedef sonuçları ' +
  '(kırmızı: başarısız, gri: iptal), alttaki işaretler panelin müdahaleleri (kare: kutu atıldı, ' +
  'boş kare: kaldırıldı, üçgen: elle hedef). Atılan kutular ve elle hedefler (E1, E2…) olay ' +
  'kaydındaki anlarında görünür. Hareketli kutuların konum kaydı yok: doğdukları yer ve şeritleri ' +
  'çiziliyor.';
// Araç çubuğundaki ipuçları kısa (tek satır); ayrıntı haritanın altındaki açıklamada (TOOLS_HINT).
const TOOL_HINTS = {
  goal: 'Tıkla: robot oraya gider; yön için tıklayıp sürükle.',
  box: 'Tıkla: 0.4 m kutu düşer; atılmış kutuya tıkla: kaldırılır.',
};
const TOOLS_HINT = ' Hedef ver: sürüklenmezse robot geldiği yöne bakarak durur; senaryo koşusunda ' +
  'görevin kalanı iptal edilir (önce onay istenir). Engel at: kutu statik, robota 0.25 m’den ' +
  'yakına ya da başka bir şeyin üstüne atılamaz. Kaldırılan kutunun SLAM haritasındaki izi uzun ' +
  'süre kalır; Nav2 orayı dolu görebilir.';
// Kaldırılan kutu SLAM haritasında dolu kalır: slam_toolbox eski taramaların isabetlerini tutar,
// robot dururken yeni tarama eklemez (P6'da ölçüldü: 28 s sonra 22 hücreden 20'si dolu, PT11).
const GHOST_NOTE = 'Kutu kaldırıldı; SLAM haritasındaki izi bir süre kalır.';
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
const READY = new Set(['running', 'recovering', 'idle']);  // görev elle hedef ve kutu kabul eder
const DRAG_MIN = 0.15;  // m; hedef verirken bundan kısa sürükleme yön sayılmaz
const BOX_SNAP = 0.05;  // m; atılan kutunun merkezi bu ızgaraya oturur (kapıyı tam kapatmak kolay olsun)
const STALE_MS = 10000;  // bu kadar süre veri gelmezse bağlantı kopmuş sayılır
const HEALTH_MS = 10000;  // sunucu bilgisinin yenilenme aralığı
const POSE_STALE_MS = 3000;  // bu kadar süre konum gelmezse anlık metrikler '–'
const EXTERNAL_STALE_MS = 5000;  // panel dışı görevin durumu bu kadar gelmezse izleme biter
const FRESH_MS = 120000;  // bu kadar süre önce yazılmış koşu tabloda "az önce" diye işaretlenir
const LOG_LINES = 80;
// Tekrar oynatma
const MAX_RUNS = 6;  // aynı anda oynayan koşu (renk paleti de 6: style.css --run-*)
const SPEEDS = [1, 2, 10, 30];
const HOLD_GAP = 1.5;  // s; iki kayıt arası bundan uzunsa robot yerinde durmuş (kayıt hareketle yazılır)
const HOLD_RING = 4;  // s; bundan uzun hareketsizlikte robotun çevresinde halka
const RESULT_HOLD = 6;  // s; hedef bitince koşu kartında sonucu bu kadar gösterilir
const STUCK_RADIUS = 0.5;  // m; takılma: robotun bu yarıçapın içinde kaldığı en uzun süre
const STUCK_MIN = 20;  // s; daha kısası takılma sayılmaz (temel koşularda hedefte dönüş ≤ 12 s)
const TL = {row: 22, top: 4, axis: 22, pad: 10};  // zaman çizelgesinin ölçüleri (px)
const LEGENDS = {
  preview: [['dot', '--map-robot', 'Robot (doğduğu yer)'], ['ring', '--map-goal', 'Hedef'],
    ['box', '--map-wall', 'Duvar'], ['box', '--map-obstacle', 'Engel'],
    ['box', '--map-moving', 'Hareketli kutu ve şeridi']],
  live: [['dot', '--map-robot', 'Robot'], ['line', '--map-path', 'İzlenen yol'],
    ['dash', '--map-robot', 'Nav2 planı'], ['ring', '--map-goal', 'Hedef'],
    ['ring', '--map-manual', 'Elle hedef (E1…)'], ['ring', '--map-ok', 'Varıldı'],
    ['ring', '--map-bad', 'Başarısız'], ['box', '--map-slam', 'SLAM: dolu'],
    ['box', '--map-fog', 'Henüz görülmedi'], ['outline', '--map-wall', 'Gerçek geometri'],
    ['box', '--map-moving', 'Hareketli kutu'], ['box', '--map-thrown', 'Atılan kutu']],
  replay: [['dot', '--run-1', 'Robot (numarası koşu kartında)'], ['line', '--run-1', 'İzlenen yol'],
    ['ring', '--text-muted', 'Başlangıç; hareketsiz > 4 s'], ['ring', '--map-goal', 'Hedef'],
    ['ring', '--map-manual', 'Elle hedef (E1…)'], ['box', '--map-wall', 'Duvar'],
    ['box', '--map-obstacle', 'Engel'], ['dash', '--map-moving', 'Hareketli kutunun şeridi'],
    ['box', '--map-thrown', 'Atılan kutu']],
};
// Tek koşu görünürken hedefler sonucuyla renklenir.
LEGENDS.replay1 = LEGENDS.replay.slice(0, 5).concat(
  [['ring', '--map-ok', 'Varıldı'], ['ring', '--map-bad', 'Başarısız']], LEGENDS.replay.slice(5));

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
  free: false,  // serbest sürüş seçili (hedefleri kullanıcı verir)
  live: emptyLive(),
  legendKey: '',
  // Harita araçları (canlı): seçili araç, son çizimin dünya <-> piksel dönüşümü, sürüklenen hedef
  // ({a, b}: dünya koordinatı), fare altındaki nokta, onay bekleyen hedef, yoldaki istek, ileti.
  tool: 'goal',
  mapT: null,
  drag: null,
  hover: null,
  pendingGoal: null,
  toolBusy: false,
  toolMsg: null,  // {text, bad, until}
  // Geçmiş koşular (/api/runs, en yeni önce). freshId: bu sekmede az önce biten koşu.
  runs: [],
  runsRoot: '',
  runsWarnings: [],
  runsError: '',
  runsLoaded: false,
  freshId: null,
  filter: {world: '', result: ''},
  runIndex: new Map(),  // kimlik -> koşu
  // Tekrar oynatma. sel: dünya -> seçili koşu kimlikleri; colors: kimlik -> renk ve numara sırası
  // (0–5; koşu seçimden çıkana kadar sabit); tracks: kimlik -> yörünge ve ondan türetilenler.
  replay: {world: null, sel: {}, colors: {}, hidden: {}, tracks: {}, errors: {}, pending: {},
    t: 0, playing: false, speed: 10, hoverT: null, dragging: false, labelW: 0, legacy: null},
};

// Canlı verinin önbelleği; yeni koşu başlarken boşaltılır. scenario: verinin ait olduğu senaryo.
// boxes: panelin attığı kutular ({name, x, y, size}).
function emptyLive() {
  return {scenario: null, status: null, statusAt: 0, pose: null, poseAt: 0, trail: [], plan: [],
    map: null, mapImg: null, mapCanvas: null, pendingMap: null, boxes: []};
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
    loadRuns();  // sunucu yeniden başlamış olabilir
  });
  on('ping', () => {});
  on('run', (run) => applyRun(run));
  on('log', (entry) => addLog(entry));
  // Biten görev listeye düşer: runs_changed /mission_status 'finished' gelince (panel dışı
  // görevde de), run_finished panelin koşusunun launch'ı kapanınca (harita da yazılmıştır).
  on('runs_changed', (d) => {
    if (d.id) state.freshId = d.id;
    loadRuns();
  });
  on('run_finished', (run) => {
    if (run.index_id) state.freshId = run.index_id;
    loadRuns();
  });
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
  on('obstacles', (d) => {
    state.live.boxes = d.boxes || [];
    renderTools();
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
    renderRuns();  // senaryo adları ve dünya kartlarının adları artık biliniyor
    renderReplayAll();
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
  const launch = state.free ? 'free.launch.py' : 'scenario.launch.py';
  return `ros2 launch otonom_surus ${launch} scenario:=${state.scenario}` +
    ` gui:=${$('gz').checked} rviz:=${$('rv').checked} results_dir:=${dir} run_id:=p<tarih-saat>` +
    ' source:=panel';
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
  // Serbest sürüş hedef alabilir durumdayken Durdur görevi bitirir (sonuçlar yazılır).
  const finish = state.run.mode === 'interactive' && state.run.phase === 'running';
  setText($('stopText'), finish ? 'Bitir' : 'Durdur');
  stop.title = finish ? 'Serbest sürüşü bitir: sonuçlar kaydedilir, simülasyon kapanır'
    : state.run.phase === 'finishing' ? 'Beklemeden durdur (sonuçlar yazılmamış olabilir)' : '';
  $('gz').disabled = active;
  $('rv').disabled = active;
  $('free').disabled = active;
  renderTools();
}

async function post(url, body, method = 'POST') {
  const r = await fetch(url, {
    method,
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
    const url = state.free ? '/api/free/start' : '/api/run/start';
    const run = await post(url, {scenario: state.scenario, gui: $('gz').checked, rviz: $('rv').checked});
    // SSE'deki 'run' olayı yanıttan önce gelmiş olabilir; eski yanıt yeni durumu ezmesin.
    if (run.run_id !== state.run.run_id) applyRun(run);
    note(`${run.mode === 'interactive' ? 'Serbest sürüş' : 'Koşu'} başlatıldı: S${run.scenario}, ` +
      run.run_id);
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
    goal_results: ext.goal_results, result: ext.result, recovering: ext.state === 'recovering',
    mode: ext.mode, idle: ext.state === 'idle'};
}

// Koşunun hedef kuyruğu [[x, y, yaw, kaynak], ...]: görevin durumundan (panelin hedefi kuyruğu
// değiştirir), yoksa senaryodan.
function liveGoals(run) {
  const st = state.live.status;
  if (st && st.run_id === run.run_id && Array.isArray(st.goals)) return st.goals;
  const s = scenarioById(run.scenario);
  return s ? s.goals : [];
}

// Hedef etiketleri: senaryonun hedefleri 1, 2…; panelden verilenler E1, E2… (elle).
function goalLabels(goals) {
  let n = 0;
  let e = 0;
  return goals.map((g) => (g[3] === 'panel' ? `E${++e}` : String(++n)));
}

function renderRun() {
  const run = runView();
  const going = run.phase === 'running' || run.phase === 'watching';
  const free = run.mode === 'interactive';
  let [label, tone] = PHASES[run.phase] || [run.phase, 'neutral'];
  if (run.phase === 'finished' && run.result !== 'SUCCEEDED') {
    label = `Bitti: ${run.result}`;
    tone = 'bad';
  } else if (run.phase === 'running' && run.recovering) {
    label = 'Kurtarma davranışı';
    tone = 'warn';
  } else if (run.phase === 'running' && free) {
    label = 'Serbest sürüş';
  }
  const badge = $('badge');
  badge.textContent = label;
  badge.className = 'badge ' + tone;

  const s = scenarioById(run.scenario);
  $('runScn').textContent = run.scenario ? `S${run.scenario}${s ? ' · ' + s.name : ''}` +
    `${free ? ' · serbest' : ''}${run.external ? ' · panel dışından' : ''}` : '';

  const goals = liveGoals(run);
  const labels = goalLabels(goals);
  const count = run.goal_count || goals.length;
  const results = run.goal_results || [];
  let line = 'Koşu yok';
  if (run.phase === 'starting') line = 'Gazebo, SLAM ve Nav2 açılıyor…';
  else if (going && run.idle) line = 'Hedef bekleniyor · haritaya tıkla';
  else if (going && run.goal) {
    const g = goals[run.goal - 1];
    const name = labels[run.goal - 1] || String(run.goal);
    const total = labels.filter((l) => !l.startsWith('E')).length;  // senaryonun hedefleri
    line = `Hedef ${name}` + (name.startsWith('E') ? '' : ` / ${total}`) +
      (g ? `  →  (${g[0]}, ${g[1]})` : '');
    if (run.external && run.recovering) line += ' · kurtarma';
  } else if (going) line = 'Nav2 bekleniyor…';
  else if (run.phase === 'finishing') line = 'Sonuçlar ve harita yazılıyor…';
  else if (run.phase === 'stopping') line = 'Süreçler kapatılıyor…';
  else if (run.phase === 'finished' && run.summary) {
    line = `${run.summary.goals_reached}/${run.summary.goal_count} hedef · ${run.result}`;
  } else if (run.phase === 'watched') {
    const counted = results.filter((r) => r !== 'CANCELED').length;
    line = `${results.filter((r) => r === 'SUCCEEDED').length}/${counted} hedef · ${run.result}`;
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
      else if (r === 'CANCELED') cls = 'skip';
      else if (r) cls = 'bad';
      else if (going && run.goal === i + 1) cls = 'cur';
      const bar = el('span', cls);
      bar.title = `Hedef ${labels[i] || i + 1}${r ? ': ' + r : ''}`;
      bars.appendChild(bar);
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
  state.live.boxes = snap.boxes || [];
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

// --- Harita araçları (müdahale) ------------------------------------------------------------
// Hedef ver: tıkla (ve yön için sürükle) -> /api/goal; senaryo koşusunda kalan görev iptal
// edileceğinden önce onay istenir. Engel at: tıkla -> /api/obstacle; atılmış kutuya tıklamak onu
// kaldırır. Denetimleri sunucu yapar (oda dışı, duvar, robota yakın…), iletisi araç çubuğunda.

// Araçlar yalnızca panelin kendi koşusunda, görev hedef alabilir durumdayken (Nav2 hazır) açık.
function toolsReady() {
  const st = state.live.status;
  return state.mode === 'live' && Boolean(state.connected) && state.run.phase === 'running' &&
    Boolean(st) && st.run_id === state.run.run_id && READY.has(st.state) && liveShown();
}

function setTool(tool) {
  state.tool = tool;
  store('panel.tool', tool);
  state.drag = null;
  closeConfirm();
  renderTools();
}

// Araç çubuğundaki geçici ileti (ör. sunucunun reddi); 6 s sonra araç ipucuna döner.
function toolNote(text, bad) {
  state.toolMsg = {text, bad: Boolean(bad), until: Date.now() + 6000};
  renderTools();
}

function renderTools() {
  const ready = toolsReady();
  document.querySelectorAll('#toolbar [data-tool]').forEach((b) => {
    const on = b.dataset.tool === state.tool;
    b.classList.toggle('on', on);
    b.setAttribute('aria-pressed', String(on));
    b.disabled = !ready;
  });
  const n = state.live.boxes.length;
  setText($('boxCount'), n ? ` (${n})` : '');
  $('clearBoxes').disabled = !ready || !n || state.toolBusy;
  $('map').classList.toggle('tool', ready);
  if (!ready) {
    state.drag = null;
    if (state.pendingGoal) closeConfirm();
  }
  const m = state.toolMsg;
  let text;
  let bad = false;
  if (m && Date.now() < m.until) ({text, bad} = m);
  else if (state.toolBusy) text = 'Gönderiliyor…';
  else if (ready) text = TOOL_HINTS[state.tool];
  else if (externalStatus()) text = 'Panel dışından başlatılan görev yalnızca izlenir.';
  else if (state.run.phase === 'starting' || state.run.phase === 'running') {
    text = 'Araçlar Nav2 hazır olunca açılır.';
  } else text = 'Araçlar panelden başlatılan koşuda açılır.';
  const msg = $('toolMsg');
  setText(msg, text);
  msg.title = text;  // tek satıra sığmazsa tamamı ipucunda
  msg.classList.toggle('bad', bad);
}

function toWorld(e) {
  const T = state.mapT;
  if (!T) return null;
  const r = $('map').getBoundingClientRect();
  return [T.wx(e.clientX - r.left), T.wy(e.clientY - r.top)];
}

function snap(v) {
  return Math.round(Math.round(v / BOX_SNAP) * BOX_SNAP * 100) / 100;
}

function boxAt(p) {
  return state.live.boxes.find((b) => Math.abs(p[0] - b.x) <= b.size[0] / 2 &&
    Math.abs(p[1] - b.y) <= b.size[1] / 2) || null;
}

// Hedefin yönü (rad): yeterince sürüklendiyse sürükleme yönü, değilse robottan hedefe geliş yönü.
function dragYaw(a, b) {
  if (Math.hypot(b[0] - a[0], b[1] - a[1]) >= DRAG_MIN) return Math.atan2(b[1] - a[1], b[0] - a[0]);
  const p = state.live.pose;
  return p ? Math.atan2(a[1] - p.y, a[0] - p.x) : 0;
}

function onMapDown(e) {
  if (e.button !== 0 || !toolsReady() || state.toolBusy) return;
  const p = toWorld(e);
  if (!p) return;
  e.preventDefault();
  if (state.tool === 'goal') {
    closeConfirm();
    state.drag = {a: p, b: p};
    $('map').setPointerCapture(e.pointerId);
  } else {
    const box = boxAt(p);
    if (box) removeBoxes(box.name);
    else throwBox(snap(p[0]), snap(p[1]));
  }
  redraw();
}

function onMapMove(e) {
  state.hover = toWorld(e);
  if (state.drag && state.hover) state.drag.b = state.hover;
  if (toolsReady()) redraw();
}

function onMapUp() {
  const d = state.drag;
  if (!d) return;
  state.drag = null;
  proposeGoal(d.a[0], d.a[1], dragYaw(d.a, d.b) * 180 / Math.PI);
}

// Senaryo koşusunda elle hedef görevin kalanını iptal eder: önce onay. Serbest sürüşte hemen gider.
function proposeGoal(x, y, yaw) {
  const g = {x: Math.round(x * 100) / 100, y: Math.round(y * 100) / 100, yaw: Math.round(yaw)};
  if (state.run.mode === 'interactive') {
    sendGoal(g);
    return;
  }
  const st = state.live.status;
  const left = st ? st.goal_count - st.goal_results.length : 0;  // yürütülen dahil
  state.pendingGoal = g;
  const text = `Kalan görev iptal edilecek (${left} hedef); robot ` +
    `(${g.x.toFixed(2)}, ${g.y.toFixed(2)}) noktasına gidecek.`;
  $('confirmText').textContent = text;
  $('confirmText').title = text;  // dar ekranda kısalırsa tamamı ipucunda
  // Onay çubuğu araç çubuğunun yerine geçer (aynı yükseklikte; dar ekranda o iki satır): harita
  // kaymaz.
  $('confirmBar').style.minHeight = `${$('toolbar').offsetHeight}px`;
  $('toolbar').style.display = 'none';
  $('confirmBar').style.display = '';
  $('confirmGo').focus();
  redraw();
}

function closeConfirm() {
  if (!state.pendingGoal && $('confirmBar').style.display === 'none') return;
  state.pendingGoal = null;
  $('confirmBar').style.display = 'none';
  $('toolbar').style.display = state.mode === 'live' ? '' : 'none';
  redraw();
}

// Araç isteği: yolda iken araçlar kilitli; ret iletisi araç çubuğunda ve olay akışında.
async function toolRequest(run) {
  state.toolBusy = true;
  state.toolMsg = null;
  renderTools();
  try {
    await run();
  } catch (e) {
    toolNote(e.message, true);
    note(e.message, 'WARN');
  } finally {
    state.toolBusy = false;
    renderTools();
    redraw();
  }
}

function sendGoal(g) {
  closeConfirm();
  return toolRequest(async () => {
    const r = await post('/api/goal', {x: g.x, y: g.y, yaw_deg: g.yaw});
    note(`Elle hedef gönderildi: (${r.x.toFixed(2)}, ${r.y.toFixed(2)}), yön ${Math.round(r.yaw_deg)}°`);
  });
}

function throwBox(x, y) {
  return toolRequest(async () => {
    const b = await post('/api/obstacle', {x, y});
    note(`Kutu atıldı: ${b.name} (${b.x.toFixed(2)}, ${b.y.toFixed(2)})`);
  });
}

function removeBoxes(name) {
  return toolRequest(async () => {
    const r = await post('/api/obstacle', name ? {name} : {}, 'DELETE');
    if (r.removed.length) {
      note(`Kutu kaldırıldı: ${r.removed.join(', ')}`);
      // toolRequest iletiyi silmesin diye istek bittikten sonra
      setTimeout(() => toolNote(GHOST_NOTE, false), 0);
    }
  });
}

// Atılan kutular: dolu, kendi renginde (Gazebo'dakiyle aynı ton).
function drawBoxes(T, boxes, color, outline) {
  const x = T.x;
  x.save();
  boxes.forEach((b) => {
    const [w, h] = b.size;
    const X = T.X(b.x - w / 2);
    const Y = T.Y(b.y + h / 2);
    x.fillStyle = color;
    x.fillRect(X, Y, w * T.k, h * T.k);
    if (outline) {
      x.strokeStyle = outline;
      x.lineWidth = 2;
      x.strokeRect(X, Y, w * T.k, h * T.k);
    }
  });
  x.restore();
}

// Hedef önizlemesi: halka ve yön oku.
function drawGoalArrow(T, [px, py], yaw, color) {
  const x = T.x;
  const r = 0.25 * T.k;
  const ax = T.X(px);
  const ay = T.Y(py);
  const ex = ax + Math.cos(yaw) * 0.55 * T.k;
  const ey = ay - Math.sin(yaw) * 0.55 * T.k;
  x.save();
  x.strokeStyle = color;
  x.fillStyle = color;
  x.lineWidth = 2;
  x.beginPath();
  x.arc(ax, ay, r, 0, 2 * Math.PI);
  x.moveTo(ax, ay);
  x.lineTo(ex, ey);
  x.stroke();
  x.beginPath();
  x.moveTo(ex, ey);
  x.lineTo(ex - Math.cos(yaw - 0.5) * 8, ey + Math.sin(yaw - 0.5) * 8);
  x.lineTo(ex - Math.cos(yaw + 0.5) * 8, ey + Math.sin(yaw + 0.5) * 8);
  x.closePath();
  x.fill();
  x.restore();
}

// Canlı haritanın üstünde aracın önizlemesi: sürüklenen ya da onay bekleyen hedef, fare altındaki
// kutu (atılmış kutunun üstündeyse kaldırılacağını gösteren çerçeve).
function drawToolOverlay(T) {
  if (!toolsReady()) return;
  const manual = cssVar('--map-manual');
  if (state.drag) {
    drawGoalArrow(T, state.drag.a, dragYaw(state.drag.a, state.drag.b), manual);
  } else if (state.pendingGoal) {
    const g = state.pendingGoal;
    drawGoalArrow(T, [g.x, g.y], g.yaw * Math.PI / 180, manual);
  } else if (state.hover && state.tool === 'box' && !state.toolBusy) {
    const hit = boxAt(state.hover);
    const x = T.x;
    if (hit) {
      drawBoxes(T, [hit], 'transparent', cssVar('--text-primary'));
    } else {
      x.save();
      x.globalAlpha = 0.45;
      drawBoxes(T, [{x: snap(state.hover[0]), y: snap(state.hover[1]), size: [0.4, 0.4]}],
        cssVar('--map-thrown'));
      x.restore();
    }
  }
}

// --- Geçmiş koşular ------------------------------------------------------------------------

let runsSeq = 0;  // üst üste gelen isteklerde yalnızca sonuncunun yanıtı uygulanır
async function loadRuns() {
  const seq = ++runsSeq;
  try {
    const d = await getJson('/api/runs');
    if (seq !== runsSeq) return;
    state.runs = d.runs;
    state.runsRoot = d.root;
    state.runsWarnings = d.warnings || [];
    state.runsError = '';
    state.runsLoaded = true;
  } catch (e) {
    if (seq !== runsSeq) return;
    state.runsError = `Koşu listesi yüklenemedi: ${e.message}`;
  }
  state.runIndex = new Map(state.runs.map((r) => [r.id, r]));
  renderRuns();
  replayRunsLoaded();
  redraw();
}

function isSuccess(run) {
  return run.result === 'SUCCEEDED';
}

function runName(run) {
  const s = scenarioById(run.scenario);
  return run.scenario_name || (s ? s.name : `Senaryo ${run.scenario}`);
}

// "Az önce": bu sekmede biten son koşu ya da son iki dakikada yazılmış koşu (sayfa yenilense de).
function isFresh(run) {
  const age = Date.now() - new Date(run.timestamp).getTime();
  return run.id === state.freshId || (age >= 0 && age < FRESH_MS);
}

function fixed(v, digits, unit) {
  return v === null || v === undefined ? '–' : `${v.toFixed(digits)}${unit}`;
}

function shortDate(ts) {
  const d = new Date(ts);
  if (Number.isNaN(d.getTime())) return '–';
  const now = new Date();
  const hm = d.toLocaleTimeString('tr-TR', {hour: '2-digit', minute: '2-digit', hour12: false});
  if (d.toDateString() === now.toDateString()) return `bugün ${hm}`;
  const opts = {day: '2-digit', month: '2-digit'};
  if (d.getFullYear() !== now.getFullYear()) opts.year = 'numeric';
  return `${d.toLocaleDateString('tr-TR', opts)} ${hm}`;
}

function worldKey(run) {
  return run.world || '?';
}

// Koşusu olan dünyalar: katalogdaki sırayla ve kart adlarıyla, sonra bilinmeyenler; koşu sayılarıyla.
function runWorlds() {
  const counts = new Map();
  state.runs.forEach((r) => counts.set(worldKey(r), (counts.get(worldKey(r)) || 0) + 1));
  const titles = new Map(worldGroups().map((g) => [g.file, g.title]));
  return [...new Set([...titles.keys(), ...counts.keys()])].filter((f) => counts.has(f)).map((f) => (
    {file: f, title: f === '?' ? 'Bilinmeyen dünya' : titles.get(f) || f, count: counts.get(f)}));
}

// Dünya süzgeci. Kaydedilmiş seçimin koşusu (henüz) yoksa "Hepsi" gösterilir ama seçim unutulmaz.
function renderWorldFilter() {
  const sel = $('fWorld');
  const worlds = runWorlds();
  sel.textContent = '';
  sel.appendChild(new Option(`Hepsi (${state.runs.length})`, ''));
  worlds.forEach((w) => sel.appendChild(new Option(`${w.title} (${w.count})`, w.file)));
  sel.value = worlds.some((w) => w.file === state.filter.world) ? state.filter.world : '';
}

function renderRuns() {
  renderWorldFilter();
  $('fResult').value = state.filter.result;
  const world = $('fWorld').value;
  const result = $('fResult').value;
  const rows = state.runs.filter((r) => (!world || worldKey(r) === world) &&
    (!result || (result === 'ok') === isSuccess(r)));
  const body = $('runRows');
  body.textContent = '';
  rows.forEach((run) => body.appendChild(runRow(run)));

  const ok = rows.filter(isSuccess).length;
  const total = rows.length !== state.runs.length ? ` · toplam ${state.runs.length}` : '';
  $('histCount').textContent = state.runsLoaded ? `${rows.length} koşu · ${ok} başarılı${total}` : '';
  $('histRoot').textContent = state.runsRoot;
  $('histRoot').title = 'Taranan klasör: kendisi ve bir alt düzeyi (runs.csv olanlar)';
  let msg = state.runsError || state.runsWarnings.join(' · ');
  if (!msg && !state.runsLoaded) msg = 'Yükleniyor…';
  else if (!msg && !state.runs.length) msg = `Koşu bulunamadı: ${state.runsRoot} ve alt klasörleri.`;
  else if (!msg && !rows.length) msg = 'Süzgece uyan koşu yok.';
  $('histMsg').textContent = msg;
}

function runRow(run) {
  const tr = el('tr');
  tr.tabIndex = 0;
  tr.dataset.id = run.id;
  tr.title = `${run.id} · tekrar oynatmada aç`;
  const date = el('td', 'c-date', shortDate(run.timestamp));
  date.title = run.timestamp || '';
  tr.appendChild(date);
  tr.appendChild(el('td', 'scn', `S${run.scenario}`));
  const task = el('td', 'task', runName(run));
  if (run.mode === 'interactive') task.appendChild(el('span', 'pill', 'serbest sürüş'));
  if (isFresh(run)) task.appendChild(el('span', 'pill', 'az önce'));
  if (!run.files.trajectory) task.appendChild(el('span', 'pill warn', 'yörünge yok'));
  tr.appendChild(task);
  tr.appendChild(el('td', 'c-var', run.label));
  // Sonuç rengin yanında metinle de: başarısızda sonuç adı yazılır.
  const reached = run.goal_count !== null ? `${run.goals_reached}/${run.goal_count}` : '';
  tr.appendChild(el('td', isSuccess(run) ? 'ok' : 'bad',
    isSuccess(run) ? reached : `${reached} ${run.result || 'sonuç yok'}`.trim()));
  tr.appendChild(el('td', 'num', fixed(run.duration_s, 1, ' s')));
  tr.appendChild(el('td', 'num c-path', fixed(run.path_length_m, 2, ' m')));
  tr.appendChild(el('td', 'num c-eff', fixed(run.path_efficiency, 2, '')));
  markRow(tr);
  tr.addEventListener('click', () => openRun(run.id));
  tr.addEventListener('keydown', (e) => {
    if (e.key === 'Enter' || e.key === ' ') {
      e.preventDefault();
      openRun(run.id);
    }
  });
  return tr;
}

// Satır, tekrar oynatmada seçili koşulardan biriyse işaretlenir.
function markRow(tr) {
  const on = isPicked(tr.dataset.id);
  tr.classList.toggle('on', on);
  tr.setAttribute('aria-selected', String(on));
}

// Geçmiş koşular tablosunda satıra tıklayınca koşu tekrar oynatmada açılır: dünyası seçilir, koşu
// o dünyanın seçimine eklenir (6 doluysa en eski çıkar); harita görünsün diye sayfa ona kayar.
function openRun(id) {
  const run = state.runIndex.get(id);
  if (!run) return;
  const R = state.replay;
  R.world = worldKey(run);
  const ids = pickedIds();
  if (!ids.includes(id)) {
    if (ids.length >= MAX_RUNS) dropRun(ids[0]);
    pickRun(id);
  }
  pause();
  R.t = 0;
  saveReplay();
  if (state.mode !== 'replay') setMode('replay');
  else renderReplayAll();
  document.querySelector('.mapcard').scrollIntoView({behavior: 'smooth', block: 'start'});
}

// --- Tekrar oynatma ------------------------------------------------------------------------
// Önce dünya seçilir (aynı dünyayı paylaşan senaryolar birlikte: S3, S4, S5), sonra o dünyanın en
// çok 6 koşusu; hepsi aynı zaman ekseninde oynar (t = sim zamanı − t0, her koşu kendi başından).
// Koşunun kimliği numarasıdır (1–6: robot etiketi, kart, zaman çizelgesi); renk ikincil ipucu,
// çünkü 6 renk tüm çiftlerde ayrışamıyor (style.css, --run-*).

function pickedIds() {
  return state.replay.sel[state.replay.world] || [];
}

// Seçili ve listede bulunan koşular, numara sırasıyla.
function pickedRuns() {
  const R = state.replay;
  return pickedIds().map((id) => state.runIndex.get(id)).filter(Boolean)
    .sort((a, b) => R.colors[a.id] - R.colors[b.id]);
}

function isPicked(id) {
  return Object.values(state.replay.sel).some((ids) => ids.includes(id));
}

// Seçime ekler; numara (ve renk) dünyanın seçiminde boştaki ilk sıra, koşu çıkana kadar değişmez.
function pickRun(id) {
  const R = state.replay;
  const ids = R.sel[R.world] || (R.sel[R.world] = []);
  const used = new Set(ids.map((k) => R.colors[k]));
  let slot = 0;
  while (used.has(slot)) slot++;
  R.colors[id] = slot;
  ids.push(id);
}

function dropRun(id) {
  const R = state.replay;
  R.sel[R.world] = pickedIds().filter((k) => k !== id);
  delete R.colors[id];
  delete R.hidden[id];
}

function toggleRun(id) {
  if (pickedIds().includes(id)) {
    dropRun(id);
  } else if (pickedIds().length >= MAX_RUNS) {
    syncPicker();  // işaretlenen kutu geri alınır
    $('pickMsg').textContent = `En çok ${MAX_RUNS} koşu seçilebilir; önce birini çıkar.`;
    return;
  } else {
    pickRun(id);
  }
  saveReplay();
  renderReplayAll(false);
}

function setReplayWorld(world) {
  const R = state.replay;
  if (R.world === world) return;
  R.world = world;
  pause();
  R.t = 0;
  saveReplay();
  renderReplayAll();
}

function saveReplay() {
  const R = state.replay;
  store('panel.replay.v2', JSON.stringify({world: R.world, sel: R.sel, colors: R.colors}));
}

function restoreReplay() {
  const R = state.replay;
  const raw = restore('panel.replay.v2');
  try {
    const saved = JSON.parse(raw || 'null');
    if (saved && typeof saved === 'object') {
      R.world = typeof saved.world === 'string' ? saved.world : null;
      Object.entries(saved.sel || {}).forEach(([world, ids]) => {
        if (Array.isArray(ids)) R.sel[world] = ids.filter((id) => typeof id === 'string');
      });
      Object.entries(saved.colors || {}).forEach(([id, slot]) => {
        if (Number.isInteger(slot) && slot >= 0 && slot < MAX_RUNS) R.colors[id] = slot;
      });
    }
  } catch (e) {
    // Bozuk kayıt: boş seçimle başlanır.
  }
  if (raw === null) R.legacy = restore('panel.replay');  // P4'ün tek koşusu seçime eklenir
  const speed = Number(restore('panel.replay.speed'));
  if (SPEEDS.includes(speed)) R.speed = speed;
}

// Koşu listesi geldi: seçim temizlenir (listede olmayanlar ve geçersiz numaralar), dünya seçilir.
function replayRunsLoaded() {
  const R = state.replay;
  if (!state.runsLoaded) {
    renderReplayAll();  // liste gelmedi: seçicide "Yükleniyor" ya da hata
    return;
  }
  Object.keys(R.sel).forEach((world) => {
    const seen = new Set();
    const ids = R.sel[world].filter((id) => state.runIndex.has(id) && !seen.has(id) &&
      seen.add(id)).slice(0, MAX_RUNS);
    R.sel[world] = [];
    const prev = R.world;
    R.world = world;
    ids.forEach((id) => {
      const slot = R.colors[id];
      const taken = R.sel[world].some((k) => R.colors[k] === slot);
      if (Number.isInteger(slot) && !taken) R.sel[world].push(id);
      else pickRun(id);
    });
    R.world = prev;
  });
  const legacy = R.legacy && state.runIndex.get(R.legacy);
  R.legacy = null;
  if (legacy) {
    R.world = worldKey(legacy);
    if (!isPicked(legacy.id)) pickRun(legacy.id);
  }
  if (!runWorlds().some((w) => w.file === R.world)) {
    R.world = state.runs.length ? worldKey(state.runs[0]) : null;
  }
  saveReplay();
  renderReplayAll();
}

// Seçim, dünya ya da koşu listesi değişti. rebuild: seçici tablosu baştan kurulsun mu (yalnızca
// seçim değiştiyse satırlar yerinde güncellenir, klavye odağı kaybolmaz).
function renderReplayAll(rebuild = true) {
  if (rebuild) renderPicker();
  else syncPicker();
  renderCards();
  renderDiff();
  ensureTracks();
  sizeTimeline();
  state.replay.t = Math.min(state.replay.t, timeMax());
  renderPlayButton();
  document.querySelectorAll('#runRows tr').forEach(markRow);
  renderFrame();
}

// --- Koşu seçici ---------------------------------------------------------------------------

function shortLabel(label) {
  // forward_prune_distance 2.0 -> fpd 2.0 (alt çizgili parametre adının baş harfleri)
  return label.replace(/\b[a-z]+(?:_[a-z]+)+\b/g, (w) => w.split('_').map((s) => s[0]).join(''));
}

function runTag(run) {
  const m = /^p(\d{4})(\d{2})(\d{2})-(\d{2})(\d{2})\d{2}$/.exec(run.run_id);
  return m ? `${m[3]}.${m[2]} ${m[4]}:${m[5]}` : run.run_id;
}

function shortName(run) {
  const free = run.mode === 'interactive' ? ' serbest' : '';
  return `S${run.scenario}${free} · ${shortLabel(run.label)} · ${runTag(run)}`;
}

function recordText(run) {
  const tags = [['events', 'olay'], ['obstacles', 'engel'], ['replay', 'ayrıntılı']]
    .filter(([k]) => run.files[k]).map(([, text]) => text);
  if (tags.length) return tags.join(' · ');
  return run.files.trajectory ? 'yörünge' : 'yok';
}

function numBadge(id) {
  const slot = state.replay.colors[id];
  const badge = el('span', 'rnum');
  const dot = el('i');
  dot.style.background = `var(--run-${slot + 1})`;
  badge.append(dot, String(slot + 1));
  return badge;
}

function renderPicker() {
  const R = state.replay;
  const chips = $('rpWorlds');
  chips.textContent = '';
  runWorlds().forEach((w) => {
    const chip = el('button', 'chip' + (w.file === R.world ? ' on' : ''));
    chip.type = 'button';
    chip.setAttribute('aria-pressed', String(w.file === R.world));
    chip.appendChild(document.createTextNode(w.title));
    chip.appendChild(el('span', 'tag', String(w.count)));
    chip.addEventListener('click', () => setReplayWorld(w.file));
    chips.appendChild(chip);
  });
  const body = $('pickRows');
  body.textContent = '';
  state.runs.filter((r) => worldKey(r) === R.world).forEach((run) => body.appendChild(pickRow(run)));
  syncPicker();
}

function pickRow(run) {
  const tr = el('tr');
  tr.dataset.id = run.id;
  tr.title = `${run.id} · ${runName(run)}`;
  const pick = el('td', 'c-pick');
  const box = el('input');
  box.type = 'checkbox';
  box.setAttribute('aria-label', `${shortName(run)} seç`);
  box.addEventListener('change', () => toggleRun(run.id));
  pick.append(box, el('span', 'rnum'));
  tr.appendChild(pick);
  const date = el('td', 'c-date', shortDate(run.timestamp));
  date.title = run.timestamp || '';
  tr.appendChild(date);
  const scn = el('td', 'scn', `S${run.scenario}`);
  if (run.mode === 'interactive') scn.appendChild(el('span', 'pill', 'serbest'));
  tr.appendChild(scn);
  tr.appendChild(el('td', 'pvar', run.label));  // seçicide dar ekranda da görünür
  const reached = run.goal_count !== null ? `${run.goals_reached}/${run.goal_count}` : '';
  tr.appendChild(el('td', isSuccess(run) ? 'ok' : 'bad',
    isSuccess(run) ? reached : `${reached} ${run.result || 'sonuç yok'}`.trim()));
  tr.appendChild(el('td', 'num', fixed(run.duration_s, 1, ' s')));
  tr.appendChild(el('td', 'num c-path', fixed(run.path_length_m, 2, ' m')));
  tr.appendChild(el('td', 'c-rec rec', recordText(run)));
  tr.addEventListener('click', (e) => {
    if (e.target !== box) toggleRun(run.id);
  });
  return tr;
}

// Seçici satırlarını seçimle eşler (yeniden kurmadan).
function syncPicker() {
  const R = state.replay;
  const ids = pickedRuns().map((r) => r.id);
  const full = ids.length >= MAX_RUNS;
  document.querySelectorAll('#pickRows tr').forEach((tr) => {
    const id = tr.dataset.id;
    const on = ids.includes(id);
    tr.querySelector('input').checked = on;
    tr.classList.toggle('on', on);
    tr.classList.toggle('full', full && !on);
    tr.setAttribute('aria-selected', String(on));
    const num = tr.querySelector('.rnum');
    num.replaceWith(on ? numBadge(id) : el('span', 'rnum'));
  });
  $('rpSelCount').textContent = `${ids.length} / ${MAX_RUNS} seçili`;
  $('rpClear').disabled = !ids.length;
  let msg = '';
  if (!state.runsLoaded) msg = state.runsError || 'Yükleniyor…';
  else if (!state.runs.length) msg = `Koşu bulunamadı: ${state.runsRoot}`;
  else if (full) msg = `En çok ${MAX_RUNS} koşu; başka koşu için önce birini çıkar.`;
  $('pickMsg').textContent = msg;
  if (R.world === null) $('pickMsg').textContent = msg || 'Dünya seçilmedi';
}

// --- Yörüngeler ----------------------------------------------------------------------------

function runPath(id) {
  return id.split('/').map(encodeURIComponent).join('/');
}

function ensureTracks() {
  pickedRuns().forEach(loadTrack);
}

async function loadTrack(run) {
  const R = state.replay;
  if (R.tracks[run.id] || R.errors[run.id] || R.pending[run.id]) return;
  if (!run.files.trajectory) {
    R.errors[run.id] = 'yörünge kaydı yok';
    return;
  }
  R.pending[run.id] = true;
  try {
    // Olay kaydı yoksa ya da okunamazsa koşu olaysız oynar (yörünge yeter).
    const [traj, events] = await Promise.all([
      getJson(`/api/runs/${runPath(run.id)}/trajectory`),
      run.files.events ? getJson(`/api/runs/${runPath(run.id)}/events`).then((d) => d.events)
        .catch(() => []) : [],
    ]);
    R.tracks[run.id] = buildTrack(run, traj.pts, events);
  } catch (e) {
    R.errors[run.id] = e.message;
  } finally {
    delete R.pending[run.id];
  }
  if (pickedIds().includes(run.id)) renderReplayAll(false);
}

// Olay kaydından: atılan kutular (görünür oldukları aralıkla), elle hedeflerin verildiği an
// (sırası -> t) ve zaman çizelgesindeki işaretler.
function trackEvents(events) {
  const boxes = new Map();
  const given = new Map();
  const marks = [];
  events.forEach((e) => {
    if (e.type === 'spawn' && typeof e.x === 'number' && typeof e.y === 'number') {
      const size = Array.isArray(e.size) && e.size.length >= 2 ? e.size : [0.4, 0.4];
      boxes.set(e.name, {name: e.name, x: e.x, y: e.y, size, from: e.t, to: Infinity});
      marks.push({t: e.t, kind: 'spawn'});
    } else if (e.type === 'delete' && boxes.has(e.name)) {
      boxes.get(e.name).to = e.t;
      marks.push({t: e.t, kind: 'delete'});
    } else if (e.type === 'goal' && Number.isInteger(e.index)) {
      given.set(e.index, e.t);
      marks.push({t: e.t, kind: 'goal'});
    }
  });
  return {boxes: [...boxes.values()], given, marks};
}

// Yörüngeden ve olay kaydından türetilenler: birikimli yol; hedeflerin başlangıç ve bitiş anları
// (elle hedef olay anında başlar, senaryonunki öncekinin bitişinde: hedefler ardışık gönderilir;
// bitiş başlangıç + goals.csv'deki süre); hedef etiketleri (1, 2… ve elle E1, E2…); atılan kutular;
// hareket aralıkları (iki kayıt arası ≤ HOLD_GAP), en uzun takılma ve duraklama. end: zaman
// ekseninin sonu (robot sonda dururken kayıt yazılmaz).
function buildTrack(run, pts, events) {
  const cum = [0];
  for (let i = 1; i < pts.length; i++) {
    cum.push(cum[i - 1] + Math.hypot(pts[i][1] - pts[i - 1][1], pts[i][2] - pts[i - 1][2]));
  }
  const ev = trackEvents(events || []);
  const goals = run.goals.slice().sort((a, b) => a.index - b.index);
  const manual = goals.map((g) => ev.given.has(g.index));
  const starts = [];
  let prev = 0;
  const ends = goals.map((g, k) => {
    const start = manual[k] ? ev.given.get(g.index) : prev;
    starts.push(start);
    prev = start + (g.duration_s || 0);
    return prev;
  });
  const labels = goalLabels(goals.map((g, k) => [g.x, g.y, g.yaw, manual[k] ? 'panel' : '']));
  const last = pts.length ? pts[pts.length - 1][0] : 0;
  const duration = run.duration_s !== null ? run.duration_s : last;
  const end = Math.max(duration, last);
  const moving = [];
  for (let i = 1; i < pts.length; i++) {
    const a = pts[i - 1][0];
    const b = pts[i][0];
    if (b - a > HOLD_GAP) continue;
    const m = moving[moving.length - 1];
    if (m && m[1] >= a) m[1] = b;
    else moving.push([a, b]);
  }
  return {run, pts, cum, goals, manual, labels, starts, ends, duration, end, moving,
    boxes: ev.boxes, marks: ev.marks,
    results: goals.map((g) => g.result), recov: goals.map((g) => g.recoveries || 0),
    stuck: longestDwell(pts, end), pause: longestPause(pts, end)};
}

// Takılma: robotun STUCK_RADIUS içinden çıkmadan kaldığı en uzun süre ve o noktaların ortası.
// Dönerek, geri geri giderek kıpırdayan ama ilerleyemeyen robotu da yakalar.
function longestDwell(pts, end) {
  let best = {d: 0, x: 0, y: 0};
  const r2 = STUCK_RADIUS * STUCK_RADIUS;
  for (let i = 0; i < pts.length; i++) {
    const [ti, xi, yi] = pts[i];
    if (end - ti <= best.d) break;  // kalan süre en iyiyi geçemez
    let j = i + 1;
    let sx = xi;
    let sy = yi;
    while (j < pts.length && (pts[j][1] - xi) ** 2 + (pts[j][2] - yi) ** 2 <= r2) {
      sx += pts[j][1];
      sy += pts[j][2];
      j++;
    }
    const d = (j < pts.length ? pts[j][0] : end) - ti;
    if (d > best.d) best = {d, x: sx / (j - i), y: sy / (j - i)};
  }
  return best;
}

// Duraklama: ardışık iki kayıt arasındaki (ve son kayıttan sona kadarki) en uzun süre.
function longestPause(pts, end) {
  let best = {d: 0, x: 0, y: 0};
  pts.forEach((p, i) => {
    const d = (i + 1 < pts.length ? pts[i + 1][0] : end) - p[0];
    if (d > best.d) best = {d, x: p[1], y: p[2]};
  });
  return best;
}

// Koşunun t anındaki hali: konum (ara değerli), yön, hareketsizlik süresi, o ana kadarki yol.
function sample(tr, t) {
  const p = tr.pts;
  const n = p.length;
  if (!n) return {x: 0, y: 0, th: null, hold: 0, dist: 0, i: 0};
  let lo = 0;
  let hi = n - 1;
  while (lo < hi) {
    const mid = (lo + hi + 1) >> 1;
    if (p[mid][0] <= t) lo = mid;
    else hi = mid - 1;
  }
  const i = lo;
  const a = p[i];
  const b = p[i + 1];
  if (!b || b[0] - a[0] > HOLD_GAP) {
    return {x: a[1], y: a[2], th: heading(p, i), hold: Math.max(0, t - a[0]), dist: tr.cum[i], i};
  }
  const gap = b[0] - a[0];
  const u = gap > 0 ? Math.min(1, Math.max(0, (t - a[0]) / gap)) : 1;
  return {x: a[1] + (b[1] - a[1]) * u, y: a[2] + (b[2] - a[2]) * u, th: heading(p, i + 1), hold: 0,
    dist: tr.cum[i] + (tr.cum[i + 1] - tr.cum[i]) * u, i};
}

// Hareket yönü: son anlamlı hareketten (2 cm'den kısa adımlar atlanır); hareket yoksa null.
function heading(pts, i) {
  const [, x, y] = pts[i];
  for (let j = i - 1; j >= 0; j--) {
    const dx = x - pts[j][1];
    const dy = y - pts[j][2];
    if (dx * dx + dy * dy >= 0.02 * 0.02) return Math.atan2(dy, dx);
  }
  return null;
}

// Koşu kartındaki anlık durum: [metin, sınıf]. Sıra taslaktaki gibi: bitti, az önce biten hedefin
// sonucu (iptal edilen gösterilmez: yerine gelen hedef yürütülüyor), uzun hareketsizlik, yürütülen
// hedef. İptal edilen hedefler (CANCELED) hedef sayısına girmez (goal_count da öyle).
function runStatus(tr, s, t) {
  const n = tr.run.goal_count !== null ? tr.run.goal_count
    : tr.results.filter((r) => r !== 'CANCELED').length;
  if (t >= tr.duration) {
    const ok = tr.results.filter((r) => r === 'SUCCEEDED').length;
    return [`Bitti · ${ok}/${n} hedef`, ok === n ? 'ok' : 'bad'];
  }
  for (let k = 0; k < tr.ends.length; k++) {
    if (tr.ends[k] <= t && t - tr.ends[k] < RESULT_HOLD && tr.results[k] !== 'CANCELED') {
      const r = tr.results[k] || 'sonuç yok';
      const rec = tr.recov[k] ? ` · ${tr.recov[k]} kurtarma` : '';
      return [`Hedef ${tr.labels[k]}: ${r}${rec}`, r === 'SUCCEEDED' ? 'ok' : 'bad'];
    }
  }
  if (s.hold > HOLD_RING) return [`Hareketsiz · ${Math.round(s.hold)} s`, 'warn'];
  const gi = tr.ends.findIndex((e, k) => tr.starts[k] <= t && t < e);
  if (gi >= 0) return [`Hedef ${tr.labels[gi]} yolunda`, ''];
  return [tr.run.mode === 'interactive' ? 'Hedef bekleniyor' : 'Yolda', ''];
}

// --- Koşu kartları ve fark ------------------------------------------------------------------

const SVG_NS = 'http://www.w3.org/2000/svg';
const ICONS = {
  eye: ['M2 12s3.5-7 10-7 10 7 10 7-3.5 7-10 7S2 12 2 12z', 'M12 9a3 3 0 1 0 0 6 3 3 0 0 0 0-6z'],
  eyeOff: ['M3 3l18 18', 'M10.6 5.1A10 10 0 0 1 12 5c6.5 0 10 7 10 7a17 17 0 0 1-3 4',
    'M6.6 6.6C3.9 8.4 2 12 2 12s3.5 7 10 7a10 10 0 0 0 5.4-1.6', 'M9.9 9.9a3 3 0 0 0 4.2 4.2'],
  close: ['M6 6l12 12', 'M18 6L6 18'],
  play: ['M8 5v14l11-7z'],
  pause: ['M8 5v14', 'M16 5v14'],
};

function icon(name, svg) {
  svg = svg || document.createElementNS(SVG_NS, 'svg');
  svg.setAttribute('class', 'icon');
  svg.setAttribute('viewBox', '0 0 24 24');
  svg.setAttribute('aria-hidden', 'true');
  svg.textContent = '';
  ICONS[name].forEach((d) => {
    const path = document.createElementNS(SVG_NS, 'path');
    path.setAttribute('d', d);
    svg.appendChild(path);
  });
  return svg;
}

function iconButton(name, label) {
  const b = el('button', 'iconbtn');
  b.type = 'button';
  b.setAttribute('aria-label', label);
  b.title = label;
  b.appendChild(icon(name));
  return b;
}

function renderCards() {
  const R = state.replay;
  const box = $('rpCards');
  box.textContent = '';
  const runs = pickedRuns();
  runs.forEach((run) => {
    const card = el('div', 'rcard' + (R.hidden[run.id] ? ' off' : ''));
    card.dataset.id = run.id;
    const top = el('div', 'rtop');
    const name = el('span', 'rname', shortName(run));
    name.title = `${run.id} · ${runName(run)}` +
      (run.timestamp ? ` · ${run.timestamp.replace('T', ' ')}` : '');
    const eye = iconButton(R.hidden[run.id] ? 'eyeOff' : 'eye', R.hidden[run.id] ? 'Göster' : 'Gizle');
    eye.addEventListener('click', () => {
      R.hidden[run.id] = !R.hidden[run.id];
      const off = R.hidden[run.id];
      card.classList.toggle('off', off);
      icon(off ? 'eyeOff' : 'eye', eye.firstChild);
      eye.setAttribute('aria-label', off ? 'Göster' : 'Gizle');
      eye.title = off ? 'Göster' : 'Gizle';
      renderFrame();
    });
    const remove = iconButton('close', 'Seçimden çıkar');
    remove.addEventListener('click', () => toggleRun(run.id));
    top.append(numBadge(run.id), name, eye, remove);
    card.append(top, el('p', 'rstat', '–'), el('p', 'rmet', ''));
    box.appendChild(card);
  });
  $('rpEmpty').style.display = runs.length ? 'none' : '';
  $('rpCount').textContent = runs.length ? `${runs.length} / ${MAX_RUNS}` : '';
  updateCards();
}

function setText(e, text) {
  if (e.textContent !== text) e.textContent = text;
}

// Kartlardaki anlık durum ve o ana kadarki yol ile kurtarma (oynarken her karede).
function updateCards() {
  const R = state.replay;
  document.querySelectorAll('#rpCards .rcard').forEach((card) => {
    const id = card.dataset.id;
    const tr = R.tracks[id];
    const [stat, met] = [card.children[1], card.children[2]];
    let text = R.errors[id] || 'yörünge yükleniyor…';
    let cls = '';
    let metric = '';
    if (tr) {
      const s = sample(tr, R.t);
      [text, cls] = runStatus(tr, s, R.t);
      const rec = tr.ends.reduce((acc, e, k) => acc + (e <= R.t ? tr.recov[k] : 0), 0);
      metric = `yol ${s.dist.toFixed(2)} m · kurtarma ${rec}`;
    }
    setText(stat, text);
    const name = 'rstat' + (cls ? ' ' + cls : '');
    if (stat.className !== name) stat.className = name;
    setText(met, metric);
  });
}

// a − b işaretiyle; değerlerden biri yoksa '–'.
function delta(a, b, digits) {
  if (a === null || b === null) return '–';
  const v = a - b;
  const sign = v > 0 ? '+' : v < 0 ? '−' : '±';
  return `${sign}${Math.abs(v).toFixed(digits)}`;
}

// Fark kutusu: veriden üretilir. Her koşu için en uzun takılma (yeri ve süresi) ya da duraklama,
// başarısız hedefler; iki ve daha çok koşuda süre ve yol farkı (ilk numaraya göre).
function renderDiff() {
  const R = state.replay;
  const list = $('rpDiff');
  list.textContent = '';
  const runs = pickedRuns();
  $('diffCard').style.display = state.mode === 'replay' && runs.length ? '' : 'none';
  const tracks = runs.map((r) => R.tracks[r.id]).filter(Boolean);
  const item = (...parts) => {
    const li = el('li');
    parts.forEach((p) => li.append(typeof p === 'string' ? p : numBadge(p.id)));
    list.appendChild(li);
  };
  if (runs.length && !tracks.length) item('Yörüngeler yükleniyor…');
  tracks.forEach((tr) => {
    const {stuck, pause} = tr;
    let text = ` takılmadan ilerledi; en uzun duraklama ${Math.round(pause.d)} s`;
    if (stuck.d >= STUCK_MIN) {
      text = ` (${stuck.x.toFixed(1)}, ${stuck.y.toFixed(1)}) yakınında ${Math.round(stuck.d)} s takıldı`;
    } else if (pause.d <= HOLD_GAP) {
      text = ' takılmadan, duraksamadan ilerledi';
    }
    const failed = [];
    tr.goals.forEach((g, k) => {
      if (g.result !== 'SUCCEEDED' && g.result !== 'CANCELED') {
        failed.push(`hedef ${tr.labels[k]} ${g.result}` +
          (g.recoveries ? ` (${g.recoveries} kurtarma)` : ''));
      }
    });
    if (failed.length) text += `; ${failed.join(', ')}`;
    // Panelin müdahaleleri (olay kaydından).
    const given = tr.manual.filter(Boolean).length;
    const canceled = tr.results.filter((r) => r === 'CANCELED').length;
    const parts = [];
    if (tr.boxes.length) parts.push(`${tr.boxes.length} kutu atıldı`);
    if (given) parts.push(`${given} elle hedef`);
    if (canceled) parts.push(`${canceled} hedef iptal`);
    if (parts.length) text += `; ${parts.join(', ')}`;
    item({id: tr.run.id}, text);
  });
  if (tracks.length >= 2) {
    const ref = tracks[0].run;
    const parts = ['Süre ve yol farkı, ', {id: ref.id}, ' numaraya göre:'];
    tracks.slice(1).forEach((tr, k) => {
      const r = tr.run;
      parts.push(k ? ' · ' : ' ', {id: r.id}, ` ${delta(r.duration_s, ref.duration_s, 1)} s, ` +
        `${delta(r.path_length_m, ref.path_length_m, 2)} m`);
    });
    item(...parts);
  } else if (tracks.length === 1) {
    const r = tracks[0].run;
    item(`Süre ${fixed(r.duration_s, 1, ' s')}, yol ${fixed(r.path_length_m, 2, ' m')}, verim ` +
      `${fixed(r.path_efficiency, 2, '')}, en yakın engel ${fixed(r.min_obstacle_dist_m, 2, ' m')}, ` +
      `${r.recoveries === null ? '–' : r.recoveries} kurtarma`);
  }
}

// --- Oynatıcı ------------------------------------------------------------------------------

// Zaman ekseninin sonu: seçili koşuların en uzunu, 10 s'ye yuvarlanmış.
function timeMax() {
  const R = state.replay;
  let m = 0;
  pickedRuns().forEach((r) => {
    const tr = R.tracks[r.id];
    m = Math.max(m, tr ? tr.end : r.duration_s || 0);
  });
  return Math.max(10, Math.ceil(m / 10) * 10);
}

let playRaf = 0;
let playLast = 0;

function play() {
  const R = state.replay;
  if (!pickedRuns().length) return;
  if (R.t >= timeMax()) R.t = 0;
  R.playing = true;
  playLast = performance.now();
  if (!playRaf) playRaf = requestAnimationFrame(playTick);
  renderPlayButton();
}

function pause() {
  state.replay.playing = false;
  if (playRaf) cancelAnimationFrame(playRaf);
  playRaf = 0;
  renderPlayButton();
}

function playTick(now) {
  playRaf = 0;
  const R = state.replay;
  if (!R.playing) return;
  const dt = Math.min((now - playLast) / 1000, 0.1);  // sekme arka plandayken sıçramasın
  playLast = now;
  const tmax = timeMax();
  R.t = Math.min(tmax, R.t + dt * R.speed);
  if (R.t >= tmax) pause();
  renderFrame();
  if (R.playing) playRaf = requestAnimationFrame(playTick);
}

function seek(t) {
  state.replay.t = Math.max(0, Math.min(timeMax(), t));
  renderFrame();
}

function setSpeed(s) {
  state.replay.speed = s;
  store('panel.replay.speed', String(s));
  renderSpeeds();
}

function renderSpeeds() {
  const box = $('rpSpeeds');
  if (!box.childElementCount) {
    SPEEDS.forEach((s) => {
      const chip = el('button', 'chip', `${s}×`);
      chip.type = 'button';
      chip.dataset.s = String(s);
      chip.addEventListener('click', () => setSpeed(s));
      box.appendChild(chip);
    });
  }
  box.querySelectorAll('button').forEach((b) => {
    const on = Number(b.dataset.s) === state.replay.speed;
    b.classList.toggle('on', on);
    b.setAttribute('aria-pressed', String(on));
  });
}

function renderPlayButton() {
  const playing = state.replay.playing;
  setText($('rpPlayText'), playing ? 'Duraklat' : 'Oynat');
  icon(playing ? 'pause' : 'play', $('rpPlayIcon'));
  const none = !pickedRuns().length;
  $('rpPlay').disabled = none;
  $('rpBack').disabled = none;
}

// Bir kare: harita, zaman çizelgesi, kartlar ve süre (yalnızca tekrar oynatma modunda).
function renderFrame() {
  if (state.mode !== 'replay') return;
  drawMap();
  drawTimeline();
  updateCards();
  setText($('rpTime'), `${state.replay.t.toFixed(1)} s / ${timeMax()} s`);
}

// --- Zaman çizelgesi -----------------------------------------------------------------------

// Yükseklik koşu sayısına göre; etiket sütununun genişliği en uzun kısa addan.
function sizeTimeline() {
  const runs = pickedRuns();
  const cv = $('tl');
  cv.style.height = `${TL.top + Math.max(1, runs.length) * TL.row + TL.axis}px`;
  const x = cv.getContext('2d');
  x.font = `12px ${getComputedStyle(document.body).fontFamily}`;
  state.replay.labelW = Math.max(0, ...runs.map((r) => x.measureText(shortName(r)).width));
}

function tlGeometry(W) {
  const L0 = Math.round(Math.min(state.replay.labelW + 34, W * 0.4));
  return {L0, R0: W - TL.pad};
}

function fitText(x, text, X, Y, maxW) {
  if (maxW <= 8) return;
  let s = text;
  if (x.measureText(s).width > maxW) {
    while (s.length > 1 && x.measureText(s + '…').width > maxW) s = s.slice(0, -1);
    s += '…';
  }
  x.fillText(s, X, Y);
}

function drawTimeline() {
  const cv = $('tl');
  const W = cv.clientWidth;
  const H = cv.clientHeight;
  if (!W || !H) return;
  const dpr = window.devicePixelRatio || 1;
  if (cv.width !== Math.round(W * dpr) || cv.height !== Math.round(H * dpr)) {
    cv.width = Math.round(W * dpr);
    cv.height = Math.round(H * dpr);
  }
  const x = cv.getContext('2d');
  x.setTransform(dpr, 0, 0, dpr, 0, 0);
  x.clearRect(0, 0, W, H);
  const R = state.replay;
  const runs = pickedRuns();
  const font = getComputedStyle(document.body).fontFamily;
  const ink = {primary: cssVar('--text-primary'), secondary: cssVar('--text-secondary'),
    muted: cssVar('--text-muted'), bad: cssVar('--map-bad'), manual: cssVar('--map-manual'),
    thrown: cssVar('--map-thrown')};
  x.font = `12px ${font}`;
  x.textBaseline = 'middle';
  if (!runs.length) {
    x.fillStyle = ink.secondary;
    x.fillText('Koşu seçilmedi', 4, H / 2);
    return;
  }
  const tmax = timeMax();
  const g = tlGeometry(W);
  const sx = (t) => g.L0 + (g.R0 - g.L0) * t / tmax;
  runs.forEach((run, r) => {
    const y = TL.top + r * TL.row;
    const cy = y + TL.row / 2;
    const tr = R.tracks[run.id];
    const color = cssVar(`--run-${R.colors[run.id] + 1}`);
    const base = R.hidden[run.id] ? 0.35 : 1;
    x.globalAlpha = base;
    // Etiket: renk noktası, numara ve kısa ad; metin metin renginde, renk yalnızca noktada.
    x.fillStyle = color;
    x.beginPath();
    x.arc(6, cy, 4.5, 0, 2 * Math.PI);
    x.fill();
    x.fillStyle = ink.primary;
    x.fillText(String(R.colors[run.id] + 1), 14, cy);
    x.fillStyle = ink.secondary;
    fitText(x, shortName(run), 26, cy, g.L0 - 32);
    if (tr) {
      x.fillStyle = color;
      x.globalAlpha = base * 0.22;  // koşu süresi boyunca soluk bant: hareketsiz aralıklar
      x.fillRect(sx(0), y + 6, sx(tr.duration) - sx(0), TL.row - 12);
      x.globalAlpha = base;
      tr.moving.forEach(([a, b]) => x.fillRect(sx(a), y + 6, Math.max(1, sx(b) - sx(a)), TL.row - 12));
      tr.ends.forEach((e, k) => {
        const r = tr.results[k];
        x.fillStyle = r === 'SUCCEEDED' ? ink.primary : r === 'CANCELED' ? ink.muted : ink.bad;
        x.fillRect(sx(e) - 1, y + 2, 2, TL.row - 4);
      });
      // Panelin müdahaleleri bandın altında: kutu atıldı (dolu kare), kaldırıldı (boş kare),
      // elle hedef (üçgen).
      tr.marks.forEach((m) => {
        const X = sx(Math.max(0, m.t));
        const Y = y + TL.row - 4;
        if (m.kind === 'goal') {
          x.fillStyle = ink.manual;
          x.beginPath();
          x.moveTo(X, Y - 5);
          x.lineTo(X + 4, Y + 2);
          x.lineTo(X - 4, Y + 2);
          x.closePath();
          x.fill();
        } else if (m.kind === 'spawn') {
          x.fillStyle = ink.thrown;
          x.fillRect(X - 3, Y - 3, 6, 6);
        } else {
          x.strokeStyle = ink.thrown;
          x.lineWidth = 1.5;
          x.strokeRect(X - 2.5, Y - 2.5, 5, 5);
        }
      });
    }
    x.globalAlpha = 1;
  });
  // Eksen: en çok 8 aralık
  const ay = TL.top + runs.length * TL.row + 2;
  const step = [5, 10, 15, 30, 60, 120, 300].find((s) => tmax / s <= 8) || 600;
  x.fillStyle = ink.muted;
  x.font = `11px ${font}`;
  x.textBaseline = 'top';
  for (let t = 0; t <= tmax; t += step) {
    const X = sx(t);
    x.fillRect(X, ay, 1, 4);
    x.textAlign = t === 0 ? 'left' : X > g.R0 - 16 ? 'right' : 'center';
    x.fillText(`${t} s`, X, ay + 6);
  }
  // Fare altındaki an ve oynatma imleci
  if (R.hoverT !== null && !R.dragging) {
    const X = sx(R.hoverT);
    x.globalAlpha = 0.5;
    x.fillRect(X, 0, 1, ay);
    x.globalAlpha = 1;
    x.textAlign = X > g.R0 - 40 ? 'right' : 'left';
    x.fillText(`${R.hoverT.toFixed(1)} s`, X + (X > g.R0 - 40 ? -4 : 4), 0);
  }
  x.textAlign = 'left';
  x.fillStyle = ink.primary;
  x.fillRect(sx(R.t) - 1, 0, 2, ay + 2);
}

function timelineTime(e) {
  const cv = $('tl');
  const r = cv.getBoundingClientRect();
  const g = tlGeometry(cv.clientWidth);
  const tmax = timeMax();
  return Math.max(0, Math.min(tmax, (e.clientX - r.left - g.L0) / (g.R0 - g.L0) * tmax));
}

// --- Harita (tekrar oynatma) ---------------------------------------------------------------

// Dünyası bilinmeyen koşularda sınır kutusu yörüngelerden ve hedeflerden.
function extentOf(tracks, goals) {
  let [x0, y0, x1, y1] = [Infinity, Infinity, -Infinity, -Infinity];
  const add = (x, y) => {
    x0 = Math.min(x0, x);
    y0 = Math.min(y0, y);
    x1 = Math.max(x1, x);
    y1 = Math.max(y1, y);
  };
  tracks.forEach((tr) => tr.pts.forEach((p) => add(p[1], p[2])));
  goals.forEach((gl) => add(gl[0], gl[1]));
  return x0 === Infinity ? null : [x0 - 0.5, y0 - 0.5, x1 + 0.5, y1 + 0.5];
}

// Koşunun hedefleri [x, y, yaw]: goals.csv'den (elle verilenler ve iptal edilenler dahil), yoksa
// senaryodan.
function runGoals(run) {
  const known = run.goals.filter((g) => g.x !== null && g.y !== null)
    .sort((a, b) => a.index - b.index);
  if (known.length) return known.map((g) => [g.x, g.y, g.yaw || 0]);
  const s = scenarioById(run.scenario);
  return s ? s.goals : [];
}

// Tekrar oynatmada hedef işaretleri. Tek koşu görünürken hedefleri bittikleri andan itibaren
// sonucuyla renklenir, elle verilenler verildikleri anda belirir. Çoklu seçimde ilk koşunun
// senaryo hedefleri nötr, her koşunun elle hedefleri kendi renginde (verildikleri andan itibaren).
function replayGoalMarks(runs, shown, t) {
  const R = state.replay;
  const first = R.tracks[runs[0].id];
  if (shown.length === 1) {
    const tr = shown[0];
    const goals = tr.goals.map((g, k) => [g.x, g.y, g.yaw || 0, tr.manual[k] ? 'panel' : '']);
    const results = tr.results.map((r, k) => (tr.ends[k] <= t ? r : null));
    const gi = tr.ends.findIndex((e, k) => tr.starts[k] <= t && t < e);
    const current = gi >= 0 && t < tr.duration ? gi + 1 : null;
    return goalMarks(goals, results, current, true).filter((m, k) => !tr.manual[k] || tr.starts[k] <= t);
  }
  const base = first ? first.goals.filter((g, k) => !first.manual[k]).map((g) => [g.x, g.y, g.yaw || 0])
    : runGoals(runs[0]);
  const marks = goalMarks(base, [], null, true);
  shown.forEach((tr) => {
    const color = cssVar(`--run-${R.colors[tr.run.id] + 1}`);
    tr.goals.forEach((g, k) => {
      if (tr.manual[k] && tr.starts[k] <= t) {
        marks.push({x: g.x, y: g.y, yaw: g.yaw || 0, label: tr.labels[k], color, width: 2,
          dash: tr.results[k] === 'CANCELED' && tr.ends[k] <= t});
      }
    });
  });
  return marks;
}

// Seçili koşular t anında: dünya, hareketli kutuların şeritleri, atılan kutular (atıldıkları andan
// kaldırıldıkları ana kadar), hedefler, her koşunun o ana kadarki izi ve hayalet robotu (numaralı);
// 4 s'den uzun hareketsizlikte robotun çevresinde halka (oynarken yanıp söner).
function drawReplay(cv) {
  const R = state.replay;
  const bg = cssVar('--surface-1');
  const runs = pickedRuns();
  if (!runs.length) return setupCanvas(cv, null, bg);
  const geometry = R.world !== '?' ? state.worlds[R.world] : null;
  const tracks = runs.map((r) => R.tracks[r.id]).filter(Boolean);
  const shown = tracks.filter((tr) => !R.hidden[tr.run.id]);
  const bounds = geometry ? geometry.bounds : extentOf(tracks, runGoals(runs[0]));
  const T = setupCanvas(cv, bounds, bg);
  if (!T) return null;
  const t = R.t;
  drawGrid(T, bounds);
  if (geometry) {
    drawShapes(T, geometry, false, null);
    const mover = runs.map((r) => scenarioById(r.scenario)).find((s) => s && s.moving_obstacles.length);
    if (mover) drawLanes(T, geometry, mover);  // konum kaydı yok (P8)
  }
  const thrown = cssVar('--map-thrown');
  shown.forEach((tr) => {
    const boxes = tr.boxes.filter((b) => b.from <= t && t < b.to);
    // Birden çok koşu görünürken kutunun hangi koşuya ait olduğu kenar rengiyle.
    const edge = shown.length > 1 ? cssVar(`--run-${R.colors[tr.run.id] + 1}`) : null;
    drawBoxes(T, boxes, thrown, edge);
  });
  drawGoalMarks(T, replayGoalMarks(runs, shown, t), true);
  if (tracks.length && tracks[0].pts.length) {
    drawStart(T, tracks[0].pts[0][1], tracks[0].pts[0][2], cssVar('--text-muted'));
  }
  const samples = shown.map((tr) => [tr, sample(tr, t), cssVar(`--run-${R.colors[tr.run.id] + 1}`)]);
  samples.forEach(([tr, s, color]) => drawTrail(T, tr.pts, s, color));
  const phase = R.playing ? (performance.now() / 700) % 1 : 0.4;
  samples.forEach(([tr, s, color]) => {
    if (s.hold > HOLD_RING && t < tr.duration) drawHoldRing(T, s, color, phase);
    drawRobot(T, [s.x, s.y, s.th], true, color);
    drawTag(T, s.x, s.y, R.colors[tr.run.id]);
  });
  T.coord = `t = ${t.toFixed(1)} s`;
  return T;
}

function drawStart(T, px, py, color) {
  const x = T.x;
  x.save();
  x.strokeStyle = color;
  x.lineWidth = 1.5;
  x.beginPath();
  x.arc(T.X(px), T.Y(py), Math.max(0.105 * T.k, 5), 0, 2 * Math.PI);
  x.stroke();
  x.restore();
}

function drawTrail(T, pts, s, color) {
  if (!pts.length) return;
  const x = T.x;
  x.save();
  x.strokeStyle = color;
  x.globalAlpha = 0.85;
  x.lineWidth = 2.5;
  x.lineJoin = 'round';
  x.lineCap = 'round';
  x.beginPath();
  x.moveTo(T.X(pts[0][1]), T.Y(pts[0][2]));
  for (let i = 1; i <= s.i; i++) x.lineTo(T.X(pts[i][1]), T.Y(pts[i][2]));
  x.lineTo(T.X(s.x), T.Y(s.y));
  x.stroke();
  x.restore();
}

function drawHoldRing(T, s, color, phase) {
  const x = T.x;
  x.save();
  x.strokeStyle = color;
  x.lineWidth = 2;
  x.globalAlpha = 1 - 0.8 * phase;
  x.beginPath();
  x.arc(T.X(s.x), T.Y(s.y), (0.15 + 0.4 * phase) * T.k, 0, 2 * Math.PI);
  x.stroke();
  x.restore();
}

// Robotun numarası: metin renginde, zemin renginde haleyle; aynı noktadaki robotların etiketleri
// üst üste binmesin diye her numara robotun çevresinde ayrı yöne konur.
function drawTag(T, px, py, slot) {
  const x = T.x;
  const a = -Math.PI / 4 + slot * (Math.PI / 3);
  const r = Math.max(0.105 * T.k, 6) + 8;
  const X = T.X(px) + Math.cos(a) * r;
  const Y = T.Y(py) - Math.sin(a) * r;
  x.save();
  x.font = `600 12px ${getComputedStyle(document.body).fontFamily}`;
  x.textAlign = 'center';
  x.textBaseline = 'middle';
  x.lineWidth = 3;
  x.lineJoin = 'round';
  x.strokeStyle = cssVar('--surface-1');
  x.strokeText(String(slot + 1), X, Y);
  x.fillStyle = cssVar('--text-primary');
  x.fillText(String(slot + 1), X, Y);
  x.restore();
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
  return {x, w, h, k, X: (v) => ox + v * k, Y: (v) => oy - v * k,
    wx: (px) => (px - ox) / k, wy: (py) => (oy - py) / k};  // piksel -> dünya (fare)
}

// Dünyayı, görevin hedeflerini ve robotu çizer. live verilirse (canlı önbellek) sis, SLAM
// haritası, atılan kutular, iz, plan, hareketli kutuların konumu ve robotun gerçek konumu da
// çizilir. goals verilmezse senaryonun hedefleri.
function drawWorld(cv, geometry, scenario, detailed, live, goals) {
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
    drawBoxes(T, live.boxes, cssVar('--map-thrown'));
    const going = st && MOVING.has(st.state);
    const trail = live.trail.slice();
    if (going && live.pose && trail.length) trail.push([live.pose.x, live.pose.y]);
    drawLine(T, trail, cssVar('--map-path'), 2.5);
    if (going) drawLine(T, live.plan, cssVar('--map-robot'), 1.5, [5, 4]);
  }
  if (scenario) {
    drawGoals(T, goals || scenario.goals, st ? st.goal_results : [], st ? st.goal_index : null,
      detailed);
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

// Hedef işaretleri: halka; ayrıntılıda yön çizgisi ve etiket. m: {x, y, yaw (derece), label,
// color, width, dash}.
function drawGoalMarks(T, marks, detailed) {
  const x = T.x;
  const r = 0.25 * T.k;
  x.save();
  x.font = `500 11px ${getComputedStyle(document.body).fontFamily}`;
  marks.forEach((m) => {
    const cx = T.X(m.x);
    const cy = T.Y(m.y);
    x.strokeStyle = m.color;
    x.lineWidth = m.width;
    x.setLineDash(m.dash ? [3, 3] : []);
    x.beginPath();
    x.arc(cx, cy, r, 0, 2 * Math.PI);
    x.stroke();
    if (detailed) {
      const a = m.yaw * Math.PI / 180;
      x.setLineDash([]);
      x.beginPath();
      x.moveTo(cx, cy);
      x.lineTo(cx + Math.cos(a) * r, cy - Math.sin(a) * r);
      x.stroke();
      x.fillStyle = m.color;
      x.fillText(m.label, cx + r + 2, cy - r + 4);
    }
  });
  x.restore();
}

// Görevin hedefleri ([x, y, yaw, kaynak]): varılan yeşil, başarısız kırmızı, iptal edilen gri ve
// kesikli, yürütülen kalın; elle verilenler (kaynak 'panel') kendi renginde, E1, E2… etiketli.
function goalMarks(goals, results, current, detailed) {
  const c = {goal: cssVar('--map-goal'), manual: cssVar('--map-manual'), ok: cssVar('--map-ok'),
    bad: cssVar('--map-bad'), muted: cssVar('--text-muted')};
  const labels = goalLabels(goals);
  return goals.map((g, i) => {
    const res = results[i];
    const color = res === 'SUCCEEDED' ? c.ok : res === 'CANCELED' ? c.muted : res ? c.bad
      : g[3] === 'panel' ? c.manual : c.goal;
    return {x: g[0], y: g[1], yaw: g[2], label: labels[i], color, dash: res === 'CANCELED',
      width: !detailed ? 1.5 : current === i + 1 ? 3 : 2};
  });
}

function drawGoals(T, goals, results, current, detailed) {
  drawGoalMarks(T, goalMarks(goals, results || [], current, detailed), detailed);
}

// Robot: th null ise yalnızca doğduğu nokta, değilse yönünü gösteren çizgiyle canlı konum.
// color: tekrar oynatmada koşunun rengi (varsayılan robot rengi).
function drawRobot(T, [px, py, th], detailed, color) {
  const x = T.x;
  const live = th !== null;
  const rr = Math.max(0.105 * T.k, detailed ? (live ? 6 : 5) : 3);
  x.fillStyle = color || cssVar('--map-robot');
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
  let view;
  let kind = 'none';
  let coord = '';
  if (state.mode === 'replay') {
    view = drawReplay(cv);
    if (view) {
      const shown = pickedRuns().filter((r) => !state.replay.hidden[r.id]).length;
      kind = shown === 1 ? 'replay1' : 'replay';
      coord = view.coord;
    }
  } else {
    const s = state.mode === 'live' ? scenarioById(state.scenario) : null;
    const geometry = s ? state.worlds[baseName(s.world)] : null;
    const live = geometry && liveShown() ? state.live : null;
    // Hedefler görevin kuyruğundan (elle hedefler dahil); serbest sürüşte senaryonunkiler yok.
    const free = ACTIVE.has(state.run.phase) || live ? state.run.mode === 'interactive' : state.free;
    const st = live && live.status;
    const goals = st && Array.isArray(st.goals) ? st.goals : free ? [] : undefined;
    view = drawWorld(cv, geometry, s, true, live, goals);
    if (view) kind = live ? 'live' : 'preview';
    if (view && live) drawToolOverlay(view);
    const p = live && live.pose;
    if (p) coord = `x ${p.x.toFixed(2)} · y ${p.y.toFixed(2)} · θ ${Math.round(p.th * 180 / Math.PI)}°`;
    const h = state.hover;
    if (h && toolsReady()) coord = `fare ${h[0].toFixed(2)}, ${h[1].toFixed(2)}` + (coord ? ` · ${coord}` : '');
  }
  state.mapT = kind === 'live' || kind === 'preview' ? view : null;
  if (!view) drawEmpty(cv, emptyText());
  renderMapInfo(kind);
  $('coord').textContent = coord;
}

function emptyText() {
  const runs = state.mode === 'replay' ? pickedRuns() : [];
  if (!runs.length) return MODES[state.mode].empty;
  return runs.map((r) => state.replay.errors[r.id]).find(Boolean) || 'Yörünge yükleniyor…';
}

function drawEmpty(cv, text) {
  const x = cv.getContext('2d');
  x.fillStyle = cssVar('--text-secondary');
  x.font = `14px ${getComputedStyle(document.body).fontFamily}`;
  x.textAlign = 'center';
  x.textBaseline = 'middle';
  x.fillText(text, cv.clientWidth / 2, cv.clientHeight / 2);
  x.textAlign = 'start';
  x.textBaseline = 'alphabetic';
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

// Lejant ve açıklama haritanın türüne göre (önizleme / canlı / tekrar oynatma); yalnızca tür
// değişince kurulur.
function renderMapInfo(key) {
  if (key === state.legendKey) return;
  state.legendKey = key;
  $('hint').textContent = {live: LIVE_HINT + TOOLS_HINT, replay: REPLAY_HINT,
    replay1: REPLAY_HINT}[key] || MODES[state.mode].hint;
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
  const replay = mode === 'replay';
  $('liveTop').style.display = mode === 'live' ? '' : 'none';
  $('replayTop').style.display = replay ? '' : 'none';
  $('playbar').style.display = replay ? '' : 'none';
  $('toolbar').style.display = mode === 'live' ? '' : 'none';
  if (mode !== 'live') closeConfirm();
  $('tlBox').style.display = replay ? '' : 'none';
  $('runCard').style.display = replay ? 'none' : '';
  $('metrics').style.display = replay ? 'none' : '';
  $('replayCard').style.display = replay ? '' : 'none';
  // Tekrar oynatmada koşu seçici geçmiş koşular tablosunun yerini alır.
  $('history').style.display = mode === 'live' ? '' : 'none';
  $('mapTitle').textContent = MODES[mode].title;
  state.legendKey = '';  // açıklama yeni moda göre yeniden yazılsın
  store('panel.mode', mode);
  if (replay) renderReplayAll();  // seçili koşuların yörüngeleri gerekirse yüklenir
  else {
    pause();
    $('diffCard').style.display = 'none';
  }
  drawMap();
  renderMetrics();
  if (mode === 'live') drawPreviews();  // gizliyken çizilen önizlemeler boş kalır
}

function onTheme() {
  if (state.live.mapImg) state.live.mapCanvas = recolorMap(state.live.mapImg);
  drawMap();
  drawPreviews();
  renderFrame();  // tuvallerin renkleri çizerken okunur; kartlar CSS değişkeni kullanıyor
}

// --- Başlangıç -----------------------------------------------------------------------------

document.querySelectorAll('#modes button').forEach((b) => {
  b.addEventListener('click', () => setMode(b.dataset.m));
});
$('go').addEventListener('click', startRun);
$('stop').addEventListener('click', stopRun);
$('gz').addEventListener('change', renderCommand);
$('rv').addEventListener('change', renderCommand);
$('free').addEventListener('change', () => {
  state.free = $('free').checked;
  store('panel.free', state.free ? '1' : '');
  renderCommand();
  drawMap();
});
document.querySelectorAll('#toolbar [data-tool]').forEach((b) => {
  b.addEventListener('click', () => setTool(b.dataset.tool));
});
$('clearBoxes').addEventListener('click', () => removeBoxes(null));
$('confirmGo').addEventListener('click', () => {
  if (state.pendingGoal) sendGoal(state.pendingGoal);
});
$('confirmNo').addEventListener('click', closeConfirm);
const mapCanvas = $('map');
mapCanvas.addEventListener('pointerdown', onMapDown);
mapCanvas.addEventListener('pointermove', onMapMove);
mapCanvas.addEventListener('pointerup', onMapUp);
mapCanvas.addEventListener('pointercancel', () => {
  state.drag = null;
  redraw();
});
mapCanvas.addEventListener('pointerleave', () => {
  state.hover = null;
  if (!state.drag) redraw();
});
document.addEventListener('keydown', (e) => {
  if (e.key !== 'Escape' || !(state.drag || state.pendingGoal)) return;
  state.drag = null;
  closeConfirm();
});
$('fWorld').addEventListener('change', () => {
  state.filter.world = $('fWorld').value;
  store('panel.runs.world', state.filter.world);
  renderRuns();
});
$('fResult').addEventListener('change', () => {
  state.filter.result = $('fResult').value;
  store('panel.runs.result', state.filter.result);
  renderRuns();
});
$('histReload').addEventListener('click', () => {
  state.replay.errors = {};  // okunamayan yörüngeler yeniden denensin
  loadRuns();
});
$('rpPlay').addEventListener('click', () => (state.replay.playing ? pause() : play()));
$('rpBack').addEventListener('click', () => {
  pause();
  seek(0);
});
$('rpClear').addEventListener('click', () => {
  pickedIds().slice().forEach(dropRun);
  saveReplay();
  renderReplayAll(false);
});
const timeline = $('tl');
timeline.addEventListener('pointerdown', (e) => {
  timeline.setPointerCapture(e.pointerId);
  state.replay.dragging = true;
  seek(timelineTime(e));
});
timeline.addEventListener('pointermove', (e) => {
  if (state.replay.dragging) {
    seek(timelineTime(e));
  } else {
    state.replay.hoverT = timelineTime(e);
    drawTimeline();
  }
});
timeline.addEventListener('pointerup', () => {
  state.replay.dragging = false;
});
timeline.addEventListener('pointerleave', () => {
  state.replay.hoverT = null;
  drawTimeline();
});
timeline.addEventListener('keydown', (e) => {
  const step = e.shiftKey ? 10 : 1;
  const keys = {ArrowLeft: -step, ArrowRight: step};
  if (e.key in keys) seek(state.replay.t + keys[e.key]);
  else if (e.key === 'Home') seek(0);
  else if (e.key === 'End') seek(timeMax());
  else if (e.key === ' ') state.replay.playing ? pause() : play();
  else return;
  e.preventDefault();
});
new ResizeObserver(() => {
  drawMap();
  drawPreviews();
}).observe($('map'));
new ResizeObserver(() => {
  if (state.mode === 'replay') drawTimeline();
}).observe(timeline);
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

state.filter.world = restore('panel.runs.world') || '';
state.filter.result = ['ok', 'bad'].includes(restore('panel.runs.result')) ? restore('panel.runs.result') : '';
state.free = restore('panel.free') === '1';
$('free').checked = state.free;
if (TOOL_HINTS[restore('panel.tool')]) state.tool = restore('panel.tool');
restoreReplay();
renderSpeeds();
renderRun();
renderControls();
renderMetrics();
renderRuns();
const savedMode = restore('panel.mode');
setMode(MODES[savedMode] ? savedMode : 'live');
connect();
