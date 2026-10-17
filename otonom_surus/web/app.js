// Otonom sürüş paneli istemcisi (P1: bağlantı göstergesi, mod seçici, boş harita).
// Sunucuyla iki yol: /api/stream (SSE, sunucu -> tarayıcı) ve /api/... (JSON istekleri).
'use strict';

const $ = (id) => document.getElementById(id);

const MODES = {
  live: {title: 'Canlı harita', empty: 'Simülasyon kapalı',
    hint: 'Dünya ve görev seçimi P2’de, canlı harita P3’te gelecek.'},
  replay: {title: 'Tekrar oynatma', empty: 'Koşu seçilmedi',
    hint: 'Koşu seçici ve hayalet robotlar P4–P5’te gelecek.'},
  edit: {title: 'Dünya tasarla', empty: 'Tasarlayıcı yakında',
    hint: 'Dünya tasarlayıcı P9’da gelecek.'},
};
const STALE_MS = 10000;  // bu kadar süre veri gelmezse bağlantı kopmuş sayılır
const HEALTH_MS = 10000;  // sunucu bilgisinin yenilenme aralığı
const LOG_LINES = 8;

const state = {
  mode: 'live',
  connected: null,  // null: henüz bilinmiyor (ilk açılış)
  lastSeen: 0,
  health: null,
  healthAt: 0,
  source: null,
};

function log(text) {
  const box = $('log');
  const time = new Date().toLocaleTimeString('tr-TR', {hour12: false});
  const lines = box.textContent ? box.textContent.split('\n') : [];
  lines.push(`${time}  ${text}`);
  box.textContent = lines.slice(-LOG_LINES).join('\n');
}

function setConnection(ok) {
  if (state.connected === ok) return;
  const first = state.connected === null;
  state.connected = ok;
  $('conn').className = 'conn ' + (ok ? 'on' : 'off');
  $('connText').textContent = ok ? 'ROS 2 bağlı' : 'Bağlantı yok';
  if (ok) log('Sunucuya bağlandı');
  else if (!first) log('Bağlantı koptu, yeniden deneniyor');
  else log('Sunucuya ulaşılamıyor, yeniden deneniyor');
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

function duration(s) {
  s = Math.max(0, Math.round(s));
  if (s < 60) return `${s} s`;
  if (s < 3600) return `${Math.floor(s / 60)} dk ${s % 60} s`;
  return `${Math.floor(s / 3600)} sa ${Math.floor((s % 3600) / 60)} dk`;
}

function renderHealth() {
  const h = state.health;
  if (!h) return;
  const live = state.connected;
  $('hVersion').textContent = h.version;
  $('hSim').textContent = h.sim === 'idle' ? 'kapalı' : h.sim;
  $('hClients').textContent = live ? String(h.clients) : '–';
  $('hUptime').textContent = live ? duration(h.uptime_s + (Date.now() - state.healthAt) / 1000) : '–';
  $('hSeen').textContent = state.lastSeen ? `${duration((Date.now() - state.lastSeen) / 1000)} önce` : '–';
  $('hResults').textContent = h.results_dir;
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
    seen();
    setHealth(JSON.parse(e.data));
  });
  es.addEventListener('ping', seen);
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

// Bağlantı açık görünse de veri kesildiyse (asılı bağlantı) yeniden bağlan.
setInterval(() => {
  if (state.connected && Date.now() - state.lastSeen > STALE_MS) {
    setConnection(false);
    state.source.close();
    connect();
  }
  renderHealth();
}, 1000);
setInterval(refreshHealth, HEALTH_MS);

function cssVar(name) {
  return getComputedStyle(document.documentElement).getPropertyValue(name).trim();
}

// Şimdilik boş harita: 1 m'lik ızgara (6.4 m = oda + kenar payı) ve modun boş durum metni.
function drawMap() {
  const cv = $('map');
  const dpr = window.devicePixelRatio || 1;
  const w = cv.clientWidth;
  const h = cv.clientHeight;
  if (!w || !h) return;
  cv.width = Math.round(w * dpr);
  cv.height = Math.round(h * dpr);
  const x = cv.getContext('2d');
  x.setTransform(dpr, 0, 0, dpr, 0, 0);
  x.fillStyle = cssVar('--surface-1');
  x.fillRect(0, 0, w, h);
  const k = w / 6.4;
  x.strokeStyle = cssVar('--border');
  x.lineWidth = 1;
  x.beginPath();
  for (let i = 0; i <= 6; i++) {
    const p = Math.round((0.2 + i) * k) + 0.5;
    x.moveTo(p, 0);
    x.lineTo(p, h);
    x.moveTo(0, p);
    x.lineTo(w, p);
  }
  x.stroke();
  x.fillStyle = cssVar('--text-secondary');
  x.font = `14px ${getComputedStyle(document.body).fontFamily}`;
  x.textAlign = 'center';
  x.textBaseline = 'middle';
  x.fillText(MODES[state.mode].empty, w / 2, h / 2);
}

function setMode(mode) {
  state.mode = mode;
  document.querySelectorAll('#modes button').forEach((b) => {
    const on = b.dataset.m === mode;
    b.classList.toggle('on', on);
    b.setAttribute('aria-pressed', String(on));
  });
  $('mapTitle').textContent = MODES[mode].title;
  $('hint').textContent = MODES[mode].hint;
  try {
    localStorage.setItem('panel.mode', mode);
  } catch (e) {
    // Gizli pencere ya da engellenmiş depolama: mod yalnızca bu oturumda hatırlanır.
  }
  drawMap();
}

document.querySelectorAll('#modes button').forEach((b) => {
  b.addEventListener('click', () => setMode(b.dataset.m));
});
new ResizeObserver(drawMap).observe($('map'));
window.matchMedia('(prefers-color-scheme: dark)').addEventListener('change', drawMap);

let saved = null;
try {
  saved = localStorage.getItem('panel.mode');
} catch (e) {
  saved = null;
}
setMode(MODES[saved] ? saved : 'live');
connect();
