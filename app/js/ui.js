// Interfejs aplikacji: widoki, przepływ „Pobierz wszystko”, prędkościomierz,
// lista startów, wykresy, porównanie, ustawienia, eksport/import.

import { WebBluetoothTransport, DemoTransport, bleErrorMessage } from './transport.js';
import { Device, BusyError } from './protocol.js';
import { parseRun, parseHeader, parseRace, toBase64, fromBase64, RF, fwVersionString, RACE_END_NAMES } from './format.js';
import { analyze, analyzeRace, mergeSettings, DEFAULT_SETTINGS, METRIC_LABELS, RACE_METRIC_LABELS, interpAt } from './analysis.js';
import * as db from './storage.js';
import { syncedCharts, scatterChart, trackMap, PALETTE } from './charts.js';
import { GitHubStore, syncAll } from './sync.js';
import { driftScore, mountCalibrate, DRIFT_SUMMARY_LABELS } from './drift.js';
import { LimiterSound } from './sound.js';

const limiter = new LimiterSound();
// dźwięk – ustawienie tego urządzenia (nie profilu auta)
const soundPref = { on: false, volume: 0.6, turbo: true };
try { Object.assign(soundPref, JSON.parse(localStorage.getItem('x4f_sound') || '{}')); } catch (e) { /* */ }
limiter.setVolume(soundPref.volume);
limiter.setTurbo(soundPref.turbo);
function saveSoundPref() { try { localStorage.setItem('x4f_sound', JSON.stringify(soundPref)); } catch (e) { /* */ } }

const APP_VERSION = '1.9.0';
const $ = (id) => document.getElementById(id);
const fmt = (v, d = 2) => (v == null || Number.isNaN(v) ? '–' : v.toFixed(d).replace('.', ','));

const state = {
  transport: null,
  device: null,
  demo: false,
  settings: mergeSettings(null),
  analysisCache: new Map(),
  partial: new Map(),          // seq → częściowo pobrane dane (wznawianie)
  lastBatch: [],               // {kind: 'run'|'race', id} z ostatniego pobrania (tag serii)
  raceInfo: null,              // RLIST (protokół ≥ 2)
  currentRace: null,
  raceCharts: null,
  sync: null,                  // {repo, token, auto} – tylko na tym urządzeniu (kv 'sync')
  syncing: false,
  lastStatus: null,
  info: null,                  // INFO z urządzenia (proto, montaż)
  profiles: [],                // profile aut [{id, name, settings}] (kv 'profiles')
  profileId: null,             // aktywny profil (kv 'profile_active')
  xiaoMount: null,             // co Xiao ma zapisane: [f, l, u, yaw, znak skrętu, nazwa] (+ saved)
  driftPrev: null,             // poprzednie liczniki driftu z LIVE (zdarzenia)
  calib: null,                 // wynik kalibracji montażu do zastosowania
  live: { on: false, last: 0, data: null },
  speedUnit: 'kmh',
  currentRun: null,
  charts: null,
  cmpCharts: null,
  wakeLock: null,
  downloading: false,
  userDisconnect: false,       // rozłączenie przyciskiem – bez automatycznego ponownego łączenia
  reconnecting: false,
};

// ------------------------------------------------------------------ narzędzia UI
function toast(msg, ms = 2600) {
  const t = $('toast');
  t.textContent = msg;
  t.classList.remove('hidden');
  clearTimeout(toast._t);
  toast._t = setTimeout(() => t.classList.add('hidden'), ms);
}

function show(el, on = true) { el.classList.toggle('hidden', !on); }

function setChip(text, cls) {
  const c = $('connChip');
  c.textContent = text;
  c.className = `chip ${cls}`;
}

async function wakeLock(on) {
  try {
    if (on && 'wakeLock' in navigator && !state.wakeLock) {
      state.wakeLock = await navigator.wakeLock.request('screen');
      state.wakeLock.addEventListener('release', () => { state.wakeLock = null; });
    } else if (!on && state.wakeLock) {
      await state.wakeLock.release();
      state.wakeLock = null;
    }
  } catch (e) { /* brak wsparcia / karta w tle */ }
}

// Przycisk wymagający drugiego dotknięcia (bez okien confirm())
function armedButton(btn, label, action) {
  btn.addEventListener('click', async () => {
    if (!btn.classList.contains('armed')) {
      btn.classList.add('armed');
      const orig = btn.textContent;
      btn.dataset.orig = orig;
      btn.textContent = label;
      setTimeout(() => { btn.classList.remove('armed'); btn.textContent = btn.dataset.orig; }, 3000);
      return;
    }
    btn.classList.remove('armed');
    btn.textContent = btn.dataset.orig;
    await action();
  });
}

// ------------------------------------------------------------------ nawigacja
let currentView = 'connect';
const LIVE_VIEWS = ['speed', 'drift'];
function goto(view) {
  const wasLive = LIVE_VIEWS.includes(currentView);
  if (wasLive && !LIVE_VIEWS.includes(view)) leaveSpeed();
  currentView = view;
  document.querySelectorAll('.view').forEach((v) => v.classList.toggle('active', v.id === `view-${view}`));
  document.querySelectorAll('nav.tabs button').forEach((b) => b.classList.toggle('active', b.dataset.view === view ||
    (view === 'run' && b.dataset.view === 'runs') || (view === 'race' && b.dataset.view === 'races')));
  window.scrollTo(0, 0);
  if (view === 'runs') renderRunsList();
  if (view === 'races') renderRacesList();
  if (view === 'compare') renderCompare();
  if (LIVE_VIEWS.includes(view) && !wasLive) enterSpeed();
  if (view === 'drift') {
    $('drProfile').textContent = `profil: ${activeProfile().name}`;
    $('drWarn').textContent = state.settings.mount_set ? '' :
      'Montaż nieskalibrowany – kąt driftu będzie błędny. Kilka spokojnych kółek bez driftu → Wyścigi → kalibracja montażu.';
    show($('drWarn'), !state.settings.mount_set);
    if (state.live.data) renderDrift(state.live.data);
  }
  if (view === 'settings') renderSettings();
}

// ------------------------------------------------------------------ połączenie
async function connect(kind, opts = {}) {
  $('connError').classList.add('hidden');
  try {
    if (state.transport) await disconnect();
    state.userDisconnect = false;
    state.reconnecting = false;
    const tr = kind === 'demo' ? new DemoTransport() : new WebBluetoothTransport();
    setChip('łączenie…', 'busy');
    await tr.connect(opts);
    state.transport = tr;
    state.demo = kind === 'demo';
    state.device = new Device(tr);
    state.device.onLive(onLive);
    tr.onDisconnect(onDisconnected);
    if (!state.demo) try { localStorage.setItem('x4f_paired', '1'); } catch (e) { /* */ }
    setChip(tr.name, 'on');
    show($('btnDisconnect'), true);
    show($('demoTools'), state.demo);
    $('connInfo').textContent = state.demo ? 'Tryb demo – wirtualne Xiao z syntetycznymi startami.' : `Połączono z ${tr.name}.`;
    await state.device.setTime();
    await refreshDevice();
    await sendMount();
    if (LIVE_VIEWS.includes(currentView)) enterSpeed();
  } catch (e) {
    setChip('rozłączony', 'off');
    const msg = kind === 'demo' ? `Demo: ${e.message}` : bleErrorMessage(e);
    $('connError').textContent = msg;
    show($('connError'), true);
  }
}

async function disconnect() {
  state.userDisconnect = true;
  state.reconnecting = false;
  const tr = state.transport;
  state.transport = null;
  state.device = null;
  if (tr) { try { await tr.disconnect(); } catch (e) { /* */ } }
  onDisconnected();
}

function onDisconnected() {
  // utrata zasięgu (nie przycisk „Rozłącz”) – próby ponownego połączenia z tym samym Xiao
  if (!state.userDisconnect && state.transport && state.transport.reconnect) {
    autoReconnect();
    return;
  }
  state.userDisconnect = false;
  setChip('rozłączony', 'off');
  show($('btnDisconnect'), false);
  show($('demoTools'), false);
  $('btnDownload').disabled = true;
  $('connInfo').textContent = 'Rozłączono.';
  show($('devStatus'), false);
  state.raceInfo = null;
  renderRaceDev(null);
  state.live.on = false;
  $('liveToggle').checked = false;
  if (state.downloading) $('dlText').textContent = 'Połączenie zerwane – pobieranie zostanie wznowione po ponownym połączeniu.';
  state.transport = null;
  state.device = null;
}

// Ponowne łączenie w pętli (próba na Androidzie trwa do ~30 s), aż do skutku albo „Rozłącz”.
async function autoReconnect() {
  if (state.reconnecting) return;
  state.reconnecting = true;
  const tr = state.transport;
  const t0 = performance.now();
  setChip('łączę ponownie…', 'busy');
  $('connInfo').textContent = 'Utracono zasięg – łączę ponownie automatycznie…';
  if (state.downloading) $('dlText').textContent = 'Połączenie zerwane – po ponownym połączeniu naciśnij „Pobierz wszystko” (wznowi od miejsca przerwania).';
  let n = 0;
  while (state.reconnecting && state.transport === tr && !state.userDisconnect) {
    n++;
    try {
      await tr.reconnect();
      if (!state.reconnecting || state.userDisconnect) { try { await tr.disconnect(); } catch (e) { /* */ } break; }
      state.reconnecting = false;
      setChip(tr.name, 'on');
      $('connInfo').textContent = `Połączono ponownie z ${tr.name} (po ${Math.round((performance.now() - t0) / 1000)} s).`;
      toast('Połączono ponownie');
      try {
        await state.device.setTime();
        await refreshDevice();
        await sendMount();
        if (LIVE_VIEWS.includes(currentView) || $('liveToggle').checked) {
          await state.device.live(true);
          state.live.on = true;
        }
      } catch (e) { /* kolejne zerwanie – obsłuży onDisconnected */ }
      return;
    } catch (e) {
      setChip(`łączę ponownie… (${n})`, 'busy');
      await new Promise((r) => setTimeout(r, 1500));
    }
  }
  state.reconnecting = false;
}

async function refreshDevice() {
  const d = state.device;
  if (!d) return;
  const [info, st, runs] = [await d.info(), await d.status(), await d.list()];
  state.lastStatus = st;
  state.info = info;
  state.xiaoMount = info.mount ? { mount: info.mount, saved: !!info.mount_saved } : null;
  state.raceInfo = (info.proto || 1) >= 2 ? await d.rlist() : null;
  renderRaceDev(state.raceInfo);
  const racesNew = state.raceInfo ? state.raceInfo.races.filter((r) => !r.read).length : 0;
  const box = $('devStatus');
  box.innerHTML = '';
  const rows = [
    ['Firmware', `${info.fw} (format ${info.format}, halla: ${info.hall_lines} lin.)`],
    ['Stan', st.state],
    ['Neutrum gazu', st.neutral_learned ? `${st.neutral_us} µs` : 'uczenie…'],
    ['Starty w pamięci', `${st.runs} (wolne sloty: ${st.free_slots})`],
    ['Czas ustawiony', st.time_set ? 'tak' : 'nie'],
  ];
  for (const [k, v] of rows) box.insertAdjacentHTML('beforeend', `<span class="muted">${k}</span><b>${v}</b>`);
  show(box, true);
  const parts = [];
  if (runs.length) parts.push(`${runs.length} ${plural(runs.length, 'start', 'starty', 'startów')}`);
  if (racesNew) parts.push(`${racesNew} ${plural(racesNew, 'wyścig', 'wyścigi', 'wyścigów')}`);
  $('dlInfo').textContent = parts.length ? `Do pobrania: ${parts.join(' i ')}.` : 'Brak nowych danych na urządzeniu.';
  $('btnDownload').disabled = !runs.length && !racesNew;
  const warn = $('dlWarn');
  if (st.overwritten_unread > 0) {
    warn.textContent = `Uwaga: ${st.overwritten_unread} ${plural(st.overwritten_unread, 'start został nadpisany', 'starty zostały nadpisane', 'startów zostało nadpisanych')} przed pobraniem (brak miejsca w pamięci Xiao).`;
    show(warn, true);
  } else show(warn, false);
  return runs;
}

// Karta „Pamięć wyścigów”: zajętość, stan, przycisk kasowania (tylko po pobraniu wszystkich)
const RACE_STATE_NAMES = { IDLE: 'gotowa – wyścig zacznie się sam przy gazie', RECORDING: 'nagrywanie wyścigu',
  FINISHING: 'zapis końcówki wyścigu', FULL: 'PEŁNA – nagrywanie wstrzymane', ERASING: 'kasowanie…',
  DIRTY: 'zawiera stare dane – wymaga przygotowania', ERROR: 'błąd zapisu' };

function renderRaceDev(rl) {
  const btn = $('btnRaceErase'), hint = $('raceEraseHint'), bar = $('raceBar');
  if (!rl) {
    $('raceDevInfo').textContent = state.device ? 'Firmware bez obsługi wyścigów (protokół 1).' : 'Połącz się, aby zobaczyć stan.';
    show(btn, false); show(bar, false); hint.textContent = '';
    return;
  }
  const unread = rl.races.filter((r) => !r.read).length;
  const pct = rl.size ? Math.min(100, (rl.used / rl.size) * 100) : 0;
  $('raceDevInfo').textContent = `${RACE_STATE_NAMES[rl.state] || rl.state} · ${rl.races.length} ${plural(rl.races.length, 'wyścig', 'wyścigi', 'wyścigów')}` +
    (unread ? ` (${unread} niepobranych)` : '') + ` · zajęte ${Math.round(rl.used / 1024)} / ${Math.round(rl.size / 1024)} kB`;
  show(bar, true);
  bar.firstElementChild.style.width = `${pct}%`;
  btn.classList.remove('armed');
  if (rl.state === 'DIRTY') {
    btn.textContent = 'Przygotuj pamięć wyścigów';
    btn.dataset.force = '1';
    btn.disabled = false;
    hint.textContent = 'W tej części pamięci są stare dane (np. starty sprzed zmiany układu pamięci). Kasowanie je usunie.';
  } else {
    btn.textContent = 'Skasuj pamięć wyścigów';
    btn.dataset.force = '';
    btn.disabled = unread > 0 || !rl.races.length || rl.state === 'RECORDING' || rl.state === 'FINISHING' || rl.state === 'ERASING';
    hint.textContent = unread ? 'Najpierw pobierz wszystkie wyścigi – kasowanie jest możliwe dopiero po zapisaniu ich w telefonie.'
      : rl.races.length ? 'Wszystkie wyścigi są zapisane w telefonie.' : '';
  }
  btn.dataset.orig = btn.textContent;
  show(btn, rl.state === 'DIRTY' || rl.races.length > 0);
}

async function eraseRaces() {
  const d = state.device;
  if (!d) return;
  const force = $('btnRaceErase').dataset.force === '1';
  try {
    await d.rerase(force);
    toast('Kasowanie pamięci wyścigów…');
    setTimeout(() => { if (state.device && !state.downloading) refreshDevice().catch(() => {}); }, 1500);
  } catch (e) {
    toast(e instanceof BusyError ? 'Trwa nagrywanie wyścigu – spróbuj później.' : e.message, 4000);
  }
}

function plural(n, one, few, many) {
  if (n === 1) return one;
  const d = n % 10, h = n % 100;
  return (d >= 2 && d <= 4 && (h < 12 || h > 14)) ? few : many;
}

// ------------------------------------------------------------------ pobieranie
async function downloadAll() {
  const d = state.device;
  if (!d || state.downloading) return;
  state.downloading = true;
  $('btnDownload').disabled = true;
  show($('tagBatch'), false);
  const bar = $('dlProgress');
  show(bar, true);
  await wakeLock(true);
  const batch = [];
  let failed = null;
  try {
    await d.setTime();
    const st = await d.status();
    state.lastStatus = st;
    const runs = await d.list();         // od najstarszego
    const total = runs.reduce((a, r) => a + r.len, 0);
    let doneBytes = 0;
    for (let k = 0; k < runs.length; k++) {
      const r = runs[k];
      const label = `Start ${k + 1}/${runs.length} (seq ${r.seq})`;
      const meta = { seq: r.seq, crc: r.crc, boot: r.boot };
      const id = db.runId(meta);
      if (await db.hasRun(id)) {
        // już zapisany (np. DEL nie doszedł) – tylko kasowanie z Xiao
        await d.del(r.seq);
        doneBytes += r.len;
        continue;
      }
      const partial = state.partial.get(id) || null;
      let data;
      try {
        data = await d.download(r, {
          partial,
          onProgress: (got) => {
            bar.firstElementChild.style.width = `${((doneBytes + got) / total) * 100}%`;
            $('dlText').textContent = `${label}: ${Math.round(got / 1024)} / ${Math.round(r.len / 1024)} kB`;
          },
        });
      } catch (e) {
        if (e.partial && e.partial.length) state.partial.set(id, e.partial);
        throw e;
      }
      state.partial.delete(id);
      const h = parseRun(data).header;      // weryfikuje CRC i format
      const rec = {
        id, raw: data.buffer.slice(data.byteOffset, data.byteOffset + data.byteLength),
        seq: r.seq, crc: r.crc, boot: r.boot, uptime_ms: r.uptime_ms, unix: h.unix_s,
        approx_unix: approxUnix(h, st), downloaded_at: Date.now(), tag: '', note: '',
        device: state.transport.name, flags: h.flags,
      };
      await db.putRun(rec);                 // czeka na transaction.oncomplete
      await d.del(r.seq);                   // dopiero teraz kasowanie z Xiao
      batch.push({ kind: 'run', id });
      doneBytes += r.len;
      bar.firstElementChild.style.width = `${(doneBytes / total) * 100}%`;
    }
    // Wyścigi: po zapisie w IndexedDB tylko RDEL (oznaczenie „pobrany”);
    // kasowanie całej pamięci wyścigów osobnym przyciskiem.
    const info = await d.info();
    let nRaces = 0;
    if ((info.proto || 1) >= 2) {
      const races = (await d.rlist()).races.filter((r) => !r.read);
      const rTotal = races.reduce((a, r) => a + r.len, 0);
      let rDone = 0;
      for (let k = 0; k < races.length; k++) {
        const r = races[k];
        const id = db.raceId({ seq: r.seq, crc: r.crc, boot: r.boot });
        if (await db.hasRace(id)) { await d.rdel(r.seq); rDone += r.len; continue; }
        const label = `Wyścig ${k + 1}/${races.length} (seq ${r.seq})`;
        let data;
        try {
          data = await d.download(r, {
            race: true,
            partial: state.partial.get(id) || null,
            onProgress: (got) => {
              bar.firstElementChild.style.width = `${((rDone + got) / rTotal) * 100}%`;
              $('dlText').textContent = `${label}: ${Math.round(got / 1024)} / ${Math.round(r.len / 1024)} kB`;
            },
          });
        } catch (e) {
          if (e.partial && e.partial.length) state.partial.set(id, e.partial);
          throw e;
        }
        state.partial.delete(id);
        const race = parseRace(data);          // weryfikuje CRC i format
        const h = race.header;
        await db.putRace({
          id, raw: data.buffer.slice(data.byteOffset, data.byteOffset + data.byteLength),
          seq: r.seq, crc: r.crc, boot: r.boot, uptime_ms: r.uptime_ms, unix: h.unix_s,
          approx_unix: approxUnix(h, st), downloaded_at: Date.now(), tag: '', note: '',
          device: state.transport.name, flags: h.flags, end_reason: race.end_reason,
        });
        await d.rdel(r.seq);
        batch.push({ kind: 'race', id });
        nRaces++;
        rDone += r.len;
      }
    }
    const nRuns = batch.length - nRaces;
    const got = [];
    if (nRuns) got.push(`${nRuns} ${plural(nRuns, 'start', 'starty', 'startów')}`);
    if (nRaces) got.push(`${nRaces} ${plural(nRaces, 'wyścig', 'wyścigi', 'wyścigów')}`);
    $('dlText').textContent = got.length ? `Pobrano ${got.join(' i ')}.` : 'Nic nowego do pobrania.';
  } catch (e) {
    failed = e;
    const msg = e instanceof BusyError ? 'Urządzenie nagrywa/zapisuje – spróbuj za chwilę.' : e.message;
    $('dlText').textContent = `Przerwano: ${msg}. Niepobrane dane zostały na Xiao.`;
  } finally {
    state.downloading = false;
    await wakeLock(false);
    if (state.device) { try { await refreshDevice(); } catch (e) { /* */ } }
  }
  if (batch.length) {
    state.lastBatch = batch;
    await showTagBatch();
    if (state.sync && state.sync.auto) doSync({ quiet: true });
  }
  return failed;
}

// Start sprzed ustawienia czasu: przeliczenie z uptime, jeśli ten sam rozruch Xiao
function approxUnix(h, st) {
  if (h.unix_s) return h.unix_s;
  if (st && st.boot === h.boot_count && st.uptime_ms >= h.uptime_ms) {
    return Math.round(Date.now() / 1000 - (st.uptime_ms - h.uptime_ms) / 1000);
  }
  return 0;
}

async function showTagBatch() {
  const box = $('tagBatch');
  const chips = $('tagRecent');
  chips.innerHTML = '';
  for (const t of await db.recentTags()) {
    const b = document.createElement('button');
    b.textContent = t;
    b.onclick = () => applyBatchTag(t);
    chips.appendChild(b);
  }
  $('tagInput').value = '';
  show(box, true);
}

async function applyBatchTag(tag) {
  tag = tag.trim();
  for (const b of state.lastBatch) await (b.kind === 'race' ? db.updateRace(b.id, { tag }) : db.updateRun(b.id, { tag }));
  await db.pushRecentTag(tag);
  show($('tagBatch'), false);
  toast(tag ? `Tag „${tag}” przypisany do ${state.lastBatch.length} nagrań` : 'Pominięto tag');
}

// ------------------------------------------------------------------ LIVE / prędkościomierz
function onLive(m) {
  state.live.last = performance.now();
  state.live.data = m;
  const box = $('liveBox');
  if (state.live.on && currentView === 'connect') {
    box.innerHTML = [['v', `${fmt(m.v)} m/s`], ['v kół', `${fmt(m.v_front)} m/s`], ['obroty', `${m.rpm} rpm`],
      ['gaz', `${m.thr} µs (${fmt(m.thr_pct, 0)} %)`], ['a', `${fmt(m.ax)} g`], ['stan', m.state],
      ['wyścig', m.race ? (m.race === 'RECORDING' ? `nagrywanie ${mmss(m.race_s)}` : m.race) : '–']]
      .map(([k, v]) => `<span class="muted">${k}</span><b>${v}</b>`).join('');
  }
  if (currentView === 'speed') renderSpeed(m);
  if (currentView === 'drift') renderDrift(m);
}

// Montaż płytki z ustawień → Xiao (protokół ≥ 3; Xiao pamięta go do wyłączenia zasilania)
// Profil, w którym nikt nie ustawiał montażu (kalibracja / ręcznie), przejmuje montaż, kierunek skrętu
// i pełny gaz zapisane na Xiao pod tą samą nazwą – np. ustawione z komputera po analizie nagrania.
async function adoptFromXiao() {
  const i = state.info;
  if (!i || (i.proto || 1) < 7 || !i.mount || !i.mount_saved || state.settings.mount_set) return false;
  if (profileShowName(i.mount[5]) !== activeProfile().name) return false;
  const s = mergeSettings(state.settings);
  s.imu_axes = { forward: i.mount[0], left: i.mount[1], up: i.mount[2] };
  s.estimator.yaw_offset_deg = +i.mount[3] || 0;
  s.drift.steer_sign = +i.mount[4] || 0;
  if (i.full_pct) s.drift.full_thr_pct = +i.full_pct;
  if (i.full_s !== undefined) s.drift.full_thr_s = +i.full_s;
  s.drift.full_bonus = i.drift_bonus ? 1 : 0;
  s.mount_set = true;
  await persistSettings(s);
  toast('Przejęto z Xiao: montaż, kierunek skrętu i próg gazu');
  return true;
}

async function sendMount() {
  const d = state.device;
  if (!d || !state.info || (state.info.proto || 1) < 3) return;
  await adoptFromXiao();
  const proto = state.info.proto || 1;
  const st = proto >= 4 ? (state.settings.drift.steer_sign || 0) : undefined;
  const name = proto >= 5 ? profileWireName(activeProfile().name) : undefined;
  try {
    const r = await d.mount(state.settings.imu_axes, state.settings.estimator.yaw_offset_deg, st, name);
    if (r && r.mount) state.xiaoMount = { mount: r.mount, saved: !!r.mount_saved };
    const dr = state.settings.drift;
    if (proto >= 7) await d.driftBonus(!!dr.full_bonus, dr.full_thr_pct, dr.full_thr_s);
    else if (proto >= 6) await d.driftBonus(!!dr.full_bonus);
    if (r && r.mount && !r.mount_saved) {
      // zapis na Xiao w następnej pętli (albo po nagrywaniu) – potwierdzenie chwilę później
      setTimeout(async () => {
        try {
          const q = state.device && await state.device.mount();
          if (q && q.mount) state.xiaoMount = { mount: q.mount, saved: !!q.mount_saved };
          if (currentView === 'settings') renderProfiles();
        } catch (e) { /* */ }
      }, 400);
    }
  } catch (e) { toast(`Montaż: ${e.message}`); }
  if (currentView === 'settings') renderProfiles();
}

const fmtPts = (v) => Math.round(v || 0).toLocaleString('pl-PL');

let drEventTimer = null;
function driftEvent(text, cls) {
  const el = $('drEvent');
  el.textContent = text;
  el.className = `dr-event show ${cls}`;
  clearTimeout(drEventTimer);
  drEventTimer = setTimeout(() => { el.className = 'dr-event'; }, 1800);
}

function renderDrift(m) {
  if (m.d_total === undefined) {
    $('drHint').textContent = 'Firmware bez punktów za drift – wgraj wersję 1.2 (drift).';
    return;
  }
  const p = state.driftPrev;
  if (p) {
    if (m.d_lost > p.d_lost) { driftEvent('OBRÓT – łańcuch stracony', 'bad'); vibrate([80, 60, 80]); }
    else if (m.d_banks > p.d_banks) { driftEvent(`+${fmtPts(m.d_last)}`, 'good'); vibrate(40); }
    else if (m.d_mult > p.d_mult) { driftEvent(`PRZEKŁADKA ×${m.d_mult}`, 'mult'); vibrate(30); }
  }
  state.driftPrev = { d_lost: m.d_lost, d_banks: m.d_banks, d_mult: m.d_mult };
  $('drPts').textContent = fmtPts(m.d_pts);
  $('drChainLbl').textContent = m.d_on ? 'DRIFT' : m.d_chain ? 'łańcuch trwa…' : 'łańcuch';
  document.querySelector('.drift').classList.toggle('on', !!m.d_on);
  $('drMult').textContent = `×${m.d_mult}`;
  $('drTotal').textContent = fmtPts(m.d_total);
  $('drBest').textContent = fmtPts(m.d_best);
  $('drTrans').textContent = m.d_trans;
  $('drLost').textContent = m.d_lost;
  const a = Math.max(0, Math.min(90, m.d_ang || 0));
  $('drAngle').textContent = `${Math.round(m.d_ang || 0)}°`;
  $('drAngleBar').style.width = `${a / 90 * 100}%`;
  $('drCs').classList.toggle('on', !!m.d_cs);
  const full = !!m.d_full;
  show($('drFull'), m.d_full !== undefined);
  $('drFull').classList.toggle('on', full);
  $('drFull').textContent = full && state.settings.drift.full_bonus ? `PEŁNY GAZ ×${fmt(state.settings.drift.full_bonus_mult, 1)}` : 'PEŁNY GAZ';
  limiter.update(m.thr_pct || 0, state.settings.drift.full_thr_pct, soundPref.on);
  show($('drCs'), m.d_cs !== undefined);
  renderSteer(m);
}

// Podgląd skrętu: „+” = w lewo po uwzględnieniu kierunku (Xiao: znak z profilu albo nauczony).
// Skręć kołami w lewo – napis musi pokazać W LEWO; jeśli W PRAWO, zmień „Kierunek skrętu” w Opcjach.
function renderSteer(m) {
  const sgn = m.d_steer_sign;
  if (m.st === undefined) { $('drSteerTxt').textContent = 'skręt: firmware bez skrętu'; return; }
  const v = Math.max(-100, Math.min(100, m.st || 0));
  $('drSteerDot').style.left = `${50 - v / 2}%`;
  const dir = Math.abs(v) < 3 ? 'prosto' : v > 0 ? 'W LEWO' : 'W PRAWO';
  const kind = sgn > 0 ? 'normalny' : sgn < 0 ? 'odwrócony' : 'kierunek nieznany – znak surowy';
  $('drSteerTxt').textContent = `skręt: ${dir} ${Math.abs(Math.round(v))} % (${kind})`;
}

function vibrate(p) { try { if (navigator.vibrate) navigator.vibrate(p); } catch (e) { /* */ } }

function mmss(s) {
  s = Math.max(0, Math.round(s || 0));
  return `${Math.floor(s / 60)}:${String(s % 60).padStart(2, '0')}`;
}

function speedConv(v) { return state.speedUnit === 'kmh' ? v * 3.6 : v; }

function renderSpeed(m) {
  $('spdValue').textContent = fmt(speedConv(m.v), 1);
  $('spdMax').textContent = fmt(speedConv(m.v_max), 1);
  $('spdFront').textContent = fmt(speedConv(m.v_front), 1);
  state.rpmMax = Math.max(state.rpmMax || 0, m.rpm || 0);
  $('spdRpm').textContent = Math.round(m.rpm || 0);
  $('spdRpmMax').textContent = Math.round(state.rpmMax);
  $('spdThr').textContent = `${fmt(m.thr_pct, 0)} %`;
  $('spdState').textContent = m.state === 'RECORDING' ? '● nagrywanie' : '';
  $('spdRace').textContent = m.race === 'RECORDING' ? `⚑ wyścig ${mmss(m.race_s)}` : m.race === 'FULL' ? '⚑ pamięć wyścigów pełna' : '';
  const s = $('spdSlip');
  s.className = `slip ${m.slip ? 'bad' : 'ok'}`;
  s.textContent = m.slip ? 'BUKSOWANIE' : 'przyczepność';
}

let speedTimer = null;
async function enterSpeed() {
  await wakeLock(true);
  show($('spdHint'), !state.device);
  if (state.device) { try { await state.device.live(true); state.live.on = true; } catch (e) { /* */ } }
  clearInterval(speedTimer);
  speedTimer = setInterval(() => {
    const stale = !state.device || performance.now() - state.live.last > 300;
    show($('spdNoSignal'), stale);
    document.querySelector('.speedo').classList.toggle('stale', stale);
    show($('drNoSignal'), stale);
    if (stale) limiter.stop();
    document.querySelector('.drift').classList.toggle('stale', stale);
  }, 100);
}

async function leaveSpeed() {
  clearInterval(speedTimer);
  limiter.stop();
  if (!state.downloading) await wakeLock(false);
  if (state.device && !$('liveToggle').checked) { try { await state.device.live(false); } catch (e) { /* */ } }
}

// ------------------------------------------------------------------ analiza (pamięć podręczna)
function settingsKey() { return JSON.stringify(state.settings); }

function getAnalysis(rec) {
  const key = settingsKey();
  const c = state.analysisCache.get(rec.id);
  if (c && c.key === key) return c.res;
  const run = parseRun(new Uint8Array(rec.raw), { verifyCrc: false });
  const res = analyze(run, state.settings);
  res.run = run;
  state.analysisCache.set(rec.id, { key, res });
  return res;
}

function runDate(rec) {
  const u = rec.unix || rec.approx_unix;
  if (!u) return `rozruch ${rec.boot}, ${Math.round(rec.uptime_ms / 1000)} s od włączenia`;
  const d = new Date(u * 1000);
  return (rec.unix ? '' : '≈ ') + d.toLocaleString('pl-PL', { dateStyle: 'short', timeStyle: 'medium' });
}

function metricValue(k, v) {
  if (v == null || Number.isNaN(v)) return '–';
  if (k === 'peak_slip') return fmt(v * 100, 0);
  if (k === 'esc_delay_s') return fmt(v * 1000, 0);
  if (k === 'rpm_max') return fmt(v, 0);
  if (k === 'rpm_max_t') return fmt(v, 3);
  if (k.startsWith('t_') || k === 'peak_slip_t') return fmt(v, 3);
  return fmt(v, 2);
}

// ------------------------------------------------------------------ lista startów
async function renderRunsList() {
  const all = (await db.allRuns()).sort((a, b) => (b.unix || b.approx_unix || b.downloaded_at / 1000) - (a.unix || a.approx_unix || a.downloaded_at / 1000));
  const sel = $('runsFilter');
  const cur = sel.value;
  const tags = [...new Set(all.map((r) => r.tag).filter(Boolean))].sort();
  sel.innerHTML = '<option value="">wszystkie tagi</option>' + tags.map((t) => `<option>${esc(t)}</option>`).join('');
  sel.value = tags.includes(cur) ? cur : '';
  const runs = sel.value ? all.filter((r) => r.tag === sel.value) : all;
  $('runsCount').textContent = `${runs.length} ${plural(runs.length, 'start', 'starty', 'startów')}`;
  const list = $('runsList');
  list.innerHTML = '';
  if (!runs.length) {
    list.innerHTML = '<p class="muted center">Brak startów. Pobierz je z urządzenia (zakładka Połączenie) albo użyj trybu demo.</p>';
    return;
  }
  for (const rec of runs) {
    let m = {};
    try { m = getAnalysis(rec).metrics; } catch (e) { m = {}; }
    const b = document.createElement('button');
    b.className = 'item';
    b.innerHTML = `<div class="top"><b>${runDate(rec)}</b>${rec.tag ? `<span class="tag">${esc(rec.tag)}</span>` : ''}</div>
      ${rec.note ? `<div class="muted small">${esc(rec.note)}</div>` : ''}
      <div class="m"><span>a 0–0,3 s <b>${metricValue('a', m.a_long_0_300ms)}</b> m/s²</span>
      <span>v 1 s <b>${metricValue('v', m.v_imu_1_0s)}</b> m/s</span>
      <span>obroty <b>${metricValue('rpm_max', m.rpm_max)}</b> rpm</span>
      <span>poślizg <b>${metricValue('peak_slip', m.peak_slip)}</b> %</span>
      <span class="muted">seq ${rec.seq}</span></div>`;
    b.onclick = () => openRun(rec.id);
    list.appendChild(b);
  }
}

function esc(s) { return String(s).replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' })[c]); }

// ------------------------------------------------------------------ szczegóły startu
async function openRun(id) {
  const rec = await db.getRun(id);
  if (!rec) return;
  state.currentRun = rec;
  goto('run');
  const res = getAnalysis(rec);
  const h = res.header;
  $('runTitle').textContent = `Start seq ${rec.seq}${rec.tag ? ' – ' + rec.tag : ''}`;
  $('runMeta').textContent = `${runDate(rec)} · halla: ${h.hall_lines} lin. · ${h.n_samples} próbek, ${h.n_edges} zboczy · fw ${fwVersionString(h.fw_version)}` +
    (h.flags & RF.SIMULATED ? ' · dane syntetyczne' : '');
  const warns = [...res.warnings];
  if (h.flags & RF.EDGES_TRUNCATED) warns.push('Bufor zboczy przepełniony – v_front z okresów obrotu.');
  if (h.flags & RF.IMU_ERROR) warns.push('Błąd IMU podczas nagrania.');
  $('runWarn').textContent = warns.join(' ');
  show($('runWarn'), warns.length > 0);
  $('runTag').value = rec.tag || '';
  $('runNote').value = rec.note || '';
  const chips = $('runTagRecent');
  chips.innerHTML = '';
  for (const t of await db.recentTags()) {
    const b = document.createElement('button');
    b.textContent = t;
    b.onclick = () => { $('runTag').value = t; saveRunTag(); };
    chips.appendChild(b);
  }
  renderRunCharts(res);
  const mbox = $('runMetrics');
  mbox.innerHTML = Object.entries(METRIC_LABELS).map(([k, [label, unit]]) =>
    `<div class="metric"><div class="l">${label}</div><div class="v">${metricValue(k, res.metrics[k])} <span class="small muted">${unit}</span></div></div>`).join('');
}

function renderRunCharts(res) {
  if (state.charts) state.charts.destroy();
  const s = res.series;
  const pct = (a) => Float64Array.from(a, (x) => x * 100);
  state.charts = syncedCharts($('runCharts'), s.t, [
    { label: 'gaz', unit: '%', height: 110, series: [{ label: 'gaz', values: s.throttle_pct, color: PALETTE[4] }] },
    { label: 'v', unit: 'm/s', height: 190, series: [
      { label: 'v_front', values: s.v_front, color: PALETTE[1] },
      { label: 'v_imu', values: s.v_imu, color: PALETTE[0], width: 2 },
      { label: 'v_disp', values: s.v_disp, color: PALETTE[2], dash: [6, 4] }] },
    { label: 'a', unit: 'm/s²', height: 170, series: [
      { label: 'a_front', values: s.a_front, color: PALETTE[1] },
      { label: 'a_long', values: s.a_long, color: PALETTE[0], width: 2 }] },
    { label: 'poślizg', unit: '%', height: 140, series: [{ label: 'poślizg', values: pct(s.slip), color: PALETTE[6] }] },
    { label: 'obroty', unit: 'rpm', height: 140, series: [{ label: 'obroty silnika', values: s.rpm, color: PALETTE[5] }] },
    { label: 'pitch', unit: '°', height: 110, series: [{ label: 'pitch', values: s.pitch_deg, color: PALETTE[3] }] },
  ]);
  setRunRange('1');
}

function setRunRange(r) {
  const res = state.currentRun && getAnalysis(state.currentRun);
  if (!res || !state.charts) return;
  const t0 = res.metrics.t_launch_s;
  const t = res.series.t;
  if (r === 'all') state.charts.setRange(t[0], t[t.length - 1]);
  else state.charts.setRange(t0 - 0.05, t0 + Number(r));
}

async function saveRunTag() {
  const rec = state.currentRun;
  if (!rec) return;
  const tag = $('runTag').value.trim();
  await db.updateRun(rec.id, { tag, note: $('runNote').value });
  rec.tag = tag;
  await db.pushRecentTag(tag);
  $('runTitle').textContent = `Start seq ${rec.seq}${tag ? ' – ' + tag : ''}`;
  toast('Zapisano');
}

function runCsv(rec) {
  const res = getAnalysis(rec);
  const s = res.series;
  const cols = ['t', 'throttle_pct', 'v_front', 'v_imu', 'v_disp', 'a_front', 'a_long', 'slip', 'spin', 'pitch_deg', 'd_front', 'd_imu', 'rpm'];
  const lines = [cols.join(';')];
  for (let i = 0; i < s.t.length; i++) lines.push(cols.map((c) => Number(s[c][i]).toFixed(5)).join(';'));
  return lines.join('\n');
}

async function shareOrDownload(name, text, mime) {
  const blob = new Blob([text], { type: mime });
  const file = new File([blob], name, { type: mime });
  if (navigator.canShare && navigator.canShare({ files: [file] })) {
    try { await navigator.share({ files: [file], title: name }); return; } catch (e) { if (e.name === 'AbortError') return; }
  }
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = name;
  document.body.appendChild(a);
  a.click();
  setTimeout(() => { URL.revokeObjectURL(a.href); a.remove(); }, 1000);
}

// ------------------------------------------------------------------ wyścigi
function getRaceDrift(rec) {
  const key = settingsKey();
  const c = state.analysisCache.get('drift:' + rec.id);
  if (c && c.key === key) return c.res;
  const race = parseRace(new Uint8Array(rec.raw), { verifyCrc: false });
  const res = driftScore(race, state.settings);
  state.analysisCache.set('drift:' + rec.id, { key, res });
  return res;
}

function getRaceAnalysis(rec) {
  const key = settingsKey();
  const c = state.analysisCache.get('race:' + rec.id);
  if (c && c.key === key) return c.res;
  const race = parseRace(new Uint8Array(rec.raw), { verifyCrc: false });
  const res = analyzeRace(race, state.settings);
  state.analysisCache.set('race:' + rec.id, { key, res });
  return res;
}

function raceMetricValue(k, v) {
  if (v == null || Number.isNaN(v)) return '–';
  if (k === 'duration_s') return mmss(v);
  if (k === 'rpm_max' || k === 'heading_total_deg' || k === 'gaps' || k === 'distance_m') return fmt(v, 0);
  if (k === 'full_throttle_frac') return fmt(v * 100, 0);
  if (k === 'laps_n') return fmt(v, 0);
  if (k === 'track_length_m') return fmt(v, 1);
  return fmt(v, 2);
}

async function renderRacesList() {
  const all = (await db.allRaces()).sort((a, b) => (b.unix || b.approx_unix || b.downloaded_at / 1000) - (a.unix || a.approx_unix || a.downloaded_at / 1000));
  $('racesCount').textContent = `${all.length} ${plural(all.length, 'wyścig', 'wyścigi', 'wyścigów')}`;
  const list = $('racesList');
  list.innerHTML = '';
  if (!all.length) {
    list.innerHTML = '<p class="muted center">Brak wyścigów. Xiao nagrywa je samo przy jeździe; pobierzesz je przyciskiem „Pobierz wszystko”.</p>';
    return;
  }
  for (const rec of all) {
    let m = {}, dr = null;
    try { m = getRaceAnalysis(rec).metrics; } catch (e) { m = {}; }
    try { dr = getRaceDrift(rec).summary; } catch (e) { dr = null; }
    const b = document.createElement('button');
    b.className = 'item';
    b.innerHTML = `<div class="top"><b>${runDate(rec)}</b>${rec.tag ? `<span class="tag">${esc(rec.tag)}</span>` : ''}</div>
      ${rec.note ? `<div class="muted small">${esc(rec.note)}</div>` : ''}
      <div class="m"><span>czas <b>${raceMetricValue('duration_s', m.duration_s)}</b></span>
      <span>droga <b>${raceMetricValue('distance_m', m.distance_m)}</b> m</span>
      <span>v maks. <b>${fmt(speedConv(m.v_front_max), 1)}</b> ${state.speedUnit === 'kmh' ? 'km/h' : 'm/s'}</span>
      <span>obroty <b>${raceMetricValue('rpm_max', m.rpm_max)}</b> rpm</span>
      ${dr ? `<span>drift <b>${fmtPts(dr.total)}</b> pkt</span>` : ''}
      <span class="muted">seq ${rec.seq}</span></div>`;
    b.onclick = () => openRace(rec.id);
    list.appendChild(b);
  }
}

async function openRace(id) {
  const rec = await db.getRace(id);
  if (!rec) return;
  state.currentRace = rec;
  goto('race');
  const res = getRaceAnalysis(rec);
  const h = res.header;
  $('raceTitle').textContent = `Wyścig seq ${rec.seq}${rec.tag ? ' – ' + rec.tag : ''}`;
  $('raceMeta').textContent = `${runDate(rec)} · ${h.sample_rate_hz} Hz · ${res.series.t.length} próbek · koniec: ${RACE_END_NAMES[res.end_reason] || res.end_reason} · fw ${fwVersionString(h.fw_version)}` +
    (h.flags & RF.SIMULATED ? ' · dane syntetyczne' : '');
  const warns = [...res.warnings];
  if (h.flags & RF.IMU_ERROR) warns.push('Błąd IMU podczas nagrania.');
  $('raceWarn').textContent = warns.join(' ');
  show($('raceWarn'), warns.length > 0);
  $('raceTag').value = rec.tag || '';
  $('raceNote').value = rec.note || '';
  if (state.raceCharts) state.raceCharts.destroy();
  const s = res.series;
  const dr = getRaceDrift(rec);
  renderRaceDrift(dr);
  state.calib = null;
  show($('raceCalibApply'), false);
  $('raceCalibMsg').textContent = '';
  const v = Float64Array.from(s.v_front, (x) => speedConv(x));
  state.raceCharts = syncedCharts($('raceCharts'), s.t, [
    { label: 'gaz', unit: '%', height: 100, series: [{ label: 'gaz', values: s.throttle_pct, color: PALETTE[4] }] },
    { label: 'v kół', unit: state.speedUnit === 'kmh' ? 'km/h' : 'm/s', height: 170, series: [{ label: 'v_front', values: v, color: PALETTE[1], width: 2 }] },
    { label: 'obroty', unit: 'rpm', height: 130, series: [{ label: 'obroty silnika', values: s.rpm, color: PALETTE[5] }] },
    { label: 'a', unit: 'm/s²', height: 160, series: [
      { label: 'a_long', values: s.a_long, color: PALETTE[0], width: 2 },
      { label: 'a_lat', values: s.a_lat, color: PALETTE[2] }] },
    { label: 'skręt', unit: '°/s', height: 130, series: [{ label: 'prędkość skrętu', values: s.yaw_rate, color: PALETTE[3] }] },
    { label: 'kąt driftu', unit: '°', height: 130, series: [{ label: 'kąt', values: dr.series.angle, color: PALETTE[6 % PALETTE.length] }] },
    { label: 'skręt', unit: '%', height: 110, series: [
      { label: 'skręt (+ lewo)', values: dr.series.steer, color: PALETTE[7] },
      { label: 'kontra', values: Float64Array.from(dr.series.cs, (x) => x * 20), color: PALETTE[2] }] },
    { label: 'punkty', unit: 'pkt', height: 130, series: [
      { label: 'łańcuch', values: dr.series.pts, color: PALETTE[2] },
      { label: 'suma', values: dr.series.total, color: PALETTE[0], width: 2 }] },
  ], { syncKey: 'race' });
  setRaceRange('all');
  state.raceLap = -1;
  renderRaceTrack(res);
  $('raceMetrics').innerHTML = Object.entries(RACE_METRIC_LABELS).map(([k, [label, unit]]) => {
    let val = raceMetricValue(k, res.metrics[k]), u = unit;
    if (k === 'v_front_max' || k === 'v_front_avg') { val = fmt(speedConv(res.metrics[k]), 1); u = state.speedUnit === 'kmh' ? 'km/h' : 'm/s'; }
    return `<div class="metric"><div class="l">${label}</div><div class="v">${val} <span class="small muted">${u}</span></div></div>`;
  }).join('');
}

function renderRaceDrift(dr) {
  const ss = dr.summary.steer_sign;
  $('raceDrift').innerHTML = Object.entries(DRIFT_SUMMARY_LABELS).map(([k, [label, unit]]) => {
    const v = dr.summary[k];
    const val = k === 'drift_time_s' || k === 'angle_max_deg' ? fmt(v, 1) : fmtPts(v);
    return `<div class="metric"><div class="l">${label}</div><div class="v">${val} <span class="small muted">${unit}</span></div></div>`;
  }).join('') + `<div class="metric"><div class="l">Skręt</div><div class="v small">${
    ss > 0 ? 'normalny' : ss < 0 ? 'odwrócony' : 'brak / nieustalony'}</div></div>`;
}

// Wycinek nagrania [a, b] s (do kalibracji) – postój sprzed gazu zostaje (oś „up”, bias)
function raceSlice(race, a, b) {
  const ti = race.header.trigger_index;
  const idx = [];
  for (let i = 0; i < race.n; i++) if (i < ti || (race.t_s[i] >= a && race.t_s[i] <= b)) idx.push(i);
  const out = { ...race, n: idx.length };
  for (const k of ['edges', 'period_rev', 'throttle_us', 'ax', 'ay', 'az', 'gx', 'gy', 'gz', 'flags', 'n_ticks', 'steer_us', 't_s']) {
    const src = race[k];
    const dst = new src.constructor(idx.length);
    idx.forEach((i, j) => { dst[j] = src[i]; });
    out[k] = dst;
  }
  return out;
}

function calibrateFromRace() {
  const rec = state.currentRace;
  if (!rec) return;
  try {
    let race = parseRace(new Uint8Array(rec.raw), { verifyCrc: false });
    let rangeTxt = '';
    if (state.raceCharts && state.raceCharts.getRange) {
      const [a, b] = state.raceCharts.getRange();
      const t = race.t_s;
      if (a > t[0] + 0.5 || b < t[t.length - 1] - 0.5) { race = raceSlice(race, a, b); rangeTxt = ` Zakres ${fmt(a, 0)}–${fmt(b, 0)} s.`; }
    }
    const cal = mountCalibrate(race);
    state.calib = cal;
    const ax = cal.imu_axes;
    const cur = state.settings.imu_axes;
    const same = ax.forward === cur.forward && ax.left === cur.left && ax.up === cur.up &&
      Math.abs(cal.yaw_offset_deg - state.settings.estimator.yaw_offset_deg) < 2 &&
      (!cal.steer_sign || cal.steer_sign === state.settings.drift.steer_sign);
    const stTxt = cal.steer_sign > 0 ? 'normalny' : cal.steer_sign < 0 ? 'odwrócony' : 'nieustalony (brak skrętu w nagraniu)';
    $('raceCalibMsg').innerHTML = `Do przodu <b>${ax.forward}</b>, w lewo <b>${ax.left}</b>, w górę <b>${ax.up}</b>, ` +
      `kąt w poziomie <b>${fmt(cal.yaw_offset_deg, 1)}°</b> (z ${cal.samples} próbek w zakrętach, rozrzut ${fmt(cal.spread_deg, 0)}°). ` +
      `Skręt: <b>${stTxt}</b>.${rangeTxt}` +
      (cal.spread_deg > 30 ? ' <span class="warn">Duży rozrzut – nagranie z poślizgami albo za wolna jazda; wynik niepewny.</span>' : '') +
      (getRaceDrift(rec).summary.drift_time_s > 2 ? ' <span class="warn">W nagraniu jest drift – kalibruj na spokojnych kółkach z przyczepnością (inaczej montaż i kierunek skrętu wyjdą błędnie).</span>' : '') +
      (same ? ' Tak jak w ustawieniach.' : '');
    show($('raceCalibApply'), !same);
  } catch (e) {
    state.calib = null;
    $('raceCalibMsg').textContent = e.message;
    show($('raceCalibApply'), false);
  }
}

async function applyCalibration() {
  const cal = state.calib;
  if (!cal) return;
  const s = mergeSettings(state.settings);
  s.imu_axes = { ...cal.imu_axes };
  s.estimator.yaw_offset_deg = Math.round(cal.yaw_offset_deg * 10) / 10;
  if (cal.steer_sign) s.drift.steer_sign = cal.steer_sign;
  s.mount_set = true;
  await persistSettings(s);
  await sendMount();
  show($('raceCalibApply'), false);
  toast(state.device && (state.info?.proto || 1) >= 3 ? 'Montaż zapisany i wysłany do Xiao' : 'Montaż zapisany – trafi do Xiao po połączeniu');
  if (state.currentRace) openRace(state.currentRace.id);
}

// Mapa toru + tabela okrążeń; dotknięcie wiersza = to okrążenie na mapie i na wykresach
function renderRaceTrack(res) {
  const card = $('raceTrackCard');
  const laps = res.laps || [];
  if (!res.track) {
    $('raceTrackTitle').textContent = 'Tor';
    show($('raceTrack'), false);
    $('raceTrackNote').textContent = laps.length < 2
      ? 'Za mało okrążeń do narysowania toru (potrzebne co najmniej 2 pełne okrążenia w kółko).'
      : 'Nie udało się wyznaczyć toru.';
    $('raceLaps').innerHTML = '';
    show(card, true);
    return;
  }
  show($('raceTrack'), true);
  const good = laps.map((lp, j) => (lp.good ? j : -1)).filter((j) => j >= 0);
  const best = res.metrics.best_lap_s;
  const unit = state.speedUnit === 'kmh' ? 'km/h' : 'm/s';
  const sel = state.raceLap;
  $('raceTrackTitle').textContent = sel >= 0 ? `Tor – okrążenie ${sel + 1}` : `Tor – średnia z ${good.length} okrążeń`;
  trackMap($('raceTrack'), res.track, { lap: sel, goodLaps: good, vConv: speedConv, unit });
  $('raceTrackNote').textContent = `Długość toru ok. ${fmt(res.track.length_m, 1)} m. Kolor = prędkość, kropka = początek okrążenia, strzałka = kierunek jazdy. ` +
    'Pozycja z kierunku (żyroskop) i drogi kół – okrążenia z buksowaniem lub uderzeniem (droga ±20 % od typowej) są pomijane w średniej.';
  let html = `<table><tr><th>#</th><th>czas</th><th>strata</th><th>droga</th><th>v maks.</th></tr>`;
  html += `<tr class="lap${sel < 0 ? ' sel' : ''}" data-lap="-1"><td colspan="5">tor średni · czas [s], droga [m], v [${unit}]</td></tr>`;
  laps.forEach((lp, j) => {
    const isBest = lp.good && lp.time_s === best;
    html += `<tr class="lap${lp.good ? '' : ' bad'}${sel === j ? ' sel' : ''}" data-lap="${j}"><td>${j + 1}</td>` +
      `<td class="${isBest ? 'best' : ''}">${fmt(lp.time_s, 2)}</td><td>${lp.good ? (isBest ? '★' : '+' + fmt(lp.time_s - best, 2)) : 'pominięte'}</td>` +
      `<td>${fmt(lp.dist_m, 1)}</td><td>${fmt(speedConv(lp.v_max), 1)}</td></tr>`;
  });
  $('raceLaps').innerHTML = html + '</table>';
  $('raceLaps').querySelectorAll('tr.lap').forEach((tr) => {
    tr.onclick = () => {
      const j = Number(tr.dataset.lap);
      state.raceLap = j;
      renderRaceTrack(res);
      if (j >= 0 && state.raceCharts) {
        const lp = laps[j];
        state.raceCharts.setRange(lp.t_start, lp.t_start + lp.time_s);
      } else setRaceRange('all');
    };
  });
  show(card, true);
}

function setRaceRange(r) {
  const rec = state.currentRace;
  if (!rec || !state.raceCharts) return;
  const t = getRaceAnalysis(rec).series.t;
  const t0 = Math.max(t[0], -1);
  if (r === 'all') state.raceCharts.setRange(t[0], t[t.length - 1]);
  else state.raceCharts.setRange(t0, t0 + Number(r));
}

async function saveRaceTag() {
  const rec = state.currentRace;
  if (!rec) return;
  const tag = $('raceTag').value.trim();
  await db.updateRace(rec.id, { tag, note: $('raceNote').value });
  rec.tag = tag;
  await db.pushRecentTag(tag);
  $('raceTitle').textContent = `Wyścig seq ${rec.seq}${tag ? ' – ' + tag : ''}`;
  toast('Zapisano');
}

function raceCsv(rec) {
  const s = getRaceAnalysis(rec).series;
  const cols = ['t', 'throttle_pct', 'v_front', 'rpm', 'd_front', 'a_long', 'a_lat', 'yaw_rate', 'heading_deg'];
  const lines = [cols.join(';')];
  for (let i = 0; i < s.t.length; i++) lines.push(cols.map((c) => Number(s[c][i]).toFixed(4)).join(';'));
  return lines.join('\n');
}

// ------------------------------------------------------------------ porównanie
const cmpSel = { tags: new Set(), runs: new Set() };

async function renderCompare() {
  const all = await db.allRuns();
  const tags = [...new Set(all.map((r) => r.tag || '(bez tagu)'))].sort();
  const tbox = $('cmpTags');
  tbox.innerHTML = '';
  for (const t of tags) {
    const b = document.createElement('button');
    b.textContent = t;
    b.classList.toggle('sel', cmpSel.tags.has(t));
    b.onclick = () => { cmpSel.tags.has(t) ? cmpSel.tags.delete(t) : cmpSel.tags.add(t); renderCompare(); };
    tbox.appendChild(b);
  }
  const rbox = $('cmpRuns');
  rbox.innerHTML = '';
  for (const r of all) {
    const l = document.createElement('label');
    l.innerHTML = `<input type="checkbox" ${cmpSel.runs.has(r.id) ? 'checked' : ''}> seq ${r.seq} · ${esc(r.tag || '–')} · ${runDate(r)}`;
    l.querySelector('input').onchange = (e) => { e.target.checked ? cmpSel.runs.add(r.id) : cmpSel.runs.delete(r.id); renderCompare(); };
    rbox.appendChild(l);
  }
  const chosen = all.filter((r) => cmpSel.runs.has(r.id) || cmpSel.tags.has(r.tag || '(bez tagu)'));
  // nałożone v_imu(t − t0)
  if (state.cmpCharts) { state.cmpCharts.destroy(); state.cmpCharts = null; }
  const grid = Float64Array.from({ length: 1601 }, (_, i) => -0.2 + i * 0.002);
  const series = chosen.slice(0, 16).map((r, i) => {
    const res = getAnalysis(r);
    const t0 = res.metrics.t_launch_s;
    const v = Float64Array.from(grid, (tt) => { const y = interpAt(res.series.t, res.series.v_imu, t0 + tt); return Number.isNaN(y) ? null : y; });
    return { label: `${r.tag || '–'} #${r.seq}`, values: v, color: PALETTE[i % PALETTE.length] };
  });
  if (series.length) state.cmpCharts = syncedCharts($('cmpChart'), grid, [{ label: 'v_imu', unit: 'm/s', height: 240, series }], { syncKey: 'cmp' });
  else $('cmpChart').innerHTML = '<p class="muted">Nic nie wybrano.</p>';
  // tabela tagów
  const keys = ['a_long_0_300ms', 'a_long_0_1000ms', 'v_imu_1_0s', 't_2m', 'peak_slip', 'spin_integral', 'v_max'];
  const byTag = new Map();
  for (const r of all) {
    const t = r.tag || '(bez tagu)';
    if (cmpSel.tags.size && !cmpSel.tags.has(t)) continue;
    if (!byTag.has(t)) byTag.set(t, []);
    byTag.get(t).push(getAnalysis(r).metrics);
  }
  let html = '<table><tr><th>tag</th><th>n</th>' + keys.map((k) => `<th>${METRIC_LABELS[k][0]}</th>`).join('') + '</tr>';
  for (const [t, ms] of byTag) {
    html += `<tr><td>${esc(t)}</td><td>${ms.length}</td>` + keys.map((k) => {
      const vals = ms.map((m) => m[k]).filter((v) => v != null && !Number.isNaN(v));
      if (!vals.length) return '<td>–</td>';
      const mean = vals.reduce((a, b) => a + b, 0) / vals.length;
      const sd = Math.sqrt(vals.reduce((a, b) => a + (b - mean) ** 2, 0) / vals.length);
      return `<td>${metricValue(k, mean)} ± ${metricValue(k, sd)}</td>`;
    }).join('') + '</tr>';
  }
  $('cmpTable').innerHTML = html + '</table>';
  // wykres punktowy
  const pts = all.map((r) => { const m = getAnalysis(r).metrics; return { x: m.peak_slip * 100, y: m.a_long_0_300ms, group: r.tag || '(bez tagu)' }; });
  const leg = scatterChart($('cmpScatter'), pts, { xLabel: 'szczyt poślizgu [%]', yLabel: 'a 0–0,3 s [m/s²]' }) || [];
  $('cmpLegend').innerHTML = leg.map((l) => `<span class="small" style="color:${l.color}">● ${esc(l.name)}</span>`).join(' ');
}

// ------------------------------------------------------------------ ustawienia
const AXES = ['+x', '-x', '+y', '-y', '+z', '-z'];
const DRIFT_FIELDS = [
  ['pts_per_m', 'Punkty za metr driftu'], ['pts_per_m_deg', 'Punkty za metr × stopień kąta'],
  ['min_angle_deg', 'Kąt od którego liczy się drift [°]'], ['chain_gap_s', 'Przerwa kończąca łańcuch [s]'],
  ['max_mult', 'Maks. mnożnik'], ['min_acc_g', 'Min. przyspieszenie boczne [g]'], ['min_v_mps', 'Min. prędkość [m/s]'],
  ['spin_angle_deg', 'Kąt obrotu (utrata) [°]'],
  ['full_thr_pct', 'Pełny gaz od [%] (dźwięk odcięcia i bonus)'], ['full_thr_s', 'Bonus po [s] pełnego gazu'],
];
const EST_FIELDS = [
  ['yaw_offset_deg', 'Kąt montażu w poziomie [°]'], ['zupt_ms', 'ZUPT – czas bezruchu [ms]'],
  ['zupt_gyro_dps', 'ZUPT – maks. ω [dps]'], ['zupt_acc_g', 'ZUPT – maks. ||a|−1| [g]'],
  ['slip_acc_thr', 'Próg |a_front−a_long| [m/s²]'], ['slip_v_thr', 'Próg |v_front−v| [m/s]'],
  ['grip_hold_ms', 'Czas warunku przyczepności [ms]'], ['wheel_corr_tau_s', 'Stała korekty z kół [s]'],
  ['brake_pct', 'Strefa hamowania gazu [%]'], ['afront_tau_s', 'Wygładzanie a_front [s]'],
  ['zupt_lpf_tau_s', 'Filtr bezruchu ZUPT [s]'],
];

function renderSettings() {
  renderProfiles();
  const s = state.settings;
  $('setFdr').value = s.final_drive_ratio;
  $('setTire').value = s.front_tire_diameter_mm;
  for (const [id, key] of [['setAxF', 'forward'], ['setAxL', 'left'], ['setAxU', 'up']]) {
    const el = $(id);
    el.innerHTML = AXES.map((a) => `<option>${a}</option>`).join('');
    el.value = s.imu_axes[key];
  }
  $('setLp').value = s.lowpass_hz;
  $('setVmin').value = s.v_min_mps;
  $('setRaceLp').value = s.race_lowpass_hz;
  $('setDrift').innerHTML = DRIFT_FIELDS.map(([k, l]) => `<label>${l}<input data-drift="${k}" type="number" step="any" inputmode="decimal" value="${s.drift[k]}"></label>`).join('');
  $('setDriftSpin').checked = !!s.drift.spin_loses_chain;
  $('setSteerSign').value = String(s.drift.steer_sign || 0);
  $('setCsReq').checked = !!s.drift.cs_required;
  $('setFullBonus').checked = !!s.drift.full_bonus;
  $('setSound').checked = !!soundPref.on;
  $('setTurbo').checked = !!soundPref.turbo;
  $('setVolume').value = soundPref.volume;
  $('setMount').textContent = `Montaż płytki: do przodu ${s.imu_axes.forward}, w lewo ${s.imu_axes.left}, w górę ${s.imu_axes.up}, ` +
    `kąt ${fmt(s.estimator.yaw_offset_deg, 1)}° – kalibracja w szczegółach wyścigu (jazda bez driftu). ` +
    'Znak skrętu i montaż trafiają do Xiao po połączeniu; „wymaga kontry” Xiao bierze z firmware.';
  $('setEst').innerHTML = EST_FIELDS.map(([k, l]) => `<label>${l}<input data-est="${k}" type="number" step="any" inputmode="decimal" value="${s.estimator[k]}"></label>`).join('');
  $('setTheme').value = localStorage.getItem('x4f_theme') || 'auto';
  $('appVersion').textContent = `X4F Drift Logger ${APP_VERSION} · analiza zgodna z tools/rc_launch/analyze.py i drift.py`;
}

async function saveSettings() {
  const num = (id) => parseFloat($(id).value);
  const s = mergeSettings(state.settings);
  s.final_drive_ratio = num('setFdr');
  s.front_tire_diameter_mm = num('setTire');
  s.imu_axes = { forward: $('setAxF').value, left: $('setAxL').value, up: $('setAxU').value };
  s.lowpass_hz = num('setLp');
  s.v_min_mps = num('setVmin');
  s.race_lowpass_hz = num('setRaceLp');
  document.querySelectorAll('[data-est]').forEach((el) => { s.estimator[el.dataset.est] = parseFloat(el.value); });
  document.querySelectorAll('[data-drift]').forEach((el) => { s.drift[el.dataset.drift] = parseFloat(el.value); });
  s.drift.spin_loses_chain = $('setDriftSpin').checked ? 1 : 0;
  s.drift.steer_sign = parseInt($('setSteerSign').value, 10) || 0;
  s.drift.cs_required = $('setCsReq').checked ? 1 : 0;
  s.drift.full_bonus = $('setFullBonus').checked ? 1 : 0;
  const old = state.settings;
  if (JSON.stringify(old.imu_axes) !== JSON.stringify(s.imu_axes) || old.estimator.yaw_offset_deg !== s.estimator.yaw_offset_deg ||
      old.drift.steer_sign !== s.drift.steer_sign) s.mount_set = true;
  if (DRIFT_FIELDS.some(([k]) => !(s.drift[k] >= 0))) { $('setMsg').textContent = 'Niepoprawne wartości punktów za drift.'; return; }
  const axesUsed = new Set(Object.values(s.imu_axes).map((a) => a[1]));
  if (axesUsed.size !== 3) { $('setMsg').textContent = 'Każda oś auta musi mieć inną oś płytki.'; return; }
  if (!(s.final_drive_ratio > 0 && s.front_tire_diameter_mm > 0 && s.lowpass_hz > 0 && s.race_lowpass_hz > 0)) { $('setMsg').textContent = 'Niepoprawne wartości.'; return; }
  await persistSettings(s);
  await sendMount();
  $('setMsg').textContent = 'Zapisano – wszystkie starty zostaną przeliczone.';
  toast('Zapisano ustawienia');
}

function applyTheme(t) {
  if (t === 'auto') document.documentElement.removeAttribute('data-theme');
  else document.documentElement.setAttribute('data-theme', t);
}


// ------------------------------------------------------------------ profile aut
// Profil = komplet ustawień (auto, osie i montaż IMU, estymator, punkty driftu, kierunek skrętu).
// Aktywny profil trafia do Xiao komendą MOUNT (montaż, znak skrętu, nazwa); Xiao zapisuje go
// na stałe (protokół ≥ 5) – po resecie pamięta ostatni, dopóki nie wybierzesz innego.
function activeProfile() { return state.profiles.find((p) => p.id === state.profileId) || state.profiles[0]; }

// Nazwa do MOUNT: bez spacji (→ _), bez " i \, maks. 23 bajty UTF-8
function profileWireName(name) {
  let n = String(name || 'auto').trim().replace(/\s+/g, '_').replace(/["\\]/g, '');
  const enc = new TextEncoder();
  while (enc.encode(n).length > 23) n = n.slice(0, -1);
  return n || 'auto';
}
const profileShowName = (wire) => String(wire || '').replace(/_/g, ' ');

async function loadProfiles() {
  let profiles = await db.kvGet('profiles');
  let id = await db.kvGet('profile_active');
  if (!Array.isArray(profiles) || !profiles.length) {
    const saved = await db.kvGet('settings');      // przed profilami: jedne ustawienia
    profiles = [{ id: newProfileId(), name: 'Moje auto', settings: mergeSettings(saved) }];
    id = profiles[0].id;
    await db.kvSet('profiles', profiles);
    await db.kvSet('profile_active', id);
  }
  state.profiles = profiles.map((p) => ({ ...p, settings: mergeSettings(p.settings) }));
  state.profileId = state.profiles.some((p) => p.id === id) ? id : state.profiles[0].id;
  state.settings = activeProfile().settings;
}

function newProfileId() { return `p${Date.now().toString(36)}${Math.random().toString(36).slice(2, 6)}`; }

async function persistSettings(s) {
  state.settings = s;
  state.analysisCache.clear();
  activeProfile().settings = s;
  await db.kvSet('profiles', state.profiles);
  await db.kvSet('settings', s);
}

async function selectProfile(id) {
  if (!state.profiles.some((p) => p.id === id)) id = state.profiles[0].id;
  state.profileId = id;
  await db.kvSet('profile_active', id);
  state.settings = activeProfile().settings;
  state.analysisCache.clear();
  await db.kvSet('settings', state.settings);
  await sendMount();
  renderSettings();
}

function renderProfiles() {
  const sel = $('profSel');
  sel.innerHTML = state.profiles.map((p) => `<option value="${esc(p.id)}">${esc(p.name)}</option>`).join('');
  sel.value = state.profileId;
  $('profName').value = '';
  $('profDelete').disabled = state.profiles.length < 2;
  const xm = state.xiaoMount || (state.info && state.info.mount ? { mount: state.info.mount, saved: !!state.info.mount_saved } : null);
  let txt;
  if (!state.device) txt = 'Po połączeniu aplikacja wyśle ten profil do Xiao.';
  else if (!state.info || (state.info.proto || 1) < 5) txt = 'Firmware Xiao nie zapamiętuje profilu (potrzebny fw 1.4) – montaż wysyłany przy każdym połączeniu.';
  else if (xm) txt = `Xiao pamięta: „${profileShowName(xm.mount[5]) || '(bez nazwy)'}”${xm.saved ? ' – zapisany, zostaje po wyłączeniu' : ' – zapis w toku'}.`;
  else txt = '';
  $('profXiao').textContent = txt;
}

async function profileNew() {
  const name = $('profName').value.trim() || `Auto ${state.profiles.length + 1}`;
  const p = { id: newProfileId(), name, settings: mergeSettings(state.settings) };
  state.profiles.push(p);
  await db.kvSet('profiles', state.profiles);
  await selectProfile(p.id);
  toast(`Profil „${name}” – kopia ustawień poprzedniego; skalibruj montaż w nowym aucie`);
}

async function profileRename() {
  const name = $('profName').value.trim();
  if (!name) { toast('Wpisz nową nazwę w polu obok'); return; }
  activeProfile().name = name;
  await db.kvSet('profiles', state.profiles);
  await sendMount();
  renderProfiles();
  toast('Zmieniono nazwę profilu');
}

async function profileDelete() {
  if (state.profiles.length < 2) return;
  const name = activeProfile().name;
  state.profiles = state.profiles.filter((p) => p.id !== state.profileId);
  await db.kvSet('profiles', state.profiles);
  await selectProfile(state.profiles[0].id);
  toast(`Usunięto profil „${name}”`);
}

// ------------------------------------------------------------------ synchronizacja (GitHub)
const syncConfigured = () => !!(state.sync && state.sync.repo && state.sync.token);

function setSyncMsg(t) { $('syncMsg').textContent = t; }

function renderSyncButtons() {
  for (const id of ['racesSync', 'runsSync']) {
    show($(id), syncConfigured());
    $(id).disabled = state.syncing;
  }
}

async function doSync({ quiet = false } = {}) {
  if (!syncConfigured()) { if (!quiet) toast('Ustaw synchronizację w Ustawieniach'); return; }
  if (state.syncing || state.downloading) return;
  if (!navigator.onLine) { if (!quiet) toast('Brak internetu – synchronizacja później'); return; }
  state.syncing = true;
  renderSyncButtons();
  setSyncMsg('Synchronizacja…');
  try {
    const st = await syncAll(new GitHubStore({ repo: state.sync.repo, token: state.sync.token }), db, {
      device: /Android|iPhone/i.test(navigator.userAgent) ? 'telefonu' : 'komputera', onProgress: setSyncMsg,
    });
    const parts = [];
    if (st.up) parts.push(`wysłano ${st.up}`);
    if (st.down) parts.push(`pobrano ${st.down}`);
    if (st.meta) parts.push(`zmiany opisów ${st.meta}`);
    if (st.deleted) parts.push(`usunięto ${st.deleted}`);
    const msg = parts.length ? `Zsynchronizowano: ${parts.join(', ')}.` : 'Wszystko zsynchronizowane.';
    setSyncMsg(`${msg} (${new Date().toLocaleTimeString('pl-PL')})`);
    if (!quiet || parts.length) toast(msg);
    if (parts.length) {
      if (currentView === 'runs') renderRunsList();
      if (currentView === 'races') renderRacesList();
    }
  } catch (e) {
    setSyncMsg(`Błąd synchronizacji: ${e.message}`);
    if (!quiet) toast(e.message, 4000);
  } finally {
    state.syncing = false;
    renderSyncButtons();
  }
}

async function saveSync() {
  const repo = $('syncRepo').value.trim().replace(/^https?:\/\/github\.com\//, '').replace(/\/$/, '');
  const token = $('syncToken').value.trim();
  if (repo && !/^[\w.-]+\/[\w.-]+$/.test(repo)) { setSyncMsg('Repozytorium w postaci właściciel/nazwa, np. migomat/x4f-dane.'); return; }
  state.sync = { repo, token, auto: $('syncAuto').checked };
  await db.kvSet('sync', state.sync);
  renderSyncButtons();
  setSyncMsg(syncConfigured() ? 'Zapisano.' : 'Zapisano – synchronizacja wyłączona (brak repozytorium albo tokenu).');
  if (syncConfigured()) doSync();
}

// ------------------------------------------------------------------ eksport / import bazy
const stripSync = (r) => { const o = { ...r }; delete o.sync_meta; delete o.sync_meta_sha; return o; };

async function exportDb() {
  const runs = await db.allRuns();
  const races = await db.allRaces();
  const out = {
    app: 'x4f-launch-logger', version: 1, exported_at: new Date().toISOString(), settings: state.settings,
    profiles: state.profiles, profile_active: state.profileId,
    recent_tags: await db.recentTags(),
    runs: runs.map((r) => ({ ...stripSync(r), raw: undefined, raw_b64: toBase64(new Uint8Array(r.raw)) })),
    races: races.map((r) => ({ ...stripSync(r), raw: undefined, raw_b64: toBase64(new Uint8Array(r.raw)) })),
  };
  const name = `x4f_baza_${new Date().toISOString().slice(0, 10)}.json`;
  await shareOrDownload(name, JSON.stringify(out), 'application/json');
  $('dbMsg').textContent = `Wyeksportowano ${runs.length} startów i ${races.length} wyścigów.`;
}

async function importDb(file) {
  try {
    const obj = JSON.parse(await file.text());
    if (obj.app !== 'x4f-launch-logger') throw new Error('To nie jest eksport X4F');
    let n = 0, bad = 0;
    for (const r of obj.runs || []) {
      try {
        const raw = fromBase64(r.raw_b64);
        parseRun(raw);   // weryfikacja CRC
        const rec = { ...stripSync(r), raw: raw.buffer };
        delete rec.raw_b64;
        await db.putRun(rec);
        n++;
      } catch (e) { bad++; }
    }
    let nr = 0;
    for (const r of obj.races || []) {
      try {
        const raw = fromBase64(r.raw_b64);
        parseRace(raw);   // weryfikacja CRC
        const rec = { ...stripSync(r), raw: raw.buffer };
        delete rec.raw_b64;
        await db.putRace(rec);
        nr++;
      } catch (e) { bad++; }
    }
    if (Array.isArray(obj.profiles) && obj.profiles.length) {
      for (const p of obj.profiles) {
        if (!p || !p.id || !p.settings) continue;
        const prof = { id: String(p.id), name: String(p.name || 'auto'), settings: mergeSettings(p.settings) };
        const i = state.profiles.findIndex((x) => x.id === prof.id);
        if (i >= 0) state.profiles[i] = prof; else state.profiles.push(prof);
      }
      await db.kvSet('profiles', state.profiles);
      await selectProfile(state.profileId);
    } else if (obj.settings) {
      await persistSettings(mergeSettings(obj.settings));
    }
    for (const t of (obj.recent_tags || []).reverse()) await db.pushRecentTag(t);
    $('dbMsg').textContent = `Zaimportowano ${n} startów i ${nr} wyścigów${bad ? `, odrzucono ${bad} uszkodzonych` : ''}.`;
    renderSettings();
  } catch (e) {
    $('dbMsg').textContent = `Błąd importu: ${e.message}`;
  }
}

// ------------------------------------------------------------------ start aplikacji
async function init() {
  applyTheme(localStorage.getItem('x4f_theme') || 'auto');
  await loadProfiles();

  document.querySelectorAll('nav.tabs button').forEach((b) => {
    b.onclick = () => { if (b.dataset.view === 'drift' && soundPref.on) limiter.unlock(); goto(b.dataset.view); };
  });
  $('btnConnect').onclick = () => connect('ble');
  $('btnReconnect').onclick = () => connect('ble', { reconnect: true });
  $('btnDemo').onclick = () => connect('demo');
  $('btnDisconnect').onclick = () => disconnect();
  $('btnDownload').onclick = () => downloadAll();
  $('tagSave').onclick = () => applyBatchTag($('tagInput').value);
  $('tagSkip').onclick = () => applyBatchTag('');
  $('liveToggle').onchange = async (e) => {
    state.live.on = e.target.checked;
    if (state.device) { try { await state.device.live(e.target.checked); } catch (x) { toast(x.message); } }
    if (!e.target.checked) $('liveBox').innerHTML = '';
  };
  $('btnDemoRefill').onclick = async () => { if (state.demo) { await state.transport.refill(); await refreshDevice(); } };
  $('btnDemoDrop').onclick = () => { if (state.demo) state.transport.dropConnection(); };

  $('spdUnit').onclick = () => {
    state.speedUnit = state.speedUnit === 'kmh' ? 'ms' : 'kmh';
    const u = state.speedUnit === 'kmh' ? 'km/h' : 'm/s';
    $('spdUnit').textContent = u;
    $('spdMaxUnit').textContent = u;
    try { localStorage.setItem('x4f_unit', state.speedUnit); } catch (e) { /* */ }
    if (state.live.data) renderSpeed(state.live.data);
  };
  try {
    if (localStorage.getItem('x4f_unit') === 'ms') $('spdUnit').click();
  } catch (e) { /* */ }
  $('spdReset').onclick = async () => {
    state.rpmMax = 0;
    if (state.device) { await state.device.vmaxReset(); toast('Wyzerowano prędkość i obroty maksymalne'); }
  };

  $('drReset').onclick = async () => {
    if (!state.device) return;
    try { await state.device.driftReset(); state.driftPrev = null; toast('Wyzerowano punkty'); } catch (e) { toast(e.message); }
  };
  $('raceCalib').onclick = () => calibrateFromRace();
  const renderSoundBtn = () => { $('drSound').textContent = soundPref.on ? '🔊 dźwięk' : '🔇 dźwięk'; };
  renderSoundBtn();
  $('drSound').onclick = () => {
    soundPref.on = !soundPref.on;
    if (soundPref.on) limiter.unlock(); else limiter.stop();
    saveSoundPref(); renderSoundBtn();
  };
  $('setTurbo').onchange = (e) => { soundPref.turbo = e.target.checked; limiter.setTurbo(soundPref.turbo); saveSoundPref(); };
  $('setSound').onchange = (e) => {
    soundPref.on = e.target.checked;
    if (soundPref.on) limiter.unlock(); else limiter.stop();
    saveSoundPref(); renderSoundBtn();
  };
  $('setVolume').oninput = (e) => { soundPref.volume = parseFloat(e.target.value); limiter.setVolume(soundPref.volume); saveSoundPref(); };
  $('setSoundTest').onclick = () => limiter.demo();
  $('raceCalibApply').onclick = () => applyCalibration();
  $('setDriftSave').onclick = () => saveSettings();
  $('profSel').onchange = (e) => selectProfile(e.target.value).then(() => toast(`Profil: ${activeProfile().name}`));
  $('profNew').onclick = () => profileNew();
  $('profRename').onclick = () => profileRename();
  armedButton($('profDelete'), 'Na pewno usunąć?', () => profileDelete());

  $('runsFilter').onchange = () => renderRunsList();
  $('runBack').onclick = () => goto('runs');
  document.querySelectorAll('[data-range]').forEach((b) => { b.onclick = () => setRunRange(b.dataset.range); });
  $('runTagSave').onclick = () => saveRunTag();
  $('runNote').onchange = () => saveRunTag();
  $('runExportCsv').onclick = () => {
    const r = state.currentRun;
    if (r) shareOrDownload(`x4f_seq${r.seq}${r.tag ? '_' + r.tag.replace(/\W+/g, '_') : ''}.csv`, runCsv(r), 'text/csv');
  };
  armedButton($('runDelete'), 'Na pewno usunąć?', async () => {
    const r = state.currentRun;
    if (!r) return;
    await db.deleteRun(r.id);
    await db.addTombstone('runs', r.id);
    state.analysisCache.delete(r.id);
    toast('Usunięto start');
    goto('runs');
  });

  armedButton($('btnRaceErase'), 'Na pewno skasować?', eraseRaces);
  $('raceBack').onclick = () => goto('races');
  document.querySelectorAll('[data-rrange]').forEach((b) => { b.onclick = () => setRaceRange(b.dataset.rrange); });
  $('raceTagSave').onclick = () => saveRaceTag();
  $('raceNote').onchange = () => saveRaceTag();
  $('raceExportCsv').onclick = () => {
    const r = state.currentRace;
    if (r) shareOrDownload(`x4f_wyscig_seq${r.seq}${r.tag ? '_' + r.tag.replace(/\W+/g, '_') : ''}.csv`, raceCsv(r), 'text/csv');
  };
  armedButton($('raceDelete'), 'Na pewno usunąć?', async () => {
    const r = state.currentRace;
    if (!r) return;
    await db.deleteRace(r.id);
    await db.addTombstone('races', r.id);
    state.analysisCache.delete('race:' + r.id);
    toast('Usunięto wyścig');
    goto('races');
  });

  $('setSave').onclick = () => saveSettings();
  $('setDefaults').onclick = () => { state.settings = mergeSettings(DEFAULT_SETTINGS); renderSettings(); $('setMsg').textContent = 'Przywrócono domyślne – kliknij Zapisz.'; };
  $('setTheme').onchange = (e) => { localStorage.setItem('x4f_theme', e.target.value); applyTheme(e.target.value); };
  $('dbExport').onclick = () => exportDb();
  $('syncSave').onclick = () => saveSync();
  $('syncNow').onclick = () => doSync();
  $('racesSync').onclick = () => doSync();
  $('runsSync').onclick = () => doSync();
  state.sync = (await db.kvGet('sync')) || null;
  $('syncRepo').value = state.sync ? state.sync.repo : 'migomat/x4f-dane';
  $('syncToken').value = state.sync ? state.sync.token : '';
  $('syncAuto').checked = state.sync ? !!state.sync.auto : true;
  renderSyncButtons();
  const last = await db.kvGet('sync_last');
  if (last) setSyncMsg(`Ostatnia synchronizacja: ${new Date(last).toLocaleString('pl-PL')}`);
  if (syncConfigured() && state.sync.auto) setTimeout(() => doSync({ quiet: true }), 1500);
  $('dbImport').onchange = (e) => { if (e.target.files[0]) importDb(e.target.files[0]); e.target.value = ''; };

  const bleOk = WebBluetoothTransport.available();
  if (!bleOk) {
    $('btnConnect').disabled = true;
    $('connInfo').textContent = 'Web Bluetooth niedostępny w tej przeglądarce – dostępny jest tryb demo. Na telefonie użyj Chrome (HTTPS).';
  } else if (navigator.bluetooth.getDevices) {
    try { show($('btnReconnect'), localStorage.getItem('x4f_paired') === '1'); } catch (e) { /* */ }
  }
  document.addEventListener('visibilitychange', () => {
    if (document.visibilityState === 'visible' && (LIVE_VIEWS.includes(currentView) || state.downloading)) wakeLock(true);
  });
  if ('serviceWorker' in navigator && location.protocol !== 'file:') {
    // Nowa wersja aplikacji przejęła stronę → jednorazowe przeładowanie (nie w trakcie pobierania)
    const hadController = !!navigator.serviceWorker.controller;
    let reloaded = false;
    navigator.serviceWorker.addEventListener('controllerchange', () => {
      if (!hadController || reloaded || state.downloading || state.device) return;
      reloaded = true;
      location.reload();
    });
    navigator.serviceWorker.register('sw.js').catch(() => {});
  }
  renderSettings();
  if (location.hash === '#demo') connect('demo');
}

init();
