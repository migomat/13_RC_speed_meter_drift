// Warstwa transportu. Interfejs:
//   Transport { connect(), send(cmd), onFrame(cb), onDisconnect(cb), disconnect(), connected }
// Tylko ten plik odwołuje się do navigator.bluetooth. Plan awaryjny:
// CapacitorBleTransport (@capacitor-community/bluetooth-le) z tym samym interfejsem.

import { crc32, runCrc, parseHeader, raceCrc, parseRace } from './format.js';
import { driftInit, driftStep, driftParams, driftResetScore, raceInputs } from './drift.js';
import { DEFAULT_SETTINGS } from './analysis.js';

export const NUS_SERVICE = '6e400001-b5a3-f393-e0a9-e50e24dcca9e';
export const NUS_RX = '6e400002-b5a3-f393-e0a9-e50e24dcca9e';
export const NUS_TX = '6e400003-b5a3-f393-e0a9-e50e24dcca9e';
const NAME_PREFIX = 'X4F';

export class TransportError extends Error {}

/** Czytelny komunikat po polsku dla błędów Web Bluetooth. */
export function bleErrorMessage(e) {
  const name = e && e.name, msg = (e && e.message) || String(e);
  if (!('bluetooth' in navigator)) {
    return 'Ta przeglądarka nie obsługuje Web Bluetooth. Użyj Chrome na Androidzie (strona musi być otwarta przez HTTPS).';
  }
  if (name === 'NotFoundError') {
    if (/cancel/i.test(msg)) return 'Anulowano wybór urządzenia.';
    return 'Nie znaleziono X4F-LOG. Sprawdź, czy Xiao jest zasilane i w zasięgu, a Bluetooth i lokalizacja są włączone.';
  }
  if (name === 'NotAllowedError' || name === 'SecurityError') {
    return 'Brak uprawnień do Bluetooth. Zezwól Chrome na dostęp do „Urządzeń w pobliżu” (Ustawienia → Aplikacje → Chrome → Uprawnienia) i spróbuj ponownie.';
  }
  if (name === 'NetworkError') return 'Połączenie Bluetooth zerwane. Podejdź bliżej auta i połącz ponownie.';
  if (/Bluetooth adapter not available|adapter is off|Bluetooth is turned off/i.test(msg)) {
    return 'Bluetooth jest wyłączony – włącz go w telefonie.';
  }
  if (/globally disabled|blocklist/i.test(msg)) return 'Web Bluetooth jest wyłączony w Chrome (chrome://flags).';
  return `Błąd Bluetooth: ${msg}`;
}

export class WebBluetoothTransport {
  constructor() {
    this.device = null;
    this.rx = null;
    this.tx = null;
    this.frameCb = () => {};
    this.discCb = () => {};
    this.connected = false;
    this.name = '';
    this.writeChunk = 180;   // bezpiecznie poniżej MTU−3 (Xiao: MTU 247)
    this._q = Promise.resolve();
  }

  static available() { return typeof navigator !== 'undefined' && 'bluetooth' in navigator; }

  onFrame(cb) { this.frameCb = cb; }
  onDisconnect(cb) { this.discCb = cb; }

  /** Musi być wywołane z gestu użytkownika (przycisk), chyba że reconnect=true. */
  async connect({ reconnect = false } = {}) {
    if (!WebBluetoothTransport.available()) throw new TransportError(bleErrorMessage(null));
    let dev = null;
    if (reconnect && navigator.bluetooth.getDevices) {
      const devs = await navigator.bluetooth.getDevices();
      dev = devs.find((d) => (d.name || '').startsWith(NAME_PREFIX)) || null;
    }
    if (!dev) {
      if (reconnect) throw new TransportError('Brak zapamiętanego urządzenia');
      dev = await navigator.bluetooth.requestDevice({
        filters: [{ namePrefix: NAME_PREFIX }, { services: [NUS_SERVICE] }],
        optionalServices: [NUS_SERVICE],
      });
    }
    this.device = dev;
    this.name = dev.name || 'X4F-LOG';
    dev.addEventListener('gattserverdisconnected', () => this._onDisc());
    const server = await dev.gatt.connect();
    const svc = await server.getPrimaryService(NUS_SERVICE);
    this.rx = await svc.getCharacteristic(NUS_RX);
    this.tx = await svc.getCharacteristic(NUS_TX);
    this.tx.addEventListener('characteristicvaluechanged', (ev) => {
      const v = ev.target.value;
      this.frameCb(new Uint8Array(v.buffer.slice(v.byteOffset, v.byteOffset + v.byteLength)));
    });
    await this.tx.startNotifications();
    this.connected = true;
  }

  _onDisc() {
    if (!this.connected) return;
    this.connected = false;
    this.discCb();
  }

  send(text) {
    const data = new TextEncoder().encode(text);
    // zapisy szeregowane – GATT nie pozwala na równoległe operacje
    this._q = this._q.then(async () => {
      if (!this.connected) throw new TransportError('Brak połączenia');
      for (let i = 0; i < data.length; i += this.writeChunk) {
        const part = data.subarray(i, i + this.writeChunk);
        if (this.rx.writeValueWithoutResponse) await this.rx.writeValueWithoutResponse(part);
        else await this.rx.writeValue(part);
      }
    });
    return this._q;
  }

  async disconnect() {
    this.connected = false;
    try { if (this.device && this.device.gatt.connected) this.device.gatt.disconnect(); } catch (e) { /* */ }
  }
}

// ---------------------------------------------------------------------------
// DemoTransport – wirtualne Xiao: te same komendy i ramki co firmware, starty
// z app/demo/*.bin (wektory testowe), syntetyczny przejazd dla LIVE.
export class DemoTransport {
  constructor({ baseUrl = './demo/', bytesPerSec = 30000 } = {}) {
    this.baseUrl = baseUrl;
    this.bps = bytesPerSec;
    this.frameCb = () => {};
    this.discCb = () => {};
    this.connected = false;
    this.name = 'X4F-DEMO';
    this.runs = [];         // {seq, data, crc, unix, boot, uptime_ms}
    this.nextSeq = 101;
    this.timeSet = false;
    this.liveTimer = null;
    this.vmax = 0;
    this.t0 = performance.now();
    this.overwritten = 0;
    this.mtuPayload = 243;
    this.races = [];        // {seq, data, crc, unix, boot, uptime_ms, n, end, read}
    this.raceState = 'IDLE';
    this.driftRace = null;  // nagranie z driftem odtwarzane w LIVE (punkty jak na Xiao)
    this.drift = driftInit();
    this.driftI = 0;
    this.mount = ['+x', '+y', '+z', 0];
  }

  onFrame(cb) { this.frameCb = cb; }
  onDisconnect(cb) { this.discCb = cb; }

  async loadRuns() {
    const idx = await (await fetch(this.baseUrl + 'index.json')).json();
    this.runs = [];
    let k = 0;
    for (const name of idx) {
      const raw = new Uint8Array(await (await fetch(this.baseUrl + name + '.bin')).arrayBuffer());
      this.runs.push(this._wrap(raw, k++));
    }
    await this._loadRaces();
    try {
      const idx = await (await fetch(this.baseUrl + 'drift.json')).json();
      const raw = new Uint8Array(await (await fetch(this.baseUrl + idx[0] + '.bin')).arrayBuffer());
      const race = parseRace(raw);
      this.driftRace = { race, inp: raceInputs(race, DEFAULT_SETTINGS) };
    } catch (e) { this.driftRace = null; }
  }

  async _loadRaces() {
    let idx = [];
    try { idx = await (await fetch(this.baseUrl + 'races.json')).json(); } catch (e) { return; }
    for (const name of idx) {
      const raw = new Uint8Array(await (await fetch(this.baseUrl + name + '.bin')).arrayBuffer());
      this.races.push(this._wrapRace(raw, this.races.length));
    }
  }

  // Wyścig z nowym seq i przeliczonym CRC (trailer na końcu pliku).
  _wrapRace(raw, k) {
    const data = raw.slice();
    const dv = new DataView(data.buffer);
    const seq = this.nextSeq++;
    dv.setUint32(8, seq, true);
    dv.setUint32(12, 0, true);
    dv.setUint32(16, 7, true);
    dv.setUint32(20, 30000 + k * 40000, true);
    const crc = raceCrc(data);
    dv.setUint32(data.length - 4, crc, true);
    const r = parseRace(data);
    return { seq, data, crc, unix: 0, boot: 7, uptime_ms: r.header.uptime_ms, n: r.n, end: r.end_reason, read: false };
  }

  _raceUsed() { return this.races.reduce((a, r) => a + Math.ceil((r.data.length + 32) / 256) * 256, 0); }

  // Nadaje kolejny seq (jak nowe starty z urządzenia), zachowując poprawne CRC.
  _wrap(raw, k) {
    const data = raw.slice();
    const dv = new DataView(data.buffer);
    const seq = this.nextSeq++;
    dv.setUint32(8, seq, true);
    dv.setUint32(12, 0, true);                 // czas nieustawiony – przeliczenie z uptime
    dv.setUint32(16, 7, true);                 // boot_count
    dv.setUint32(20, 60000 + k * 9000, true);  // uptime_ms
    dv.setUint32(124, 0, true);
    dv.setUint32(124, runCrc(data), true);
    const h = parseHeader(data);
    return { seq, data, crc: h.crc32, unix: 0, boot: 7, uptime_ms: h.uptime_ms };
  }

  async connect() {
    if (!this.runs.length) await this.loadRuns();
    await sleep(300);
    this.connected = true;
    this.t0 = performance.now() - 120000;
  }

  async disconnect() {
    this.connected = false;
    this._stopLive();
    this.discCb();
  }

  /** Symulacja zerwania połączenia (test wznawiania pobierania). */
  dropConnection() {
    this.connected = false;
    this._stopLive();
    this.discCb();
  }

  /** Nowa seria startów demo (jak kolejne starty nagrane przez Xiao). */
  async refill() {
    const idx = await (await fetch(this.baseUrl + 'index.json')).json();
    for (const name of idx) {
      const raw = new Uint8Array(await (await fetch(this.baseUrl + name + '.bin')).arrayBuffer());
      this.runs.push(this._wrap(raw, this.runs.length));
    }
    await this._loadRaces();
    try {
      const idx = await (await fetch(this.baseUrl + 'drift.json')).json();
      const raw = new Uint8Array(await (await fetch(this.baseUrl + idx[0] + '.bin')).arrayBuffer());
      const race = parseRace(raw);
      this.driftRace = { race, inp: raceInputs(race, DEFAULT_SETTINGS) };
    } catch (e) { this.driftRace = null; }
  }

  async _emit(type, bytes) {
    for (let i = 0; i < bytes.length; i += this.mtuPayload) {
      if (!this.connected) return;
      const part = bytes.subarray(i, i + this.mtuPayload);
      const pkt = new Uint8Array(part.length + 1);
      pkt[0] = type;
      pkt.set(part, 1);
      this.frameCb(pkt);
      if (this.bps) await sleep((pkt.length / this.bps) * 1000);
    }
  }

  _text(s) { return this._emit(0x01, new TextEncoder().encode(s + '\n')); }

  uptime() { return Math.round(performance.now() - this.t0); }

  async send(text) {
    if (!this.connected) throw new TransportError('Brak połączenia');
    await sleep(15);
    const [cmd, ...args] = text.trim().split(/\s+/);
    switch (cmd) {
      case 'INFO':
        return this._text(JSON.stringify({ fw: '1.2.0-demo', format: 1, proto: 3, hall_lines: 1, pole_pairs: 1,
          edges_per_rev: 2, slot_size: 131072, slots: 8, sample_rate: 500, get_block_max: 4096, name: this.name,
          race_rate: 20, race_area: 1048576, race_sample_size: 20, drift: 1, gyro_range: 1000, mount: this.mount }));
      case 'STATUS':
        return this._text(JSON.stringify({ state: 'IDLE', neutral_us: 1500, neutral_learned: true, throttle_ok: true,
          runs: this.runs.length, free_slots: Math.max(0, 8 - this.runs.length), overwritten_unread: this.overwritten,
          time_set: this.timeSet, uptime_ms: this.uptime(), boot: 7, connected: true,
          race_state: this.raceState, races: this.races.length, races_unread: this.races.filter((r) => !r.read).length,
          race_used: this._raceUsed(), race_size: 1048576, race_rec_s: 0 }));
      case 'TIME':
        this.timeSet = true;
        return this._text('OK');
      case 'LIST':
        return this._text(JSON.stringify({ runs: this.runs.map((r) => ({ seq: r.seq, len: r.data.length, crc: r.crc,
          unix: r.unix, boot: r.boot, uptime_ms: r.uptime_ms })) }));
      case 'RLIST':
        return this._text(JSON.stringify({ races: this.races.map((r) => ({ seq: r.seq, len: r.data.length, crc: r.crc,
          unix: r.unix, boot: r.boot, uptime_ms: r.uptime_ms, n: r.n, end: r.end, read: r.read })),
        state: this.raceState, used: this._raceUsed(), size: 1048576 }));
      case 'RDEL': {
        const seq = Number(args[0]);
        const r = this.races.find((x) => x.seq === seq);
        if (!r) return this._text(`{"err":"NOT_FOUND","seq":${seq}}`);
        r.read = true;
        return this._text(`OK RDEL ${seq}`);
      }
      case 'RERASE': {
        const unread = this.races.filter((r) => !r.read).length;
        if (unread && args[0] !== 'FORCE') return this._text(`{"err":"UNREAD","unread":${unread},"state":"${this.raceState}"}`);
        this.races = [];
        this.raceState = 'ERASING';
        setTimeout(() => { this.raceState = 'IDLE'; }, 600);
        return this._text('OK');
      }
      case 'RSTOP': return this._text('OK');
      case 'GET':
      case 'RGET': {
        const [seq, off, len] = args.map(Number);
        const r = (cmd === 'RGET' ? this.races : this.runs).find((x) => x.seq === seq);
        if (!r) return this._text('{"err":"NOT_FOUND"}');
        if (off >= r.data.length) return this._text('{"err":"BAD_OFFSET"}');
        const n = Math.min(len, 4096, r.data.length - off);
        const data = r.data.subarray(off, off + n);
        const fr = new Uint8Array(20 + n);
        const dv = new DataView(fr.buffer);
        dv.setUint32(0, 0x42463458, true); dv.setUint32(4, seq, true); dv.setUint32(8, off, true);
        dv.setUint32(12, n, true); dv.setUint32(16, crc32(data), true);
        fr.set(data, 20);
        return this._emit(0x02, fr);
      }
      case 'DEL': {
        const seq = Number(args[0]);
        const i = this.runs.findIndex((x) => x.seq === seq);
        if (i < 0) return this._text(`{"err":"NOT_FOUND","seq":${seq}}`);
        this.runs.splice(i, 1);
        return this._text(`OK DEL ${seq}`);
      }
      case 'ARM': return this._text('OK');
      case 'VMAX':
        this.vmax = 0;
        return this._text('OK');
      case 'DRIFT':
        driftResetScore(this.drift);
        return this._text('OK');
      case 'MOUNT':
        if (args.length >= 4) this.mount = [args[0], args[1], args[2], +parseFloat(args[3]).toFixed(1)];
        return this._text(JSON.stringify({ mount: this.mount }));
      case 'LIVE':
        if (args[0] === '1') this._startLive(); else this._stopLive();
        return this._text('OK');
      default:
        return this._text('{"err":"UNKNOWN"}');
    }
  }

  // Syntetyczny przejazd: postój, start z buksowaniem, jazda, hamowanie.
  _startLive() {
    this._stopLive();
    const t0 = performance.now();
    this.liveTimer = setInterval(() => {
      const t = ((performance.now() - t0) / 1000) % 14;
      let v = 0, vf = 0, thr = 1500, slip = 0, ax = 0;
      if (t < 2) { v = 0; vf = 0; }
      else if (t < 7) {
        const x = t - 2;
        v = 12 * (1 - Math.exp(-x / 1.6));
        ax = (12 / 1.6) * Math.exp(-x / 1.6) / 9.81;
        const over = x < 0.5 ? 3.5 * Math.sin(Math.PI * x / 0.5) ** 2 : 0;
        vf = v * 1.02 + over;
        slip = over > 0.5 ? 1 : 0;
        thr = 1950;
      } else if (t < 10) { v = 12 * (1 - Math.exp(-5 / 1.6)); vf = v * 1.01; thr = 1800; }
      else if (t < 12) { const x = t - 10; v = Math.max(0, 11.5 - 6 * x); vf = v; thr = 1250; ax = -0.6; }
      v += (Math.random() - 0.5) * 0.05;
      v = Math.max(0, v);
      this.vmax = Math.max(this.vmax, v);
      const msg = { live: 1, v: +v.toFixed(3), v_max: +this.vmax.toFixed(3), v_front: +vf.toFixed(3), slip,
        rpm: Math.round(vf / (Math.PI * 0.064) * 5 * 60), thr, thr_pct: (thr - 1500) / 5, ax: +ax.toFixed(3),
        zupt: v === 0 ? 1 : 0, state: thr > 1600 && t < 5.5 ? 'RECORDING' : 'IDLE',
        race: t >= 2 && t < 12 ? 'RECORDING' : 'IDLE', race_s: Math.max(0, Math.round(t - 2)) };
      Object.assign(msg, this._driftTick());
      this._text(JSON.stringify(msg));
    }, 50);
  }

  // Kolejna próbka nagrania z driftem (20 Hz = tempo LIVE) przez ten sam algorytm co Xiao.
  _driftTick() {
    const d = this.driftRace;
    if (!d) return {};
    const { dt, af, al, gu, rest } = d.inp;
    const i = this.driftI;
    this.driftI = (i + 1) % dt.length;
    driftStep(this.drift, driftParams(DEFAULT_SETTINGS), dt[i], af[i], al[i], gu[i], rest[i]);
    const s = this.drift;
    return { d: [Math.round(s.pts), s.mult, Math.round(s.total), Math.round(s.last), Math.round(s.best),
      s.corner ? Math.round(s.angle) : 0, s.in_drift ? 1 : 0, s.chain ? 1 : 0, s.n_banks, s.n_lost, s.n_trans] };
  }

  _stopLive() {
    if (this.liveTimer) clearInterval(this.liveTimer);
    this.liveTimer = null;
  }
}

function sleep(ms) { return new Promise((r) => setTimeout(r, ms)); }
