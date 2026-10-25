// Dźwięk silnika na odcięciu + turbina – Web Audio.
//
// 1. Własne nagranie (pierwszeństwo): app/sounds/limiter.(mp3|ogg|wav) – zapętlany klip odcięcia,
//    opcjonalnie app/sounds/blowoff.(mp3|ogg|wav) – zawór upustowy po puszczeniu gazu.
// 2. Bez plików – synteza: odcięcie składane z pojedynczych zapłonów (rzędowa czwórka ~9000 rpm,
//    zapłon co 3,3 ms), ogranicznik przerywa serię zapłonów ~14 razy na sekundę, strzały z wydechu,
//    rezonans wydechu (filtry), turbina: gwizd sprężarki rosnący z doładowaniem i zawór upustowy
//    z „ćwierkaniem” sprężarki (flutter) po puszczeniu gazu.
// AudioContext można uruchomić tylko po geście użytkownika – unlock() z obsługi kliknięcia.

const RPM = 9000;
const FIRE_HZ = RPM / 60 * 2;      // rzędowa czwórka: 2 zapłony na obrót → 300 Hz
const CUT_PERIOD = 0.072;          // [s] cykl ogranicznika
const CUT_ON = 0.046;              // [s] część cyklu z zapłonami
const LOOP_CYCLES = 28;            // pętla 28 cykli ≈ 2 s (bez słyszalnego szwu)

function biquad(x, sr, type, f, q) {
  // jednorazowe filtrowanie bufora (RBJ cookbook)
  const w = 2 * Math.PI * f / sr, cw = Math.cos(w), al = Math.sin(w) / (2 * q);
  let b0, b1, b2;
  if (type === 'bp') { b0 = al; b1 = 0; b2 = -al; } else { b0 = (1 - cw) / 2; b1 = 1 - cw; b2 = (1 - cw) / 2; }
  const a0 = 1 + al, a1 = -2 * cw, a2 = 1 - al;
  const y = new Float32Array(x.length);
  let x1 = 0, x2 = 0, y1 = 0, y2 = 0;
  for (let i = 0; i < x.length; i++) {
    const v = (b0 * x[i] + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2) / a0;
    x2 = x1; x1 = x[i]; y2 = y1; y1 = v; y[i] = v;
  }
  return y;
}

// Zapętlany klip odcięcia z pojedynczych zapłonów
function synthLimiter(sr) {
  const n = Math.round(CUT_PERIOD * LOOP_CYCLES * sr);
  const raw = new Float32Array(n);
  let rnd = 12345;
  const rand = () => { rnd = (rnd * 1103515245 + 12345) & 0x7fffffff; return rnd / 0x7fffffff; };
  for (let c = 0; c < LOOP_CYCLES; c++) {
    const t0 = c * CUT_PERIOD;
    // zapłony: obroty rosną do odcięcia
    let t = t0, k = 0;
    while (t < t0 + CUT_ON * (0.92 + 0.16 * rand())) {
      const f = FIRE_HZ * (0.965 + 0.05 * (t - t0) / CUT_ON);
      const amp = 0.8 + 0.2 * ((k % 4) === 0 ? 1 : rand()) ;            // nierówność cylindrów
      const i0 = Math.round(t * sr);
      for (let j = 0; j < sr * 0.004 && i0 + j < n; j++) {
        const e = Math.exp(-j / (sr * 0.0009));
        raw[i0 + j] += amp * e * (0.75 + 0.5 * (rand() - 0.5));
      }
      t += 1 / f; k++;
    }
    // strzał z wydechu na odcięciu (niespalone paliwo)
    if (rand() < 0.55) {
      const i0 = Math.round((t + 0.004 + 0.01 * rand()) * sr);
      const amp = 0.9 + 0.8 * rand();
      for (let j = 0; j < sr * 0.012 && i0 + j < n; j++) raw[i0 + j] += amp * (rand() * 2 - 1) * Math.exp(-j / (sr * 0.003));
    }
  }
  // rezonans wydechu: dudnienie + chrapliwość + „wycie” dolotu (VTEC-owe wyższe rzędy)
  const low = biquad(raw, sr, 'bp', 320, 1.6);
  const mid = biquad(raw, sr, 'bp', 1150, 1.2);
  const high = biquad(raw, sr, 'bp', 2600, 2.5);
  const out = new Float32Array(n);
  let mx = 0;
  for (let i = 0; i < n; i++) {
    const v = Math.tanh(2.2 * (1.0 * low[i] + 0.7 * mid[i] + 0.35 * high[i]));
    out[i] = v;
    mx = Math.max(mx, Math.abs(v));
  }
  for (let i = 0; i < n; i++) out[i] /= mx || 1;
  // łagodne łączenie końca z początkiem pętli
  const fade = Math.round(0.004 * sr);
  for (let i = 0; i < fade; i++) { const w = i / fade; out[i] = out[i] * w + out[n - fade + i] * (1 - w); }
  return out;
}

// Zawór upustowy z flutterem sprężarki
function synthBlowoff(sr) {
  const n = Math.round(0.75 * sr);
  const raw = new Float32Array(n);
  let rnd = 777;
  const rand = () => { rnd = (rnd * 1103515245 + 12345) & 0x7fffffff; return rnd / 0x7fffffff; };
  for (let i = 0; i < n; i++) {
    const t = i / sr;
    const env = Math.min(1, t / 0.01) * Math.exp(-t / 0.22);
    const flutter = 0.55 + 0.45 * Math.sin(2 * Math.PI * (24 - 10 * t) * t);   // „stu-tu-tu”
    raw[i] = (rand() * 2 - 1) * env * flutter;
  }
  const a = biquad(raw, sr, 'bp', 3200, 0.9), b = biquad(raw, sr, 'bp', 6500, 1.5);
  const out = new Float32Array(n);
  for (let i = 0; i < n; i++) out[i] = 0.9 * a[i] + 0.5 * b[i];
  return out;
}

export class LimiterSound {
  constructor() {
    this.ctx = null;
    this.on = false;
    this.volume = 0.6;
    this.turbo = true;
    this.boost = 0;          // 0..1 – doładowanie (wolno narasta z gazem)
    this.lastThr = 0;
  }

  unlock() {
    const AC = window.AudioContext || window.webkitAudioContext;
    if (!AC) return false;
    if (!this.ctx) this._build(new AC());
    if (this.ctx.state === 'suspended') this.ctx.resume();
    return true;
  }

  async _loadFile(base) {
    for (const ext of ['mp3', 'ogg', 'wav']) {
      try {
        const r = await fetch(`sounds/${base}.${ext}`, { cache: 'no-store' });
        if (!r.ok) continue;
        return await this.ctx.decodeAudioData(await r.arrayBuffer());
      } catch (e) { /* brak pliku albo format nieobsługiwany */ }
    }
    return null;
  }

  _build(ctx) {
    this.ctx = ctx;
    const sr = ctx.sampleRate;
    this.master = ctx.createGain();
    this.master.gain.value = this.volume;
    this.master.connect(ctx.destination);
    // odcięcie
    this.limGain = ctx.createGain();
    this.limGain.gain.value = 0;
    this.limGain.connect(this.master);
    const mk = (data) => { const b = ctx.createBuffer(1, data.length, sr); b.copyToChannel(data, 0); return b; };
    this.limBuf = mk(synthLimiter(sr));
    this.boBuf = mk(synthBlowoff(sr));
    this.custom = false;
    this._startLoop();
    // własne nagrania (podmiana, gdy są)
    this._loadFile('limiter').then((b) => { if (b) { this.limBuf = b; this.custom = true; this._startLoop(); } });
    this._loadFile('blowoff').then((b) => { if (b) this.boBuf = b; });
    // turbina: gwizd sprężarki (dwa bliskie tony – dudnienie), wysokość i głośność z doładowania
    this.turboGain = ctx.createGain();
    this.turboGain.gain.value = 0;
    this.turboGain.connect(this.master);
    this.whistle = [7200, 7650].map((f) => {
      const o = ctx.createOscillator(); o.type = 'sine'; o.frequency.value = f;
      const g = ctx.createGain(); g.gain.value = 0.5;
      o.connect(g); g.connect(this.turboGain); o.start();
      return o;
    });
  }

  _startLoop() {
    if (this.src) { try { this.src.stop(); } catch (e) { /* */ } }
    const s = this.ctx.createBufferSource();
    s.buffer = this.limBuf; s.loop = true;
    s.connect(this.limGain); s.start();
    this.src = s;
  }

  setVolume(v) {
    this.volume = Math.max(0, Math.min(1, v));
    if (this.master) this.master.gain.setTargetAtTime(this.volume, this.ctx.currentTime, 0.05);
  }

  setTurbo(on) { this.turbo = !!on; if (!on && this.turboGain) this.turboGain.gain.setTargetAtTime(0, this.ctx.currentTime, 0.05); }

  /**
   * Wywoływane przy każdej linii LIVE (20 Hz): thrPct – gaz [%], fullPct – próg pełnego gazu z profilu,
   * enabled – dźwięk włączony i jest sygnał. Odcięcie od razu po przekroczeniu progu.
   */
  update(thrPct, fullPct, enabled) {
    if (!this.ctx) return;
    const t = this.ctx.currentTime;
    const lim = enabled && thrPct >= fullPct;
    if (lim !== this.on) {
      this.on = lim;
      this.limGain.gain.setTargetAtTime(lim ? 1 : 0, t, lim ? 0.015 : 0.04);
    }
    if (!this.turbo || !enabled) {
      this.turboGain.gain.setTargetAtTime(0, t, 0.05);
      this.boost = 0;
      this.lastThr = enabled ? thrPct : 0;
      return;
    }
    // doładowanie narasta z gazem (~0,6 s), opada szybciej
    const target = Math.max(0, Math.min(1, thrPct / Math.max(fullPct, 1)));
    this.boost += (target > this.boost ? 0.08 : 0.2) * (target - this.boost);
    for (const [i, o] of this.whistle.entries()) o.frequency.setTargetAtTime((4800 + 3600 * this.boost) * (i ? 1.06 : 1), t, 0.08);
    this.turboGain.gain.setTargetAtTime(0.10 * this.boost * this.boost, t, 0.08);
    // zawór upustowy: szybkie puszczenie gazu przy dużym doładowaniu
    if (this.lastThr - thrPct > 25 && this.boost > 0.45) this.blowoff(this.boost);
    this.lastThr = thrPct;
  }

  blowoff(level = 1) {
    if (!this.ctx || !this.turbo) return;
    const s = this.ctx.createBufferSource();
    s.buffer = this.boBuf;
    const g = this.ctx.createGain();
    g.gain.value = 0.55 * level;
    s.connect(g); g.connect(this.master);
    s.start();
    this.boost *= 0.3;
  }

  /** Wyłączenie wszystkiego (wyjście z zakładki, brak sygnału). */
  stop() { this.update(0, 100, false); this.lastThr = 0; }

  /** Test z ustawień: 2 s odcięcia, potem zawór upustowy. */
  demo() {
    if (!this.unlock()) return;
    let k = 0;
    const id = setInterval(() => {
      k++;
      const thr = k < 8 ? k * 12 : k < 48 ? 100 : 0;
      this.update(thr, 95, true);
      if (k > 60) { clearInterval(id); this.stop(); }
    }, 50);
  }
}
