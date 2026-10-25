// Dźwięk silnika jak w grach – Web Audio.
//
// Silnik gra cały czas (gdy dźwięk włączony i jest sygnał z Xiao): obroty idą za gazem – mało gazu =
// niskie obroty, pełny gaz (próg z profilu) = ogranicznik odcina zapłony. Po puszczeniu gazu: strzały
// z wydechu (hamowanie silnikiem) i turbina – syk zaworu upustowego z „ćwierkaniem” sprężarki;
// przy gazie gwizd sprężarki rosnący z doładowaniem.
// Silnik: AudioWorklet (engine-worklet.js) składa dźwięk z pojedynczych zapłonów rzędowej czwórki,
// za nim przester i rezonanse wydechu. Bez AudioWorklet – zapętlony klip odcięcia (synteza poniżej).
// Własne nagrania: app/sounds/limiter.(mp3|ogg|wav) – zamiast syntezy na odcięciu,
// app/sounds/blowoff.(mp3|ogg|wav) – zawór upustowy.
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


const IDLE_RPM = 1400;
const REDLINE_RPM = 9000;
const REV_UP = 9000;      // [rpm/s] narastanie obrotów (lekki silnik RC-owo-gierkowy)
const REV_DOWN = 6000;    // [rpm/s] opadanie

export class LimiterSound {
  constructor() {
    this.ctx = null;
    this.volume = 0.6;
    this.turbo = true;
    this.rpm = IDLE_RPM;
    this.boost = 0;
    this.lastThr = 0;
    this.crackle = 0;
    this.active = false;
    this.lastT = 0;
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
    this.master.gain.value = 0;
    this.master.connect(ctx.destination);
    const mk = (data) => { const b = ctx.createBuffer(1, data.length, sr); b.copyToChannel(data, 0); return b; };
    this.boBuf = mk(synthBlowoff(sr));
    if (!this.offline) this._loadFile('blowoff').then((b) => { if (b) this.boBuf = b; });
    // tor wydechu: przester + rezonanse (dudnienie, chrapliwość, wycie dolotu)
    this.engineIn = ctx.createGain();
    const shaper = ctx.createWaveShaper();
    const curve = new Float32Array(2048);
    for (let i = 0; i < curve.length; i++) { const x = i / 1023.5 - 1; curve[i] = Math.tanh(2.5 * x); }
    shaper.curve = curve;
    this.engineIn.connect(shaper);
    this.engineOut = ctx.createGain();
    for (const [f, q, g] of [[300, 1.4, 1.0], [1100, 1.1, 0.7], [2700, 2.2, 0.35]]) {
      const bp = ctx.createBiquadFilter(); bp.type = 'bandpass'; bp.frequency.value = f; bp.Q.value = q;
      const gg = ctx.createGain(); gg.gain.value = g;
      shaper.connect(bp); bp.connect(gg); gg.connect(this.engineOut);
    }
    this.engineOut.connect(this.master);
    // strzały z wydechu: bez przesteru, „bang” 500–1500 Hz + trochę wysokich (trzask)
    this.popIn = ctx.createGain();
    this.popIn.gain.value = 1;
    const pb = ctx.createBiquadFilter(); pb.type = 'bandpass'; pb.frequency.value = 850; pb.Q.value = 0.7;
    const ph = ctx.createBiquadFilter(); ph.type = 'highpass'; ph.frequency.value = 2500;
    const pg1 = ctx.createGain(); pg1.gain.value = 2.2;
    const pg2 = ctx.createGain(); pg2.gain.value = 0.5;
    this.popIn.connect(pb); pb.connect(pg1); pg1.connect(this.master);
    this.popIn.connect(ph); ph.connect(pg2); pg2.connect(this.master);
    this.node = null;
    if (ctx.audioWorklet) {
      this.ready = ctx.audioWorklet.addModule(this.workletUrl || 'js/engine-worklet.js').then(() => {
        this.node = new AudioWorkletNode(ctx, 'x4f-engine', { numberOfOutputs: 2, outputChannelCount: [1, 1] });
        this.node.connect(this.engineIn, 0);
        this.node.connect(this.popIn, 1);
      }).catch(() => this._fallback());
    } else {
      this._fallback();
      this.ready = Promise.resolve();
    }
    // własny klip odcięcia
    this.limGain = ctx.createGain();
    this.limGain.gain.value = 0;
    this.limGain.connect(this.master);
    if (!this.offline) this._loadFile('limiter').then((b) => { if (b) this._startLimClip(b); });
    // turbina: gwizd sprężarki
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

  // Bez AudioWorklet: tylko zapętlony klip odcięcia
  _fallback() {
    const b = this.ctx.createBuffer(1, 1, this.ctx.sampleRate);
    const data = synthLimiter(this.ctx.sampleRate);
    const buf = this.ctx.createBuffer(1, data.length, this.ctx.sampleRate);
    buf.copyToChannel(data, 0);
    void b;
    this.fallbackOnly = true;
    this._startLimClip(buf);
  }

  _startLimClip(buf) {
    if (this.limSrc) { try { this.limSrc.stop(); } catch (e) { /* */ } }
    const s = this.ctx.createBufferSource();
    s.buffer = buf; s.loop = true;
    s.connect(this.limGain); s.start();
    this.limSrc = s;
    this.customLimiter = !this.fallbackOnly;
  }

  setVolume(v) { this.volume = Math.max(0, Math.min(1, v)); }

  setTurbo(on) { this.turbo = !!on; if (!on && this.turboGain) this.turboGain.gain.setTargetAtTime(0, this.ctx.currentTime, 0.05); }

  /**
   * Przy każdej linii LIVE (20 Hz): thrPct – gaz [%], fullPct – próg pełnego gazu z profilu,
   * enabled – dźwięk włączony i jest sygnał.
   */
  update(thrPct, fullPct, enabled, atTime) {
    if (!this.ctx) return;
    const t = atTime ?? this.ctx.currentTime;
    const dt = Math.min(0.2, Math.max(0.01, t - (this.lastT || t - 0.05)));
    this.lastT = t;
    if (!enabled) {
      if (this.active) {
        this.active = false;
        this.master.gain.setTargetAtTime(0, t, 0.08);
        this.turboGain.gain.setTargetAtTime(0, t, 0.05);
        this.limGain.gain.setTargetAtTime(0, t, 0.05);
      }
      this.rpm = IDLE_RPM; this.boost = 0; this.lastThr = 0; this.crackle = 0;
      return;
    }
    if (!this.active) { this.active = true; this.master.gain.setTargetAtTime(this.volume, t, 0.1); }
    else this.master.gain.setTargetAtTime(this.volume, t, 0.05);
    const x = Math.max(0, Math.min(1, thrPct / Math.max(fullPct, 1)));
    // obroty za gazem, z ograniczoną szybkością zmian
    const target = IDLE_RPM + (REDLINE_RPM - IDLE_RPM) * Math.pow(x, 0.8);
    const step = (target > this.rpm ? REV_UP : REV_DOWN) * dt;
    this.rpm += Math.max(-step, Math.min(step, target - this.rpm));
    const limit = thrPct >= fullPct && this.rpm >= REDLINE_RPM * 0.97;
    // puszczenie gazu przy wysokich obrotach: strzały z wydechu i upust turbiny
    const drop = this.lastThr - thrPct;
    if (drop > 20 && this.rpm > 4500) {
      this.crackle = 1;
      if (this.turbo && this.boost > 0.4) this.blowoff(this.boost, t);
    }
    this.crackle *= Math.exp(-dt / 0.45);
    if (x > 0.3) this.crackle = 0;
    this.lastThr = thrPct;
    if (this.node) {
      const P = this.node.parameters;
      P.get('rpm').setTargetAtTime(this.rpm, t, 0.03);
      P.get('load').setTargetAtTime(x, t, 0.03);
      P.get('limit').setValueAtTime(limit && !this.customLimiter ? 1 : 0, t);
      P.get('crackle').setValueAtTime(this.crackle, t);
      this.engineOut.gain.setTargetAtTime(limit && this.customLimiter ? 0 : 1, t, 0.02);
    }
    this.limGain.gain.setTargetAtTime(limit && (this.customLimiter || this.fallbackOnly) ? 1 : 0, t, 0.02);
    // turbina: doładowanie narasta z gazem (~0,6 s), gwizd z doładowaniem
    if (this.turbo) {
      this.boost += (x > this.boost ? 0.08 : 0.25) * (x - this.boost);
      for (const [i, o] of this.whistle.entries()) o.frequency.setTargetAtTime((4800 + 3600 * this.boost) * (i ? 1.06 : 1), t, 0.08);
      this.turboGain.gain.setTargetAtTime(0.08 * this.boost * this.boost, t, 0.08);
    } else {
      this.boost = 0;
    }
  }

  blowoff(level = 1, atTime) {
    if (!this.ctx) return;
    const s = this.ctx.createBufferSource();
    s.buffer = this.boBuf;
    const g = this.ctx.createGain();
    g.gain.value = 0.6 * level;
    s.connect(g); g.connect(this.master);
    s.start(atTime ?? 0);
    this.boost *= 0.3;
  }

  stop() { this.update(0, 100, false); }

  /** Podgląd offline (testy, plik WAV): profil gazu [%] co 50 ms → próbki mono. */
  static async render(profile, { sampleRate = 44100, fullPct = 95, workletUrl } = {}) {
    const n = Math.ceil((profile.length * 0.05 + 0.5) * sampleRate);
    const ctx = new OfflineAudioContext(1, n, sampleRate);
    const e = new LimiterSound();
    e.offline = true;
    e.workletUrl = workletUrl;
    e._build(ctx);
    await e.ready;
    profile.forEach((thr, k) => e.update(thr, fullPct, true, k * 0.05 + 0.001));
    const buf = await ctx.startRendering();
    return buf.getChannelData(0);
  }

  /** Test z ustawień: wolne obroty → gaz → odcięcie → puszczenie (strzały, upust) → wolne. */
  demo() {
    if (!this.unlock()) return;
    let k = 0;
    const prof = (k) => (k < 20 ? 0 : k < 40 ? 30 : k < 50 ? 60 : k < 90 ? 100 : k < 92 ? 40 : 0);
    const id = setInterval(() => {
      this.update(prof(k), 95, true);
      if (++k > 140) { clearInterval(id); this.stop(); }
    }, 50);
  }
}
