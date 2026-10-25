// Dźwięk silnika na odcięciu (ogranicznik obrotów) – syntezowany w Web Audio, bez plików audio.
// Charakter: wysokoobrotowa rzędowa czwórka (~9000 rpm → zapłon 300 Hz) z ogranicznikiem
// przerywającym zapłon ~14 razy na sekundę; w przerwach spadek obrotów i strzały z wydechu.
// AudioContext można uruchomić tylko po geście użytkownika – unlock() z obsługi kliknięcia.

const RPM = 9000;               // obroty na odcięciu
const CYL = 4;                  // rzędowa czwórka: 2 zapłony na obrót
const CUT_HZ = 14;              // częstotliwość odcinania
const CUT_DUTY = 0.62;          // część okresu z zapłonem

export class LimiterSound {
  constructor() {
    this.ctx = null;
    this.on = false;
    this.timer = null;
    this.volume = 0.5;
  }

  /** Wywołać z gestu użytkownika (kliknięcie) – przeglądarka blokuje dźwięk bez niego. */
  unlock() {
    const AC = window.AudioContext || window.webkitAudioContext;
    if (!AC) return false;
    if (!this.ctx) this._build(new AC());
    if (this.ctx.state === 'suspended') this.ctx.resume();
    return true;
  }

  _build(ctx) {
    this.ctx = ctx;
    const f0 = RPM / 60 * CYL / 2;                       // częstotliwość zapłonów [Hz]
    this.out = ctx.createGain();
    this.out.gain.value = 0;
    this.out.connect(ctx.destination);
    // przester – chrapliwość wydechu
    const shaper = ctx.createWaveShaper();
    const curve = new Float32Array(1024);
    for (let i = 0; i < curve.length; i++) { const x = i / 511.5 - 1; curve[i] = Math.tanh(3 * x); }
    shaper.curve = curve;
    const bp = ctx.createBiquadFilter();
    bp.type = 'bandpass'; bp.frequency.value = 1400; bp.Q.value = 0.6;
    const lp = ctx.createBiquadFilter();
    lp.type = 'lowpass'; lp.frequency.value = 5000;
    this.gate = ctx.createGain();
    this.gate.gain.value = 0;
    shaper.connect(bp); bp.connect(lp); lp.connect(this.gate); this.gate.connect(this.out);
    // ton: zapłon (piła), pół-rząd (kwadrat) i drugi rząd – brzmienie czwórki
    this.oscs = [['sawtooth', 1, 0.55], ['square', 0.5, 0.25], ['sawtooth', 2, 0.2]].map(([type, mul, g]) => {
      const o = ctx.createOscillator();
      o.type = type; o.frequency.value = f0 * mul;
      const gg = ctx.createGain(); gg.gain.value = g;
      o.connect(gg); gg.connect(shaper); o.start();
      return { o, mul };
    });
    this.f0 = f0;
    // szum na strzały z wydechu
    const len = ctx.sampleRate * 0.2;
    const buf = ctx.createBuffer(1, len, ctx.sampleRate);
    const d = buf.getChannelData(0);
    for (let i = 0; i < len; i++) d[i] = (Math.random() * 2 - 1) * Math.exp(-i / (len / 6));
    this.popBuf = buf;
  }

  setVolume(v) { this.volume = Math.max(0, Math.min(1, v)); }

  /** Włącza / wyłącza odcięcie (wywoływane przy każdej linii LIVE – zmiany tylko przy przejściu). */
  set(active) {
    if (!this.ctx || active === this.on) return;
    this.on = active;
    const t = this.ctx.currentTime;
    this.out.gain.cancelScheduledValues(t);
    this.out.gain.setTargetAtTime(active ? this.volume : 0, t, 0.03);
    clearInterval(this.timer);
    this.timer = null;
    if (!active) { this.gate.gain.setTargetAtTime(0, t, 0.02); return; }
    this.next = t + 0.02;
    this._schedule();
    this.timer = setInterval(() => this._schedule(), 40);
  }

  // Planowanie cykli odcięcia z wyprzedzeniem ~0,15 s (setInterval bywa nierówny)
  _schedule() {
    const ctx = this.ctx;
    const T = 1 / CUT_HZ;
    while (this.next < ctx.currentTime + 0.15) {
      const t = this.next;
      const on = T * CUT_DUTY * (0.9 + Math.random() * 0.2);
      // zapłon: pełna głośność, obroty lekko rosną do odcięcia
      this.gate.gain.setTargetAtTime(1, t, 0.004);
      for (const { o, mul } of this.oscs) {
        o.frequency.setValueAtTime(this.f0 * mul * 0.97, t);
        o.frequency.linearRampToValueAtTime(this.f0 * mul * 1.01, t + on);
      }
      // odcięcie: cisza, spadek obrotów, czasem strzał
      this.gate.gain.setTargetAtTime(0.08, t + on, 0.006);
      for (const { o, mul } of this.oscs) o.frequency.linearRampToValueAtTime(this.f0 * mul * 0.95, t + T);
      if (Math.random() < 0.45) this._pop(t + on + 0.005);
      this.next = t + T;
    }
  }

  _pop(t) {
    const src = this.ctx.createBufferSource();
    src.buffer = this.popBuf;
    const f = this.ctx.createBiquadFilter();
    f.type = 'bandpass'; f.frequency.value = 600 + Math.random() * 900; f.Q.value = 1.2;
    const g = this.ctx.createGain();
    g.gain.value = 0.9 * this.volume;
    src.connect(f); f.connect(g); g.connect(this.ctx.destination);
    src.start(t);
    src.stop(t + 0.12);
  }

  /** Krótki test z ustawień (2 s). */
  demo() {
    if (!this.unlock()) return;
    this.set(true);
    setTimeout(() => this.set(false), 2000);
  }
}
