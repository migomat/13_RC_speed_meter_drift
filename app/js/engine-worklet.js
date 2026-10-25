// AudioWorklet: silnik składany z pojedynczych zapłonów (rzędowa czwórka – 2 zapłony na obrót).
// Parametry (k-rate): rpm, load (0..1 – gaz), limit (1 = ogranicznik odcina zapłony),
// crackle (0..1 – prawdopodobieństwo strzałów z wydechu przy hamowaniu silnikiem).
// Wyjście 0: zapłony (dalej przester i rezonanse wydechu), wyjście 1: strzały z wydechu – osobno,
// bez przesteru, żeby było je wyraźnie słychać (sound.js).

class EngineProcessor extends AudioWorkletProcessor {
  static get parameterDescriptors() {
    return [
      { name: 'rpm', defaultValue: 1500, minValue: 300, maxValue: 14000, automationRate: 'k-rate' },
      { name: 'load', defaultValue: 0, minValue: 0, maxValue: 1, automationRate: 'k-rate' },
      { name: 'limit', defaultValue: 0, minValue: 0, maxValue: 1, automationRate: 'k-rate' },
      { name: 'crackle', defaultValue: 0, minValue: 0, maxValue: 1, automationRate: 'k-rate' },
    ];
  }

  constructor() {
    super();
    this.phase = 0;
    this.cyl = 0;
    this.envHi = 0;     // trzask zapłonu (krótki)
    this.envLo = 0;     // uderzenie ciśnienia w wydechu (dłuższe)
    this.pop = 0;       // strzał z wydechu
    this.cutT = 0;      // czas w cyklu ogranicznika [s]
    this.seed = 22222;
    this.amp = [1, 0.72, 0.9, 0.8];    // nierówność cylindrów – bulgot zamiast czystego tonu
    this.jit = 0;                      // drobne przesunięcie zapłonu (bez tego brzmi jak chiptune)
    this.dcX = 0; this.dcY = 0;        // filtr składowej stałej (impulsy ciśnienia są jednokierunkowe)
    this.ph = 0;                       // faza warstw tonalnych (wycie dolotu, pomruk wału)
    this.howl = 0;                     // wygładzona głośność wycia
  }

  rand() {
    this.seed = (this.seed * 1103515245 + 12345) & 0x7fffffff;
    return this.seed / 0x7fffffff;
  }

  process(inputs, outputs, params) {
    const out = outputs[0][0];
    const outPop = outputs[1] ? outputs[1][0] : null;
    if (!out) return true;
    const dt = 1 / sampleRate;
    const rpm = params.rpm[0], load = params.load[0], limit = params.limit[0] >= 0.5, crackle = params.crackle[0];
    const fire = rpm / 60 * 2;                       // zapłony na sekundę
    const kHi = Math.exp(-dt / 0.0005), kLo = Math.exp(-dt / (0.004 + 0.35 / fire));
    const kPop = Math.exp(-dt / 0.009);   // dłuższy, niższy „bang” (krótki brzmiał jak zgniatana puszka)
    const strength = 0.25 + 0.75 * load;             // bez gazu cichy, „pusty” dźwięk
    // druga warstwa: tonalne „wycie” dolotu (zapłony + 2. i 3. harmoniczna) – od ~2500 rpm, z gazem;
    // trzecia: pomruk wału (pół-rząd, częstotliwość obrotu) – cały zakres
    const u = Math.min(1, Math.max(0, (rpm - 2500) / 4000));
    const howlT = (0.15 + 0.85 * load) * u * u * (3 - 2 * u) * (limit ? 0.8 : 1);
    const rumble = 0.10 + 0.12 * load;
    for (let i = 0; i < out.length; i++) {
      this.phase += fire * (1 + this.jit) * dt;
      if (limit) this.cutT = (this.cutT + dt) % 0.072; else this.cutT = 0;
      if (this.phase >= 1) {
        this.phase -= 1;
        this.cyl = (this.cyl + 1) & 3;
        this.jit = (this.rand() - 0.5) * 0.06;     // ±3 % odstępu zapłonów
        const cut = limit && this.cutT > 0.046;      // ogranicznik: 46 ms zapłonów, 26 ms przerwy
        if (!cut) {
          const a = this.amp[this.cyl] * strength * (0.9 + 0.2 * this.rand());
          this.envHi += 0.45 * a;
          this.envLo += a;
        } else if (this.rand() < 0.03) {
          this.pop += 0.8 + 0.8 * this.rand();       // niespalone paliwo na odcięciu – sporadyczny strzał
        }
        // strzały przy hamowaniu silnikiem: pojedyncze, z przerwami (seria krótkich trzasków = „puszka”)
        if (crackle > 0 && !cut && this.pop < 0.05 && this.rand() < 0.06 * crackle) this.pop += 0.9 + 1.1 * this.rand();
      }
      const n = this.rand() * 2 - 1;
      this.ph += fire * dt;
      if (this.ph >= 2) this.ph -= 2;                // 2 zapłony = 1 obrót (pół-rząd)
      this.howl += 0.0005 * (howlT - this.howl);
      const w = 2 * Math.PI * this.ph;
      const tone = Math.sin(w) + 0.55 * Math.sin(2 * w + 0.7) + 0.3 * Math.sin(3 * w + 1.9);
      const gateOn = !(limit && this.cutT > 0.046);  // odcięcie przerywa też wycie
      const raw = (this.envHi * (0.5 + 0.5 * n) + this.envLo * (0.85 + 0.15 * n)) * 0.35 +
                  (gateOn ? this.howl * 0.16 * tone : 0) + rumble * 0.12 * Math.sin(w / 2) * (this.envLo + 0.2);
      this.dcY = raw - this.dcX + 0.997 * this.dcY;   // górnoprzepustowy ~20 Hz
      this.dcX = raw;
      out[i] = this.dcY;
      if (outPop) outPop[i] = this.pop * n * 0.5; else out[i] += this.pop * n * 0.35;
      this.envHi *= kHi;
      this.envLo *= kLo;
      this.pop *= kPop;
    }
    return true;
  }
}

registerProcessor('x4f-engine', EngineProcessor);
