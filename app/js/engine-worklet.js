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
    this.amp = [1, 0.86, 0.95, 0.9];   // nierówność cylindrów – „charakter” silnika
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
    const kHi = Math.exp(-dt / 0.0007), kLo = Math.exp(-dt / (0.0025 + 1.5 / fire * 0.25));
    const kPop = Math.exp(-dt / 0.004);
    const strength = 0.25 + 0.75 * load;             // bez gazu cichy, „pusty” dźwięk
    for (let i = 0; i < out.length; i++) {
      this.phase += fire * dt;
      if (limit) this.cutT = (this.cutT + dt) % 0.072; else this.cutT = 0;
      if (this.phase >= 1) {
        this.phase -= 1;
        this.cyl = (this.cyl + 1) & 3;
        const cut = limit && this.cutT > 0.046;      // ogranicznik: 46 ms zapłonów, 26 ms przerwy
        if (!cut) {
          const a = this.amp[this.cyl] * strength * (0.9 + 0.2 * this.rand());
          this.envHi += a;
          this.envLo += 0.8 * a;
        } else if (this.rand() < 0.03) {
          this.pop += 0.8 + 0.8 * this.rand();       // niespalone paliwo na odcięciu – sporadyczny strzał
        }
        if (crackle > 0 && !cut && this.rand() < 0.22 * crackle) this.pop += 0.7 + 1.3 * this.rand();
      }
      const n = this.rand() * 2 - 1;
      out[i] = (this.envHi * (0.6 + 0.4 * n) + this.envLo * 0.9) * 0.35;
      if (outPop) outPop[i] = this.pop * n * 0.5; else out[i] += this.pop * n * 0.35;
      this.envHi *= kHi;
      this.envLo *= kLo;
      this.pop *= kPop;
    }
    return true;
  }
}

registerProcessor('x4f-engine', EngineProcessor);
