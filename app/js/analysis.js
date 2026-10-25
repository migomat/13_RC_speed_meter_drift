// Analiza startu – port 1:1 tools/rc_launch/analyze.py (implementacja referencyjna).
// Każda zmiana algorytmu: jednocześnie tutaj i w analyze.py + wektory testowe.

import { DEFAULT_EST_PARAMS, estInit, estStep } from './estimator.js';
import { DEFAULT_DRIFT } from './drift.js';
import { mapAxes } from './axes.js';

export { mapAxes };

const G = 9.80665;

export const DEFAULT_SETTINGS = {
  final_drive_ratio: 5.0,
  front_tire_diameter_mm: 64.0,
  imu_axes: { forward: '+x', left: '+y', up: '+z' },
  lowpass_hz: 25,
  v_min_mps: 0.3,
  rest_gyro_std_dps: 1.5,
  rest_acc_std_g: 0.06,   // po filtrze lowpass_hz (drgania wentylatora ESC ~0,03–0,05 g)
  edge_min_us: 300,       // [µs] min. odstęp zboczy linii (drgnięcia wirnika przy ruszaniu)
  race_lowpass_hz: 5,     // [Hz] filtr przyspieszeń w analizie wyścigu (próbki 50 Hz)
  race_lap_dist_tol: 0.2, // okrążenie z drogą ±20 % od mediany = odrzucone (buksowanie, uderzenie)
  race_track_points: 200, // punkty toru (przepróbkowanie okrążeń po drodze)
  race_track_v_lowpass_hz: 2, // [Hz] wygładzanie prędkości z licznika zboczy (mapa toru)
  estimator: { ...DEFAULT_EST_PARAMS },
  drift: { ...DEFAULT_DRIFT },
};

// Zmienione wartości domyślne: zapisane ustawienie równe staremu domyślnemu → nowe.
const SETTINGS_MIGRATIONS = [
  [['rest_acc_std_g'], 0.02, 0.06],
  [['estimator', 'zupt_acc_g'], 0.04, 0.06],
];

export function mergeSettings(user) {
  const s = JSON.parse(JSON.stringify(DEFAULT_SETTINGS));
  if (!user) return s;
  user = JSON.parse(JSON.stringify(user));
  for (const [path, oldV, newV] of SETTINGS_MIGRATIONS) {
    const o = path.length > 1 ? user[path[0]] : user;
    const k = path[path.length - 1];
    if (o && o[k] === oldV) o[k] = newV;
  }
  for (const [k, v] of Object.entries(user)) {
    if (v && typeof v === 'object' && !Array.isArray(v) && typeof s[k] === 'object') Object.assign(s[k], v);
    else if (v !== undefined && v !== null) s[k] = v;
  }
  return s;
}

// ------------------------------------------------------------------ filtr
export function butter2Coeffs(fc, fs) {
  const k = Math.tan(Math.PI * fc / fs);
  const norm = 1 / (1 + Math.SQRT2 * k + k * k);
  const b0 = k * k * norm;
  return { b: [b0, 2 * b0, b0], a: [2 * (k * k - 1) * norm, (1 - Math.SQRT2 * k + k * k) * norm] };
}

function biquadPass(x, b, a) {
  const [b0, b1, b2] = b, [a1, a2] = a;
  const n = x.length;
  const y = new Float64Array(n);
  if (!n) return y;
  const u = x[0];
  let z1 = (b1 - a1 + b2 - a2) * u;
  let z2 = (b2 - a2) * u;
  for (let i = 0; i < n; i++) {
    const xi = x[i];
    const yi = b0 * xi + z1;
    z1 = b1 * xi - a1 * yi + z2;
    z2 = b2 * xi - a2 * yi;
    y[i] = yi;
  }
  return y;
}

/** Zero-fazowy LP – odbicie nieparzyste, padlen = min(n−1, 3·ceil(fs/fc)). */
export function filtfiltButter2(x, fc, fs) {
  const n = x.length;
  if (n < 2 || fc <= 0 || fc >= fs / 2) return Float64Array.from(x);
  const { b, a } = butter2Coeffs(fc, fs);
  const pad = Math.min(n - 1, 3 * Math.ceil(fs / fc));
  const ext = new Float64Array(n + 2 * pad);
  for (let i = 0; i < pad; i++) ext[i] = 2 * x[0] - x[pad - i];
  for (let i = 0; i < n; i++) ext[pad + i] = x[i];
  for (let i = 0; i < pad; i++) ext[pad + n + i] = 2 * x[n - 1] - x[n - 2 - i];
  let y = biquadPass(ext, b, a);
  y.reverse();
  y = biquadPass(y, b, a);
  y.reverse();
  return y.slice(pad, pad + n);
}

export function gradient(v, t) {
  const n = v.length;
  const out = new Float64Array(n);
  if (n < 2) return out;
  out[0] = (v[1] - v[0]) / (t[1] - t[0]);
  out[n - 1] = (v[n - 1] - v[n - 2]) / (t[n - 1] - t[n - 2]);
  for (let i = 1; i < n - 1; i++) out[i] = (v[i + 1] - v[i - 1]) / (t[i + 1] - t[i - 1]);
  return out;
}

export function interpAt(t, y, tq) {
  const n = t.length;
  if (!n || tq < t[0] || tq > t[n - 1]) return NaN;
  for (let i = 1; i < n; i++) {
    if (t[i] >= tq) {
      if (t[i] === t[i - 1]) return y[i];
      const f = (tq - t[i - 1]) / (t[i] - t[i - 1]);
      return y[i - 1] + f * (y[i] - y[i - 1]);
    }
  }
  return y[n - 1];
}

export function firstCrossing(t, y, level, iFrom = 0) {
  for (let i = Math.max(iFrom, 1); i < y.length; i++) {
    if (y[i] >= level && y[i - 1] < level) {
      const f = (level - y[i - 1]) / (y[i] - y[i - 1]);
      return t[i - 1] + f * (t[i] - t[i - 1]);
    }
    if (i === iFrom && y[i] >= level) return t[i];
  }
  return NaN;
}

// osie płytki → osie auta: axes.js (wspólne z drift.js)

// ------------------------------------------------------------------ v_front
export function vFrontFromPeriods(period, mPerRev) {
  const out = new Float64Array(period.length);
  for (let i = 0; i < period.length; i++) out[i] = period[i] > 0 ? mPerRev * 1e6 / period[i] : 0;
  return out;
}

// Filtr drgnięć jak w firmware (hall.cpp): zbocze bliżej niż minS od poprzedniego
// zbocza tej samej linii jest odrzucane razem z tamtym (o ile to ono było ostatnim
// przyjętym zboczem). Zwraca czasy przyjętych zboczy.
export function filterEdges(eT, eLine, minS) {
  const kept = [];
  const last = new Map();
  let undo = null;     // [linia, poprzedni czas tej linii]
  for (let k = 0; k < eT.length; k++) {
    const ln = eLine[k], tk = eT[k];
    if (last.has(ln) && tk - last.get(ln) < minS) {
      if (undo !== null && undo[0] === ln) {
        kept.pop();
        if (undo[1] === null) last.delete(ln); else last.set(ln, undo[1]);
        undo = null;
      }
      continue;
    }
    undo = [ln, last.has(ln) ? last.get(ln) : null];
    kept.push(tk);
    last.set(ln, tk);
  }
  return Float64Array.from(kept);
}

export function vFrontFromEdges(tS, eT, epr, timeoutS, mPerRev) {
  const n = tS.length, ne = eT.length;
  const vk = new Float64Array(ne);
  let runStart = 0;
  for (let k = 0; k < ne; k++) {
    if (k === 0 || eT[k] - eT[k - 1] > timeoutS) runStart = k;
    const kr = k - runStart;
    if (kr >= epr) vk[k] = mPerRev / (eT[k] - eT[k - epr]);
    else if (kr >= 1) vk[k] = mPerRev * (kr / epr) / (eT[k] - eT[k - kr]);
    else vk[k] = 0;
  }
  const out = new Float64Array(n);
  let k = -1;
  runStart = 0;
  for (let i = 0; i < n; i++) {
    const t = tS[i];
    while (k + 1 < ne && eT[k + 1] <= t) {
      k++;
      if (k === 0 || eT[k] - eT[k - 1] > timeoutS) runStart = k;
    }
    if (k < 0 || t - eT[k] > timeoutS) { out[i] = 0; continue; }
    let v = vk[k];
    const j = k + 1 - epr;
    const ref = j >= runStart ? eT[j] : eT[runStart];
    if (t > ref) {
      const bound = mPerRev / (t - ref);
      if (bound < v) v = bound;
    }
    out[i] = v;
  }
  return out;
}

// ------------------------------------------------------------------ analiza
export function analyze(run, settings) {
  const cfg = mergeSettings(settings);
  const h = run.header;
  const n = run.n;
  const fs = h.sample_rate_hz;
  const t = run.t_s;
  const ti = h.trigger_index;
  const warnings = [];

  const ka = h.accel_scale_g, kg = h.gyro_scale_dps;
  const sc = (arr, k) => { const o = new Float64Array(arr.length); for (let i = 0; i < arr.length; i++) o[i] = arr[i] * k; return o; };
  const [aFwd, aLeft, aUp] = mapAxes(sc(run.ax, ka), sc(run.ay, ka), sc(run.az, ka), cfg.imu_axes);
  const [gFwd, gLeft, gUp] = mapAxes(sc(run.gx, kg), sc(run.gy, kg), sc(run.gz, kg), cfg.imu_axes);

  // start: wcześniejsze z (wyzwolenie gazem, pierwsze zbocze halla po ostatnim
  // postoju przed wyzwoleniem); postój w próbce: period_rev_us = 0 i brak zbocza
  // dłużej niż hall_timeout (period = 0 bywa też tuż po ruszeniu – okres nieznany)
  const edges = run.hall_edges;
  const period = run.period_rev_us;
  const timeoutS = h.hall_timeout_us * 1e-6;
  const lastChg = new Int32Array(n).fill(-1);
  for (let i = 1; i < n; i++) lastChg[i] = edges[i] !== edges[i - 1] ? i : lastChg[i - 1];
  const stopped = (i) => period[i] === 0 && (lastChg[i] < 0 || t[i] - t[lastChg[i]] > timeoutS);
  let iStop = -1;
  for (let i = Math.min(ti, n - 1); i >= 0; i--) if (stopped(i)) { iStop = i; break; }
  let iEdge0 = n;
  if (iStop >= 0) {
    for (let i = iStop + 1; i < n; i++) if (edges[i] !== edges[i - 1]) { iEdge0 = i; break; }
  } else {
    warnings.push('brak postoju przed wyzwoleniem');
  }
  const i0 = Math.min(ti, iEdge0);
  // okno spoczynku: ciągły postój kończący się w iStop (najwyżej do i0)
  let restStart = 0;
  if (iStop >= 0) {
    restStart = iStop;
    while (restStart > 0 && stopped(restStart - 1)) restStart--;
  }

  // 3. bias
  let restEnd = i0;
  if (restEnd - restStart < 50) {
    warnings.push('krótkie okno spoczynku przed startem');
    restStart = Math.max(0, restEnd - 50);
    restEnd = restStart + Math.min(50, n);
  }
  const rn = restEnd - restStart;
  const mean = (x) => { let s = 0; for (let i = restStart; i < restEnd; i++) s += x[i]; return s / rn; };
  const std = (x) => { const m = mean(x); let s = 0; for (let i = restStart; i < restEnd; i++) s += (x[i] - m) ** 2; return Math.sqrt(s / rn); };
  const amag = new Float64Array(n);
  for (let i = 0; i < n; i++) amag[i] = Math.sqrt(aFwd[i] ** 2 + aLeft[i] ** 2 + aUp[i] ** 2);
  // rozrzut po filtrze lowpass_hz (tylko okno spoczynku) – drgania wentylatora
  // ESC (~180 Hz) nie mogą udawać ruchu auta
  const lp = (x) => {
    const y = new Float64Array(n);
    y.set(filtfiltButter2(x.subarray(restStart, restEnd), cfg.lowpass_hz, fs), restStart);
    return y;
  };
  const atRest = Math.max(std(lp(gFwd)), std(lp(gLeft)), std(lp(gUp))) < cfg.rest_gyro_std_dps &&
    std(lp(amag)) < cfg.rest_acc_std_g && edges[restEnd - 1] === edges[restStart];
  if (!atRest) warnings.push('auto nie stało przed startem – bias niepewny');
  const aRef = mean(aFwd);
  const gbLeft = mean(gLeft);

  // 4.
  const dt = new Float64Array(n);
  for (let i = 1; i < n; i++) dt[i] = t[i] - t[i - 1];
  const pitch = new Float64Array(n);
  let acc = 0;
  for (let i = 0; i < n; i++) {
    acc += -(gLeft[i] - gbLeft) * dt[i] * (Math.PI / 180);
    pitch[i] = acc;
  }
  const aLongRaw = new Float64Array(n);
  for (let i = 0; i < n; i++) aLongRaw[i] = G * (aFwd[i] - aRef - Math.sin(pitch[i]));

  // 5.
  const fc = cfg.lowpass_hz;
  const aLong = filtfiltButter2(aLongRaw, fc, fs);

  // 6.
  const vImu = new Float64Array(n);
  for (let i = i0 + 1; i < n; i++) vImu[i] = vImu[i - 1] + aLong[i] * dt[i];

  // 7.
  const epr = h.edges_per_rev;
  const mPerRev = Math.PI * cfg.front_tire_diameter_mm * 1e-3 / cfg.final_drive_ratio;
  const useEdges = h.n_edges > 0 && !(h.flags & 0x0001);
  // zbocza po filtrze drgnięć (starsze nagrania miały filtr 50 µs)
  const edgeMinS = Math.max(h.hall_min_edge_us, cfg.edge_min_us) * 1e-6;
  const edgesT = h.n_edges > 0 ? filterEdges(run.edges_t_s, run.edges_line, edgeMinS) : new Float64Array(0);
  const vFront = useEdges ? vFrontFromEdges(t, edgesT, epr, timeoutS, mPerRev)
    : vFrontFromPeriods(run.period_rev_us, mPerRev);
  const vFrontSample = vFrontFromPeriods(run.period_rev_us, mPerRev);

  // 8.
  const aFront = gradient(filtfiltButter2(vFront, fc, fs), t);

  // 9.–10.
  const vmin = cfg.v_min_mps;
  const slip = new Float64Array(n);
  for (let i = i0; i < n; i++) slip[i] = (vFront[i] - vImu[i]) / Math.max(vFront[i], vmin);
  const spin = new Float64Array(n);
  for (let i = 0; i < n; i++) spin[i] = aFront[i] - aLong[i];

  // 11.
  const dFront = new Float64Array(n);
  for (let i = i0; i < n; i++) dFront[i] = Math.max(0, edges[i] - edges[i0]) / epr * mPerRev;
  const dImu = new Float64Array(n);
  for (let i = i0 + 1; i < n; i++) dImu[i] = dImu[i - 1] + vImu[i] * dt[i];

  // 12. v_disp
  const neutral = h.throttle_neutral_us;
  const full = h.throttle_full_us || 500;
  const sgn = h.throttle_fwd_higher ? 1 : -1;
  const thrPct = new Float64Array(n);
  for (let i = 0; i < n; i++) {
    const w = run.throttle_us[i];
    thrPct[i] = (w && neutral) ? (w - neutral) * 100 / full * sgn : 0;
  }
  const p = { ...DEFAULT_EST_PARAMS, ...(cfg.estimator || {}) };
  const st = estInit();
  const vDisp = new Float64Array(n);
  for (let i = 0; i < n; i++) {
    if (i > 0) {
      const stopped = run.period_rev_us[i] === 0 && edges[i] === edges[i - 1];
      estStep(st, p, dt[i], aFwd[i], aLeft[i], aUp[i], gFwd[i], gLeft[i], gUp[i], vFrontSample[i], stopped, thrPct[i]);
    }
    vDisp[i] = st.v;
  }

  let tFirstEdge = NaN;
  // (pierwsze zbocze po próbce postoju iStop)
  if (h.n_edges > 0 && iStop >= 0) {
    const tStop = t[iStop];
    for (const te of edgesT) if (te > tStop) { tFirstEdge = te; break; }
  } else if (iEdge0 < n) tFirstEdge = t[iEdge0];

  const pitchDeg = new Float64Array(n);
  for (let i = 0; i < n; i++) pitchDeg[i] = pitch[i] * 180 / Math.PI;

  const series = {
    t, throttle_pct: thrPct, v_front: vFront, v_imu: vImu, v_disp: vDisp, a_front: aFront,
    a_long: aLong, slip, spin, pitch_deg: pitchDeg, d_front: dFront, d_imu: dImu,
    // obroty silnika z v_front (niezależne od FDR i średnicy opony)
    rpm: Float64Array.from(vFront, (v) => v / mPerRev * 60),
  };
  const metrics = computeMetrics(series, i0, vmin, tFirstEdge);
  return { series, metrics, warnings, at_rest: atRest, header: h, used_raw_edges: useEdges, i0 };
}

function windowMean(t, y, t0, t1) {
  let s = 0, c = 0;
  for (let i = 0; i < t.length; i++) if (t0 <= t[i] && t[i] < t1) { s += y[i]; c++; }
  return c ? s / c : NaN;
}

export function computeMetrics(sr, i0, vmin, tFirstEdge) {
  const t = sr.t;
  const t0 = t[i0];
  const n = t.length;
  let peakSlip = NaN, peakT = NaN;
  for (let i = i0; i < n; i++) {
    if (sr.v_front[i] >= vmin && (Number.isNaN(peakSlip) || sr.slip[i] > peakSlip)) {
      peakSlip = sr.slip[i];
      peakT = t[i] - t0;
    }
  }
  let spinInt = 0;
  for (let i = i0 + 1; i < n; i++) {
    const x = sr.spin[i];
    if (x > 0) spinInt += x * (t[i] - t[i - 1]);
  }
  let rpmMax = 0, rpmMaxT = NaN;
  for (let i = i0; i < n; i++) if (sr.rpm[i] > rpmMax) { rpmMax = sr.rpm[i]; rpmMaxT = t[i] - t0; }
  let vmax = -Infinity;
  for (let i = 0; i < n; i++) if (sr.v_disp[i] > vmax) vmax = sr.v_disp[i];
  return {
    a_long_0_300ms: windowMean(t, sr.a_long, t0, t0 + 0.3),
    a_long_0_1000ms: windowMean(t, sr.a_long, t0, t0 + 1.0),
    v_imu_0_5s: interpAt(t, sr.v_imu, t0 + 0.5),
    v_imu_1_0s: interpAt(t, sr.v_imu, t0 + 1.0),
    v_max: vmax,
    t_1m: firstCrossing(t, sr.d_imu, 1.0, i0) - t0,
    t_2m: firstCrossing(t, sr.d_imu, 2.0, i0) - t0,
    peak_slip: peakSlip,
    peak_slip_t: peakT,
    spin_integral: spinInt,
    esc_delay_s: tFirstEdge,
    rpm_max: rpmMax,
    rpm_max_t: rpmMaxT,
    t_launch_s: t0,
  };
}

// ------------------------------------------------------------------ wyścig
// Port 1:1 analyze_race() z analyze.py. Podział na okrążenia – osobny krok (później).
const RSF_GAP = 0x40;

export function analyzeRace(race, settings) {
  const cfg = mergeSettings(settings);
  const h = race.header;
  const n = race.n;
  const warnings = [];
  if (n < 2) throw new Error('Za mało próbek wyścigu');
  const fs = h.sample_rate_hz;
  const t = race.t_s;
  const ti = Math.min(h.trigger_index, n - 1);
  const dt = new Float64Array(n);
  for (let i = 0; i < n; i++) dt[i] = race.n_ticks[i] / h.tick_rate_hz;

  const ka = h.accel_scale_g, kg = h.gyro_scale_dps;
  const sc = (arr, k) => { const o = new Float64Array(arr.length); for (let i = 0; i < arr.length; i++) o[i] = arr[i] * k; return o; };
  const [aFwd, aLeft] = mapAxes(sc(race.ax, ka), sc(race.ay, ka), sc(race.az, ka), cfg.imu_axes);
  const [, , gUp] = mapAxes(sc(race.gx, kg), sc(race.gy, kg), sc(race.gz, kg), cfg.imu_axes);

  // bias: próbki przed gazem bez zboczy (auto stało na starcie)
  const rest = [];
  for (let i = 0; i < ti; i++) if (race.edges[i] === 0) rest.push(i);
  let aRefF = 0, aRefL = 0, gbUp = 0;
  if (rest.length >= 10) {
    for (const i of rest) aRefF += aFwd[i];
    for (const i of rest) aRefL += aLeft[i];
    for (const i of rest) gbUp += gUp[i];
    aRefF /= rest.length; aRefL /= rest.length; gbUp /= rest.length;
  } else {
    warnings.push('brak postoju przed startem – bez korekty biasu IMU');
  }

  const neutral = h.throttle_neutral_us, full = h.throttle_full_us || 500;
  const sign = h.throttle_fwd_higher ? 1 : -1;
  const thrPct = new Float64Array(n);
  for (let i = 0; i < n; i++) {
    const w = race.throttle_us[i];
    thrPct[i] = (w && neutral) ? (w - neutral) * 100 / full * sign : 0;
  }

  const epr = h.edges_per_rev;
  const mPerRev = Math.PI * cfg.front_tire_diameter_mm * 1e-3 / cfg.final_drive_ratio;
  const unit = h.period_unit_us;
  const rpm = new Float64Array(n), vFront = new Float64Array(n), dFront = new Float64Array(n);
  let acc = 0;
  for (let i = 0; i < n; i++) {
    // obroty z okresu; niezgodne z licznikiem zboczy (drgnięcie wirnika) → z liczby zboczy w 2 próbkach
    const p = race.period_rev[i];
    let rp = p > 0 ? 60e6 / (p * unit) : 0;
    const e2 = race.edges[i] + (i > 0 ? race.edges[i - 1] : 0);
    const t2 = dt[i] + (i > 0 ? dt[i - 1] : 0);
    const exp2 = rp / 60 * epr * t2;
    if (exp2 >= 6 && e2 < exp2 / 3) rp = e2 / epr / t2 * 60;
    rpm[i] = rp;
    vFront[i] = rpm[i] / 60 * mPerRev;
    acc += race.edges[i];
    dFront[i] = acc / epr * mPerRev;
  }

  const fc = cfg.race_lowpass_hz;
  const aLongRaw = new Float64Array(n), aLatRaw = new Float64Array(n);
  for (let i = 0; i < n; i++) { aLongRaw[i] = G * (aFwd[i] - aRefF); aLatRaw[i] = G * (aLeft[i] - aRefL); }
  const aLong = filtfiltButter2(aLongRaw, fc, fs);
  const aLat = filtfiltButter2(aLatRaw, fc, fs);
  const yawRate = new Float64Array(n), heading = new Float64Array(n);
  let hd = 0;
  for (let i = 0; i < n; i++) {
    yawRate[i] = gUp[i] - gbUp;
    if (i > 0) hd += yawRate[i] * dt[i];
    heading[i] = hd;
  }

  // koniec jazdy: ostatnia próbka z ruchem albo gazem poza neutrum
  let iLast = ti;
  for (let i = n - 1; i >= ti; i--) {
    if (race.edges[i] > 0 || Math.abs(thrPct[i]) >= 10) { iLast = i; break; }
  }
  let gaps = 0;
  for (let i = 0; i < n; i++) if (race.flags[i] & RSF_GAP) gaps++;
  if (gaps) warnings.push(`${gaps} przerw w zapisie (bufor RAM pełny)`);

  const argMax = (f, from, to) => {
    let best = -Infinity, bi = from;
    for (let i = from; i <= to; i++) { const y = f(i); if (y > best) { best = y; bi = i; } }
    return [best, bi];
  };
  const [vmax, iv] = argMax((i) => vFront[i], ti, iLast);
  const [rmax, ir] = argMax((i) => rpm[i], ti, iLast);
  const [alatMax] = argMax((i) => Math.abs(aLat[i]), ti, iLast);
  const [alongMax] = argMax((i) => aLong[i], ti, iLast);
  const [brakeMax] = argMax((i) => -aLong[i], ti, iLast);
  let fullN = 0;
  for (let i = ti; i <= iLast; i++) if (thrPct[i] > 90) fullN++;
  const metrics = {
    duration_s: t[iLast] - t[ti],
    distance_m: dFront[iLast] - dFront[ti],
    v_front_max: vmax,
    v_front_max_t: t[iv],
    v_front_avg: (dFront[iLast] - dFront[ti]) / Math.max(t[iLast] - t[ti], 1e-9),
    rpm_max: rmax,
    rpm_max_t: t[ir],
    a_long_max_g: alongMax / G,
    brake_max_g: brakeMax / G,
    a_lat_max_g: alatMax / G,
    full_throttle_frac: fullN / (iLast - ti + 1),
    heading_total_deg: heading[iLast] - heading[ti],
    gaps,
  };
  const ds = new Float64Array(n), vRaw = new Float64Array(n);
  for (let i = 0; i < n; i++) { ds[i] = race.edges[i] / epr * mPerRev; vRaw[i] = ds[i] / dt[i]; }
  const vTrack = filtfiltButter2(vRaw, cfg.race_track_v_lowpass_hz, fs);
  const { laps, track } = raceLaps(ds, heading, vTrack, t, ti, iLast, cfg);
  const good = laps.filter((lp) => lp.good);
  metrics.laps_n = laps.length;
  metrics.best_lap_s = good.length ? Math.min(...good.map((lp) => lp.time_s)) : NaN;
  metrics.track_length_m = track ? track.length_m : NaN;
  const series = { t, throttle_pct: thrPct, rpm, v_front: vFront, d_front: dFront,
    a_long: aLong, a_lat: aLat, yaw_rate: yawRate, heading_deg: heading, v_track: vTrack };
  return { series, metrics, warnings, header: h, end_reason: race.end_reason, i_start: ti, i_last: iLast, laps, track };
}

// Interpolacja liniowa dla rosnących xq (xp niemalejące) – jak interp_lin() w analyze.py.
export function interpLin(xq, xp, fp) {
  const out = new Float64Array(xq.length);
  let k = 0;
  const m = xp.length;
  for (let j = 0; j < xq.length; j++) {
    const x = xq[j];
    while (k < m - 1 && xp[k] < x) k++;
    if (k === 0 || xp[k] < x) out[j] = fp[k];
    else if (xp[k] === xp[k - 1]) out[j] = fp[k];
    else out[j] = fp[k - 1] + (x - xp[k - 1]) / (xp[k] - xp[k - 1]) * (fp[k] - fp[k - 1]);
  }
  return out;
}

// Obrót + przesunięcie punktów P na wzorzec R – jak align_2d() w analyze.py.
function align2d(px, py, rx, ry) {
  const n = px.length;
  let pcx = 0, pcy = 0, rcx = 0, rcy = 0;
  for (let i = 0; i < n; i++) pcx += px[i];
  for (let i = 0; i < n; i++) pcy += py[i];
  for (let i = 0; i < n; i++) rcx += rx[i];
  for (let i = 0; i < n; i++) rcy += ry[i];
  pcx /= n; pcy /= n; rcx /= n; rcy /= n;
  let sc = 0, ss = 0;
  for (let i = 0; i < n; i++) {
    const ax = px[i] - pcx, ay = py[i] - pcy, bx = rx[i] - rcx, by = ry[i] - rcy;
    sc += ax * bx + ay * by;
    ss += ax * by - ay * bx;
  }
  const th = Math.atan2(ss, sc), c = Math.cos(th), s = Math.sin(th);
  const ox = new Float64Array(n), oy = new Float64Array(n);
  for (let i = 0; i < n; i++) {
    ox[i] = c * (px[i] - pcx) - s * (py[i] - pcy) + rcx;
    oy[i] = s * (px[i] - pcx) + c * (py[i] - pcy) + rcy;
  }
  return [ox, oy];
}

// Okrążenia i tor – port race_laps() z analyze.py (opis tam).
export function raceLaps(ds, heading, vTrack, t, i0, i1, cfg) {
  const n = ds.length;
  const x = new Float64Array(n), y = new Float64Array(n);
  for (let i = 1; i < n; i++) {
    const a = heading[i] * (Math.PI / 180);
    x[i] = x[i - 1] + ds[i] * Math.cos(a);
    y[i] = y[i - 1] + ds[i] * Math.sin(a);
  }
  const dir = heading[i1] - heading[i0] >= 0 ? 1 : -1;
  const bounds = [i0];
  let k = 1;
  for (let i = i0; i <= i1; i++) {
    if ((heading[i] - heading[i0]) * dir >= 360 * k) { bounds.push(i); k++; }
  }
  const laps = [];
  for (let j = 0; j < bounds.length - 1; j++) {
    const a = bounds[j], b = bounds[j + 1];
    let d = 0, vmax = -Infinity;
    for (let i = a + 1; i <= b; i++) d += ds[i];
    for (let i = a; i <= b; i++) if (vTrack[i] > vmax) vmax = vTrack[i];
    laps.push({ i_a: a, i_b: b, t_start: t[a], time_s: t[b] - t[a], dist_m: d, v_max: vmax, good: false, err_m: NaN });
  }
  if (laps.length < 2) return { laps, track: null };
  const dists = laps.map((lp) => lp.dist_m).sort((p, q) => p - q);
  const m = dists.length;
  const med = m % 2 ? dists[(m - 1) / 2] : 0.5 * (dists[m / 2 - 1] + dists[m / 2]);
  for (const lp of laps) lp.good = Math.abs(lp.dist_m - med) < cfg.race_lap_dist_tol * med;
  const good = [];
  laps.forEach((lp, j) => { if (lp.good) good.push(j); });
  if (!good.length) return { laps, track: null };
  const npts = cfg.race_track_points;
  const rs = laps.map((lp) => {
    const a = lp.i_a, b = lp.i_b;
    const d = new Float64Array(b - a + 1);
    for (let i = a + 1; i <= b; i++) d[i - a] = d[i - a - 1] + ds[i];
    const q = new Float64Array(npts);
    for (let kk = 0; kk < npts; kk++) q[kk] = d[d.length - 1] * kk / (npts - 1);
    return [interpLin(q, d, x.subarray(a, b + 1)), interpLin(q, d, y.subarray(a, b + 1)), interpLin(q, d, vTrack.subarray(a, b + 1))];
  });
  let jref = good[0];
  for (const j of good) if (Math.abs(laps[j].dist_m - med) < Math.abs(laps[jref].dist_m - med)) jref = j;
  let rx = rs[jref][0], ry = rs[jref][1];
  let al = [];
  for (let it = 0; it < 3; it++) {
    al = laps.map((_, j) => align2d(rs[j][0], rs[j][1], rx, ry));
    const nx = new Float64Array(npts), ny = new Float64Array(npts);
    for (let kk = 0; kk < npts; kk++) {
      let sx = 0, sy = 0;
      for (const j of good) sx += al[j][0][kk];
      for (const j of good) sy += al[j][1][kk];
      nx[kk] = sx / good.length; ny[kk] = sy / good.length;
    }
    rx = nx; ry = ny;
  }
  laps.forEach((lp, j) => {
    let e = 0;
    for (let kk = 0; kk < npts; kk++) e += Math.hypot(al[j][0][kk] - rx[kk], al[j][1][kk] - ry[kk]);
    lp.err_m = e / npts;
  });
  const tv = new Float64Array(npts);
  for (let kk = 0; kk < npts; kk++) { let sv = 0; for (const j of good) sv += rs[j][2][kk]; tv[kk] = sv / good.length; }
  let length = 0;
  for (let kk = 1; kk < npts; kk++) length += Math.hypot(rx[kk] - rx[kk - 1], ry[kk] - ry[kk - 1]);
  return { laps, track: { x: rx, y: ry, v: tv, length_m: length,
    laps_x: al.map((a) => a[0]), laps_y: al.map((a) => a[1]), laps_v: rs.map((r) => r[2]) } };
}

export const RACE_METRIC_LABELS = {
  duration_s: ['czas jazdy', 'min:s'],
  distance_m: ['droga (koła)', 'm'],
  v_front_max: ['v kół maks.', 'm/s'],
  v_front_avg: ['v kół śr.', 'm/s'],
  rpm_max: ['maks. obroty silnika', 'rpm'],
  a_long_max_g: ['maks. przyspieszenie', 'g'],
  brake_max_g: ['maks. hamowanie', 'g'],
  a_lat_max_g: ['maks. przeciążenie boczne', 'g'],
  full_throttle_frac: ['pełny gaz', '%'],
  heading_total_deg: ['suma skrętu', '°'],
  gaps: ['przerwy w zapisie', ''],
  laps_n: ['okrążenia', ''],
  best_lap_s: ['najlepsze okrążenie', 's'],
  track_length_m: ['długość toru', 'm'],
};

export const METRIC_LABELS = {
  a_long_0_300ms: ['a śr. 0–0,3 s', 'm/s²'],
  a_long_0_1000ms: ['a śr. 0–1 s', 'm/s²'],
  v_imu_0_5s: ['v po 0,5 s', 'm/s'],
  v_imu_1_0s: ['v po 1 s', 'm/s'],
  v_max: ['v maks.', 'm/s'],
  t_1m: ['czas 1 m', 's'],
  t_2m: ['czas 2 m', 's'],
  peak_slip: ['szczyt poślizgu', '%'],
  peak_slip_t: ['czas szczytu', 's'],
  spin_integral: ['przebuksowanie', 'm/s'],
  esc_delay_s: ['opóźnienie ESC', 'ms'],
  rpm_max: ['maks. obroty silnika', 'rpm'],
  rpm_max_t: ['czas maks. obrotów', 's'],
};
