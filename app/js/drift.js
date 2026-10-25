// Punkty za drift – port 1:1 tools/rc_launch/drift.py (implementacja referencyjna,
// tam specyfikacja kroku) i firmware/src/drift.c (na żywo na Xiao).
// Każda zmiana: jednocześnie we wszystkich trzech + wektory test_vectors/drift_*.

import { mapAxes } from './axes.js';

const G = 9.80665;
const D2R = Math.PI / 180;

export const DEFAULT_DRIFT = {
  lpf_tau_s: 0.05,        // [s] filtr przyspieszeń i ω
  bias_tau_s: 1.0,        // [s] uśrednianie biasu w postoju
  v_tau_s: 0.15,          // [s] filtr prędkości z promienia skrętu
  vdot_tau_s: 0.3,        // [s] filtr przyspieszenia stycznego
  steady_yaw_acc_dps2: 150.0, // [dps/s] powyżej – przyspieszenie styczne nieznane
  steady_s: 0.25,         // [s] po wejściu w łuk – przyspieszenie styczne nieznane
  max_dt_s: 0.2,          // [s] ograniczenie kroku
  min_yaw_dps: 30.0,      // [dps] poniżej – jazda prosto
  min_acc_g: 0.3,         // [g] poniżej – kąt nieokreślony
  min_v_mps: 1.5,         // [m/s]
  v_max_mps: 30.0,        // [m/s]
  min_angle_deg: 15.0,    // [deg] kąt, od którego liczy się drift
  spin_angle_deg: 80.0,   // [deg] powyżej przez spin_s = obrót
  spin_s: 0.25,           // [s]
  chain_gap_s: 1.0,       // [s] przerwa, po której łańcuch jest zaliczany
  min_seg_s: 0.3,         // [s] minimalny segment (przekładka)
  pts_per_m: 10.0,        // punkty za metr driftu
  pts_per_m_deg: 1.0,     // punkty za metr × stopień kąta
  max_mult: 10,           // maks. mnożnik
  spin_loses_chain: 1,    // 1 = obrót kasuje łańcuch, 0 = zalicza
};

export function driftParams(settings) {
  const p = { ...DEFAULT_DRIFT, ...((settings && settings.drift) || {}) };
  p.yaw_offset_deg = +(((settings && settings.estimator) || {}).yaw_offset_deg || 0);
  return p;
}

export function driftInit() {
  return {
    have_bias: false, b_af: 0, b_al: 0, b_r: 0,
    first: true, f_af: 0, f_al: 0, f_r: 0,
    corner: false, corner_t: 0, r_prev: 0, v: 0, vdot: 0, angle: 0, spin_t: 0,
    in_drift: false, seg_t: 0, seg_dir: 0, seg_ok: false,
    chain: false, chain_ok: false, prev_dir: 0, grace_t: 0,
    pts: 0, mult: 1, total: 0, best: 0, last: 0,
    n_banks: 0, n_lost: 0, n_trans: 0, drift_t: 0, angle_max: 0,
  };
}

function bank(s) {
  if (s.chain_ok) {
    const amount = s.pts * s.mult;
    s.total += amount;
    s.last = amount;
    s.best = Math.max(s.best, amount);
    s.n_banks += 1;
  }
  s.chain = s.chain_ok = false;
  s.pts = 0;
  s.mult = 1;
  s.prev_dir = 0;
  s.grace_t = 0;
}

/** Zerowanie wyniku (DRIFT RESET) – filtry i bias zostają. */
export function driftResetScore(s) {
  Object.assign(s, { in_drift: false, seg_t: 0, seg_dir: 0, seg_ok: false, chain: false, chain_ok: false,
    prev_dir: 0, grace_t: 0, pts: 0, mult: 1, total: 0, best: 0, last: 0, n_banks: 0, n_lost: 0,
    n_trans: 0, drift_t: 0, angle_max: 0 });
}

export function driftStep(s, p, dt, aFwd, aLeft, rDps, rest) {
  if (dt <= 0) return;
  dt = Math.min(dt, p.max_dt_s);
  const yaw = p.yaw_offset_deg * D2R;
  const c = Math.cos(yaw), sn = Math.sin(yaw);
  let af = aFwd * c + aLeft * sn;
  let al = -aFwd * sn + aLeft * c;

  if (rest) {
    if (!s.have_bias) {
      s.b_af = af; s.b_al = al; s.b_r = rDps;
      s.have_bias = true;
    } else {
      const k = dt / (p.bias_tau_s + dt);
      s.b_af += k * (af - s.b_af);
      s.b_al += k * (al - s.b_al);
      s.b_r += k * (rDps - s.b_r);
    }
  }
  af -= s.b_af;
  al -= s.b_al;
  const r = rDps - s.b_r;

  if (s.first) {
    s.f_af = af; s.f_al = al; s.f_r = r;
    s.first = false;
  } else {
    const k = dt / (p.lpf_tau_s + dt);
    s.f_af += k * (af - s.f_af);
    s.f_al += k * (al - s.f_al);
    s.f_r += k * (r - s.f_r);
  }
  const fa = s.f_af, fl = s.f_al, fr = s.f_r;

  const amag = Math.sqrt(fa * fa + fl * fl);
  const rr = Math.abs(fr);
  const corner = rr >= p.min_yaw_dps && amag >= p.min_acc_g;
  let angle = 0, d = 0;
  if (corner) {
    const vRaw = Math.min(amag * G / (rr * D2R), p.v_max_mps);
    s.corner_t = s.corner ? s.corner_t + dt : 0;
    if (s.corner_t < p.steady_s) {
      s.v = vRaw;
      s.vdot = 0;
    } else {
      const vPrev = s.v;
      s.v += dt / (p.v_tau_s + dt) * (vRaw - s.v);
      const steady = Math.abs(fr - s.r_prev) / dt < p.steady_yaw_acc_dps2;
      const target = steady ? (s.v - vPrev) / dt : 0;
      s.vdot += dt / (p.vdot_tau_s + dt) * (target - s.vdot);
    }
    const sg = fr >= 0 ? 1 : -1;
    const b0 = -sg * Math.atan2(fa, fl * sg);
    const at = s.vdot / G;
    const fa2 = fa - at * Math.cos(b0);
    const fl2 = fl - at * Math.sin(b0);
    angle = Math.atan2(fa2, fl2 * sg) / D2R;
    d = fr >= 0 ? 1 : -1;
  } else {
    s.vdot = 0;
    s.corner_t = 0;
  }
  s.corner = corner;
  s.r_prev = fr;
  s.angle = angle;

  s.spin_t = (corner && Math.abs(angle) > p.spin_angle_deg) ? s.spin_t + dt : 0;
  const drifting = corner && s.v >= p.min_v_mps && angle >= p.min_angle_deg && angle <= p.spin_angle_deg;

  if (s.chain && s.spin_t >= p.spin_s) {
    if (p.spin_loses_chain) {
      s.n_lost += 1;
      s.chain_ok = false;
    }
    bank(s);
    s.in_drift = false;
  } else if (drifting) {
    if (!s.in_drift || d !== s.seg_dir) {
      s.in_drift = true;
      s.seg_t = 0;
      s.seg_dir = d;
      s.seg_ok = false;
      if (!s.chain) {
        s.chain = true;
        s.chain_ok = false;
        s.pts = 0;
        s.mult = 1;
        s.prev_dir = 0;
      }
    }
    s.seg_t += dt;
    s.grace_t = 0;
    if (!s.seg_ok && s.seg_t >= p.min_seg_s) {
      s.seg_ok = true;
      s.chain_ok = true;
      if (s.prev_dir !== 0 && s.seg_dir !== s.prev_dir) {
        s.mult = Math.min(s.mult + 1, Math.trunc(p.max_mult));
        s.n_trans += 1;
      }
      s.prev_dir = s.seg_dir;
    }
    s.pts += s.v * dt * (p.pts_per_m + p.pts_per_m_deg * angle);
    s.drift_t += dt;
    s.angle_max = Math.max(s.angle_max, angle);
  } else {
    s.in_drift = false;
    if (s.chain) {
      s.grace_t += dt;
      if (s.grace_t >= p.chain_gap_s) bank(s);
    }
  }
}

function restMask(race) {
  const h = race.header;
  const neutral = h.throttle_neutral_us, full = h.throttle_full_us || 500;
  const sign = h.throttle_fwd_higher ? 1 : -1;
  const out = new Array(race.n);
  for (let i = 0; i < race.n; i++) {
    const w = race.throttle_us[i];
    const pct = (w && neutral) ? (w - neutral) * 100 / full * sign : 0;
    out[i] = race.edges[i] === 0 && Math.abs(pct) < 10;
  }
  return out;
}

/** Wejścia kroku z próbek wyścigu – jak race_inputs() w drift.py i firmware. */
export function raceInputs(race, settings) {
  const h = race.header;
  const ka = h.accel_scale_g, kg = h.gyro_scale_dps;
  const sc = (arr, k) => { const o = new Float64Array(arr.length); for (let i = 0; i < arr.length; i++) o[i] = arr[i] * k; return o; };
  const axes = (settings && settings.imu_axes) || { forward: '+x', left: '+y', up: '+z' };
  const [af, al] = mapAxes(sc(race.ax, ka), sc(race.ay, ka), sc(race.az, ka), axes);
  const [, , gu] = mapAxes(sc(race.gx, kg), sc(race.gy, kg), sc(race.gz, kg), axes);
  const dt = new Float64Array(race.n);
  for (let i = 0; i < race.n; i++) dt[i] = race.n_ticks[i] / h.tick_rate_hz;
  return { dt, af, al, gu, rest: restMask(race) };
}

/** Przebiegi i podsumowanie punktów za drift dla nagrania wyścigu – jak drift_score(). */
export function driftScore(race, settings) {
  const p = driftParams(settings);
  const { dt, af, al, gu, rest } = raceInputs(race, settings);
  const n = dt.length;
  const s = driftInit();
  const out = {};
  for (const k of ['angle', 'v', 'pts', 'mult', 'total', 'drift']) out[k] = new Float64Array(n);
  for (let i = 0; i < n; i++) {
    driftStep(s, p, dt[i], af[i], al[i], gu[i], rest[i]);
    out.angle[i] = s.angle;
    out.v[i] = s.corner ? s.v : 0;
    out.pts[i] = s.pts;
    out.mult[i] = s.mult;
    out.total[i] = s.total;
    out.drift[i] = s.in_drift ? 1 : 0;
  }
  const end = { ...s };
  bank(end);
  const summary = { total: end.total, best_chain: end.best, chains: end.n_banks, transitions: s.n_trans,
    spins: s.n_lost, drift_time_s: s.drift_t, angle_max_deg: s.angle_max };
  out.t = race.t_s;
  return { series: out, summary };
}

export const DRIFT_SUMMARY_LABELS = {
  total: ['Suma punktów', ''],
  best_chain: ['Najlepszy łańcuch', ''],
  chains: ['Zaliczone łańcuchy', ''],
  transitions: ['Przekładki', ''],
  spins: ['Obroty (stracone)', ''],
  drift_time_s: ['Czas w drifcie', 's'],
  angle_max_deg: ['Maks. kąt', '°'],
};

// ------------------------------------------------------------------ kalibracja montażu
const remainder2pi = (x) => x - 2 * Math.PI * Math.round(x / (2 * Math.PI));

/**
 * Orientacja płytki z nagrania jazdy BEZ driftu – jak mount_calibrate() w drift.py.
 * Zwraca {imu_axes, yaw_offset_deg, samples, spread_deg}; wyjątek, gdy za mało danych.
 */
export function mountCalibrate(race, { minYawDps = 40, minAccG = 0.1, lpfTauS = 0.15, trimDeg = 25 } = {}) {
  const h = race.header;
  const ka = h.accel_scale_g, kg = h.gyro_scale_dps;
  const n = race.n;
  const a = [race.ax, race.ay, race.az].map((x) => Array.from(x, (v) => v * ka));
  const g = [race.gx, race.gy, race.gz].map((x) => Array.from(x, (v) => v * kg));
  const rest = restMask(race);
  const ti = h.trigger_index;
  let ri = [];
  for (let i = 0; i < n; i++) if (rest[i] && i < ti) ri.push(i);
  if (ri.length < 10) { ri = []; for (let i = 0; i < n; i++) if (rest[i]) ri.push(i); }
  if (ri.length < 10) throw new Error('Za mało próbek postoju (auto musi chwilę stać przed jazdą)');
  const mean = (x) => ri.reduce((acc, i) => acc + x[i], 0) / ri.length;
  const am = a.map(mean), gb = g.map(mean);
  let up = 0;
  for (let k = 1; k < 3; k++) if (Math.abs(am[k]) > Math.abs(am[up])) up = k;
  const us = am[up] > 0 ? 1 : -1;
  const [e1, e2] = us > 0 ? [(up + 1) % 3, (up + 2) % 3] : [(up + 2) % 3, (up + 1) % 3];
  let f1 = 0, f2 = 0, fr = 0;
  const pts = [];
  for (let i = 0; i < n; i++) {
    const dt = race.n_ticks[i] / h.tick_rate_hz;
    const h1 = a[e1][i] - am[e1], h2 = a[e2][i] - am[e2];
    const r = (g[up][i] - gb[up]) * us;
    if (i === 0) { f1 = h1; f2 = h2; fr = r; } else {
      const k = dt / (lpfTauS + dt);
      f1 += k * (h1 - f1); f2 += k * (h2 - f2); fr += k * (r - fr);
    }
    if (Math.abs(fr) >= minYawDps && Math.hypot(f1, f2) >= minAccG && race.edges[i] > 0) {
      const s = fr > 0 ? 1 : -1;
      pts.push([Math.atan2(f2 * s, f1 * s), Math.abs(fr) * Math.hypot(f1, f2)]);
    }
  }
  if (pts.length < 20) throw new Error('Za mało zakrętów w nagraniu (potrzebne kilka okrążeń)');
  const cmean = (sel) => Math.atan2(sel.reduce((x, [t, w]) => x + w * Math.sin(t), 0),
    sel.reduce((x, [t, w]) => x + w * Math.cos(t), 0));
  let phi = cmean(pts);
  let sel = pts;
  for (let it = 0; it < 2; it++) {
    sel = pts.filter(([t]) => Math.abs(remainder2pi(t - phi)) < trimDeg * D2R);
    if (sel.length < 20) { sel = pts; break; }
    phi = cmean(sel);
  }
  const phiLeft = phi / D2R;
  const phiF = ((phiLeft - 90 + 180) % 360 + 360) % 360 - 180;
  const q = Math.round(phiF / 90) * 90;
  const yaw = phiF - q;
  const names = 'xyz';
  const spec = (ang) => ({ 0: `+${names[e1]}`, 90: `+${names[e2]}`, 180: `-${names[e1]}`, 270: `-${names[e2]}` })[((ang % 360) + 360) % 360];
  const c = pts.reduce((x, [t]) => x + Math.cos(t), 0) / pts.length;
  const sn = pts.reduce((x, [t]) => x + Math.sin(t), 0) / pts.length;
  const R = Math.min(1, Math.hypot(c, sn));
  const spread = Math.sqrt(Math.max(0, -2 * Math.log(Math.max(R, 1e-9)))) / D2R;
  return { imu_axes: { forward: spec(q), left: spec(q + 90), up: (us > 0 ? '+' : '-') + names[up] },
    yaw_offset_deg: yaw, samples: sel.length, spread_deg: spread };
}
