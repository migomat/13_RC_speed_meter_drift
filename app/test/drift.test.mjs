// Punkty za drift: driftScore (drift.js) = drift_score (drift.py) na wspólnych wektorach;
// kalibracja montażu (mountCalibrate) na wektorze jazdy z przyczepnością.
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';

import { parseRace } from '../js/format.js';
import { DEFAULT_SETTINGS } from '../js/analysis.js';
import { driftScore, mountCalibrate, driftInit, driftStep, driftParams, driftResetScore } from '../js/drift.js';

const here = dirname(fileURLToPath(import.meta.url));
const VEC = join(here, '..', '..', 'test_vectors');
const drifts = JSON.parse(readFileSync(join(VEC, 'drift.json'), 'utf8'));
const load = (name) => parseRace(new Uint8Array(readFileSync(join(VEC, `${name}.bin`))));
const close = (a, b, rel = 1e-6, abs = 1e-6) => Math.abs(a - b) <= Math.max(abs, rel * Math.abs(b));

for (const name of drifts) {
  test(`drift ${name}: przebiegi i podsumowanie jak drift_score`, () => {
    const exp = JSON.parse(readFileSync(join(VEC, `${name}.expected.json`), 'utf8'));
    const res = driftScore(load(name), exp.config);
    for (const [k, v] of Object.entries(exp.summary)) assert.ok(close(res.summary[k], v), `${k}: ${res.summary[k]} != ${v}`);
    for (const [k, vals] of Object.entries(exp.series)) {
      vals.forEach((v, i) => assert.ok(v === null ? Number.isNaN(res.series[k][i]) : close(res.series[k][i], v),
        `${k}[${i}]: ${res.series[k][i]} != ${v}`));
    }
  });
}

test('jazda z przyczepnością (race_oval) – zero punktów', () => {
  const res = driftScore(load('race_oval'), DEFAULT_SETTINGS);
  assert.equal(res.summary.total, 0);
});

test('DRIFT RESET zeruje wynik, filtry zostają', () => {
  const s = driftInit();
  const p = driftParams(DEFAULT_SETTINGS);
  for (let i = 0; i < 40; i++) driftStep(s, p, 0.05, 0.01, 0, 0.5, true);
  s.total = 123; s.mult = 3; s.chain = true;
  driftResetScore(s);
  assert.equal(s.total, 0); assert.equal(s.mult, 1); assert.equal(s.chain, false);
  assert.equal(s.have_bias, true);
});

function rotated(race, R) {
  const out = { ...race };
  for (const keys of [['ax', 'ay', 'az'], ['gx', 'gy', 'gz']]) {
    const src = keys.map((k) => race[k]);
    keys.forEach((k, i) => {
      const o = new Int16Array(race.n);
      for (let j = 0; j < race.n; j++) o[j] = Math.round(R[i][0] * src[0][j] + R[i][1] * src[1][j] + R[i][2] * src[2][j]);
      out[k] = o;
    });
  }
  return out;
}

test('kalibracja montażu: domyślny i obrócony o 110°', () => {
  const race = load('race_oval');
  const cal = mountCalibrate(race);
  assert.deepEqual(cal.imu_axes, { forward: '+x', left: '+y', up: '+z' });
  assert.ok(Math.abs(cal.yaw_offset_deg) < 3);
  const a = 110 * Math.PI / 180;
  const R = [[Math.cos(a), -Math.sin(a), 0], [Math.sin(a), Math.cos(a), 0], [0, 0, 1]];
  const cal2 = mountCalibrate(rotated(race, R));
  assert.deepEqual(cal2.imu_axes, { forward: '+y', left: '-x', up: '+z' });
  assert.ok(Math.abs(cal2.yaw_offset_deg - 20) < 3);
  const cal3 = mountCalibrate(rotated(race, [[1, 0, 0], [0, -1, 0], [0, 0, -1]]));
  assert.deepEqual(cal3.imu_axes, { forward: '+x', left: '-y', up: '-z' });
});

test('kalibracja: znak skrętu z jazdy z przyczepnością (normalny i odwrócony)', () => {
  for (const [name, sign] of [['drift_eights', 1], ['drift_eights_rev', -1]]) {
    const r = load(name);
    let n = 0, t = 0;
    while (n < r.n && t < 10.5) { t += r.n_ticks[n] / 500; n++; }
    const cut = { ...r, n };
    for (const k of ['edges', 'period_rev', 'throttle_us', 'ax', 'ay', 'az', 'gx', 'gy', 'gz', 'flags', 'n_ticks', 'steer_us', 't_s']) cut[k] = r[k].subarray(0, n);
    const cal = mountCalibrate(cut);
    assert.equal(cal.steer_sign, sign, name);
  }
});

test('bonus za pełny gaz: więcej punktów, te same łańcuchy', () => {
  const r = load('drift_eights');
  const off = driftScore(r, DEFAULT_SETTINGS).summary;
  const cfg = JSON.parse(JSON.stringify(DEFAULT_SETTINGS));
  cfg.drift.full_bonus = 1;
  const on = driftScore(r, cfg).summary;
  assert.ok(on.total > 1.05 * off.total);
  assert.equal(on.transitions, off.transitions);
  assert.ok(off.full_drift_s > 1.9 && off.full_drift_s < 2.7);
});

test('auto-korekta odchyłki montażu: 12° → trim ≈ 12°', () => {
  const p = driftParams(DEFAULT_SETTINGS);
  const s = driftInit();
  const d = 12 * Math.PI / 180;
  for (let i = 0; i < 60; i++) driftStep(s, p, 0.05, 0, 0, 0, true, 0, 0);
  for (let k = 0; k < 400; k++) {
    const a = Math.floor(k / 20) % 2 === 0 ? 0.2 : -0.25;
    driftStep(s, p, 0.05, a * Math.cos(d), a * Math.sin(d), 0, false, 0, 50);
  }
  assert.ok(Math.abs(s.trim - 12) < 1, String(s.trim));
});

test('puszczony gaz / hamulec: nauka opóźnień bokiem (jak drift.py)', () => {
  const r = load('drift_lift');
  const cfg = JSON.parse(JSON.stringify(DEFAULT_SETTINGS));
  Object.assign(cfg.drift, { dec_auto: 1, dec_coast_g: 0, dec_brake_g: 0, dec_learn_tau_s: 0.3 });
  const sm = driftScore(r, cfg).summary;
  assert.ok(sm.dec_coast_g > 0.04 && sm.dec_coast_g < 0.16, `${sm.dec_coast_g}`);
  assert.ok(sm.dec_brake_g > 0.08 && sm.dec_brake_g < 0.22, `${sm.dec_brake_g}`);
  assert.ok(sm.dec_learn_s > 0.5);
});

test('kontra podtrzymuje drift po puszczeniu gazu (zanik kąta 1 s)', () => {
  const run = (holdOn) => {
    const p = { ...driftParams(DEFAULT_SETTINGS), steer_sign: 1, hold_on: holdOn, dec_on: 0 };
    const s = driftInit();
    for (let i = 0; i < 40; i++) driftStep(s, p, 0.05, 0, 0, 0, true, 0, 0);
    const th = 35 * Math.PI / 180;
    let n = 0;
    for (let k = 0; k < 80; k++) {
      const lift = k >= 40;
      const ang = lift ? 0 : th;
      driftStep(s, p, 0.05, 0.4 * Math.sin(ang), 0.4 * Math.cos(ang), 120, false, -20, lift ? 0 : 40);
      if (lift && s.in_drift) n++;
    }
    return n;
  };
  const n = run(1);
  assert.ok(n >= 14 && n <= 19, `${n}`);
  assert.ok(run(0) <= 2);
});
