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
