/* Punkty driftu z firmware (firmware/src/drift.c) uruchamiane na PC na pliku wyścigu –
 * wejścia liczone tak jak w firmware/src/main.cpp (float32).
 * Wypisuje dla każdej próbki: kąt, V, punkty łańcucha, mnożnik, sumę, drift (0/1), kontra (0/1).
 * Format 1 (próbka 20 B) i 2 (22 B, skręt) – krok próbki z nagłówka.
 * Używane przez tools/tests/test_drift.py.
 *   gcc -O2 -I../../firmware/src drift_host.c ../../firmware/src/drift.c -lm
 *   drift_host plik.bin [forward left up yaw_deg [steer_sign [full_bonus [dec_auto [dec_coast_g dec_brake_g]]]]]
 *   np. +x +y +z 0 0 1; na końcu (stderr): nauczone opóźnienia "dec <coast> <brake> <czas nauki>" */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "drift.h"

#pragma pack(push, 1)
typedef struct {
  uint16_t edges, period_rev, throttle_us;
  int16_t ax, ay, az, gx, gy, gz;
  uint8_t flags, n_ticks;
  uint16_t steer_us;       /* tylko format 2 */
} RaceSample;
#pragma pack(pop)

static void parse_axis(const char* s, int* idx, float* sign) {
  *sign = s[0] == '-' ? -1.0f : 1.0f;
  char c = s[strlen(s) - 1];
  *idx = c == 'x' ? 0 : c == 'y' ? 1 : 2;
}

int main(int argc, char** argv) {
  if (argc < 2) return 2;
  FILE* f = fopen(argv[1], "rb");
  if (!f) return 1;
  static uint8_t buf[1 << 21];
  size_t n = fread(buf, 1, sizeof(buf), f);
  fclose(f);
  if (n < 128 + 16) return 1;
  float ka, kg;
  uint16_t neutral, full, tick, ssize, steer_neutral, steer_full;
  uint32_t ns;
  memcpy(&ka, buf + 36, 4);
  memcpy(&kg, buf + 40, 4);
  memcpy(&neutral, buf + 44, 2);
  memcpy(&full, buf + 46, 2);
  memcpy(&tick, buf + 52, 2);
  memcpy(&ssize, buf + 64, 2);
  memcpy(&steer_neutral, buf + 86, 2);
  memcpy(&steer_full, buf + 88, 2);
  if (ssize < 22) steer_neutral = 0;   /* format 1: bez skrętu */
  uint8_t fwd = buf[27];
  memcpy(&ns, buf + n - 12, 4);
  int ai[3] = {0, 1, 2};
  float as[3] = {1, 1, 1};
  float yaw = 0.0f;
  if (argc >= 6) {
    for (int k = 0; k < 3; k++) parse_axis(argv[2 + k], &ai[k], &as[k]);
    yaw = (float)atof(argv[5]);
  }
  int steer_sign = argc >= 7 ? atoi(argv[6]) : 0;
  int full_bonus = argc >= 8 ? atoi(argv[7]) : 0;
  DriftParams p;
  drift_default_params(&p);
  p.yaw_offset_deg = yaw;
  p.steer_sign = steer_sign;
  p.full_bonus = full_bonus;
  if (argc >= 9) p.dec_auto = atoi(argv[8]);
  if (argc >= 11) {
    p.dec_coast_g = (float)atof(argv[9]);
    p.dec_brake_g = (float)atof(argv[10]);
  }
  DriftState s;
  drift_init(&s);
  for (uint32_t i = 0; i < ns; i++) {
    RaceSample sx;
    memset(&sx, 0, sizeof(sx));
    memcpy(&sx, buf + 128 + (size_t)i * ssize, ssize < sizeof(sx) ? ssize : sizeof(sx));
    const RaceSample* x = &sx;
    float a[3] = {x->ax * ka, x->ay * ka, x->az * ka};
    float g[3] = {x->gx * kg, x->gy * kg, x->gz * kg};
    float pct = 0.0f;
    if (x->throttle_us && neutral) {
      pct = ((int)x->throttle_us - (int)neutral) * 100.0f / (full ? full : 500);
      if (!fwd) pct = -pct;
    }
    int rest = x->edges == 0 && fabsf(pct) < 10.0f;
    int sv = steer_neutral && x->steer_us;
    float sp = sv ? ((int)x->steer_us - (int)steer_neutral) * 100.0f / (steer_full ? steer_full : 500) : 0.0f;
    float dt = (float)x->n_ticks / (float)tick;
    drift_step(&s, &p, dt, a[ai[0]] * as[0], a[ai[1]] * as[1], g[ai[2]] * as[2], rest, sv, sp, pct);
    printf("%.4f %.4f %.3f %d %.3f %d %d\n", s.angle, s.corner ? s.v : 0.0f, s.pts, s.mult, s.total,
           s.in_drift, s.cs);
  }
  fprintf(stderr, "dec %.5f %.5f %.3f\n", s.dec_set ? s.dec_c : p.dec_coast_g, s.dec_set ? s.dec_b : p.dec_brake_g,
          s.dec_learn_t);
  return 0;
}
