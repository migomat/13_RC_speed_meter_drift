/* Punkty driftu z firmware (firmware/src/drift.c) uruchamiane na PC na pliku wyścigu –
 * wejścia liczone tak jak w firmware/src/main.cpp (float32).
 * Wypisuje dla każdej próbki: kąt, V, punkty łańcucha, mnożnik, sumę, drift (0/1).
 * Używane przez tools/tests/test_drift.py.
 *   gcc -O2 -I../../firmware/src drift_host.c ../../firmware/src/drift.c -lm
 *   drift_host plik.bin [forward left up yaw_deg]   np. +x +y +z 0 */
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
  uint16_t neutral, full, tick;
  uint32_t ns;
  memcpy(&ka, buf + 36, 4);
  memcpy(&kg, buf + 40, 4);
  memcpy(&neutral, buf + 44, 2);
  memcpy(&full, buf + 46, 2);
  memcpy(&tick, buf + 52, 2);
  uint8_t fwd = buf[27];
  memcpy(&ns, buf + n - 12, 4);
  int ai[3] = {0, 1, 2};
  float as[3] = {1, 1, 1};
  float yaw = 0.0f;
  if (argc >= 6) {
    for (int k = 0; k < 3; k++) parse_axis(argv[2 + k], &ai[k], &as[k]);
    yaw = (float)atof(argv[5]);
  }
  DriftParams p;
  drift_default_params(&p);
  p.yaw_offset_deg = yaw;
  DriftState s;
  drift_init(&s);
  const RaceSample* smp = (const RaceSample*)(buf + 128);
  for (uint32_t i = 0; i < ns; i++) {
    const RaceSample* x = &smp[i];
    float a[3] = {x->ax * ka, x->ay * ka, x->az * ka};
    float g[3] = {x->gx * kg, x->gy * kg, x->gz * kg};
    float pct = 0.0f;
    if (x->throttle_us && neutral) {
      pct = ((int)x->throttle_us - (int)neutral) * 100.0f / (full ? full : 500);
      if (!fwd) pct = -pct;
    }
    int rest = x->edges == 0 && fabsf(pct) < 10.0f;
    float dt = (float)x->n_ticks / (float)tick;
    drift_step(&s, &p, dt, a[ai[0]] * as[0], a[ai[1]] * as[1], g[ai[2]] * as[2], rest);
    printf("%.4f %.4f %.3f %d %.3f %d\n", s.angle, s.corner ? s.v : 0.0f, s.pts, s.mult, s.total,
           s.in_drift);
  }
  return 0;
}
