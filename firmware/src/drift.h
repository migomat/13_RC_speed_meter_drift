/* Punkty za drift (jak w Need for Speed) – liczone na Xiao na żywo.
 *
 * Czysty C, bez zależności od Arduino – ten sam kod kompiluje się na PC
 * (tools/drift_host) i jest porównywany z tools/rc_launch/drift.py (implementacja
 * referencyjna, tam pełna specyfikacja kroku) oraz app/js/drift.js.
 *
 * Wejście = jedna próbka wyścigu (RACE_RATE_HZ, średnia z taktów 500 Hz) w osiach auta:
 * ten sam strumień trafia do pamięci wyścigów, więc nagranie da się przeliczyć w aplikacji.
 *
 * Skrót: kąt driftu z kierunku przyspieszenia poziomego (prostopadłe do prędkości przy jeździe
 * po łuku), prędkość z promienia skrętu V = |a|/|ω| (koła napędzane w drifcie buksują).
 * Łańcuch: punkty = Σ V·dt·(pts_per_m + pts_per_m_deg·kąt), przekładka (drift w przeciwną
 * stronę bez przerwy dłuższej niż chain_gap_s) = mnożnik +1, koniec łańcucha =
 * suma += punkty·mnożnik, obrót = łańcuch stracony.
 */
#ifndef DRIFT_H
#define DRIFT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  float yaw_offset_deg;
  float lpf_tau_s, bias_tau_s, v_tau_s, vdot_tau_s;
  float steady_yaw_acc_dps2, steady_s, max_dt_s;
  float min_yaw_dps, min_acc_g, min_v_mps, v_max_mps;
  float min_angle_deg, spin_angle_deg, spin_s;
  float chain_gap_s, min_seg_s;
  float pts_per_m, pts_per_m_deg;
  int   max_mult;
  int   spin_loses_chain;
} DriftParams;

typedef struct {
  int   have_bias;
  float b_af, b_al, b_r;
  int   first;
  float f_af, f_al, f_r;
  int   corner;
  float corner_t, r_prev, v, vdot, angle, spin_t;
  int   in_drift;
  float seg_t;
  int   seg_dir, seg_ok;
  int   chain, chain_ok, prev_dir;
  float grace_t;
  float pts;          /* punkty bieżącego łańcucha (przed mnożnikiem) */
  int   mult;
  float total, best, last;
  uint32_t n_banks, n_lost, n_trans;
  float drift_t, angle_max;
} DriftState;

void drift_default_params(DriftParams* p);
void drift_init(DriftState* s);
/* Zerowanie wyniku (DRIFT RESET) – filtry i bias zostają */
void drift_reset_score(DriftState* s);
/* dt [s], a_fwd / a_left [g] i r_up [dps] w osiach auta, rest = postój (brak zboczy, gaz w neutrum) */
void drift_step(DriftState* s, const DriftParams* p, float dt, float a_fwd, float a_left,
                float r_up, int rest);

#ifdef __cplusplus
}
#endif

#endif
