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
  float rest_da_g, rest_dr_dps, rest_min_s;
  float steady_yaw_acc_dps2, steady_s, max_dt_s;
  float min_yaw_dps, min_acc_g, min_acc_cs_g, min_v_mps, v_max_mps;
  float min_angle_deg, spin_angle_deg, spin_s, spin_cooldown_s;
  float chain_gap_s, min_seg_s;
  float pts_per_m, pts_per_m_deg;
  int   max_mult;
  int   spin_loses_chain;
  int   steer_sign;         /* 1 normalny, -1 odwrócony, 0 = nauka */
  float steer_learn_s, learn_drift_deg, cs_min_pct;
  int   cs_required;
  float angle_no_cs_deg;
  int   full_bonus;         /* 1 = bonus za pełny gaz w drifcie */
  float full_thr_pct, full_thr_s, full_bonus_mult;
} DriftParams;

typedef struct {
  int   have_prev;
  float p_af, p_al, p_r, quiet_t;
  int   have_bias;
  float b_af, b_al, b_r;
  int   first;
  float f_af, f_al, f_r;
  int   geo, corner;
  float corner_t, r_prev, v, vdot, angle, spin_t;
  float steer_ev;
  int   steer_learned;
  int   steer_ok;         /* steer ważny (znany znak i sygnał) */
  float steer;            /* skręt [%], + = w lewo */
  int   cs;               /* kontra */
  float full_t;           /* czas nieprzerwanego pełnego gazu */
  int   full_on;          /* pełny gaz ≥ full_thr_s (bonus / dźwięk odcięcia) */
  float full_drift_t;
  float cool_t;           /* po obrocie – bez nowego driftu */
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
/* Znak skrętu w użyciu: ustawiony albo nauczony (0 = nieznany) */
int  drift_steer_sign(const DriftState* s, const DriftParams* p);
/* dt [s], a_fwd / a_left [g] i r_up [dps] w osiach auta, rest_hint = brak zboczy i gaz
 * w neutrum, steer_valid / steer_raw = skręt [% od neutrum, znak surowy], thr_pct = gaz [%] */
void drift_step(DriftState* s, const DriftParams* p, float dt, float a_fwd, float a_left,
                float r_up, int rest_hint, int steer_valid, float steer_raw, float thr_pct);

#ifdef __cplusplus
}
#endif

#endif
