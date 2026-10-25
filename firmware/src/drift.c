/* Punkty za drift – port 1:1 tools/rc_launch/drift.py (specyfikacja kroku tam). */
#include "drift.h"

#include <math.h>
#include <string.h>

#include "config.h"

#define G_MPS2 9.80665f
#define DEG2RAD 0.017453292519943295f
#define RAD2DEG 57.29577951308232f

void drift_default_params(DriftParams* p) {
  p->yaw_offset_deg = IMU_YAW_OFFSET_DEG;
  p->lpf_tau_s = DRIFT_LPF_TAU_S;
  p->bias_tau_s = DRIFT_BIAS_TAU_S;
  p->v_tau_s = DRIFT_V_TAU_S;
  p->vdot_tau_s = DRIFT_VDOT_TAU_S;
  p->steady_yaw_acc_dps2 = DRIFT_STEADY_YAW_ACC_DPS2;
  p->steady_s = DRIFT_STEADY_S;
  p->max_dt_s = DRIFT_MAX_DT_S;
  p->min_yaw_dps = DRIFT_MIN_YAW_DPS;
  p->min_acc_g = DRIFT_MIN_ACC_G;
  p->min_v_mps = DRIFT_MIN_V_MPS;
  p->v_max_mps = DRIFT_V_MAX_MPS;
  p->min_angle_deg = DRIFT_MIN_ANGLE_DEG;
  p->spin_angle_deg = DRIFT_SPIN_ANGLE_DEG;
  p->spin_s = DRIFT_SPIN_S;
  p->chain_gap_s = DRIFT_CHAIN_GAP_S;
  p->min_seg_s = DRIFT_MIN_SEG_S;
  p->pts_per_m = DRIFT_PTS_PER_M;
  p->pts_per_m_deg = DRIFT_PTS_PER_M_DEG;
  p->max_mult = DRIFT_MAX_MULT;
  p->spin_loses_chain = DRIFT_SPIN_LOSES_CHAIN;
}

void drift_init(DriftState* s) {
  memset(s, 0, sizeof(*s));
  s->first = 1;
  s->mult = 1;
}

void drift_reset_score(DriftState* s) {
  s->in_drift = s->seg_ok = s->seg_dir = 0;
  s->seg_t = s->grace_t = 0.0f;
  s->chain = s->chain_ok = s->prev_dir = 0;
  s->pts = s->total = s->best = s->last = 0.0f;
  s->mult = 1;
  s->n_banks = s->n_lost = s->n_trans = 0;
  s->drift_t = s->angle_max = 0.0f;
}

static void bank(DriftState* s) {
  if (s->chain_ok) {
    float amount = s->pts * (float)s->mult;
    s->total += amount;
    s->last = amount;
    if (amount > s->best) s->best = amount;
    s->n_banks++;
  }
  s->chain = s->chain_ok = 0;
  s->pts = 0.0f;
  s->mult = 1;
  s->prev_dir = 0;
  s->grace_t = 0.0f;
}

void drift_step(DriftState* s, const DriftParams* p, float dt, float a_fwd, float a_left,
                float r_dps, int rest) {
  if (dt <= 0.0f) return;
  if (dt > p->max_dt_s) dt = p->max_dt_s;
  float yaw = p->yaw_offset_deg * DEG2RAD;
  float c = cosf(yaw), sn = sinf(yaw);
  float af = a_fwd * c + a_left * sn;
  float al = -a_fwd * sn + a_left * c;

  if (rest) {
    if (!s->have_bias) {
      s->b_af = af; s->b_al = al; s->b_r = r_dps;
      s->have_bias = 1;
    } else {
      float k = dt / (p->bias_tau_s + dt);
      s->b_af += k * (af - s->b_af);
      s->b_al += k * (al - s->b_al);
      s->b_r += k * (r_dps - s->b_r);
    }
  }
  af -= s->b_af;
  al -= s->b_al;
  float r = r_dps - s->b_r;

  if (s->first) {
    s->f_af = af; s->f_al = al; s->f_r = r;
    s->first = 0;
  } else {
    float k = dt / (p->lpf_tau_s + dt);
    s->f_af += k * (af - s->f_af);
    s->f_al += k * (al - s->f_al);
    s->f_r += k * (r - s->f_r);
  }
  float fa = s->f_af, fl = s->f_al, fr = s->f_r;

  float amag = sqrtf(fa * fa + fl * fl);
  float rr = fabsf(fr);
  int corner = rr >= p->min_yaw_dps && amag >= p->min_acc_g;
  float angle = 0.0f;
  int d = 0;
  if (corner) {
    float v_raw = amag * G_MPS2 / (rr * DEG2RAD);
    if (v_raw > p->v_max_mps) v_raw = p->v_max_mps;
    s->corner_t = s->corner ? s->corner_t + dt : 0.0f;
    if (s->corner_t < p->steady_s) {
      s->v = v_raw;
      s->vdot = 0.0f;
    } else {
      float v_prev = s->v;
      s->v += dt / (p->v_tau_s + dt) * (v_raw - s->v);
      int steady = fabsf(fr - s->r_prev) / dt < p->steady_yaw_acc_dps2;
      float target = steady ? (s->v - v_prev) / dt : 0.0f;
      s->vdot += dt / (p->vdot_tau_s + dt) * (target - s->vdot);
    }
    float sg = fr >= 0.0f ? 1.0f : -1.0f;
    float b0 = -sg * atan2f(fa, fl * sg);
    float at = s->vdot / G_MPS2;
    float fa2 = fa - at * cosf(b0);
    float fl2 = fl - at * sinf(b0);
    angle = atan2f(fa2, fl2 * sg) * RAD2DEG;
    d = fr >= 0.0f ? 1 : -1;
  } else {
    s->vdot = 0.0f;
    s->corner_t = 0.0f;
  }
  s->corner = corner;
  s->r_prev = fr;
  s->angle = angle;

  s->spin_t = (corner && fabsf(angle) > p->spin_angle_deg) ? s->spin_t + dt : 0.0f;
  int drifting = corner && s->v >= p->min_v_mps && angle >= p->min_angle_deg &&
                 angle <= p->spin_angle_deg;

  if (s->chain && s->spin_t >= p->spin_s) {
    if (p->spin_loses_chain) {
      s->n_lost++;
      s->chain_ok = 0;
    }
    bank(s);
    s->in_drift = 0;
  } else if (drifting) {
    if (!s->in_drift || d != s->seg_dir) {
      s->in_drift = 1;
      s->seg_t = 0.0f;
      s->seg_dir = d;
      s->seg_ok = 0;
      if (!s->chain) {
        s->chain = 1;
        s->chain_ok = 0;
        s->pts = 0.0f;
        s->mult = 1;
        s->prev_dir = 0;
      }
    }
    s->seg_t += dt;
    s->grace_t = 0.0f;
    if (!s->seg_ok && s->seg_t >= p->min_seg_s) {
      s->seg_ok = 1;
      s->chain_ok = 1;
      if (s->prev_dir != 0 && s->seg_dir != s->prev_dir) {
        if (s->mult < p->max_mult) s->mult++;
        s->n_trans++;
      }
      s->prev_dir = s->seg_dir;
    }
    s->pts += s->v * dt * (p->pts_per_m + p->pts_per_m_deg * angle);
    s->drift_t += dt;
    if (angle > s->angle_max) s->angle_max = angle;
  } else {
    s->in_drift = 0;
    if (s->chain) {
      s->grace_t += dt;
      if (s->grace_t >= p->chain_gap_s) bank(s);
    }
  }
}
