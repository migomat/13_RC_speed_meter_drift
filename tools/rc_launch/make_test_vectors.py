"""Syntetyczne starty w formacie binarnym + oczekiwane metryki.

  python -m rc_launch.make_test_vectors [--out ../test_vectors] [--demo ../app/demo]

Dla każdego wektora:
  <name>.bin            – start w formacie firmware (run_format.h)
  <name>.expected.json  – metryki i punkty kontrolne przebiegów z analyze.py
                          (wzorzec dla app/test i testu estymatora w C),
                          oraz „prawda” z symulacji (sprawdzenie sensowności).
Licznik czasu startuje tuż przed przepełnieniem 2^32 (test arytmetyki uint32).
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

from .analyze import DEFAULT_CONFIG, analyze, analyze_race
from .drift import drift_score
from .parse import (RACE_SAMPLE_DTYPE, RACE_SAMPLE_DTYPE_V1, RSF_GAP, SAMPLE_DTYPE, SF_DIR_FORWARD, SF_DIR_VALID,
                    SF_HALL_VALID, SF_THROTTLE_VALID, SF_TRIGGER, RF_NEUTRAL_LEARNED, RF_SIMULATED,
                    build_race, build_run, parse_race, parse_run)

G = 9.80665
FS = 500
PRE_S, POST_S = 0.5, 3.0
SIM_HZ = 20000
ACC_SCALE = 0.122e-3
GYRO_SCALE = 17.5e-3

SEQ3 = [(1, 1), (0, 0), (2, 1), (1, 0), (0, 1), (2, 0)]  # jak firmware/src/sim.cpp


def simulate_vehicle(scn: dict, rng):
    """Zwraca siatkę czasu symulacji i przebiegi: v_car, a_car, v_wheel, pitch."""
    dt = 1.0 / SIM_HZ
    t = np.arange(-PRE_S, POST_S + 0.01, dt)
    n = len(t)
    v = np.zeros(n)
    a = np.zeros(n)
    vw = np.zeros(n)
    te = scn["esc_delay"]
    A, top = scn["accel"], scn["top_speed"]
    k = A / top ** 2
    for i in range(1, n):
        tt = t[i]
        if tt > te and not scn.get("lifted"):
            drive = A * (1.0 - math.exp(-(tt - te) / 0.08))
            a[i] = drive - k * v[i - 1] ** 2
        v[i] = max(0.0, v[i - 1] + a[i] * dt)
        if tt > te:
            x = tt - te
            if scn.get("lifted"):
                vw[i] = min(12.0, 30.0 * x)          # koła w powietrzu
            else:
                over = 0.0
                if scn["slip_peak"] > 0 and x < scn["slip_len"]:
                    over = scn["slip_peak"] * math.sin(math.pi * x / scn["slip_len"]) ** 2
                vw[i] = v[i] * (1.0 + scn["slip_base"]) + over
    pitch = math.radians(scn["pitch0_deg"]) + math.radians(scn["squat_deg"]) * (a / max(A, 1e-6))
    return t, v, a, vw, pitch


def edge_times(t, vw, m_per_rev, epr, mode_lines, rng, asym):
    """Czasy zboczy z kąta silnika; niesymetryczne wypełnienie / rozstaw czujników."""
    rev = np.concatenate([[0.0], np.cumsum(vw[1:] * np.diff(t))]) / m_per_rev
    pos = rev * epr  # numer zbocza (ciągły)
    k_max = int(pos[-1])
    # przesunięcia zboczy w obrębie obrotu (ułamek odstępu)
    offs = np.zeros(epr)
    if mode_lines == 1:
        offs[1::2] = asym                 # wypełnienie ≠ 50 %
    else:
        offs = rng.uniform(-asym, asym, epr)
    out = []
    for k in range(1, k_max + 1):
        target = k + offs[k % epr]
        if target <= 0 or target > pos[-1]:
            continue
        j = np.searchsorted(pos, target)
        if j <= 0 or j >= len(pos):
            continue
        f = (target - pos[j - 1]) / (pos[j] - pos[j - 1])
        out.append(t[j - 1] + f * (t[j] - t[j - 1]))
    return np.array(sorted(out))


def firmware_hall(edge_us, sample_us, epr, timeout_us):
    """Algorytm halla z firmware (hall.cpp): licznik, okres pełnego obrotu, timeout."""
    ring = [0] * (epr + 1)
    count = since = period = last = 0
    out_edges, out_period, out_flags = [], [], []
    e = 0
    for ts in sample_us:
        while e < len(edge_us) and edge_us[e] <= ts:
            t = int(edge_us[e])
            n = count
            ring[n % (epr + 1)] = t
            count += 1
            since += 1
            period = (t - ring[(n - epr) % (epr + 1)]) if since > epr else 0
            last = t
            e += 1
        if since and ts - last > timeout_us:
            since = 0
            period = 0
        out_edges.append(count)
        out_period.append(period)
        out_flags.append(SF_HALL_VALID if period else 0)
    return out_edges, out_period, out_flags


def make_vector(name: str, scn: dict, seed: int):
    rng = np.random.default_rng(seed)
    lines = scn["hall_lines"]
    epr = 2 * lines
    fdr, dia = 5.0, 64.0
    m_per_rev = math.pi * dia * 1e-3 / fdr
    timeout_us = 80000 if lines == 1 else 50000

    t_sim, v, a, vw, pitch = simulate_vehicle(scn, rng)
    et = edge_times(t_sim, vw, m_per_rev, epr, lines, rng, scn["asym"])

    n = int((PRE_S + POST_S) * FS)
    ti = int(PRE_S * FS)
    t_rel = (np.arange(n) - ti) / FS                     # s względem wyzwolenia
    trig_abs = 0xFFFFFFFF - 1_000_000 + 12345            # przepełnienie ~1 s po starcie
    sample_abs = (trig_abs + np.round(t_rel * 1e6).astype(np.int64))
    edge_abs = trig_abs + np.round(et * 1e6).astype(np.int64)
    # hall liczony na wartościach bez zawijania, zapis modulo 2^32
    ecount, eperiod, eflags = firmware_hall(edge_abs, sample_abs, epr, timeout_us)
    ecount = np.array(ecount, dtype=np.int64) + scn.get("edge_count0", 1000)

    # IMU na siatce próbek
    def at(x):
        return np.interp(t_rel, t_sim, x)
    a_s, p_s = at(a), at(pitch)
    pitch_rate = np.gradient(p_s, t_rel)
    b = scn["imu_bias"]
    ns = scn["noise"]
    ax = a_s / G + np.sin(p_s) + b[0] + rng.normal(0, ns, n)
    ay = b[1] + rng.normal(0, ns, n) + (0.05 * np.sin(2 * np.pi * 7 * t_rel) * (t_rel > 0) if scn.get("vibe") else 0)
    az = np.cos(p_s) + b[2] + rng.normal(0, ns, n)
    gyro_ns = scn["gyro_noise"]
    gx = 0.4 + rng.normal(0, gyro_ns, n)
    gy = -np.degrees(pitch_rate) + scn["gyro_bias_y"] + rng.normal(0, gyro_ns, n)  # nos w górę = −ω_left
    gz = -0.3 + rng.normal(0, gyro_ns, n)

    # gaz: rampa przez próg dokładnie w próbce wyzwolenia
    thr = np.where(t_rel < -0.01, 1500.0, np.minimum(1900.0, 1600.0 + (t_rel) * 10000.0))
    thr = np.round(thr).astype(int)
    thr[ti] = max(thr[ti], 1601)

    s = np.zeros(n, dtype=SAMPLE_DTYPE)
    s["t_us"] = (sample_abs & 0xFFFFFFFF).astype(np.uint32)
    s["hall_edges"] = (ecount & 0xFFFFFFFF).astype(np.uint32)
    s["period_rev_us"] = eperiod
    s["throttle_us"] = thr
    for key, val, sc in (("ax", ax, ACC_SCALE), ("ay", ay, ACC_SCALE), ("az", az, ACC_SCALE),
                         ("gx", gx, GYRO_SCALE), ("gy", gy, GYRO_SCALE), ("gz", gz, GYRO_SCALE)):
        s[key] = np.clip(np.round(val / sc), -32768, 32767).astype(np.int16)
    flags = np.array(eflags, dtype=np.uint8) | SF_THROTTLE_VALID
    if lines == 3:
        flags |= np.where(np.array(eperiod) > 0, SF_DIR_VALID | SF_DIR_FORWARD, 0).astype(np.uint8)
    flags[ti] |= SF_TRIGGER
    s["flags"] = flags

    t0 = int(s["t_us"][0])
    use_edges = scn.get("raw_edges", True)
    if use_edges:
        mask = (edge_abs >= sample_abs[0]) & (edge_abs <= sample_abs[-1])
        e_rel = ((edge_abs[mask] - sample_abs[0]) & 0x1FFFFFFF).astype(np.uint32)
        k = np.arange(len(et))[mask]
        if lines == 1:
            line = np.zeros(len(k), dtype=np.uint32)
            level = ((k % 2) == 0).astype(np.uint32)
        else:
            line = np.array([SEQ3[i % 6][0] for i in k], dtype=np.uint32)
            level = np.array([SEQ3[i % 6][1] for i in k], dtype=np.uint32)
    else:
        e_rel = line = level = np.zeros(0, dtype=np.uint32)

    header = dict(
        seq=scn.get("seq", 1), unix_s=1760000000 + seed, boot_count=3, uptime_ms=123456 + seed,
        hall_lines=lines, motor_pole_pairs=1, edges_per_rev=epr, throttle_fwd_higher=1,
        sample_rate_hz=FS, imu_odr_hz=833, accel_range_g=4, gyro_range_dps=500,
        accel_scale_g=ACC_SCALE, gyro_scale_dps=GYRO_SCALE, throttle_neutral_us=1500,
        throttle_full_us=500, trigger_index=ti, flags=RF_NEUTRAL_LEARNED | RF_SIMULATED | 0x0002,
        t0_us=t0, trigger_t_us=int(s["t_us"][ti]), fw_version=(1 << 16), hall_min_edge_us=50,
        pretrigger_ms=500, posttrigger_ms=3000, trigger_pct=20, hall_pullup_v_x10=50,
        hall_timeout_us=timeout_us,
    )
    data = build_run(header, s, e_rel, line, level)
    truth = dict(
        v_car_0_5s=float(np.interp(0.5, t_sim, v)),
        v_car_1_0s=float(np.interp(1.0, t_sim, v)),
        a_car_0_300ms=float(np.mean(a[(t_sim >= 0) & (t_sim < 0.3)])),
        motion_start_s=scn["esc_delay"],
    )
    return data, truth


SCENARIOS = {
    "mode1_grip": dict(hall_lines=1, esc_delay=0.025, accel=11.0, top_speed=14.0, slip_peak=0.0,
                       slip_len=0.3, slip_base=0.02, asym=0.06, pitch0_deg=0.8, squat_deg=0.6,
                       imu_bias=(0.01, -0.004, 0.006), noise=0.002, gyro_noise=0.07,
                       gyro_bias_y=0.7, seq=11),
    "mode1_slip": dict(hall_lines=1, esc_delay=0.03, accel=9.0, top_speed=14.0, slip_peak=3.0,
                       slip_len=0.45, slip_base=0.03, asym=0.08, pitch0_deg=-0.5, squat_deg=0.8,
                       imu_bias=(-0.008, 0.003, 0.01), noise=0.003, gyro_noise=0.08,
                       gyro_bias_y=-1.1, seq=12, vibe=True),
    "mode3_grip": dict(hall_lines=3, esc_delay=0.02, accel=11.5, top_speed=15.0, slip_peak=0.0,
                       slip_len=0.3, slip_base=0.015, asym=0.04, pitch0_deg=1.2, squat_deg=0.5,
                       imu_bias=(0.006, 0.002, -0.005), noise=0.002, gyro_noise=0.07,
                       gyro_bias_y=0.4, seq=13),
    "mode3_slip": dict(hall_lines=3, esc_delay=0.028, accel=8.5, top_speed=13.0, slip_peak=4.0,
                       slip_len=0.5, slip_base=0.03, asym=0.05, pitch0_deg=0.3, squat_deg=0.9,
                       imu_bias=(0.012, -0.002, 0.004), noise=0.004, gyro_noise=0.1,
                       gyro_bias_y=1.5, seq=14, vibe=True),
    "mode1_slip_noedges": dict(hall_lines=1, esc_delay=0.03, accel=9.0, top_speed=14.0,
                               slip_peak=3.0, slip_len=0.45, slip_base=0.03, asym=0.08,
                               pitch0_deg=-0.5, squat_deg=0.8, imu_bias=(-0.008, 0.003, 0.01),
                               noise=0.003, gyro_noise=0.08, gyro_bias_y=-1.1, seq=15,
                               raw_edges=False),
    "mode1_lifted": dict(hall_lines=1, esc_delay=0.02, accel=0.0, top_speed=1.0, slip_peak=0.0,
                         slip_len=0.3, slip_base=0.0, asym=0.05, pitch0_deg=0.0, squat_deg=0.0,
                         imu_bias=(0.005, 0.0, 0.0), noise=0.002, gyro_noise=0.07,
                         gyro_bias_y=0.5, seq=16, lifted=True),
}

N_CHECK = 60


# ------------------------------------------------------------------ wyścigi
# Tor: owal – prosta L, półokrąg R, prosta L, półokrąg R (lewe zakręty).
# Próbki 50 Hz jak w firmware (race.cpp): przyrost zboczy, okres obrotu w
# jednostkach 4 µs, średnie IMU i gazu.
RACE_SCENARIOS = {
    "race_oval": dict(laps=5, straight=12.0, radius=3.0, v_straight=11.0, v_turn=5.5,
                      hall_lines=1, end_reason=3, tail_s=5.0, padding=0, gap_at=None, seq=21),
    "race_recovered": dict(laps=3, straight=10.0, radius=2.5, v_straight=9.0, v_turn=5.0,
                           hall_lines=3, end_reason=2, tail_s=2.0, padding=37, gap_at=400, seq=22),
}


def make_race_vector(scn: dict, seed: int):
    rng = np.random.default_rng(seed)
    fs, tick = 50, 500
    lines = scn["hall_lines"]
    epr = 2 * lines
    m_per_rev = math.pi * 64.0e-3 / 5.0
    L, R = scn["straight"], scn["radius"]
    lap = 2 * L + 2 * math.pi * R
    total = scn["laps"] * lap
    # profil prędkości po drodze: prosta → v_straight, zakręt → v_turn, przejścia ±a
    a_acc, a_brk = 6.0, 9.0

    def v_target(d):
        x = d % lap
        seg = [(0, L, "s"), (L, L + math.pi * R, "t"), (L + math.pi * R, 2 * L + math.pi * R, "s"),
               (2 * L + math.pi * R, lap, "t")]
        for a0, a1, k in seg:
            if a0 <= x < a1:
                if k == "t":
                    return scn["v_turn"]
                # hamowanie przed końcem prostej
                rem = a1 - x
                vb = math.sqrt(scn["v_turn"] ** 2 + 2 * a_brk * rem)
                return min(scn["v_straight"], vb)
        return scn["v_turn"]

    def curvature(d):
        x = d % lap
        if L <= x < L + math.pi * R or 2 * L + math.pi * R <= x < lap:
            return 1.0 / R
        return 0.0

    pre = fs * 1                     # 1 s postoju przed gazem
    dt = 1.0 / fs
    v, d = 0.0, 0.0
    rows = []
    edges_prev = 0
    yaw_bias = 0.6                   # [dps]
    acc_bias = (0.012, -0.008, 0.0)  # [g] montaż
    i = 0
    phase_tail = 0
    while True:
        if i < pre:
            thr, v_new = 1352, 0.0
        elif d < total:
            vt = v_target(d)
            if vt > v:
                v_new = min(vt, v + a_acc * dt)
                thr = 1352 + 480
            else:
                v_new = max(vt, v - a_brk * dt)
                thr = 1352 - 200 if vt < v - 0.5 else 1352 + 150
        else:
            v_new = max(0.0, v - a_brk * dt)
            thr = 1352
            if v_new == 0.0:
                phase_tail += 1
                if phase_tail > scn["tail_s"] * fs:
                    break
        a_long = (v_new - v) / dt
        d += 0.5 * (v + v_new) * dt
        v = v_new
        a_lat = v * v * curvature(d)
        yaw = math.degrees(v * curvature(d))
        edges_now = int(d / m_per_rev * epr)
        de = edges_now - edges_prev
        edges_prev = edges_now
        rpm = v / m_per_rev * 60.0
        period = int(round(60e6 / rpm / 4)) if rpm > 400 else 0
        noise = rng.normal(0, 0.004, 3)
        ax = (a_long / G + acc_bias[0] + noise[0]) / ACC_SCALE
        ay = (a_lat / G + acc_bias[1] + noise[1]) / ACC_SCALE
        az = (1.0 + noise[2]) / ACC_SCALE
        gz = (yaw + yaw_bias + rng.normal(0, 0.05)) / GYRO_SCALE
        rows.append((min(de, 0xFFFF), min(period, 0xFFFF), thr, int(round(ax)), int(round(ay)),
                     int(round(az)), int(round(rng.normal(0, 0.05) / GYRO_SCALE)),
                     int(round(rng.normal(0, 0.05) / GYRO_SCALE)), int(round(gz)),
                     SF_THROTTLE_VALID | (SF_HALL_VALID if period else 0), tick // fs))
        i += 1
    smp = np.array(rows, dtype=RACE_SAMPLE_DTYPE_V1)   # wyścigi z fw 1.1 (format 1)
    if scn["gap_at"]:
        smp["flags"][scn["gap_at"]] |= RSF_GAP
        smp["n_ticks"][scn["gap_at"]] = 30
    hdr = dict(seq=scn["seq"], unix_s=1_793_000_000, boot_count=7, uptime_ms=120_000,
               hall_lines=lines, motor_pole_pairs=1, edges_per_rev=epr, throttle_fwd_higher=1,
               sample_rate_hz=fs, imu_odr_hz=833, accel_range_g=4, gyro_range_dps=500,
               accel_scale_g=ACC_SCALE, gyro_scale_dps=GYRO_SCALE, throttle_neutral_us=1352,
               throttle_full_us=500, trigger_index=pre, tick_rate_hz=tick, decim=tick // fs,
               flags=RF_NEUTRAL_LEARNED | RF_SIMULATED, t0_us=123456, fw_version=0x010100,
               period_unit_us=4, pretrigger_ms=1000, end_idle_s=30, start_pct=30,
               hall_pullup_v_x10=38, hall_timeout_us=80000 if lines == 1 else 50000,
               hall_min_edge_us=300)
    data = build_race(hdr, smp, scn["end_reason"], scn["padding"])
    truth = dict(distance_m=total, laps=scn["laps"], lap_m=lap,
                 heading_total_deg=360.0 * scn["laps"])
    return data, truth


# ------------------------------------------------------------------ drift (20 Hz, format 2)
# Segmenty: (czas [s], V [m/s], β [deg], ω toru [dps], skręt [% od neutrum, + = w lewo],
# kąt driftu w ustalonym drifcie albo None) – wartości docelowe na końcu segmentu, przejście
# gładkie (smoothstep). β = kąt prędkości względem osi auta (lewy drift: β < 0, kąt driftu
# θ = −β; prawy: θ = β). ω auta = ω toru − dβ/dt. Skręt: w zakręcie z przyczepnością w stronę
# zakrętu, w drifcie kontra (przeciwnie). steer_polarity = −1: serwo odwrócone (surowy znak).
_EIGHTS = [
    (3.0, 0.0, 0.0, 0.0, 0.0, None),       # postój (bias, neutrum)
    (1.5, 4.0, 0.0, 0.0, 0.0, None),       # rozpędzanie prosto
    (0.5, 4.0, 0.0, 60.0, 20.0, None),     # łuk z przyczepnością (nauka znaku skrętu)
    (4.0, 4.0, 0.0, 60.0, 20.0, None),
    (0.5, 4.0, 0.0, 0.0, 0.0, None),       # prosto
    (1.5, 4.0, 0.0, 0.0, 0.0, None),       # łańcuch nie zaczęty
    (0.5, 4.0, -30.0, 115.0, -18.0, None), # wejście w lewy drift (kontra w prawo)
    (2.5, 4.0, -30.0, 115.0, -18.0, 30.0),
    (0.7, 4.0, 30.0, -115.0, 18.0, None),  # przekładka → prawy
    (2.5, 4.0, 30.0, -115.0, 18.0, 30.0),
    (0.7, 4.0, -35.0, 115.0, -21.0, None), # przekładka → lewy
    (2.0, 4.0, -35.0, 115.0, -21.0, 35.0),
    (0.5, 4.0, 0.0, 0.0, 0.0, None),       # wyjście
    (2.5, 4.0, 0.0, 0.0, 0.0, None),       # prosta – łańcuch zaliczony
    (0.5, 4.0, -25.0, 115.0, -15.0, None), # drugi łańcuch
    (2.0, 4.0, -25.0, 115.0, -15.0, 25.0),
    (0.7, 2.0, -150.0, 115.0, -30.0, None),  # obrót – łańcuch stracony
    (1.0, 0.0, -150.0, 0.0, 0.0, None),
    (2.0, 0.0, -150.0, 0.0, 0.0, None),    # postój
]
DRIFT_SCENARIOS = {
    "drift_eights": dict(seq=31, segments=_EIGHTS, steer_polarity=1,
                         full_throttle=[(14.9, 17.2), (23.4, 25.4)],   # pełny gaz: prawy drift, drugi łańcuch
                         truth=dict(transitions=2, chains=1, spins=1, steer_sign=1, full_drift_s=2.3)),
    "drift_eights_rev": dict(seq=32, segments=_EIGHTS, steer_polarity=-1,
                             truth=dict(transitions=2, chains=1, spins=1, steer_sign=-1)),
}
DRIFT_GYRO_SCALE = 35.0e-3   # ±1000 dps
STEER_NEUTRAL_US = 1480


def make_drift_vector(scn: dict, seed: int):
    rng = np.random.default_rng(seed)
    tick, fs = 500, 20
    decim = tick // fs
    epr = 2
    m_per_rev = math.pi * 64.0e-3 / 5.0
    dtt = 1.0 / tick
    V, B, W, S = [], [], [], []
    v0 = b0 = w0 = s0 = 0.0
    angles, t_seg = [], 0.0
    for dur, v1, b1, w1, s1, th in scn["segments"]:
        n = int(round(dur * tick))
        for k in range(n):
            u = (k + 1) / n
            sm = u * u * (3 - 2 * u)
            V.append(v0 + (v1 - v0) * sm)
            B.append(math.radians(b0 + (b1 - b0) * sm))
            W.append(math.radians(w0 + (w1 - w0) * sm))
            S.append(s0 + (s1 - s0) * sm)
        if th is not None:
            angles.append((round(t_seg + 0.7, 2), round(t_seg + dur - 0.2, 2), th))
        t_seg += dur
        v0, b0, w0, s0 = v1, b1, w1, s1
    V, B, W, S = np.array(V), np.array(B), np.array(W), np.array(S)
    Vd = np.gradient(V, dtt)
    Bd = np.gradient(B, dtt)
    r = W - Bd                                  # ω auta [rad/s]
    a_x = Vd * np.cos(B) - V * W * np.sin(B)    # [m/s²] osie auta
    a_y = Vd * np.sin(B) + V * W * np.cos(B)
    wheel = np.cumsum(V * np.where(np.abs(B) > math.radians(5), 1.3, 1.0)) * dtt
    nt = len(V) // decim
    yaw_bias = 0.8
    acc_bias = (0.010, -0.006)
    rows = []
    edges_prev = 0
    pre = int(round(scn["segments"][0][0] * fs))
    for j in range(nt):
        sl = slice(j * decim, (j + 1) * decim)
        e_now = int(wheel[sl.stop - 1] / m_per_rev * epr)
        de = e_now - edges_prev
        edges_prev = e_now
        vw = float(np.mean(V[sl]))
        rpm = vw * 1.3 / m_per_rev * 60.0
        period = int(round(60e6 / rpm / 4)) if rpm > 400 else 0
        tj = (j + 1) / fs
        full = any(a <= tj < b for a, b in scn.get("full_throttle", []))
        thr = 1352 if vw < 0.05 else (1352 + 500 if full else 1352 + 220)
        steer = STEER_NEUTRAL_US + int(round(scn["steer_polarity"] * float(np.mean(S[sl])) * 5.0
                                             + rng.normal(0, 1.0)))
        nz = rng.normal(0, 0.008, 3)
        ax = (float(np.mean(a_x[sl])) / G + acc_bias[0] + nz[0]) / ACC_SCALE
        ay = (float(np.mean(a_y[sl])) / G + acc_bias[1] + nz[1]) / ACC_SCALE
        az = (1.0 + nz[2]) / ACC_SCALE
        gz = (math.degrees(float(np.mean(r[sl]))) + yaw_bias + rng.normal(0, 0.3)) / DRIFT_GYRO_SCALE
        rows.append((min(de, 0xFFFF), min(period, 0xFFFF), thr, int(round(ax)), int(round(ay)),
                     int(round(az)), int(round(rng.normal(0, 0.3) / DRIFT_GYRO_SCALE)),
                     int(round(rng.normal(0, 0.3) / DRIFT_GYRO_SCALE)), int(round(gz)),
                     SF_THROTTLE_VALID | (SF_HALL_VALID if period else 0), decim, steer))
    smp = np.array(rows, dtype=RACE_SAMPLE_DTYPE)
    hdr = dict(seq=scn["seq"], unix_s=1_793_100_000, boot_count=9, uptime_ms=60_000,
               hall_lines=1, motor_pole_pairs=1, edges_per_rev=epr, throttle_fwd_higher=1,
               sample_rate_hz=fs, imu_odr_hz=833, accel_range_g=4, gyro_range_dps=1000,
               accel_scale_g=ACC_SCALE, gyro_scale_dps=DRIFT_GYRO_SCALE, throttle_neutral_us=1352,
               throttle_full_us=500, trigger_index=pre, tick_rate_hz=tick, decim=decim,
               flags=RF_NEUTRAL_LEARNED | RF_SIMULATED, t0_us=654321, fw_version=0x010300,
               period_unit_us=4, pretrigger_ms=3000, end_idle_s=30, start_pct=30,
               hall_pullup_v_x10=38, hall_timeout_us=80000, hall_min_edge_us=300,
               steer_neutral_us=STEER_NEUTRAL_US, steer_full_us=500)
    truth = dict(scn["truth"])
    truth["angles"] = angles
    return build_race(hdr, smp, 3, 0), truth


def drift_expected_for(data: bytes, cfg: dict) -> dict:
    res = drift_score(parse_race(data), cfg)
    sr = res["series"]
    return dict(config=cfg, summary={k: float(v) for k, v in res["summary"].items()},
                series={k: [None if math.isnan(x) else float(x) for x in sr[k]]
                        for k in ("angle", "v", "pts", "mult", "total", "drift", "steer", "cs", "full")})


def race_expected_for(data: bytes, cfg: dict) -> dict:
    res = analyze_race(parse_race(data), cfg)
    sr = res["series"]
    n = len(sr["t"])
    idx = sorted(set(np.linspace(0, n - 1, N_CHECK).astype(int).tolist()))

    def clean(x):
        x = float(x)
        return None if math.isnan(x) else x

    return dict(
        config=cfg,
        metrics={k: clean(v) for k, v in res["metrics"].items()},
        warnings=res["warnings"],
        end_reason=res["end_reason"],
        i_start=res["i_start"],
        i_last=res["i_last"],
        check_idx=idx,
        series={k: [clean(sr[k][i]) for i in idx]
                for k in ("t", "throttle_pct", "rpm", "v_front", "d_front", "a_long", "a_lat",
                          "yaw_rate", "heading_deg", "v_track")},
        laps=[{k: (clean(v) if isinstance(v, float) else v) for k, v in lp.items()} for lp in res["laps"]],
        track=None if not res["track"] else dict(
            length_m=res["track"]["length_m"],
            x=[res["track"]["x"][k] for k in range(0, len(res["track"]["x"]), 20)],
            y=[res["track"]["y"][k] for k in range(0, len(res["track"]["y"]), 20)],
            v=[res["track"]["v"][k] for k in range(0, len(res["track"]["v"]), 20)]),
    )


def expected_for(data: bytes, cfg: dict) -> dict:
    run = parse_run(data)
    res = analyze(run, cfg)
    sr = res["series"]
    n = len(sr["t"])
    idx = sorted(set(np.linspace(0, n - 1, N_CHECK).astype(int).tolist()))

    def clean(x):
        x = float(x)
        return None if math.isnan(x) else x

    return dict(
        config=cfg,
        metrics={k: clean(v) for k, v in res["metrics"].items()},
        warnings=res["warnings"],
        at_rest=res["at_rest"],
        i0=res["i0"],
        used_raw_edges=res["used_raw_edges"],
        check_idx=idx,
        series={k: [clean(sr[k][i]) for i in idx]
                for k in ("t", "throttle_pct", "v_front", "v_imu", "v_disp", "a_front", "a_long",
                          "slip", "spin", "pitch_deg", "d_front", "d_imu")},
        v_disp_full=[float(x) for x in sr["v_disp"]],
    )


def main(argv=None):
    ap = argparse.ArgumentParser()
    root = Path(__file__).resolve().parents[2]
    ap.add_argument("--out", default=str(root / "test_vectors"))
    ap.add_argument("--demo", default=str(root / "app" / "demo"))
    a = ap.parse_args(argv)
    out, demo = Path(a.out), Path(a.demo)
    out.mkdir(parents=True, exist_ok=True)
    demo.mkdir(parents=True, exist_ok=True)
    cfg = json.loads(json.dumps(DEFAULT_CONFIG))
    index = []
    for i, (name, scn) in enumerate(SCENARIOS.items()):
        data, truth = make_vector(name, scn, seed=100 + i)
        (out / f"{name}.bin").write_bytes(data)
        exp = expected_for(data, cfg)
        exp["truth"] = truth
        exp["name"] = name
        (out / f"{name}.expected.json").write_text(json.dumps(exp, indent=1))
        (demo / f"{name}.bin").write_bytes(data)
        index.append(name)
        m = exp["metrics"]
        print(f"{name:20s} v1s={m['v_imu_1_0s']:.2f} (prawda {truth['v_car_1_0s']:.2f})  "
              f"a300={m['a_long_0_300ms']:.2f}  slip={m['peak_slip']:.2f}  "
              f"esc={m['esc_delay_s'] * 1000:.1f} ms (ruch od {truth['motion_start_s'] * 1000:.0f})")
    (out / "index.json").write_text(json.dumps(index, indent=1))
    (demo / "index.json").write_text(json.dumps(index, indent=1))
    races = []
    for i, (name, scn) in enumerate(RACE_SCENARIOS.items()):
        data, truth = make_race_vector(scn, seed=200 + i)
        (out / f"{name}.bin").write_bytes(data)
        exp = race_expected_for(data, cfg)
        exp["truth"] = truth
        exp["name"] = name
        (out / f"{name}.expected.json").write_text(json.dumps(exp, indent=1))
        (demo / f"{name}.bin").write_bytes(data)
        races.append(name)
        m = exp["metrics"]
        print(f"{name:20s} {m['duration_s']:.1f} s  droga {m['distance_m']:.1f} m (prawda "
              f"{truth['distance_m']:.1f})  kurs {m['heading_total_deg']:.0f}° "
              f"(prawda {truth['heading_total_deg']:.0f})  vmax {m['v_front_max']:.2f}  okrążenia "
              f"{m['laps_n']}, tor {m['track_length_m'] or 0:.2f} m (prawda {truth['lap_m']:.2f})")
    (out / "races.json").write_text(json.dumps(races, indent=1))
    (demo / "races.json").write_text(json.dumps(races, indent=1))
    drifts = []
    for i, (name, scn) in enumerate(DRIFT_SCENARIOS.items()):
        data, truth = make_drift_vector(scn, seed=300 + i)
        (out / f"{name}.bin").write_bytes(data)
        exp = drift_expected_for(data, cfg)
        exp["truth"] = truth
        exp["name"] = name
        (out / f"{name}.expected.json").write_text(json.dumps(exp, indent=1))
        (demo / f"{name}.bin").write_bytes(data)
        drifts.append(name)
        sm = exp["summary"]
        print(f"{name:20s} suma {sm['total']:.0f}  łańcuchy {sm['chains']:.0f} (prawda {truth['chains']})  "
              f"przekładki {sm['transitions']:.0f} (prawda {truth['transitions']})  obroty "
              f"{sm['spins']:.0f} (prawda {truth['spins']})  kąt maks. {sm['angle_max_deg']:.1f}")
    (out / "drift.json").write_text(json.dumps(drifts, indent=1))
    (demo / "drift.json").write_text(json.dumps(drifts, indent=1))
    # demo: nagranie z driftem także na liście wyścigów (karta driftu w szczegółach)
    (demo / "races.json").write_text(json.dumps(races + drifts, indent=1))


if __name__ == "__main__":
    main()
