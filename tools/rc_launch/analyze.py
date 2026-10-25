"""Analiza startu – IMPLEMENTACJA REFERENCYJNA (spec wspólna z app/js/analysis.js).

Każda zmiana tutaj = ta sama zmiana w analysis.js i przejście wektorów testowych.
Algorytmy celowo napisane prostymi pętlami (łatwy, dokładny port do JS).

Kroki (numeracja jak w CLAUDE.md):
 1. surowe → jednostki (współczynniki z nagłówka)
 2. osie płytki → osie auta (imu_axes)
 3. bias: średnie z okna przed wyzwoleniem, jeśli auto stało
 4. pochylenie: całka ω_pitch = −(ω_left − bias); a_long = G·(a_fwd − a_ref − sin(pitch))
 5. Butterworth 2. rzędu, filtfilt z odbiciem nieparzystym (filtfilt_butter2)
 6. v_imu = Σ a_long·dt od startu i0 = min(wyzwolenie gazem, pierwsze zbocze halla)
 7. v_front z surowych zboczy (albo period_rev_us), ograniczenie między zboczami
 8. a_front = pochodna wygładzonego v_front
 9. poślizg s = (v_front − v_imu) / max(v_front, v_min)
10. wskaźnik buksowania a_front − a_long
11. droga z licznika zboczy i z całki v_imu
12. v_disp – estymator jak na Xiao (estimator.py)
"""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np

from .drift import DEFAULT_DRIFT
from .estimator import DEFAULT_PARAMS, EstState, est_step
from .parse import RSF_GAP, Race, Run, file_kind, parse_race, parse_run

G = 9.80665

DEFAULT_CONFIG = dict(
    final_drive_ratio=5.0,
    front_tire_diameter_mm=64.0,
    imu_axes=dict(forward="+x", left="+y", up="+z"),
    lowpass_hz=25.0,
    v_min_mps=0.3,
    rest_gyro_std_dps=1.5,
    rest_acc_std_g=0.06,
    edge_min_us=300,
    race_lowpass_hz=5.0,
    race_lap_dist_tol=0.2,
    race_track_points=200,
    race_track_v_lowpass_hz=2.0,
    estimator=dict(DEFAULT_PARAMS),
    drift=dict(DEFAULT_DRIFT),
)


def load_config(path: str | Path | None = None) -> dict:
    cfg = {k: (dict(v) if isinstance(v, dict) else v) for k, v in DEFAULT_CONFIG.items()}
    if path is None:
        p = Path(__file__).resolve().parent.parent / "config.yaml"
        path = p if p.exists() else None
    if path:
        import yaml

        user = yaml.safe_load(open(path)) or {}
        for k, v in user.items():
            if isinstance(v, dict) and isinstance(cfg.get(k), dict):
                cfg[k].update(v)
            else:
                cfg[k] = v
    return cfg


# ------------------------------------------------------------------ filtr
def butter2_coeffs(fc: float, fs: float):
    """Butterworth 2. rzędu, transformacja biliniowa z prewarpingiem (= scipy.signal.butter(2))."""
    k = math.tan(math.pi * fc / fs)
    norm = 1.0 / (1.0 + math.sqrt(2.0) * k + k * k)
    b0 = k * k * norm
    return (b0, 2.0 * b0, b0), (2.0 * (k * k - 1.0) * norm, (1.0 - math.sqrt(2.0) * k + k * k) * norm)


def _biquad_pass(x, b, a):
    """DF-II transponowana, stan początkowy = stan ustalony dla x[0]."""
    b0, b1, b2 = b
    a1, a2 = a
    n = len(x)
    y = [0.0] * n
    if n == 0:
        return y
    u = x[0]
    z1 = (b1 - a1 + b2 - a2) * u
    z2 = (b2 - a2) * u
    for i in range(n):
        xi = x[i]
        yi = b0 * xi + z1
        z1 = b1 * xi - a1 * yi + z2
        z2 = b2 * xi - a2 * yi
        y[i] = yi
    return y


def filtfilt_butter2(x, fc: float, fs: float) -> np.ndarray:
    """Zero-fazowy LP: odbicie nieparzyste na brzegach (padlen = min(n−1, 3·ceil(fs/fc))),
    przebieg w przód i w tył ze stanem ustalonym na początku każdego przebiegu."""
    x = [float(v) for v in x]
    n = len(x)
    if n < 2 or fc <= 0 or fc >= fs / 2:
        return np.asarray(x, dtype=np.float64)
    b, a = butter2_coeffs(fc, fs)
    pad = min(n - 1, 3 * int(math.ceil(fs / fc)))
    left = [2.0 * x[0] - x[i] for i in range(pad, 0, -1)]
    right = [2.0 * x[n - 1] - x[n - 2 - i] for i in range(pad)]
    ext = left + x + right
    y = _biquad_pass(ext, b, a)
    y.reverse()
    y = _biquad_pass(y, b, a)
    y.reverse()
    return np.asarray(y[pad : pad + n], dtype=np.float64)


def gradient(v, t) -> np.ndarray:
    """Pochodna: różnice centralne wewnątrz, jednostronne na brzegach."""
    n = len(v)
    out = np.zeros(n)
    if n < 2:
        return out
    out[0] = (v[1] - v[0]) / (t[1] - t[0])
    out[n - 1] = (v[n - 1] - v[n - 2]) / (t[n - 1] - t[n - 2])
    for i in range(1, n - 1):
        out[i] = (v[i + 1] - v[i - 1]) / (t[i + 1] - t[i - 1])
    return out


def interp_at(t, y, tq: float) -> float:
    """Interpolacja liniowa y(tq); NaN poza zakresem."""
    n = len(t)
    if n == 0 or tq < t[0] or tq > t[n - 1]:
        return float("nan")
    for i in range(1, n):
        if t[i] >= tq:
            if t[i] == t[i - 1]:
                return float(y[i])
            f = (tq - t[i - 1]) / (t[i] - t[i - 1])
            return float(y[i - 1] + f * (y[i] - y[i - 1]))
    return float(y[n - 1])


def first_crossing(t, y, level: float, i_from: int = 0) -> float:
    """Pierwszy czas, gdy y ≥ level (interpolacja liniowa); NaN gdy brak."""
    for i in range(max(i_from, 1), len(y)):
        if y[i] >= level and y[i - 1] < level:
            f = (level - y[i - 1]) / (y[i] - y[i - 1])
            return float(t[i - 1] + f * (t[i] - t[i - 1]))
        if i == i_from and y[i] >= level:
            return float(t[i])
    return float("nan")


# ------------------------------------------------------------------ osie
def parse_axis(spec: str):
    spec = spec.strip().lower()
    sign = -1.0 if spec.startswith("-") else 1.0
    return "xyz".index(spec[-1]), sign


def map_axes(vx, vy, vz, axes: dict):
    board = (vx, vy, vz)
    out = []
    for name in ("forward", "left", "up"):
        idx, sign = parse_axis(axes[name])
        out.append(board[idx] * sign)
    return out


# ------------------------------------------------------------------ v_front
def v_front_from_periods(period_us, m_per_rev):
    return np.array([m_per_rev * 1e6 / p if p > 0 else 0.0 for p in period_us], dtype=np.float64)


def filter_edges(e_t, e_line, min_s: float) -> list:
    """Filtr drgnięć jak w firmware (hall.cpp): zbocze bliżej niż min_s od poprzedniego
    zbocza tej samej linii jest odrzucane razem z tamtym (o ile to ono było ostatnim
    przyjętym zboczem). Zwraca czasy przyjętych zboczy."""
    kept = []
    last = {}
    undo = None          # (linia, poprzedni czas tej linii)
    for k in range(len(e_t)):
        ln, tk = int(e_line[k]), float(e_t[k])
        if ln in last and tk - last[ln] < min_s:
            if undo is not None and undo[0] == ln:
                kept.pop()
                if undo[1] is None:
                    del last[ln]
                else:
                    last[ln] = undo[1]
                undo = None
            continue
        undo = (ln, last.get(ln))
        kept.append(tk)
        last[ln] = tk
    return kept


def v_front_from_edges(t_s, e_t, epr: int, timeout_s: float, m_per_rev: float) -> np.ndarray:
    """Prędkość z surowych zboczy na siatce próbek.

    Dla zbocza k (k_run = numer zbocza od początku ruchu, przerwa > timeout = postój):
      k_run ≥ epr: v_k = m_per_rev / (t[k] − t[k−epr])      (zbocze vs zbocze obrót wcześniej)
      1 ≤ k_run < epr: v_k = m_per_rev·(k_run/epr) / (t[k] − t[k−k_run])
      k_run = 0: v_k = 0
    Na siatce: v(t) = min(v_k, m_per_rev / (t − t[k+1−epr])) dla ostatniego zbocza k ≤ t
    (następne zbocze nie może dać krótszego okresu); po przerwie > timeout → 0.
    """
    n = len(t_s)
    ne = len(e_t)
    vk = [0.0] * ne
    run_start = 0
    for k in range(ne):
        if k == 0 or e_t[k] - e_t[k - 1] > timeout_s:
            run_start = k
        kr = k - run_start
        if kr >= epr:
            vk[k] = m_per_rev / (e_t[k] - e_t[k - epr])
        elif kr >= 1:
            vk[k] = m_per_rev * (kr / epr) / (e_t[k] - e_t[k - kr])
        else:
            vk[k] = 0.0
    out = np.zeros(n)
    k = -1
    run_start = 0
    for i in range(n):
        t = t_s[i]
        while k + 1 < ne and e_t[k + 1] <= t:
            k += 1
            if k == 0 or e_t[k] - e_t[k - 1] > timeout_s:
                run_start = k
        if k < 0 or t - e_t[k] > timeout_s:
            out[i] = 0.0
            continue
        v = vk[k]
        j = k + 1 - epr
        ref = e_t[j] if j >= run_start else e_t[run_start]
        if t > ref:
            bound = m_per_rev / (t - ref)
            if bound < v:
                v = bound
        out[i] = v
    return out


# ------------------------------------------------------------------ analiza
def analyze(run: Run, cfg: dict | None = None) -> dict:
    cfg = cfg or load_config()
    h = run.header
    s = run.samples
    n = len(s)
    fs = float(h["sample_rate_hz"])
    t = run.t_s                                  # względem wyzwolenia
    ti = int(h["trigger_index"])
    warnings = []

    # 1.–2.
    ka, kg = float(h["accel_scale_g"]), float(h["gyro_scale_dps"])
    a_fwd, a_left, a_up = map_axes(s["ax"] * ka, s["ay"] * ka, s["az"] * ka, cfg["imu_axes"])
    g_fwd, g_left, g_up = map_axes(s["gx"] * kg, s["gy"] * kg, s["gz"] * kg, cfg["imu_axes"])
    a_fwd, a_left, a_up = (np.asarray(x, dtype=np.float64) for x in (a_fwd, a_left, a_up))
    g_fwd, g_left, g_up = (np.asarray(x, dtype=np.float64) for x in (g_fwd, g_left, g_up))

    # Start: wcześniejsze z (wyzwolenie gazem, pierwsze zbocze halla po ostatnim
    # postoju przed wyzwoleniem). ESC zwykle rusza silnik poniżej progu wyzwolenia,
    # więc zbocze bywa pierwsze. Postój w próbce: period_rev_us = 0 i brak zbocza
    # dłużej niż hall_timeout (period = 0 bywa też tuż po ruszeniu – okres nieznany).
    # Koła kręcące się jeszcze z poprzedniego przejazdu nie są startem.
    edges = s["hall_edges"]
    period = s["period_rev_us"]
    timeout_s = h["hall_timeout_us"] * 1e-6
    last_chg = [-1] * n
    for i in range(1, n):
        last_chg[i] = i if int(edges[i]) != int(edges[i - 1]) else last_chg[i - 1]
    def stopped(i):
        return int(period[i]) == 0 and (last_chg[i] < 0 or t[i] - t[last_chg[i]] > timeout_s)
    i_stop = -1
    for i in range(min(ti, n - 1), -1, -1):
        if stopped(i):
            i_stop = i
            break
    i_edge0 = n
    if i_stop >= 0:
        for i in range(i_stop + 1, n):
            if int(edges[i]) != int(edges[i - 1]):
                i_edge0 = i
                break
    else:
        warnings.append("brak postoju przed wyzwoleniem")
    i0 = min(ti, i_edge0)
    # okno spoczynku: ciągły postój kończący się w i_stop (najwyżej do i0)
    rest_start = 0
    if i_stop >= 0:
        rest_start = i_stop
        while rest_start > 0 and stopped(rest_start - 1):
            rest_start -= 1

    # 3. bias z okna spoczynku przed startem (min. 50 próbek)
    rest_end = i0
    if rest_end - rest_start < 50:
        warnings.append("krótkie okno spoczynku przed startem")
        rest_start = max(0, rest_end - 50)
        rest_end = rest_start + min(50, n)
    rn = rest_end - rest_start
    def mean(x):
        return float(sum(float(v) for v in x[rest_start:rest_end]) / rn)
    def std(x):
        m = mean(x)
        return math.sqrt(sum((float(v) - m) ** 2 for v in x[rest_start:rest_end]) / rn)
    amag = np.sqrt(a_fwd ** 2 + a_left ** 2 + a_up ** 2)
    # rozrzut po filtrze lowpass_hz (tylko okno spoczynku) – drgania wentylatora
    # ESC (~180 Hz) nie mogą udawać ruchu auta
    def lp(x):
        y = np.zeros(n)
        y[rest_start:rest_end] = filtfilt_butter2(x[rest_start:rest_end], float(cfg["lowpass_hz"]), fs)
        return y
    at_rest = (max(std(lp(g_fwd)), std(lp(g_left)), std(lp(g_up))) < cfg["rest_gyro_std_dps"]
               and std(lp(amag)) < cfg["rest_acc_std_g"]
               and int(edges[rest_end - 1]) == int(edges[rest_start]))
    if not at_rest:
        warnings.append("auto nie stało przed startem – bias niepewny")
    a_ref = mean(a_fwd)
    gb_left = mean(g_left)

    # 4. pochylenie i a_long (surowe)
    dt = np.zeros(n)
    for i in range(1, n):
        dt[i] = t[i] - t[i - 1]
    pitch = np.zeros(n)
    acc = 0.0
    for i in range(n):
        acc += -(g_left[i] - gb_left) * dt[i] * (math.pi / 180.0)
        pitch[i] = acc
    a_long_raw = np.array([G * (a_fwd[i] - a_ref - math.sin(pitch[i])) for i in range(n)])

    # 5. filtr
    fc = float(cfg["lowpass_hz"])
    a_long = filtfilt_butter2(a_long_raw, fc, fs)

    # 6. v_imu od startu
    v_imu = np.zeros(n)
    for i in range(i0 + 1, n):
        v_imu[i] = v_imu[i - 1] + a_long[i] * dt[i]

    # 7. v_front
    epr = int(h["edges_per_rev"])
    m_per_rev = math.pi * cfg["front_tire_diameter_mm"] * 1e-3 / cfg["final_drive_ratio"]
    use_edges = h["n_edges"] > 0 and not (h["flags"] & 0x0001)
    # zbocza po filtrze drgnięć (starsze nagrania miały filtr 50 µs)
    edge_min_s = max(float(h["hall_min_edge_us"]), float(cfg["edge_min_us"])) * 1e-6
    edges_t = filter_edges(run.edges_t_s, run.edges_line, edge_min_s) if h["n_edges"] > 0 else []
    if use_edges:
        v_front = v_front_from_edges(t, edges_t, epr, timeout_s, m_per_rev)
    else:
        v_front = v_front_from_periods(s["period_rev_us"], m_per_rev)
    v_front_sample = v_front_from_periods(s["period_rev_us"], m_per_rev)

    # 8. a_front
    v_front_f = filtfilt_butter2(v_front, fc, fs)
    a_front = gradient(v_front_f, t)

    # 9.–10.
    vmin = float(cfg["v_min_mps"])
    slip = np.array([(v_front[i] - v_imu[i]) / max(v_front[i], vmin) if i >= i0 else 0.0
                     for i in range(n)])
    spin = a_front - a_long

    # 11. droga
    d_front = np.array([max(0, int(edges[i]) - int(edges[i0])) / epr * m_per_rev if i >= i0 else 0.0
                        for i in range(n)])
    d_imu = np.zeros(n)
    for i in range(i0 + 1, n):
        d_imu[i] = d_imu[i - 1] + v_imu[i] * dt[i]

    # 12. v_disp – estymator (wejście jak na Xiao: v_front z period_rev_us)
    neutral = h["throttle_neutral_us"]
    full = h["throttle_full_us"] or 500
    sign = 1.0 if h["throttle_fwd_higher"] else -1.0
    thr_pct = np.array([((int(w) - neutral) * 100.0 / full * sign) if (w and neutral) else 0.0
                        for w in s["throttle_us"]])
    p = dict(DEFAULT_PARAMS)
    p.update(cfg.get("estimator", {}))
    st = EstState()
    v_disp = np.zeros(n)
    for i in range(n):
        if i > 0:
            stopped = s["period_rev_us"][i] == 0 and edges[i] == edges[i - 1]
            est_step(st, p, dt[i], a_fwd[i], a_left[i], a_up[i], g_fwd[i], g_left[i], g_up[i],
                     v_front_sample[i], stopped, thr_pct[i])
        v_disp[i] = st.v

    # czas pierwszego zbocza (względem wyzwolenia; ujemny = silnik ruszył przed progiem)
    t_first_edge = float("nan")
    # (pierwsze zbocze po próbce postoju i_stop)
    if h["n_edges"] > 0 and i_stop >= 0:
        t_stop = float(t[i_stop])
        for te in edges_t:
            if te > t_stop:
                t_first_edge = float(te)
                break
    elif i_edge0 < n:
        t_first_edge = float(t[i_edge0])

    series = dict(t=t, throttle_pct=thr_pct, v_front=v_front, v_imu=v_imu, v_disp=v_disp,
                  a_front=a_front, a_long=a_long, slip=slip, spin=spin, pitch_deg=np.degrees(pitch),
                  d_front=d_front, d_imu=d_imu,
                  # obroty silnika z v_front (niezależne od FDR i średnicy opony)
                  rpm=np.array([float(v) / m_per_rev * 60.0 for v in v_front]))
    metrics = compute_metrics(series, i0, vmin, t_first_edge)
    return dict(series=series, metrics=metrics, warnings=warnings, at_rest=at_rest,
                header=h, used_raw_edges=use_edges, i0=i0)


def _window_mean(t, y, t0, t1):
    vals = [float(y[i]) for i in range(len(t)) if t0 <= t[i] < t1]
    return sum(vals) / len(vals) if vals else float("nan")


def compute_metrics(sr: dict, i0: int, vmin: float, t_first_edge: float) -> dict:
    """Metryki względem chwili startu t0 = t[i0]. esc_delay_s – od progu gazu (t = 0)."""
    t = sr["t"]
    ti = i0
    t0 = float(t[i0])
    n = len(t)
    peak_slip, peak_t = float("nan"), float("nan")
    for i in range(ti, n):
        if sr["v_front"][i] >= vmin and (math.isnan(peak_slip) or sr["slip"][i] > peak_slip):
            peak_slip, peak_t = float(sr["slip"][i]), float(t[i] - t0)
    spin_int = 0.0
    for i in range(ti + 1, n):
        x = sr["spin"][i]
        if x > 0:
            spin_int += x * (t[i] - t[i - 1])
    rpm_max, rpm_max_t = 0.0, float("nan")
    for i in range(ti, n):
        if sr["rpm"][i] > rpm_max:
            rpm_max, rpm_max_t = float(sr["rpm"][i]), float(t[i] - t0)
    t1m = first_crossing(t, sr["d_imu"], 1.0, ti)
    t2m = first_crossing(t, sr["d_imu"], 2.0, ti)
    return dict(
        a_long_0_300ms=_window_mean(t, sr["a_long"], t0, t0 + 0.3),
        a_long_0_1000ms=_window_mean(t, sr["a_long"], t0, t0 + 1.0),
        v_imu_0_5s=interp_at(t, sr["v_imu"], t0 + 0.5),
        v_imu_1_0s=interp_at(t, sr["v_imu"], t0 + 1.0),
        v_max=float(max(sr["v_disp"])),
        t_1m=t1m - t0,
        t_2m=t2m - t0,
        peak_slip=peak_slip,
        peak_slip_t=peak_t,
        spin_integral=spin_int,
        esc_delay_s=t_first_edge,
        rpm_max=rpm_max,
        rpm_max_t=rpm_max_t,
        t_launch_s=t0,
    )


# ------------------------------------------------------------------ wyścig
# Analiza całego wyścigu (próbki uśrednione RACE_RATE_HZ). Spec wspólna z
# analyzeRace() w analysis.js. Podział na okrążenia – osobny krok (później).
def analyze_race(race: Race, cfg: dict | None = None) -> dict:
    cfg = cfg or load_config()
    h = race.header
    s = race.samples
    n = len(s)
    warnings = []
    if n < 2:
        raise ValueError("Za mało próbek wyścigu")
    fs = float(h["sample_rate_hz"])
    t = race.t_s
    ti = min(int(h["trigger_index"]), n - 1)
    dt = np.array([float(s["n_ticks"][i]) / float(h["tick_rate_hz"]) for i in range(n)])

    ka, kg = float(h["accel_scale_g"]), float(h["gyro_scale_dps"])
    a_fwd, a_left, a_up = map_axes(s["ax"] * ka, s["ay"] * ka, s["az"] * ka, cfg["imu_axes"])
    g_fwd, g_left, g_up = map_axes(s["gx"] * kg, s["gy"] * kg, s["gz"] * kg, cfg["imu_axes"])

    # bias: próbki przed gazem bez zboczy (auto stało na starcie)
    rest = [i for i in range(ti) if int(s["edges"][i]) == 0]
    if len(rest) >= 10:
        rn = float(len(rest))
        a_ref_f = sum(float(a_fwd[i]) for i in rest) / rn
        a_ref_l = sum(float(a_left[i]) for i in rest) / rn
        gb_up = sum(float(g_up[i]) for i in rest) / rn
    else:
        warnings.append("brak postoju przed startem – bez korekty biasu IMU")
        a_ref_f = a_ref_l = gb_up = 0.0

    neutral = h["throttle_neutral_us"]
    full = h["throttle_full_us"] or 500
    sign = 1.0 if h["throttle_fwd_higher"] else -1.0
    thr_pct = np.array([((int(w) - neutral) * 100.0 / full * sign) if (w and neutral) else 0.0
                        for w in s["throttle_us"]])

    epr = int(h["edges_per_rev"])
    m_per_rev = math.pi * cfg["front_tire_diameter_mm"] * 1e-3 / cfg["final_drive_ratio"]
    unit = float(h["period_unit_us"])
    # Obroty z okresu obrotu; niezgodne z licznikiem zboczy (drgnięcie wirnika przy ruszaniu:
    # kilka zboczy w ~1 ms udaje 60 000 rpm) → obroty z liczby zboczy w 2 próbkach.
    rpm = np.zeros(n)
    for i in range(n):
        p = int(s["period_rev"][i])
        rp = 60e6 / (p * unit) if p > 0 else 0.0
        e2 = int(s["edges"][i]) + (int(s["edges"][i - 1]) if i > 0 else 0)
        t2 = dt[i] + (dt[i - 1] if i > 0 else 0.0)
        exp2 = rp / 60.0 * epr * t2
        if exp2 >= 6.0 and e2 < exp2 / 3.0:
            rp = e2 / epr / t2 * 60.0
        rpm[i] = rp
    v_front = np.array([float(r) / 60.0 * m_per_rev for r in rpm])
    d_front = np.zeros(n)
    acc = 0
    for i in range(n):
        acc += int(s["edges"][i])
        d_front[i] = acc / epr * m_per_rev

    fc = float(cfg["race_lowpass_hz"])
    a_long = filtfilt_butter2([G * (float(a_fwd[i]) - a_ref_f) for i in range(n)], fc, fs)
    a_lat = filtfilt_butter2([G * (float(a_left[i]) - a_ref_l) for i in range(n)], fc, fs)
    yaw_rate = np.array([float(g_up[i]) - gb_up for i in range(n)])
    heading = np.zeros(n)
    hd = 0.0
    for i in range(n):
        if i > 0:
            hd += yaw_rate[i] * dt[i]
        heading[i] = hd

    # koniec jazdy: ostatnia próbka z ruchem albo gazem poza neutrum
    i_last = ti
    for i in range(n - 1, ti - 1, -1):
        if int(s["edges"][i]) > 0 or abs(thr_pct[i]) >= 10.0:
            i_last = i
            break
    gaps = int(sum(1 for f in s["flags"] if int(f) & RSF_GAP))
    if gaps:
        warnings.append(f"{gaps} przerw w zapisie (bufor RAM pełny)")

    def arg_max(y, i_from, i_to):
        best, bi = float("-inf"), i_from
        for i in range(i_from, i_to + 1):
            if y[i] > best:
                best, bi = float(y[i]), i
        return best, bi

    vmax, iv = arg_max(v_front, ti, i_last)
    rmax, ir = arg_max(rpm, ti, i_last)
    alat_max, _ = arg_max([abs(float(x)) for x in a_lat], ti, i_last)
    along_max, _ = arg_max(a_long, ti, i_last)
    brake_max, _ = arg_max([-float(x) for x in a_long], ti, i_last)
    cnt = i_last - ti + 1
    full_thr = sum(1 for i in range(ti, i_last + 1) if thr_pct[i] > 90.0) / cnt
    metrics = dict(
        duration_s=float(t[i_last] - t[ti]),
        distance_m=float(d_front[i_last] - d_front[ti]),
        v_front_max=vmax,
        v_front_max_t=float(t[iv]),
        v_front_avg=float((d_front[i_last] - d_front[ti]) / max(t[i_last] - t[ti], 1e-9)),
        rpm_max=rmax,
        rpm_max_t=float(t[ir]),
        a_long_max_g=along_max / G,
        brake_max_g=brake_max / G,
        a_lat_max_g=alat_max / G,
        full_throttle_frac=full_thr,
        heading_total_deg=float(heading[i_last] - heading[ti]),
        gaps=gaps,
    )
    ds = [int(s["edges"][i]) / epr * m_per_rev for i in range(n)]
    v_track = filtfilt_butter2([ds[i] / dt[i] for i in range(n)], float(cfg["race_track_v_lowpass_hz"]), fs)
    laps, track = race_laps(ds, heading, v_track, t, ti, i_last, cfg)
    good = [lp for lp in laps if lp["good"]]
    metrics["laps_n"] = len(laps)
    metrics["best_lap_s"] = min(lp["time_s"] for lp in good) if good else float("nan")
    metrics["track_length_m"] = track["length_m"] if track else float("nan")
    series = dict(t=t, throttle_pct=thr_pct, rpm=rpm, v_front=v_front, d_front=d_front,
                  a_long=a_long, a_lat=a_lat, yaw_rate=yaw_rate, heading_deg=heading, v_track=v_track)
    return dict(series=series, metrics=metrics, warnings=warnings, header=h,
                end_reason=race.end_reason, i_start=ti, i_last=i_last, laps=laps, track=track)


def interp_lin(xq, xp, fp) -> list:
    """Interpolacja liniowa dla rosnących xq (xp niemalejące); poza zakresem – wartości brzegowe.
    Przy równych xp bierze pierwszy punkt z xp ≥ x."""
    out = []
    k = 0
    m = len(xp)
    for x in xq:
        while k < m - 1 and xp[k] < x:
            k += 1
        if k == 0 or xp[k] < x:
            out.append(float(fp[k]))
        elif xp[k] == xp[k - 1]:
            out.append(float(fp[k]))
        else:
            f = (x - xp[k - 1]) / (xp[k] - xp[k - 1])
            out.append(float(fp[k - 1] + f * (fp[k] - fp[k - 1])))
    return out


def align_2d(px, py, rx, ry):
    """Obrót + przesunięcie punktów P na wzorzec R (najmniejsze kwadraty, postać zamknięta)."""
    n = len(px)
    pcx, pcy = sum(px) / n, sum(py) / n
    rcx, rcy = sum(rx) / n, sum(ry) / n
    sc = ss = 0.0
    for i in range(n):
        ax, ay = px[i] - pcx, py[i] - pcy
        bx, by = rx[i] - rcx, ry[i] - rcy
        sc += ax * bx + ay * by
        ss += ax * by - ay * bx
    th = math.atan2(ss, sc)
    c, s_ = math.cos(th), math.sin(th)
    ox = [c * (px[i] - pcx) - s_ * (py[i] - pcy) + rcx for i in range(n)]
    oy = [s_ * (px[i] - pcx) + c * (py[i] - pcy) + rcy for i in range(n)]
    return ox, oy


def race_laps(ds, heading, v_track, t, i0, i1, cfg):
    """Okrążenia i tor (tor stały, jazda w kółko).
    Pozycja: droga z licznika zboczy × kierunek z żyroskopu. Granica okrążenia: kurs od startu
    przekracza kolejne wielokrotności 360° (w kierunku jazdy). Okrążenia z drogą odbiegającą od
    mediany o > race_lap_dist_tol (buksowanie, uderzenie) – odrzucone. Tor = średnia dobrych
    okrążeń przepróbkowanych po drodze i dopasowanych (obrót + przesunięcie) do wzorca."""
    n = len(ds)
    x, y = [0.0] * n, [0.0] * n
    for i in range(1, n):
        a = math.radians(heading[i])
        x[i] = x[i - 1] + ds[i] * math.cos(a)
        y[i] = y[i - 1] + ds[i] * math.sin(a)
    direction = 1.0 if heading[i1] - heading[i0] >= 0 else -1.0
    bounds = [i0]
    k = 1
    for i in range(i0, i1 + 1):
        if (heading[i] - heading[i0]) * direction >= 360.0 * k:
            bounds.append(i)
            k += 1
    laps = []
    for j in range(len(bounds) - 1):
        a, b = bounds[j], bounds[j + 1]
        laps.append(dict(i_a=a, i_b=b, t_start=float(t[a]), time_s=float(t[b] - t[a]),
                         dist_m=float(sum(ds[a + 1:b + 1])),
                         v_max=float(max(v_track[a:b + 1])), good=False, err_m=float("nan")))
    if len(laps) < 2:
        return laps, None
    dists = sorted(lp["dist_m"] for lp in laps)
    m = len(dists)
    med = dists[m // 2] if m % 2 else 0.5 * (dists[m // 2 - 1] + dists[m // 2])
    tol = float(cfg["race_lap_dist_tol"])
    for lp in laps:
        lp["good"] = abs(lp["dist_m"] - med) < tol * med
    good = [j for j, lp in enumerate(laps) if lp["good"]]
    if not good:
        return laps, None
    npts = int(cfg["race_track_points"])
    rs = []
    for lp in laps:
        a, b = lp["i_a"], lp["i_b"]
        d = [0.0]
        for i in range(a + 1, b + 1):
            d.append(d[-1] + ds[i])
        q = [d[-1] * k / (npts - 1) for k in range(npts)]
        rs.append((interp_lin(q, d, x[a:b + 1]), interp_lin(q, d, y[a:b + 1]),
                   interp_lin(q, d, v_track[a:b + 1])))
    jref = good[0]
    for j in good:
        if abs(laps[j]["dist_m"] - med) < abs(laps[jref]["dist_m"] - med):
            jref = j
    rx, ry = rs[jref][0], rs[jref][1]
    al = []
    for _ in range(3):
        al = [align_2d(rs[j][0], rs[j][1], rx, ry) for j in range(len(laps))]
        rx = [sum(al[j][0][k] for j in good) / len(good) for k in range(npts)]
        ry = [sum(al[j][1][k] for j in good) / len(good) for k in range(npts)]
    for j, lp in enumerate(laps):
        lp["err_m"] = sum(math.hypot(al[j][0][k] - rx[k], al[j][1][k] - ry[k]) for k in range(npts)) / npts
    tv = [sum(rs[j][2][k] for j in good) / len(good) for k in range(npts)]
    length = sum(math.hypot(rx[k] - rx[k - 1], ry[k] - ry[k - 1]) for k in range(1, npts))
    track = dict(x=rx, y=ry, v=tv, length_m=length,
                 laps_x=[al[j][0] for j in range(len(laps))], laps_y=[al[j][1] for j in range(len(laps))],
                 laps_v=[rs[j][2] for j in range(len(laps))])
    return laps, track


def analyze_file(path, cfg=None):
    data = open(path, "rb").read()
    if file_kind(data) == "race":
        return analyze_race(parse_race(data), cfg)
    return analyze(parse_run(data), cfg)


def main(argv=None):
    import argparse
    import json

    ap = argparse.ArgumentParser(description="Metryki startu")
    ap.add_argument("files", nargs="+")
    ap.add_argument("--config")
    a = ap.parse_args(argv)
    cfg = load_config(a.config)
    for f in a.files:
        r = analyze_file(f, cfg)
        print(f, json.dumps({k: round(v, 4) for k, v in r["metrics"].items()}, ensure_ascii=False))
        for w in r["warnings"]:
            print("  !", w)


if __name__ == "__main__":
    main()
