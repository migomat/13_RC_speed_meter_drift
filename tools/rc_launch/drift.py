"""Punkty za drift – IMPLEMENTACJA REFERENCYJNA.

Ten sam algorytm: firmware/src/drift.c (na żywo, float32) i app/js/drift.js.
Każda zmiana = zmiana we wszystkich trzech + przejście wektorów (test_vectors/drift_*).

Wejście: próbki wyścigu (RACE_RATE_HZ, średnie z taktów 500 Hz) – na Xiao ten sam
strumień, który trafia do pamięci wyścigów, więc wynik z nagrania = wynik na żywo
(poza stanem początkowym: Xiao liczy od włączenia, nagranie od pre-triggera).

Kąt driftu z kierunku przyspieszenia (tylko IMU – koła napędzane w drifcie buksują):
przy jeździe po łuku przyspieszenie poziome jest prostopadłe do wektora prędkości,
więc jego kierunek w osiach auta wyznacza kąt między osią auta a kierunkiem jazdy.

Krok (dt [s], a_fwd / a_left [g] i ω_up [dps] w osiach auta, rest = postój):
 1. obrót poziomy o yaw_offset (jak estymator): af = a_f·cosψ + a_l·sinψ, al = −a_f·sinψ + a_l·cosψ
 2. bias (af, al, ω) – średnia wykładnicza w postoju (bias_tau_s); pierwszy postój = wartość
 3. filtr IIR 1. rzędu (lpf_tau_s) af, al, ω; dt ograniczone do max_dt_s (przerwy w zapisie)
 4. jazda po łuku: |ω| ≥ min_yaw_dps ∧ |a| ≥ min_acc_g; corner_t = czas od wejścia w łuk
    V_raw = min(|a|·G / |ω|, v_max) [m/s] (prędkość z promienia skrętu)
    corner_t < steady_s: V = V_raw, V' = 0 (próbki mieszane: hamowanie + skręt)
    inaczej: V filtr v_tau_s; V' = filtr vdot_tau_s z dV/dt, gdy |Δω|/dt < steady_yaw_acc,
    w przeciwnym razie z 0 (przy zmianie ω prędkość z promienia jest błędna)
 5. s = znak ω; θ0 = atan2(af, al·s); β0 = −s·θ0 (kierunek prędkości w osiach auta)
    usunięcie przyspieszenia stycznego: af −= V'/G·cos β0, al −= V'/G·sin β0
    θ = atan2(af, al·s) [deg] – kąt driftu, > 0 = nadsterowność (tył na zewnątrz)
 6. drift = łuk ∧ V ≥ min_v_mps ∧ min_angle_deg ≤ θ ≤ spin_angle_deg
    obrót = łuk ∧ |θ| > spin_angle_deg przez spin_s
 7. łańcuch (combo): segment driftu w jednym kierunku (s); segment potwierdzony po min_seg_s;
    potwierdzony segment w przeciwnym kierunku niż poprzedni = przekładka → mnożnik +1 (do max_mult)
    punkty w drifcie: V·dt·(pts_per_m + pts_per_m_deg·θ)
    brak driftu dłużej niż chain_gap_s → łańcuch zaliczony: suma += punkty·mnożnik
    (łańcuch bez potwierdzonego segmentu – odrzucony, to szum)
    obrót w trakcie łańcucha → łańcuch stracony (spin_loses_chain) albo zaliczony
"""
from __future__ import annotations

import math

import numpy as np

G = 9.80665

DEFAULT_DRIFT = dict(
    lpf_tau_s=0.05,        # [s] filtr przyspieszeń i ω (próbka 20 Hz to już średnia z 25 taktów)
    bias_tau_s=1.0,        # [s] uśrednianie biasu w postoju
    v_tau_s=0.15,          # [s] filtr prędkości z promienia
    vdot_tau_s=0.3,        # [s] filtr przyspieszenia stycznego
    steady_yaw_acc_dps2=150.0,  # [dps/s] powyżej (zmiana ω) przyspieszenie styczne nieznane → 0
    steady_s=0.25,         # [s] po wejściu w łuk – wcześniej przyspieszenie styczne nieznane → 0
    max_dt_s=0.2,          # [s] ograniczenie kroku (przerwa w zapisie)
    min_yaw_dps=30.0,      # [dps] poniżej – jazda prosto, bez driftu
    min_acc_g=0.3,         # [g] poniżej – za słabe przyspieszenie, kąt nieokreślony (szum, ciasna jazda)
    min_v_mps=1.5,         # [m/s]
    v_max_mps=30.0,        # [m/s] ograniczenie prędkości z promienia
    min_angle_deg=15.0,    # [deg] kąt, od którego liczy się drift
    spin_angle_deg=80.0,   # [deg] powyżej przez spin_s = obrót
    spin_s=0.25,           # [s]
    chain_gap_s=1.0,       # [s] przerwa w drifcie, po której łańcuch jest zaliczany
    min_seg_s=0.3,         # [s] minimalny czas segmentu (przekładka, potwierdzenie łańcucha)
    pts_per_m=10.0,        # punkty za metr driftu
    pts_per_m_deg=1.0,     # punkty za metr × stopień kąta
    max_mult=10,           # maks. mnożnik
    spin_loses_chain=1,    # 1 = obrót kasuje punkty łańcucha, 0 = zalicza je
)


def drift_params(cfg: dict) -> dict:
    p = dict(DEFAULT_DRIFT)
    p.update((cfg or {}).get("drift", {}) or {})
    p["yaw_offset_deg"] = float(((cfg or {}).get("estimator") or {}).get("yaw_offset_deg", 0.0))
    return p


def drift_init() -> dict:
    return dict(
        have_bias=False, b_af=0.0, b_al=0.0, b_r=0.0,
        first=True, f_af=0.0, f_al=0.0, f_r=0.0,
        corner=False, corner_t=0.0, r_prev=0.0, v=0.0, vdot=0.0, angle=0.0, spin_t=0.0,
        in_drift=False, seg_t=0.0, seg_dir=0, seg_ok=False,
        chain=False, chain_ok=False, prev_dir=0, grace_t=0.0,
        pts=0.0, mult=1, total=0.0, best=0.0, last=0.0,
        n_banks=0, n_lost=0, n_trans=0, drift_t=0.0, angle_max=0.0,
    )


def _bank(s: dict) -> None:
    if s["chain_ok"]:
        amount = s["pts"] * s["mult"]
        s["total"] += amount
        s["last"] = amount
        s["best"] = max(s["best"], amount)
        s["n_banks"] += 1
    s["chain"] = s["chain_ok"] = False
    s["pts"] = 0.0
    s["mult"] = 1
    s["prev_dir"] = 0
    s["grace_t"] = 0.0


def drift_step(s: dict, p: dict, dt: float, a_fwd: float, a_left: float, r_dps: float, rest: bool) -> None:
    if dt <= 0:
        return
    dt = min(dt, p["max_dt_s"])
    yaw = math.radians(p["yaw_offset_deg"])
    c, sn = math.cos(yaw), math.sin(yaw)
    af = a_fwd * c + a_left * sn
    al = -a_fwd * sn + a_left * c

    if rest:
        if not s["have_bias"]:
            s["b_af"], s["b_al"], s["b_r"] = af, al, r_dps
            s["have_bias"] = True
        else:
            k = dt / (p["bias_tau_s"] + dt)
            s["b_af"] += k * (af - s["b_af"])
            s["b_al"] += k * (al - s["b_al"])
            s["b_r"] += k * (r_dps - s["b_r"])
    af -= s["b_af"]
    al -= s["b_al"]
    r = r_dps - s["b_r"]

    if s["first"]:
        s["f_af"], s["f_al"], s["f_r"] = af, al, r
        s["first"] = False
    else:
        k = dt / (p["lpf_tau_s"] + dt)
        s["f_af"] += k * (af - s["f_af"])
        s["f_al"] += k * (al - s["f_al"])
        s["f_r"] += k * (r - s["f_r"])
    fa, fl, fr = s["f_af"], s["f_al"], s["f_r"]

    amag = math.sqrt(fa * fa + fl * fl)
    rr = abs(fr)
    corner = rr >= p["min_yaw_dps"] and amag >= p["min_acc_g"]
    if corner:
        v_raw = min(amag * G / math.radians(rr), p["v_max_mps"])
        s["corner_t"] = s["corner_t"] + dt if s["corner"] else 0.0
        if s["corner_t"] < p["steady_s"]:
            # początek łuku: próbki mieszane (hamowanie + skręt) – V bez filtra, bez pochodnej
            s["v"] = v_raw
            s["vdot"] = 0.0
        else:
            v_prev = s["v"]
            s["v"] += dt / (p["v_tau_s"] + dt) * (v_raw - s["v"])
            # V z promienia jest poprawne tylko przy ustalonym kącie (ω auta = ω toru):
            # przy zmianie ω (przekładka) przyspieszenie styczne → 0
            steady = abs(fr - s["r_prev"]) / dt < p["steady_yaw_acc_dps2"]
            target = (s["v"] - v_prev) / dt if steady else 0.0
            s["vdot"] += dt / (p["vdot_tau_s"] + dt) * (target - s["vdot"])
        sg = 1.0 if fr >= 0 else -1.0
        b0 = -sg * math.atan2(fa, fl * sg)
        at = s["vdot"] / G
        fa2 = fa - at * math.cos(b0)
        fl2 = fl - at * math.sin(b0)
        angle = math.degrees(math.atan2(fa2, fl2 * sg))
        d = 1 if fr >= 0 else -1
    else:
        s["vdot"] = 0.0
        angle = 0.0
        d = 0
    if not corner:
        s["corner_t"] = 0.0
    s["corner"] = corner
    s["r_prev"] = fr
    s["angle"] = angle

    s["spin_t"] = s["spin_t"] + dt if (corner and abs(angle) > p["spin_angle_deg"]) else 0.0
    drifting = (corner and s["v"] >= p["min_v_mps"]
                and p["min_angle_deg"] <= angle <= p["spin_angle_deg"])

    if s["chain"] and s["spin_t"] >= p["spin_s"]:
        if p["spin_loses_chain"]:
            s["n_lost"] += 1
            s["chain_ok"] = False   # _bank() bez zaliczenia
        _bank(s)
        s["in_drift"] = False
    elif drifting:
        if not s["in_drift"] or d != s["seg_dir"]:
            s["in_drift"] = True
            s["seg_t"] = 0.0
            s["seg_dir"] = d
            s["seg_ok"] = False
            if not s["chain"]:
                s["chain"] = True
                s["chain_ok"] = False
                s["pts"] = 0.0
                s["mult"] = 1
                s["prev_dir"] = 0
        s["seg_t"] += dt
        s["grace_t"] = 0.0
        if not s["seg_ok"] and s["seg_t"] >= p["min_seg_s"]:
            s["seg_ok"] = True
            s["chain_ok"] = True
            if s["prev_dir"] != 0 and s["seg_dir"] != s["prev_dir"]:
                s["mult"] = min(s["mult"] + 1, int(p["max_mult"]))
                s["n_trans"] += 1
            s["prev_dir"] = s["seg_dir"]
        s["pts"] += s["v"] * dt * (p["pts_per_m"] + p["pts_per_m_deg"] * angle)
        s["drift_t"] += dt
        s["angle_max"] = max(s["angle_max"], angle)
    else:
        s["in_drift"] = False
        if s["chain"]:
            s["grace_t"] += dt
            if s["grace_t"] >= p["chain_gap_s"]:
                _bank(s)


def race_inputs(race, cfg: dict):
    """Wejścia kroku z próbek wyścigu: dt, a_fwd, a_left [g], ω_up [dps], rest – jak w firmware."""
    from .analyze import map_axes

    h = race.header
    smp = race.samples
    ka, kg = float(h["accel_scale_g"]), float(h["gyro_scale_dps"])
    axes = cfg.get("imu_axes", dict(forward="+x", left="+y", up="+z"))
    af, al, _ = map_axes(smp["ax"].astype(np.float64) * ka, smp["ay"].astype(np.float64) * ka,
                         smp["az"].astype(np.float64) * ka, axes)
    _, _, gu = map_axes(smp["gx"].astype(np.float64) * kg, smp["gy"].astype(np.float64) * kg,
                        smp["gz"].astype(np.float64) * kg, axes)
    dt = smp["n_ticks"].astype(np.float64) / float(h["tick_rate_hz"])
    neutral, full = int(h["throttle_neutral_us"]), int(h["throttle_full_us"]) or 500
    sign = 1.0 if h["throttle_fwd_higher"] else -1.0
    rest = []
    for e, w in zip(smp["edges"], smp["throttle_us"]):
        pct = (int(w) - neutral) * 100.0 / full * sign if (w and neutral) else 0.0
        rest.append(int(e) == 0 and abs(pct) < 10.0)
    return dt, af, al, gu, rest


def drift_score(race, cfg: dict | None = None) -> dict:
    """Przebiegi i podsumowanie punktów za drift dla nagrania wyścigu."""
    cfg = cfg or {}
    p = drift_params(cfg)
    dt, af, al, gu, rest = race_inputs(race, cfg)
    n = len(dt)
    s = drift_init()
    out = {k: np.zeros(n) for k in ("angle", "v", "pts", "mult", "total", "drift")}
    for i in range(n):
        drift_step(s, p, float(dt[i]), float(af[i]), float(al[i]), float(gu[i]), bool(rest[i]))
        out["angle"][i] = s["angle"]
        out["v"][i] = s["v"] if s["corner"] else 0.0
        out["pts"][i] = s["pts"]
        out["mult"][i] = s["mult"]
        out["total"][i] = s["total"]
        out["drift"][i] = 1.0 if s["in_drift"] else 0.0
    # łańcuch trwający na końcu nagrania – zaliczony
    end = dict(s)
    _bank(end)
    summary = dict(total=end["total"], best_chain=end["best"], chains=end["n_banks"],
                   transitions=s["n_trans"], spins=s["n_lost"], drift_time_s=s["drift_t"],
                   angle_max_deg=s["angle_max"])
    out["t"] = race.t_s
    return dict(series=out, summary=summary)


# ------------------------------------------------------------------ kalibracja montażu
def _rest_mask(race) -> list:
    h = race.header
    smp = race.samples
    neutral, full = int(h["throttle_neutral_us"]), int(h["throttle_full_us"]) or 500
    sign = 1.0 if h["throttle_fwd_higher"] else -1.0
    out = []
    for e, w in zip(smp["edges"], smp["throttle_us"]):
        pct = (int(w) - neutral) * 100.0 / full * sign if (w and neutral) else 0.0
        out.append(int(e) == 0 and abs(pct) < 10.0)
    return out


def mount_calibrate(race, min_yaw_dps: float = 40.0, min_acc_g: float = 0.1, lpf_tau_s: float = 0.15,
                    trim_deg: float = 25.0) -> dict:
    """Orientacja płytki w aucie z nagrania jazdy BEZ driftu (zwykłe zakręty z przyczepnością).

    Oś „up” = oś płytki z największą |średnią a| w postoju (grawitacja).
    W zakręcie przyspieszenie poziome × znak ω_up wskazuje „left” (przy przyczepności
    prędkość jest wzdłuż osi auta) – średnia ważona |ω| po próbkach z jazdą po łuku.
    Wynik: imu_axes (najbliższe ustawienie co 90°) + yaw_offset_deg (reszta, −45…45°).
    """
    h = race.header
    smp = race.samples
    ka, kg = float(h["accel_scale_g"]), float(h["gyro_scale_dps"])
    a = [smp[k].astype(np.float64) * ka for k in ("ax", "ay", "az")]
    g = [smp[k].astype(np.float64) * kg for k in ("gx", "gy", "gz")]
    rest = _rest_mask(race)
    # postój przed startem (auto na kołach); wywrotka z postojem później nie psuje osi „up”
    ti = int(h["trigger_index"])
    ri = [i for i, r in enumerate(rest) if r and i < ti]
    if len(ri) < 10:
        ri = [i for i, r in enumerate(rest) if r]
    if len(ri) < 10:
        raise ValueError("za mało próbek postoju (auto musi chwilę stać)")
    am = [float(np.mean(x[ri])) for x in a]
    gb = [float(np.mean(x[ri])) for x in g]
    up = max(range(3), key=lambda k: abs(am[k]))
    us = 1.0 if am[up] > 0 else -1.0
    e1, e2 = ((up + 1) % 3, (up + 2) % 3) if us > 0 else ((up + 2) % 3, (up + 1) % 3)
    dt = smp["n_ticks"].astype(np.float64) / float(h["tick_rate_hz"])
    f1 = f2 = fr = 0.0
    pts = []   # (kąt kierunku „left”, waga)
    for i in range(len(dt)):
        h1, h2 = a[e1][i] - am[e1], a[e2][i] - am[e2]
        r = (g[up][i] - gb[up]) * us
        if i == 0:
            f1, f2, fr = h1, h2, r
        else:
            k = dt[i] / (lpf_tau_s + dt[i])
            f1 += k * (h1 - f1)
            f2 += k * (h2 - f2)
            fr += k * (r - fr)
        if abs(fr) >= min_yaw_dps and math.hypot(f1, f2) >= min_acc_g and int(smp["edges"][i]) > 0:
            s = 1.0 if fr > 0 else -1.0
            pts.append((math.atan2(f2 * s, f1 * s), abs(fr) * math.hypot(f1, f2)))
    if len(pts) < 20:
        raise ValueError("za mało zakrętów w nagraniu (potrzebne kilka okrążeń)")
    # średnia kołowa ważona, potem 2× średnia z odrzuceniem próbek dalej niż trim_deg
    # (hamowanie przed zakrętem i gaz na wyjściu przechylają kierunek przyspieszenia)
    phi = math.atan2(sum(w * math.sin(t) for t, w in pts), sum(w * math.cos(t) for t, w in pts))
    sel = pts
    for _ in range(2):
        sel = [(t, w) for t, w in pts if abs(math.remainder(t - phi, 2 * math.pi)) < math.radians(trim_deg)]
        if len(sel) < 20:
            sel = pts
            break
        phi = math.atan2(sum(w * math.sin(t) for t, w in sel), sum(w * math.cos(t) for t, w in sel))
    angs = [t for t, _ in pts]
    n = len(sel)
    phi_left = math.degrees(phi)
    phi_f = (phi_left - 90.0 + 180.0) % 360.0 - 180.0
    q = round(phi_f / 90.0) * 90
    yaw = phi_f - q
    names = "xyz"

    def spec(ang):
        ang = int(ang) % 360
        return {0: f"+{names[e1]}", 90: f"+{names[e2]}", 180: f"-{names[e1]}", 270: f"-{names[e2]}"}[ang]

    # rozrzut kierunku (kołowe odchylenie standardowe) – jakość kalibracji
    c = np.mean(np.cos(angs))
    s_ = np.mean(np.sin(angs))
    R = min(1.0, math.hypot(c, s_))
    spread = math.degrees(math.sqrt(max(0.0, -2.0 * math.log(max(R, 1e-9)))))
    return dict(imu_axes=dict(forward=spec(q), left=spec(q + 90), up=("+" if us > 0 else "-") + names[up]),
                yaw_offset_deg=yaw, samples=n, spread_deg=spread)
