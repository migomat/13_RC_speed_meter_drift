"""Punkty za drift – IMPLEMENTACJA REFERENCYJNA.

Ten sam algorytm: firmware/src/drift.c (na żywo, float32) i app/js/drift.js.
Każda zmiana = zmiana we wszystkich trzech + przejście wektorów (test_vectors/drift_*).

Wejście: próbki wyścigu (RACE_RATE_HZ, średnie z taktów 500 Hz) – na Xiao ten sam
strumień, który trafia do pamięci wyścigów, więc wynik z nagrania = wynik na żywo
(poza stanem początkowym: Xiao liczy od włączenia, nagranie od pre-triggera).

Kąt driftu z kierunku przyspieszenia (tylko IMU – koła napędzane w drifcie buksują,
auto do driftu może nie mieć czujnika halla): przy jeździe po łuku przyspieszenie poziome
jest prostopadłe do wektora prędkości, więc jego kierunek w osiach auta wyznacza kąt między
osią auta a kierunkiem jazdy. Skręt (opcjonalnie): kontra potwierdza drift.

Krok (dt [s], a_fwd / a_left [g] i ω_up [dps] w osiach auta, rest_hint = brak zboczy halla
i gaz w neutrum, steer_raw = skręt [% od neutrum, znak surowy] albo NaN):
 1. obrót poziomy o yaw_offset (jak estymator): af = a_f·cosψ + a_l·sinψ, al = −a_f·sinψ + a_l·cosψ
 2. postój: rest_hint ∧ IMU spokojne (|Δaf|, |Δal| < rest_da_g, |Δω| < rest_dr_dps między próbkami)
    przez rest_min_s (bez halla toczenie się bez gazu też jest „bez zboczy” – stąd czas)
    bias (af, al, ω) – średnia wykładnicza w postoju (bias_tau_s); pierwszy postój = wartość
 3. filtr IIR 1. rzędu (lpf_tau_s) af, al, ω; dt ograniczone do max_dt_s (przerwy w zapisie)
 4. skręt: znak = steer_sign (ustawiony / z kalibracji) albo nauczony (gdy steer_sign = 0);
    steer = steer_raw·znak (+ = w lewo); kontra = |steer| ≥ cs_min_pct ∧ znak(steer) = −znak(ω)
 5. geo: |ω| ≥ min_yaw_dps ∧ |a| ≥ min_acc_cs_g – liczony kąt i V (corner_t = czas od wejścia);
    łuk do punktów (corner): geo ∧ |a| ≥ a_min, a_min = min_acc_cs_g, gdy skręt znany i kontra
    wymagana (cs_required), inaczej min_acc_g
    V_raw = min(|a|·G / |ω|, v_max) [m/s] (prędkość z promienia skrętu)
    corner_t < steady_s: V = V_raw, V' = 0 (próbki mieszane: hamowanie + skręt)
    inaczej: V filtr v_tau_s; V' = filtr vdot_tau_s z dV/dt, gdy |Δω|/dt < steady_yaw_acc,
    w przeciwnym razie z 0 (przy zmianie ω prędkość z promienia jest błędna)
 6. s = znak ω; θ0 = atan2(af, al·s); β0 = −s·θ0 (kierunek prędkości w osiach auta)
    usunięcie przyspieszenia stycznego: af −= V'/G·cos β0, al −= V'/G·sin β0
    θ = atan2(af, al·s) [deg] – kąt driftu, > 0 = nadsterowność (tył na zewnątrz)
 7. nauka znaku skrętu (steer_sign = 0, jeszcze nieznany, |steer_raw| ≥ cs_min_pct, geo;
    kąt wymaga skalibrowanego montażu):
    |θ| < min_angle_deg (przyczepność – skręt w stronę zakrętu): ev += dt·znak(steer_raw·ω)
    θ ≥ learn_drift_deg (głęboki drift – kontra): ev −= dt·znak(steer_raw·ω)
    |ev| ≥ steer_learn_s → znak = znak(ev)
 8. drift = łuk ∧ V ≥ min_v_mps ∧ min_angle_deg ≤ θ ≤ spin_angle_deg ∧
    (skręt nieznany ∨ ¬cs_required ∨ kontra ∨ θ ≥ angle_no_cs_deg)
    obrót = łuk ∧ |θ| > spin_angle_deg przez spin_s
 9. pełny gaz: thr ≥ full_thr_pct nieprzerwanie → full_t += dt (inaczej 0); full_on = full_t ≥ full_thr_s;
    bonus (full_bonus = 1): punkty w drifcie × full_bonus_mult, gdy full_on (sygnał dla dźwięku: full_on)
10. łańcuch (combo): segment driftu w jednym kierunku (s); segment potwierdzony po min_seg_s;
    potwierdzony segment w przeciwnym kierunku niż poprzedni = przekładka → mnożnik +1 (do max_mult)
    punkty w drifcie: V·dt·(pts_per_m + pts_per_m_deg·θ) [× full_bonus_mult przy bonusie]
    brak driftu dłużej niż chain_gap_s → łańcuch zaliczony: suma += punkty·mnożnik
    (łańcuch bez potwierdzonego segmentu – odrzucony, to szum)
    obrót w trakcie łańcucha → łańcuch stracony (spin_loses_chain) albo zaliczony
"""
from __future__ import annotations

import math

import numpy as np

G = 9.80665
# [s] margines porównań sumowanych czasów: Σdt w float32 (Xiao) i float64 różni się o ułamek
# próbki – bez marginesu próg (np. 30 × 0,05 s ≥ 1,5 s) przeskakiwał o jedną próbkę
T_EPS = 1e-3

DEFAULT_DRIFT = dict(
    lpf_tau_s=0.05,        # [s] filtr przyspieszeń i ω (próbka 20 Hz to już średnia z 25 taktów)
    bias_tau_s=1.0,        # [s] uśrednianie biasu w postoju
    rest_da_g=0.05,        # [g] maks. zmiana a między próbkami w postoju
    rest_dr_dps=3.0,       # [dps] maks. zmiana ω między próbkami w postoju
    rest_min_s=1.5,        # [s] tak długo spokojnie (bez gazu) = postój
    v_tau_s=0.15,          # [s] filtr prędkości z promienia
    vdot_tau_s=0.3,        # [s] filtr przyspieszenia stycznego
    steady_yaw_acc_dps2=150.0,  # [dps/s] powyżej (zmiana ω) przyspieszenie styczne nieznane → 0
    steady_s=0.25,         # [s] po wejściu w łuk – wcześniej przyspieszenie styczne nieznane → 0
    max_dt_s=0.2,          # [s] ograniczenie kroku (przerwa w zapisie)
    min_yaw_dps=30.0,      # [dps] poniżej – jazda prosto, bez driftu
    min_acc_g=0.3,         # [g] poniżej – kąt nieokreślony (bez skrętu: szum, ciasna jazda)
    min_acc_cs_g=0.1,      # [g] jw., gdy kontra wymagana (skręt znany) – śliskie auto; też próg liczenia kąta
    min_v_mps=0.5,         # [m/s] (driftowóz 25.10: ~0,25 g, ~180 °/s → ~0,8 m/s)
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
    steer_sign=0,          # skręt: 1 normalny, −1 odwrócony, 0 = automatycznie (nauka)
    steer_learn_s=3.0,     # [s] zgodnej „przewagi” do ustalenia znaku skrętu
    learn_drift_deg=30.0,  # [deg] drift tak głęboki, że skręt to na pewno kontra (nauka znaku)
    cs_min_pct=8.0,        # [%] skręt od neutrum uznany za kontrę
    cs_required=1,         # 1 = przy znanym skręcie drift wymaga kontry (albo dużego kąta)
    angle_no_cs_deg=90.0,  # [deg] od tego kąta drift także bez kontry (90 = przy znanym skręcie kontra zawsze;
                           #   jazda 25.10: 35° przepuszczało szum z jazdy z przyczepnością)
    full_bonus=0,          # 1 = bonus za pełny gaz w drifcie (przełącznik w profilu auta)
    full_thr_pct=95.0,     # [%] gaz uznany za pełny
    full_thr_s=1.0,        # [s] tak długo nieprzerwanie = pełny gaz (full_on)
    full_bonus_mult=1.5,   # mnożnik punktów w drifcie przy pełnym gazie
)


def drift_params(cfg: dict) -> dict:
    p = dict(DEFAULT_DRIFT)
    p.update((cfg or {}).get("drift", {}) or {})
    p["yaw_offset_deg"] = float(((cfg or {}).get("estimator") or {}).get("yaw_offset_deg", 0.0))
    return p


def drift_init() -> dict:
    return dict(
        have_prev=False, p_af=0.0, p_al=0.0, p_r=0.0, quiet_t=0.0,
        have_bias=False, b_af=0.0, b_al=0.0, b_r=0.0,
        first=True, f_af=0.0, f_al=0.0, f_r=0.0,
        geo=False, corner=False, corner_t=0.0, r_prev=0.0, v=0.0, vdot=0.0, angle=0.0, spin_t=0.0,
        steer_ev=0.0, steer_learned=0, steer=math.nan, cs=False, full_t=0.0, full_on=False,
        full_drift_t=0.0,
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


def steer_sign_of(s: dict, p: dict) -> int:
    """Znak skrętu w użyciu: ustawiony albo nauczony (0 = nieznany)."""
    return int(p["steer_sign"]) if p["steer_sign"] else s["steer_learned"]


def drift_step(s: dict, p: dict, dt: float, a_fwd: float, a_left: float, r_dps: float,
               rest_hint: bool, steer_raw: float = math.nan, thr_pct: float = 0.0) -> None:
    if dt <= 0:
        return
    dt = min(dt, p["max_dt_s"])
    yaw = math.radians(p["yaw_offset_deg"])
    c, sn = math.cos(yaw), math.sin(yaw)
    af = a_fwd * c + a_left * sn
    al = -a_fwd * sn + a_left * c

    quiet = (s["have_prev"] and abs(af - s["p_af"]) < p["rest_da_g"]
             and abs(al - s["p_al"]) < p["rest_da_g"] and abs(r_dps - s["p_r"]) < p["rest_dr_dps"])
    s["p_af"], s["p_al"], s["p_r"] = af, al, r_dps
    s["have_prev"] = True
    s["quiet_t"] = s["quiet_t"] + dt if (rest_hint and quiet) else 0.0
    if s["quiet_t"] >= p["rest_min_s"] - T_EPS:
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

    sgn = steer_sign_of(s, p)
    steer_ok = sgn != 0 and not math.isnan(steer_raw)
    s["steer"] = steer_raw * sgn if steer_ok else math.nan
    gate = steer_ok and bool(p["cs_required"])

    amag = math.sqrt(fa * fa + fl * fl)
    rr = abs(fr)
    # geo: kąt i V liczone od niższego progu (nauka skrętu, podgląd); corner: łuk do punktów
    geo = rr >= p["min_yaw_dps"] and amag >= p["min_acc_cs_g"]
    corner = geo and amag >= (p["min_acc_cs_g"] if gate else p["min_acc_g"])
    if geo:
        v_raw = min(amag * G / math.radians(rr), p["v_max_mps"])
        s["corner_t"] = s["corner_t"] + dt if s["geo"] else 0.0
        if s["corner_t"] < p["steady_s"] - T_EPS:
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
        s["corner_t"] = 0.0
        angle = 0.0
        d = 0
    s["geo"] = geo
    s["corner"] = corner
    s["r_prev"] = fr
    s["angle"] = angle

    # nauka znaku skrętu (tylko automatycznie i dopóki nieznany; próg śliskiego auta – geo)
    if (not p["steer_sign"] and s["steer_learned"] == 0 and geo and not math.isnan(steer_raw)
            and abs(steer_raw) >= p["cs_min_pct"]):
        agree = 1.0 if steer_raw * fr > 0 else -1.0
        if abs(angle) < p["min_angle_deg"]:
            s["steer_ev"] += dt * agree
        elif angle >= p["learn_drift_deg"]:
            s["steer_ev"] -= dt * agree
        if abs(s["steer_ev"]) >= p["steer_learn_s"] - T_EPS:
            s["steer_learned"] = 1 if s["steer_ev"] > 0 else -1

    st = s["steer"]
    s["cs"] = bool(corner and steer_ok and abs(st) >= p["cs_min_pct"] and (st > 0) == (d < 0))
    drifting = (corner and s["v"] >= p["min_v_mps"]
                and p["min_angle_deg"] <= angle <= p["spin_angle_deg"]
                and (not gate or s["cs"] or angle >= p["angle_no_cs_deg"]))
    s["spin_t"] = s["spin_t"] + dt if (corner and abs(angle) > p["spin_angle_deg"]) else 0.0
    s["full_t"] = s["full_t"] + dt if thr_pct >= p["full_thr_pct"] else 0.0
    s["full_on"] = s["full_t"] >= p["full_thr_s"] - T_EPS

    if s["chain"] and s["spin_t"] >= p["spin_s"] - T_EPS:
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
        if not s["seg_ok"] and s["seg_t"] >= p["min_seg_s"] - T_EPS:
            s["seg_ok"] = True
            s["chain_ok"] = True
            if s["prev_dir"] != 0 and s["seg_dir"] != s["prev_dir"]:
                s["mult"] = min(s["mult"] + 1, int(p["max_mult"]))
                s["n_trans"] += 1
            s["prev_dir"] = s["seg_dir"]
        bonus = p["full_bonus_mult"] if (p["full_bonus"] and s["full_on"]) else 1.0
        s["pts"] += s["v"] * dt * (p["pts_per_m"] + p["pts_per_m_deg"] * angle) * bonus
        s["drift_t"] += dt
        if s["full_on"]:
            s["full_drift_t"] += dt
        s["angle_max"] = max(s["angle_max"], angle)
    else:
        s["in_drift"] = False
        if s["chain"]:
            s["grace_t"] += dt
            if s["grace_t"] >= p["chain_gap_s"] - T_EPS:
                _bank(s)


def race_inputs(race, cfg: dict):
    """Wejścia kroku z próbek wyścigu – jak w firmware: dt, a_fwd, a_left [g], ω_up [dps],
    rest_hint (brak zboczy i gaz w neutrum), steer_raw [% od neutrum, NaN = brak], thr_pct [%]."""
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
    rest = _rest_mask(race)
    steer = _steer_pct(race)
    return dt, af, al, gu, rest, steer, _thr_pct(race)


def drift_score(race, cfg: dict | None = None) -> dict:
    """Przebiegi i podsumowanie punktów za drift dla nagrania wyścigu."""
    cfg = cfg or {}
    p = drift_params(cfg)
    dt, af, al, gu, rest, steer, thr = race_inputs(race, cfg)
    n = len(dt)
    s = drift_init()
    out = {k: np.zeros(n) for k in ("angle", "v", "pts", "mult", "total", "drift", "steer", "cs", "full")}
    for i in range(n):
        drift_step(s, p, float(dt[i]), float(af[i]), float(al[i]), float(gu[i]), bool(rest[i]),
                   float(steer[i]), float(thr[i]))
        out["angle"][i] = s["angle"]
        out["v"][i] = s["v"] if s["corner"] else 0.0
        out["pts"][i] = s["pts"]
        out["mult"][i] = s["mult"]
        out["total"][i] = s["total"]
        out["drift"][i] = 1.0 if s["in_drift"] else 0.0
        out["steer"][i] = s["steer"]
        out["cs"][i] = 1.0 if s["cs"] else 0.0
        out["full"][i] = 1.0 if s["full_on"] else 0.0
    # łańcuch trwający na końcu nagrania – zaliczony
    end = dict(s)
    _bank(end)
    summary = dict(total=end["total"], best_chain=end["best"], chains=end["n_banks"],
                   transitions=s["n_trans"], spins=s["n_lost"], drift_time_s=s["drift_t"],
                   angle_max_deg=s["angle_max"], steer_sign=steer_sign_of(s, p),
                   full_drift_s=s["full_drift_t"])
    out["t"] = race.t_s
    return dict(series=out, summary=summary)


# ------------------------------------------------------------------ kalibracja montażu
def _thr_pct(race) -> np.ndarray:
    """Gaz [% do przodu] jak w firmware; brak sygnału / nienauczone neutrum → 0."""
    h = race.header
    neutral, full = int(h["throttle_neutral_us"]), int(h["throttle_full_us"]) or 500
    sign = 1.0 if h["throttle_fwd_higher"] else -1.0
    return np.array([(int(w) - neutral) * 100.0 / full * sign if (w and neutral) else 0.0
                     for w in race.samples["throttle_us"]], dtype=np.float64)


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


def _steer_pct(race) -> np.ndarray:
    """Skręt [% od neutrum, znak surowy]; NaN = brak sygnału, nienauczone neutrum albo plik v1."""
    h = race.header
    neutral, full = int(h.get("steer_neutral_us", 0)), int(h.get("steer_full_us", 0)) or 500
    out = np.full(len(race.samples), math.nan)
    if not neutral:
        return out
    for i, w in enumerate(race.samples["steer_us"]):
        if w:
            out[i] = (int(w) - neutral) * 100.0 / full
    return out


def mount_calibrate(race, min_yaw_dps: float = 40.0, min_acc_g: float = 0.1, lpf_tau_s: float = 0.15,
                    trim_deg: float = 25.0) -> dict:
    """Orientacja płytki w aucie z nagrania jazdy BEZ driftu (zwykłe zakręty z przyczepnością).

    Oś „up” = oś płytki z największą |średnią a| w postoju (grawitacja).
    W zakręcie przyspieszenie poziome × znak ω_up wskazuje „left” (przy przyczepności
    prędkość jest wzdłuż osi auta) – średnia ważona |ω| po próbkach z jazdą po łuku.
    Wynik: imu_axes (najbliższe ustawienie co 90°) + yaw_offset_deg (reszta, −45…45°)
    oraz steer_sign: przy przyczepności skręt jest w stronę zakrętu (znak skręt·ω > 0 = normalny),
    0 = brak skrętu w nagraniu albo niejednoznaczny.
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
    steer = _steer_pct(race)
    f1 = f2 = fr = 0.0
    pts = []   # (kąt kierunku „left”, waga)
    st_sum = st_abs = 0.0
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
        # skręt ω ≥ min_yaw_dps oznacza jazdę (bez wymogu zboczy – auto może nie mieć halla)
        if abs(fr) >= min_yaw_dps and math.hypot(f1, f2) >= min_acc_g:
            s = 1.0 if fr > 0 else -1.0
            w = abs(fr) * math.hypot(f1, f2)
            pts.append((math.atan2(f2 * s, f1 * s), w))
            if not math.isnan(steer[i]) and abs(steer[i]) >= 8.0:
                st_sum += w * (1.0 if steer[i] * fr > 0 else -1.0)
                st_abs += w
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
    # znak skrętu: wyraźna przewaga (≥ 60 % wagi) jednego znaku
    steer_sign = 0
    if st_abs > 0 and abs(st_sum) >= 0.6 * st_abs:
        steer_sign = 1 if st_sum > 0 else -1
    return dict(imu_axes=dict(forward=spec(q), left=spec(q + 90), up=("+" if us > 0 else "-") + names[up]),
                yaw_offset_deg=yaw, samples=n, spread_deg=spread, steer_sign=steer_sign)
