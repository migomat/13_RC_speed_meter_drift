"""Punkty za drift: drift.py (referencja) na wektorach, sensowność fizyczna, drift.c (float32) i kalibracja montażu."""
import json
import math
import shutil
import subprocess
from pathlib import Path

import numpy as np
import pytest

from rc_launch.analyze import DEFAULT_CONFIG
from rc_launch.drift import drift_score, mount_calibrate
from rc_launch.parse import Race, parse_race

ROOT = Path(__file__).resolve().parents[2]
VEC = ROOT / "test_vectors"
DRIFTS = json.loads((VEC / "drift.json").read_text())


def load(name):
    return (VEC / f"{name}.bin").read_bytes(), json.loads((VEC / f"{name}.expected.json").read_text())


@pytest.mark.parametrize("name", DRIFTS)
def test_drift_matches_expected(name):
    data, exp = load(name)
    res = drift_score(parse_race(data), exp["config"])
    for k, v in exp["summary"].items():
        assert math.isclose(res["summary"][k], v, rel_tol=1e-9, abs_tol=1e-9), k
    for k, vals in exp["series"].items():
        v = np.array([np.nan if x is None else x for x in vals])
        assert np.allclose(res["series"][k], v, rtol=1e-9, atol=1e-9, equal_nan=True), k


@pytest.mark.parametrize("name", DRIFTS)
def test_drift_physics(name):
    data, exp = load(name)
    tr = exp["truth"]
    res = drift_score(parse_race(data), exp["config"])
    sm, sr = res["summary"], res["series"]
    assert sm["transitions"] == tr["transitions"]
    assert sm["chains"] == tr["chains"]
    assert sm["spins"] == tr["spins"]
    assert sm["steer_sign"] == tr["steer_sign"]          # znak skrętu nauczony automatycznie
    t = np.cumsum(np.full(len(sr["angle"]), 0.05))
    for t0, t1, th in tr["angles"]:
        m = (t >= t0) & (t <= t1)
        assert abs(np.mean(sr["angle"][m]) - th) < 4.0, (t0, np.mean(sr["angle"][m]), th)
        assert np.all(sr["drift"][m] == 1.0)
        assert np.all(sr["cs"][m] == 1.0)                  # kontra w ustalonym drifcie
    assert max(sr["mult"]) == 1 + tr["transitions"]


def without_steering(race: Race) -> Race:
    smp = race.samples.copy()
    smp["steer_us"] = 0
    return Race(header=race.header, samples=smp, n_samples=race.n_samples,
                end_reason=race.end_reason, crc32=race.crc32)


def test_drift_without_steering_still_scores():
    # skręt niepodłączony – sam kąt z IMU (jak w fw 1.2)
    race = without_steering(parse_race((VEC / "drift_eights.bin").read_bytes()))
    sm = drift_score(race, DEFAULT_CONFIG)["summary"]
    assert (sm["transitions"], sm["chains"], sm["spins"], sm["steer_sign"]) == (2, 1, 1, 0)


@pytest.mark.parametrize("name,wrong", [("drift_eights", -1), ("drift_eights_rev", 1)])
def test_wrong_steer_sign_blocks_drift(name, wrong):
    # zły znak skrętu = kontra wygląda jak skręt w zakręt → drift poniżej angle_no_cs nie liczy się;
    # dlatego znak z kalibracji / nauki, a ustawienie ręczne tylko świadomie
    cfg = json.loads(json.dumps(DEFAULT_CONFIG))
    ok = drift_score(parse_race((VEC / f"{name}.bin").read_bytes()), cfg)["summary"]["total"]
    cfg["drift"]["steer_sign"] = wrong
    bad = drift_score(parse_race((VEC / f"{name}.bin").read_bytes()), cfg)["summary"]["total"]
    assert bad < 0.2 * ok


@pytest.mark.parametrize("name", ["race_oval", "race_recovered"])
def test_grip_driving_scores_nothing(name):
    # zakręty z przyczepnością (hamowanie przed, gaz na wyjściu) – bez punktów
    res = drift_score(parse_race((VEC / f"{name}.bin").read_bytes()), DEFAULT_CONFIG)
    assert res["summary"]["total"] == 0.0
    assert res["summary"]["drift_time_s"] < 1.0


@pytest.fixture(scope="module")
def drift_bin(tmp_path_factory):
    cc = shutil.which("gcc") or shutil.which("cc")
    if not cc:
        pytest.skip("brak kompilatora C")
    out = tmp_path_factory.mktemp("drift") / "drift_host"
    subprocess.check_call([cc, "-O2", "-I", str(ROOT / "firmware/src"),
                           str(ROOT / "tools/drift_host/drift_host.c"),
                           str(ROOT / "firmware/src/drift.c"), "-lm", "-o", str(out)])
    return out


@pytest.mark.parametrize("name", DRIFTS)
def test_c_drift_matches_python(drift_bin, name):
    _, exp = load(name)
    out = subprocess.check_output([str(drift_bin), str(VEC / f"{name}.bin")], text=True)
    rows = np.array([[float(x) for x in ln.split()] for ln in out.splitlines()])
    sr = exp["series"]
    assert len(rows) == len(sr["angle"])
    # float32 vs float64: progi mogą przeskoczyć o próbkę, wynik w granicach 1 %
    assert np.sum(rows[:, 5] != np.array(sr["drift"])) <= 2
    assert np.sum(rows[:, 6] != np.array(sr["cs"])) <= 2
    assert np.array_equal(rows[:, 3], np.array(sr["mult"]))
    assert math.isclose(rows[-1, 4], sr["total"][-1], rel_tol=0.01)
    corner = np.array(sr["v"]) > 0
    assert np.max(np.abs(rows[corner, 0] - np.array(sr["angle"])[corner])) < 0.05


# ------------------------------------------------------------------ kalibracja montażu
def rotated(race: Race, R: np.ndarray) -> Race:
    """Ta sama jazda z płytką obróconą: wektor_płytki = R · wektor_auta."""
    smp = race.samples.copy()
    for keys in (("ax", "ay", "az"), ("gx", "gy", "gz")):
        v = np.stack([race.samples[k].astype(np.float64) for k in keys])
        w = R @ v
        for i, k in enumerate(keys):
            smp[k] = np.round(w[i]).astype(np.int16)
    return Race(header=race.header, samples=smp, n_samples=race.n_samples,
                end_reason=race.end_reason, crc32=race.crc32)


def rot_z(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def test_mount_calibrate_default():
    cal = mount_calibrate(parse_race((VEC / "race_oval.bin").read_bytes()))
    assert cal["imu_axes"] == dict(forward="+x", left="+y", up="+z")
    assert abs(cal["yaw_offset_deg"]) < 3.0


@pytest.mark.parametrize("deg,axes,yaw", [
    (110.0, dict(forward="+y", left="-x", up="+z"), 20.0),
    (-160.0, dict(forward="-x", left="-y", up="+z"), 20.0),
    (30.0, dict(forward="+x", left="+y", up="+z"), 30.0),
])
def test_mount_calibrate_rotated(deg, axes, yaw):
    race = rotated(parse_race((VEC / "race_oval.bin").read_bytes()), rot_z(deg))
    cal = mount_calibrate(race)
    assert cal["imu_axes"] == axes
    assert abs(cal["yaw_offset_deg"] - yaw) < 3.0


def test_mount_calibrate_upside_down():
    flip = np.diag([1.0, -1.0, -1.0])          # obrót o 180° wokół x
    race = rotated(parse_race((VEC / "race_oval.bin").read_bytes()), flip)
    cal = mount_calibrate(race)
    assert cal["imu_axes"] == dict(forward="+x", left="-y", up="-z")
    assert abs(cal["yaw_offset_deg"]) < 3.0


def test_calibrated_mount_restores_score():
    """Drift nagrany płytką obróconą o 110° liczy się po kalibracji tak jak przy montażu domyślnym."""
    base = parse_race((VEC / "drift_eights.bin").read_bytes())
    ref = drift_score(base, DEFAULT_CONFIG)["summary"]
    rot = rotated(base, rot_z(110.0))
    cfg = json.loads(json.dumps(DEFAULT_CONFIG))
    cfg["imu_axes"] = dict(forward="+y", left="-x", up="+z")
    cfg["estimator"]["yaw_offset_deg"] = 20.0
    got = drift_score(rot, cfg)["summary"]
    assert got["transitions"] == ref["transitions"] and got["spins"] == ref["spins"]
    assert math.isclose(got["total"], ref["total"], rel_tol=0.02)


def grip_part(race: Race, t_end: float) -> Race:
    n = int(np.searchsorted(np.cumsum(race.samples["n_ticks"]) / 500.0, t_end))
    return Race(header=race.header, samples=race.samples[:n].copy(), n_samples=n,
                end_reason=race.end_reason, crc32=race.crc32)


@pytest.mark.parametrize("name,sign", [("drift_eights", 1), ("drift_eights_rev", -1)])
def test_mount_calibrate_steer_sign(name, sign):
    # kalibracja z jazdy bez driftu (część nagrania przed driftem: postój, łuk z przyczepnością)
    cal = mount_calibrate(grip_part(parse_race((VEC / f"{name}.bin").read_bytes()), 10.5))
    assert cal["imu_axes"] == dict(forward="+x", left="+y", up="+z")
    assert cal["steer_sign"] == sign


def test_v1_race_has_no_steering():
    r = parse_race((VEC / "race_oval.bin").read_bytes())
    assert r.header["format_version"] == 1 and r.header["steer_neutral_us"] == 0
    assert np.all(r.samples["steer_us"] == 0)


def test_full_throttle_bonus():
    race = parse_race((VEC / "drift_eights.bin").read_bytes())
    tr = json.loads((VEC / "drift_eights.expected.json").read_text())["truth"]
    off = drift_score(race, DEFAULT_CONFIG)["summary"]
    cfg = json.loads(json.dumps(DEFAULT_CONFIG))
    cfg["drift"]["full_bonus"] = 1
    on = drift_score(race, cfg)["summary"]
    assert abs(off["full_drift_s"] - tr["full_drift_s"]) < 0.4
    assert on["total"] > 1.05 * off["total"]
    assert (on["chains"], on["transitions"], on["spins"]) == (off["chains"], off["transitions"], off["spins"])


def test_c_full_bonus_matches_python(drift_bin):
    cfg = json.loads(json.dumps(DEFAULT_CONFIG))
    cfg["drift"]["full_bonus"] = 1
    py = drift_score(parse_race((VEC / "drift_eights.bin").read_bytes()), cfg)["series"]
    out = subprocess.check_output([str(drift_bin), str(VEC / "drift_eights.bin"), "+x", "+y", "+z", "0", "0", "1"],
                                  text=True)
    rows = np.array([[float(x) for x in ln.split()] for ln in out.splitlines()])
    assert math.isclose(rows[-1, 4], py["total"][-1], rel_tol=0.01)


def test_auto_yaw_trim_converges():
    """Płytka obrócona o 12° względem ustawionego montażu: przyspieszanie / hamowanie na wprost → trim ≈ 12°."""
    from rc_launch.drift import drift_init, drift_params, drift_step
    p = drift_params(DEFAULT_CONFIG)
    s = drift_init()
    d = math.radians(12.0)
    for _ in range(60):                                   # postój – bias
        drift_step(s, p, 0.05, 0.0, 0.0, 0.0, True, 0.0, 0.0)
    for k in range(400):                                  # 20 s: 1 s gazu, 1 s hamowania
        a = 0.2 if (k // 20) % 2 == 0 else -0.25
        drift_step(s, p, 0.05, a * math.cos(d), a * math.sin(d), 0.0, False, 0.0, 50.0)
    assert abs(s["trim"] - 12.0) < 1.0, s["trim"]
    for k in range(200):                                  # kręcenie w kółko nie rusza korekty
        drift_step(s, p, 0.05, 0.1, 0.4, 120.0, False, -20.0, 50.0)
    assert abs(s["trim"] - 12.0) < 1.0
