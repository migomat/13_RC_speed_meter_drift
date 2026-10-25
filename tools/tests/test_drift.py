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
        assert np.allclose(res["series"][k], vals, rtol=1e-9, atol=1e-9), k


@pytest.mark.parametrize("name", DRIFTS)
def test_drift_physics(name):
    data, exp = load(name)
    tr = exp["truth"]
    res = drift_score(parse_race(data), exp["config"])
    sm, sr = res["summary"], res["series"]
    assert sm["transitions"] == tr["transitions"]
    assert sm["chains"] == tr["chains"]
    assert sm["spins"] == tr["spins"]
    t = np.cumsum(np.full(len(sr["angle"]), 0.05))
    for t0, t1, th in tr["angles"]:
        m = (t >= t0) & (t <= t1)
        assert abs(np.mean(sr["angle"][m]) - th) < 4.0, (t0, np.mean(sr["angle"][m]), th)
        assert np.all(sr["drift"][m] == 1.0)
    assert max(sr["mult"]) == 1 + tr["transitions"]


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
    assert np.array_equal(rows[:, 3], np.array(sr["mult"]))
    assert math.isclose(rows[-1, 4], sr["total"][-1], rel_tol=0.01)
    corner = np.array(sr["v"]) > 0
    assert np.max(np.abs(rows[corner, 0] - np.array(sr["angle"])[corner])) < 0.5


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
