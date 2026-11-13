"""The benchmark itself: metrics, reproducibility, and the headline claims."""

import math

import numpy as np
import pytest

from gtbench.benchmark import passed, sample_plants
from gtbench.metrics import compute
from gtbench.scenarios import BY_KEY, TEST, TUNING, Scenario
from gtbench.simulate import TS, SimResult, simulate
from gtbench.tuning import CORNERS, ORDER, SEED, build, load_tuned


def _fake(speed, vce=None, t_event=1.0):
    n = len(speed)
    t = np.arange(n) * TS
    sc = Scenario("fake", "", "", n * TS, ((0.0, 0.5), (t_event, 0.6)))
    z = np.zeros(n)
    return SimResult(sc, "x", t, np.asarray(speed), z if vce is None else vce, z, z, z + 900, 0.0)


def test_metrics_on_a_known_signal():
    t = np.arange(1000) * TS
    speed = 1.0 - 0.02 * np.exp(-(t - 1.0)) * (t >= 1.0)  # 2 % dip decaying with tau = 1 s
    m = compute(_fake(speed))
    assert m.peak_dev_pct == pytest.approx(2.0, abs=0.01)
    assert m.iae_pct_s == pytest.approx(2.0, rel=0.03)  # integral of 2*exp(-t) = 2 %*s
    assert m.settling_s == pytest.approx(math.log(20), abs=0.05)  # 2 % -> 0.1 % takes ln(20) s
    assert m.ss_error_pct < 1e-6
    assert m.stable


def test_metrics_flag_offset_and_oscillation():
    assert math.isnan(compute(_fake(np.full(1000, 0.99))).settling_s)
    t = np.arange(1000) * TS
    assert not compute(_fake(1.0 + 0.01 * np.sin(2 * t))).stable
    assert not compute(_fake(np.full(1000, 1.11))).stable  # above the overspeed trip


def test_fuel_travel_is_total_variation():
    vce = np.array([0.5, 0.7, 0.6, 0.6, 0.9])
    assert compute(_fake(np.ones(5), vce)).fuel_travel_pu == pytest.approx(0.6)


def test_simulation_is_reproducible():
    a = simulate(build("PID"), BY_KEY["pickup"], seed=7)
    b = simulate(build("PID"), BY_KEY["pickup"], seed=7)
    assert np.array_equal(a.speed, b.speed)


def test_tuning_and_test_scenarios_do_not_overlap():
    assert not {s.load_steps for s in TUNING} & {s.load_steps for s in TEST}


def test_corner_turbines_are_not_in_the_random_test_sample():
    assert not set(CORNERS) & set(sample_plants(50))


@pytest.mark.parametrize("sc", TEST, ids=lambda s: s.key)
@pytest.mark.parametrize("name", ["PID", "Fuzzy", "MPC"])
def test_final_design_passes_every_test_scenario(name, sc):
    m = compute(simulate(build(name), sc, seed=SEED))
    assert m.stable
    assert m.max_speed_pct < 110.0
    assert m.ss_error_pct < 0.05


def test_headline_claims():
    """Guards the statements made in the README against silent regressions."""
    iae = {n: compute(simulate(build(n), BY_KEY["pickup"], seed=SEED)).iae_pct_s for n in ORDER}
    assert iae["MPC"] < iae["PID"] < iae["Droop"]
    assert iae["Droop"] > 5 * iae["PID"]
    tuned = load_tuned()
    for name in ("PID", "Fuzzy", "MPC"):  # robustness costs nominal performance
        assert tuned["robust"][name]["nominal_cost"] > tuned["nominal"][name]["nominal_cost"]


def test_robust_tuning_survives_a_sample_of_perturbed_turbines():
    for i, plant in enumerate(sample_plants(25)):
        for name in ("PID", "Fuzzy", "MPC"):
            assert passed(compute(simulate(build(name), BY_KEY["pickup"], plant, seed=i)))


def test_nominal_tuning_is_fragile():
    """Iteration 1 finding: tuning on the nominal turbine alone leaves unstable cases."""
    unstable = 0
    for i, plant in enumerate(sample_plants(60)):
        m = compute(simulate(build("PID", mode="nominal"), BY_KEY["pickup"], plant, seed=i))
        unstable += not m.stable
    assert unstable > 0
