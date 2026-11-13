"""Each controller: the property that defines it."""

from itertools import pairwise

import numpy as np
import pytest
from scipy.optimize import minimize

from gtbench.controllers import MPC, PID, DroopGovernor, FuzzyPID
from gtbench.controllers.fuzzy import memberships
from gtbench.controllers.mpc import box_qp, linear_model
from gtbench.metrics import compute
from gtbench.plant import TurbineParams
from gtbench.scenarios import BY_KEY, Scenario
from gtbench.simulate import TS, simulate
from gtbench.tuning import ORDER, build

QUIET = Scenario("quiet", "", "", 60.0, ((0.0, 0.5), (5.0, 0.8)), noise_std=0.0)


def test_droop_steady_state_error_matches_theory():
    """4 % droop: a load change of dL pu moves the speed by about dL / 25."""
    r = simulate(DroopGovernor(TS), QUIET)
    p = TurbineParams()
    d_vce = p.steady_state_vce(0.8) - p.steady_state_vce(0.5)
    # Speed sag reduces fuel demand slightly (fuel is proportional to speed) and raises
    # torque (0.5 * (1 - N)); the closed-form balance is solved here.
    wf0 = p.w_min + 0.8 / p.torque_gain
    k = 25 * (1 - p.w_min) * p.torque_gain + p.torque_speed - p.torque_gain * wf0
    expected = (1 - p.w_min) * p.torque_gain * d_vce / k
    assert 1.0 - r.speed[-1] == pytest.approx(expected, rel=0.03)


@pytest.mark.parametrize("name", ["PID", "Fuzzy", "MPC"])
def test_integral_action_removes_steady_state_error(name):
    r = simulate(build(name), QUIET)
    assert abs(r.speed[-1] - 1.0) < 1e-5


@pytest.mark.parametrize("name", ORDER)
def test_bumpless_start(name):
    """Starting at steady state with no disturbance, nothing should move."""
    flat = Scenario("flat", "", "", 5.0, ((0.0, 0.6),), noise_std=0.0)
    r = simulate(build(name), flat)
    assert np.abs(r.speed - 1.0).max() < 1e-6
    assert np.ptp(r.vce) < 1e-4


def test_pid_anti_windup_recovers_quickly():
    """After a long saturation the integrator must not be wound up."""
    pid = PID(TS, kp=20, ki=10, kd=0)
    pid.reset(0.5)
    for _ in range(2000):
        assert pid.update(1.0, 0.5) == pid.u_max  # huge error: saturated
    assert pid._i < pid.u_max + 1.0
    assert pid.update(1.0, 1.2) < pid.u_max  # error reverses: output leaves the limit at once


def test_fuzzy_memberships_sum_to_one():
    centres = (-1.0, -0.4, 0.0, 0.4, 1.0)
    for x in np.linspace(-1.5, 1.5, 61):
        mu = memberships(float(x), centres)
        assert sum(mu) == pytest.approx(1.0)
        assert min(mu) >= 0.0


def test_fuzzy_surface_is_odd_and_monotonic():
    f = FuzzyPID(TS, ke=1, kde=1, ku=1, ki=0, warp=0.6)
    grid = np.linspace(-1, 1, 21)
    for e in grid:
        for d in grid:
            assert f.surface(e, d) == pytest.approx(-f.surface(-e, -d), abs=1e-12)
    row = [f.surface(e, 0.0) for e in grid]
    assert all(b >= a - 1e-12 for a, b in pairwise(row))


def test_fuzzy_with_unit_warp_is_linear():
    f = FuzzyPID(TS, ke=1, kde=1, ku=1, ki=0, warp=1.0)
    for e, d in [(0.1, 0.2), (-0.3, 0.05), (0.25, -0.4)]:
        assert f.surface(e, d) == pytest.approx(e + d)


def test_box_qp_matches_scipy():
    rng = np.random.default_rng(0)
    for _ in range(20):
        m = rng.normal(size=(10, 10))
        h = m @ m.T + 0.5 * np.eye(10)
        f = rng.normal(size=10) * 3
        x = box_qp(h, f, -0.5, 0.5, np.zeros(10), sweeps=2000)
        ref = minimize(
            lambda v, h=h, f=f: 0.5 * v @ h @ v + f @ v,
            np.zeros(10),
            jac=lambda v, h=h, f=f: h @ v + f,
            bounds=[(-0.5, 0.5)] * 10,
            method="L-BFGS-B",
            options={"ftol": 1e-15, "gtol": 1e-12},
        )
        assert 0.5 * x @ h @ x + f @ x == pytest.approx(ref.fun, abs=1e-6)


def test_linear_model_matches_nonlinear_plant_for_small_steps():
    p = TurbineParams(e_cr=0.0, e_td=0.0)
    a, b, _, c = linear_model(p, TS, load0=0.7)
    x = np.zeros(4)
    du = 0.01
    for _ in range(250):
        x = a @ x + b[:, 0] * du
    from gtbench.plant import GasTurbine

    gt = GasTurbine(p)
    gt.reset(0.7)
    for _ in range(1000):
        gt.step(p.steady_state_vce(0.7) + du, 0.7)
    assert (c @ x).item() == pytest.approx(gt.n - 1.0, rel=0.02)


def test_mpc_respects_fuel_limits_on_full_load_rejection():
    r = simulate(build("MPC"), BY_KEY["rejection"])
    assert r.vce.min() >= -0.1 - 1e-12
    assert r.vce.max() <= 1.5 + 1e-12
    assert compute(r).max_speed_pct < 110.0


def test_mpc_estimates_the_unmeasured_load():
    mpc = build("MPC")
    simulate(mpc, QUIET)
    assert mpc.x[4] == pytest.approx(0.8 - 0.7, abs=0.01)  # deviation from linearisation load
    assert isinstance(mpc, MPC)
