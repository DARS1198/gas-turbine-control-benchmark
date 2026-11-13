"""The turbine model: steady state, physics sanity, and numerical accuracy."""

import numpy as np
import pytest
from scipy.integrate import solve_ivp

from gtbench.plant import GasTurbine, TurbineParams, exhaust_temp_fn, torque_fn


@pytest.mark.parametrize("load", [0.0, 0.3, 0.7, 1.0])
def test_steady_state_holds(load):
    p = TurbineParams()
    gt = GasTurbine(p)
    gt.reset(load)
    vce = p.steady_state_vce(load)
    for _ in range(3000):
        gt.step(vce, load)
    assert gt.n == pytest.approx(1.0, abs=1e-9)
    assert gt.torque == pytest.approx(load, abs=1e-9)


def test_published_operating_points():
    p = TurbineParams()
    assert torque_fn(p, 0.23, 1.0) == pytest.approx(0.0)  # no-load fuel gives zero torque
    assert torque_fn(p, 1.0, 1.0) == pytest.approx(1.0, abs=2e-3)  # rated fuel, rated torque
    assert exhaust_temp_fn(p, 1.0, 1.0) == pytest.approx(950.0)  # rated exhaust temperature


def test_more_fuel_accelerates_and_more_load_decelerates():
    p = TurbineParams()
    for dv, dl, sign in [(0.1, 0.0, 1), (0.0, 0.1, -1)]:
        gt = GasTurbine(p)
        gt.reset(0.5)
        for _ in range(500):
            gt.step(p.steady_state_vce(0.5) + dv, 0.5 + dl)
        assert np.sign(gt.n - 1.0) == sign


def test_fuel_command_is_limited():
    p = TurbineParams()
    gt = GasTurbine(p)
    gt.reset(0.5)
    for _ in range(2000):
        gt.step(99.0, 0.5)
    assert gt.vp <= ((1 - p.w_min) * p.vce_max + p.w_min) * gt.n + 1e-9


def _open_loop(dt, p, t_end=10.0):
    gt = GasTurbine(p, dt)
    gt.reset(0.5)
    vce = p.steady_state_vce(0.8)
    for _ in range(round(t_end / dt)):
        gt.step(vce, 0.8)
    return gt.n


def test_matches_adaptive_ode_solver():
    """Fixed-step scheme versus SciPy's adaptive solver (transport delays set to zero)."""
    p = TurbineParams(e_cr=0.0, e_td=0.0)
    vce, load = p.steady_state_vce(0.8), 0.6

    def rhs(_, x):
        vp, wf2, wf, n = x
        wd = ((1 - p.w_min) * vce + p.w_min) * n
        return [
            (p.a * wd - p.c * vp) / p.b,
            (vp - wf2) / p.t_f,
            (wf2 - wf) / p.t_cd,
            (torque_fn(p, wf, n) - load) / p.t_i,
        ]

    wf0 = p.w_min + 0.5 / p.torque_gain
    sol = solve_ivp(rhs, (0, 10), [wf0, wf0, wf0, 1.0], rtol=1e-10, atol=1e-12)
    gt = GasTurbine(p, 0.01)
    gt.reset(0.5)
    for _ in range(1000):
        gt.step(vce, load)
    assert gt.n == pytest.approx(sol.y[3, -1], abs=2e-4)  # within 0.02 % of rated speed


def test_step_size_convergence():
    """Dividing the step by 10 changes the answer by less than 0.01 % of rated speed."""
    p = TurbineParams()
    assert _open_loop(0.01, p) == pytest.approx(_open_loop(0.001, p), abs=1e-4)
