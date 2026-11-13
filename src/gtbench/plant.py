"""Simplified heavy-duty gas turbine model (Rowen, 1983).

All quantities are in per unit (pu) of rated value unless stated otherwise.
Speed N = 1.0 means rated speed (3600 rpm / 60 Hz on a 60 Hz machine).

Signal path
-----------
    VCE --> fuel demand --> valve positioner --> fuel system --> combustor delay
        --> compressor discharge lag --> torque function --> rotor inertia --> N

The integration scheme is deliberately simple so that it can be ported line by
line to JavaScript (see ``docs/sim.js``): first-order lags use the exact
zero-order-hold update driven by the mid-step value of their input, the rotor
uses forward Euler, and transport delays are ring buffers.
``tests/test_plant.py`` checks the scheme against an adaptive ODE solver.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, replace


@dataclass(frozen=True)
class TurbineParams:
    """Model constants. Defaults are the values published for Rowen's model."""

    a: float = 1.0  # valve positioner gain
    b: float = 0.05  # valve positioner time constant [s]
    c: float = 1.0  # valve positioner feedback
    w_min: float = 0.23  # fuel flow needed to hold rated speed at no load [pu]
    t_f: float = 0.4  # fuel system time constant [s]
    e_cr: float = 0.01  # combustion reaction delay [s]
    e_td: float = 0.04  # turbine and exhaust transport delay [s]
    t_cd: float = 0.2  # compressor discharge volume time constant [s]
    t_i: float = 15.64  # rotor time constant (= 2H) [s]
    t_r: float = 950.0  # rated exhaust temperature [deg F]
    torque_gain: float = 1.3  # d(torque)/d(fuel)
    torque_speed: float = 0.5  # torque sensitivity to speed deviation
    vce_min: float = -0.1  # fuel command limits [pu]
    vce_max: float = 1.5
    k_rs: float = 0.8  # radiation shield: direct fraction
    t_rs: float = 15.0  # radiation shield time constant [s]
    t_tc: float = 2.5  # thermocouple time constant [s]
    extra_delay: float = 0.0  # additional fuel-path dead time, robustness tests only [s]

    def steady_state_vce(self, load: float) -> float:
        """Fuel command that balances ``load`` torque at rated speed."""
        wf = self.w_min + load / self.torque_gain
        return (wf - self.w_min) / (1.0 - self.w_min)

    def perturbed(self, **changes: float) -> TurbineParams:
        return replace(self, **changes)


def torque_fn(p: TurbineParams, wf: float, n: float) -> float:
    """Rowen's f2: turbine torque from fuel flow and speed."""
    return p.torque_gain * (wf - p.w_min) + p.torque_speed * (1.0 - n)


def exhaust_temp_fn(p: TurbineParams, wf: float, n: float) -> float:
    """Rowen's f1: exhaust temperature [deg F] from fuel flow and speed."""
    return p.t_r - 700.0 * (1.0 - wf) + 550.0 * (1.0 - n)


class GasTurbine:
    """Discrete-time simulation of the turbine at a fixed step ``dt``."""

    def __init__(self, params: TurbineParams | None = None, dt: float = 0.01):
        self.p = params or TurbineParams()
        self.dt = dt
        p = self.p
        self._k_vp = 1.0 - math.exp(-dt * p.c / p.b)
        self._k_f = 1.0 - math.exp(-dt / p.t_f)
        self._k_cd = 1.0 - math.exp(-dt / p.t_cd)
        self._k_rs = 1.0 - math.exp(-dt / p.t_rs)
        self._k_tc = 1.0 - math.exp(-dt / p.t_tc)
        self._n_cr = max(0, round((p.e_cr + p.extra_delay) / dt))
        self._n_td = max(0, round(p.e_td / dt))
        self.reset(0.5)

    def reset(self, load: float) -> None:
        """Place the turbine in steady state at rated speed carrying ``load``."""
        p = self.p
        wf = p.w_min + load / p.torque_gain
        self.n = 1.0
        self.vp = wf * p.a / p.c
        self.wf2 = wf
        self.wf = wf
        self._buf_cr = deque([wf] * (self._n_cr + 1), maxlen=self._n_cr + 1)
        self._buf_td = deque([wf] * (self._n_td + 1), maxlen=self._n_td + 1)
        tx = exhaust_temp_fn(p, wf, 1.0)
        self.tx = tx
        self._x_rs = tx
        self.tx_meas = tx
        self.torque = load

    def step(self, vce: float, load: float) -> None:
        """Advance one ``dt`` with fuel command ``vce`` and load torque ``load``."""
        p = self.p
        vce = min(p.vce_max, max(p.vce_min, vce))
        wd = ((1.0 - p.w_min) * vce + p.w_min) * self.n
        vp_old = self.vp
        self.vp += self._k_vp * (wd * p.a / p.c - self.vp)
        self.wf2 += self._k_f * (0.5 * (vp_old + self.vp) - self.wf2)
        comb_old = self._buf_cr[0]
        self._buf_cr.append(self.wf2)
        wf_comb = self._buf_cr[0]
        wf_old = self.wf
        self.wf += self._k_cd * (0.5 * (comb_old + wf_comb) - self.wf)
        self._buf_td.append(wf_comb)
        self.torque = torque_fn(p, 0.5 * (wf_old + self.wf), self.n)
        self.tx = exhaust_temp_fn(p, self._buf_td[0], self.n)
        self._x_rs += self._k_rs * (self.tx - self._x_rs)
        shield = p.k_rs * self.tx + (1.0 - p.k_rs) * self._x_rs
        self.tx_meas += self._k_tc * (shield - self.tx_meas)
        self.n += self.dt * (self.torque - load) / p.t_i
