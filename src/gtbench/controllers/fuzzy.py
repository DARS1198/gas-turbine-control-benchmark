"""Fuzzy PD + I controller (Sugeno type), written from scratch.

    u = Ku * F(Ke * e, Kde * de/dt)  +  Ki * integral(e)

``F`` is a fuzzy rule base with the speed error ``e`` and its rate ``de`` as
inputs (both scaled to [-1, 1]). It replaces the proportional and derivative
terms of a PID; the integral term is kept linear so that the speed always
returns exactly to the reference.

Five triangular membership functions per input (NB, NS, Z, PS, PB) and a
5 x 5 rule table. The rule table is the classic "diagonal" one: the further
and faster the speed is moving away from the reference, the harder the fuel
command is pushed. ``warp`` bends the membership-function centres: below 1 the
controller reacts strongly to small errors and saturates early, above 1 it is
gentle near the reference and aggressive for large upsets. With ``warp = 1``
and unsaturated inputs the controller is exactly a linear PID.
"""

from __future__ import annotations

import math

from .base import Controller

_BASE = (-1.0, -0.5, 0.0, 0.5, 1.0)


def _warp(c: float, p: float) -> float:
    return math.copysign(abs(c) ** p, c)


def memberships(x: float, centres: tuple[float, ...]) -> list[float]:
    """Triangular membership degrees of ``x``; they always sum to 1."""
    n = len(centres)
    mu = [0.0] * n
    if x <= centres[0]:
        mu[0] = 1.0
        return mu
    if x >= centres[-1]:
        mu[-1] = 1.0
        return mu
    for i in range(n - 1):
        lo, hi = centres[i], centres[i + 1]
        if lo <= x <= hi:
            w = (x - lo) / (hi - lo)
            mu[i] = 1.0 - w
            mu[i + 1] = w
            break
    return mu


class FuzzyPID(Controller):
    name = "Fuzzy"

    def __init__(
        self,
        ts: float,
        ke: float,
        kde: float,
        ku: float,
        ki: float,
        warp: float = 1.0,
        tf: float = 0.1,
        **kw,
    ):
        super().__init__(ts, **kw)
        self.ke, self.kde, self.ku, self.ki = ke, kde, ku, ki
        self.centres = tuple(_warp(c, warp) for c in _BASE)
        self.out = _BASE
        self._kf = 1.0 - math.exp(-ts / tf)
        self.reset(0.0)

    def reset(self, u0: float) -> None:
        self._i = u0
        self._meas_prev = None
        self._de = 0.0

    def surface(self, e_n: float, de_n: float) -> float:
        """Normalised controller output for normalised inputs (the 'control surface')."""
        mu_e = memberships(e_n, self.centres)
        mu_d = memberships(de_n, self.centres)
        out = 0.0
        for i, a in enumerate(mu_e):
            if a == 0.0:
                continue
            for j, b in enumerate(mu_d):
                if b == 0.0:
                    continue
                k = min(4, max(0, i + j - 2))
                out += a * b * self.out[k]
        return out

    def update(self, ref: float, meas: float) -> float:
        e = ref - meas
        if self._meas_prev is None:
            self._meas_prev = meas
        rate = -(meas - self._meas_prev) / self.ts
        self._meas_prev = meas
        self._de += self._kf * (rate - self._de)
        e_n = min(1.0, max(-1.0, self.ke * e))
        de_n = min(1.0, max(-1.0, self.kde * self._de))
        u_raw = self.ku * self.surface(e_n, de_n) + self._i
        u = self._clip(u_raw)
        saturated_same_way = (u_raw > self.u_max and e > 0) or (u_raw < self.u_min and e < 0)
        if not saturated_same_way:
            self._i += self.ki * e * self.ts
        return u
