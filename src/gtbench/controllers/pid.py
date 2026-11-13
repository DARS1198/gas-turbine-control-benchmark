"""PID speed controller with filtered derivative and anti-windup.

u = Kp*e + I + D
I += Ki*e*Ts            (frozen while the output is saturated in the same direction)
D  = -Kd * d(meas)/dt   (first-order filtered, acts on the measurement to avoid setpoint kick)
"""

from __future__ import annotations

import math

from .base import Controller


class PID(Controller):
    name = "PID"

    def __init__(self, ts: float, kp: float, ki: float, kd: float, tf: float = 0.1, **kw):
        super().__init__(ts, **kw)
        self.kp, self.ki, self.kd = kp, ki, kd
        self._kf = 1.0 - math.exp(-ts / tf)
        self.reset(0.0)

    def reset(self, u0: float) -> None:
        self._i = u0
        self._meas_prev = None
        self._d = 0.0

    def update(self, ref: float, meas: float) -> float:
        e = ref - meas
        if self._meas_prev is None:
            self._meas_prev = meas
        rate = (meas - self._meas_prev) / self.ts
        self._meas_prev = meas
        self._d += self._kf * (-self.kd * rate - self._d)
        u_raw = self.kp * e + self._i + self._d
        u = self._clip(u_raw)
        saturated_same_way = (u_raw > self.u_max and e > 0) or (u_raw < self.u_min and e < 0)
        if not saturated_same_way:
            self._i += self.ki * e * self.ts
        return u
