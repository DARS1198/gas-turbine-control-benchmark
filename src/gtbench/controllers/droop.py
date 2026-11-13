"""Droop governor: the speed governor published with Rowen's model.

Transfer function  W / (Y s + Z)  with W = 25 (4 % droop), Y = 0.05 s, Z = 1.
It is proportional only, so after a load change the speed does not return to
the reference. It is the untuned baseline of the benchmark.
"""

from __future__ import annotations

import math

from .base import Controller


class DroopGovernor(Controller):
    name = "Droop"

    def __init__(self, ts: float, w: float = 25.0, y: float = 0.05, z: float = 1.0, **kw):
        super().__init__(ts, **kw)
        self.gain = w / z
        self._k = 1.0 - math.exp(-ts * z / y)
        self.reset(0.0)

    def reset(self, u0: float) -> None:
        self._load_ref = u0
        self._x = 0.0

    def update(self, ref: float, meas: float) -> float:
        self._x += self._k * (self.gain * (ref - meas) - self._x)
        return self._clip(self._load_ref + self._x)
