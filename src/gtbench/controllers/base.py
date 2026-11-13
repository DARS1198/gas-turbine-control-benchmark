"""Common controller interface.

Every controller sees exactly the same information: the speed reference and
the measured speed, once per control frame. Nothing else (no load measurement,
no fuel flow), so the comparison is like for like.
"""

from __future__ import annotations

from abc import ABC, abstractmethod


class Controller(ABC):
    name: str = "controller"

    def __init__(self, ts: float, u_min: float = -0.1, u_max: float = 1.5):
        self.ts = ts
        self.u_min = u_min
        self.u_max = u_max

    @abstractmethod
    def reset(self, u0: float) -> None:
        """Initialise for bumpless start at steady state with output ``u0``."""

    @abstractmethod
    def update(self, ref: float, meas: float) -> float:
        """Return the fuel command for this control frame."""

    def _clip(self, u: float) -> float:
        return min(self.u_max, max(self.u_min, u))
