"""Test scenarios: what happens to the electrical load, and when.

The controllers are *tuned* on ``TUNING`` and *scored* on ``TEST``. The two
sets do not share any load profile, so the scores are not the result of
tuning to the test.
"""

from __future__ import annotations

from dataclasses import dataclass

NOISE = 1e-4  # speed measurement noise, standard deviation [pu] (0.01 % of rated speed)


@dataclass(frozen=True)
class Scenario:
    key: str
    title: str
    plain: str  # one-sentence, non-technical description
    duration: float  # [s]
    load_steps: tuple[tuple[float, float], ...]  # (time [s], load torque [pu])
    noise_std: float = NOISE

    def load_at(self, t: float) -> float:
        load = self.load_steps[0][1]
        for ts, value in self.load_steps:
            if t >= ts:
                load = value
        return load

    @property
    def first_event(self) -> float:
        return self.load_steps[1][0] if len(self.load_steps) > 1 else 0.0

    @property
    def last_event(self) -> float:
        return self.load_steps[-1][0]


TUNING = (
    Scenario("tune_up", "Tuning: step up", "", 30.0, ((0.0, 0.60), (2.0, 0.75))),
    Scenario("tune_down", "Tuning: step down", "", 30.0, ((0.0, 0.80), (2.0, 0.40))),
)

TEST = (
    Scenario(
        "pickup",
        "Load pickup: 0.50 to 0.80 pu",
        "A large block of consumers is switched on at once.",
        40.0,
        ((0.0, 0.50), (5.0, 0.80)),
    ),
    Scenario(
        "rejection",
        "Load rejection: 1.00 to 0.10 pu",
        "The turbine suddenly loses 90 % of its load (a breaker opens). The danger is overspeed.",
        40.0,
        ((0.0, 1.00), (5.0, 0.10)),
    ),
    Scenario(
        "blocks",
        "Block loading: 0.20, 0.45, 0.70, 0.95 pu",
        "Load is added in three blocks, as when restoring a grid after a blackout.",
        80.0,
        ((0.0, 0.20), (5.0, 0.45), (30.0, 0.70), (55.0, 0.95)),
    ),
    Scenario(
        "noisy",
        "Noisy speed sensor: 0.60 to 0.70 pu",
        "A small load change measured with a speed signal five times noisier than normal.",
        40.0,
        ((0.0, 0.60), (5.0, 0.70)),
        noise_std=5 * NOISE,
    ),
)

BY_KEY = {s.key: s for s in TUNING + TEST}
