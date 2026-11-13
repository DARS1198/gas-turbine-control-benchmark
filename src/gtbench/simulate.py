"""Closed-loop simulation: plant at 10 ms, controller at a 40 ms frame."""

from __future__ import annotations

import time
from dataclasses import dataclass

import numpy as np

from .controllers.base import Controller
from .plant import GasTurbine, TurbineParams
from .scenarios import Scenario

DT = 0.01  # plant integration step [s]
TS = 0.04  # control frame [s]
SUBSTEPS = round(TS / DT)


@dataclass
class SimResult:
    scenario: Scenario
    controller: str
    t: np.ndarray  # [s], one sample per control frame
    speed: np.ndarray  # true speed [pu]
    vce: np.ndarray  # fuel command [pu]
    load: np.ndarray  # load torque [pu]
    torque: np.ndarray  # turbine torque [pu]
    tx: np.ndarray  # exhaust temperature [deg F]
    ctrl_time_us: float  # mean controller computation time per frame [microseconds]


def simulate(
    controller: Controller,
    scenario: Scenario,
    params: TurbineParams | None = None,
    seed: int = 0,
    speed_ref: float = 1.0,
) -> SimResult:
    params = params or TurbineParams()
    plant = GasTurbine(params, DT)
    load0 = scenario.load_at(0.0)
    plant.reset(load0)
    controller.reset(params.steady_state_vce(load0))
    rng = np.random.default_rng(seed)
    n = round(scenario.duration / TS)
    noise = rng.normal(0.0, scenario.noise_std, n) if scenario.noise_std > 0 else np.zeros(n)
    out = np.zeros((6, n))
    spent = 0.0
    for k in range(n):
        t = k * TS
        load = scenario.load_at(t)
        meas = plant.n + noise[k]
        t0 = time.perf_counter()
        vce = controller.update(speed_ref, meas)
        spent += time.perf_counter() - t0
        out[:, k] = (t, plant.n, vce, load, plant.torque, plant.tx)
        for _ in range(SUBSTEPS):
            plant.step(vce, load)
    return SimResult(scenario, controller.name, *out, ctrl_time_us=1e6 * spent / n)
