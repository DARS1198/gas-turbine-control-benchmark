"""Automatic tuning: every controller gets the same optimiser, cost and settings.

Cost of one turbine (lower is better), averaged over the tuning scenarios:

    J = IAE [%*s]  +  TRAVEL_WEIGHT * fuel-command travel [pu]

The first term rewards holding speed. The second penalises a nervous fuel
valve, which is what an over-aggressive controller produces once measurement
noise is present.

Two tuning modes are kept side by side:

* ``nominal``: minimise J on the nominal turbine only (iteration 1).
* ``robust`` : minimise  0.5 * J(nominal) + 0.5 * max J(corner turbines),
  where the two corner turbines sit at the edges of the assumed parameter
  uncertainty (iteration 2, the final design).

Parameters are searched on a log10 scale with SciPy's differential evolution
and a fixed random seed, so the tuning is reproducible.
"""

from __future__ import annotations

import json
from functools import lru_cache
from pathlib import Path

import numpy as np
from scipy.optimize import differential_evolution

from .controllers import MPC, PID, Controller, DroopGovernor, FuzzyPID
from .metrics import compute
from .plant import TurbineParams
from .scenarios import TUNING
from .simulate import TS, simulate

TRAVEL_WEIGHT = 0.5
SEED = 2026
TUNED_FILE = Path(__file__).with_name("tuned.json")

# name -> (parameter names, log10 bounds)
SPACES: dict[str, tuple[tuple[str, ...], tuple[tuple[float, float], ...]]] = {
    "PID": (("kp", "ki", "kd"), ((0.0, 2.5), (0.0, 2.5), (-3.0, 2.0))),
    "Fuzzy": (
        ("ke", "kde", "ku", "ki", "warp"),
        ((1.0, 3.0), (0.0, 3.0), (-1.5, 1.0), (0.0, 2.5), (-0.5, 0.5)),
    ),
    "MPC": (("move_penalty", "dist_cov"), ((-6.0, 0.0), (-10.0, -2.0))),
}

_BASE = TurbineParams()
# Edges of the assumed uncertainty. "sluggish": light rotor, slow fuel path, extra dead
# time (hardest to keep stable). "stiff": heavy rotor, fast fuel path (slowest to recover).
CORNERS = (
    _BASE.perturbed(t_i=12.2, t_f=0.4 * 1.6, t_cd=0.2 * 1.6, b=0.05 * 2.0,
                    torque_gain=1.3 * 1.05, extra_delay=0.15),
    _BASE.perturbed(t_i=25.2, t_f=0.4 * 0.7, t_cd=0.2 * 0.7, b=0.05 * 0.7,
                    torque_gain=1.3 * 0.90),
)  # fmt: skip
MODES = ("nominal", "robust")

_CLASSES = {"Droop": DroopGovernor, "PID": PID, "Fuzzy": FuzzyPID, "MPC": MPC}
ORDER = ("Droop", "PID", "Fuzzy", "MPC")


def build(name: str, values: dict[str, float] | None = None, mode: str = "robust") -> Controller:
    """Create a controller; with ``values=None`` use the committed tuned values of ``mode``."""
    if name == "Droop":
        return DroopGovernor(TS)
    if values is None:
        values = load_tuned()[mode][name]["params"]
    return _CLASSES[name](TS, **values)


def _plant_cost(name: str, values: dict[str, float], plant: TurbineParams) -> float:
    total = 0.0
    for sc in TUNING:
        m = compute(simulate(build(name, values), sc, plant, seed=SEED))
        if not m.stable or not np.isfinite(m.iae_pct_s):
            return 1e6
        total += m.iae_pct_s + TRAVEL_WEIGHT * m.fuel_travel_pu
    return total / len(TUNING)


def cost(name: str, values: dict[str, float], mode: str = "robust") -> float:
    nominal = _plant_cost(name, values, _BASE)
    if mode == "nominal" or nominal >= 1e6:
        return nominal
    return 0.5 * nominal + 0.5 * max(_plant_cost(name, values, c) for c in CORNERS)


def tune(name: str, mode: str = "robust", maxiter: int = 40, popsize: int = 15) -> dict:
    names, bounds = SPACES[name]

    def objective(x: np.ndarray) -> float:
        try:
            return cost(name, dict(zip(names, 10.0**x, strict=True)), mode)
        except (np.linalg.LinAlgError, ValueError, FloatingPointError):
            return 1e6

    res = differential_evolution(
        objective, bounds, seed=SEED, maxiter=maxiter, popsize=popsize, tol=1e-3, polish=False
    )
    params = {k: float(f"{v:.5g}") for k, v in zip(names, 10.0**res.x, strict=True)}
    return {
        "params": params,
        "cost": float(cost(name, params, mode)),
        "nominal_cost": float(cost(name, params, "nominal")),
        "evaluations": int(res.nfev),
    }


def tune_all(path: Path = TUNED_FILE) -> dict:
    tuned = {mode: {name: tune(name, mode) for name in SPACES} for mode in MODES}
    path.write_text(json.dumps(tuned, indent=2) + "\n")
    return tuned


@lru_cache(maxsize=4)
def load_tuned(path: Path = TUNED_FILE) -> dict:
    return json.loads(path.read_text())
