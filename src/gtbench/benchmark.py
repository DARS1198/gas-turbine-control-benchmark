"""Run the full benchmark and write machine-readable results."""

from __future__ import annotations

import csv
import json
from pathlib import Path

import numpy as np

from .metrics import Metrics, compute
from .plant import TurbineParams
from .scenarios import BY_KEY, TEST
from .simulate import SimResult, simulate
from .tuning import ORDER, SEED, build, load_tuned

N_PLANTS = 300  # randomly perturbed turbines in the robustness study
ROBUST_SCENARIOS = ("pickup", "rejection")
SETTLE_LIMIT = 20.0  # a robustness run "passes" if it is stable and settles within this [s]
DELAY_SWEEP = tuple(round(0.05 * i, 2) for i in range(13))  # extra dead time 0 .. 0.6 s
INERTIA_SWEEP = (8.0, 10.0, 12.2, 15.64, 20.0, 25.2, 30.0, 40.0)  # rotor time constant [s]


def nominal_runs() -> dict[str, dict[str, SimResult]]:
    """Each controller on each test scenario with the nominal turbine."""
    return {sc.key: {n: simulate(build(n), sc, seed=SEED) for n in ORDER} for sc in TEST}


def sample_plants(n: int = N_PLANTS, seed: int = SEED) -> list[TurbineParams]:
    """Turbines whose parameters differ from what the controllers were tuned for.

    The rotor time constant spans the 12.2 - 25.2 s range reported for
    heavy-duty units; the other ranges represent ageing, a different fuel
    system, and unmodelled dead time.
    """
    rng = np.random.default_rng(seed)
    base = TurbineParams()
    plants = []
    for _ in range(n):
        plants.append(
            base.perturbed(
                t_i=rng.uniform(12.2, 25.2),
                t_f=base.t_f * rng.uniform(0.7, 1.6),
                t_cd=base.t_cd * rng.uniform(0.7, 1.6),
                b=base.b * rng.uniform(0.7, 2.0),
                torque_gain=base.torque_gain * rng.uniform(0.90, 1.05),
                extra_delay=float(np.round(rng.uniform(0.0, 0.15), 2)),
            )
        )
    return plants


def passed(m: Metrics) -> bool:
    return bool(m.stable and np.isfinite(m.settling_s) and m.settling_s <= SETTLE_LIMIT)


def robustness(mode: str = "robust") -> list[dict]:
    """Monte Carlo over perturbed turbines, with the tuning of ``mode``."""
    names = ORDER if mode == "robust" else ORDER[1:]
    rows = []
    for i, plant in enumerate(sample_plants()):
        for key in ROBUST_SCENARIOS:
            for name in names:
                m = compute(simulate(build(name, mode=mode), BY_KEY[key], plant, seed=SEED + i))
                rows.append(
                    {"plant": i, "scenario": key, "controller": name, **m.as_dict(),
                     "passed": passed(m)}
                )  # fmt: skip
    return rows


def sweep(field: str, values: tuple[float, ...], scenario: str = "pickup") -> list[dict]:
    """Change one turbine parameter at a time and record how each controller copes."""
    rows = []
    for v in values:
        plant = TurbineParams().perturbed(**{field: v})
        for name in ORDER:
            m = compute(simulate(build(name), BY_KEY[scenario], plant, seed=SEED))
            rows.append({"value": v, "controller": name, **m.as_dict()})
    return rows


def _write_csv(path: Path, rows: list[dict]) -> None:
    with path.open("w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0]))
        w.writeheader()
        for row in rows:
            w.writerow({k: (f"{v:.6g}" if isinstance(v, float) else v) for k, v in row.items()})


def run_all(out: Path) -> dict:
    out.mkdir(parents=True, exist_ok=True)
    runs = nominal_runs()
    nominal = [
        {"scenario": key, "controller": name, **compute(r).as_dict()}
        for key, by_ctrl in runs.items()
        for name, r in by_ctrl.items()
    ]
    robust = robustness()
    robust_it1 = robustness("nominal")
    nominal_it1 = [
        {"scenario": sc.key, "controller": name,
         **compute(simulate(build(name, mode="nominal"), sc, seed=SEED)).as_dict()}
        for sc in TEST
        for name in ORDER[1:]
    ]  # fmt: skip
    delay = sweep("extra_delay", DELAY_SWEEP)
    inertia = sweep("t_i", INERTIA_SWEEP)
    _write_csv(out / "nominal_metrics.csv", nominal)
    _write_csv(out / "robustness_runs.csv", robust)
    _write_csv(out / "iteration1_nominal_metrics.csv", nominal_it1)
    _write_csv(out / "iteration1_robustness_runs.csv", robust_it1)
    _write_csv(out / "sweep_dead_time.csv", delay)
    _write_csv(out / "sweep_inertia.csv", inertia)
    (out / "tuned_parameters.json").write_text(json.dumps(load_tuned(), indent=2) + "\n")
    return {
        "runs": runs,
        "nominal": nominal,
        "robust": robust,
        "nominal_it1": nominal_it1,
        "robust_it1": robust_it1,
        "delay": delay,
        "inertia": inertia,
    }
