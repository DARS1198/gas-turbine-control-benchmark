"""Export everything the browser demo needs into one JSON file.

The demo (``docs/index.html`` + ``docs/sim.js``) re-implements the simulator in
JavaScript. This file gives it the tuned parameters and the MPC matrices, plus
noise-free reference trajectories that ``tests/js/parity.test.mjs`` uses to
prove that the JavaScript port reproduces the Python results.
"""

from __future__ import annotations

import json
from dataclasses import replace
from pathlib import Path

from .scenarios import TEST
from .simulate import simulate
from .tuning import MODES, ORDER, build, load_tuned


def demo_data() -> dict:
    tuned = load_tuned()
    data: dict = {"modes": {}, "scenarios": [], "reference": []}
    for mode in MODES:
        mpc = build("MPC", mode=mode)
        data["modes"][mode] = {
            "PID": tuned[mode]["PID"]["params"],
            "Fuzzy": tuned[mode]["Fuzzy"]["params"],
            "MPC": {
                "uLin": mpc.u_lin,
                "a": mpc.a.tolist(),
                "b": mpc.b.ravel().tolist(),
                "l": mpc.l.tolist(),
                "phi": mpc.phi.tolist(),
                "g2": mpc._g2.tolist(),
                "h": mpc.h.tolist(),
                "hInv": mpc.h_inv.tolist(),
                "rho2": mpc._rho2,
            },
        }
    for sc in TEST:
        data["scenarios"].append(
            {"key": sc.key, "title": sc.title, "plain": sc.plain, "duration": sc.duration,
             "loadSteps": [list(s) for s in sc.load_steps], "noiseStd": sc.noise_std}
        )  # fmt: skip
    for sc in TEST[:2]:
        quiet = replace(sc, noise_std=0.0)
        for mode in MODES:
            for name in ORDER:
                r = simulate(build(name, mode=mode), quiet)
                data["reference"].append(
                    {"scenario": sc.key, "mode": mode, "controller": name,
                     "speed": r.speed[::5].tolist(), "vce": r.vce[::5].tolist()}
                )  # fmt: skip
    return data


def write_demo_data(path: Path) -> None:
    path.write_text(json.dumps(demo_data(), separators=(",", ":")) + "\n")
