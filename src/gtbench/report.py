"""Turn benchmark results into figures and a Markdown results page."""

from __future__ import annotations

import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from .benchmark import N_PLANTS, ROBUST_SCENARIOS, SETTLE_LIMIT
from .metrics import BAND
from .scenarios import BY_KEY, TEST
from .tuning import ORDER

# Okabe-Ito colours: distinguishable with the common forms of colour blindness.
COLOR = {"Droop": "#7f7f7f", "PID": "#0072B2", "Fuzzy": "#E69F00", "MPC": "#D55E00"}
STYLE = {"Droop": (0, (4, 2)), "PID": "solid", "Fuzzy": (0, (6, 1, 1, 1)), "MPC": "solid"}
WIDTH = {"Droop": 1.4, "PID": 1.6, "Fuzzy": 1.8, "MPC": 2.2}

plt.rcParams.update(
    {
        "font.size": 10,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "axes.grid": True,
        "grid.alpha": 0.25,
        "figure.dpi": 110,
        "savefig.dpi": 150,
        "savefig.bbox": "tight",
    }
)


def _fmt(v: float, digits: int = 2) -> str:
    return "never" if isinstance(v, float) and math.isnan(v) else f"{v:.{digits}f}"


def fig_scenario(runs: dict, key: str, path: Path) -> None:
    sc = BY_KEY[key]
    fig, (a0, a1, a2) = plt.subplots(3, 1, figsize=(8, 7), sharex=True, height_ratios=[1, 2.4, 1.6])
    first = runs[ORDER[0]]
    a0.step(first.t, first.load, where="post", color="black", lw=1.5)
    a0.set_ylabel("Load [pu]")
    a0.set_title(sc.title, loc="left", fontweight="bold")
    for name in ORDER:
        r = runs[name]
        kw = {"color": COLOR[name], "ls": STYLE[name], "lw": WIDTH[name], "label": name}
        a1.plot(r.t, 100 * r.speed, **kw)
        a2.plot(r.t, r.vce, **kw)
    a1.axhspan(100 - BAND, 100 + BAND, color="green", alpha=0.12, lw=0)
    a1.set_ylabel("Speed [% of rated]")
    sec = a1.secondary_yaxis("right", functions=(lambda x: x * 0.6, lambda x: x / 0.6))
    sec.set_ylabel("Frequency [Hz]")
    a1.legend(ncol=4, frameon=False, loc="best")
    a2.set_ylabel("Fuel command [pu]")
    a2.set_xlabel("Time [s]")
    fig.align_ylabels()
    fig.savefig(path)
    plt.close(fig)


def fig_hero(runs: dict, path: Path) -> None:
    fig, axes = plt.subplots(1, 2, figsize=(11, 3.8))
    for ax, key, window in zip(axes, ("pickup", "rejection"), ((3, 22), (3, 25)), strict=True):
        for name in ORDER:
            r = runs[key][name]
            ax.plot(r.t, 100 * r.speed, color=COLOR[name], ls=STYLE[name], lw=WIDTH[name],
                    label=name)  # fmt: skip
        ax.axhspan(100 - BAND, 100 + BAND, color="green", alpha=0.12, lw=0)
        ax.set_xlim(*window)
        ax.set_title(BY_KEY[key].title, loc="left", fontweight="bold")
        ax.set_xlabel("Time [s]")
        ax.set_ylabel("Speed [% of rated]")
    axes[0].legend(frameon=False, loc="lower right")
    fig.savefig(path)
    plt.close(fig)


def scorecard(nominal: list[dict], robust: list[dict]) -> dict[str, dict[str, float]]:
    """One number per controller and axis, from the raw results."""
    card: dict[str, dict[str, float]] = {}
    for name in ORDER:
        rows = [r for r in nominal if r["controller"] == name]
        rob = [r for r in robust if r["controller"] == name]
        card[name] = {
            "Speed holding\n(IAE)": float(np.mean([r["iae_pct_s"] for r in rows])),
            "Peak deviation": float(np.mean([r["peak_dev_pct"] for r in rows])),
            "Valve travel": float(np.mean([r["fuel_travel_pu"] for r in rows])),
            "Thermal stress": float(np.mean([r["overtemp_degf_s"] for r in rows])),
            "Robustness\n(95th pct IAE)": float(np.percentile([r["iae_pct_s"] for r in rob], 95)),
            "Computation": float(np.mean([r["ctrl_time_us"] for r in rows])),
        }
    return card


def fig_radar(card: dict, path: Path) -> None:
    """Radar chart: every axis is 'best controller on this axis / this controller'."""
    axes_names = list(next(iter(card.values())))
    angles = np.linspace(0, 2 * np.pi, len(axes_names), endpoint=False)
    fig, ax = plt.subplots(figsize=(6.4, 6.4), subplot_kw={"projection": "polar"})
    for name in ORDER:
        score = []
        for axis in axes_names:
            best = min(card[n][axis] for n in ORDER)
            eps = 1e-9 + 0.02 * max(card[n][axis] for n in ORDER)
            score.append((best + eps) / (card[name][axis] + eps))
        closed = np.append(score, score[0])
        ang = np.append(angles, angles[0])
        ax.plot(ang, closed, color=COLOR[name], ls=STYLE[name], lw=WIDTH[name], label=name)
        ax.fill(ang, closed, color=COLOR[name], alpha=0.07)
    ax.set_xticks(angles)
    ax.set_xticklabels(axes_names)
    ax.set_ylim(0, 1.05)
    ax.set_yticks([0.25, 0.5, 0.75, 1.0])
    ax.set_yticklabels(["", "0.5", "", "best"], fontsize=8)
    ax.tick_params(axis="x", pad=24)
    ax.legend(loc="upper right", bbox_to_anchor=(1.28, 1.12), frameon=False)
    ax.set_title("Scorecard (outer edge = best in class)", y=1.12, fontweight="bold")
    fig.savefig(path)
    plt.close(fig)


def fig_robustness(robust: list[dict], path: Path) -> None:
    fig, axes = plt.subplots(1, 2, figsize=(11, 4))
    for ax, key in zip(axes, ROBUST_SCENARIOS, strict=True):
        data, labels = [], []
        for name in ORDER:
            vals = [r["iae_pct_s"] for r in robust if r["controller"] == name
                    and r["scenario"] == key]  # fmt: skip
            data.append(vals)
            labels.append(name)
        bp = ax.boxplot(data, tick_labels=labels, patch_artist=True, whis=(5, 95),
                        showfliers=True, flierprops={"ms": 2.5, "alpha": 0.4})  # fmt: skip
        for patch, name in zip(bp["boxes"], ORDER, strict=True):
            patch.set_facecolor(COLOR[name])
            patch.set_alpha(0.55)
        for med in bp["medians"]:
            med.set_color("black")
        ax.set_yscale("log")
        ax.set_ylabel("Accumulated speed error, IAE [%·s]  (log scale, lower is better)")
        ax.set_title(f"{BY_KEY[key].title}\n{N_PLANTS} perturbed turbines", loc="left",
                     fontweight="bold")  # fmt: skip
    fig.savefig(path)
    plt.close(fig)


def fig_sweep(rows: list[dict], xlabel: str, nominal_x: float, path: Path, title: str) -> None:
    fig, ax = plt.subplots(figsize=(7.5, 4))
    for name in ORDER:
        mine = [r for r in rows if r["controller"] == name]
        x = np.array([r["value"] for r in mine])
        y = np.array([r["iae_pct_s"] for r in mine])
        ok = np.array([r["stable"] for r in mine])
        ax.plot(x, y, color=COLOR[name], ls=STYLE[name], lw=WIDTH[name], label=name)
        ax.plot(x[~ok], y[~ok], "x", color=COLOR[name], ms=9, mew=2.5)
    ax.axvline(nominal_x, color="black", lw=0.8, ls=":")
    ax.text(nominal_x, ax.get_ylim()[1], " tuned here", va="top", fontsize=8)
    ax.plot([], [], "x", color="black", ms=8, mew=2, label="unstable")
    ax.set_yscale("log")
    ax.set_xlabel(xlabel)
    ax.set_ylabel("IAE [%·s]  (log scale)")
    ax.set_title(title, loc="left", fontweight="bold")
    ax.legend(frameon=False, ncol=5, fontsize=9, loc="upper center", bbox_to_anchor=(0.5, -0.2))
    fig.savefig(path)
    plt.close(fig)


def _nominal_table(nominal: list[dict], key: str) -> str:
    head = (
        "| Controller | Peak deviation [%] | Max speed [%] | Settling [s] | IAE [%·s] | "
        "Final error [%] | Valve travel [pu] | Over-temp [°F·s] | CPU [µs/frame] |\n"
        "|---|---|---|---|---|---|---|---|---|\n"
    )
    body = ""
    for r in (r for r in nominal if r["scenario"] == key):
        body += (
            f"| {r['controller']} | {_fmt(r['peak_dev_pct'])} | {_fmt(r['max_speed_pct'])} | "
            f"{_fmt(r['settling_s'])} | {_fmt(r['iae_pct_s'])} | {_fmt(r['ss_error_pct'], 3)} | "
            f"{_fmt(r['fuel_travel_pu'])} | {_fmt(r['overtemp_degf_s'], 1)} | "
            f"{_fmt(r['ctrl_time_us'], 1)} |\n"
        )
    return head + body


def _robust_table(robust: list[dict]) -> str:
    head = (
        "| Scenario | Controller | Pass rate [%] | Unstable runs | IAE median [%·s] | "
        "IAE 95th pct [%·s] | Worst max speed [%] |\n|---|---|---|---|---|---|---|\n"
    )
    body = ""
    for key in ROBUST_SCENARIOS:
        for name in ORDER:
            rows = [r for r in robust if r["controller"] == name and r["scenario"] == key]
            iae = [r["iae_pct_s"] for r in rows]
            body += (
                f"| {key} | {name} | {100 * np.mean([r['passed'] for r in rows]):.1f} | "
                f"{sum(not r['stable'] for r in rows)} | {np.median(iae):.2f} | "
                f"{np.percentile(iae, 95):.2f} | {max(r['max_speed_pct'] for r in rows):.2f} |\n"
            )
    return head + body


def _iteration_table(res: dict) -> str:
    head = (
        "| Controller | Tuning | Pass rate [%] | Unstable runs | "
        "Nominal IAE, mean of test scenarios [%·s] |\n|---|---|---|---|---|\n"
    )
    body = ""
    for name in ORDER[1:]:
        for label, rob, nom in (
            ("1: nominal only", res["robust_it1"], res["nominal_it1"]),
            ("2: robust (final)", res["robust"], res["nominal"]),
        ):
            rows = [r for r in rob if r["controller"] == name]
            iae = np.mean([r["iae_pct_s"] for r in nom if r["controller"] == name])
            body += (
                f"| {name} | {label} | {100 * np.mean([r['passed'] for r in rows]):.1f} | "
                f"{sum(not r['stable'] for r in rows)} of {len(rows)} | {iae:.2f} |\n"
            )
    return head + body


def _sweep_table(rows: list[dict], label: str) -> str:
    values = sorted({r["value"] for r in rows})
    head = f"| {label} | " + " | ".join(ORDER) + " |\n|---|" + "---|" * len(ORDER) + "\n"
    body = ""
    for v in values:
        cells = []
        for name in ORDER:
            r = next(r for r in rows if r["value"] == v and r["controller"] == name)
            cells.append(f"{r['iae_pct_s']:.2f}" + ("" if r["stable"] else " ✗"))
        body += f"| {v:g} | " + " | ".join(cells) + " |\n"
    return head + body


def write_report(res: dict, out: Path) -> None:
    fig_dir = out / "figures"
    fig_dir.mkdir(parents=True, exist_ok=True)
    for sc in TEST:
        fig_scenario(res["runs"][sc.key], sc.key, fig_dir / f"scenario_{sc.key}.png")
    fig_hero(res["runs"], fig_dir / "hero.png")
    card = scorecard(res["nominal"], res["robust"])
    fig_radar(card, fig_dir / "scorecard_radar.png")
    fig_robustness(res["robust"], fig_dir / "robustness.png")
    fig_sweep(
        res["delay"],
        "Extra dead time in the fuel path [s]",
        0.0,
        fig_dir / "sweep_dead_time.png",
        "How much unmodelled delay can each one take?",
    )
    fig_sweep(
        res["inertia"],
        "Rotor time constant T_I [s]",
        15.64,
        fig_dir / "sweep_inertia.png",
        "Lighter or heavier rotor than expected",
    )

    md = [
        "# Benchmark results (generated file)",
        "",
        "> Generated by `gtbench run`. Do not edit by hand. "
        "The interpretation is in [docs/05-results.md](../docs/05-results.md).",
        "",
        "## 1. Nominal turbine, test scenarios",
        "",
    ]
    for sc in TEST:
        md += [f"### {sc.title}", "", sc.plain, "", _nominal_table(res["nominal"], sc.key),
               f"![{sc.key}](figures/scenario_{sc.key}.png)", ""]  # fmt: skip
    md += [
        "## 2. Scorecard",
        "",
        "Raw values behind the radar chart; lower is better on every axis. All are means over "
        "the four test scenarios, except robustness, which is the 95th percentile of IAE over "
        "the perturbed turbines of section 3. Units: %·s, %, pu, °F·s, %·s, µs per frame.",
        "",
        "| Controller | " + " | ".join(a.replace("\n", " ") for a in card[ORDER[0]]) + " |",
        "|---|" + "---|" * len(card[ORDER[0]]),
    ]
    md += [f"| {n} | " + " | ".join(f"{v:.2f}" for v in card[n].values()) + " |" for n in ORDER]
    md += [
        "",
        "![radar](figures/scorecard_radar.png)",
        "",
        f"## 3. Robustness: {N_PLANTS} perturbed turbines",
        "",
        f"A run passes if it is stable and settles within ±{BAND} % in {SETTLE_LIMIT:g} s. "
        "The droop governor never returns to the reference by design, so its pass rate is 0.",
        "",
        _robust_table(res["robust"]),
        "![robustness](figures/robustness.png)",
        "",
        "## 4. One-parameter sweeps (load pickup scenario, IAE in %·s, ✗ = unstable)",
        "",
        "### Extra dead time [s]",
        "",
        _sweep_table(res["delay"], "Dead time [s]"),
        "![dead time](figures/sweep_dead_time.png)",
        "",
        "### Rotor time constant [s]",
        "",
        _sweep_table(res["inertia"], "T_I [s]"),
        "![inertia](figures/sweep_inertia.png)",
        "",
        "## 5. Design iteration: nominal tuning versus robust tuning",
        "",
        f"Same {N_PLANTS} perturbed turbines, both robustness scenarios pooled. "
        "Sections 1 to 4 use the robust tuning.",
        "",
        _iteration_table(res),
        "",
    ]
    (out / "RESULTS.md").write_text("\n".join(md))
