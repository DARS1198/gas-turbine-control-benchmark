# 01. Project plan

This document records what the project set out to do, how success was defined before any result existed, and how it ended.

## Problem

Engineers who commission turbine controls usually meet one control method (the vendor's) and rarely get to see alternatives side by side. Published comparisons of PID, fuzzy and predictive control on gas turbines exist, but most of them tune each controller by hand, test on the same events used for tuning, and report only the speed response. That makes it hard to trust the ranking.

## Goal

Build an open, reproducible benchmark that compares four speed controllers on a published gas-turbine model, **under rules that are the same for all of them**, and report cost and robustness as well as performance.

## Questions to answer

1. How much better than a plain droop governor are PID, fuzzy and MPC at holding speed through load changes?
2. What does each one cost in fuel-valve movement, thermal stress and computation?
3. How does each one behave when the real turbine differs from the design model?

## Scope

| In scope | Out of scope |
|---|---|
| Single-shaft heavy-duty gas turbine, Rowen model | Plant-specific or vendor-specific models and data |
| Isolated operation, speed (frequency) control | Grid-parallel operation, load sharing, AVR |
| Droop, PID, fuzzy, linear MPC | Temperature and acceleration limit loops, start-up sequence |
| Simulation, Python, plus a browser demo | Hardware, PLC code, real-time targets |

## Success criteria (fixed before the results)

| # | Criterion | Outcome |
|---|---|---|
| 1 | One command regenerates every table and figure from scratch | Met: `gtbench tune && gtbench run` |
| 2 | Simulator verified against an independent ODE solver | Met: within 0.02 % of rated speed, see `tests/test_plant.py` |
| 3 | Identical tuning procedure for every tuned controller; test events not used for tuning | Met, enforced by a test |
| 4 | At least five metrics per run, including a cost metric and a robustness study with at least 200 turbines | Met: 9 metrics, 300 turbines |
| 5 | Final designs stable on 100 % of the perturbed turbines | Not met in iteration 1 (93 to 94 %), met in iteration 2 |
| 6 | A non-specialist can understand the README and operate the demo | Plain-language README, glossary, live demo |
| 7 | Automated tests and continuous integration | Met: 46 Python tests, 16 parity checks, CI on every push |

## Milestones

| Stage | Deliverable | Exit check |
|---|---|---|
| M0 Planning | This plan, scope, success criteria | Criteria written before results |
| M1 Plant model | `plant.py` with published parameters | Steady state, physics sanity and ODE-solver tests pass |
| M2 Controllers | Four controllers behind one interface | Each controller's defining property tested |
| M3 Benchmark | Scenarios, metrics, automatic tuning | Reproducible to the last digit with a fixed seed |
| M4 Robustness | 300-turbine study, parameter sweeps | Criterion 5 evaluated |
| M5 Iteration | Robust tuning after M4 exposed instability | Criterion 5 met; cost of the fix quantified |
| M6 Communication | Docs, figures, browser demo | Demo numerically identical to Python |

## Risks identified at the start, and what happened

| Risk | Mitigation | What happened |
|---|---|---|
| Unfair comparison through uneven tuning effort | One optimiser, one cost, one seed for all | Worked; the fuzzy controller had to be restructured once so that it had the same P, I and D ingredients as the PID |
| Results that only hold for one model | Robustness study over published parameter ranges | Found real fragility; led to iteration 2 |
| Simulator errors that go unnoticed | Independent ODE check, step-size convergence test | First integration scheme missed the accuracy target by a factor of three and was replaced before any result was published |
| Browser demo drifting away from the Python code | Automated parity test in CI | 16 trajectories agree to 1e-7 |
| Over-claiming | Limitations document, no plant validation claimed | See [06](06-limitations-and-roadmap.md) |
