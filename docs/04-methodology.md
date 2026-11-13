# 04. Benchmark methodology

## Principle

A comparison of controllers is only as good as the effort spent on each one. If the PID is tuned in five minutes and the MPC over a weekend, the result measures the weekend. This benchmark removes the human from the tuning and applies the same rules to every controller.

## Rules

1. **Same information.** Each controller receives the speed reference and the noisy measured speed, once per 40 ms frame. Nothing else.
2. **Same tuning procedure.** One optimiser (SciPy differential evolution, population 15 per parameter, up to 40 generations, fixed seed), one cost function, logarithmic search ranges several decades wide. The optimiser stops early when the population has converged, so the number of evaluations differs (330 to 3075).
3. **Tune on one set of events, score on another.** The two tuning events and the four test events share no load profile. A unit test enforces it.
4. **Score on turbines the controllers were not tuned on.** The 300 random turbines of the robustness study do not contain the tuning turbines. A unit test enforces it.
5. **Everything seeded.** Same inputs, same outputs, to the last digit.

The droop governor is the exception to rule 2: it is used with its published parameters as a fixed reference point.

## Tuning cost

For one turbine, averaged over the two tuning events:

```
J = IAE [%·s]  +  0.5 * fuel-command travel [pu]
```

The first term rewards holding speed. The second penalises valve movement; with sensor noise present it stops the optimiser from choosing gains so high that the valve never rests. The weight 0.5 is a design choice and is the same for all controllers. An unstable run costs one million.

Two tuning modes are kept:

| Mode | Cost minimised | Used for |
|---|---|---|
| Nominal (iteration 1) | `J` on the nominal turbine | Showing what "optimal on the model" delivers |
| Robust (iteration 2, final) | `0.5 * J(nominal) + 0.5 * max(J(sluggish), J(stiff))` | Every headline result |

The two corner turbines sit at the edges of the assumed uncertainty ([table](02-plant-model.md#parameter-uncertainty-used-in-the-robustness-study)): *sluggish* has the lightest rotor, the slowest fuel path, the highest torque gain and 0.15 s of extra dead time, which is the hardest combination to keep stable; *stiff* has the heaviest rotor, the fastest fuel path and the lowest gain.

## Scenarios

| Set | Event | Why it is there |
|---|---|---|
| Tuning | Load 0.60 to 0.75 pu | Medium step up |
| Tuning | Load 0.80 to 0.40 pu | Large step down |
| Test | **Load pickup** 0.50 to 0.80 pu | The standard frequency-dip test |
| Test | **Load rejection** 1.00 to 0.10 pu | Overspeed risk; drives the fuel command into its lower limit |
| Test | **Block loading** 0.20, 0.45, 0.70, 0.95 pu | Three events at different operating points, ending near full load |
| Test | **Noisy sensor** 0.60 to 0.70 pu with 5 times the normal noise | Shows how each controller treats the fuel valve |

Normal sensor noise is 0.01 % of rated speed (standard deviation), present in every run.

## Metrics

| Metric | Unit | Meaning | Better |
|---|---|---|---|
| Peak deviation | % | Largest speed error | lower |
| Maximum speed | % of rated | Margin to the 110 % overspeed trip | lower |
| Settling time | s | Time after the last event until the speed stays within ±0.1 % | lower |
| IAE | %·s | Speed error accumulated over the run | lower |
| Final error | % | Mean error over the last 5 s | lower |
| Valve travel | pu | Total movement of the fuel command (actuator wear) | lower |
| Over-temperature | °F·s | Exhaust temperature above rated, accumulated (thermal stress) | lower |
| CPU time | µs per frame | Mean time spent in the controller (Python, single core) | lower |
| Stable | yes/no | Speed stayed between 94 and 110 % and swings less than 0.5 % peak to peak in the last 5 s | yes |

Speed errors are in percent of rated speed. On a 60 Hz unit 1 % is 0.6 Hz.

## Robustness study

- 300 turbines drawn at random from the uncertainty ranges, same 300 for every controller.
- Two events each (load pickup, load rejection): 600 runs per controller.
- A run **passes** if it is stable and settles within 20 s.
- In addition, two one-parameter sweeps on the load pickup event: extra dead time from 0 to 0.6 s, and rotor time constant from 8 to 40 s (beyond the range used in tuning, to find where each design breaks).

## Reproducing

```bash
gtbench tune   # writes src/gtbench/tuned.json (about 5 minutes)
gtbench run    # writes results/ (about 1 minute)
```

CPU-time figures depend on the machine; every other number is deterministic.

## Threats to validity

- **The cost function shapes the ranking.** A different weight on valve travel would produce different gains. The weight is disclosed and identical for all.
- **The corner turbines come from the same ranges as the test turbines.** That is how robust design works (the designer states the uncertainty and designs for it), but it means the 100 % pass rate is a statement about *that* uncertainty set, not about any turbine.
- **MPC is given a model of the same structure as the plant.** Its model is linearised and has no delays, but it is still closer to the truth than a model identified from plant data would be.
- **One model, one unit size.** See [06](06-limitations-and-roadmap.md).
