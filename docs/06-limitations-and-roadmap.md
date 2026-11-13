# 06. Limitations and roadmap

## What this project does not show

| Limitation | Consequence |
|---|---|
| **No validation against plant data.** The model is a published simplification, not a specific machine. | Absolute numbers (a 1.5 % dip, a 104 % peak) are indicative. The ranking and the trade-offs are the result. |
| **Only the speed loop is closed.** Real turbine controllers also run temperature and acceleration limiters and select the lowest fuel demand. | Over-firing is measured, not prevented. Large load pickups near full load would be slower on a real unit. |
| **Isolated unit, load as a torque step.** No generator dynamics, no grid, no voltage regulator, no load that depends on frequency. | Not applicable as-is to grid-parallel operation, where droop is the correct choice. |
| **One unit size and one fuel system.** | Results may differ for aeroderivative or twin-shaft machines. |
| **MPC uses a model with the same structure as the plant.** | An identified model would be less accurate; MPC's margin over PID is probably optimistic. |
| **Linear MPC, single linearisation point.** | Fine here because the model is nearly linear in load; not a general statement. |
| **Uncertainty ranges are assumptions.** | "0 of 600 unstable" holds for the stated ranges only. Beyond about 0.5 s of unmodelled dead time designs start to fail. |
| **Timing in Python on a PC.** | Says nothing about feasibility on a specific industrial controller. |
| **Sensor noise is white and Gaussian.** | Real speed pickups also show periodic and quantisation effects. |
| **One tuning cost.** | A different trade-off between speed error and valve travel changes the gains and could change the close PID/fuzzy result. |

## Roadmap

| Priority | Item | Why |
|---|---|---|
| 1 | Temperature and acceleration loops with low-value select | Closes the largest gap to a real turbine controller |
| 2 | Constraint on exhaust temperature inside the MPC | Lets MPC use its main structural advantage |
| 3 | Grid-parallel scenario with droop load sharing between two units | The other half of real operation |
| 4 | Reinforcement-learning controller under the same rules | A fair test of a learned policy against classical methods |
| 5 | MPC with a model identified from simulated step tests instead of the true structure | Removes the optimistic assumption |
| 6 | Controller code in IEC 61131-3 Structured Text and a hardware-in-the-loop run | Shows the path to an industrial controller |
| 7 | Sensitivity of the ranking to the tuning cost weight | Tests how far the conclusions depend on one design choice |

## Contributing

Issues and pull requests are welcome. A new controller needs only a class with `reset` and `update` (see `controllers/base.py`) and an entry in `tuning.py`; the benchmark, the report and the tests pick it up from there.
