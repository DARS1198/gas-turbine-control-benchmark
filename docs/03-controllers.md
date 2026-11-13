# 03. Controllers

All four controllers share one interface (`controllers/base.py`). Every 40 ms each receives the speed reference and the measured speed, and returns a fuel command between -0.1 and 1.5 pu. None of them can see the load, the fuel flow or the turbine parameters at run time.

| | Droop | PID | Fuzzy | MPC |
|---|---|---|---|---|
| Idea | Fuel proportional to speed error | Proportional + accumulated + rate of change | Rules blended smoothly, plus accumulated error | Predict with a model, optimise, repeat |
| Returns exactly to 60 Hz | No | Yes | Yes | Yes |
| Knows the fuel limits in advance | No | No (reacts after hitting them) | No | Yes |
| Tuned parameters | 0 (published values) | 3 | 5 | 2 |
| Code | `droop.py` | `pid.py` | `fuzzy.py` | `mpc.py` |

## Droop governor (baseline)

The governor published with the turbine model: `W / (Y s + Z)` with `W = 25`, `Y = 0.05 s`, `Z = 1`. A gain of 25 is a 4 % droop: the speed must fall 4 % for the governor to add 100 % fuel. It is used with its published values and is not tuned.

**Plain words:** push the fuel in proportion to how far the speed is off. Because some error is needed to hold any extra fuel, the speed never fully returns after a load change. On a large grid that is a feature (units share load); on an isolated unit it leaves the frequency off target.

## PID

```
u = Kp * e  +  I  +  D
I accumulates Ki * e          (frozen while the output is saturated in the same direction)
D = -Kd * d(speed)/dt         (filtered with a 0.1 s time constant)
```

- The derivative acts on the measurement, not on the error, to avoid a kick when the reference changes.
- Anti-windup: without it, the integral would keep growing during a load rejection while the fuel command sits at its lower limit, and the speed would undershoot badly afterwards. A unit test checks the behaviour.

## Fuzzy PD + I

```
u = Ku * F(Ke * e, Kde * de/dt)  +  Ki * integral(e)
```

`F` is a rule base with two inputs, the error and its rate of change. Each input is described by five overlapping categories (negative big, negative small, zero, positive small, positive big). The 25 rules follow one pattern: the further the speed is from the reference and the faster it is moving away, the larger the fuel correction. Output is the weighted average of the rule outputs (zero-order Sugeno inference).

A `warp` parameter bends the categories. Below 1 the controller reacts strongly to small errors and flattens for large ones; above 1 the opposite. With `warp = 1` the rule base reduces exactly to a linear PD, which a unit test confirms. This makes the comparison with the PID meaningful: the fuzzy controller has the same ingredients plus the freedom to be nonlinear.

The inference is about 40 lines of plain Python, with no fuzzy-logic library.

## MPC (model predictive control)

Each control frame:

1. **Estimate.** A steady-state Kalman filter updates the state of a linear turbine model *and* an estimate of the load torque, using only the measured speed. Estimating the load is what makes the controller return exactly to the reference (offset-free MPC).
2. **Predict.** The linear model predicts the speed over the next 60 frames (2.4 s).
3. **Optimise.** Choose the fuel commands that minimise

   ```
   sum (predicted speed error)^2  +  move_penalty * sum (change in fuel command)^2
   ```

   subject to -0.1 ≤ fuel command ≤ 1.5. Ten decision variables cover the 60 frames (move blocking: 1, 1, 2, 2, 4, 4, 8, 8, 14, 16 frames).
4. **Apply** the first command only, then start again at the next frame.

Design choices:

- **The model is imperfect on purpose.** It is linearised at 70 % load and ignores both transport delays. The controller is never given the true plant.
- **The solver is in the repository.** With only box constraints the quadratic program is solved by projected Gauss-Seidel (`box_qp`, 15 lines). If the unconstrained solution is already inside the limits, which is most of the time, it is used directly. A test compares the solver with SciPy on random problems.
- **Two tuned parameters:** the move penalty (how gently to move the valve) and the assumed variability of the load (how fast the load estimate reacts).

## Starting conditions

Every controller is initialised at steady state for the initial load (bumpless start). A test confirms that with no disturbance nothing moves.
