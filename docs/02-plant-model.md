# 02. Plant model

## In plain words

The model follows the fuel from the controller's command to the shaft:

1. The **controller** asks for an amount of fuel.
2. The **valve positioner** moves the fuel valve (fast, about 0.05 s).
3. The **fuel system** delivers the new flow (slower, about 0.4 s).
4. The fuel burns and the hot gas reaches the turbine after a short **delay** and a **compressor-discharge lag** (0.2 s).
5. More fuel means more **torque** on the shaft.
6. Torque above the load torque **accelerates the rotor**; below, the rotor slows. The rotor is heavy, so speed changes take seconds.

Even at no load the turbine needs 23 % of rated fuel just to keep its own compressor turning. That is why the fuel command is rescaled before it reaches the valve.

## Source

The model is the simplified single-shaft heavy-duty gas turbine representation published by W. I. Rowen in 1983 and widely reused in power-system stability studies (see the references in the [README](../README.md)). All parameters below are the values published with the model. No proprietary data is involved.

## Equations

All quantities are per unit; `N` is speed, `s` the Laplace variable.

| Block | Equation | Parameters |
|---|---|---|
| Fuel demand | `Wd = (0.77 * VCE + 0.23) * N`, with `VCE` limited to [-0.1, 1.5] | minimum fuel `Wmin = 0.23` |
| Valve positioner | `Vp / Wd = a / (b s + c)` | `a = 1`, `b = 0.05 s`, `c = 1` |
| Fuel system | `Wf2 / Vp = 1 / (Tf s + 1)` | `Tf = 0.4 s` |
| Combustor delay | `e^(-Ecr s)` | `Ecr = 0.01 s` |
| Compressor discharge | `Wf / Wf2 = 1 / (Tcd s + 1)` | `Tcd = 0.2 s` |
| Torque (Rowen's f2) | `T = 1.3 * (Wf - 0.23) + 0.5 * (1 - N)` | |
| Rotor | `dN/dt = (T - T_load) / Ti` | `Ti = 15.64 s` |
| Exhaust temperature (Rowen's f1) | `Tx = 950 - 700 * (1 - Wf) + 550 * (1 - N)` in °F, after a transport delay `Etd = 0.04 s` | rated `TR = 950 °F` |

Sanity checks built into the tests: no-load fuel gives zero torque, rated fuel gives rated torque and rated exhaust temperature.

The measured exhaust temperature (radiation shield `0.8 + 0.2 / (15 s + 1)` and thermocouple `1 / (2.5 s + 1)`) is simulated as well, ready for a future temperature-control loop. It is not used by the speed controllers.

## What is deliberately left out

- **Temperature control and acceleration control.** A real turbine controller runs these in parallel and lets the lowest fuel demand win. Here only the speed loop is closed. The exhaust temperature is still computed, and time spent above rated temperature is reported as a "thermal stress" metric, so a controller that over-fires is penalised in the results instead of being limited in the loop.
- **Inlet guide vanes, ambient conditions, fuel type.**
- **The generator and the grid.** The electrical load appears as a torque on the shaft.

## Numerical method

- Fixed step of 10 ms. The two transport delays are whole numbers of steps (1 and 4).
- First-order lags are advanced with their exact discrete solution, driven by the mid-step value of their input. The rotor uses forward Euler.
- The scheme is simple on purpose: the same lines are ported to JavaScript for the browser demo.

Verification (`tests/test_plant.py`):

| Check | Result |
|---|---|
| Against SciPy's adaptive Runge-Kutta solver, 10 s open-loop transient | within 0.02 % of rated speed |
| Step 10 ms against step 1 ms | within 0.01 % of rated speed |
| Steady state at 0, 30, 70 and 100 % load | holds to 1e-9 |

A first version of the scheme (lags driven by end-of-step inputs) failed the second check by a factor of three and was replaced.

## Parameter uncertainty used in the robustness study

| Parameter | Nominal | Range in the 300-turbine study | Reason |
|---|---|---|---|
| Rotor time constant `Ti` | 15.64 s | 12.2 to 25.2 s | range quoted for heavy-duty units in studies using this model |
| Fuel system `Tf` | 0.4 s | ×0.7 to ×1.6 | different fuel system, ageing |
| Compressor discharge `Tcd` | 0.2 s | ×0.7 to ×1.6 | different machine size |
| Valve positioner `b` | 0.05 s | ×0.7 to ×2.0 | actuator wear |
| Torque gain | 1.3 | ×0.90 to ×1.05 | fouling, ambient temperature |
| Extra dead time | 0 s | 0 to 0.15 s | signal processing and transport not in the model |
