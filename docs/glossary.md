# Glossary

Plain-language meaning of the terms used in this project.

| Term | Meaning |
|---|---|
| **Per unit (pu)** | A quantity divided by its rated value. Load 0.8 pu means 80 % of rated load. Speed 1.0 pu means rated speed. |
| **Rated speed** | The speed the turbine is built to run at: 3600 rpm for a 60 Hz machine. Speed and grid frequency are the same thing seen from two sides, so 1 % of speed is 0.6 Hz. |
| **Isolated (island) operation** | The turbine alone supplies its consumers. Nothing else holds the frequency, so the turbine controller must. |
| **Load pickup** | Consumers are added. The turbine slows down until more fuel arrives. |
| **Load rejection** | Load is lost suddenly, for example a breaker opens. The turbine accelerates; if it reaches about 110 % it is tripped to protect it. |
| **Controller** | The algorithm that reads the speed and decides the fuel command, many times per second. |
| **Fuel command (VCE)** | The controller output: how much fuel it asks for, in per unit. |
| **Droop governor** | The simplest controller: fuel proportional to speed error. Stable and simple, but after a load change the speed stays a little off. |
| **PID** | Proportional, integral, derivative. The workhorse of industrial control. The integral part removes the lasting error that droop leaves. |
| **Fuzzy controller** | A controller built from rules such as "if the speed is far too low and still falling, add a lot of fuel", blended smoothly. |
| **MPC** | Model predictive control. It carries a model of the turbine, predicts the next few seconds, and picks the fuel plan that gives the best predicted result within the valve limits. It repeats this every cycle. |
| **Tuning** | Choosing a controller's numbers (gains). Here an optimiser does it, with the same rules for every controller. |
| **Peak deviation** | The largest distance between actual and rated speed after an event. |
| **Settling time** | How long until the speed stays within ±0.1 % of rated. |
| **IAE** | Integral of absolute error: the speed error added up over time, in %·s. One number for "how far and for how long". |
| **Valve travel** | The total distance the fuel command moved. A nervous controller makes the valve work constantly, which wears it. |
| **Dead time** | A pure delay: nothing happens for a moment after the command changes. Delay is what most often makes control loops unstable. |
| **Rotor time constant** | How heavy the rotor is, in seconds. A heavy rotor changes speed slowly. |
| **Robustness** | Whether the controller still works when the real machine differs from the model used to design it. |
| **Monte Carlo study** | Repeating a test many times with randomly varied conditions, here 300 different turbines. |
| **Unstable** | The speed keeps oscillating, or leaves the safe range, instead of settling. |
| **Control frame** | One cycle of the controller. Here 40 ms, a typical order of magnitude for turbine controllers. |
| **Kalman filter** | The part of the MPC that estimates what cannot be measured (here the load) from what can (the speed). |
