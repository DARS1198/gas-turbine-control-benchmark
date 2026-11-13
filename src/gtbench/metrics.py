"""Performance metrics computed from one simulation run.

Speed errors are reported in percent of rated speed (1 % = 0.6 Hz at 60 Hz).
"""

from __future__ import annotations

from dataclasses import asdict, dataclass

import numpy as np

from .simulate import TS, SimResult

BAND = 0.1  # settling band [% of rated speed]
OVERSPEED_TRIP = 110.0  # typical overspeed trip level [% of rated speed]
UNDERSPEED = 94.0  # below this a 60 Hz unit would be tripped by underfrequency protection [%]
RIPPLE = 0.5  # largest peak-to-peak speed swing tolerated in the final 5 s [%]


@dataclass
class Metrics:
    peak_dev_pct: float  # largest |speed - reference| [%]
    max_speed_pct: float  # highest speed reached [% of rated]
    settling_s: float  # time after the last load change to stay within +/-BAND; nan if never
    iae_pct_s: float  # integral of |speed error| [%*s]
    ss_error_pct: float  # mean |speed error| over the final 5 s [%]
    fuel_travel_pu: float  # total variation of the fuel command [pu] (valve wear proxy)
    overtemp_degf_s: float  # exhaust temperature above rated, integrated [degF*s]
    ctrl_time_us: float  # mean computation time per control frame [microseconds]
    stable: bool  # no overspeed trip, no underspeed, and no sustained oscillation at the end

    def as_dict(self) -> dict:
        return asdict(self)


def compute(r: SimResult, speed_ref: float = 1.0, t_rated: float = 950.0) -> Metrics:
    err = 100.0 * (r.speed - speed_ref)
    abs_err = np.abs(err)
    after = r.t >= r.scenario.last_event
    outside = np.where(after & (abs_err > BAND))[0]
    if len(outside) == 0:
        settling = 0.0
    elif outside[-1] == len(r.t) - 1 or r.t[-1] - r.t[outside[-1]] < 2.0:
        settling = float("nan")  # has not demonstrably settled
    else:
        settling = float(r.t[outside[-1]] + TS - r.scenario.last_event)
    tail = r.t >= r.t[-1] - 5.0
    max_speed = 100.0 * float(r.speed.max())
    return Metrics(
        peak_dev_pct=float(abs_err.max()),
        max_speed_pct=max_speed,
        settling_s=settling,
        iae_pct_s=float(abs_err.sum() * TS),
        ss_error_pct=float(abs_err[tail].mean()),
        fuel_travel_pu=float(np.abs(np.diff(r.vce)).sum()),
        overtemp_degf_s=float(np.clip(r.tx - t_rated, 0.0, None).sum() * TS),
        ctrl_time_us=r.ctrl_time_us,
        stable=bool(
            max_speed < OVERSPEED_TRIP
            and 100.0 * float(r.speed.min()) > UNDERSPEED
            and float(np.ptp(err[tail])) < RIPPLE
        ),
    )
