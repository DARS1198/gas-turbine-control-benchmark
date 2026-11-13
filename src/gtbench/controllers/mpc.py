"""Linear, offset-free Model Predictive Control (MPC).

At every control frame the controller:

1. updates an estimate of the turbine state and of the (unmeasured) load torque
   with a steady-state Kalman filter;
2. predicts the speed over the next ``horizon`` frames with a linear model;
3. chooses the fuel-command sequence that minimises
       sum (speed error)^2 + move_penalty * sum (fuel command change)^2
   subject to the fuel command limits;
4. applies only the first move and repeats at the next frame.

The quadratic program has box constraints only, so it is solved with a small
projected Gauss-Seidel routine (``box_qp``) that is easy to port to JavaScript.
The model is the turbine linearised at one operating point and it ignores the
two short transport delays, so the controller always works with an imperfect
model, as it would on a real unit.
"""

from __future__ import annotations

import numpy as np
from scipy.linalg import expm, solve_discrete_are

from ..plant import TurbineParams
from .base import Controller

BLOCKS = (1, 1, 2, 2, 4, 4, 8, 8, 14, 16)  # move blocking: 10 decisions cover 60 frames


def linear_model(p: TurbineParams, ts: float, load0: float = 0.7):
    """Discrete model x+ = A x + B u + Bd d,  y = C x  around rated speed.

    States: valve position, fuel system output, fuel flow at turbine, speed
    (all as deviations). ``u`` is the fuel command deviation, ``d`` the load
    torque deviation.
    """
    wf0 = p.w_min + load0 / p.torque_gain
    ac = np.array(
        [
            [-p.c / p.b, 0.0, 0.0, p.a * wf0 / p.b],
            [1.0 / p.t_f, -1.0 / p.t_f, 0.0, 0.0],
            [0.0, 1.0 / p.t_cd, -1.0 / p.t_cd, 0.0],
            [0.0, 0.0, p.torque_gain / p.t_i, -p.torque_speed / p.t_i],
        ]
    )
    bc = np.array([[p.a * (1.0 - p.w_min) / p.b], [0.0], [0.0], [0.0]])
    bdc = np.array([[0.0], [0.0], [0.0], [-1.0 / p.t_i]])
    m = np.zeros((6, 6))
    m[:4, :4] = ac
    m[:4, 4:5] = bc
    m[:4, 5:6] = bdc
    md = expm(m * ts)
    return md[:4, :4], md[:4, 4:5], md[:4, 5:6], np.array([[0.0, 0.0, 0.0, 1.0]])


def box_qp(h: np.ndarray, f: np.ndarray, lb: float, ub: float, x0: np.ndarray, sweeps: int = 60):
    """Minimise 0.5 x'Hx + f'x subject to lb <= x <= ub (projected Gauss-Seidel)."""
    x = np.clip(x0, lb, ub)
    n = len(x)
    for _ in range(sweeps):
        change = 0.0
        for i in range(n):
            xi = (-(f[i] + h[i] @ x) + h[i, i] * x[i]) / h[i, i]
            xi = min(ub, max(lb, xi))
            change = max(change, abs(xi - x[i]))
            x[i] = xi
        if change < 1e-10:
            break
    return x


class MPC(Controller):
    name = "MPC"

    def __init__(
        self,
        ts: float,
        move_penalty: float,
        dist_cov: float,
        params: TurbineParams | None = None,
        load0: float = 0.7,
        meas_cov: float = 1e-8,
        **kw,
    ):
        super().__init__(ts, **kw)
        p = params or TurbineParams()
        self.u_lin = p.steady_state_vce(load0)
        a, b, bd, c = linear_model(p, ts, load0)

        # Augmented model: load torque deviation as a fifth, constant state.
        self.a = np.block([[a, bd], [np.zeros((1, 4)), np.ones((1, 1))]])
        self.b = np.vstack([b, [[0.0]]])
        self.c = np.hstack([c, [[0.0]]])
        q = np.diag([1e-10, 1e-10, 1e-10, 1e-10, dist_cov])
        pk = solve_discrete_are(self.a.T, self.c.T, q, np.array([[meas_cov]]))
        self.l = (pk @ self.c.T / (self.c @ pk @ self.c.T + meas_cov)).ravel()

        # Prediction matrices: Y = phi x + gamma U, with U the blocked moves.
        np_ = sum(BLOCKS)
        nc = len(BLOCKS)
        phi = np.zeros((np_, 5))
        gam_full = np.zeros((np_, np_))
        ak = np.eye(5)
        imp = []
        for k in range(np_):
            imp.append((self.c @ ak @ self.b).item())
            ak = self.a @ ak
            phi[k] = (self.c @ ak).ravel()
        for k in range(np_):
            for j in range(k + 1):
                gam_full[k, j] = imp[k - j]
        blk = np.zeros((np_, nc))
        row = 0
        for j, size in enumerate(BLOCKS):
            blk[row : row + size, j] = 1.0
            row += size
        self.phi = phi
        self.gamma = gam_full @ blk
        diff = np.eye(nc) - np.eye(nc, k=-1)
        self.h = 2.0 * (self.gamma.T @ self.gamma + move_penalty * diff.T @ diff)
        self.h_inv = np.linalg.inv(self.h)
        self._g2 = 2.0 * self.gamma.T
        self._rho2 = 2.0 * move_penalty
        self.reset(self.u_lin)

    def reset(self, u0: float) -> None:
        du = u0 - self.u_lin
        # Steady state of the linear model for this input, at rated speed.
        lhs = np.zeros((5, 5))
        lhs[:4, :] = (self.a - np.eye(5))[:4, :]
        lhs[4, :] = self.c
        rhs = np.concatenate([-(self.b[:4, 0] * du), [0.0]])
        self.x = np.linalg.solve(lhs, rhs)
        self._u_prev = du
        self._warm = np.full(len(BLOCKS), du)

    def update(self, ref: float, meas: float) -> float:
        # 1. Kalman filter (predict with last input, correct with new measurement).
        x_pred = self.a @ self.x + self.b[:, 0] * self._u_prev
        self.x = x_pred + self.l * ((meas - 1.0) - x_pred[3])
        # 2-3. Quadratic program.
        free = self.phi @ self.x - (ref - 1.0)
        f = self._g2 @ free
        f[0] -= self._rho2 * self._u_prev
        lb, ub = self.u_min - self.u_lin, self.u_max - self.u_lin
        u = -self.h_inv @ f
        if u.min() < lb or u.max() > ub:
            u = box_qp(self.h, f, lb, ub, self._warm)
        self._warm = u
        # 4. Receding horizon: apply the first move only.
        self._u_prev = float(u[0])
        return self._u_prev + self.u_lin
