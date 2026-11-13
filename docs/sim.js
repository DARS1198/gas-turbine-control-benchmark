// JavaScript port of the gtbench simulator (src/gtbench). Kept line-for-line close to the
// Python code; tests/js/parity.test.mjs checks that both produce the same trajectories.

export const DT = 0.01;
export const TS = 0.04;
const SUBSTEPS = 4;

export const DEFAULT_PARAMS = {
  a: 1.0, b: 0.05, c: 1.0, wMin: 0.23, tF: 0.4, eCr: 0.01, eTd: 0.04, tCd: 0.2, tI: 15.64,
  tR: 950.0, torqueGain: 1.3, torqueSpeed: 0.5, vceMin: -0.1, vceMax: 1.5, extraDelay: 0.0,
};

const clip = (x, lo, hi) => Math.min(hi, Math.max(lo, x));

export function steadyStateVce(p, load) {
  return load / p.torqueGain / (1.0 - p.wMin);
}

export class GasTurbine {
  constructor(params = {}) {
    const p = (this.p = { ...DEFAULT_PARAMS, ...params });
    this.kVp = 1.0 - Math.exp((-DT * p.c) / p.b);
    this.kF = 1.0 - Math.exp(-DT / p.tF);
    this.kCd = 1.0 - Math.exp(-DT / p.tCd);
    this.nCr = Math.max(0, Math.round((p.eCr + p.extraDelay) / DT));
    this.nTd = Math.max(0, Math.round(p.eTd / DT));
    this.reset(0.5);
  }

  reset(load) {
    const p = this.p;
    const wf = p.wMin + load / p.torqueGain;
    this.n = 1.0;
    this.vp = (wf * p.a) / p.c;
    this.wf2 = wf;
    this.wf = wf;
    this.bufCr = new Array(this.nCr + 1).fill(wf);
    this.bufTd = new Array(this.nTd + 1).fill(wf);
    this.tx = p.tR - 700.0 * (1.0 - wf);
    this.torque = load;
  }

  step(vceIn, load) {
    const p = this.p;
    const vce = clip(vceIn, p.vceMin, p.vceMax);
    const wd = ((1.0 - p.wMin) * vce + p.wMin) * this.n;
    const vpOld = this.vp;
    this.vp += this.kVp * ((wd * p.a) / p.c - this.vp);
    this.wf2 += this.kF * (0.5 * (vpOld + this.vp) - this.wf2);
    const combOld = this.bufCr[0];
    this.bufCr.push(this.wf2);
    this.bufCr.shift();
    const wfComb = this.bufCr[0];
    const wfOld = this.wf;
    this.wf += this.kCd * (0.5 * (combOld + wfComb) - this.wf);
    this.bufTd.push(wfComb);
    this.bufTd.shift();
    this.torque =
      p.torqueGain * (0.5 * (wfOld + this.wf) - p.wMin) + p.torqueSpeed * (1.0 - this.n);
    this.tx = p.tR - 700.0 * (1.0 - this.bufTd[0]) + 550.0 * (1.0 - this.n);
    this.n += (DT * (this.torque - load)) / p.tI;
  }
}

class Controller {
  constructor() {
    this.uMin = -0.1;
    this.uMax = 1.5;
  }
  clip(u) {
    return clip(u, this.uMin, this.uMax);
  }
  windup(uRaw, e) {
    return (uRaw > this.uMax && e > 0) || (uRaw < this.uMin && e < 0);
  }
}

export class Droop extends Controller {
  constructor() {
    super();
    this.gain = 25.0;
    this.k = 1.0 - Math.exp(-TS / 0.05);
    this.reset(0);
  }
  reset(u0) {
    this.loadRef = u0;
    this.x = 0.0;
  }
  update(ref, meas) {
    this.x += this.k * (this.gain * (ref - meas) - this.x);
    return this.clip(this.loadRef + this.x);
  }
}

export class PID extends Controller {
  constructor({ kp, ki, kd }) {
    super();
    Object.assign(this, { kp, ki, kd });
    this.kf = 1.0 - Math.exp(-TS / 0.1);
    this.reset(0);
  }
  reset(u0) {
    this.i = u0;
    this.prev = null;
    this.d = 0.0;
  }
  update(ref, meas) {
    const e = ref - meas;
    if (this.prev === null) this.prev = meas;
    const rate = (meas - this.prev) / TS;
    this.prev = meas;
    this.d += this.kf * (-this.kd * rate - this.d);
    const uRaw = this.kp * e + this.i + this.d;
    if (!this.windup(uRaw, e)) this.i += this.ki * e * TS;
    return this.clip(uRaw);
  }
}

const BASE = [-1.0, -0.5, 0.0, 0.5, 1.0];

export function memberships(x, centres) {
  const n = centres.length;
  const mu = new Array(n).fill(0.0);
  if (x <= centres[0]) mu[0] = 1.0;
  else if (x >= centres[n - 1]) mu[n - 1] = 1.0;
  else
    for (let i = 0; i < n - 1; i++) {
      const lo = centres[i];
      const hi = centres[i + 1];
      if (lo <= x && x <= hi) {
        const w = (x - lo) / (hi - lo);
        mu[i] = 1.0 - w;
        mu[i + 1] = w;
        break;
      }
    }
  return mu;
}

export class Fuzzy extends Controller {
  constructor({ ke, kde, ku, ki, warp }) {
    super();
    Object.assign(this, { ke, kde, ku, ki });
    this.centres = BASE.map((c) => Math.sign(c) * Math.abs(c) ** warp);
    this.kf = 1.0 - Math.exp(-TS / 0.1);
    this.reset(0);
  }
  reset(u0) {
    this.i = u0;
    this.prev = null;
    this.de = 0.0;
  }
  surface(eN, deN) {
    const muE = memberships(eN, this.centres);
    const muD = memberships(deN, this.centres);
    let out = 0.0;
    for (let i = 0; i < 5; i++) {
      if (muE[i] === 0.0) continue;
      for (let j = 0; j < 5; j++) {
        if (muD[j] === 0.0) continue;
        out += muE[i] * muD[j] * BASE[clip(i + j - 2, 0, 4)];
      }
    }
    return out;
  }
  update(ref, meas) {
    const e = ref - meas;
    if (this.prev === null) this.prev = meas;
    const rate = -(meas - this.prev) / TS;
    this.prev = meas;
    this.de += this.kf * (rate - this.de);
    const uRaw =
      this.ku * this.surface(clip(this.ke * e, -1, 1), clip(this.kde * this.de, -1, 1)) + this.i;
    if (!this.windup(uRaw, e)) this.i += this.ki * e * TS;
    return this.clip(uRaw);
  }
}

const dot = (a, b) => {
  let s = 0.0;
  for (let i = 0; i < a.length; i++) s += a[i] * b[i];
  return s;
};

export function boxQp(h, f, lb, ub, x0, sweeps = 60) {
  const x = x0.map((v) => clip(v, lb, ub));
  for (let s = 0; s < sweeps; s++) {
    let change = 0.0;
    for (let i = 0; i < x.length; i++) {
      const xi = clip((-(f[i] + dot(h[i], x)) + h[i][i] * x[i]) / h[i][i], lb, ub);
      change = Math.max(change, Math.abs(xi - x[i]));
      x[i] = xi;
    }
    if (change < 1e-10) break;
  }
  return x;
}

// Solve the 5x5 linear system used to initialise the MPC observer (Gaussian elimination).
function solve(m, rhs) {
  const n = rhs.length;
  const a = m.map((row, i) => [...row, rhs[i]]);
  for (let c = 0; c < n; c++) {
    let piv = c;
    for (let r = c + 1; r < n; r++) if (Math.abs(a[r][c]) > Math.abs(a[piv][c])) piv = r;
    [a[c], a[piv]] = [a[piv], a[c]];
    for (let r = c + 1; r < n; r++) {
      const k = a[r][c] / a[c][c];
      for (let j = c; j <= n; j++) a[r][j] -= k * a[c][j];
    }
  }
  const x = new Array(n).fill(0);
  for (let r = n - 1; r >= 0; r--) {
    let s = a[r][n];
    for (let j = r + 1; j < n; j++) s -= a[r][j] * x[j];
    x[r] = s / a[r][r];
  }
  return x;
}

export class MPC extends Controller {
  constructor(m) {
    super();
    this.m = m;
    this.reset(m.uLin);
  }
  reset(u0) {
    const m = this.m;
    const du = u0 - m.uLin;
    const lhs = [];
    for (let i = 0; i < 4; i++) lhs.push(m.a[i].map((v, j) => v - (i === j ? 1 : 0)));
    lhs.push([0, 0, 0, 1, 0]);
    this.x = solve(lhs, [0, 1, 2, 3].map((i) => -m.b[i] * du).concat([0]));
    this.uPrev = du;
    this.warm = new Array(m.h.length).fill(du);
  }
  update(ref, meas) {
    const m = this.m;
    const xPred = m.a.map((row, i) => dot(row, this.x) + m.b[i] * this.uPrev);
    const innov = meas - 1.0 - xPred[3];
    this.x = xPred.map((v, i) => v + m.l[i] * innov);
    const free = m.phi.map((row) => dot(row, this.x) - (ref - 1.0));
    const f = m.g2.map((row) => dot(row, free));
    f[0] -= m.rho2 * this.uPrev;
    const lb = this.uMin - m.uLin;
    const ub = this.uMax - m.uLin;
    let u = m.hInv.map((row) => -dot(row, f));
    if (Math.min(...u) < lb || Math.max(...u) > ub) u = boxQp(m.h, f, lb, ub, this.warm);
    this.warm = u;
    this.uPrev = u[0];
    return this.uPrev + m.uLin;
  }
}

export function build(name, modeData) {
  if (name === "Droop") return new Droop();
  if (name === "PID") return new PID(modeData.PID);
  if (name === "Fuzzy") return new Fuzzy(modeData.Fuzzy);
  return new MPC(modeData.MPC);
}

// Small seeded generator so that every controller sees the same sensor noise.
export function gaussianSource(seed) {
  let s = seed >>> 0;
  const uniform = () => {
    s = (s + 0x6d2b79f5) >>> 0;
    let t = s;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
  return () => Math.sqrt(-2 * Math.log(uniform() + 1e-12)) * Math.cos(2 * Math.PI * uniform());
}

export function loadAt(loadSteps, t) {
  let load = loadSteps[0][1];
  for (const [ts, v] of loadSteps) if (t >= ts) load = v;
  return load;
}

export function simulate(controller, { duration, loadSteps, noiseStd = 0, seed = 1 }, params = {}) {
  const plant = new GasTurbine(params);
  const load0 = loadAt(loadSteps, 0);
  plant.reset(load0);
  controller.reset(steadyStateVce(plant.p, load0));
  const gauss = gaussianSource(seed);
  const n = Math.round(duration / TS);
  const out = { t: [], speed: [], vce: [], load: [] };
  for (let k = 0; k < n; k++) {
    const t = k * TS;
    const load = loadAt(loadSteps, t);
    const meas = plant.n + (noiseStd > 0 ? noiseStd * gauss() : 0);
    const vce = controller.update(1.0, meas);
    out.t.push(t);
    out.speed.push(plant.n);
    out.vce.push(vce);
    out.load.push(load);
    for (let i = 0; i < SUBSTEPS; i++) plant.step(vce, load);
  }
  return out;
}

export function metrics(r, lastEvent, band = 0.1) {
  const err = r.speed.map((s) => 100 * (s - 1));
  const abs = err.map(Math.abs);
  let lastOut = -1;
  for (let k = 0; k < abs.length; k++) if (r.t[k] >= lastEvent && abs[k] > band) lastOut = k;
  const end = r.t[r.t.length - 1];
  let settling = 0;
  if (lastOut >= 0) settling = end - r.t[lastOut] < 2.0 ? NaN : r.t[lastOut] + TS - lastEvent;
  const tail = err.filter((_, k) => r.t[k] >= end - 5);
  let travel = 0;
  for (let k = 1; k < r.vce.length; k++) travel += Math.abs(r.vce[k] - r.vce[k - 1]);
  const maxSpeed = 100 * Math.max(...r.speed);
  const minSpeed = 100 * Math.min(...r.speed);
  return {
    peak: Math.max(...abs),
    maxSpeed,
    settling,
    iae: abs.reduce((a, b) => a + b, 0) * TS,
    travel,
    stable: maxSpeed < 110 && minSpeed > 94 && Math.max(...tail) - Math.min(...tail) < 0.5,
  };
}
