// Proves that the browser simulator (docs/sim.js) reproduces the Python results.
import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { test } from "node:test";

import { build, simulate } from "../../docs/sim.js";

const data = JSON.parse(readFileSync(new URL("../../docs/demo-data.json", import.meta.url)));

for (const ref of data.reference) {
  test(`${ref.controller} / ${ref.scenario} / ${ref.mode} tuning matches Python`, () => {
    const sc = data.scenarios.find((s) => s.key === ref.scenario);
    const r = simulate(build(ref.controller, data.modes[ref.mode]), { ...sc, noiseStd: 0 });
    let worst = 0;
    ref.speed.forEach((v, i) => (worst = Math.max(worst, Math.abs(v - r.speed[5 * i]))));
    ref.vce.forEach((v, i) => (worst = Math.max(worst, Math.abs(v - r.vce[5 * i]))));
    assert.ok(worst < 1e-7, `largest difference ${worst}`);
  });
}
