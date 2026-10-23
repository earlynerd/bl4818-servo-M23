// Run with: node --test tests/test_latency_model.cjs
// tests/test_latency_model.py asserts the same parity scenario against the
// server's LatencyTracker, so the browser and server models stay in step.
const {test} = require('node:test');
const assert = require('node:assert/strict');
const LatencyModel = require('../player/latency_model.js');

const PARITY_SAMPLES = [[0, 150, 48.0], [0, 220, 41.5], [0, 310, 36.2], [0, 480, 30.9],
                        [0, 220, 40.7], [1, 260, 40.0]];
const PARITY_ESTIMATES = [[0, 100, 58.787754], [0, 260, 38.35218], [0, 700, 25.587631],
                          [1, 150, 48.219028], [1, 600, 27.6378], [2, 333, 34.951512]];
const close = (actual, expected, eps = 1e-5) =>
  assert.ok(Math.abs(actual - expected) < eps, `${actual} != ${expected}`);

test('parity with the server model', () => {
  const slots = {}, shared = {};
  for (const [slot, ma, ms] of PARITY_SAMPLES) {
    LatencyModel.learn(slots[slot] ??= {}, ma, ms);
    LatencyModel.learn(shared, ma, ms);
  }
  for (const [slot, ma, expected] of PARITY_ESTIMATES) {
    close(LatencyModel.estimate(slots[slot], shared, ma), expected);
  }
});

test('one sample is reproduced at its own current; no data gives null', () => {
  for (const ma of [30, 100, 137, 333, 800, 2900]) {
    const table = {};
    LatencyModel.learn(table, ma, 40);
    close(LatencyModel.estimate(table, table, ma), 40);
    close(LatencyModel.estimate(undefined, table, ma), 40);
  }
  assert.equal(LatencyModel.estimate({}, {}, 400), null);
  assert.equal(LatencyModel.source({}, {}, 400), 'default');
});

test('ten distinct bins across 100-500 mA and exact constant-torque interpolation', () => {
  const bins = new Set([100, 120, 140, 170, 200, 240, 280, 340, 400, 480]
    .map(ma => Math.floor(LatencyModel.position(ma) + 0.5)));
  assert.equal(bins.size, 10);
  const table = {};
  LatencyModel.learn(table, 100, 60);
  LatencyModel.learn(table, 400, 30);
  close(LatencyModel.estimate(table, table, 200), 60 / Math.SQRT2);
  assert.equal(LatencyModel.source(table, table, 200), 'this slot');
  assert.equal(LatencyModel.source({}, table, 200), 'shared');
});
