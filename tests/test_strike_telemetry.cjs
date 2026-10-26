const {test} = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const html = fs.readFileSync(require('node:path').join(__dirname, '../player/midi_player.html'), 'utf8');
const slice = (from, to) => html.slice(html.indexOf(from), html.indexOf(to));
function context() {
  const c = vm.createContext({
    S: {compEma: {}, compEmaGlobal: {}, compSequence: {}, compEnabled: true, compDefaultMs: 50},
    LatencyModel: require('../player/latency_model.js')});
  vm.runInContext(slice('const STRIKE_TIMING_RETRIGGERED', '// Default pitch per address slot'), c);
  vm.runInContext(slice('function latencyTables(', '// Fire-and-forget strike'), c);
  return c;
}
const close = (actual, expected) => assert.ok(Math.abs(actual - expected) < 1e-9, `${actual} != ${expected}`);

test('browser bins the reported current even when the host has no prior command', () => {
  const c = context();
  assert.equal(c.updateLatencyEma(0, undefined, {strike_current_ma: -1700,
    trigger_to_impact_ms: 23, flags: 0x100, sequence: 8}), true);
  close(c.compensationMs(0, 1700), 23);
  assert.equal(c.LatencyModel.learned(c.S.compEma[0]).length, 1);
});
test('duplicate, muted, retriggered, and invalid reports never train the browser model', () => {
  const c = context();
  const report = {strike_current_ma: 700, trigger_to_impact_ms: 21, flags: 0x100, sequence: 1};
  c.updateLatencyEma(0, 2500, report);
  assert.equal(c.updateLatencyEma(0, 2500, {...report, trigger_to_impact_ms: 80}), false);
  assert.equal(c.updateLatencyEma(0, 2500, {...report, sequence: 2, flags: 0x300}), false);
  assert.equal(c.updateLatencyEma(0, 2500, {...report, sequence: 3, flags: 0x110}), false);
  assert.equal(c.updateLatencyEma(0, 2500, {...report, sequence: 4, trigger_to_impact_ms: null}), false);
  close(c.compensationMs(0, 700), 21);
  assert.equal(c.S.compSequence[0], 1);
});
test('legacy reports can still use the known prior current', () => {
  const c = context();
  c.updateLatencyEma(1, 2200, {trigger_to_impact_ms: 35, flags: 0x100, sequence: 0});
  close(c.compensationMs(1, 2200), 35);
});
test('the 0.1 ms impact time is preferred when reported', () => {
  const c = context();
  c.updateLatencyEma(2, 300, {trigger_to_impact_ms: 21, trigger_to_impact_ms_precise: 21.4,
    flags: 0x100, sequence: 5});
  close(c.compensationMs(2, 300), 21.4);
  close(c.compensationMs(3, 300), 21.4);  // shared curve for an unseen slot
});
test('the server tables are preferred and each ring uses its own shared curve', () => {
  const c = context();
  c.updateLatencyEma(3, 800, {trigger_to_impact_ms: 99, flags: 0x100, sequence: 1});
  close(c.compensationMs(3, 800), 99);  // browser-only fallback
  c.S.serverLatency = {offsets: [0, 15], shared: [{20: 30}, {20: 60}], slots: {3: {20: 28}}};
  close(c.compensationMs(3, 800), 28);   // server slot table
  close(c.compensationMs(5, 800), 30);   // ring A shared curve
  close(c.compensationMs(16, 800), 60);  // ring B shared curve
});
