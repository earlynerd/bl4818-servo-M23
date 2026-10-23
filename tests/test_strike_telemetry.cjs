const {test} = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const html = fs.readFileSync(require('node:path').join(__dirname, '../player/midi_player.html'), 'utf8');
function context() {
  const c = vm.createContext({S: {compEma: {}, compEmaGlobal: {}, compSequence: {}},
    COMP_EMA_ALPHA: 0.3,
    currentBucket: ma => Number.isFinite(ma) ? Math.min(5, Math.floor(Math.abs(ma) / 500)) : -1});
  vm.runInContext(html.slice(html.indexOf('function updateLatencyEma('),
    html.indexOf('// Fire-and-forget strike')), c);
  return c;
}
test('browser bins the reported current even when the host has no prior command', () => {
  const c = context();
  assert.equal(c.updateLatencyEma(0, undefined, {strike_current_ma: -1700,
    trigger_to_impact_ms: 23, flags: 0x100, sequence: 8}), true);
  assert.equal(c.S.compEma[0][3], 23);
  assert.equal(c.S.compEma[0][0], undefined);
});
test('duplicate, muted, and invalid reports never train the browser model', () => {
  const c = context();
  const report = {strike_current_ma: 700, trigger_to_impact_ms: 21, flags: 0x100, sequence: 1};
  c.updateLatencyEma(0, 2500, report);
  assert.equal(c.updateLatencyEma(0, 2500, {...report, trigger_to_impact_ms: 80}), false);
  c.updateLatencyEma(0, 2500, {...report, sequence: 2, flags: 0x300});
  c.updateLatencyEma(0, 2500, {...report, sequence: 3, trigger_to_impact_ms: null});
  assert.equal(c.S.compEma[0][1], 21);
  assert.equal(c.S.compSequence[0], 1);
});
test('legacy reports can still use the known prior current', () => {
  const c = context();
  c.updateLatencyEma(1, 2200, {trigger_to_impact_ms: 35, flags: 0x100, sequence: 0});
  assert.equal(c.S.compEma[1][4], 35);
});
