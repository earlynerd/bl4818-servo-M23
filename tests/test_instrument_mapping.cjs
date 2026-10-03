const { test } = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

const html = fs.readFileSync(path.join(__dirname, '../player/midi_player.html'), 'utf8');
const script = [...html.matchAll(/<script>([\s\S]*?)<\/script>/g)].at(-1)[1];
const storage = new Map();
const elements = new Map();
function element(id) {
  if (!elements.has(id)) elements.set(id, {
    value: '', disabled: false, classList: { toggle() {} },
  });
  return elements.get(id);
}
const context = vm.createContext({
  document: { addEventListener() {}, getElementById: element },
  window: {},
  localStorage: {
    getItem(key) { return storage.get(key) ?? null; },
    setItem(key, value) { storage.set(key, value); },
  },
  MidiTranspose: require('../player/midi_transpose.js'),
});

test('creating an instrument copies current trims and routes', async () => {
  element('instrumentName').value = 'Handpan';
  element('currentSlider').value = '950';
  vm.runInContext(`
    S.count = 2;
    S.activeInstrumentId = 'default';
    S.mapping = [60, 64];
    S.trim = [0.75, 1.25];
    S.fallback = {70: 1};
    S.velFloor = 0.35;
    S.compEnabled = false;
    S.compDefaultMs = 63;
    api = async (_method, _path, body) => {
      globalThis.createdSettings = body.settings;
      return ({
      active: 'handpan-id',
      profiles: [{id: 'default', name: 'Default', count: 2},
                 {id: 'handpan-id', name: 'Handpan', count: 2}],
      mapping: [60, 64],
      });
    };
    renderInstrumentControls = () => {};
    renderMapTable = () => {};
  `, context);
  await vm.runInContext('changeInstrument("create")', context);
  assert.deepEqual(JSON.parse(storage.get('robotdrum.trim.v1.handpan-id')), [0.75, 1.25]);
  assert.deepEqual(JSON.parse(storage.get('robotdrum.fallback.v1.handpan-id')), {70: 1});
  assert.equal(storage.get('robotdrum.velfloor.v1.handpan-id'), '0.35');
  assert.deepEqual(JSON.parse(storage.get('robotdrum.comp.v1.handpan-id')),
    {enabled: false, defaultMs: 63});
  assert.equal(storage.get('robotdrum.current.v1.handpan-id'), '950');
  assert.deepEqual(Array.from(vm.runInContext('createdSettings.trim', context)), [0.75, 1.25]);
  assert.equal(vm.runInContext('createdSettings.current_ma', context), 950);
  assert.deepEqual(Array.from(vm.runInContext('S.trim', context)), [0.75, 1.25]);
});

test('an existing new profile recovers trims from the old shared key', () => {
  storage.set('robotdrum.trim.v1', JSON.stringify([0.9, 1.1]));
  vm.runInContext("S.activeInstrumentId = 'older-new-profile'", context);
  assert.deepEqual(Array.from(vm.runInContext('loadTrim()', context)), [0.9, 1.1]);
  assert.equal(storage.get('robotdrum.trim.v1.older-new-profile'), '[0.9,1.1]');
});

test('loading an instrument restores its playing settings', async () => {
  element('instrumentSelect').value = 'default';
  vm.runInContext(`
    S.activeInstrumentId = 'handpan-id';
    S.instruments = [{id: 'default', name: 'Default', count: 2}];
    api = async () => ({
      active: 'default',
      profiles: [{id: 'default', name: 'Default', count: 2}],
      mapping: [52, 55],
      settings: {trim: [1.1, 0.8], fallback: {'70': 0},
                 vel_floor: 0.6, comp_enabled: true,
                 comp_default_ms: 42, current_ma: 1100},
    });
  `, context);
  await vm.runInContext('changeInstrument("select")', context);
  assert.deepEqual(Array.from(vm.runInContext('S.trim', context)), [1.1, 0.8]);
  assert.equal(vm.runInContext('S.velFloor', context), 0.6);
  assert.equal(vm.runInContext('S.compDefaultMs', context), 42);
  assert.equal(element('currentSlider').value, '1100');
});
vm.runInContext(script, context);

test('saved extra mallets and fallbacks cannot route to absent actuators', () => {
  vm.runInContext(`
    S.count = 2;
    S.mapping = [60, 64, 67];
    S.fallback = {70: 2, 71: 1};
  `, context);
  assert.equal(vm.runInContext('pitchToSlot(67)', context), -1);
  assert.equal(vm.runInContext('pitchToSlot(70)', context), -1);
  assert.equal(vm.runInContext('pitchToSlot(71)', context), 1);
  assert.equal(vm.runInContext('pitchIsFallback(67)', context), false);
  assert.deepEqual(Array.from(vm.runInContext('pitchAscendingSlots()', context), x => x.slot), [0, 1]);
  const schedule = vm.runInContext(`buildSchedule([
    {pitch: 67, timeMs: 0, vel: 90},
    {pitch: 71, timeMs: 100, vel: 90}
  ])`, context);
  assert.deepEqual(Array.from(schedule, x => x.slot), [-1, 1]);
});
