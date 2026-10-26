const { test } = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

function page(filename) {
  const nodes = new Map();
  const element = (tagName = '') => ({
    tagName,
    style: {}, dataset: {}, value: '', textContent: '', innerHTML: '', children: [], listeners: {},
    classList: { add() {}, remove() {}, toggle() {} },
    setAttribute() {}, append(...children) { this.children.push(...children); },
    replaceChildren(...children) { this.children = children; },
    addEventListener(event, handler) { this.listeners[event] = handler; },
    querySelectorAll(selector) {
      return this.children.flatMap(child => [
        ...(child.tagName === selector ? [child] : []), ...child.querySelectorAll(selector),
      ]);
    },
  });
  const node = id => {
    if (!nodes.has(id)) nodes.set(id, element());
    return nodes.get(id);
  };
  const storage = new Map([
    ['robotdrum.fallback.v1', '{"60":0}'],
    ['robotdrum.trim.v1', '[2,2]'],
  ]);
  const context = vm.createContext({
    document: { getElementById: node, createElement: element, addEventListener() {}, querySelectorAll() { return []; } },
    window: { addEventListener() {} }, console, setInterval, clearInterval, setTimeout,
    localStorage: { getItem: key => storage.get(key) ?? null,
                    setItem: (key, value) => storage.set(key, value) },
    MidiTranspose: require('../player/midi_transpose.js'),
    LatencyModel: require('../player/latency_model.js'),
  });
  const html = fs.readFileSync(path.join(__dirname, '../player', filename), 'utf8');
  vm.runInContext(html.match(/<script>([\s\S]*?)<\/script>/)[1], context);
  const run = code => vm.runInContext(code, context);
  run('log = () => {};');
  return { run, node, storage };
}

test('player starts new ring layouts unmapped and does not inherit old trim/fallback', async () => {
  const p = page('midi_player.html');
  p.run('S.trim = [2,2]; S.fallback = {60: 0}; api = async () => ({context: "fleet-a", mapping: Array(24).fill(null)});');
  const mapping = await p.run('fetchServerMapping()');
  assert.equal(mapping.length, 24);
  assert.ok(mapping.every(pitch => pitch === null));
  assert.equal(p.run('S.trim.length'), 0);
  assert.equal(p.run('Object.keys(S.fallback).length'), 0);
  p.storage.set('robotdrum.trim.v1.fleet-a', '[1.25]');
  await p.run('fetchServerMapping()');
  assert.equal(p.run('S.trim[0]'), 1.25);
});

test('ring profile selectors load existing drums and apply shared settings and local trims', async () => {
  const p = page('midi_player.html');
  p.run(`
    S.count = 20;
    S.instruments = [{id:'j',name:'Jameson Drum',count:10},{id:'r',name:'retuned_drum',count:10}];
    S.instrumentRings = [{name:'jameson',port:'COM29',count:10,profile_id:'j'},
                        {name:'retuned',port:'COM48',count:10,profile_id:null}];
    S.activeInstrumentId = 'selection-a';
    renderKeyboardPanel = () => {};
    showError = error => { throw new Error(error); };
    api = async (method, path, body) => {
      globalThis.selectionRequest = {method,path,body};
      return {active:'selection-b', profiles:S.instruments,
        rings:S.instrumentRings.map(r=>({...r,profile_id:r.name==='retuned'?'r':'j'})),
        mapping:Array.from({length:20},(_,i)=>50+i),
        settings:{trim:Array(10).fill(1.25).concat(Array(10).fill(0.75)),
                  fallback:{40:19},current_ma:250,vel_floor:0.1,comp_enabled:true,comp_default_ms:37}};
    };
    renderInstrumentControls();
  `);
  assert.equal(p.node('singleInstrumentControls').style.display, 'none');
  const rows = p.node('ringInstrumentControls').children;
  assert.equal(rows.length, 2);
  assert.match(rows[1].children[0].textContent, /COM48/);
  rows[1].children[1].value = 'r';
  await rows[1].children[2].listeners.click();
  const request = JSON.parse(p.run('JSON.stringify(selectionRequest)'));
  assert.deepEqual(request.body, {action:'select_ring', ring:'retuned',id:'r',expected_id:'selection-a'});
  assert.equal(p.run('S.activeInstrumentId'), 'selection-b');
  assert.equal(p.run('S.mapping[19]'), 69);
  assert.equal(p.run('S.trim[0]'), 1.25);
  assert.equal(p.run('S.trim[19]'), 0.75);
  assert.equal(p.node('currentSlider').value, '250');
  assert.equal(p.run('S.velFloor'), 0.1);
});

test('player renders ring/local address while retaining global slot controls', () => {
  const p = page('midi_player.html');
  p.run('S.count = 24; S.mapping = Array(24).fill(null); S.slots = Array.from({length:24}, (_,i) => ({ring:i<14?"pan":"drum",port:i<14?"COM7":"COM8",local_address:i<14?i:i-14})); renderKeyboardPanel = () => {}; renderMapTable();');
  const html = p.node('mapTableBody').innerHTML;
  assert.match(html, /drum:0/);
  assert.match(html, /drum:9/);
  assert.match(html, /data-test="23"/);
  assert.match(html, /title="COM8"/);
});

test('firmware update button remains disabled when the server reports updates unavailable', () => {
  const p = page('midi_player.html');
  p.run('refreshTransposeControls = () => {}; renderFirmware({phase:"ready", update_supported:false, artifact:{image_size:100, image_crc32_hex:"0x1234",image_version:1,built_at:"now"}});');
  assert.equal(p.node('firmwareUpdateBtn').disabled, true);
});

function firmwarePage() {
  const p = page('midi_player.html');
  p.run(`
    S.count = 24;
    refreshTransposeControls = () => {};
    globalThis.firmwareReady = {phase:'ready',busy:false,update_supported:true,
      rings:[{name:'pan',port:'COM7',expected_count:14},{name:'drum',port:'COM8',expected_count:10}],
      artifact:{image_size:100,image_crc32:1234,image_crc32_hex:'000004D2',image_version:1,built_at:'now'}};
    renderFirmware(firmwareReady);
  `);
  return p;
}

test('firmware ring selection survives polling, disables empty selections and locks while busy', () => {
  const p = firmwarePage();
  const container = p.node('firmwareRingSelection');
  assert.equal(container.children.length, 3);
  assert.match(container.children[2].children[1].textContent, /drum.*COM8.*10/);
  const inputs = container.querySelectorAll('input');
  assert.ok(inputs.every(input => input.checked));
  assert.equal(p.node('firmwareUpdateBtn').disabled, false);
  inputs[0].checked = false;
  inputs[0].listeners.change();
  p.run('renderFirmware(firmwareReady);');
  assert.equal(container.querySelectorAll('input')[0], inputs[0]);
  assert.equal(inputs[0].checked, false);
  assert.equal(p.run('S.firmwareRingSelection.join()'), 'drum');
  inputs[1].checked = false;
  inputs[1].listeners.change();
  assert.equal(p.node('firmwareUpdateBtn').disabled, true);
  inputs[1].checked = true;
  inputs[1].listeners.change();
  p.run(`renderFirmware({...firmwareReady,busy:true,operation:'update',phase:'writing',
    message:'drum (1/1): pages',progress:{done:1,total:2,ring:'drum'},
    targets:[{name:'drum',state:'updating'}]});`);
  assert.ok(inputs.every(input => input.disabled));
  assert.equal(p.node('firmwareUpdateBtn').disabled, true);
  assert.equal(p.node('firmwareTargets').textContent, 'drum: updating');
  assert.equal(p.node('firmwareMessage').textContent, 'drum (1/1): pages');
});

test('firmware confirmation and request include only selected rings', async () => {
  const p = firmwarePage();
  const first = p.node('firmwareRingSelection').querySelectorAll('input')[0];
  first.checked = false;
  first.listeners.change();
  p.run(`
    window.confirm = message => {globalThis.confirmation = message; return true;};
    api = async (method,path,body) => {globalThis.updateRequest = {method,path,body};
      return {...firmwareReady,busy:true,operation:'update',targets:[{name:'drum',state:'pending'}]};};
  `);
  await p.run('startFirmwareUpdate()');
  const request = JSON.parse(p.run('JSON.stringify(updateRequest)'));
  assert.deepEqual(request, {method:'POST',path:'/api/firmware/update',
    body:{confirm:true,expected_crc32:1234,rings:['drum']}});
  const confirmation = p.run('confirmation');
  assert.match(confirmation, /10 actuator/);
  assert.match(confirmation, /drum \(COM8\)/);
  assert.doesNotMatch(confirmation, /pan|COM7/);
  assert.match(confirmation, /one after the other/);
});

test('firmware cancellation sends no update and single-ring requests keep their original shape', async () => {
  const p = firmwarePage();
  p.run(`window.confirm = () => false; api = async () => {throw new Error('unexpected update');};`);
  await p.run('startFirmwareUpdate()');
  p.run(`
    renderFirmware({...firmwareReady,rings:[]});
    window.confirm = () => true;
    api = async (method,path,body) => {globalThis.updateBody = body; return {...firmwareReady,rings:[]};};
  `);
  await p.run('startFirmwareUpdate()');
  assert.deepEqual(JSON.parse(p.run('JSON.stringify(updateBody)')), {confirm:true,expected_crc32:1234});
  assert.equal(p.node('firmwareRingSelection').style.display, 'none');
});

test('looper shares ring-specific settings and renders ring labels', async () => {
  const p = page('looper.html');
  p.run('api = async () => ({context: "fleet-a", mapping: Array(24).fill(null)});');
  const mapping = await p.run('fetchServerMapping()');
  assert.equal(mapping.length, 24);
  assert.equal(p.run('loadTrim().length'), 0);
  assert.equal(p.run('Object.keys(loadFallback()).length'), 0);
  p.run('LP.count=1; LP.mapping=[60]; LP.homed=[true]; LP.slots=[{ring:"drum",local_address:0}]; renderMappingSummary();');
  assert.match(p.node('mapSummary').innerHTML, /drum:0/);
});

test('strike popup reports the server compensation, residual and last strike', async () => {
  const p = page('midi_player.html');
  p.node('currentSlider').value = '800';
  p.node('cfgTarget').options = [{}, {}];
  p.run(`
    refreshCfgModalValues = () => {}; loadCfgConfig = async () => {};
    S.count = 2; S.mapping = [60, 62]; S.compEnabled = true;
    api = async (method, path) => ({
      '/api/latency': {offsets: [0], shared: [{20: 30}], slots: {1: {16: 41.2, 20: 32.0}}},
      '/api/strike-timing': {slots: [null, {address: 1, sequence: 7, flags: 0x1C3,
        trigger_to_coast_ms: 15, trigger_to_impact_ms: 18, trigger_to_impact_ms_precise: 17.5,
        trigger_to_rebound_ms: 21, estimated_strike_velocity_dps: 1930}]},
      '/api/play': {playing: false, timing_stats: {},
        residual_stats: {1: {last: 1.5, sum: 3, sumSq: 9, n: 2, maxAbs: 2.5}}},
    })[path];
    openCfgModal(1);
  `);
  await new Promise(resolve => setTimeout(resolve, 20));
  try {
    assert.match(p.node('cfgCompApplied').textContent, /^32\.0 ms @ 800 mA · this slot \(2\/29 bins\)$/);
    assert.match(p.node('cfgResidual').textContent, /^last \+1\.50 ms · mean \+1\.50/);
    assert.equal(p.node('cfgTimingSeq').textContent, 'seq #7');
    assert.equal(p.node('cfgTimingImpact').textContent, '17.5 ms');
  } finally {
    p.run('closeCfgModal()');
  }
});

test('piano roll selects a mallet and trims it in step with the mapping table', () => {
  const p = page('midi_player.html');
  p.node('currentSlider').value = '800';
  p.node('mapTable').querySelector = () => null;
  p.run(`
    log = () => {}; flushInstrumentSettings = async () => {}; saveTrim = () => {};
    S.count = 2; S.mapping = [60, 62]; S.trim = [1.1, 1.0]; S.velFloor = 0.5;
    S.slots = [{ring: '15B', local_address: 0}, {}];
    S.composition = {originalEvents: [], maxMs: 1000, visibleVoices: null, events: [
      {pitch: 60, timeMs: 0, durMs: 200, vel: 100, slot: 0, track: 0, channel: 0},
      {pitch: 62, timeMs: 300, durMs: 200, vel: 90, slot: 1, track: 0, channel: 0}]};
    selectRollMallet(0);
  `);
  const labels = p.node('pianoLabels').innerHTML, grid = p.node('pianoGrid').innerHTML;
  assert.match(labels, /class="piano-label selected" data-slot="0"[^>]*>.*?<span class="trim">110%<\/span>/);
  assert.match(labels, /data-slot="1"[^>]*>.*?<span class="trim">100%<\/span>/);
  assert.match(grid, /roll-canvas has-selection/);
  assert.match(grid, /class="note-bar selected" data-slot="0"/);
  assert.match(grid, /class="note-bar" data-slot="1"/);
  assert.equal(p.node('rollTrimBar').hidden, false);
  assert.equal(p.node('rollTrimName').textContent, 'C4 · addr 0 (15B:0)');

  p.run('setSlotTrim(0, 87)');  // snaps to the table's 5% steps
  assert.equal(p.run('S.trim[0]'), 0.85);
  assert.match(p.node('rollTrimValue').textContent, /^85% · \d+ mA at full velocity$/);
  p.run('setSlotTrim(1, 500)');  // clamped to the table's range
  assert.equal(p.run('S.trim[1]'), 2);

  p.run('selectRollMallet(-1)');
  assert.equal(p.node('rollTrimBar').hidden, true);
  assert.doesNotMatch(p.node('pianoGrid').innerHTML, /has-selection/);
});

test('trim changes during playback are sent live as a ratio to the scheduled trim', () => {
  const p = page('midi_player.html');
  p.node('mapTable').querySelector = () => null;
  p.run(`
    saveTrim = () => {}; sent = [];
    fetch = (url, opts) => { sent.push([url, JSON.parse(opts.body)]); return Promise.resolve(); };
    S.count = 2; S.trim = [1.0, 1.2];
    setSlotTrim(1, 110);                         // not playing: nothing sent
    S.playing = true; S.scheduleMasterMa = 800; S.scheduleTrim = [1.0, 1.2];
    setSlotTrim(1, 90);
  `);
  assert.equal(p.run('sent.length'), 1);
  assert.equal(p.run('sent[0][0]'), '/api/play/update');
  assert.equal(p.run('Object.keys(sent[0][1].trim_scale).join()'), '1');
  assert.ok(Math.abs(p.run('sent[0][1].trim_scale[1]') - 0.75) < 1e-9);
});
