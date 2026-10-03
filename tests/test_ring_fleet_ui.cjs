const { test } = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

function page(filename) {
  const nodes = new Map();
  const element = () => ({
    style: {}, dataset: {}, value: '', textContent: '', innerHTML: '', children: [], listeners: {},
    classList: { add() {}, remove() {}, toggle() {} },
    setAttribute() {}, append(...children) { this.children.push(...children); },
    replaceChildren(...children) { this.children = children; },
    addEventListener(event, handler) { this.listeners[event] = handler; },
    querySelectorAll() { return []; },
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
    window: { addEventListener() {} }, console,
    localStorage: { getItem: key => storage.get(key) ?? null,
                    setItem: (key, value) => storage.set(key, value) },
    MidiTranspose: require('../player/midi_transpose.js'),
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

test('firmware update button remains disabled for a built image in multi-ring mode', () => {
  const p = page('midi_player.html');
  p.run('refreshTransposeControls = () => {}; renderFirmware({phase:"ready", update_supported:false, artifact:{image_size:100, image_crc32_hex:"0x1234",image_version:1,built_at:"now"}});');
  assert.equal(p.node('firmwareUpdateBtn').disabled, true);
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
