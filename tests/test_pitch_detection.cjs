// node --test tests/test_pitch_detection.cjs
const { test } = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const detector = require('../player/pitch_detector.js');

function tone(frequency, rate = 48000, harmonics = [1], noise = 0) {
  let seed = 37;
  return Float32Array.from({ length: rate > 60000 ? 16384 : 8192 }, (_, i) => {
    seed = (Math.imul(seed, 1664525) + 1013904223) >>> 0;
    const t = i / rate;
    return 0.15 * Math.exp(-t * 2) * harmonics.reduce((sum, amplitude, h) =>
      sum + amplitude * Math.sin(2 * Math.PI * frequency * (h + 1) * t + h * 0.3), 0) + noise * (seed / 2 ** 32 - 0.5);
  });
}

test('chromatic drum range at common device sample rates, including detuning', () => {
  for (const rate of [44100, 48000, 96000]) {
    for (let midi = 40; midi <= 86; midi++) {
      const frequency = 440 * 2 ** ((midi - 69 + 0.12) / 12);
      const result = detector.estimate(tone(frequency, rate), rate);
      assert.ok(result, `${midi} at ${rate}`);
      assert.equal(result.midi, midi);
      assert.ok(Math.abs(result.cents - 12) < 8, `${midi}: ${result.cents} cents`);
    }
  }
});

test('dominant upper harmonics do not automatically become the assigned octave', () => {
  for (const f of [130.81, 164.81, 220, 329.63, 523.25]) {
    const result = detector.estimate(tone(f, 48000, [0.45, 1, 0.5], 0.006), 48000);
    assert.ok(result);
    assert.ok(Math.abs(1200 * Math.log2(result.frequency / f)) < 8, `${f}: ${result.frequency}`);
  }
});

test('silence, DC, low signal, broadband noise, and clipping cannot assign', () => {
  assert.equal(detector.estimate(new Float32Array(8192), 48000), null);
  assert.equal(detector.estimate(new Float32Array(8192).fill(0.1), 48000), null);
  assert.equal(detector.estimate(tone(440).map(v => v * 0.001), 48000), null);
  assert.equal(detector.estimate(tone(440, 48000, [0], 0.2), 48000), null);
  assert.equal(detector.estimate(tone(440).map(v => Math.max(-1, Math.min(1, v * 20))), 48000), null);
});

test('consensus rejects inconsistent octaves and marks between-note results', () => {
  const frame = f => ({ ...detector.note(f), confidence: 0.95 });
  assert.equal(detector.consensus(Array(5).fill(frame(440))), null);
  assert.equal(detector.consensus(Array.from({ length: 10 }, (_, i) => frame(i % 2 ? 440 : 880))), null);
  assert.equal(detector.consensus(Array(8).fill(frame(440))).midi, 69);
  assert.equal(detector.consensus(Array(8).fill(frame(440 * 2 ** (45 / 1200)))).assignable, false);
});

function workflow() {
  const nodes = new Map();
  const node = id => {
    if (!nodes.has(id)) nodes.set(id, { value: id === 'pitchSlot' ? '0' : '', textContent: '', disabled: false,
      selectedOptions: [{ textContent: '' }], addEventListener() {} });
    return nodes.get(id);
  };
  const requests = [];
  const context = vm.createContext({ console, AbortController, DOMException, setTimeout, clearTimeout,
    S: { pitchOpen: true, mapping: [60, 62], count: 2, homed: [true, true], slots: [{}, {}] },
    API_BASE: '', STORAGE_KEY: 'map', navigator: {},
    document: { getElementById: node, addEventListener() {} }, window: { addEventListener() {} },
    localStorage: { setItem() {} }, renderMapTable() {}, showComposition() {}, log() {},
    effectiveCurrentForSlot: () => 800,
    PitchDetector: { listen: async options => { await options.onReady(); return { ...detector.note(329.63), frames: 6, assignable: true }; } },
    fetch: async (url, options) => {
      requests.push({ url, options });
      return { ok: true, json: async () => url === '/api/play' ? { playing: false } :
        url === '/api/strike' ? { accepted: true } : { ok: true, mapping: JSON.parse(options.body).mapping } };
    },
  });
  vm.runInContext(fs.readFileSync(require.resolve('../player/pitch_assignment.js'), 'utf8'), context);
  return { context, node, requests, run: code => vm.runInContext(code, context) };
}

test('detection targets the selected slot and persistence waits for explicit assignment', async () => {
  const p = workflow();
  await p.run('startPitchDetection(true)');
  assert.deepEqual(p.requests.map(r => r.url), ['/api/play', '/api/strike']);
  assert.deepEqual(JSON.parse(p.requests[1].options.body), { address: 0, current_ma: 800 });
  assert.equal(p.run('S.mapping[0]'), 60);
  assert.equal(p.node('pitchApply').disabled, false);
  await p.run('applyDetectedPitch()');
  assert.equal(p.run('S.mapping[0]'), 64);
  assert.equal(p.requests[2].url, '/api/mapping');
  assert.match(p.node('pitchStatus').textContent, /Saved E4/);
});

test('listen only never sends motion; unhomed and busy requests cannot strike', async () => {
  const p = workflow();
  await p.run('startPitchDetection(false)');
  assert.deepEqual(p.requests.map(r => r.url), ['/api/play']);
  p.run('S.homed[0] = false');
  await p.run('startPitchDetection(true)');
  assert.equal(p.requests.length, 1);
  p.run('S.playing = true');
  await p.run('startPitchDetection(false)');
  assert.equal(p.requests.length, 1);
});

test('rejected strikes and playback in another browser do not produce assignments', async () => {
  for (const playback of [true, false]) {
    const p = workflow();
    p.context.fetch = async url => ({ ok: true, json: async () => url === '/api/play'
      ? { playing: playback } : { accepted: false, result_name: 'NOT_HOMED' } });
    await p.run('startPitchDetection(true)');
    assert.equal(p.run('pitchResult'), null);
    assert.equal(p.node('pitchApply').disabled, true);
  }
});

test('cancelling an in-flight detection discards late results', async () => {
  const p = workflow();
  let resolve;
  p.context.PitchDetector.listen = () => new Promise(r => { resolve = r; });
  const pending = p.run('startPitchDetection(false)');
  p.run('cancelPitchDetection()');
  resolve({ ...detector.note(440), assignable: true });
  await pending;
  assert.equal(p.run('pitchResult'), null);
  assert.equal(p.node('pitchApply').disabled, true);
});

test('failed saves preserve the local map and permit retry; changed mappings reject stale results', async () => {
  const p = workflow();
  await p.run('startPitchDetection(false)');
  p.context.fetch = async () => { throw new Error('Disconnected'); };
  await p.run('applyDetectedPitch()');
  assert.equal(p.run('S.mapping[0]'), 60);
  assert.match(p.node('pitchStatus').textContent, /Could not confirm save/);
  assert.equal(p.node('pitchApply').disabled, false);
  p.run('S.mapping[1] = 70');
  await p.run('applyDetectedPitch()');
  assert.equal(p.run('pitchResult'), null);
  assert.match(p.node('pitchStatus').textContent, /mapping changed/);
});

function microphone({ permissionPending = false, sound = true } = {}) {
  let now = 0, tick, expire, stopped = 0, closed = 0, grant;
  const track = { stop() { stopped++; }, addEventListener() {} };
  const stream = { getTracks: () => [track], getAudioTracks: () => [track] };
  const connectable = () => ({ connect() {}, disconnect() {} });
  class Audio {
    state = 'running'; sampleRate = 48000; destination = {};
    get currentTime() { return now / 1000; }
    async resume() {}
    async close() { this.state = 'closed'; closed++; }
    createMediaStreamSource() { return connectable(); }
    createGain() { return { ...connectable(), gain: {} }; }
    createAnalyser() { return { ...connectable(), fftSize: 8192,
      getFloatTimeDomainData(buffer) { buffer.set(sound && now >= 1100 ? tone(329.63) : new Float32Array(8192)); } }; }
  }
  const context = vm.createContext({ console, DOMException, isSecureContext: true, AudioContext: Audio,
    navigator: { mediaDevices: { getUserMedia: () => permissionPending ? new Promise(resolve => { grant = resolve; }) : Promise.resolve(stream) } },
    performance: { now: () => now },
    setInterval: callback => { tick = callback; return 1; }, clearInterval: () => { tick = null; },
    setTimeout: callback => { expire = callback; return 1; }, clearTimeout: () => { expire = null; },
  });
  vm.runInContext(fs.readFileSync(require.resolve('../player/pitch_detector.js'), 'utf8'), context);
  return { detector: context.PitchDetector, stopped: () => stopped, closed: () => closed,
    grant: () => grant(stream), expire: () => expire?.(),
    async advance() {
      for (let i = 0; i < 8; i++) await Promise.resolve();
      for (now = 0; now < 10000 && tick; now += 60) {
        tick();
        for (let i = 0; i < 8; i++) await Promise.resolve();
      }
    },
  };
}

test('capture waits for onset, measures the decay and releases all microphone resources', async () => {
  const mic = microphone();
  let ready = 0;
  const pending = mic.detector.listen({ signal: new AbortController().signal, onReady: async () => { ready++; } });
  await mic.advance();
  const result = await pending;
  assert.equal(result.midi, 64);
  assert.equal(ready, 1);
  assert.equal(mic.stopped(), 1);
  assert.equal(mic.closed(), 1);
});

test('silent capture times out without proposing a pitch and releases the microphone', async () => {
  const mic = microphone({ sound: false });
  const pending = assert.rejects(mic.detector.listen({ signal: new AbortController().signal }), /No clear, stable note/);
  await mic.advance();
  await pending;
  assert.equal(mic.stopped(), 1);
  assert.equal(mic.closed(), 1);
});

test('cancellation while permission is pending closes audio and stops a late-granted stream', async () => {
  const mic = microphone({ permissionPending: true });
  const controller = new AbortController();
  const pending = assert.rejects(mic.detector.listen({ signal: controller.signal }), { name: 'AbortError' });
  controller.abort();
  await pending;
  mic.grant();
  await Promise.resolve();
  assert.equal(mic.stopped(), 1);
  assert.equal(mic.closed(), 1);
});
