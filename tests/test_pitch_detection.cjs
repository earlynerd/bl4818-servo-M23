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
      selectedOptions: [{ textContent: '' }], options: [{ value: '0' }, { value: '1' }], addEventListener() {} });
    return nodes.get(id);
  };
  const requests = [];
  const context = vm.createContext({ console, AbortController, DOMException, setTimeout, clearTimeout,
    S: { pitchOpen: true, mapping: [60, 62], count: 2, homed: [true, true], slots: [{}, {}] },
    API_BASE: '', STORAGE_KEY: 'map', STRIKE_MA_MAX: 3000, navigator: {},
    document: { getElementById: node, addEventListener() {} }, window: { addEventListener() {} },
    localStorage: { setItem() {} }, renderMapTable() {}, showComposition() {}, log() {},
    effectiveCurrentForSlot: () => 800,
    midiName: midi => detector.note(440 * 2 ** ((midi - 69) / 12)).name,
    PitchDetector: { listen: async options => { await options.onReady(); return { ...detector.note(329.63), frames: 6, assignable: true }; } },
    fetch: async (url, options) => {
      requests.push({ url, options });
      return { ok: true, json: async () => url === '/api/play' ? { playing: false } :
        url === '/api/status' ? { count: 2, homed: [true, true], slots: [{ fault: 0 }, { fault: 0 }] } :
        url === '/api/strike' ? { accepted: true } :
        { ok: true, mapping: options.method === 'GET' ? [60, 62] : JSON.parse(options.body).mapping } };
    },
  });
  vm.runInContext(fs.readFileSync(require.resolve('../player/pitch_assignment.js'), 'utf8'), context);
  context.PitchDetector.scan = async options => {
    options.onProgress('Permission requested');
    for (let i = 0; i < options.steps; i++) {
      await options.onPrepare(i);
      await options.onReady(i);
      options.onResult({ ...detector.note(i ? 440 : 329.63), frames: 6, assignable: true }, i);
    }
  };
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

function microphone({ permissionPending = false, sound = true, pulses = false, ambient = () => 0,
                      transform = samples => samples, recoverAudio = false } = {}) {
  let now = 0, tick, expire, stopped = 0, closed = 0, grant, opened = 0, pulse = null, frozenAt = null;
  let audioState = 'running';
  const track = { stop() { stopped++; }, addEventListener() {} };
  const stream = { getTracks: () => [track], getAudioTracks: () => [track] };
  const connectable = () => ({ connect() {}, disconnect() {} });
  class Audio {
    sampleRate = 48000; destination = {};
    get state() { return audioState; }
    set state(value) { audioState = value; }
    get currentTime() { return (frozenAt ?? now) / 1000; }
    async resume() { if (recoverAudio) { frozenAt = null; audioState = 'running'; } }
    async close() { this.state = 'closed'; closed++; }
    createMediaStreamSource() { return connectable(); }
    createGain() { return { ...connectable(), gain: {} }; }
    createAnalyser() { return { ...connectable(), fftSize: 8192,
      getFloatTimeDomainData(buffer) {
        const frequency = pulses ? (pulse && now >= pulse.start && now < pulse.end ? pulse.frequency : 0)
          : (sound && now >= 1100 ? 329.63 : 0);
        const samples = frequency ? tone(frequency) : new Float32Array(8192);
        const noise = tone(100, 48000, [0], ambient(now));
        buffer.set(transform(samples.map((v, i) => v + noise[i]), now));
      } }; }
  }
  const context = vm.createContext({ console, DOMException, isSecureContext: true, AudioContext: Audio,
    navigator: { mediaDevices: { getUserMedia: () => {
      opened++;
      return permissionPending ? new Promise(resolve => { grant = resolve; }) : Promise.resolve(stream);
    } } },
    performance: { now: () => now },
    setInterval: callback => { tick = callback; return 1; }, clearInterval: () => { tick = null; },
    setTimeout: callback => { expire = callback; return 1; }, clearTimeout: () => { expire = null; },
  });
  vm.runInContext(fs.readFileSync(require.resolve('../player/pitch_detector.js'), 'utf8'), context);
  return { detector: context.PitchDetector, stopped: () => stopped, closed: () => closed,
    opened: () => opened, now: () => now,
    freeze: (state = 'running') => { frozenAt = now; audioState = state; },
    strike: (frequency, delay = 120) => { pulse = { frequency, start: now + delay, end: now + delay + 1380 }; },
    grant: () => grant(stream), expire: () => expire?.(),
    async advance(until = 10000) {
      for (let i = 0; i < 8; i++) await Promise.resolve();
      for (; now < until && tick; now += 60) {
        tick();
        for (let i = 0; i < 20; i++) await Promise.resolve();
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

test('full scan strikes each slot once at frozen current and only saves after review', async () => {
  const p = workflow();
  await p.run('startPitchScan()');
  const strikes = p.requests.filter(r => r.url === '/api/strike').map(r => JSON.parse(r.options.body));
  assert.deepEqual(strikes, [{ address: 0, current_ma: 800 }, { address: 1, current_ma: 800 }]);
  assert.equal(p.run('JSON.stringify(S.mapping)'), '[60,62]');
  assert.ok(!p.requests.some(r => r.options.method === 'POST' && r.url === '/api/mapping'));
  await p.run('savePitchScan()');
  assert.equal(p.run('JSON.stringify(S.mapping)'), '[64,69]');
  assert.equal(p.run('pitchScan'), null);
});

test('partial save preserves unresolved slots and extra mapping entries', async () => {
  const p = workflow();
  p.run('S.mapping.push(90)');
  const originalFetch = p.context.fetch;
  p.context.fetch = async (url, options) => url === '/api/mapping' && options.method === 'GET'
    ? { ok: true, json: async () => ({ mapping: [60, 62, 90] }) } : originalFetch(url, options);
  p.context.PitchDetector.scan = async options => {
    await options.onPrepare(0);
    await options.onReady(0);
    options.onResult({ ...detector.note(440), assignable: true }, 0);
    options.onResult({ error: 'Unclear note', assignable: false }, 1);
  };
  await p.run('startPitchScan()');
  await p.run('savePitchScan()');
  assert.equal(p.run('JSON.stringify(S.mapping)'), '[69,62,90]');
});

test('retry unresolved skips already detected notes', async () => {
  const p = workflow();
  await p.run('startPitchScan()');
  p.run('pitchScan.rows[1] = {slot: 1, state: "uncertain", assignable: false}');
  p.requests.length = 0;
  await p.run('startPitchScan(true)');
  assert.deepEqual(p.requests.filter(r => r.url === '/api/strike').map(r => JSON.parse(r.options.body).address), [1]);
  assert.equal(p.run('pitchScan.rows[0].midi'), 64);
});

test('cancellation preserves completed rows but prevents the next strike and late results', async () => {
  const p = workflow();
  let resume;
  p.context.PitchDetector.scan = async options => {
    await options.onPrepare(0);
    await options.onReady(0);
    options.onResult({ ...detector.note(440), assignable: true }, 0);
    await new Promise(resolve => { resume = resolve; });
    await options.onPrepare(1);
    await options.onReady(1);
    options.onResult({ ...detector.note(330), assignable: true }, 1);
  };
  const running = p.run('startPitchScan()');
  for (let i = 0; i < 30 && !resume; i++) await Promise.resolve();
  assert.ok(resume);
  p.run('cancelPitchDetection()');
  resume();
  await running;
  assert.equal(p.requests.filter(r => r.url === '/api/strike').length, 1);
  assert.equal(p.run('pitchScan.rows[0].midi'), 69);
  assert.equal(p.run('pitchScan.rows[1].assignable'), false);
});

test('unavailable, unhomed, faulted or changed instruments stop without skipping forward', async () => {
  for (const status of [
    { count: 2, homed: [false, true], slots: [{}, {}] },
    { count: 2, homed: [true, true], slots: [{ fault: 2 }, {}] },
    { count: 2, homed: [true, true], slots: [{ status_cached: true }, {}] },
    { count: 3, homed: [true, true, true], slots: [{}, {}, {}] },
  ]) {
    const p = workflow(), originalFetch = p.context.fetch;
    p.context.fetch = async (url, options) => url === '/api/status'
      ? { ok: true, json: async () => status } : originalFetch(url, options);
    await p.run('startPitchScan()');
    assert.equal(p.requests.filter(r => r.url === '/api/strike').length, 0);
    assert.match(p.node('pitchStatus').textContent, /Scan stopped/);
    assert.doesNotMatch(p.run('pitchScan.rows[0].error'), /Stopped before completion/);
    assert.match(p.run('pitchScan.rows[0].error'), /mallet|mallets|status unavailable/);
  }
});

test('save rejects a remotely changed map and a failed write leaves local assignments intact', async () => {
  for (const remoteChanged of [true, false]) {
    const p = workflow();
    await p.run('startPitchScan()');
    const originalFetch = p.context.fetch;
    p.context.fetch = async (url, options) => {
      if (url === '/api/mapping') {
        if (options.method === 'GET') return { ok: true, json: async () => ({ mapping: remoteChanged ? [80, 81] : [60, 62] }) };
        throw new Error('Offline');
      }
      return originalFetch(url, options);
    };
    await p.run('savePitchScan()');
    assert.equal(p.run('JSON.stringify(S.mapping)'), '[60,62]');
    assert.notEqual(p.run('pitchScan'), null);
    assert.match(p.node('pitchStatus').textContent, /Could not confirm save/);
  }
});

test('sequence reuses one microphone with short bounded pauses between distinct pitches', async () => {
  const mic = microphone({ pulses: true });
  const frequencies = [220, 329.63, 523.25], results = [], strikes = [];
  const pending = mic.detector.scan({ signal: new AbortController().signal, steps: 3,
    onReady: async index => { strikes.push(mic.now()); mic.strike(frequencies[index]); },
    onResult: result => results.push(result),
  });
  await mic.advance(15000);
  await pending;
  assert.deepEqual(results.map(result => result.midi), [57, 64, 72]);
  assert.equal(mic.opened(), 1);
  assert.equal(mic.stopped(), 1);
  assert.equal(mic.closed(), 1);
  for (let i = 1; i < strikes.length; i++) {
    assert.ok(strikes[i] - strikes[i - 1] >= 1200);
    assert.ok(strikes[i] - strikes[i - 1] <= 2400);
  }
});

test('persistent ringing does not stall the scan or become the next mallet pitch', async () => {
  const mic = microphone();
  const strikes = [], results = [];
  const pending = mic.detector.scan({ signal: new AbortController().signal, steps: 2, acousticRetries: 1,
    onReady: async (index, attempt) => { strikes.push([index, attempt, mic.now()]); },
    onResult: result => results.push(result),
  });
  await mic.advance(20000);
  await pending;
  assert.deepEqual(strikes.map(s => s.slice(0, 2)), [[0, 0], [1, 0], [1, 1]]);
  assert.ok(strikes[1][2] - strikes[0][2] < 2500);
  assert.equal(results[0].midi, 64);
  assert.equal(results[1].assignable, false);
  assert.match(results[1].error, /No new strike/);
  assert.equal(mic.stopped(), 1);
});

test('a silent mallet exhausts its retries before the next mallet is measured', async () => {
    const mic = microphone({ pulses: true }), results = [];
    const strikes = [];
    const pending = mic.detector.scan({ signal: new AbortController().signal, steps: 2, acousticRetries: 1,
      onReady: async (index, attempt) => {
        strikes.push([index, attempt]);
        if (index === 1) mic.strike(440);
      }, onResult: result => results.push(result),
    });
    await mic.advance(20000);
    await pending;
    assert.equal(results[0].assignable, false);
    assert.equal(results[1].midi, 69);
    assert.equal(mic.opened(), 1);
    assert.deepEqual(strikes, [[0, 0], [0, 1], [1, 0]]);
});

test('missing strike acknowledgement halts the scan instead of issuing another strike', async () => {
  const mic = microphone({ sound: false });
  let strikes = 0;
  const pending = assert.rejects(mic.detector.scan({ signal: new AbortController().signal, steps: 2, acousticRetries: 1,
    onReady: () => { strikes++; return new Promise(() => {}); },
  }), /acknowledgment timed out/);
  await mic.advance(20000);
  await pending;
  assert.equal(strikes, 1);
  assert.equal(mic.stopped(), 1);
});

test('recent cached status is usable and an old cache is rechecked on the same mallet', async () => {
  for (const initiallyStale of [false, true]) {
    const p = workflow(), originalFetch = p.context.fetch;
    let reads = 0;
    p.context.fetch = async (url, options) => {
      if (url !== '/api/status') return originalFetch(url, options);
      reads++;
      return { ok: true, json: async () => ({ count: 2, homed: [true, true],
        slots: [0, 1].map(() => ({ fault: 0, status_cached: true,
          status_age_ms: initiallyStale && reads === 1 ? 5000 : 100 })) }) };
    };
    await p.run('startPitchScan()');
    assert.deepEqual(p.requests.filter(r => r.url === '/api/strike').map(r => JSON.parse(r.options.body).address), [0, 1]);
    assert.equal(reads, initiallyStale ? 3 : 2);
  }
});

test('silent retry current is temporary and bounded by percentage, increment and absolute limit', async () => {
  for (const [base, expected] of [[0, 0], [800, 960], [2000, 2200], [2950, 3000]]) {
    const p = workflow();
    p.context.effectiveCurrentForSlot = () => base;
    p.context.PitchDetector.scan = async options => {
      assert.equal(options.acousticRetries, 1);
      for (const attempt of [0, 1]) {
        await options.onPrepare(0, attempt);
        await options.onReady(0, attempt);
      }
    };
    await p.run('startPitchScan()');
    assert.deepEqual(p.requests.filter(r => r.url === '/api/strike').map(r => JSON.parse(r.options.body).current_ma), [base, expected]);
    assert.equal(p.context.effectiveCurrentForSlot(0), base);
  }
});

test('silent first strike is retried on the same slot and can yield a pitch', async () => {
  const mic = microphone({ pulses: true }), attempts = [];
  const pending = mic.detector.scan({ signal: new AbortController().signal, steps: 1, acousticRetries: 1,
    onReady: async (index, attempt) => { attempts.push(attempt); if (attempt) mic.strike(440); },
  });
  await mic.advance();
  const results = await pending;
  assert.deepEqual(attempts, [0, 1]);
  assert.equal(results[0].midi, 69);
});

test('overloaded audio requests a retry at the original current', async () => {
  const p = workflow();
  p.context.PitchDetector.scan = async options => {
    await options.onPrepare(0, 1);
    await options.onReady(0, 1, false);
  };
  await p.run('startPitchScan()');
  const strikes = p.requests.filter(r => r.url === '/api/strike').map(r => JSON.parse(r.options.body));
  assert.deepEqual(strikes, [{ address: 0, current_ma: 800 }]);
});

test('steady city noise and an increased background between mallets do not stall mapping', async () => {
  const mic = microphone({ pulses: true, ambient: now => now < 1500 ? 0.015 : 0.035 });
  const strikes = [];
  const pending = mic.detector.scan({ signal: new AbortController().signal, steps: 3, acousticRetries: 1,
    onReady: async (index, attempt) => {
      strikes.push([index, attempt, mic.now()]);
      mic.strike([220, 329.63, 440][index]);
    },
  });
  await mic.advance(15000);
  const results = await pending;
  assert.deepEqual(Array.from(results, r => r.midi), [57, 64, 69]);
  assert.equal(strikes.length, 3);
  assert.ok(strikes[1][2] - strikes[0][2] <= 2400);
});

test('a delayed impact arriving at the silent deadline is measured without a retry', async () => {
  const mic = microphone({ pulses: true });
  let strikes = 0;
  const pending = mic.detector.scan({ signal: new AbortController().signal, steps: 1, acousticRetries: 1,
    onReady: async () => { strikes++; mic.strike(440, 1800); },
  });
  await mic.advance();
  const results = await pending;
  assert.equal(strikes, 1);
  assert.equal(results[0].midi, 69);
});

test('noise-triggered unclear audio and clipping retry before advancing unresolved', async () => {
  for (const clipped of [false, true]) {
    const mic = microphone({ pulses: true, transform: (samples, now) => now < 1100 ? samples :
      clipped ? samples.map(v => Math.max(-1, Math.min(1, v * 50))) : tone(440, 48000, [0], 0.6) });
    const strikes = [];
    const pending = mic.detector.scan({ signal: new AbortController().signal, steps: 2, acousticRetries: 1,
      onReady: async (index, attempt, stronger) => { strikes.push([index, attempt, stronger]); mic.strike(440); },
    });
    await mic.advance(20000);
    const results = await pending;
    assert.deepEqual(strikes.map(s => s.slice(0, 2)), [[0, 0], [0, 1], [1, 0], [1, 1]]);
    assert.equal(strikes[1][2], !clipped);
    assert.equal(results[0].assignable, false);
    assert.match(results[0].error, /after 2 attempts/);
  }
});

test('cancellation during the silent wait prevents a retry', async () => {
  const mic = microphone({ sound: false }), controller = new AbortController();
  let strikes = 0;
  const pending = assert.rejects(mic.detector.scan({ signal: controller.signal, steps: 2, acousticRetries: 1,
    onReady: async () => { strikes++; },
  }), { name: 'AbortError' });
  await mic.advance(1800);
  controller.abort();
  await mic.advance();
  await pending;
  assert.equal(strikes, 1);
  assert.equal(mic.stopped(), 1);
});

test('late readiness after capture timeout cannot dispatch a strike', async () => {
  const mic = microphone({ sound: false });
  let release, strikes = 0;
  const pending = assert.rejects(mic.detector.scan({ signal: new AbortController().signal, steps: 2, acousticRetries: 1,
    onPrepare: () => new Promise(resolve => { release = resolve; }),
    onReady: async () => { strikes++; },
  }), /timed out/);
  await mic.advance(1500);
  mic.expire();
  await pending;
  release();
  await mic.advance();
  assert.equal(strikes, 0);
});

test('microphone stall after onset has a short wall-clock deadline and cannot retrigger motion', async () => {
  const mic = microphone();
  let failure, strikes = 0;
  const messages = [];
  const pending = mic.detector.scan({ signal: new AbortController().signal, steps: 2, acousticRetries: 1,
    onReady: async () => { strikes++; }, onProgress: message => messages.push(message),
  }).catch(error => { failure = error; });
  await mic.advance(1260);
  assert.ok(messages.some(message => /Heard the strike/.test(message)));
  mic.freeze();
  await mic.advance(5000);
  const endedPromptly = !!failure;
  if (!failure) mic.expire();
  await pending;
  assert.ok(endedPromptly, 'stalled audio must not wait for the 25-second fallback');
  assert.match(failure.message, /Microphone.*paused|Microphone.*stopped/);
  assert.equal(strikes, 1);
  assert.equal(mic.stopped(), 1);
});

test('a temporary stalled audio clock can resume and complete the original strike', async () => {
  for (const state of ['running', 'interrupted']) {
    const mic = microphone({ recoverAudio: true });
    let strikes = 0;
    const pending = mic.detector.scan({ signal: new AbortController().signal, steps: 1, acousticRetries: 1,
      onReady: async () => { strikes++; },
    });
    await mic.advance(1260);
    mic.freeze(state);
    await mic.advance(10000);
    // Bound cleanup even when running this regression against the old code.
    mic.expire();
    const results = await pending;
    assert.equal(results[0].midi, 64);
    assert.equal(strikes, 1);
  }
});

test('an analyser exception settles capture instead of leaving the ringing message stuck', async () => {
  const mic = microphone({ transform: (samples, now) => {
    if (now >= 1400) throw new Error('audio read failed');
    return samples;
  } });
  let strikes = 0;
  const pending = assert.rejects(mic.detector.scan({ signal: new AbortController().signal, steps: 2, acousticRetries: 1,
    onReady: async () => { strikes++; },
  }), /Microphone analysis failed: audio read failed/);
  await mic.advance(5000);
  await pending;
  assert.equal(strikes, 1);
  assert.equal(mic.stopped(), 1);
});
