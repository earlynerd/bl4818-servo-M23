// Run with: node --test tests/test_midi_transpose.cjs
const { test } = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const transpose = require('../player/midi_transpose.js');

const drum = [52, 55, 57, 60, 62, 64, 65, 67, 69, 72];
const pan = [48, 50, 52, 53, 55, 57, 58, 60, 62, 64, 65, 67, 69, 72];
const events = pitches => pitches.map((pitch, i) =>
  ({ pitch, timeMs: i * 200, durMs: 100, vel: 80 + i % 30, track: 0, channel: 0 }));

test('already fitting files choose zero and leave their source intact', () => {
  for (const target of [drum, pan]) {
    const input = events(target);
    const before = structuredClone(input);
    const result = transpose.findBest(input, target);
    assert.equal(result.shift, 0);
    assert.equal(result.counts.exact, input.length);
    assert.deepEqual(input, before);
    assert.deepEqual(result.events.map(e => e.pitch), target);
  }
});

test('uniform transposition finds a complete fit and preserves timing/dynamics', () => {
  const input = events(pan.map(p => p + 1));
  const result = transpose.findBest(input, pan, { mode: 'strict' });
  assert.equal(result.shift, -1);
  assert.equal(result.counts.exact, pan.length);
  result.events.forEach((e, i) => {
    assert.equal(e.pitch, pan[i]);
    assert.equal(e.timeMs, input[i].timeMs);
    assert.equal(e.durMs, input[i].durMs);
    assert.equal(e.vel, input[i].vel);
  });
});

test('octave folding keeps pitch classes and missing notes stay explicit', () => {
  assert.deepEqual(transpose.mapPitch(84, 0, drum, 'fold'),
    { pitch: 72, kind: 'folded', distance: 12 });
  assert.equal(transpose.mapPitch(71, 0, drum, 'fold').kind, 'unmapped');
  assert.equal(transpose.mapPitch(84, 0, drum, 'strict').kind, 'unmapped');
  assert.equal(transpose.mapPitch(58, 0, drum, 'adapt').pitch, 57);
  assert.equal(transpose.mapPitch(58, 0, pan, 'adapt').kind, 'exact');
});

test('standalone folding never searches for a key or register shift', () => {
  const chromatic = Array.from({ length: 21 }, (_, i) => 52 + i);
  const input = events([73, 85, 52, 0, 127]);
  const before = structuredClone(input);
  const result = transpose.findBest(input, chromatic, { transpose: false });
  assert.equal(result.shift, 0);
  assert.equal(result.shiftMode, 'none');
  assert.deepEqual(result.events.map(e => e.pitch), [61, 61, 52, 60, 67]);
  assert.ok(result.events.every((e, i) => (e.pitch - input[i].pitch) % 12 === 0));
  assert.deepEqual(input, before);
  // Automatic best fit is still available, and deliberately chooses a shift.
  assert.equal(transpose.findBest(events([73, 85]), chromatic).shift, -13);
  const missing = transpose.findBest(events([71, 84]), drum, { transpose: false });
  assert.equal(missing.counts.unmapped, 1);
  assert.equal(missing.events[1].pitch, 72);
});

test('manual shifts use the requested signed semitones without optimizing them', () => {
  for (const shift of [-5, 7]) {
    const input = events([53, 65]);
    const result = transpose.findBest(input, pan, { shift, mode: 'strict' });
    assert.equal(result.shift, shift);
    assert.equal(result.shiftMode, 'manual');
    assert.deepEqual(result.events.map(e => e.pitch), [53 + shift, 65 + shift]);
    assert.equal(result.counts.exact, 2);
    result.events.forEach((e, i) => {
      assert.equal(e.timeMs, input[i].timeMs);
      assert.equal(e.durMs, input[i].durMs);
      assert.equal(e.vel, input[i].vel);
      assert.equal(e.originalPitch, input[i].pitch);
    });
  }
  const input = events([61]);
  assert.equal(transpose.findBest(input, [60, 62]).shift, -1);
  assert.equal(transpose.findBest(input, [60, 62], { shift: 1 }).events[0].pitch, 62);
  assert.equal(transpose.findBest(input, [60, 62], { shift: 0 }).counts.unmapped, 1);
});

test('manual transposition precedes optional octave folding', () => {
  const input = events([84, 96]);
  const target = [60, 61, 72];
  const folded = transpose.findBest(input, target, { shift: 1 });
  assert.equal(folded.shift, 1);
  assert.deepEqual(folded.events.map(e => e.pitch), [61, 61]);
  assert.equal(folded.counts.folded, 2);
  assert.equal(transpose.findBest(input, target, { shift: 1, mode: 'strict' }).counts.unmapped, 2);
  const unchanged = transpose.findBest(input, target, { transpose: false, shift: 1, mode: 'strict' });
  assert.deepEqual(unchanged.events.map(e => e.pitch), [84, 96]);
  assert.equal(unchanged.shift, 0);
});

test('manual shifts validate integers and MIDI limits without routing clamped notes', () => {
  for (const shift of [-128, 128, 0.5, NaN, Infinity, '12']) {
    assert.throws(() => transpose.findBest(events([60]), drum, { shift }), /whole-number/);
  }
  assert.equal(transpose.findBest(events([127]), [0], { shift: -127, mode: 'strict' }).counts.exact, 1);
  assert.equal(transpose.findBest(events([0]), [127], { shift: 127, mode: 'strict' }).counts.exact, 1);
  const outside = transpose.findBest(events([127]), [127], { shift: 1, mode: 'strict' });
  assert.equal(outside.counts.unmapped, 1);
  assert.equal(outside.events[0].transposeKind, 'unmapped');
});

test('pitch fitting does not change octave placement to avoid closely spaced notes', () => {
  // Both 0 and -12 give two exact matches and one octave fold. Previously,
  // the closely spaced final two notes made -12 win at 1x, despite moving
  // the whole piece farther from its original register.
  for (const times of [[0, 100, 120], [0, 400, 480], [0, 40, 48]]) {
    const input = events([60, 72, 84]).map((e, i) => ({ ...e, timeMs: times[i] }));
    const result = transpose.findBest(input, [60, 72]);
    assert.equal(result.shift, 0);
    assert.deepEqual(result.events.map(e => e.pitch), [60, 72, 72]);
  }
});

test('all chromatic notes expose substitutions; actual instrument defines the scale', () => {
  const input = events(Array.from({ length: 12 }, (_, i) => 60 + i));
  const fold = transpose.findBest(input, drum);
  const adapted = transpose.findBest(input, drum, { mode: 'adapt' });
  assert.equal(fold.counts.unmapped, 6);
  assert.equal(adapted.counts.substituted, 6);
  assert.equal(adapted.counts.unmapped, 0);
  assert.ok(adapted.events.every(e => drum.includes(e.pitch)));
  assert.equal(transpose.findBest(input, pan).counts.unmapped, 5);
});

test('voice choices exclude drums and do not bias the selected melody', () => {
  const melody = events(pan.map(p => p + 1));
  const drums = events(Array(80).fill(35)).map(e => ({ ...e, channel: 9 }));
  const result = transpose.findBest([...melody, ...drums], pan,
    { voices: new Set(['0:0']) });
  assert.equal(result.shift, -1);
  assert.equal(result.excluded, 80);
  assert.equal(result.events.length, melody.length);
  assert.throws(() => transpose.findBest(melody, pan, { voices: new Set() }), /Select/);
});

test('only enumerated mapped slots define the target, including pitch zero', () => {
  assert.deepEqual(transpose.targetPitches([0, 60, null, 60, 72, 128], 4), [0, 60]);
  assert.deepEqual(transpose.targetPitches(drum, 0), []);
  assert.throws(() => transpose.findBest(events([60]), []), /Assign pitches/);
  assert.equal(transpose.findBest(events([127]), [0]).events[0].pitch, 0);
});

test('restrike protection uses rounded real time and never shifts the music', () => {
  const input = [0, 49.49, 49.5, 100].map(timeMs => ({ timeMs, slot: 0, vel: 90 }));
  const schedule = transpose.enforceRestrike(structuredClone(input), 1);
  assert.equal(schedule[1].suppressedReason, 'restrike');
  assert.ok(!schedule[2].suppressed);
  assert.deepEqual(schedule.map(e => e.timeMs), input.map(e => e.timeMs));
  const fast = transpose.enforceRestrike(structuredClone(input), 2);
  assert.equal(fast.filter(e => !e.suppressed).length, 2);
});

function vlq(n) {
  const bytes = [n & 127];
  while ((n >>= 7)) bytes.unshift((n & 127) | 128);
  return bytes;
}
function midi(tracks, format = tracks.length > 1 ? 1 : 0) {
  const header = Buffer.from([77, 84, 104, 100, 0, 0, 0, 6, 0, format, 0, tracks.length, 1, 224]);
  const chunks = tracks.map(messages => {
    const data = Buffer.from([...messages.flat(), 0, 255, 47, 0]);
    const header = Buffer.alloc(8);
    header.write('MTrk'); header.writeUInt32BE(data.length, 4);
    return Buffer.concat([header, data]);
  });
  const bytes = Buffer.concat([header, ...chunks]);
  return bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength);
}
function melodyFile() {
  return midi([pan.flatMap(p => [[0, 0x90, p + 1, 90], [...vlq(240), 0x80, p + 1, 0]])]);
}

// Exercise the real import, apply/restore, preview, and playback functions with
// an in-memory DOM and HTTP sink. No serial ports or actual strikes are used.
function player() {
  const elements = new Map();
  let voices = [];
  const node = id => {
    if (!elements.has(id)) {
      const element = { value: '', style: {}, dataset: {}, disabled: false, textContent: '',
        classList: { add() {}, remove() {}, toggle() {} }, addEventListener() {},
        querySelectorAll: () => voices };
      Object.defineProperty(element, 'innerHTML', {
        get() { return this.html || ''; },
        set(value) {
          this.html = value;
          if (id === 'transposeVoices') voices = [...value.matchAll(/data-transpose-voice="([^"]+)"\s*(checked)?/g)]
            .map(m => ({ dataset: { transposeVoice: m[1] }, checked: !!m[2], addEventListener() {} }));
        },
      });
      elements.set(id, element);
    }
    return elements.get(id);
  };
  const requests = [];
  const context = vm.createContext({ MidiTranspose: transpose, TextDecoder,
    document: { getElementById: node, addEventListener() {},
      querySelectorAll: selector => selector.includes('data-transpose-voice')
        ? (selector.endsWith(':checked') ? voices.filter(v => v.checked) : voices) : [] },
    window: { innerHeight: 1000 }, console, performance: { now: () => 0 },
    fetch: async (url, options) => { requests.push({ url, options }); return { ok: true, json: async () => ({}) }; },
    requestAnimationFrame() {}, cancelAnimationFrame() {}, clearTimeout() {},
  });
  const html = fs.readFileSync(path.join(__dirname, '../player/midi_player.html'), 'utf8');
  vm.runInContext(html.match(/<script>([\s\S]*?)<\/script>/)[1], context);
  vm.runInContext('const realRenderPianoRoll = renderPianoRoll;', context);
  vm.runInContext('renderPianoRoll = () => {}; renderFallbackTable = () => {}; startPlayPolling = () => {}; stopPlayPolling = () => {}; log = () => {};', context);
  const run = source => vm.runInContext(source, context);
  node('tempoSlider').value = '1';
  node('currentSlider').value = '800';
  run(`S.mapping = ${JSON.stringify(pan)}; S.count = ${pan.length}; S.trim = Array(S.count).fill(1);`);
  return { context, node, requests, run, voices: () => voices,
    load(buffer = melodyFile()) { context.buffer = buffer; run('loadFromMIDI(buffer, "test.mid")'); } };
}

test('parser preserves track/channel voices, independent durations, and tempo changes', () => {
  const p = player();
  const buffer = midi([
    [[0, 255, 3, 4, 76, 101, 97, 100], [0, 0x90, 60, 80], [0, 0x91, 60, 70],
      [...vlq(240), 0x80, 60, 0], [...vlq(480), 0x81, 60, 0]],
    [[...vlq(480), 255, 81, 3, 15, 66, 64]], // 1,000,000 us/beat at tick 480
  ]);
  p.load(buffer);
  assert.equal(p.run('S.composition.events[0].durMs'), 250);
  assert.equal(p.run('S.composition.events[1].durMs'), 1000);
  assert.equal(p.run('S.composition.events[1].channel'), 1);
  assert.equal(p.run('S.composition.events[1].trackName'), 'Lead');
  assert.equal(p.voices().length, 2);
});

test('12-track preview isolates source notes without changing inclusion or playback', () => {
  const p = player();
  p.load(midi(Array.from({ length: 12 }, (_, t) => [
    [0, 0xc0, t * 8], [0, 0x90, 48 + t, 90], [...vlq(480), 0x80, 48 + t, 0],
  ])));
  assert.equal(p.voices().length, 12);
  assert.match(p.node('transposeVoices').innerHTML, /GM program 9/);
  assert.match(p.node('transposeVoices').innerHTML, /C3–C3/);
  p.voices()[3].checked = false;
  const selection = p.voices().map(v => v.checked);
  const events = p.run('JSON.stringify(S.composition.events)');
  p.run("previewMidiVoice('3:0'); realRenderPianoRoll()");
  assert.equal(p.run('S.composition.previewVoice'), '3:0');
  assert.equal((p.node('pianoGrid').innerHTML.match(/class="note-bar/g) || []).length, 1);
  assert.match(p.node('pianoGrid').innerHTML, /Track 4 · ch 1 · Guitar/);
  assert.match(p.node('pianoGrid').innerHTML, /source preview/);
  assert.equal(p.run('JSON.stringify(S.composition.events)'), events);
  assert.deepEqual(p.voices().map(v => v.checked), selection);
  assert.equal(p.requests.length, 0);
  p.run('applyTransposition()');
  assert.equal(p.run('S.composition.previewVoice'), null);
  assert.equal(p.run('S.composition.events.some(e => e.track === 3)'), false);
  p.run("previewMidiVoice('3:0'); realRenderPianoRoll()");
  assert.equal((p.node('pianoGrid').innerHTML.match(/class="note-bar/g) || []).length, 1);
  p.load();
  assert.equal(p.run('S.composition.previewVoice'), undefined);
});

test('parser retains changing programs, instrument metadata and safe track labels', () => {
  const p = player();
  p.load(midi([[
    [0, 255, 3, 3, 60, 34, 62],
    [0, 0xc0, 0], [0, 0x90, 60, 90],
    [...vlq(480), 0xc0, 32], [0, 0x90, 64, 70],
    [...vlq(480), 0x80, 64, 0],
  ]]));
  assert.equal(p.run('S.composition.originalEvents[1].program'), 32);
  assert.match(p.node('transposeVoices').innerHTML, /Piano.*Bass/);
  p.run("previewMidiVoice('0:0'); realRenderPianoRoll()");
  assert.match(p.node('pianoGrid').innerHTML, /&lt;&quot;&gt;/);
  assert.doesNotMatch(p.node('pianoGrid').innerHTML, /title="<">/);
  assert.equal(p.run('S.composition.maxMs'), 1000);
});

test('Show toggles source visibility; arrangement restores all notes and mallet colors', () => {
  const p = player();
  p.load(midi([
    [[0, 0x90, 60, 90], [...vlq(480), 0x80, 60, 0]],
    [[0, 0x90, 64, 90], [...vlq(480), 0x80, 64, 0]],
  ]));
  const selection = p.voices().map(v => v.checked);
  const notes = () => (p.node('pianoGrid').innerHTML.match(/class="note-bar/g) || []).length;
  const arrangementColor = p.run('SLOT_COLORS[S.mapping.indexOf(60)]');
  p.run('realRenderPianoRoll()');
  assert.equal(notes(), 2);
  assert.ok(p.node('pianoGrid').innerHTML.includes(`background:${arrangementColor}`));
  assert.equal(p.node('arrangementViewBtn').disabled, true);
  assert.equal(p.node('stopAuditionBtn').disabled, true);
  p.run("setMidiVoiceVisible('1:0', false); realRenderPianoRoll()");
  assert.equal(notes(), 1);
  assert.ok(p.node('pianoGrid').innerHTML.includes('background:hsl('));
  assert.equal(p.node('arrangementViewBtn').disabled, false);
  p.run("setMidiVoiceVisible('0:0', false); realRenderPianoRoll()");
  assert.equal(notes(), 0);
  p.run("setMidiVoiceVisible('1:0', true); realRenderPianoRoll()");
  assert.equal(notes(), 1);
  p.run('previewMidiVoice(null); realRenderPianoRoll()');
  assert.equal(notes(), 2);
  assert.ok(p.node('pianoGrid').innerHTML.includes(`background:${arrangementColor}`));
  assert.equal(p.node('arrangementViewBtn').disabled, true);
  assert.deepEqual(p.voices().map(v => v.checked), selection);
  assert.equal(p.requests.length, 0);
});

test('solo audition is bounded, computer-only, cancellable and leaves selection intact', async () => {
  const p = player(); p.load();
  const selection = p.voices().map(v => v.checked);
  let started = 0, closed = 0;
  p.context.setTimeout = () => 123;
  p.context.window.AudioContext = class {
    currentTime = 0;
    destination = {};
    async resume() {}
    async close() { closed++; }
    createOscillator() { return { frequency: {}, connect() {}, start() { started++; }, stop() {} }; }
    createGain() { return { connect() {}, gain: { setValueAtTime() {}, linearRampToValueAtTime() {}, exponentialRampToValueAtTime() {} } }; }
  };
  await p.run("auditionMidiVoice('0:0')");
  assert.equal(p.node('stopAuditionBtn').disabled, false);
  assert.equal(started, pan.length);
  assert.equal(p.requests.length, 0);
  assert.deepEqual(p.voices().map(v => v.checked), selection);
  assert.match(p.node('voicePreviewStatus').textContent, /computer only/);
  p.run('stopVoiceAudition()');
  assert.equal(p.node('stopAuditionBtn').disabled, true);
  assert.equal(closed, 1);
  assert.equal(p.run('voiceAudition'), null);
  p.run('S.playing = true');
  await p.run("auditionMidiVoice('0:0')");
  assert.equal(started, pan.length);
});

test('opening stays original; apply is repeatable; restore and subsequent open reset it', () => {
  const p = player(); p.load();
  const original = p.run('JSON.stringify(S.composition.events)');
  const originalBytes = p.run('S.loadedMidiBuf');
  assert.equal(p.run('S.composition.transposition'), null);
  assert.equal(p.run('S.composition.events[0].pitch'), 49);
  p.run('applyTransposition()');
  assert.equal(p.run('S.composition.transposition.shift'), -1);
  const applied = p.run('JSON.stringify(S.composition.events)');
  p.run('applyTransposition()');
  assert.equal(p.run('JSON.stringify(S.composition.events)'), applied);
  assert.equal(p.run('S.loadedMidiBuf'), originalBytes);
  p.run('restoreOriginalMidi()');
  assert.equal(p.run('JSON.stringify(S.composition.events)'), original);
  p.run('applyTransposition()'); p.load();
  assert.equal(p.run('S.composition.transposition'), null);
  assert.equal(p.node('restoreMidiBtn').disabled, true);
});

test('player checkboxes support automatic only, folding only, both and neither', () => {
  const p = player();
  p.load(midi([[[0, 0x90, 73, 90], [...vlq(480), 0x80, 73, 0],
    [0, 0x90, 85, 90], [...vlq(480), 0x80, 85, 0]]]));
  p.run('S.mapping = Array.from({length:21}, (_, i) => 52 + i); S.count = 21; S.trim = Array(21).fill(1)');
  const original = p.run('JSON.stringify(S.composition.originalEvents)');
  assert.equal(p.node('transposeEnabled').checked, true);
  assert.equal(p.node('octaveFoldEnabled').checked, true);
  assert.equal(p.node('transposeShiftMode').value, 'auto');
  p.run('applyTransposition()');
  assert.equal(p.run('S.composition.transposition.shift'), -13);
  p.node('transposeEnabled').checked = false;
  p.run('refreshTransposeControls()');
  assert.equal(p.node('transposeShiftMode').disabled, true);
  assert.match(p.node('transposeSummary').textContent, /Options changed/);
  assert.equal(p.run('S.composition.transposition.shift'), -13);
  p.run('applyTransposition()');
  assert.equal(p.run('S.composition.transposition.shift'), 0);
  assert.equal(p.run('JSON.stringify(S.composition.events.map(e => e.pitch))'), '[61,61]');
  assert.match(p.node('transposeSummary').textContent, /No transposition.*2 octave moves/);
  p.node('octaveFoldEnabled').checked = false;
  p.run('applyTransposition()');
  assert.equal(p.node('transposeMode').disabled, true);
  assert.equal(p.run('JSON.stringify(S.composition.events.map(e => e.pitch))'), '[73,85]');
  assert.equal(p.run('S.composition.transposition.counts.unmapped'), 2);
  p.node('transposeEnabled').checked = true;
  p.run('applyTransposition()');
  assert.equal(p.run('S.composition.transposition.shift'), -13);
  assert.equal(p.run('S.composition.transposition.mode'), 'strict');
  assert.equal(p.run('S.composition.transposition.counts.exact'), 2);
  assert.equal(p.run('JSON.stringify(S.composition.originalEvents)'), original);
});

test('player manual offsets apply from source, mark pending edits and reset on new files', () => {
  const p = player(); p.load();
  const original = p.run('JSON.stringify(S.composition.events)');
  p.node('transposeShiftMode').value = 'manual';
  p.node('transposeSemitones').value = '-1';
  p.node('octaveFoldEnabled').checked = false;
  p.run('applyTransposition()');
  assert.equal(p.node('transposeSemitonesControl').hidden, false);
  assert.equal(p.node('transposeSemitones').disabled, false);
  assert.match(p.node('transposeSummary').textContent, /Manual -1 semitones/);
  assert.equal(p.run('S.composition.transposition.counts.exact'), pan.length);
  const applied = p.run('JSON.stringify(S.composition.events)');
  p.run('applyTransposition()');
  assert.equal(p.run('JSON.stringify(S.composition.events)'), applied);
  p.node('transposeSemitones').value = '11';
  p.node('octaveFoldEnabled').checked = true;
  p.run('refreshTransposeControls()');
  assert.match(p.node('transposeSummary').textContent, /Options changed/);
  assert.equal(p.run('JSON.stringify(S.composition.events)'), applied);
  p.run('applyTransposition()');
  assert.match(p.node('transposeSummary').textContent, /Manual \+11 semitones/);
  assert.ok(p.run('S.composition.transposition.counts.folded') > 0);
  p.run('restoreOriginalMidi()');
  assert.equal(p.run('JSON.stringify(S.composition.events)'), original);
  p.load();
  assert.equal(p.node('transposeShiftMode').value, 'auto');
  assert.equal(p.node('transposeSemitones').value, '0');
  assert.equal(p.run('S.composition.transposition'), null);
});

test('invalid manual input preserves the applied view; disabled transpose ignores it', () => {
  const p = player(); p.load(); p.run('applyTransposition()');
  const current = p.run('JSON.stringify(S.composition.events)');
  p.node('transposeShiftMode').value = 'manual';
  for (const value of ['', '1.5', '128', '-128']) {
    p.node('transposeSemitones').value = value;
    p.run('applyTransposition()');
    assert.match(p.node('errorBox').textContent, /whole-number/);
    assert.equal(p.run('JSON.stringify(S.composition.events)'), current);
  }
  p.node('transposeEnabled').checked = false;
  p.run('applyTransposition()');
  assert.equal(p.node('transposeSemitones').disabled, true);
  assert.equal(p.run('S.composition.transposition.shift'), 0);
});

test('new pitch controls are disabled while starting, playing or updating firmware', () => {
  const p = player(); p.load();
  p.node('transposeShiftMode').value = 'manual';
  for (const flag of ['playStarting', 'playing', 'firmwareUpdating']) {
    p.run(`S.${flag} = true; refreshTransposeControls()`);
    for (const id of ['transposeEnabled', 'octaveFoldEnabled', 'transposeShiftMode',
      'transposeSemitones', 'transposeMode', 'transposeBtn']) {
      assert.equal(p.node(id).disabled, true, `${flag}: ${id}`);
    }
    p.run('applyTransposition()');
    assert.equal(p.run('S.composition.transposition'), null);
    p.run(`S.${flag} = false`);
  }
});

test('drums are excluded only on click and selections apply only on click', () => {
  const p = player();
  p.load(midi([[[0, 0x90, 60, 90], [0, 0x99, 35, 100]]]));
  assert.equal(p.run('S.composition.events.length'), 2);
  assert.equal(p.voices()[1].checked, false);
  p.run('applyTransposition()');
  assert.equal(p.run('S.composition.events.length'), 1);
  p.voices()[1].checked = true;
  p.run('refreshTransposeControls()');
  assert.match(p.node('transposeSummary').textContent, /Options changed/);
  assert.equal(p.run('S.composition.events.length'), 1);
  p.run('applyTransposition()');
  assert.equal(p.run('S.composition.events.length'), 2);
});

test('old fallback routes cannot override transposed drops; original scheduling is unchanged', () => {
  const p = player();
  p.run('S.fallback[71] = 0');
  assert.equal(p.run('buildSchedule([{pitch:71, timeMs:0, vel:90, transposeKind:"unmapped"}])[0].slot'), -1);
  assert.equal(p.run('buildSchedule([{pitch:71, timeMs:0, vel:90}])[0].slot'), 0);
  p.run('testEvents = [0, 20, 50].map(timeMs => ({pitch:60, timeMs, vel:90}))');
  assert.equal(p.run('buildSchedule(testEvents).filter(e => !e.suppressed).length'), 3);
  assert.equal(p.run('buildSchedule(testEvents.map(e => ({...e, transposeKind:"exact"}))).filter(e => !e.suppressed).length'), 2);
});

test('mapping changes require reapply; busy playback cannot modify the piece', async () => {
  const p = player(); p.load(); p.run('applyTransposition()');
  p.run('S.mapping[0] = 49');
  assert.equal(p.run('transpositionIsStale()'), true);
  await p.run('play()');
  assert.equal(p.requests.length, 0);
  p.run('applyTransposition()');
  assert.equal(p.run('transpositionIsStale()'), false);
  p.run('S.playing = true; restoreOriginalMidi()');
  assert.ok(p.run('S.composition.transposition'));
});

test('server receives the transformed schedule at the selected tempo', async () => {
  const p = player(); p.load(); p.run('applyTransposition()');
  p.node('tempoSlider').value = '2';
  await p.run('play()');
  const sent = JSON.parse(p.requests[0].options.body).events;
  assert.equal(p.requests[0].url, '/api/play');
  assert.deepEqual(sent.map(e => e.address), pan.map((_, i) => i));
  assert.equal(sent[1].t_ms, 125);
  assert.equal(p.node('transposeBtn').disabled, true);
  assert.equal(p.node('restoreMidiBtn').disabled, true);
});

test('merged and omitted notes agree between preview and submitted playback', async () => {
  const p = player();
  p.load(midi([[[0, 0x90, 60, 80], [0, 0x90, 72, 100],
    [...vlq(20), 0x90, 60, 95], [...vlq(76), 0x90, 60, 85]]]));
  p.run('S.mapping = [60]; S.count = 1; applyTransposition()');
  assert.equal(p.run('S.composition.transposition.counts.folded'), 1);
  assert.match(p.node('transposeSummary').textContent, /1 merged · 1 omitted under 50 ms · 2 playable/);
  await p.run('play()');
  const sent = JSON.parse(p.requests[0].options.body).events;
  assert.deepEqual(sent.map(e => e.t_ms), [0, 100]);
  assert.ok(sent[0].nominal_current_ma > sent[1].nominal_current_ma);
});

test('invalid or in-flight imports preserve the current original and adaptation', () => {
  const p = player(); p.load(); p.run('applyTransposition()');
  const saved = p.run('S.loadedMidiBuf');
  const current = p.run('JSON.stringify(S.composition.events)');
  p.context.invalid = new ArrayBuffer(4);
  assert.equal(p.run('loadFromMIDI(invalid, "bad.mid")'), false);
  assert.equal(p.run('JSON.stringify(S.composition.events)'), current);
  p.run('S.playStarting = true');
  assert.equal(p.run('loadFromMIDI(buffer, "next.mid")'), false);
  assert.equal(p.run('S.loadedMidiBuf'), saved);
});
