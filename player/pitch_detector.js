/* Local microphone pitch estimation; no audio is uploaded or stored. */
(function(root, factory) {
  const api = factory();
  if (typeof module === 'object' && module.exports) module.exports = api;
  else root.PitchDetector = api;
})(typeof globalThis !== 'undefined' ? globalThis : this, function() {
  'use strict';
  const median = values => {
    const a = [...values].sort((x, y) => x - y);
    return a.length ? a[Math.floor(a.length / 2)] : 0;
  };

  function level(samples) {
    let sum = 0, clipped = 0;
    for (const value of samples) {
      sum += value * value;
      if (Math.abs(value) >= 0.98) clipped++;
    }
    return { rms: Math.sqrt(sum / samples.length), clipped: clipped / samples.length };
  }

  function note(frequency) {
    const exact = 69 + 12 * Math.log2(frequency / 440);
    const midi = Math.round(exact);
    const names = ['C', 'C♯', 'D', 'E♭', 'E', 'F', 'F♯', 'G', 'A♭', 'A', 'B♭', 'B'];
    return { frequency, midi, cents: (exact - midi) * 100,
      name: names[((midi % 12) + 12) % 12] + (Math.floor(midi / 12) - 1) };
  }

  // YIN cumulative mean normalized difference. Box-average decimation bounds
  // the work on phones; the search covers 80–1500 Hz independently of mapping.
  function estimate(samples, sampleRate) {
    const inputLevel = level(samples);
    if (inputLevel.rms < 0.001 || inputLevel.clipped > 0.01) return null;
    const step = Math.max(1, Math.floor(sampleRate / 12000));
    const rate = sampleRate / step;
    const data = new Float32Array(Math.floor(samples.length / step));
    for (let i = 0; i < data.length; i++) {
      for (let j = 0; j < step; j++) data[i] += samples[i * step + j] / step;
    }
    const minLag = Math.max(2, Math.floor(rate / 1500));
    const maxLag = Math.ceil(rate / 80);
    const size = data.length - maxLag - 1;
    if (size < maxLag) return null;
    const difference = new Float64Array(maxLag + 1);
    let running = 0;
    for (let lag = 1; lag <= maxLag; lag++) {
      let sum = 0;
      for (let i = 0; i < size; i++) {
        const delta = data[i] - data[i + lag];
        sum += delta * delta;
      }
      running += sum;
      difference[lag] = running ? sum * lag / running : 1;
    }
    for (let lag = minLag; lag < maxLag; lag++) {
      if (difference[lag] >= 0.15) continue;
      while (lag + 1 < maxLag && difference[lag + 1] < difference[lag]) lag++;
      const a = difference[lag - 1], b = difference[lag], c = difference[lag + 1];
      const denominator = a - 2 * b + c;
      const correction = denominator ? Math.max(-1, Math.min(1, (a - c) / (2 * denominator))) : 0;
      // Refine the selected period at the original sample rate so high notes
      // do not inherit the decimated grid's several-cent interpolation bias.
      const center = Math.round((lag + correction) * step);
      const width = samples.length - center - step - 2;
      const rawDifference = offset => {
        let sum = 0;
        for (let i = 0; i < width; i++) sum += (samples[i] - samples[i + offset]) ** 2;
        return sum;
      };
      let best = center, bestValue = rawDifference(center);
      for (let offset = Math.max(2, center - step); offset <= center + step; offset++) {
        const value = rawDifference(offset);
        if (value < bestValue) { best = offset; bestValue = value; }
      }
      const left = rawDifference(best - 1), right = rawDifference(best + 1);
      const curve = left - 2 * bestValue + right;
      const refined = best + (curve ? Math.max(-1, Math.min(1, (left - right) / (2 * curve))) : 0);
      const frequency = sampleRate / refined;
      if (frequency < 80 || frequency > 1500) return null;
      return { ...note(frequency), confidence: 1 - b, rms: inputLevel.rms };
    }
    return null;
  }

  function consensus(frames) {
    if (frames.length < 6) return null;
    const recent = frames.slice(-10);
    const frequency = median(recent.map(f => f.frequency));
    const agreeing = recent.filter(f => Math.abs(1200 * Math.log2(f.frequency / frequency)) <= 25);
    if (agreeing.length < 6 || agreeing.length / recent.length < 0.8) return null;
    const result = note(median(agreeing.map(f => f.frequency)));
    return { ...result, confidence: median(agreeing.map(f => f.confidence)), frames: agreeing.length,
      assignable: Math.abs(result.cents) <= 35 };
  }

  function abortError() { return new DOMException('Listening cancelled', 'AbortError'); }

  async function listen({ signal, deviceId, onReady = async () => {}, onProgress = () => {} }) {
    if (!globalThis.isSecureContext) {
      throw new Error('Microphone access needs HTTPS on your phone. On this PC, open http://localhost:8765 (or your configured port).');
    }
    if (!navigator.mediaDevices?.getUserMedia) throw new Error('This browser does not support microphone input.');
    if (signal.aborted) throw abortError();
    const Audio = globalThis.AudioContext || globalThis.webkitAudioContext;
    if (!Audio) throw new Error('This browser does not support audio analysis.');
    const context = new Audio();
    let stream, source, analyser, mute, timer, deadline, rejectCapture;
    const cleanup = () => {
      clearInterval(timer);
      clearTimeout(deadline);
      stream?.getTracks().forEach(track => track.stop());
      source?.disconnect();
      analyser?.disconnect();
      mute?.disconnect();
      if (context.state !== 'closed') context.close().catch(() => {});
    };
    const abort = () => { cleanup(); rejectCapture?.(abortError()); };
    signal.addEventListener('abort', abort, { once: true });
    try {
      // Resume in the button gesture, before the permission prompt resolves.
      let resumeError;
      const resumed = context.resume().catch(error => { resumeError = error; });
      onProgress('Allow microphone access, then keep the instrument quiet.');
      const pending = navigator.mediaDevices.getUserMedia({ audio: {
        echoCancellation: false, noiseSuppression: false, autoGainControl: false,
        ...(deviceId ? { deviceId: { exact: deviceId } } : {}),
      }, video: false });
      // A dismissed permission prompt can remain pending. Close promptly, and
      // stop any stream that arrives after the user has cancelled.
      pending.then(s => { if (signal.aborted) s.getTracks().forEach(t => t.stop()); }, () => {});
      stream = await new Promise((resolve, reject) => {
        rejectCapture = reject;
        pending.then(resolve, reject);
      });
      await resumed;
      if (signal.aborted) throw abortError();
      if (resumeError) throw resumeError;
      source = context.createMediaStreamSource(stream);
      analyser = context.createAnalyser();
      // About 170 ms at 48 kHz; enough periods of an 80 Hz note.
      analyser.fftSize = context.sampleRate > 60000 ? 16384 : 8192;
      mute = context.createGain();
      mute.gain.value = 0;
      source.connect(analyser);
      analyser.connect(mute);
      mute.connect(context.destination);
      const samples = new Float32Array(analyser.fftSize);
      return await new Promise((resolve, reject) => {
        rejectCapture = reject;
        const noise = [], frames = [];
        let phase = 'baseline', armedAt = 0, onset = 0, lastAudioTime = -1;
        let acknowledged = false, candidate = null, clipping = false;
        const started = performance.now();
        const finish = () => { if (acknowledged && candidate) resolve(candidate); };
        stream.getAudioTracks()[0].addEventListener('ended', () => reject(new Error('Microphone disconnected.')));
        deadline = setTimeout(() => reject(new Error('Microphone or strike timed out. Check the connection and retry.')), 15000);
        onProgress('Measuring background sound… keep quiet for a moment.');
        timer = setInterval(() => {
          if (signal.aborted) return;
          if (context.state !== 'running' || context.currentTime === lastAudioTime) return;
          lastAudioTime = context.currentTime;
          const now = performance.now();
          analyser.getFloatTimeDomainData(samples);
          const amplitude = level(samples.subarray(samples.length - 1024));
          if (phase === 'baseline') {
            // Let the analyser fill before measuring the noise floor.
            if (now - started > 250) noise.push(amplitude.rms);
            if (now - started < 850) return;
            phase = 'armed';
            armedAt = now;
            onProgress('Listening for a new strike…');
            Promise.resolve().then(onReady).then(() => { acknowledged = true; finish(); }, reject);
            return;
          }
          if (now - armedAt > 8000) {
            reject(new Error(clipping ? 'Microphone is overloaded. Move it farther away or reduce its input gain, then retry.'
              : 'No clear, stable note. Let the drum become quiet, move the microphone closer, and retry.'));
            return;
          }
          if (!onset) {
            if (amplitude.rms > Math.max(0.004, median(noise) * 3)) {
              onset = now;
              onProgress('Heard the strike. Measuring the ringing note…');
            }
            return;
          }
          // Wait until the whole analysis window is past the impact transient.
          if (now - onset < 100 + 1000 * samples.length / context.sampleRate) return;
          clipping ||= level(samples).clipped > 0.01;
          const estimateFrame = estimate(samples, context.sampleRate);
          if (estimateFrame && estimateFrame.rms > Math.max(0.001, median(noise) * 1.5)) {
            frames.push(estimateFrame);
            candidate = consensus(frames);
            if (candidate) finish();
          } else {
            frames.length = 0;
          }
        }, 60);
      });
    } finally {
      signal.removeEventListener('abort', abort);
      cleanup();
    }
  }
  return { estimate, consensus, note, level, listen };
});
