/* Microphone workflow for the existing MIDI player's slot mapping. */
'use strict';
let pitchSession = null;
let pitchResult = null;
let pitchSaving = false;
let pitchScan = null;
const pitchElement = id => document.getElementById(id);

function pitchMessage(message) { pitchElement('pitchStatus').textContent = message; }

function refreshPitchButtons() {
  const busy = !!pitchSession || pitchSaving;
  const slot = +pitchElement('pitchSlot').value;
  const unavailable = !S.count || S.playing || S.playStarting || S.firmwareUpdating;
  pitchElement('pitchListen').disabled = busy || unavailable;
  pitchElement('pitchStrike').disabled = busy || unavailable || !S.homed[slot] || Number(S.slots[slot]?.fault) > 0;
  pitchElement('pitchApply').disabled = busy || unavailable || !pitchResult?.assignable;
  pitchElement('pitchCancel').disabled = !pitchSession;
  pitchElement('pitchSlot').disabled = busy;
  pitchElement('pitchMic').disabled = busy;
  pitchElement('pitchScanStart').disabled = busy || unavailable;
  pitchElement('pitchScanStop').disabled = !pitchSession;
  pitchElement('pitchScanRetry').disabled = busy || unavailable || !pitchScan?.rows.some(row => !row.assignable);
  const detected = pitchScan?.rows.filter(row => row.assignable).length || 0;
  pitchElement('pitchScanSave').disabled = busy || unavailable || detected === 0;
  pitchElement('pitchScanSave').textContent = detected ? `Save ${detected} detected pitches` : 'Save detected pitches';
}

function openPitchDetection() {
  if (S.playing || S.playStarting || S.firmwareUpdating) {
    showError('Stop playback or finish the firmware update before detecting pitches.');
    return;
  }
  stopVoiceAudition();
  S.pitchOpen = true;
  pitchResult = null;
  pitchScan = null;
  renderPitchScan();
  pitchElement('pitchSlot').innerHTML = Array.from({ length: S.count }, (_, slot) =>
    `<option value="${slot}">Mallet ${slot} · ${S.mapping[slot] == null ? 'unassigned' : midiName(S.mapping[slot])}</option>`).join('');
  pitchElement('pitchReading').textContent = '—';
  pitchMessage(S.count ? 'Choose a mallet. Let previous notes fade before each measurement.' : 'Enumerate the instrument first, then reopen this dialog.');
  pitchElement('pitchDialog').showModal();
  refreshPitchButtons();
}

function cancelPitchDetection() {
  pitchSession?.abort();
  pitchSession = null;
  if (pitchScan) {
    for (const row of pitchScan.rows) {
      if (row.state === 'measuring') { row.state = 'pending'; row.error = 'Stopped before completion'; }
    }
    renderPitchScan();
  }
  refreshPitchButtons();
}

async function startPitchDetection(strike) {
  if (pitchSession || pitchSaving || !S.pitchOpen) return;
  const slot = +pitchElement('pitchSlot').value;
  if (!Number.isInteger(slot) || slot < 0 || slot >= S.count || S.playing || S.playStarting || S.firmwareUpdating) return;
  if (strike && (!S.homed[slot] || Number(S.slots[slot]?.fault) > 0)) {
    pitchMessage('Home this mallet and clear any fault before striking. Listen only also works with a hand tap.');
    return;
  }
  const session = new AbortController();
  pitchSession = session;
  pitchResult = null;
  pitchElement('pitchReading').textContent = '—';
  refreshPitchButtons();
  try {
    // Start audio within the user's button gesture (required on mobile).
    const result = await PitchDetector.listen({
      signal: session.signal,
      deviceId: pitchElement('pitchMic').value,
      onProgress: message => { if (pitchSession === session) pitchMessage(message); },
      onReady: async () => {
        // Check authoritative playback state even if another browser started it.
        const playback = await pitchRequest('/api/play', undefined, session.signal);
        if (session.signal.aborted) throw new DOMException('Cancelled', 'AbortError');
        if (playback.playing) throw new Error('Stop instrument playback before detecting a note.');
        if (!strike) {
          pitchMessage(`Tap mallet ${slot}'s note once now. Other notes should stay quiet.`);
          return;
        }
        pitchMessage(`Striking mallet ${slot} at ${effectiveCurrentForSlot(slot)} mA…`);
        const reply = await pitchRequest('/api/strike', {
          address: slot, current_ma: effectiveCurrentForSlot(slot),
        }, session.signal);
        if (!reply.accepted) throw new Error(`Strike was not accepted: ${reply.result_name || 'check homing and connection'}.`);
      },
    });
    if (pitchSession !== session) return;
    pitchResult = { ...result, slot, count: S.count, mapping: JSON.stringify(S.mapping) };
    if (pitchScan && pitchScan.count === S.count && pitchScan.mapping === JSON.stringify(S.mapping)) {
      pitchScan.rows[slot] = { ...result, slot, state: result.assignable ? 'detected' : 'uncertain' };
      renderPitchScan();
    }
    pitchElement('pitchReading').textContent = `${result.name} · ${result.frequency.toFixed(1)} Hz`;
    const duplicate = S.mapping.some((value, index) => index !== slot && value === result.midi);
    pitchMessage(`MIDI ${result.midi} · ${result.cents >= 0 ? '+' : ''}${result.cents.toFixed(1)} cents · stable across ${result.frames} readings. ` +
      (!result.assignable ? 'Between notes: check the tuning and retry before assigning.' :
        `Ready to assign to mallet ${slot}.${duplicate ? ' Another mallet already has this pitch; check that the octave is correct.' : ' Check the octave before assigning.'}`));
  } catch (error) {
    if (pitchSession === session) {
      const message = error.name === 'NotAllowedError' ? 'Microphone permission was denied. Allow it in browser settings and retry.'
        : error.name === 'NotFoundError' ? 'No microphone found. Connect one and retry.'
        : error.name === 'NotReadableError' ? 'The microphone is unavailable or in use. Check its connection and other apps.'
        : error.message;
      pitchMessage(message);
    }
  } finally {
    if (pitchSession === session) {
      session.abort(); // also ends any remaining network request after a timeout
      pitchSession = null;
      refreshPitchButtons();
      refreshPitchMicrophones();
    }
  }
}

function renderPitchScan() {
  const panel = pitchElement('pitchScanResults');
  panel.hidden = !pitchScan;
  if (!pitchScan) { panel.innerHTML = ''; return; }
  const ready = pitchScan.rows.filter(row => row.assignable);
  const counts = new Map();
  ready.forEach(row => counts.set(row.midi, (counts.get(row.midi) || 0) + 1));
  const text = value => String(value).replace(/[&<>"']/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
  panel.innerHTML = `<p>${ready.length} of ${pitchScan.count} detected. Unresolved mallets keep their current assignments when you save.</p>
    <table class="map-table"><thead><tr><th>Mallet</th><th>Current</th><th>Detected</th></tr></thead><tbody>` +
    pitchScan.rows.map(row => {
      const current = pitchScan.original[row.slot];
      let detail = row.state === 'measuring' ? 'Measuring…' : 'Not measured';
      if (row.error) detail = text(row.error);
      if (Number.isFinite(row.frequency)) {
        detail = `${text(row.name)} · ${row.frequency.toFixed(1)} Hz<br><small>${row.cents >= 0 ? '+' : ''}${row.cents.toFixed(1)} cents` +
          `${!row.assignable ? ' · check tuning' : counts.get(row.midi) > 1 ? ' · duplicate pitch: check octave' : ''}</small>`;
      }
      return `<tr><td>${row.slot}</td><td>${current == null ? '—' : text(midiName(current))}</td><td>${detail}</td></tr>`;
    }).join('') + '</tbody></table>';
}

async function checkPitchScanState(scan, signal) {
  if (signal.aborted) throw new DOMException('Cancelled', 'AbortError');
  if (S.count !== scan.count || JSON.stringify(S.mapping) !== scan.mapping) {
    throw new Error('The instrument mapping changed. Start a new scan.');
  }
  const playback = await pitchRequest('/api/play', undefined, signal);
  if (playback.playing || S.playing || S.playStarting || S.firmwareUpdating) {
    throw new Error('Stop playback or finish the firmware update before scanning.');
  }
  const status = await pitchRequest('/api/status', undefined, signal);
  if (status.count !== scan.count) throw new Error('The number of mallets changed. Refresh the instrument and start a new scan.');
  if (signal.aborted) throw new DOMException('Cancelled', 'AbortError');
  return status;
}

async function pitchScanReady(scan, slot, signal) {
  let status, state;
  for (let attempt = 0; attempt < 3; attempt++) {
    status = await checkPitchScanState(scan, signal);
    state = status.slots?.[slot];
    // A competing status poll can mark a perfectly recent observation cached.
    const recent = state && !state.status_error && (!state.status_cached ||
      (Number.isFinite(state.status_age_ms) && state.status_age_ms >= 0 && state.status_age_ms <= 2000));
    if (recent) {
      if (status.homed?.[slot] && state.fault === 0) return;
      if (!status.homed?.[slot] || Number(state.fault) > 0) break;
    }
    if (attempt < 2) await new Promise((resolve, reject) => {
      const cancel = () => { clearTimeout(timer); reject(new DOMException('Cancelled', 'AbortError')); };
      const timer = setTimeout(() => { signal.removeEventListener('abort', cancel); resolve(); }, 200);
      signal.addEventListener('abort', cancel, { once: true });
      if (signal.aborted) cancel();
    });
  }
  throw new Error(!status.homed?.[slot] || Number(state?.fault) > 0
    ? `Mallet ${slot}: home this mallet and clear faults, then retry unresolved.`
    : `Mallet ${slot}: status unavailable after three checks. Check the connection, then retry unresolved.`);
}

async function startPitchScan(retry = false) {
  if (pitchSession || pitchSaving || !S.pitchOpen || !S.count || S.playing || S.playStarting || S.firmwareUpdating) return;
  if (retry && (!pitchScan || pitchScan.count !== S.count || pitchScan.mapping !== JSON.stringify(S.mapping))) {
    pitchMessage('The instrument mapping changed. Start a new scan.');
    return;
  }
  if (!retry) pitchScan = {
    count: S.count, mapping: JSON.stringify(S.mapping), original: S.mapping.slice(),
    rows: Array.from({ length: S.count }, (_, slot) => ({ slot, state: 'pending', assignable: false })),
  };
  const scan = pitchScan;
  const slots = scan.rows.filter(row => !row.assignable).map(row => row.slot);
  if (!slots.length) return;
  let activeSlot = slots[0];
  // Retry currents are temporary; never change the operator's master or trims.
  const currents = slots.map(slot => effectiveCurrentForSlot(slot));
  const session = new AbortController();
  pitchSession = session;
  pitchResult = null;
  pitchElement('pitchReading').textContent = 'Scanning all mallets';
  renderPitchScan();
  refreshPitchButtons();
  try {
    await PitchDetector.scan({
      steps: slots.length, signal: session.signal, deviceId: pitchElement('pitchMic').value, acousticRetries: 1,
      onProgress: (message, index = 0) => {
        if (pitchSession !== session) return;
        activeSlot = slots[index];
        scan.rows[slots[index]].state = 'measuring';
        pitchMessage(`Mallet ${slots[index]} · ${index + 1} of ${slots.length}: ${message}`);
        renderPitchScan();
      },
      onPrepare: index => pitchScanReady(scan, slots[index], session.signal),
      onReady: async (index, attempt = 0, stronger = attempt > 0) => {
        const slot = slots[index];
        if (session.signal.aborted) throw new DOMException('Cancelled', 'AbortError');
        const base = currents[index];
        const current = stronger ? Math.min(STRIKE_MA_MAX, base + Math.min(200, Math.round(base * 0.2))) : base;
        const reply = await pitchRequest('/api/strike', { address: slot, current_ma: current }, session.signal);
        if (!reply.accepted) throw new Error(`Mallet ${slot}: strike rejected (${reply.result_name || 'unknown'}). Check the instrument before retrying.`);
      },
      onResult: (result, index) => {
        if (pitchSession !== session) return;
        const slot = slots[index];
        scan.rows[slot] = { ...result, slot, state: result.assignable ? 'detected' : 'uncertain' };
        renderPitchScan();
      },
    });
    if (pitchSession !== session) return;
    const detected = scan.rows.filter(row => row.assignable).length;
    pitchElement('pitchReading').textContent = `${detected} / ${scan.count} detected`;
    pitchMessage(detected === scan.count ? 'Scan complete. Review the notes and save the detected map.'
      : 'Scan complete. Retry unresolved mallets, or save the successful readings. You can also remeasure a selected mallet above.');
  } catch (error) {
    if (pitchSession === session) {
      scan.rows[activeSlot] = { slot: activeSlot, state: 'stopped', assignable: false, error: error.message };
      renderPitchScan();
      pitchMessage(`Scan stopped: ${error.message} Completed readings are kept for review.`);
    }
  } finally {
    if (pitchSession === session) {
      cancelPitchDetection();
      refreshPitchMicrophones();
    }
  }
}

async function savePitchScan() {
  const scan = pitchScan;
  if (!scan || pitchSession || pitchSaving || !scan.rows.some(row => row.assignable)) return;
  pitchSaving = true;
  refreshPitchButtons();
  try {
    const signal = new AbortController().signal;
    await checkPitchScanState(scan, signal);
    const mapping = scan.original.slice();
    scan.rows.filter(row => row.assignable).forEach(row => { mapping[row.slot] = row.midi; });
    const remote = await pitchRequest('/api/mapping', undefined, signal);
    // Also permit a retry after a successful write whose response was lost.
    if (remote.mapping != null && JSON.stringify(remote.mapping) !== scan.mapping &&
        JSON.stringify(remote.mapping) !== JSON.stringify(mapping)) {
      throw new Error('Another browser changed the saved map. Reload the player before saving.');
    }
    const saved = await pitchRequest('/api/mapping', { mapping }, signal);
    if (!saved.ok || JSON.stringify(saved.mapping) !== JSON.stringify(mapping)) throw new Error('Server did not confirm the mapping.');
    S.mapping = saved.mapping;
    try { localStorage.setItem(STORAGE_KEY, JSON.stringify(S.mapping)); } catch {}
    renderMapTable();
    if (S.composition) showComposition();
    for (const option of pitchElement('pitchSlot').options) {
      const slot = +option.value;
      option.textContent = `Mallet ${slot} · ${S.mapping[slot] == null ? 'unassigned' : midiName(S.mapping[slot])}`;
    }
    pitchMessage(`Saved ${scan.rows.filter(row => row.assignable).length} detected pitches. Unresolved assignments were preserved.`);
    pitchResult = null;
    pitchScan = null;
    renderPitchScan();
  } catch (error) {
    pitchMessage(`Could not confirm save: ${error.message}`);
  } finally {
    pitchSaving = false;
    refreshPitchButtons();
  }
}

async function pitchRequest(path, body, signal) {
  const timeout = new AbortController();
  const cancel = () => timeout.abort();
  signal.addEventListener('abort', cancel, { once: true });
  if (signal.aborted) timeout.abort();
  const timer = setTimeout(cancel, 10000);
  try {
    const response = await fetch(API_BASE + path, {
      method: body === undefined ? 'GET' : 'POST', signal: timeout.signal,
      headers: body === undefined ? {} : { 'Content-Type': 'application/json' },
      ...(body === undefined ? {} : { body: JSON.stringify(body) }),
    });
    const result = await response.json();
    if (!response.ok) throw new Error(result.error || `Request failed (${response.status})`);
    return result;
  } catch (error) {
    if (timeout.signal.aborted && !signal.aborted) throw new Error('Server request timed out. Check the connection and retry.');
    throw error;
  } finally {
    clearTimeout(timer);
    signal.removeEventListener('abort', cancel);
  }
}

async function applyDetectedPitch() {
  const result = pitchResult;
  if (!result?.assignable || pitchSession || pitchSaving || S.playing || S.playStarting || S.firmwareUpdating) return;
  if (S.count !== result.count || JSON.stringify(S.mapping) !== result.mapping || +pitchElement('pitchSlot').value !== result.slot) {
    pitchResult = null;
    pitchMessage('The instrument mapping changed. Detect this mallet again.');
    refreshPitchButtons();
    return;
  }
  pitchSaving = true;
  refreshPitchButtons();
  const mapping = S.mapping.slice();
  mapping[result.slot] = result.midi;
  try {
    // Do not display a saved assignment until the server confirms persistence.
    const saved = await pitchRequest('/api/mapping', { mapping }, new AbortController().signal);
    if (!saved.ok || JSON.stringify(saved.mapping) !== JSON.stringify(mapping)) throw new Error('Server did not confirm the mapping.');
    S.mapping = saved.mapping;
    try { localStorage.setItem(STORAGE_KEY, JSON.stringify(S.mapping)); } catch {}
    renderMapTable();
    if (S.composition) showComposition();
    pitchResult = null;
    pitchScan = null;
    renderPitchScan();
    pitchElement('pitchSlot').selectedOptions[0].textContent = `Mallet ${result.slot} · ${result.name}`;
    pitchMessage(`Saved ${result.name} for mallet ${result.slot}. Choose the next mallet to continue.`);
  } catch (error) {
    pitchMessage(`Could not confirm save: ${error.message} Retry Assign, or refresh the page to check the stored mapping.`);
  } finally {
    pitchSaving = false;
    refreshPitchButtons();
  }
}

async function refreshPitchMicrophones() {
  if (!navigator.mediaDevices?.enumerateDevices || pitchSession) return;
  try {
    const devices = await navigator.mediaDevices.enumerateDevices();
    if (pitchSession) return;
    const select = pitchElement('pitchMic'), selected = select.value;
    select.replaceChildren(new Option('System default microphone', ''));
    for (const device of devices.filter(d => d.kind === 'audioinput' && d.deviceId && d.deviceId !== 'default')) {
      select.add(new Option(device.label || 'Microphone', device.deviceId));
    }
    select.value = [...select.options].some(option => option.value === selected) ? selected : '';
  } catch (error) { log('Microphone list unavailable: ' + error.message); }
}

document.addEventListener('DOMContentLoaded', () => {
  pitchElement('detectPitchBtn').addEventListener('click', openPitchDetection);
  pitchElement('pitchStrike').addEventListener('click', () => startPitchDetection(true));
  pitchElement('pitchListen').addEventListener('click', () => startPitchDetection(false));
  pitchElement('pitchApply').addEventListener('click', applyDetectedPitch);
  pitchElement('pitchScanStart').addEventListener('click', () => startPitchScan());
  pitchElement('pitchScanRetry').addEventListener('click', () => startPitchScan(true));
  pitchElement('pitchScanSave').addEventListener('click', savePitchScan);
  pitchElement('pitchScanStop').addEventListener('click', () => {
    cancelPitchDetection();
    pitchMessage('Scan stopped. Completed readings are kept; a strike already sent will still complete.');
  });
  pitchElement('pitchCancel').addEventListener('click', () => {
    cancelPitchDetection();
    pitchMessage('Listening stopped. A strike already sent will still complete.');
  });
  pitchElement('pitchClose').addEventListener('click', () => { if (!pitchSaving) pitchElement('pitchDialog').close(); });
  pitchElement('pitchDialog').addEventListener('cancel', event => { if (pitchSaving) event.preventDefault(); });
  pitchElement('pitchDialog').addEventListener('close', () => {
    cancelPitchDetection();
    S.pitchOpen = false;
    pitchResult = null;
    pitchScan = null;
    renderPitchScan();
  });
  pitchElement('pitchSlot').addEventListener('change', () => {
    pitchResult = null;
    pitchElement('pitchReading').textContent = '—';
    pitchMessage('Let previous notes fade, then detect this mallet.');
    refreshPitchButtons();
  });
  document.addEventListener('visibilitychange', () => {
    if (document.hidden && pitchSession) {
      cancelPitchDetection();
      pitchMessage('Listening stopped because this page was hidden. Keep it open while measuring.');
    }
  });
  window.addEventListener('pagehide', cancelPitchDetection);
});
