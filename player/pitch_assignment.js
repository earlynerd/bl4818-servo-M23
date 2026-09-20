/* Microphone workflow for the existing MIDI player's slot mapping. */
'use strict';
let pitchSession = null;
let pitchResult = null;
let pitchSaving = false;
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
}

function openPitchDetection() {
  if (S.playing || S.playStarting || S.firmwareUpdating) {
    showError('Stop playback or finish the firmware update before detecting pitches.');
    return;
  }
  stopVoiceAudition();
  S.pitchOpen = true;
  pitchResult = null;
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
