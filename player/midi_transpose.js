/* Pure MIDI adaptation helpers shared by the browser and Node regression tests. */
const MidiTranspose = (() => {
  const MIN_GAP_MS = 50;
  const MODES = ["strict", "fold", "adapt"];

  function voiceKey(event) {
    return `${event.track ?? 0}:${event.channel ?? 0}`;
  }

  function targetPitches(mapping, count) {
    return [...new Set(mapping.slice(0, count).filter(p =>
      Number.isInteger(p) && p >= 0 && p <= 127))].sort((a, b) => a - b);
  }

  function mapPitch(pitch, shift, target, mode) {
    const shifted = pitch + shift;
    if (target.includes(shifted)) return { pitch: shifted, kind: "exact", distance: 0 };
    const sameClass = target.filter(p => ((p - shifted) % 12) === 0);
    const candidates = mode === "strict" ? [] : sameClass;
    if (candidates.length) {
      const mapped = candidates.reduce((best, p) =>
        Math.abs(p - shifted) < Math.abs(best - shifted) ? p : best);
      return { pitch: mapped, kind: "folded", distance: Math.abs(mapped - shifted) };
    }
    if (mode === "adapt") {
      // Prefer the closest pitch class, then the nearest available register.
      const distance = p => Math.min(((p - shifted) % 12 + 12) % 12,
        ((shifted - p) % 12 + 12) % 12);
      const mapped = target.reduce((best, p) =>
        distance(p) < distance(best) || (distance(p) === distance(best) &&
        Math.abs(p - shifted) < Math.abs(best - shifted)) ? p : best);
      return { pitch: mapped, kind: "substituted", distance: Math.abs(mapped - shifted) };
    }
    return { pitch: Math.max(0, Math.min(127, shifted)), kind: "unmapped", distance: 0 };
  }

  function findBest(events, pitches, options = {}) {
    const mode = options.mode ?? "fold";
    if (!MODES.includes(mode)) throw new Error("Unknown transposition mode");
    const target = targetPitches(pitches, pitches.length);
    if (!target.length) throw new Error("Assign pitches to connected actuators first");
    const selected = events.filter(e => !options.voices || options.voices.has(voiceKey(e)))
      .slice().sort((a, b) => a.timeMs - b.timeMs);
    if (!selected.length) throw new Error("Select at least one track with notes");
    const histogram = new Map();
    for (const e of selected) histogram.set(e.pitch, (histogram.get(e.pitch) || 0) + 1);
    let best = null;
    // All twelve pitch-class shifts, in every octave that can overlap MIDI's
    // finite 0..127 range. No key estimate or instrument-specific scale table.
    for (let shift = -127; shift <= 127; shift++) {
      const lookup = new Map();
      const counts = { exact: 0, folded: 0, substituted: 0, unmapped: 0 };
      let distance = 0;
      for (const [pitch, count] of histogram) {
        const mapped = mapPitch(pitch, shift, target, mode);
        lookup.set(pitch, mapped);
        counts[mapped.kind] += count;
        distance += mapped.distance * count;
      }
      // Preserve pitch classes first, then register and minimize travel.
      // Timing and playback speed do not influence the chosen transposition.
      const score = [counts.unmapped + counts.substituted, counts.unmapped,
        -counts.exact, distance, Math.abs(shift), shift];
      const better = !best || score.some((value, i) =>
        value < best.score[i] && score.slice(0, i).every((v, j) => v === best.score[j]));
      if (better) best = { shift, counts, lookup, score };
    }
    return {
      shift: best.shift,
      counts: best.counts,
      excluded: events.length - selected.length,
      target,
      mode,
      events: selected.map(e => {
        const mapped = best.lookup.get(e.pitch);
        return { ...e, originalPitch: e.pitch, pitch: mapped.pitch,
          transposeKind: mapped.kind };
      }),
    };
  }

  // Called after the player's simultaneous-note merge. Keep the earlier hit;
  // do not move later notes in time or rewrite their velocities.
  function enforceRestrike(schedule, tempoScale) {
    const previous = new Map();
    for (const e of schedule) {
      if (e.slot < 0 || e.suppressed) continue;
      const ms = Math.round(e.timeMs / tempoScale);
      const last = previous.get(e.slot);
      if (last !== undefined && ms - last < MIN_GAP_MS) {
        e.suppressed = true;
        e.suppressedReason = "restrike";
      } else previous.set(e.slot, ms);
    }
    return schedule;
  }

  return { MIN_GAP_MS, voiceKey, targetPitches, mapPitch, findBest, enforceRestrike };
})();

if (typeof module !== "undefined") module.exports = MidiTranspose;
