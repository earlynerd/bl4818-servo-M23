/* Trigger-to-impact latency versus strike current, shared by the player,
 * the looper and Node tests. Mirrors LatencyTracker in
 * scripts/ring_midi_server.py — keep the two in step.
 *
 * Tables are plain objects {binIndex: ms}. Bins are log-spaced, four per
 * octave from 25 mA (25..3200 mA), so the commonly played 100-500 mA range
 * has ten bins where latency changes fastest. A sample is scaled to its bin's
 * centre current with the constant-torque relation t ~ I^-1/2 (at most a 9%
 * current shift) before the EMA. Estimates interpolate geometrically between
 * learned bins and follow the same relation beyond them, so one sample is
 * reproduced exactly at its own current.
 */
const LatencyModel = (() => {
  const EMA_ALPHA = 0.3;
  const BASE_MA = 25;
  const PER_OCTAVE = 4;
  const BIN_COUNT = 29;

  // Fractional bin position of a current, or null for no usable current.
  function position(currentMa) {
    if (!Number.isFinite(currentMa) || currentMa === 0) return null;
    const x = PER_OCTAVE * Math.log2(Math.max(Math.abs(currentMa), BASE_MA) / BASE_MA);
    return Math.min(x, BIN_COUNT - 1);
  }

  // t ~ I^-1/2: moving `bins` toward higher current scales latency by this.
  function prior(bins) {
    return Math.pow(2, -bins / (2 * PER_OCTAVE));
  }

  function learned(table) {
    return table ? Object.keys(table).map(Number).filter(i => Number.isFinite(table[i])) : [];
  }

  // Fold one measurement into a table; returns false when unusable.
  function learn(table, currentMa, sampleMs) {
    const x = position(currentMa);
    if (x === null || !Number.isFinite(sampleMs) || sampleMs <= 0) return false;
    const i = Math.floor(x + 0.5);
    const atCentre = sampleMs / prior(x - i);
    const prev = table[i];
    table[i] = Number.isFinite(prev) ? prev + EMA_ALPHA * (atCentre - prev) : atCentre;
    return true;
  }

  // Learned bins either side of position x (-1 when missing).
  function bracket(table, x) {
    let lo = -1, hi = -1;
    for (const i of learned(table)) {
      if (i <= x && i > lo) lo = i;
      if (i >= x && (hi < 0 || i < hi)) hi = i;
    }
    return { lo, hi };
  }

  // Table curve at position x: geometric interpolation inside the learned
  // range, the t ~ I^-1/2 prior from the nearest bin outside it.
  function curve(table, x) {
    const { lo, hi } = bracket(table, x);
    if (lo < 0 && hi < 0) return null;
    if (lo < 0) return table[hi] * prior(x - hi);
    if (hi < 0) return table[lo] * prior(x - lo);
    if (lo === hi) return table[lo];
    const f = (x - lo) / (hi - lo);
    return Math.pow(table[lo], 1 - f) * Math.pow(table[hi], f);
  }

  // This slot where it brackets the current; otherwise this slot's nearest
  // bin carried along the all-slot curve; otherwise the all-slot curve.
  // Returns null when neither table has data.
  function estimate(slotTable, sharedTable, currentMa) {
    const x = position(currentMa);
    if (x === null) return null;
    const { lo, hi } = bracket(slotTable, x);
    if (lo >= 0 && hi >= 0) return curve(slotTable, x);
    if (lo >= 0 || hi >= 0) {
      const anchor = lo >= 0 ? lo : hi;
      const here = curve(sharedTable, x), there = curve(sharedTable, anchor);
      return (here !== null && there !== null)
        ? slotTable[anchor] * here / there
        : slotTable[anchor] * prior(x - anchor);
    }
    return curve(sharedTable, x);
  }

  // Which data an estimate comes from, for display.
  function source(slotTable, sharedTable, currentMa) {
    const x = position(currentMa);
    if (x === null) return "default";
    const { lo, hi } = bracket(slotTable, x);
    if (lo >= 0 && hi >= 0) return "this slot";
    if (lo >= 0 || hi >= 0) return "this slot + shared shape";
    return learned(sharedTable).length ? "shared" : "default";
  }

  return { EMA_ALPHA, BASE_MA, PER_OCTAVE, BIN_COUNT, position, learn, curve, estimate,
           source, learned };
})();

if (typeof module !== "undefined") module.exports = LatencyModel;
