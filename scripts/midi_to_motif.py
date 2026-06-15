#!/usr/bin/env python3
"""
Convert a Standard MIDI File (.mid) into the motif event list that the
ring MIDI server's friendly API speaks (see midi_server_api.md, POST
/api/play).

Stdlib only -- no `mido` dependency, so this runs anywhere a plain Python
interpreter does, matching the rest of scripts/. It parses SMF format 0
and 1, applies the tempo map (including mid-song tempo changes), and emits
a flat list of:

    {"t_ms": <int>, "pitch": <0-127>, "velocity": <0-127>}

Two ways to use this file:

    # As a library --
    from midi_to_motif import midi_file_to_events
    events = midi_file_to_events("songs/sly.mid", min_gap_ms=150)

    # As a CLI ("convert midi to json") --
    python scripts/midi_to_motif.py songs/sly.mid
    python scripts/midi_to_motif.py songs/sly.mid --transpose -12 --max-ms 6000
    python scripts/midi_to_motif.py songs/sly.mid -o sly.json

The ring has a limited number of tines and the firmware needs ~150 ms to
re-home a mallet before it can strike the same slot again. This converter
defaults to thinning out notes that land on the same pitch closer than
`min_gap_ms` apart so a dense MIDI track stays physically playable. It
does NOT know what pitches are actually mapped on your ring -- that
"snap to nearest available tine" step lives in ring_welcome.py, which can
query the live mapping. Run the output through the browser player at
http://localhost:8765/ to hear it before wiring it to a coworker.
"""

from __future__ import annotations

import argparse
import json
import sys
from typing import NamedTuple


# ---------------------------------------------------------------------------
# Low-level SMF parsing
# ---------------------------------------------------------------------------


class _Note(NamedTuple):
    tick: int        # absolute tick from start of song
    pitch: int       # 0-127
    velocity: int    # 1-127 (note-ons only; note-offs are dropped)
    channel: int     # 0-15


class _Tempo(NamedTuple):
    tick: int        # absolute tick where this tempo takes effect
    us_per_qn: int   # microseconds per quarter note


def _read_vlq(data: bytes, i: int) -> tuple[int, int]:
    """Read a MIDI variable-length quantity at offset i. Returns (value, new_i)."""
    value = 0
    while True:
        b = data[i]
        i += 1
        value = (value << 7) | (b & 0x7F)
        if not (b & 0x80):
            return value, i


def _split_chunks(data: bytes) -> tuple[int, list[bytes]]:
    """Parse the MThd header and slice out each MTrk body.

    Returns (division, tracks). `division` is the raw 16-bit division field
    from the header -- positive is ticks-per-quarter-note, the high bit set
    means SMPTE timing (handled in _build_timeline).
    """
    if data[:4] != b"MThd":
        raise ValueError("not a Standard MIDI File (missing 'MThd' magic)")
    header_len = int.from_bytes(data[4:8], "big")
    if header_len < 6:
        raise ValueError(f"implausible MThd length {header_len}")
    ntrks = int.from_bytes(data[10:12], "big")
    division = int.from_bytes(data[12:14], "big")

    tracks: list[bytes] = []
    pos = 8 + header_len
    while pos + 8 <= len(data) and len(tracks) < ntrks:
        chunk_id = data[pos:pos + 4]
        chunk_len = int.from_bytes(data[pos + 4:pos + 8], "big")
        body = data[pos + 8:pos + 8 + chunk_len]
        pos += 8 + chunk_len
        # Per the spec, ignore (but skip over) any chunk type we don't know.
        if chunk_id == b"MTrk":
            tracks.append(body)
    if not tracks:
        raise ValueError("no MTrk track chunks found")
    return division, tracks


def _parse_track(track: bytes) -> tuple[list[_Note], list[_Tempo]]:
    """Parse one MTrk body into its note-on and set-tempo events.

    Handles running status. Note-offs (0x80) and note-ons with velocity 0
    are dropped -- the ring can only strike, never sustain or release, so
    only the attack matters.
    """
    notes: list[_Note] = []
    tempos: list[_Tempo] = []
    i = 0
    abs_tick = 0
    status = 0
    n = len(track)
    while i < n:
        delta, i = _read_vlq(track, i)
        abs_tick += delta
        if i >= n:
            break
        b = track[i]
        if b & 0x80:
            status = b
            i += 1
        # else: running status -- reuse previous `status`, `b` is the first
        # data byte and i is left pointing at it.

        if status == 0xFF:  # meta event
            meta_type = track[i]
            i += 1
            length, i = _read_vlq(track, i)
            payload = track[i:i + length]
            i += length
            if meta_type == 0x51 and length == 3:  # set tempo
                tempos.append(_Tempo(abs_tick, int.from_bytes(payload, "big")))
            status = 0  # meta cancels running status
        elif status in (0xF0, 0xF7):  # sysex / escape
            length, i = _read_vlq(track, i)
            i += length
            status = 0  # sysex cancels running status
        else:
            event = status & 0xF0
            channel = status & 0x0F
            if event in (0x80, 0x90, 0xA0, 0xB0, 0xE0):  # two data bytes
                d1 = track[i]
                d2 = track[i + 1]
                i += 2
                if event == 0x90 and d2 > 0:  # genuine note-on
                    notes.append(_Note(abs_tick, d1, d2, channel))
                # 0x80 and 0x90-vel-0 are note-offs -> ignored
            elif event in (0xC0, 0xD0):  # one data byte
                i += 1
            else:  # unknown status byte; bail on this track to avoid runaway
                break
    return notes, tempos


def _build_timeline(notes: list[_Note], tempos: list[_Tempo], division: int) -> list[_Note]:
    """Convert absolute-tick notes to absolute-millisecond notes.

    `_Note.tick` is reused as the resolved millisecond timestamp in the
    returned list. The tempo map is applied piecewise so mid-song tempo
    changes are honored.
    """
    if not notes:
        return []

    if division & 0x8000:
        # SMPTE timing: ticks are an absolute time base, tempo is irrelevant.
        frames_per_sec = 256 - (division >> 8)          # stored as negative byte
        ticks_per_frame = division & 0xFF
        ticks_per_sec = frames_per_sec * ticks_per_frame or 1
        return [n._replace(tick=int(round(n.tick * 1000.0 / ticks_per_sec))) for n in notes]

    ticks_per_qn = division if division > 0 else 480

    # Walk a merged, tick-sorted stream of tempo changes and notes, carrying
    # an accumulated millisecond clock so each segment uses the tempo in force.
    tempo_events = sorted(tempos, key=lambda t: t.tick)
    sorted_notes = sorted(range(len(notes)), key=lambda k: notes[k].tick)

    cur_us_per_qn = 500000  # MIDI default = 120 BPM
    last_tick = 0
    accrued_ms = 0.0
    ti = 0
    resolved: list[_Note] = []

    def advance_to(target_tick: int, accrued: float, frm: int, tempo: int) -> float:
        return accrued + (target_tick - frm) * (tempo / ticks_per_qn) / 1000.0

    for ni in sorted_notes:
        note = notes[ni]
        # Apply every tempo change that occurs at or before this note's tick.
        while ti < len(tempo_events) and tempo_events[ti].tick <= note.tick:
            tev = tempo_events[ti]
            accrued_ms = advance_to(tev.tick, accrued_ms, last_tick, cur_us_per_qn)
            last_tick = tev.tick
            cur_us_per_qn = tev.us_per_qn
            ti += 1
        ms = advance_to(note.tick, accrued_ms, last_tick, cur_us_per_qn)
        resolved.append(note._replace(tick=int(round(ms))))
    # `resolved` follows tick order, which is also time order here.
    return resolved


# ---------------------------------------------------------------------------
# Public conversion API
# ---------------------------------------------------------------------------


def midi_bytes_to_events(
    data: bytes,
    *,
    transpose: int = 0,
    drop_drums: bool = True,
    channels: list[int] | None = None,
    min_gap_ms: int = 150,
    max_ms: int | None = None,
    max_notes: int | None = None,
    default_velocity: int | None = None,
) -> list[dict]:
    """Parse raw .mid bytes into a sorted motif event list.

    Args:
        transpose: semitones to shift every note (e.g. -12 = down an octave).
        drop_drums: drop MIDI channel 10 (index 9), the percussion channel.
        channels: if given, keep only these 0-based channel indexes.
        min_gap_ms: drop a note if the same pitch was struck less than this
            many ms earlier (keeps the ring physically playable). 0 disables.
        max_ms: truncate the song at this many ms from its (normalized) start.
        max_notes: keep at most this many notes (earliest first).
        default_velocity: if set, override every note's velocity with this.

    Returns events as {"t_ms", "pitch", "velocity"}, sorted by t_ms, with
    the first note normalized to t_ms == 0.
    """
    division, tracks = _split_chunks(data)
    all_notes: list[_Note] = []
    all_tempos: list[_Tempo] = []
    for track in tracks:
        notes, tempos = _parse_track(track)
        all_notes.extend(notes)
        all_tempos.extend(tempos)

    timed = _build_timeline(all_notes, all_tempos, division)
    if not timed:
        return []

    # Channel filtering.
    if drop_drums:
        timed = [n for n in timed if n.channel != 9]
    if channels is not None:
        keep = set(channels)
        timed = [n for n in timed if n.channel in keep]
    if not timed:
        return []

    # Normalize so the first note is at t=0, then sort by time.
    timed.sort(key=lambda n: n.tick)
    origin = timed[0].tick

    last_struck: dict[int, int] = {}  # pitch -> last kept t_ms
    events: list[dict] = []
    for n in timed:
        t_ms = n.tick - origin
        if max_ms is not None and t_ms > max_ms:
            break
        pitch = n.pitch + transpose
        if not (0 <= pitch <= 127):
            continue  # transposed out of MIDI range
        if min_gap_ms > 0:
            prev = last_struck.get(pitch)
            if prev is not None and (t_ms - prev) < min_gap_ms:
                continue
            last_struck[pitch] = t_ms
        velocity = default_velocity if default_velocity is not None else n.velocity
        velocity = max(1, min(127, int(velocity)))
        events.append({"t_ms": int(t_ms), "pitch": int(pitch), "velocity": velocity})
        if max_notes is not None and len(events) >= max_notes:
            break

    return events


def midi_file_to_events(path: str, **kwargs) -> list[dict]:
    """Convenience wrapper: read a .mid file from disk and convert it.

    Accepts the same keyword arguments as midi_bytes_to_events.
    """
    with open(path, "rb") as fh:
        data = fh.read()
    return midi_bytes_to_events(data, **kwargs)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description="Convert a .mid file to ring-server motif events (JSON).",
    )
    ap.add_argument("midi", help="path to the .mid / .midi file")
    ap.add_argument("-o", "--output", help="write JSON here instead of stdout")
    ap.add_argument("--transpose", type=int, default=0,
                    help="semitones to shift (negative = down)")
    ap.add_argument("--keep-drums", action="store_true",
                    help="keep MIDI channel 10 (percussion); dropped by default")
    ap.add_argument("--channels", type=int, nargs="+", metavar="CH",
                    help="keep only these 0-based channels")
    ap.add_argument("--min-gap", type=int, default=150, metavar="MS",
                    help="drop same-pitch notes closer than this (default 150; 0 disables)")
    ap.add_argument("--max-ms", type=int, default=None, metavar="MS",
                    help="truncate the song at this many ms")
    ap.add_argument("--max-notes", type=int, default=None, metavar="N",
                    help="keep at most N notes")
    ap.add_argument("--velocity", type=int, default=None, metavar="V",
                    help="force every note to this velocity (1-127)")
    ap.add_argument("--name", default=None, help="optional motif name to embed")
    ap.add_argument("--master-current-ma", type=int, default=None,
                    help="optional master current to embed for /api/play")
    args = ap.parse_args(argv)

    try:
        events = midi_file_to_events(
            args.midi,
            transpose=args.transpose,
            drop_drums=not args.keep_drums,
            channels=args.channels,
            min_gap_ms=args.min_gap,
            max_ms=args.max_ms,
            max_notes=args.max_notes,
            default_velocity=args.velocity,
        )
    except (OSError, ValueError, IndexError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1

    body: dict = {"events": events}
    if args.name is not None:
        body["name"] = args.name
    if args.master_current_ma is not None:
        body["master_current_ma"] = args.master_current_ma

    text = json.dumps(body, indent=2)
    if args.output:
        with open(args.output, "w", encoding="utf-8") as fh:
            fh.write(text + "\n")
        print(f"wrote {len(events)} events to {args.output}", file=sys.stderr)
    else:
        print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
