#!/usr/bin/env python3
"""
Play a MIDI drum track on the mallet ring.

Reads General-MIDI drum note numbers (35-59 on channel 10) from a
standard MIDI file and fires a strike on the mallet assigned to that
drum's role, at every note-on. Rest/hold time is ignored — the only
thing that matters is *when* each note-on occurs.

The mallet count determines how many drum roles we can cover; smaller
kits fold extra roles onto neighboring mallets (tom -> snare, ride ->
hat, etc). The script prints the full role map and a note-by-note
assignment table before it starts playing so you can arrange the
physical objects before committing to the run.

Examples:
    python ring_midi_drummer.py song.mid
    python ring_midi_drummer.py funky.mid --current 1800 --tempo-scale 0.75
    python ring_midi_drummer.py track.mid --channel all --start 30 --max-seconds 60
    python ring_midi_drummer.py song.mid --dry-run   # print map, no strikes
"""

from __future__ import annotations

import argparse
import sys
import time
from collections import Counter

try:
    import mido
except ImportError:
    print("This script needs the 'mido' package. Install with: pip install mido",
          file=sys.stderr)
    raise SystemExit(1)

from ring_bus import (
    CommandAck,
    RingClientV2,
    auto_detect_port,
    DEFAULT_BAUD,
    REPLY_MODE_ACK,
)
from ring_drumbeat import home_all, wait_all_idle


# ---------------------------------------------------------------------------
# General-MIDI drum map
# ---------------------------------------------------------------------------

class Role:
    KICK = "kick"
    SNARE = "snare"
    GHOST = "snare-ghost"   # side stick, rim, claps, aux percussion
    HAT = "hat"
    TOM = "tom"
    CRASH = "crash"
    RIDE = "ride"


GM_NOTE_TO_ROLE: dict[int, str] = {
    35: Role.KICK,   # Acoustic Bass Drum
    36: Role.KICK,   # Bass Drum 1
    37: Role.GHOST,  # Side Stick
    38: Role.SNARE,  # Acoustic Snare
    39: Role.GHOST,  # Hand Clap
    40: Role.SNARE,  # Electric Snare
    41: Role.TOM,    # Low Floor Tom
    42: Role.HAT,    # Closed Hi-Hat
    43: Role.TOM,    # High Floor Tom
    44: Role.HAT,    # Pedal Hi-Hat
    45: Role.TOM,    # Low Tom
    46: Role.HAT,    # Open Hi-Hat
    47: Role.TOM,    # Low-Mid Tom
    48: Role.TOM,    # Hi Mid Tom
    49: Role.CRASH,  # Crash Cymbal 1
    50: Role.TOM,    # High Tom
    51: Role.RIDE,   # Ride Cymbal 1
    52: Role.CRASH,  # Chinese Cymbal
    53: Role.RIDE,   # Ride Bell
    54: Role.GHOST,  # Tambourine
    55: Role.CRASH,  # Splash Cymbal
    56: Role.GHOST,  # Cowbell
    57: Role.CRASH,  # Crash Cymbal 2
    58: Role.GHOST,  # Vibraslap
    59: Role.RIDE,   # Ride Cymbal 2
}


GM_NOTE_NAMES: dict[int, str] = {
    35: "Acoustic Bass Drum", 36: "Bass Drum 1",
    37: "Side Stick",         38: "Acoustic Snare",
    39: "Hand Clap",          40: "Electric Snare",
    41: "Low Floor Tom",      42: "Closed Hi-Hat",
    43: "High Floor Tom",     44: "Pedal Hi-Hat",
    45: "Low Tom",            46: "Open Hi-Hat",
    47: "Low-Mid Tom",        48: "Hi Mid Tom",
    49: "Crash Cymbal 1",     50: "High Tom",
    51: "Ride Cymbal 1",      52: "Chinese Cymbal",
    53: "Ride Bell",          54: "Tambourine",
    55: "Splash Cymbal",      56: "Cowbell",
    57: "Crash Cymbal 2",     58: "Vibraslap",
    59: "Ride Cymbal 2",
}


# For N mallets, which roles do we cover (in mallet-index order).
# Matches the voice ordering used by ring_drumbeat.py so the physical
# arrangement can stay the same between scripts.
MALLET_ROLES: dict[int, list[str]] = {
    1: [Role.KICK],
    2: [Role.KICK, Role.SNARE],
    3: [Role.KICK, Role.SNARE, Role.HAT],
    4: [Role.KICK, Role.SNARE, Role.HAT, Role.CRASH],
    5: [Role.KICK, Role.SNARE, Role.HAT, Role.TOM, Role.CRASH],
    6: [Role.KICK, Role.SNARE, Role.GHOST, Role.HAT, Role.TOM, Role.CRASH],
    7: [Role.KICK, Role.SNARE, Role.GHOST, Role.HAT, Role.TOM, Role.CRASH, Role.RIDE],
    8: [Role.KICK, Role.SNARE, Role.GHOST, Role.HAT, Role.TOM, Role.TOM, Role.CRASH, Role.RIDE],
}


# If a role isn't in the mallet set, walk this chain for the closest
# surrogate. Chains go all the way down to kick so a 1-mallet setup
# still gets every note folded onto something.
ROLE_FALLBACK: dict[str, list[str]] = {
    Role.GHOST: [Role.SNARE, Role.HAT, Role.KICK],
    Role.TOM:   [Role.SNARE, Role.HAT, Role.KICK],
    Role.RIDE:  [Role.HAT, Role.CRASH, Role.SNARE, Role.KICK],
    Role.CRASH: [Role.HAT, Role.SNARE, Role.KICK],
    Role.HAT:   [Role.SNARE, Role.KICK],
    Role.SNARE: [Role.KICK],
    Role.KICK:  [Role.SNARE],
}


def build_role_to_slot(mallet_count: int) -> dict[str, int]:
    """Return {role -> first mallet index that plays it}."""
    if mallet_count <= 0:
        return {}
    max_defined = max(MALLET_ROLES)
    key = min(mallet_count, max_defined)
    roles = MALLET_ROLES[key]
    role_to_slot: dict[str, int] = {}
    for slot, role in enumerate(roles):
        role_to_slot.setdefault(role, slot)
    return role_to_slot


def resolve_note_to_slot(note: int, role_to_slot: dict[str, int]) -> int | None:
    role = GM_NOTE_TO_ROLE.get(note)
    if role is None:
        return None
    if role in role_to_slot:
        return role_to_slot[role]
    for fb in ROLE_FALLBACK.get(role, []):
        if fb in role_to_slot:
            return role_to_slot[fb]
    return None


# ---------------------------------------------------------------------------
# MIDI scanning / pretty-printing
# ---------------------------------------------------------------------------

def scan_note_histogram(midi_path: str, channel: int | None) -> tuple[Counter, float]:
    """Count note-on events (velocity > 0) and total playable length in seconds."""
    mid = mido.MidiFile(midi_path)
    hist: Counter = Counter()
    length = 0.0
    for msg in mid:
        length += msg.time
        if msg.type != "note_on" or msg.velocity == 0:
            continue
        if channel is not None and msg.channel != channel:
            continue
        hist[msg.note] += 1
    return hist, length


def detect_drum_channel(midi_path: str) -> int | None:
    """Pick the channel with the most GM-drum note-ons (35-59)."""
    counts: Counter = Counter()
    mid = mido.MidiFile(midi_path)
    for msg in mid:
        if msg.type == "note_on" and msg.velocity > 0 and 35 <= msg.note <= 59:
            counts[msg.channel] += 1
    if not counts:
        return None
    return counts.most_common(1)[0][0]


def print_role_map(addresses: list[int], role_to_slot: dict[str, int]) -> None:
    slot_to_role = {slot: role for role, slot in role_to_slot.items()}
    print("  Role map:")
    for slot, addr in enumerate(addresses):
        role = slot_to_role.get(slot)
        if role is None:
            print(f"    addr {addr}  ->  (unused — not enough roles for this mallet count)")
        else:
            print(f"    addr {addr}  ->  {role}")


def print_note_assignments(
    addresses: list[int],
    role_to_slot: dict[str, int],
    hist: Counter,
) -> None:
    print("  Note assignments for this song (sorted by frequency):")
    ordered = sorted(hist.items(), key=lambda kv: (-kv[1], kv[0]))
    for note, count in ordered:
        name = GM_NOTE_NAMES.get(note, f"note {note}")
        slot = resolve_note_to_slot(note, role_to_slot)
        if slot is None or slot >= len(addresses):
            where = "DROPPED (no mallet)"
        else:
            role = GM_NOTE_TO_ROLE.get(note, "?")
            resolved_role = next(
                (r for r, s in role_to_slot.items() if s == slot), "?"
            )
            if role == resolved_role:
                where = f"addr {addresses[slot]} ({role})"
            else:
                where = f"addr {addresses[slot]} ({resolved_role}, folded from {role})"
        print(f"    {note:3d}  {name:<22}  x{count:<5}  -> {where}")


# ---------------------------------------------------------------------------
# Playback
# ---------------------------------------------------------------------------

def play_midi(
    client: RingClientV2,
    addresses: list[int],
    midi_path: str,
    channel: int | None,
    current_ma: int,
    role_to_slot: dict[str, int],
    tempo_scale: float,
    start_s: float,
    max_seconds: float | None,
    dry_run: bool,
    loops: int = 1,
    song_length_s: float = 0.0,
) -> tuple[int, int, int, int]:
    """Returns (accepts, rejects, dropped, loops_completed).

    loops <= 0 means 'repeat forever until Ctrl+C'. song_length_s is the
    scanned full duration of the MIDI file (including any trailing
    silence the file encodes); it fixes the per-loop period so boundary
    beats line up seamlessly instead of slamming the downbeat of loop
    N+1 right after loop N's last note-on.
    """
    accepts = 0
    rejects = 0
    dropped = 0
    infinite = loops <= 0
    loops_completed = 0

    # Per-loop wall-clock span — each loop's events are offset by
    # loops_completed * loop_span_wall so loops sit on a continuous timeline.
    effective_song_span = song_length_s - start_s
    if max_seconds is not None:
        effective_song_span = min(effective_song_span, max_seconds)
    effective_song_span = max(effective_song_span, 0.0)
    loop_span_wall = effective_song_span / max(tempo_scale, 1e-6)

    wall_start = time.monotonic()

    try:
        while infinite or loops_completed < loops:
            if infinite or loops > 1:
                total = "inf" if infinite else str(loops)
                print(f"\n--- loop {loops_completed + 1}/{total} ---")

            song_time = 0.0
            loop_offset_wall = loops_completed * loop_span_wall

            mid = mido.MidiFile(midi_path)
            for msg in mid:
                song_time += msg.time
                if song_time < start_s:
                    continue
                if max_seconds is not None and (song_time - start_s) > max_seconds:
                    break

                if msg.type != "note_on" or msg.velocity == 0:
                    continue
                if channel is not None and msg.channel != channel:
                    continue

                target_wall = ((song_time - start_s) / max(tempo_scale, 1e-6)
                               + loop_offset_wall)
                now_offset = time.monotonic() - wall_start
                sleep_for = target_wall - now_offset
                if sleep_for > 0:
                    time.sleep(sleep_for)

                slot = resolve_note_to_slot(msg.note, role_to_slot)
                elapsed_ms = (time.monotonic() - wall_start) * 1000.0
                if slot is None or slot >= len(addresses):
                    dropped += 1
                    print(f"  t={elapsed_ms:8.1f}ms  note={msg.note:3d}  DROPPED")
                    continue

                addr = addresses[slot]

                if dry_run:
                    accepts += 1
                    print(f"  t={elapsed_ms:8.1f}ms  note={msg.note:3d}  -> addr {addr}  (dry)")
                    continue

                reply = client.strike(addr, current_ma, reply_mode=REPLY_MODE_ACK)
                if isinstance(reply, CommandAck):
                    if reply.accepted:
                        accepts += 1
                        tag = "OK" if reply.result == 0 else "RETRIG"
                    else:
                        rejects += 1
                        tag = reply.result_name
                else:
                    rejects += 1
                    tag = "NO-ACK"
                print(f"  t={elapsed_ms:8.1f}ms  note={msg.note:3d}  -> addr {addr}  {tag}")

            loops_completed += 1
    except KeyboardInterrupt:
        print(f"\n(interrupted after {loops_completed} completed loop(s))")

    return accepts, rejects, dropped, loops_completed


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _parse_channel(value: str) -> int | None:
    """--channel accepts 'all', 'auto', or a 1-indexed integer."""
    v = value.strip().lower()
    if v in ("all", "any", "*"):
        return None
    if v == "auto":
        return -1  # sentinel; resolved after we have the file path
    try:
        n = int(v)
    except ValueError:
        raise argparse.ArgumentTypeError(f"channel must be 'all', 'auto', or 1-16 (got {value!r})")
    if not 1 <= n <= 16:
        raise argparse.ArgumentTypeError(f"channel must be 1-16 (got {n})")
    return n - 1  # to 0-indexed


def _parse_loops(value: str) -> int:
    """--loops accepts a non-negative int, or 'inf'/'forever' for infinite (0)."""
    v = value.strip().lower()
    if v in ("inf", "infinite", "forever"):
        return 0
    try:
        n = int(v)
    except ValueError:
        raise argparse.ArgumentTypeError(
            f"--loops must be a non-negative int or 'inf' (got {value!r})")
    if n < 0:
        raise argparse.ArgumentTypeError(f"--loops must be >= 0 (got {n})")
    return n


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("midi_path", help="Path to a standard MIDI file (.mid / .midi)")
    ap.add_argument("-p", "--port", help="Serial port (default: auto-detect)")
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD, help=f"Baud rate (default: {DEFAULT_BAUD})")
    ap.add_argument("--current", type=int, default=1500, dest="current_ma",
                    help="Strike current magnitude in mA (default: 1500)")
    ap.add_argument("--channel", type=_parse_channel, default=_parse_channel("10"),
                    help="MIDI channel to play: 1-16, 'all', or 'auto' (default: 10 — GM drums)")
    ap.add_argument("--tempo-scale", type=float, default=1.0,
                    help="Playback speed multiplier (1.0 = normal, 0.5 = half speed)")
    ap.add_argument("--start", type=float, default=0.0, dest="start_s",
                    help="Skip forward to this second mark in the song (default: 0)")
    ap.add_argument("--max-seconds", type=float, default=None,
                    help="Stop after this many seconds of playback (default: play the whole file)")
    ap.add_argument("--loops", type=_parse_loops, default=1,
                    help="Number of times to play the file back-to-back "
                         "(default: 1; use 'inf' or 0 to loop until Ctrl+C)")
    ap.add_argument("--addresses", type=int, nargs="+",
                    help="Specific device addresses to use (default: all enumerated)")
    ap.add_argument("--home-timeout-ms", type=int, default=8000,
                    help="Timeout per homing cycle in ms (default: 8000)")
    ap.add_argument("--poll-ms", type=int, default=20,
                    help="Status polling interval in ms (default: 20)")
    ap.add_argument("--skip-home", action="store_true",
                    help="Assume devices are already homed (will still verify)")
    ap.add_argument("--dry-run", action="store_true",
                    help="Print the voice map and the event timeline without striking anything")
    args = ap.parse_args(argv)

    # Resolve --channel auto now that we have the file path.
    channel = args.channel
    if channel == -1:
        detected = detect_drum_channel(args.midi_path)
        if detected is None:
            print("auto-detect: no GM drum notes (35-59) found in file", file=sys.stderr)
            return 4
        channel = detected
        print(f"auto-detected drum channel: {channel + 1} (0-indexed {channel})")

    # If --dry-run, we don't need the serial port at all.
    if args.dry_run:
        print(f"[dry run] loading {args.midi_path}")
        hist, length = scan_note_histogram(args.midi_path, channel)
        if not hist:
            scope = "any channel" if channel is None else f"channel {channel + 1}"
            print(f"No drum notes found on {scope}", file=sys.stderr)
            return 5
        # Pick a mallet count from --addresses if given, else use 6 as a demo.
        if args.addresses:
            mallet_count = len(args.addresses)
            addresses = args.addresses
        else:
            mallet_count = 6
            addresses = list(range(mallet_count))
            print(f"[dry run] no --addresses given; simulating {mallet_count} mallets")
        role_to_slot = build_role_to_slot(mallet_count)
        print(f"Song length: {length:.1f}s, {sum(hist.values())} drum note-ons")
        print_role_map(addresses, role_to_slot)
        print_note_assignments(addresses, role_to_slot, hist)
        print()
        play_midi(
            client=None,  # type: ignore[arg-type]
            addresses=addresses,
            midi_path=args.midi_path,
            channel=channel,
            current_ma=args.current_ma,
            role_to_slot=role_to_slot,
            tempo_scale=args.tempo_scale,
            start_s=args.start_s,
            max_seconds=args.max_seconds,
            dry_run=True,
            loops=args.loops,
            song_length_s=length,
        )
        return 0

    port = args.port or auto_detect_port()
    if not port:
        print("No serial port found; pass -p/--port", file=sys.stderr)
        return 1

    print(f"Opening {port} at {args.baud} baud")
    client = RingClientV2(port=port, baudrate=args.baud)
    client.open()

    try:
        count = client.enumerate()
        print(f"Enumerated {count} device(s) on the ring")
        if count < 1:
            print("No devices found on the ring", file=sys.stderr)
            return 2

        available = list(range(count))
        addresses = args.addresses if args.addresses else available
        for a in addresses:
            if a not in available:
                print(f"Requested address {a} not present on ring (have {available})", file=sys.stderr)
                return 2

        role_to_slot = build_role_to_slot(len(addresses))
        print(f"Loading {args.midi_path}")
        hist, length = scan_note_histogram(args.midi_path, channel)
        if not hist:
            scope = "any channel" if channel is None else f"channel {channel + 1}"
            print(f"No drum notes found on {scope}", file=sys.stderr)
            return 5
        print(f"Song length: {length:.1f}s, {sum(hist.values())} drum note-ons")
        print_role_map(addresses, role_to_slot)
        print_note_assignments(addresses, role_to_slot, hist)
        print()

        if args.skip_home:
            for addr in addresses:
                s = client.query_strike(addr)
                if not s.homed:
                    print(f"addr {addr} is not homed; rerun without --skip-home", file=sys.stderr)
                    return 3
        else:
            home_all(client, addresses, timeout_ms=args.home_timeout_ms, poll_ms=args.poll_ms)

        scope = "any channel" if channel is None else f"channel {channel + 1}"
        loop_desc = "inf" if args.loops <= 0 else str(args.loops)
        print(f"\nPlaying '{args.midi_path}' on {scope} "
              f"at {args.tempo_scale:g}x tempo, {args.current_ma}mA "
              f"across addresses {addresses} (loops: {loop_desc})")
        accepts, rejects, dropped, loops_done = play_midi(
            client=client,
            addresses=addresses,
            midi_path=args.midi_path,
            channel=channel,
            current_ma=args.current_ma,
            role_to_slot=role_to_slot,
            tempo_scale=args.tempo_scale,
            start_s=args.start_s,
            max_seconds=args.max_seconds,
            dry_run=False,
            loops=args.loops,
            song_length_s=length,
        )
        print(f"\nLoops={loops_done}  accepted={accepts}  rejected={rejects}  dropped={dropped}")

        print("\nWaiting for all actuators to return to idle...")
        wait_all_idle(client, addresses, timeout_ms=3000, poll_ms=args.poll_ms)
        print("Done.")
        return 0

    finally:
        client.close()


if __name__ == "__main__":
    raise SystemExit(main())
