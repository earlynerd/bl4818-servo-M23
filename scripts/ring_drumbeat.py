#!/usr/bin/env python3
"""
Exercise multiple strike actuators on the ring by homing them
and then tapping out a drum pattern.

Patterns come in two flavors:
  * drumline exercises (2-4 voice) that just treat the actuators as
    anonymous sticks (L/R/F) and test rudiments / rolls.
  * traditional drumset beats (4-6 voice) where each voice index has
    an expected physical role (kick, snare, hi-hat, tom, crash).
    The script prints the voice legend on start so you know which
    object to place under each actuator.

Each step in a pattern may be:
    int         -> fire that voice
    [i, j, ...] -> fire those voices simultaneously (chord)
    None        -> rest (silence for that step)

Examples:
    python ring_drumbeat.py
    python ring_drumbeat.py --pattern paradiddle --beats 32 --interval-ms 120
    python ring_drumbeat.py -p COM7 --current 1800 --pattern rock-basic
    python ring_drumbeat.py --pattern full-kit --beats 32 --interval-ms 180
    python ring_drumbeat.py --list-patterns
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass, field
from typing import Iterable

from ring_bus import (
    CommandAck,
    RingClientV2,
    StrikeStatus,
    auto_detect_port,
    DEFAULT_BAUD,
    REPLY_MODE_ACK,
)


Step = "int | list[int] | None"


@dataclass
class Pattern:
    """A drum pattern plus the expected physical role of each voice."""
    steps: list  # list[int | list[int] | None]
    voices: list  # list[str] — description per voice index
    description: str = ""

    @property
    def voice_count(self) -> int:
        return len(self.voices)


def _p(steps, voices, description=""):
    return Pattern(steps=steps, voices=voices, description=description)


# Voice-role shorthands used in the traditional kit beats.
_KIT4 = ["kick", "snare", "hat", "crash"]
_KIT5 = ["kick", "snare", "hat", "tom", "crash"]
_KIT6 = ["kick", "snare-A", "snare-B", "hat", "tom", "crash"]
_KIT10 = ["kick", "snare-A", "snare-B", "hat", "tom", "crash"]


PATTERNS: dict[str, Pattern] = {
    # ------------------------------------------------------------------
    # Drumline exercises — anonymous voices (L / R / F / vN)
    # ------------------------------------------------------------------

    # 2-voice
    "alternate":   _p([0, 1] * 4,
                       voices=["L", "R"],
                       description="Straight single-stroke alternation"),
    "double":      _p([0, 0, 1, 1] * 2,
                       voices=["L", "R"],
                       description="Double-stroke (LLRR) roll"),
    "paradiddle":  _p([0, 1, 0, 0, 1, 0, 1, 1],
                       voices=["L", "R"],
                       description="Single paradiddle (LRLL RLRR)"),
    "roll":        _p([0, 1] * 8,
                       voices=["L", "R"],
                       description="Tight roll; expect NOT_READY rejects at low intervals"),

    # 3-voice
    "triplet":      _p([0, 1, 2] * 2,
                        voices=["L", "R", "F"],
                        description="Even round-robin triplet across all three voices"),
    "cascade":      _p([0, 1, 2, 2, 1, 0],
                        voices=["L", "R", "F"],
                        description="Up-then-down fill across the kit"),
    "tripplediddle": _p([0, 1, 2, 0, 0, 1, 2, 0, 1, 1, 2, 0, 1, 2, 2],
                        voices=["L", "R", "F"],
                        description="Rotating diddle across three voices"),

    # 4-voice exercises (not kit-mapped)
    "quartet":     _p([0, 1, 2, 3] * 2,
                       voices=["v0", "v1", "v2", "v3"],
                       description="Even round-robin across four voices"),
    "cascade-4":   _p([0, 1, 2, 3, 3, 2, 1, 0],
                       voices=["v0", "v1", "v2", "v3"],
                       description="Up-then-down across four voices"),
    "tetradiddle": _p([0, 1, 0, 0, 1, 0, 1, 1, 2, 3, 2, 2, 3, 2, 3, 3],
                       voices=["v0", "v1", "v2", "v3"],
                       description="Paradiddle rotated across two pairs"),

    # ------------------------------------------------------------------
    # 4-voice traditional kit: kick / snare / hat / crash
    # ------------------------------------------------------------------

    "rock-basic": _p(
        [[0, 2], [2], [1, 2], [2], [0, 2], [2], [1, 2], [2]],
        voices=_KIT4,
        description="Classic 8th-note rock: kick 1&3, snare 2&4, hat every 8th",
    ),
    "backbeat": _p(
        [[0, 2], [1, 2], [0, 2], [1, 2]],
        voices=_KIT4,
        description="Quarter-note backbeat; slower and easy to hear",
    ),
    "four-on-floor": _p(
        [[0, 2], [2], [0, 1, 2], [2], [0, 2], [2], [0, 1, 2], [2]],
        voices=_KIT4,
        description="Disco/house: kick on every quarter, snare on 2 & 4, hats on 8ths",
    ),
    "waltz": _p(
        [[0, 2], [2], [1, 2], [2], [1, 2], [2]],
        voices=_KIT4,
        description="3/4 waltz: kick on 1, snare on 2 & 3, hat on every 8th",
    ),
    "halftime": _p(
        [[0, 2], [2], [2], [2], [1, 2], [2], [2], [2]],
        voices=_KIT4,
        description="Half-time feel: kick on 1, snare only on 3",
    ),
    "shuffle": _p(
        # 12 steps = 1 bar at triplet resolution; play 1st & 3rd of each triplet
        [[0, 2], None, [2], [2], None, [2],
         [1, 2], None, [2], [2], None, [2]],
        voices=_KIT4,
        description="Triplet-feel shuffle (blues/swing). Uses rests between triplets",
    ),

    # ------------------------------------------------------------------
    # 5-voice traditional kit: + tom
    # ------------------------------------------------------------------

    "rock-fill": _p(
        # 3 bars of rock + 1 bar tom/crash fill = 32 eighth notes
        [[0, 2], [2], [1, 2], [2], [0, 2], [2], [1, 2], [2]] * 3 +
        [1, 1, 3, 3, [3, 4], None, [0, 4], None],
        voices=_KIT5,
        description="Rock beat for 3 bars then tom-and-crash fill",
    ),
    "tom-groove": _p(
        [[0, 2], [2], [1, 2], 3, [0, 2], 3, [1, 2], [2]],
        voices=_KIT5,
        description="Rock feel with tom replacing offbeat hats as an accent",
    ),
    "funk": _p(
        [[0, 2], [2], [2], [2], [1, 2], [2], [2], [2, 3],
         [0, 2], [0, 2], [2], [2], [1, 2], [2], [2], [2, 3]],
        voices=_KIT5,
        description="Syncopated funk at 16ths; kick doubles and a tom ghost on 'a' of 4",
    ),
    "bossa": _p(
        [[0, 2], [2], [2, 1], [2], [0, 2], [2], [2, 3], [2],
         [0, 2], [2], [2, 1], [2], [0, 2], [2], [2, 3], [2]],
        voices=_KIT5,
        description="Latin bossa feel: steady kick, snare & tom on the 'and' of 2 / 3",
    ),

    # ------------------------------------------------------------------
    # 6-voice traditional kit: + a second snare voice (fat snare / ghost)
    # ------------------------------------------------------------------

    "ghost-rock": _p(
        [[0, 3], 2, [1, 3], 2, [0, 3], 2, [1, 3], [2, 3]],
        voices=_KIT6,
        description="Rock beat with ghost notes on the secondary snare",
    ),
    "double-snare": _p(
        [[0, 3], [3], [1, 2, 3], [3], [0, 3], [3], [1, 2, 3], [3]],
        voices=_KIT6,
        description="Rock beat where both snares strike together for a fat backbeat",
    ),
    "full-kit": _p(
        # 2 bars, 8ths: ghost-rock feel for bar 1, tom & crash fill end of bar 2
        [[0, 3], 2, [1, 3], [3, 2], [0, 3], 2, [1, 3], [3],
         [0, 3], 2, [1, 3], [3, 2], [0, 3], [4],  [1, 2, 4], [3, 5]],
        voices=_KIT6,
        description="Full-kit 2-bar groove: ghost snare embellishments, ends on crash+snare",
    ),
    # ------------------------------------------------------------------
    # 10-voice traditional kit: + a second snare voice (fat snare / ghost)
    # ------------------------------------------------------------------
    "arpeggio": _p(
         [2, 0, 4, 8, 6, 9, 5, 1, 3, 7, 3, 1, 5, 9, 6, 8, 4, 0, 2],
        voices=_KIT10,
        description="arp? arp arp arp!",
    ),
}


def home_all(client: RingClientV2, addresses: list[int], timeout_ms: int, poll_ms: int) -> None:
    """Kick off homing on every address then wait for each to report homed+idle."""
    print(f"Homing {len(addresses)} device(s): {addresses}")
    for addr in addresses:
        reply = client.strike_home(addr, reply_mode=REPLY_MODE_ACK)
        if not isinstance(reply, CommandAck) or not reply.accepted:
            result = "no-ack" if reply is None else reply.result_name
            raise RuntimeError(f"strike_home addr {addr} rejected: {result}")

    deadline = time.monotonic() + timeout_ms / 1000.0
    poll_s = max(poll_ms, 0) / 1000.0
    pending = set(addresses)
    while pending:
        for addr in list(pending):
            status = client.query_strike(addr)
            if status.homed and status.state == 0 and not status.active:
                print(f"  addr {addr} homed (home_position={status.home_position})")
                pending.discard(addr)
        if not pending:
            return
        if time.monotonic() >= deadline:
            raise TimeoutError(f"timed out waiting for homing on {sorted(pending)}")
        if poll_s > 0:
            time.sleep(poll_s)


def wait_all_idle(client: RingClientV2, addresses: Iterable[int], timeout_ms: int, poll_ms: int) -> None:
    deadline = time.monotonic() + timeout_ms / 1000.0
    poll_s = max(poll_ms, 0) / 1000.0
    pending = set(addresses)
    while pending:
        for addr in list(pending):
            status = client.query_strike(addr)
            if status.state == 0 and not status.active:
                pending.discard(addr)
        if not pending:
            return
        if time.monotonic() >= deadline:
            print(f"  warning: still not idle: {sorted(pending)}")
            return
        if poll_s > 0:
            time.sleep(poll_s)


def _normalize_step(step) -> list[int]:
    """Turn a step value into a list of voice indices to strike (possibly empty)."""
    if step is None:
        return []
    if isinstance(step, int):
        return [step]
    return list(step)


def play_pattern(
    client: RingClientV2,
    addresses: list[int],
    pattern: Pattern,
    beats: int,
    interval_ms: int,
    current_ma: int,
) -> None:
    interval_s = interval_ms / 1000.0
    steps = pattern.steps
    rejects = 0
    accepts = 0
    start = time.monotonic()
    next_time = start

    for beat in range(beats):
        raw = steps[beat % len(steps)]
        voices = _normalize_step(raw)

        # Wait until the next beat time before firing any voices, so the
        # chord lands together rather than spreading across the previous
        # inter-beat gap.
        now = time.monotonic()
        sleep_for = next_time - now
        if sleep_for > 0:
            time.sleep(sleep_for)

        elapsed_ms = (time.monotonic() - start) * 1000.0

        if not voices:
            print(f"  beat {beat:3d}  t={elapsed_ms:7.1f}ms  (rest)")
            next_time += interval_s
            continue

        tags: list[str] = []
        for v in voices:
            if v >= len(addresses):
                tags.append(f"v{v}:SKIP")
                continue
            addr = addresses[v]
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
            tags.append(f"{addr}:{tag}")

        print(f"  beat {beat:3d}  t={elapsed_ms:7.1f}ms  [{' '.join(tags)}]")
        next_time += interval_s

    total_ms = (time.monotonic() - start) * 1000.0
    print(f"\nPlayed {beats} beats in {total_ms:.1f}ms "
          f"(target {beats * interval_ms}ms)  accepted={accepts}  rejected={rejects}")


def print_voice_layout(pattern: Pattern, addresses: list[int]) -> None:
    """Tell the user which physical object to place under each actuator."""
    print("  Role map:")
    for idx, role in enumerate(pattern.voices):
        if idx < len(addresses):
            print(f"    addr {addresses[idx]}  ->  {role}")
        else:
            print(f"    (missing)    ->  {role}  [no actuator available; will be silent]")
    if len(addresses) > len(pattern.voices):
        extra = addresses[len(pattern.voices):]
        print(f"    extra actuators not used by this pattern: {extra}")


def list_patterns() -> None:
    """Print every pattern with its voice count and description."""
    # Group by voice count for readability.
    by_count: dict[int, list[tuple[str, Pattern]]] = {}
    for name, pat in PATTERNS.items():
        by_count.setdefault(pat.voice_count, []).append((name, pat))
    for count in sorted(by_count):
        print(f"\n  {count}-voice patterns:")
        for name, pat in sorted(by_count[count]):
            voices = "/".join(pat.voices)
            print(f"    {name:<15}  [{voices}]")
            if pat.description:
                print(f"                     {pat.description}")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-p", "--port", help="Serial port (default: auto-detect)")
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD, help=f"Baud rate (default: {DEFAULT_BAUD})")
    ap.add_argument("--current", type=int, default=1500, dest="current_ma",
                    help="Strike current magnitude in mA (default: 1500)")
    ap.add_argument("--pattern", choices=sorted(PATTERNS.keys()), default="alternate",
                    help="Drum pattern to play (default: alternate)")
    ap.add_argument("--beats", type=int, default=16, help="Total beats to play (default: 16)")
    ap.add_argument("--interval-ms", type=int, default=150,
                    help="Target ms between beats (default: 150). Use smaller for faster rolls.")
    ap.add_argument("--addresses", type=int, nargs="+",
                    help="Specific device addresses to use (default: all enumerated)")
    ap.add_argument("--home-timeout-ms", type=int, default=8000,
                    help="Timeout per homing cycle in ms (default: 8000)")
    ap.add_argument("--poll-ms", type=int, default=20,
                    help="Status polling interval in ms (default: 20)")
    ap.add_argument("--skip-home", action="store_true",
                    help="Assume devices are already homed (will still verify)")
    ap.add_argument("--list-patterns", action="store_true",
                    help="Print every pattern with its voice roles and exit")
    args = ap.parse_args(argv)

    if args.list_patterns:
        list_patterns()
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

        pattern = PATTERNS[args.pattern]

        if len(addresses) < pattern.voice_count:
            print(f"Note: pattern '{args.pattern}' wants {pattern.voice_count} voice(s) "
                  f"but only {len(addresses)} actuator(s) selected — higher voices will be silent.")
        if len(addresses) < 2:
            print(f"Note: only {len(addresses)} actuator(s) selected — pattern will collapse to that voice.")

        if args.skip_home:
            for addr in addresses:
                s = client.query_strike(addr)
                if not s.homed:
                    print(f"addr {addr} is not homed; rerun without --skip-home", file=sys.stderr)
                    return 3
        else:
            home_all(client, addresses, timeout_ms=args.home_timeout_ms, poll_ms=args.poll_ms)

        print(f"\nPlaying '{args.pattern}' — {args.beats} beats at {args.interval_ms}ms "
              f"on addresses {addresses}, {args.current_ma}mA")
        if pattern.description:
            print(f"  {pattern.description}")
        print_voice_layout(pattern, addresses)
        print()

        play_pattern(
            client,
            addresses=addresses,
            pattern=pattern,
            beats=args.beats,
            interval_ms=args.interval_ms,
            current_ma=args.current_ma,
        )

        print("\nWaiting for all actuators to return to idle...")
        wait_all_idle(client, addresses, timeout_ms=3000, poll_ms=args.poll_ms)
        print("Done.")
        return 0

    finally:
        client.close()


if __name__ == "__main__":
    raise SystemExit(main())
