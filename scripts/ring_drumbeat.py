#!/usr/bin/env python3
"""
Exercise multiple strike actuators on the ring by homing them
and then tapping out a drum pattern.

Examples:
    python ring_drumbeat.py
    python ring_drumbeat.py --pattern paradiddle --beats 32 --interval-ms 120
    python ring_drumbeat.py -p COM7 --current 1800 --pattern alternate
"""

from __future__ import annotations

import argparse
import sys
import time
from typing import Iterable

from ring_bus import (
    CommandAck,
    RingClientV2,
    StrikeStatus,
    auto_detect_port,
    DEFAULT_BAUD,
    REPLY_MODE_ACK,
)


PATTERNS: dict[str, list[int]] = {
    # Values are indices into the available actuator list; out-of-range
    # indices are silently skipped so a 3-voice pattern on 2 devices just
    # drops the missing voice.

    # 2-voice
    "alternate":   [0, 1, 0, 1, 0, 1, 0, 1],
    "double":      [0, 0, 1, 1, 0, 0, 1, 1],
    "paradiddle":  [0, 1, 0, 0, 1, 0, 1, 1],
    "roll":        [0, 1] * 8,                                       # packed tightly; expect NOT_READY rejects

    # 3-voice
    "triplet":     [0, 1, 2, 0, 1, 2],                               # even round-robin across all three
    "cascade":     [0, 1, 2, 2, 1, 0],                               # up-then-down fill across the kit
    "tripplediddle": [0, 1, 2, 0, 0, 1, 2, 0, 1, 1, 2, 0, 1, 2, 2],  # rotates the "diddle" through each voice
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


def play_pattern(
    client: RingClientV2,
    addresses: list[int],
    pattern: list[int],
    beats: int,
    interval_ms: int,
    current_ma: int,
) -> None:
    interval_s = interval_ms / 1000.0
    rejects = 0
    accepts = 0
    start = time.monotonic()
    next_time = start

    for beat in range(beats):
        slot = pattern[beat % len(pattern)]
        if slot >= len(addresses):
            # Pattern references a voice we don't have; skip it
            next_time += interval_s
            continue
        addr = addresses[slot]

        # Busy-wait the last few ms for tight timing
        now = time.monotonic()
        sleep_for = next_time - now
        if sleep_for > 0:
            time.sleep(sleep_for)

        reply = client.strike(addr, current_ma, reply_mode=REPLY_MODE_ACK)
        tag = "?"
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

        elapsed_ms = (time.monotonic() - start) * 1000.0
        print(f"  beat {beat:3d}  t={elapsed_ms:7.1f}ms  addr={addr}  {tag}")
        next_time += interval_s

    total_ms = (time.monotonic() - start) * 1000.0
    print(f"\nPlayed {beats} beats in {total_ms:.1f}ms "
          f"(target {beats * interval_ms}ms)  accepted={accepts}  rejected={rejects}")


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
    args = ap.parse_args(argv)

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
        play_pattern(
            client,
            addresses=addresses,
            pattern=PATTERNS[args.pattern],
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
