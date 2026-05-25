#!/usr/bin/env python3
"""
Example chime client for the ring MIDI server.

Demonstrates the friendly API documented in midi_server_api.md. Acts as
both a copy-pasteable integration example and a CLI you can use from the
shell to fire off named chimes.

Two ways to use this file:

    # As a library --
    from chime_demo import play_chime
    play_chime("success")
    play_chime("doorbell", wait=True)

    # As a CLI --
    python scripts/chime_demo.py list
    python scripts/chime_demo.py play success
    python scripts/chime_demo.py play doorbell --wait
    python scripts/chime_demo.py all
    python scripts/chime_demo.py cancel

The chime library adapts to whichever pitches happen to be mapped on the
bridge --- each builder picks pitches by their position in the sorted
available-pitches list rather than by absolute MIDI number. So you'll
get something reasonable whether the ring is mapped to a C major scale,
a pentatonic, or just three random tines.

If you're building your own integration, the interesting bits are:

    1. ChimeClient -- thin urllib wrapper around the friendly endpoints.
    2. The CHIMES dict at the bottom -- each entry is a function that
       takes the list returned by /api/pitches and returns a motif body
       ready to POST.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
import urllib.error
import urllib.request
from typing import Any, Callable

DEFAULT_BASE_URL = "http://127.0.0.1:8765"
# Why not "http://localhost:8765"? On Windows, Python's urllib resolves
# "localhost" to ::1 first, the server only binds IPv4, and each request
# eats a ~2 s IPv6 connect timeout before falling back. 127.0.0.1
# sidesteps that entirely. Override via --server if you bound elsewhere.


# ---------------------------------------------------------------------------
# HTTP client
# ---------------------------------------------------------------------------


class ChimeClientError(RuntimeError):
    """Raised for any non-2xx response from the bridge or transport failure.

    `status` is the HTTP status code if we got a response, or None if the
    request never reached the server (connection refused, DNS failure, etc.).
    `payload` is the decoded JSON error body if there was one.
    """

    def __init__(self, message: str, *, status: int | None = None, payload: Any = None):
        super().__init__(message)
        self.status = status
        self.payload = payload


class ChimeClient:
    """Minimal client for the ring MIDI server's friendly API.

    Stdlib only -- no requests dependency so the demo runs anywhere a
    plain Python interpreter does.
    """

    def __init__(self, base_url: str = DEFAULT_BASE_URL, timeout: float = 5.0):
        self.base_url = base_url.rstrip("/")
        self.timeout = timeout

    # ---- low-level ---------------------------------------------------------

    def _request(self, method: str, path: str, body: dict | None = None) -> Any:
        url = f"{self.base_url}{path}"
        data = None
        headers = {"Accept": "application/json"}
        if body is not None:
            data = json.dumps(body).encode("utf-8")
            headers["Content-Type"] = "application/json"
        req = urllib.request.Request(url, data=data, method=method, headers=headers)
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as resp:
                raw = resp.read()
                if not raw:
                    return None
                return json.loads(raw)
        except urllib.error.HTTPError as exc:
            try:
                payload = json.loads(exc.read())
            except Exception:
                payload = None
            msg = (payload or {}).get("error") if isinstance(payload, dict) else None
            raise ChimeClientError(
                f"{method} {path} -> HTTP {exc.code}: {msg or exc.reason}",
                status=exc.code, payload=payload,
            ) from exc
        except urllib.error.URLError as exc:
            raise ChimeClientError(
                f"{method} {path} -> transport error: {exc.reason}",
            ) from exc

    # ---- friendly API surface ---------------------------------------------

    def status(self) -> dict:
        return self._request("GET", "/api/status")

    def pitches(self) -> list[dict]:
        return self._request("GET", "/api/pitches")["pitches"]

    def mapping(self) -> list | None:
        return self._request("GET", "/api/mapping")["mapping"]

    def play_motif(
        self,
        events: list[dict],
        *,
        name: str | None = None,
        master_current_ma: int | None = None,
        vel_floor: float | None = None,
    ) -> dict:
        body: dict = {"events": events}
        if name is not None:
            body["name"] = name
        if master_current_ma is not None:
            body["master_current_ma"] = master_current_ma
        if vel_floor is not None:
            body["vel_floor"] = vel_floor
        return self._request("POST", "/api/motif", body)

    def motif_status(self) -> dict:
        return self._request("GET", "/api/motif")

    def cancel(self) -> dict:
        return self._request("POST", "/api/cancel")

    # ---- convenience -------------------------------------------------------

    def wait_for_motif(self, *, poll_interval: float = 0.1, timeout: float = 10.0) -> dict:
        """Poll /api/motif until `playing: false` or the timeout elapses.

        Returns the final status dict. Note: the worker thread keeps
        `playing: true` for a brief moment after `remaining_ms` reaches 0
        while it tears down; this method waits for the authoritative
        `playing: false`, not just `remaining_ms == 0`."""
        deadline = time.monotonic() + timeout
        while True:
            status = self.motif_status()
            if not status.get("playing"):
                return status
            if time.monotonic() >= deadline:
                return status
            time.sleep(poll_interval)


# ---------------------------------------------------------------------------
# Chime library
# ---------------------------------------------------------------------------


ChimeBuilder = Callable[[list[dict]], dict]


def _pick(pitches: list[dict], fractions: list[float]) -> list[dict]:
    """Pick pitches by position in the sorted available list.

    0.0 selects the lowest mapped pitch, 1.0 the highest, with linear
    interpolation in between. Duplicates are allowed and produce a
    same-pitch repeat (caller is responsible for spacing those at least
    ~150 ms apart per the API docs).
    """
    if not pitches:
        return []
    n = len(pitches)
    return [pitches[max(0, min(n - 1, int(round(f * (n - 1)))))] for f in fractions]


def chime_success(pitches: list[dict]) -> dict:
    """Bright ascending arpeggio. Three pitches, low-mid-high, fast."""
    selected = _pick(pitches, [0.2, 0.5, 0.85])
    events = [
        {"t_ms": i * 110, "pitch": p["pitch"], "velocity": 95}
        for i, p in enumerate(selected)
    ]
    return {"name": "success", "events": events, "master_current_ma": 900}


def chime_error(pitches: list[dict]) -> dict:
    """Two-note high-then-low. Punchy."""
    high, low = _pick(pitches, [0.85, 0.15])
    events = [
        {"t_ms": 0,   "pitch": high["pitch"], "velocity": 115},
        {"t_ms": 250, "pitch": low["pitch"],  "velocity": 115},
    ]
    return {"name": "error", "events": events, "master_current_ma": 1100}


def chime_attention(pitches: list[dict]) -> dict:
    """High strike with a quieter echo on a nearby pitch."""
    primary, echo = _pick(pitches, [0.95, 0.7])
    events = [
        {"t_ms": 0,   "pitch": primary["pitch"], "velocity": 120},
        {"t_ms": 350, "pitch": echo["pitch"],    "velocity": 60},
    ]
    return {"name": "attention", "events": events, "master_current_ma": 1100}


def chime_doorbell(pitches: list[dict]) -> dict:
    """Two-pitch ding-dong, the classic Westminster opening."""
    ding, dong = _pick(pitches, [0.7, 0.4])
    events = [
        {"t_ms": 0,   "pitch": ding["pitch"], "velocity": 100},
        {"t_ms": 350, "pitch": dong["pitch"], "velocity": 100},
    ]
    return {"name": "doorbell", "events": events, "master_current_ma": 950}


def chime_ack(pitches: list[dict]) -> dict:
    """Single soft mid-range strike. 'I heard you.'"""
    (only,) = _pick(pitches, [0.5])
    events = [{"t_ms": 0, "pitch": only["pitch"], "velocity": 60}]
    return {"name": "ack", "events": events, "master_current_ma": 600}


def chime_alarm(pitches: list[dict]) -> dict:
    """Repeated mid pitch -- attention-grabbing, harder to ignore."""
    (note,) = _pick(pitches, [0.5])
    events = [
        {"t_ms": i * 300, "pitch": note["pitch"], "velocity": 120}
        for i in range(4)
    ]
    return {"name": "alarm", "events": events, "master_current_ma": 1300}


CHIMES: dict[str, ChimeBuilder] = {
    "success":   chime_success,
    "error":     chime_error,
    "attention": chime_attention,
    "doorbell":  chime_doorbell,
    "ack":       chime_ack,
    "alarm":     chime_alarm,
}


# ---------------------------------------------------------------------------
# One-liner convenience
# ---------------------------------------------------------------------------


def play_chime(
    name: str,
    *,
    base_url: str = DEFAULT_BASE_URL,
    wait: bool = False,
) -> dict:
    """Discover available pitches, build the named chime, post it.

    Returns the bridge's response from /api/motif. If `wait=True`, polls
    /api/motif (GET) until playback finishes and returns the final
    status dict instead.
    """
    builder = CHIMES.get(name)
    if builder is None:
        raise ValueError(
            f"unknown chime {name!r}; known: {sorted(CHIMES)}"
        )
    client = ChimeClient(base_url=base_url)
    pitches = client.pitches()
    if not pitches:
        raise ChimeClientError(
            "no pitches are mapped on the bridge; configure a mapping first"
        )
    motif = builder(pitches)
    result = client.play_motif(**motif)
    if not wait:
        return result
    # Generous timeout: schedule duration + 2 s tear-down slack.
    duration_s = result.get("duration_ms", 0) / 1000.0
    return client.wait_for_motif(timeout=duration_s + 2.0)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def _print_pitches(pitches: list[dict]) -> None:
    if not pitches:
        print("  (no pitches mapped)")
        return
    for p in pitches:
        homed = "" if p["homed"] else "  [not homed]"
        print(f"  slot {p['slot']:>2}  pitch {p['pitch']:>3}  {p['name']}{homed}")


def cmd_list(args: argparse.Namespace) -> int:
    client = ChimeClient(base_url=args.server)
    pitches = client.pitches()
    print(f"Mapped pitches ({len(pitches)}):")
    _print_pitches(pitches)
    print()
    print("Available chimes:")
    for chime_name, builder in CHIMES.items():
        doc = (builder.__doc__ or "").strip().splitlines()[0]
        print(f"  {chime_name:<10}  {doc}")
    return 0


def cmd_play(args: argparse.Namespace) -> int:
    result = play_chime(args.name, base_url=args.server, wait=args.wait)
    print(json.dumps(result, indent=2))
    return 0


def cmd_all(args: argparse.Namespace) -> int:
    """Play every chime in order. Always waits between chimes so they
    don't preempt each other; otherwise the first POST would queue an
    11-second sequence and only the last one would actually play."""
    client = ChimeClient(base_url=args.server)
    pitches = client.pitches()
    if not pitches:
        print("No pitches are mapped on the bridge; configure a mapping first.",
              file=sys.stderr)
        return 1
    for chime_name, builder in CHIMES.items():
        print(f"-> {chime_name}")
        motif = builder(pitches)
        result = client.play_motif(**motif)
        # Wait for the bridge to confirm it's done, then pause briefly
        # so the listener can tell the chimes apart.
        duration_s = result.get("duration_ms", 0) / 1000.0
        client.wait_for_motif(timeout=duration_s + 2.0)
        time.sleep(args.gap)
    return 0


def cmd_cancel(args: argparse.Namespace) -> int:
    client = ChimeClient(base_url=args.server)
    result = client.cancel()
    print(json.dumps(result, indent=2))
    return 0


def cmd_status(args: argparse.Namespace) -> int:
    client = ChimeClient(base_url=args.server)
    result = client.motif_status()
    print(json.dumps(result, indent=2))
    return 0


def main(argv: list[str] | None = None) -> int:
    # Shared parent so --server works whether the user puts it before or
    # after the subcommand. argparse's top-level flags otherwise have to
    # appear before the subcommand, which trips up everyone.
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument(
        "--server", default=DEFAULT_BASE_URL,
        help=f"Base URL of the ring MIDI server (default: {DEFAULT_BASE_URL})",
    )

    parser = argparse.ArgumentParser(
        description="Example chime client for the ring MIDI server.",
        parents=[common],
    )
    sub = parser.add_subparsers(dest="cmd", required=True)

    p_list = sub.add_parser("list", parents=[common],
                            help="show mapped pitches and available chimes")
    p_list.set_defaults(func=cmd_list)

    p_play = sub.add_parser("play", parents=[common], help="play a named chime")
    p_play.add_argument("name", choices=sorted(CHIMES.keys()))
    p_play.add_argument("--wait", action="store_true",
                        help="block until the chime finishes")
    p_play.set_defaults(func=cmd_play)

    p_all = sub.add_parser("all", parents=[common],
                           help="play every chime in sequence")
    p_all.add_argument("--gap", type=float, default=0.6,
                       help="seconds of silence between chimes (default: 0.6)")
    p_all.set_defaults(func=cmd_all)

    p_cancel = sub.add_parser("cancel", parents=[common],
                              help="stop any running motif")
    p_cancel.set_defaults(func=cmd_cancel)

    p_status = sub.add_parser("status", parents=[common],
                              help="show motif playback status")
    p_status.set_defaults(func=cmd_status)

    args = parser.parse_args(argv)
    try:
        return args.func(args)
    except ChimeClientError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    except ValueError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
