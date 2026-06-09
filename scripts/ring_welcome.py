#!/usr/bin/env python3
"""
Walk-in chime daemon for the ring MIDI server.

Scans the local network for known people's devices (by hostname, MAC, or
IP) and plays each person "their song" on the actuator ring the first time
their device is seen on the LAN each day.

It is a thin layer on top of the friendly HTTP API documented in
midi_server_api.md -- it never touches the serial bus directly. Run the
ring MIDI server first (`python scripts/ring_midi_server.py`), then run
this against it.

    # One-shot discovery dump -- like `nmap -sn`, shows every device it can
    # see with IP / MAC / hostname so you can fill in the roster.
    python scripts/ring_welcome.py scan
    python scripts/ring_welcome.py scan --range 192.168.1.0/24

    # Play a configured person's song right now (verify config + ring).
    python scripts/ring_welcome.py test Sly

    # Show the roster and who has already triggered today.
    python scripts/ring_welcome.py list

    # Run the watcher (the actual daemon).
    python scripts/ring_welcome.py watch

Config lives in welcome_config.json (see welcome_config.example.json).
Per-day "already played" state lives in welcome_state.json. Both default
to the project root and are git-ignored.

Detection is stdlib-only and needs no admin rights:
  1. Ping-sweep the IP range (concurrently) to wake the ARP cache.
  2. Parse `arp -a` for IP -> MAC.
  3. Reverse-DNS each responder; on Windows also `nbtstat -A` for the
     NetBIOS name (this is the "sly"-style name you see in nmap).
A device counts as present if it answered ping, appears in the ARP table,
or resolves a name. Modern phones sleep and may ignore ICMP, so detection
can lag until the phone next talks to the network on its own -- that's the
trade-off for needing zero dependencies.
"""

from __future__ import annotations

import argparse
import concurrent.futures
import ipaddress
import json
import os
import platform
import re
import socket
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

# Reuse the documented HTTP client and our stdlib MIDI parser from the same
# directory. The script's own dir is on sys.path[0] when run directly; make
# it explicit so `import` also works when invoked oddly.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from chime_demo import ChimeClient, ChimeClientError  # noqa: E402
from midi_to_motif import midi_file_to_events  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
DEFAULT_CONFIG = ROOT / "welcome_config.json"
DEFAULT_STATE = ROOT / "welcome_state.json"
DEFAULT_BASE_URL = "http://127.0.0.1:8765"
IS_WINDOWS = platform.system() == "Windows"

_MAC_RE = re.compile(r"([0-9a-fA-F]{2}[:-]){5}[0-9a-fA-F]{2}")
_IP_RE = re.compile(r"\b(\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3})\b")


# ---------------------------------------------------------------------------
# Device discovery (stdlib only)
# ---------------------------------------------------------------------------


class Device:
    """One host seen on the LAN this scan."""

    __slots__ = ("ip", "mac", "names")

    def __init__(self, ip: str):
        self.ip = ip
        self.mac: str | None = None
        self.names: set[str] = set()  # lowercased, both FQDN and short forms

    def add_name(self, name: str | None) -> None:
        if not name:
            return
        name = name.strip().rstrip(".").lower()
        if not name:
            return
        self.names.add(name)
        # Also index the short (pre-dot) form so "sly" matches "sly.local".
        short = name.split(".", 1)[0]
        if short:
            self.names.add(short)

    def __repr__(self) -> str:
        return f"Device(ip={self.ip}, mac={self.mac}, names={sorted(self.names)})"


def _normalize_mac(mac: str | None) -> str | None:
    if not mac:
        return None
    m = _MAC_RE.search(mac)
    if not m:
        return None
    return m.group(0).replace("-", ":").lower()


def _ping(ip: str, timeout_ms: int) -> bool:
    """Send one ICMP echo. True only on a real reply.

    Windows `ping` exits 0 even for "Destination host unreachable", so we
    confirm an actual reply by looking for a TTL in the output.
    """
    if IS_WINDOWS:
        cmd = ["ping", "-n", "1", "-w", str(timeout_ms), ip]
    else:
        # -W is seconds on Linux; round up to at least 1.
        cmd = ["ping", "-c", "1", "-W", str(max(1, timeout_ms // 1000)), ip]
    try:
        proc = subprocess.run(
            cmd, capture_output=True, text=True,
            timeout=timeout_ms / 1000.0 + 2.0,
        )
    except (subprocess.TimeoutExpired, OSError):
        return False
    out = (proc.stdout or "").lower()
    if "ttl=" in out:
        return True
    # Fallback for locales that don't print TTL: trust a clean exit that
    # isn't an explicit unreachable/timeout.
    return proc.returncode == 0 and "unreachable" not in out and "timed out" not in out


def _read_arp_table() -> dict[str, str]:
    """Parse `arp -a` into {ip: normalized_mac}. Empty dict on failure."""
    try:
        proc = subprocess.run(["arp", "-a"], capture_output=True, text=True, timeout=10)
    except (subprocess.TimeoutExpired, OSError):
        return {}
    table: dict[str, str] = {}
    for line in proc.stdout.splitlines():
        mac = _normalize_mac(line)
        if not mac:
            continue
        ip_match = _IP_RE.search(line)
        if not ip_match:
            continue
        table[ip_match.group(1)] = mac
    return table


def _reverse_dns(ip: str) -> str | None:
    try:
        return socket.gethostbyaddr(ip)[0]
    except (OSError, socket.herror):
        return None


def _netbios_name(ip: str) -> str | None:
    """Query a Windows host's NetBIOS name via `nbtstat -A`.

    Returns the <00> UNIQUE name -- the workstation/computer name, which is
    the "sly"-style name nmap surfaces. Windows-only; None elsewhere or on
    any failure.
    """
    if not IS_WINDOWS:
        return None
    try:
        proc = subprocess.run(
            ["nbtstat", "-A", ip], capture_output=True, text=True, timeout=4,
        )
    except (subprocess.TimeoutExpired, OSError):
        return None
    for line in proc.stdout.splitlines():
        # e.g. "    SLY            <00>  UNIQUE      Registered"
        if "<00>" in line and "UNIQUE" in line.upper():
            name = line.split("<00>")[0].strip()
            if name:
                return name
    return None


def scan_network(
    ip_range: str,
    *,
    ping_timeout_ms: int = 600,
    workers: int = 64,
    do_netbios: bool = True,
    log=lambda _m: None,
) -> list[Device]:
    """Discover live devices in `ip_range` (a CIDR like 192.168.1.0/24).

    Returns one Device per host that answered ping, is in the ARP table, or
    resolved a name.
    """
    net = ipaddress.ip_network(ip_range, strict=False)
    hosts = [str(h) for h in net.hosts()] if net.num_addresses > 2 else [str(net.network_address)]

    log(f"pinging {len(hosts)} addresses in {ip_range} ...")
    responders: set[str] = set()
    with concurrent.futures.ThreadPoolExecutor(max_workers=workers) as pool:
        for ip, alive in zip(hosts, pool.map(lambda h: _ping(h, ping_timeout_ms), hosts)):
            if alive:
                responders.add(ip)

    arp = _read_arp_table()  # populated by the ping sweep we just ran
    candidate_ips = sorted(
        responders | {ip for ip in arp if ip in set(hosts)},
        key=lambda s: tuple(int(p) for p in s.split(".")),
    )
    log(f"{len(candidate_ips)} live host(s); resolving names ...")

    devices: list[Device] = []
    for ip in candidate_ips:
        dev = Device(ip)
        dev.mac = arp.get(ip)
        devices.append(dev)

    # Resolve names concurrently -- reverse DNS is quick, nbtstat is slow.
    def resolve(dev: Device) -> None:
        dev.add_name(_reverse_dns(dev.ip))
        if do_netbios and not dev.names:
            dev.add_name(_netbios_name(dev.ip))

    if devices:
        with concurrent.futures.ThreadPoolExecutor(max_workers=min(32, len(devices))) as pool:
            list(pool.map(resolve, devices))
    return devices


# ---------------------------------------------------------------------------
# Roster matching
# ---------------------------------------------------------------------------


def person_matches(person: dict, dev: Device) -> bool:
    """True if `dev` matches any identifier configured for `person`.

    A person's "match" block may list any of: hostname(s), mac(s), ip(s).
    Matching any single configured identifier is enough.
    """
    match = person.get("match", {})

    macs = {_normalize_mac(m) for m in _as_list(match.get("mac"))}
    macs.discard(None)
    if dev.mac and dev.mac in macs:
        return True

    ips = set(_as_list(match.get("ip")))
    if dev.ip in ips:
        return True

    wanted = {h.strip().rstrip(".").lower() for h in _as_list(match.get("hostname")) if h}
    # Match a configured name against either the device's full names or their
    # short forms (both are stored in dev.names), case-insensitively.
    if wanted & dev.names:
        return True

    return False


def _as_list(value) -> list:
    if value is None:
        return []
    if isinstance(value, list):
        return value
    return [value]


# ---------------------------------------------------------------------------
# Song resolution: config -> motif events the API can play
# ---------------------------------------------------------------------------

# Built-in melodies, defined as positions in the sorted list of *available*
# mapped pitches (0.0 = lowest tine, 1.0 = highest). Same idea as the
# fraction picker in chime_demo.py, so they adapt to any ring layout.
BUILTIN_MELODIES: dict[str, list[tuple[int, float, int]]] = {
    # name: list of (t_ms, pitch_fraction, velocity)
    "ascending":  [(0, 0.0, 95), (140, 0.33, 100), (280, 0.66, 105), (420, 1.0, 110)],
    "descending": [(0, 1.0, 105), (140, 0.66, 100), (280, 0.33, 95), (420, 0.0, 95)],
    "fanfare":    [(0, 0.0, 110), (0, 0.5, 100), (200, 0.5, 100), (400, 1.0, 115),
                   (400, 0.0, 90)],
    "pentatonic": [(0, 0.0, 90), (160, 0.25, 95), (320, 0.5, 100),
                   (480, 0.75, 105), (640, 1.0, 110)],
    "doorbell":   [(0, 0.7, 100), (380, 0.4, 100)],
    "chirp":      [(0, 0.9, 110), (120, 0.7, 70)],
}


def _pick_pitch(sorted_pitches: list[int], fraction: float) -> int:
    n = len(sorted_pitches)
    idx = max(0, min(n - 1, int(round(fraction * (n - 1)))))
    return sorted_pitches[idx]


def _snap_to_mapped(pitch: int, sorted_pitches: list[int]) -> int:
    """Return the mapped pitch nearest to `pitch`."""
    return min(sorted_pitches, key=lambda p: abs(p - pitch))


def resolve_song(song: dict, mapped_pitches: list[dict]) -> tuple[list[dict], dict]:
    """Turn a roster `song` block into (events, play_kwargs).

    `song` is one of:
      {"events": [...]}                       -- literal motif events
      {"midi": "songs/x.mid", "fit": "..."}   -- parse a .mid file
      {"builtin": "ascending"}                -- a built-in melody

    Common optional keys: master_current_ma, vel_floor, name.
    `mapped_pitches` is the list returned by /api/pitches.
    """
    sorted_pitches = sorted(p["pitch"] for p in mapped_pitches)
    if not sorted_pitches:
        raise ValueError("no pitches are mapped on the ring; set a mapping first")

    play_kwargs: dict = {}
    for key in ("name", "master_current_ma", "vel_floor"):
        if key in song:
            play_kwargs[key] = song[key]

    if "events" in song:
        events = [dict(e) for e in song["events"]]

    elif "builtin" in song:
        melody = BUILTIN_MELODIES.get(song["builtin"])
        if melody is None:
            raise ValueError(
                f"unknown builtin melody {song['builtin']!r}; "
                f"known: {sorted(BUILTIN_MELODIES)}"
            )
        events = [
            {"t_ms": t, "pitch": _pick_pitch(sorted_pitches, frac), "velocity": vel}
            for (t, frac, vel) in melody
        ]
        play_kwargs.setdefault("name", song["builtin"])

    elif "midi" in song:
        midi_path = song["midi"]
        if not os.path.isabs(midi_path):
            midi_path = str(ROOT / midi_path)
        # Pass through any converter knobs the roster wants to set.
        conv = {k: song[k] for k in
                ("transpose", "min_gap_ms", "max_ms", "max_notes", "default_velocity")
                if k in song}
        events = midi_file_to_events(midi_path, **conv)
        if song.get("fit", "nearest") == "nearest":
            mapped_set = set(sorted_pitches)
            for e in events:
                if e["pitch"] not in mapped_set:
                    e["pitch"] = _snap_to_mapped(e["pitch"], sorted_pitches)
        play_kwargs.setdefault("name", os.path.basename(midi_path))

    else:
        raise ValueError(
            "song must contain one of 'events', 'builtin', or 'midi'; got "
            f"keys {sorted(song)}"
        )

    if not events:
        raise ValueError("resolved song has no playable events")
    return events, play_kwargs


# ---------------------------------------------------------------------------
# Per-day state
# ---------------------------------------------------------------------------


def load_state(path: Path) -> dict:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}


def save_state(path: Path, state: dict) -> None:
    """Atomic write so a crash mid-write can't corrupt the state file."""
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(state, indent=2), encoding="utf-8")
    os.replace(tmp, path)


def today_str() -> str:
    return datetime.now().date().isoformat()


# ---------------------------------------------------------------------------
# Active-hours gate
# ---------------------------------------------------------------------------


def parse_active_hours(spec: str | None) -> tuple[int, int] | None:
    """Parse "HH:MM-HH:MM" into (start_minutes, end_minutes), or None."""
    if not spec:
        return None
    try:
        start_s, end_s = spec.split("-")
        sh, sm = (int(x) for x in start_s.split(":"))
        eh, em = (int(x) for x in end_s.split(":"))
        return sh * 60 + sm, eh * 60 + em
    except (ValueError, AttributeError):
        raise ValueError(f"bad active_hours {spec!r}; expected 'HH:MM-HH:MM'")


def within_active_hours(window: tuple[int, int] | None) -> bool:
    if window is None:
        return True
    now = datetime.now()
    minute = now.hour * 60 + now.minute
    start, end = window
    if start <= end:
        return start <= minute <= end
    # Window wraps past midnight (e.g. 22:00-06:00).
    return minute >= start or minute <= end


# ---------------------------------------------------------------------------
# Playback
# ---------------------------------------------------------------------------


def play_person_song(client: ChimeClient, person: dict, mapped_pitches: list[dict],
                     log=print) -> bool:
    """Resolve and play one person's song, blocking until it finishes.

    Returns True if the motif was posted. Blocks (via wait_for_motif) so the
    caller can play several arrivals back-to-back without each preempting the
    last -- the server only has one motif slot.
    """
    song = person.get("song")
    if not song:
        log(f"  ! {person.get('name', '?')} has no 'song' configured; skipping")
        return False
    try:
        events, play_kwargs = resolve_song(song, mapped_pitches)
    except (ValueError, OSError) as exc:
        log(f"  ! could not build song for {person.get('name', '?')}: {exc}")
        return False

    play_kwargs.setdefault("name", f"welcome:{person.get('name', '?')}")
    try:
        result = client.play_motif(events, **play_kwargs)
    except ChimeClientError as exc:
        log(f"  ! ring server rejected motif for {person.get('name', '?')}: {exc}")
        return False

    log(f"  ♪ playing '{play_kwargs['name']}' "
        f"({result.get('scheduled', '?')} notes, {result.get('duration_ms', '?')} ms)")
    duration_s = (result.get("duration_ms") or 0) / 1000.0
    client.wait_for_motif(timeout=duration_s + 2.0)
    return True


# ---------------------------------------------------------------------------
# Config loading
# ---------------------------------------------------------------------------


def load_config(path: Path) -> dict:
    if not path.exists():
        raise SystemExit(
            f"config not found: {path}\n"
            f"Copy welcome_config.example.json to {path.name} and edit it."
        )
    try:
        cfg = json.loads(path.read_text(encoding="utf-8"))
    except ValueError as exc:
        raise SystemExit(f"config {path} is not valid JSON: {exc}")
    if not isinstance(cfg.get("people"), list):
        raise SystemExit(f"config {path} must have a 'people' array")
    return cfg


def find_person(cfg: dict, name: str) -> dict | None:
    for person in cfg["people"]:
        if person.get("name", "").lower() == name.lower():
            return person
    return None


# ---------------------------------------------------------------------------
# Subcommands
# ---------------------------------------------------------------------------


def cmd_scan(args: argparse.Namespace) -> int:
    ip_range = args.range
    if not ip_range and args.config and Path(args.config).exists():
        ip_range = load_config(Path(args.config)).get("ip_range")
    if not ip_range:
        print("error: no IP range. Pass --range 192.168.1.0/24 (or set "
              "'ip_range' in the config).", file=sys.stderr)
        return 1

    devices = scan_network(
        ip_range, ping_timeout_ms=args.ping_timeout,
        do_netbios=not args.no_netbios, log=lambda m: print(f"[scan] {m}", file=sys.stderr),
    )
    print(f"\n{len(devices)} device(s) on {ip_range}:\n")
    print(f"  {'IP':<16} {'MAC':<18} HOSTNAMES")
    print(f"  {'-' * 16} {'-' * 18} {'-' * 30}")
    for dev in devices:
        names = ", ".join(sorted(dev.names)) or "(no name)"
        print(f"  {dev.ip:<16} {dev.mac or '(no mac)':<18} {names}")
    print("\nAdd the hostname (or MAC) of a coworker's device to the 'match' "
          "block of a person in your config.")
    return 0


def cmd_list(args: argparse.Namespace) -> int:
    cfg = load_config(Path(args.config))
    state = load_state(Path(args.state))
    today = today_str()
    print(f"Roster ({len(cfg['people'])} people)   [today: {today}]\n")
    for person in cfg["people"]:
        name = person.get("name", "?")
        match = person.get("match", {})
        ids = []
        for key in ("hostname", "mac", "ip"):
            for v in _as_list(match.get(key)):
                ids.append(f"{key}={v}")
        played = state.get(name) == today
        flag = "✓ played today" if played else "  waiting"
        song = person.get("song", {})
        song_desc = (song.get("midi") or song.get("builtin")
                     or (f"{len(song.get('events', []))} inline events"))
        print(f"  {flag}  {name:<14} [{', '.join(ids) or 'no identifiers!'}]  song: {song_desc}")
    return 0


def cmd_test(args: argparse.Namespace) -> int:
    cfg = load_config(Path(args.config))
    person = find_person(cfg, args.name)
    if person is None:
        print(f"error: no person named {args.name!r} in the roster.", file=sys.stderr)
        return 1
    client = ChimeClient(base_url=cfg.get("server", DEFAULT_BASE_URL))
    try:
        pitches = client.pitches()
    except ChimeClientError as exc:
        print(f"error: cannot reach ring server: {exc}", file=sys.stderr)
        return 1
    ok = play_person_song(client, person, pitches, log=print)
    return 0 if ok else 1


def cmd_watch(args: argparse.Namespace) -> int:
    cfg = load_config(Path(args.config))
    state_path = Path(args.state)
    state = load_state(state_path)

    ip_range = cfg.get("ip_range") or args.range
    if not ip_range:
        print("error: set 'ip_range' in the config (or pass --range).", file=sys.stderr)
        return 1
    interval = args.interval or cfg.get("interval_s", 25)
    base_url = cfg.get("server", DEFAULT_BASE_URL)
    try:
        active = parse_active_hours(cfg.get("active_hours"))
    except ValueError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    client = ChimeClient(base_url=base_url)

    print(f"[welcome] watching {ip_range} every {interval}s, "
          f"ring server {base_url}")
    print(f"[welcome] {len(cfg['people'])} people in roster"
          + (f", active hours {cfg['active_hours']}" if active else "")
          + ".  Ctrl-C to stop.")

    try:
        while True:
            ts = datetime.now().strftime("%H:%M:%S")
            if not within_active_hours(active):
                print(f"[{ts}] outside active hours; sleeping")
                time.sleep(interval)
                continue

            today = today_str()
            try:
                devices = scan_network(
                    ip_range, ping_timeout_ms=args.ping_timeout,
                    do_netbios=not args.no_netbios, log=lambda _m: None,
                )
            except OSError as exc:
                print(f"[{ts}] scan failed: {exc}")
                time.sleep(interval)
                continue

            # Who is present right now?
            arrivals = []
            for person in cfg["people"]:
                name = person.get("name", "?")
                if state.get(name) == today:
                    continue  # already greeted today
                if any(person_matches(person, dev) for dev in devices):
                    arrivals.append(person)

            if arrivals:
                # Refresh the live mapping once per batch.
                try:
                    pitches = client.pitches()
                except ChimeClientError as exc:
                    print(f"[{ts}] ring server unreachable, will retry: {exc}")
                    time.sleep(interval)
                    continue
                for person in arrivals:
                    name = person.get("name", "?")
                    print(f"[{ts}] {name} arrived -> first time today")
                    if play_person_song(client, person, pitches, log=print):
                        state[name] = today
                        save_state(state_path, state)
            else:
                present = sum(
                    1 for person in cfg["people"]
                    if any(person_matches(person, d) for d in devices)
                )
                print(f"[{ts}] {len(devices)} devices, {present} roster member(s) "
                      f"present, no new arrivals")

            time.sleep(interval)
    except KeyboardInterrupt:
        print("\n[welcome] stopped.")
        return 0


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def main(argv: list[str] | None = None) -> int:
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--config", default=str(DEFAULT_CONFIG),
                        help=f"roster config path (default: {DEFAULT_CONFIG.name})")
    common.add_argument("--state", default=str(DEFAULT_STATE),
                        help=f"per-day state path (default: {DEFAULT_STATE.name})")
    common.add_argument("--ping-timeout", type=int, default=600, metavar="MS",
                        help="per-host ping timeout in ms (default: 600)")
    common.add_argument("--no-netbios", action="store_true",
                        help="skip nbtstat NetBIOS name lookups (Windows)")

    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="cmd", required=True)

    p_scan = sub.add_parser("scan", parents=[common],
                            help="one-shot discovery dump (IP/MAC/hostname)")
    p_scan.add_argument("--range", help="CIDR to scan, e.g. 192.168.1.0/24")
    p_scan.set_defaults(func=cmd_scan)

    p_list = sub.add_parser("list", parents=[common],
                            help="show roster and who triggered today")
    p_list.set_defaults(func=cmd_list)

    p_test = sub.add_parser("test", parents=[common],
                            help="play a person's song now")
    p_test.add_argument("name", help="person's name from the roster")
    p_test.set_defaults(func=cmd_test)

    p_watch = sub.add_parser("watch", parents=[common],
                             help="run the walk-in watcher daemon")
    p_watch.add_argument("--range", help="override config ip_range")
    p_watch.add_argument("--interval", type=int, help="seconds between scans")
    p_watch.set_defaults(func=cmd_watch)

    args = parser.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
