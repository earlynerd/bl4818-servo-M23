#!/usr/bin/env python3
"""
HTTP bridge between a browser-based MIDI player and the ring bus.

Owns the serial port, exposes a tiny REST surface, and serves the static
player HTML. The browser handles MIDI parsing, scheduling, and the
pitch -> address mapping; this process just forwards strike commands to
the ring at the moment they arrive.

Endpoints
---------
GET  /                  -> redirect to /player.html
GET  /player.html       -> serves player/midi_player.html
GET  /looper.html       -> serves player/looper.html (layer/loop builder)
GET  /api/status              -> {"count": N, "homed": [bool * N], "slots": [{...}]}
GET  /api/strike-timing       -> cached compact timing per slot
GET  /api/strike-timing?addr=N -> fresh compact poll for one slot
GET  /api/mapping             -> {"mapping": [int|null, ...] | null}; persisted slot->pitch map
GET  /api/library             -> {"configured": bool, "files": [{"name", "size"}, ...]}
GET  /api/library/<name>      -> raw .mid bytes from the configured --library-dir
POST /api/library/<name>      -> writes raw .mid bytes to --library-dir; 409 on
                                 collision unless ?overwrite=1
POST /api/enumerate           -> re-enumerate; returns same shape as /api/status
POST /api/home                -> {"addresses": [int]?}; homes them, returns ok/error
POST /api/strike              -> {"address": int, "current_ma": int}; ACK + last-strike timing
POST /api/strikes             -> {"strikes": [{"address","current_ma"}, ...]}; batch
POST /api/strike-param        -> {"address": int, "param": str, "value": int}; ACK
POST /api/cancel              -> cancels all active strikes (panic stop)
POST /api/stop                -> {"addresses": [int]?}; motor_stop on each (disables PWM)
POST /api/save-settings       -> {"addresses": [int]?}; persists strike params to NVM
POST /api/mapping             -> {"mapping": [int|null, ...]}; persists slot->pitch map to disk

Usage
-----
    python scripts/ring_midi_server.py
    python scripts/ring_midi_server.py -p COM7 --http-port 8765

Then open http://localhost:8765/ in Chrome.
"""

from __future__ import annotations

import argparse
import json
import sys
import threading
import time
import traceback
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import unquote, urlparse

from ring_bus import (
    CommandAck,
    RingClientV2,
    RingError,
    RingTimeout,
    StrikeTiming,
    auto_detect_port,
    DEFAULT_BAUD,
    REPLY_MODE_ACK,
    REPLY_MODE_ACK_TIMED,
    STRIKE_PARAM_HOME_OFFSET,
    STRIKE_PARAM_COAST_DISTANCE,
    STRIKE_PARAM_HOMING_DUTY,
)


STRIKE_PARAM_BY_NAME = {
    "home_offset": STRIKE_PARAM_HOME_OFFSET,
    "coast_distance": STRIKE_PARAM_COAST_DISTANCE,
    "homing_duty": STRIKE_PARAM_HOMING_DUTY,
}

ROOT = Path(__file__).resolve().parent.parent
PLAYER_HTML = ROOT / "player" / "midi_player.html"
LOOPER_HTML = ROOT / "player" / "looper.html"
# Persisted slot->pitch mapping. Lives at the project root so it's shared
# across every browser that connects to the bridge. The previous behavior
# was per-browser localStorage, which reset the mapping every time someone
# opened the page from a fresh machine.
MAPPING_FILE = ROOT / "mapping.json"


class Bridge:
    """Thread-safe wrapper around RingClientV2."""

    def __init__(self, port: str, baud: int, timeout_ms: int = 1000, trace: bool = False):
        # The default in ring_bus is 200 ms, which is enough for a quiet
        # bus but tight when many devices are running motor work in
        # parallel (homing especially). Default the bridge a little
        # higher and let the caller override for very busy setups.
        self.client = RingClientV2(port=port, baudrate=baud, timeout_ms=timeout_ms, trace=trace)
        self.client.open()
        self.lock = threading.Lock()
        self.count: int = 0
        # Per-address cache of the most recent compact strike-timing snapshot
        # we've seen, keyed by ring address. Populated by ACK_TIMED piggyback
        # on strike replies and by explicit /api/strike-timing polls. Each
        # entry is the dict shape produced by _strike_timing_to_dict.
        self.strike_timing: dict[int, dict] = {}

    def enumerate(self) -> int:
        with self.lock:
            self.count = self.client.enumerate()
        return self.count

    def status(self) -> dict:
        with self.lock:
            count = self.count
            homed: list[bool] = []
            slots: list[dict] = []
            for addr in range(count):
                try:
                    s = self.client.query_strike(addr)
                    homed.append(bool(s.homed))
                    slot = {
                        "homed": bool(s.homed),
                        "home_offset": s.home_offset,
                        "coast_distance": s.coast_distance,
                        "homing_duty": s.homing_duty,
                    }
                    last_timing = self.strike_timing.get(addr)
                    if last_timing is not None:
                        slot["last_timing"] = last_timing
                    slots.append(slot)
                except Exception:
                    homed.append(False)
                    slots.append({
                        "homed": False,
                        "home_offset": None,
                        "coast_distance": None,
                        "homing_duty": None,
                    })
        return {"count": count, "homed": homed, "slots": slots}

    def strike(self, address: int, current_ma: int) -> dict:
        # ACK_TIMED costs 12 extra payload bytes (~480 us at 250 kbaud) and
        # replaces the otherwise-needed query_strike round trip. Net: less
        # ring traffic per strike, plus the host gets timing for free.
        with self.lock:
            reply = self.client.strike(
                address, current_ma, reply_mode=REPLY_MODE_ACK_TIMED
            )
        return self._ack_to_dict(address, reply, cache_metrics=True)

    def strikes(self, items: list[dict]) -> list[dict]:
        results: list[dict] = []
        with self.lock:
            for item in items:
                addr = int(item["address"])
                cur = int(item["current_ma"])
                reply = self.client.strike(
                    addr, cur, reply_mode=REPLY_MODE_ACK_TIMED
                )
                results.append(self._ack_to_dict(addr, reply, cache_metrics=True))
        return results

    def query_strike_timing(self, address: int) -> dict:
        """Compact poll. Use this to harvest the very last strike's timing
        (e.g. end of a song where there's no follow-up strike to piggyback
        on, or when callers want a synchronous snapshot)."""
        with self.lock:
            timing = self.client.query_strike_timing(address)
        snap = self._strike_timing_to_dict(timing)
        if snap is not None:
            self.strike_timing[address] = snap
        return snap or {"address": address, "sequence": 0, "flags": 0}

    def set_strike_param(self, address: int, param_id: int, value: int) -> dict:
        with self.lock:
            reply = self.client.set_strike_param(
                address, param_id, value, reply_mode=REPLY_MODE_ACK
            )
        return self._ack_to_dict(address, reply)

    def home(self, addresses: list[int], timeout_ms: int = 8000) -> None:
        """Fire-and-forget homing per address — mirrors ring_tool's
        strike-home behavior. Sends each command with the default
        REPLY_MODE_FULL and on RingTimeout re-enumerates and tries once
        more (same shape as ring_tool's retry_after_enumerate). No host-side
        polling: the device homes asynchronously, and callers that need to
        know when each slot has settled can poll /api/status — the
        `homed` field flips to true per slot once the device finishes."""
        del timeout_ms  # retained for API compat; no longer relevant
        with self.lock:
            for addr in addresses:
                self._strike_home_with_retry(int(addr))

    def _strike_home_with_retry(self, addr: int) -> None:
        try:
            self.client.strike_home(addr)
            return
        except RingTimeout as first_exc:
            # Single retry path — match ring_tool: re-enumerate, then try
            # the command once more. Re-enumeration is a quick re-sync
            # that often clears whatever transient parser state caused
            # the missed reply.
            try:
                self.count = self.client.enumerate()
            except RingError:
                raise first_exc
            self.client.strike_home(addr)

    def cancel_all(self) -> None:
        with self.lock:
            for addr in range(self.count):
                try:
                    self.client.strike_cancel(addr, reply_mode=REPLY_MODE_ACK)
                except Exception:
                    pass

    def stop(self, addresses: list[int]) -> list[dict]:
        """Send SUBCMD_STOP to each address — disables PWM and drops target
        velocity/current/duty to zero, motor goes to MOTOR_IDLE. Returns a
        per-slot ack list so the UI can surface partial failures."""
        results: list[dict] = []
        with self.lock:
            for addr in addresses:
                a = int(addr)
                try:
                    reply = self.client.stop(a, reply_mode=REPLY_MODE_ACK)
                    results.append(self._ack_to_dict(a, reply))
                except Exception as exc:
                    results.append({"address": a, "accepted": False, "error": str(exc)})
        return results

    def save_settings(self, addresses: list[int]) -> list[dict]:
        """Persist the current strike-param block on each address to the
        device's NVM. Slot must not be running a strike — firmware rejects
        with NOT_READY otherwise."""
        results: list[dict] = []
        with self.lock:
            for addr in addresses:
                a = int(addr)
                try:
                    reply = self.client.save_settings(a, reply_mode=REPLY_MODE_ACK)
                    results.append(self._ack_to_dict(a, reply))
                except Exception as exc:
                    results.append({"address": a, "accepted": False, "error": str(exc)})
        return results

    def load_mapping(self) -> list | None:
        """Return the persisted slot->pitch mapping, or None if no file exists
        yet. Entries are int (MIDI pitch 0-127) or null (slot disabled)."""
        try:
            raw = MAPPING_FILE.read_text(encoding="utf-8")
        except FileNotFoundError:
            return None
        try:
            obj = json.loads(raw)
        except json.JSONDecodeError:
            return None
        if isinstance(obj, dict):
            obj = obj.get("mapping")
        if not isinstance(obj, list):
            return None
        cleaned: list = []
        for v in obj:
            if v is None:
                cleaned.append(None)
            elif isinstance(v, int) and 0 <= v <= 127:
                cleaned.append(v)
            else:
                cleaned.append(None)
        return cleaned

    def save_mapping(self, mapping: list) -> list:
        """Write the slot->pitch mapping to disk. Returns the normalized list
        that was written."""
        cleaned: list = []
        for v in mapping:
            if v is None:
                cleaned.append(None)
            elif isinstance(v, int) and 0 <= v <= 127:
                cleaned.append(v)
            else:
                # Reject anything that isn't a valid pitch or null — keeps
                # the file readable and matches the browser's own validation.
                raise ValueError(f"mapping entry {v!r} is not null or int 0-127")
        tmp = MAPPING_FILE.with_suffix(MAPPING_FILE.suffix + ".tmp")
        tmp.write_text(json.dumps({"mapping": cleaned}, indent=2), encoding="utf-8")
        tmp.replace(MAPPING_FILE)
        return cleaned

    def close(self) -> None:
        try:
            self.client.close()
        except Exception:
            pass

    def _ack_to_dict(self, address: int, reply, *, cache_metrics: bool = False) -> dict:
        if isinstance(reply, CommandAck):
            payload = {
                "address": address,
                "accepted": reply.accepted,
                "result": reply.result,
                "result_name": reply.result_name,
                "detail": reply.detail,
            }
            metrics_dict = self._strike_timing_to_dict(reply.metrics)
            if metrics_dict is not None:
                payload["metrics"] = metrics_dict
                if cache_metrics:
                    self.strike_timing[address] = metrics_dict
            return payload
        return {
            "address": address,
            "accepted": False,
            "result": -1,
            "result_name": "NO-ACK",
            "detail": 0,
        }

    @staticmethod
    def _strike_timing_to_dict(timing) -> dict | None:
        if not isinstance(timing, StrikeTiming):
            return None
        if not timing.has_data:
            # An "empty" snapshot (no strike has completed yet on this device).
            # Don't pollute the cache with it; return None so callers know.
            return None
        return {
            "address": timing.address,
            "sequence": timing.sequence,
            "flags": timing.flags,
            "trigger_to_coast_ms": timing.trigger_to_coast_ms,
            "trigger_to_impact_ms": timing.trigger_to_impact_ms,
            "trigger_to_rebound_ms": timing.trigger_to_rebound_ms,
            "estimated_strike_velocity_dps": timing.estimated_strike_velocity_dps,
            "rebound_timeout": timing.rebound_timeout,
        }



class Handler(BaseHTTPRequestHandler):
    bridge: Bridge | None = None
    # Optional on-disk MIDI library. When set by --library-dir, /api/library
    # lists .mid/.midi files in this folder (flat, no recursion) and
    # /api/library/<name> serves the raw bytes.
    library_dir: Path | None = None

    def log_message(self, fmt, *args):
        # Quieter than the default per-request stderr spam.
        sys.stderr.write("%s - %s\n" % (self.address_string(), fmt % args))

    # ---- helpers ---------------------------------------------------------

    def _cors(self) -> None:
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")

    def _json(self, code: int, payload) -> None:
        body = json.dumps(payload).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self._cors()
        self.end_headers()
        self.wfile.write(body)

    def _read_json(self):
        n = int(self.headers.get("Content-Length", "0") or "0")
        if n <= 0:
            return {}
        raw = self.rfile.read(n)
        if not raw:
            return {}
        return json.loads(raw)

    # ---- methods ---------------------------------------------------------

    def do_OPTIONS(self):
        self.send_response(204)
        self._cors()
        self.send_header("Content-Length", "0")
        self.end_headers()

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self.send_response(302)
            self.send_header("Location", "/player.html")
            self._cors()
            self.end_headers()
            return

        if self.path in ("/player.html", "/looper.html"):
            html_path = PLAYER_HTML if self.path == "/player.html" else LOOPER_HTML
            try:
                data = html_path.read_bytes()
            except FileNotFoundError:
                self._json(500, {"error": f"Missing {html_path}"})
                return
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(data)))
            self._cors()
            self.end_headers()
            self.wfile.write(data)
            return

        if self.path == "/api/status":
            try:
                self._json(200, self.bridge.status())
            except Exception as exc:
                traceback.print_exc()
                self._json(500, {"error": str(exc)})
            return

        if self.path == "/api/mapping":
            try:
                self._json(200, {"mapping": self.bridge.load_mapping()})
            except Exception as exc:
                traceback.print_exc()
                self._json(500, {"error": str(exc)})
            return

        if self.path.startswith("/api/strike-timing"):
            try:
                # /api/strike-timing?address=N triggers a fresh poll;
                # /api/strike-timing alone returns the cached map.
                from urllib.parse import parse_qs
                qs = parse_qs(urlparse(self.path).query)
                if "address" in qs:
                    addr = int(qs["address"][0])
                    self._json(200, self.bridge.query_strike_timing(addr))
                else:
                    self._json(200, {
                        "slots": [
                            self.bridge.strike_timing.get(a)
                            for a in range(self.bridge.count)
                        ]
                    })
            except Exception as exc:
                traceback.print_exc()
                self._json(500, {"error": str(exc)})
            return

        if self.path == "/api/library":
            try:
                lib = Handler.library_dir
                if lib is None:
                    self._json(200, {"configured": False, "files": []})
                    return
                files = []
                for p in sorted(lib.iterdir(), key=lambda x: x.name.lower()):
                    if p.is_file() and p.suffix.lower() in (".mid", ".midi"):
                        try:
                            size = p.stat().st_size
                        except OSError:
                            continue
                        files.append({"name": p.name, "size": size})
                self._json(200, {"configured": True, "files": files})
            except Exception as exc:
                traceback.print_exc()
                self._json(500, {"error": str(exc)})
            return

        if self.path.startswith("/api/library/"):
            try:
                lib = Handler.library_dir
                if lib is None:
                    self._json(404, {"error": "No library configured"})
                    return
                name = unquote(urlparse(self.path).path[len("/api/library/"):])
                # Reject anything that tries to escape the library dir.
                # Path.name strips directory parts; comparing to the raw
                # request catches "../foo" and "sub/foo" before we touch
                # the filesystem.
                if not name or name != Path(name).name:
                    self._json(400, {"error": "Invalid filename"})
                    return
                if Path(name).suffix.lower() not in (".mid", ".midi"):
                    self._json(400, {"error": "Not a MIDI file"})
                    return
                target = (lib / name).resolve()
                try:
                    target.relative_to(lib.resolve())
                except ValueError:
                    self._json(400, {"error": "Invalid path"})
                    return
                if not target.is_file():
                    self._json(404, {"error": "Not found"})
                    return
                data = target.read_bytes()
                self.send_response(200)
                self.send_header("Content-Type", "audio/midi")
                self.send_header("Content-Length", str(len(data)))
                self._cors()
                self.end_headers()
                self.wfile.write(data)
            except Exception as exc:
                traceback.print_exc()
                self._json(500, {"error": str(exc)})
            return

        self.send_error(404, "Not Found")

    def do_POST(self):
        try:
            if self.path == "/api/enumerate":
                count = self.bridge.enumerate()
                self._json(200, {"count": count})
                return

            if self.path == "/api/home":
                data = self._read_json()
                count = self.bridge.count
                addrs = data.get("addresses")
                if not addrs:
                    addrs = list(range(count))
                addrs = [int(a) for a in addrs]
                self.bridge.home(addrs, timeout_ms=int(data.get("timeout_ms", 8000)))
                self._json(200, {"ok": True, "addresses": addrs})
                return

            if self.path == "/api/strike":
                data = self._read_json()
                addr = int(data["address"])
                cur = int(data["current_ma"])
                self._json(200, self.bridge.strike(addr, cur))
                return

            if self.path == "/api/strikes":
                data = self._read_json()
                items = data.get("strikes") or []
                self._json(200, {"results": self.bridge.strikes(items)})
                return

            if self.path == "/api/strike-param":
                data = self._read_json()
                addr = int(data["address"])
                name = str(data["param"])
                param_id = STRIKE_PARAM_BY_NAME.get(name)
                if param_id is None:
                    self._json(400, {"error": f"unknown strike param '{name}'"})
                    return
                value = int(data["value"])
                if value < -32768 or value > 32767:
                    self._json(400, {"error": "value out of int16 range"})
                    return
                self._json(200, self.bridge.set_strike_param(addr, param_id, value))
                return

            if self.path == "/api/cancel":
                self.bridge.cancel_all()
                self._json(200, {"ok": True})
                return

            if self.path == "/api/stop":
                data = self._read_json()
                count = self.bridge.count
                addrs = data.get("addresses")
                if not addrs:
                    addrs = list(range(count))
                results = self.bridge.stop([int(a) for a in addrs])
                self._json(200, {"results": results})
                return

            if self.path == "/api/mapping":
                data = self._read_json()
                mapping = data.get("mapping")
                if not isinstance(mapping, list):
                    self._json(400, {"error": "expected {'mapping': [...]}"})
                    return
                try:
                    cleaned = self.bridge.save_mapping(mapping)
                except ValueError as exc:
                    self._json(400, {"error": str(exc)})
                    return
                self._json(200, {"ok": True, "mapping": cleaned})
                return

            if self.path == "/api/save-settings":
                data = self._read_json()
                count = self.bridge.count
                addrs = data.get("addresses")
                if not addrs:
                    addrs = list(range(count))
                results = self.bridge.save_settings([int(a) for a in addrs])
                self._json(200, {"results": results})
                return

            if self.path.startswith("/api/library/"):
                from urllib.parse import parse_qs
                lib = Handler.library_dir
                if lib is None:
                    self._json(404, {"error": "No library configured"})
                    return
                parsed = urlparse(self.path)
                name = unquote(parsed.path[len("/api/library/"):])
                qs = parse_qs(parsed.query)
                overwrite = (qs.get("overwrite", [""])[0] or "").lower() in ("1", "true", "yes")
                if not name or name != Path(name).name:
                    self._json(400, {"error": "Invalid filename"})
                    return
                if Path(name).suffix.lower() not in (".mid", ".midi"):
                    self._json(400, {"error": "Not a MIDI file"})
                    return
                target = (lib / name).resolve()
                try:
                    target.relative_to(lib.resolve())
                except ValueError:
                    self._json(400, {"error": "Invalid path"})
                    return
                existed = target.exists()
                if existed and not overwrite:
                    self._json(409, {"error": "File exists", "name": name})
                    return
                n = int(self.headers.get("Content-Length", "0") or "0")
                if n <= 0:
                    self._json(400, {"error": "Empty body"})
                    return
                # Soft cap. A normal MIDI file is well under a megabyte; this
                # is just here to keep a misuse from chewing up disk.
                MAX_BYTES = 8 * 1024 * 1024
                if n > MAX_BYTES:
                    self._json(413, {"error": "File too large"})
                    return
                body = self.rfile.read(n)
                # Sanity check the header so we don't litter the library with
                # random uploads that aren't even MIDI.
                if not body.startswith(b"MThd"):
                    self._json(400, {"error": "Not a MIDI file (missing MThd header)"})
                    return
                target.write_bytes(body)
                self._json(201, {"ok": True, "name": name, "size": len(body),
                                 "overwritten": existed})
                return

            self.send_error(404, "Not Found")
        except KeyError as exc:
            self._json(400, {"error": f"missing field {exc}"})
        except json.JSONDecodeError as exc:
            self._json(400, {"error": f"bad json: {exc}"})
        except Exception as exc:
            traceback.print_exc()
            self._json(500, {"error": str(exc)})


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-p", "--port", help="Serial port (default: auto-detect)")
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD, help=f"Baud rate (default: {DEFAULT_BAUD})")
    ap.add_argument("--host", default="127.0.0.1", help="HTTP bind host (default: 127.0.0.1)")
    ap.add_argument("--http-port", type=int, default=8765, help="HTTP bind port (default: 8765)")
    ap.add_argument("--timeout-ms", type=int, default=1000,
                    help="Per-frame RX timeout in ms (default: 1000). Bump higher if you see"
                         " transient 'timeout waiting for frame' errors during homing.")
    ap.add_argument("--trace", action="store_true", help="Print raw ring TX/RX frames")
    ap.add_argument("--library-dir", default=None,
                    help="Folder of .mid/.midi files exposed at /api/library. "
                         "Flat listing only — no recursion.")
    args = ap.parse_args(argv)

    library_dir: Path | None = None
    if args.library_dir:
        library_dir = Path(args.library_dir).expanduser().resolve()
        if not library_dir.is_dir():
            print(f"--library-dir {library_dir} is not a directory", file=sys.stderr)
            return 1

    port = args.port or auto_detect_port()
    if not port:
        print("No serial port found; pass -p/--port", file=sys.stderr)
        return 1

    print(f"Opening {port} at {args.baud} baud (rx timeout {args.timeout_ms} ms)")
    bridge = Bridge(port=port, baud=args.baud, timeout_ms=args.timeout_ms, trace=args.trace)
    try:
        count = bridge.enumerate()
        print(f"Enumerated {count} device(s) on the ring")
    except Exception as exc:
        print(f"Enumeration failed: {exc}", file=sys.stderr)
        bridge.close()
        return 2

    Handler.bridge = bridge
    Handler.library_dir = library_dir
    server = ThreadingHTTPServer((args.host, args.http_port), Handler)
    url = f"http://{args.host}:{args.http_port}/"
    print(f"Serving {PLAYER_HTML.name} at {url}")
    print(f"Looper UI available at {url}looper.html")
    if library_dir is not None:
        print(f"MIDI library: {library_dir}")
    print("Open that URL in Chrome to play.  Ctrl-C to stop.")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down")
    finally:
        server.server_close()
        bridge.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
