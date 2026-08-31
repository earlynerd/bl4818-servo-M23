# Walk-in chimes (`ring_welcome.py`)

Plays each coworker "their song" on the actuator ring the first time their
phone or laptop appears on the LAN each day.

It sits entirely on top of the friendly HTTP API in
[`midi_server_api.md`](../midi_server_api.md) — it scans the network, then
POSTs a motif to `/api/play`. It never touches the serial bus, so it can run
on any machine that can reach the ring MIDI server over HTTP.

```
            ┌──────────────┐  ICMP + arp + DNS/NetBIOS   ┌───────────┐
   LAN  ───►│ ring_welcome │◄───────── scan ────────────►│  devices  │
            │   (watcher)  │                             └───────────┘
            └──────┬───────┘
                   │ POST /api/play  (events)
                   ▼
        ring_midi_server.py  ──serial──►  actuator ring
```

## Setup

1. Start the ring MIDI server (it owns the serial port):
   ```
   python scripts/ring_midi_server.py
   ```
2. Make a roster:
   ```
   copy welcome_config.example.json welcome_config.json   # PowerShell: Copy-Item
   ```
3. Discover device names/MACs and fill in the roster:
   ```
   python scripts/ring_welcome.py scan --range 192.168.1.0/24
   ```
   This prints every device it can see with IP / MAC / hostname — the same
   `sly`-style names you'd see in nmap. Copy the relevant hostname (or MAC)
   into a person's `match` block.
4. Verify a song plays:
   ```
   python scripts/ring_welcome.py test Sly
   ```
5. Run the watcher:
   ```
   python scripts/ring_welcome.py watch
   ```

## How devices are detected (stdlib only, no admin)

Each scan:
1. **Ping-sweeps** the IP range concurrently to wake the OS ARP cache.
2. Parses **`arp -a`** for IP → MAC.
3. **Reverse-DNS** each responder; on Windows also runs **`nbtstat -A`** to
   get the NetBIOS workstation name (the `<00> UNIQUE` entry — this is the
   `sly`-style name nmap surfaces).

A device counts as present if it answered ping, is in the ARP table, or
resolved a name. A roster entry matches a device if **any** configured
`hostname`, `mac`, or `ip` matches (hostnames are case-insensitive and match
both the full and short `foo.local` → `foo` forms).

> **Phones sleep.** A phone with its screen off often ignores ICMP, so it may
> not be detected until it next talks to the network on its own (push, Wi-Fi
> keepalive, etc.). Hostname/MAC still match once it does. For rock-solid
> phone presence you'd want active ARP (scapy + Npcap, admin) — deliberately
> not a dependency here.

## "First time each day"

Per-person trigger state is stored in `welcome_state.json` as
`{"Sly": "2026-06-09"}`. A person plays only when their stored date isn't
today's local date, then the date is updated (atomic write). Delete the file
to re-arm everyone. `arrival_delay_s` (default `60`) waits after the first
detection before playing, allowing time to walk from the Wi-Fi boundary to the
door. Pending greetings do not block later network scans and do not require a
sleeping phone to answer every scan. When several greetings are ready together,
`greeting_gap_s` (default `1`) adds a pause after each completed chime before
the next one starts. `active_hours` (e.g. `"06:30-19:00"`, wraps past midnight)
suppresses playback outside working hours.

## Defining a song

Each person's `song` is one of:

| Form | Example | Notes |
|------|---------|-------|
| MIDI file | `{"midi": "songs/sly.mid", "fit": "nearest"}` | Parsed by `midi_to_motif.py`. `fit:"nearest"` snaps every note to the closest mapped tine so it plays on any ring layout. Optional `transpose`, `min_gap_ms`, `max_ms`, `max_notes`. |
| Built-in melody | `{"builtin": "ascending"}` | `ascending`, `descending`, `fanfare`, `pentatonic`, `doorbell`, `chirp`. Adapt to whatever pitches are mapped. |
| Inline events | `{"events": [{"t_ms":0,"pitch":60,"velocity":100}, ...]}` | Literal motif, same shape as `POST /api/play`. |

All forms accept optional `master_current_ma`, `vel_floor`, and `name`.

### Converting MIDI to JSON by hand

`midi_to_motif.py` is also a standalone converter:

```
python scripts/midi_to_motif.py songs/sly.mid                 # print JSON
python scripts/midi_to_motif.py songs/sly.mid --transpose -12 --max-ms 6000
python scripts/midi_to_motif.py songs/sly.mid -o sly.json
```

It drops the drum channel by default and thins same-pitch notes closer than
`--min-gap` ms apart (default 150) so dense tracks stay physically playable —
the firmware needs ~150 ms to re-home a mallet before it can re-strike the
same slot. It does *not* snap to mapped pitches; that happens in the watcher,
which knows the live mapping. Preview any result in the browser player at
`http://localhost:8765/`.

## Commands

| Command | Purpose |
|---------|---------|
| `scan [--range CIDR]` | One-shot discovery dump (IP/MAC/hostname). |
| `list` | Show the roster and who has triggered today. |
| `test <name>` | Resolve and play one person's song immediately. |
| `watch [--interval S] [--delay S] [--range CIDR]` | Run the daemon; `--delay` overrides `arrival_delay_s`. |

Common flags: `--config`, `--state`, `--ping-timeout`, `--no-netbios`.
