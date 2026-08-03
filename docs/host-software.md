# Host software guide

## Setup

Use Python 3.10 or newer. From the repository root:

```powershell
py -m venv .venv
.\.venv\Scripts\Activate.ps1
py -m pip install -r requirements.txt
```

`pyserial` is required for all ring communication. `matplotlib` is used by
tuning/measurement plots, and `mido` is used by the direct General MIDI drummer.
The browser player and MIDI HTTP server do not require Node.js or a JavaScript
build toolchain. The browser's firmware-build button requires GNU Make plus
`arm-none-eabi-gcc`, `arm-none-eabi-objcopy`, and `arm-none-eabi-size` in the
MIDI server process's `PATH`. The Makefile supports Windows and POSIX hosts,
including Raspbian. A failed build is reported in both the browser's Build
output and the server shell.

## MIDI server and browser player

Start the bridge with an explicit serial port:

```powershell
py scripts/ring_midi_server.py -p COM7
```

Optionally expose a directory of MIDI files in the browser library:

```powershell
py scripts/ring_midi_server.py -p COM7 --library-dir C:\Music\robot-midi
```

Then open <http://localhost:8765/>. The normal startup sequence is:

1. Enumerate the ring.
2. Home all actuators.
3. Assign a MIDI note to every actuator slot.
4. Load a MIDI file or select one from the configured library.
5. Start at a conservative master current and raise it while observing the
   instrument.

The server owns the serial port; do not run another ring client against the same
port while it is active. The local pitch mapping is written to `mapping.json`.
The file is machine/instrument state and is ignored by Git.

The HTTP API, payloads, timing compensation, and integration examples are
documented in `midi_server_api.md`.

## Ring diagnostics

List serial ports and enumerate actuators:

```powershell
py scripts/ring_tool.py ports
py scripts/ring_tool.py -p COM7 enumerate
```

Inspect one actuator:

```powershell
py scripts/ring_tool.py -p COM7 status 0
py scripts/ring_tool.py -p COM7 strike-status 0
py scripts/ring_tool.py -p COM7 timing-status 0
```

The tool contains commands that directly drive the motor. Read
`py scripts/ring_tool.py --help` and the subcommand help before using duty,
torque, current, position, home, or strike commands.

## Firmware updates

`scripts/ring_bootload.py` updates APROM through the permanent Gen1 LDROM
loader. It stops the target application, preserves the settings pages, retries
idempotent writes, verifies the exact application CRC, and commits the image
last.

When the MIDI server is running from this repository, the browser Firmware
panel is the normal operator path:

1. Set the image-version label and click **Build**. The server runs the fixed
   `make` command in its checkout, validates `build/m2003-motor.bin`, and freezes
   that exact image in memory. The panel shows source commit/dirty state, byte
   count, CRC-32, and version.
2. Review those values and click **Update ring**. After explicit confirmation,
   the server stops playback, reserves its existing serial connection, performs
   the protocol-3 broadcast data phase, verifies/repairs/commits every actuator
   individually, and requires every application to re-enumerate.
3. Re-home the complete ring before playback.

The GUI intentionally does not run `git pull` or accept an ELF/file upload in
this first version: the checked-out server source is the single build input and
the frozen validated `.bin` is the single update artifact. A failed update may
leave one or more actuators resident in LDROM; correct the cause and retry the
same prepared artifact.

The standalone CLI remains available for recovery and bench work. Stop the MIDI
server before running it because a separate process cannot share the serial
port:

```powershell
py scripts/ring_bootload.py build/m2003-motor.bin -p COM7 --addr 0 `
    --image-version 1

# Program the same image sequentially to every enumerated actuator.
py scripts/ring_bootload.py build/m2003-motor.bin -p COM7 --all `
    --image-version 2

# Protocol 3: transmit the data once, then verify and commit every actuator.
py scripts/ring_bootload.py build/m2003-motor.bin -p COM7 --broadcast-all `
    --image-version 3
```

The single-actuator programming and cold-power recovery gates are complete.
Both `--all` and protocol-3 `--broadcast-all` have passed on a one-device ring;
farthest/middle/nearest updates, multi-node broadcast pacing, and failure
isolation remain bench gates before the first complete fleet run. `--all`
remains the sequential fallback;
`--broadcast-all` refuses older loaders, verifies each device, repairs failures
with addressed commands, and commits individually. `docs/firmware-update.md`
contains the evidence, recovery procedure, and one-time SWD provisioning pass.

## Tuning and measurement

`scripts/tune_tool.py` captures current, velocity, position, and strike step
responses. `scripts/ring_tool.py measure-strike-timing` sweeps strike current or
home offset and can produce CSV/plot evidence. Record the firmware commit,
actuator hardware revision, instrument/note, supply settings, command line, and
test date alongside every capture; a graph without that context is not a
reproducible calibration result.

## Other players and integrations

- `player/looper.html`: browser layer/loop builder.
- `player/tongue-drum-player.html`: instrument-specific player.
- `scripts/ring_midi_drummer.py`: direct General MIDI drum playback.
- `scripts/chime_demo.py`: small standard-library HTTP client example.
- `scripts/ring_welcome.py`: plays a configured motif when a known network
  device arrives; see `docs/ring_welcome.md`.
