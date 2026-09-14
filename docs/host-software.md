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

### Transpose an imported MIDI

Files open with their original pitches. Click **Transpose**, above the piano
roll, to choose and apply the best uniform shift for the pitches assigned to
the enumerated instrument. Each click starts from the original imported notes.

- **Transpose only** keeps intervals and leaves unavailable notes unplayable.
- **Transpose + octave folding** (the initial button option) also moves notes
  into available octaves of the same pitch class.
- **Also substitute missing notes** permits nearby pitch-class substitutions.

Open **Tracks to include when transposing** to choose voices. Track and channel
identities are retained, including multichannel Type 0 files. Channel 10 is
initially unchecked as the General MIDI percussion channel; enable it explicitly
for files that use that channel melodically. Changing options or selections
does nothing to the notes until **Transpose** is clicked.

Each track/channel row shows its note count, pitch range, and instrument metadata
(or GM family/program number when present). Controls sit directly below each
track description, rather than at the far right of a wide screen. **Show**
checkboxes immediately hide/show source voices; **Include** remains a separate
transposition choice. **Solo view** isolates that voice's
original source notes in the piano roll and scrolls to its first note, even if
the voice is unchecked or excluded from the current adaptation. Track/channel
colors match the note bars during source inspection; hover a note for its source
identity and velocity. **Arrangement · mallet colors** returns to the current
original/transposed playback view, restores all arrangement notes and the
original mallet color scheme, and resets Show checkboxes. The arrangement button
is disabled when that view is already active; Stop computer audition is disabled
when no audition is running.
Previewing never changes the Include checkboxes or the robot playback schedule.

**Audition** plays a synthetic sine-tone excerpt on the computer only, beginning
at that voice's first note and limited to 20 seconds of note starts or 512 notes
(plus at most one second of decay). It is a melody/rhythm aid, not a GM soundfont
or a drum command; percussion is also rendered as pitches. **Stop audition**,
switching preview, opening a valid file, transposing/restoring, or starting drum
playback stops the audition. Drum playback returns the roll to arrangement view.

The result reports the shift, exact matches, octave moves, substitutions,
unplayable notes, track exclusions, merged hits, and restrike omissions. The piano
roll shows the transformed pitches; note tooltips retain the original pitch.
Timing, tempo changes, durations, and velocities are retained. For transposed
playback, the existing 5 ms simultaneous-hit merge keeps the loudest hit, then
later hits less than 50 ms after the previous retained hit on a mallet are
omitted. This check uses the selected playback speed and the rounded millisecond
schedule; notes are never delayed to make them fit. The 50 ms value is an
instrument guideline, not a measurement of physical impact timing.

**Restore original** returns to the imported notes and existing manual fallback
routing. Transposed playback uses the instrument's exact pitch map instead of
saved fallback routes. A changed instrument mapping requires transposing again
or restoring before playback. Transpose and restore are disabled during playback.

Adaptation applies only to the current playback view. Opening any file, including
the next library track, starts from its original notes. **Save original** copies
the unchanged source MIDI to the library; this feature does not export a rewritten
MIDI file or change the instrument's pitch assignments. Restart the MIDI server
and reload the page after installing this change so it can serve the new
`player/midi_transpose.js` asset.

Software validation: `node --test tests/test_midi_transpose.cjs` exercises the
fitting engine, MIDI import/restore, tempo and channel handling, collision rules,
and the schedule submitted to a simulated server. Run the existing playback
suite with `py -m unittest discover -s tests -p test_ring_playback.py`. Browser
checks use an isolated simulated instrument; musical quality and physical
playback of adapted files still require listening on the instrument.

### Playback and status

The HTTP API, payloads, timing compensation, and integration examples are
documented in `midi_server_api.md`.

During MIDI playback, only events with the same requested impact time are
treated as a chord. Chord commands are pipelined farthest-address first with
firmware replies suppressed, preventing a missing reply from blocking later
notes. Isolated notes retain timed acknowledgments. The browser follows the
server's playback state through completion and never cancels a song merely
because its nominal wall-clock duration elapsed.

Status refreshes and bus-health probes yield to playback on the server, even
when requested from another browser or an integration. During playback they
use cached status and existing strike traffic rather than adding bus queries.
While idle, polls release the bus after each transaction; waiting commands
take priority, and overlapping status/probe requests share the cache instead
of queuing more sweeps. A query already on the wire must still finish or time
out before a command can use the connection.

The player labels cached or unavailable state and shows its age in the state
tooltip. Refresh while idle for current readings. Homing/recovery completion
checks require fresh status. This change runs entirely on the host; it adds no
encoder checks, control-loop work, or firmware protocol fields.

**Hardware validation, 2026-08-03:** on the 14-actuator handpan, chord impacts
were reported as nearly perfectly synchronized. Removing chord reply waits also
let rapid layered melodies retain their intended following notes instead of
clumping or dropping them; long-used MIDI files played audibly cleaner with the
same mappings and strike settings.

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
Both `--all` and protocol-3 `--broadcast-all` passed on a one-device ring, and
the instrument-server GUI subsequently completed a protocol-3 update of the
full 14-actuator handpan ring. The Gen1 loader is stable and maintenance-only.
`--all` remains the sequential fallback;
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
