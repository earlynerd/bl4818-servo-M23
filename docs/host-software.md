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
port while it is active. Named local pitch mappings are written to
`instruments.json`; an existing `mapping.json` is imported on first use.
The file is machine/instrument state and is ignored by Git.

### Multiple rings on one server

The wire protocol supports **16 actuators per ring**, addressed 0–15. Use
independent USB serial adapters to expand beyond that limit:

```powershell
py scripts/ring_midi_server.py --ring pan=COM7,14 --ring drum=COM8,10
```

Each `--ring NAME=PORT,COUNT` declares a name, serial port, and expected actuator
count (1–16). Repeat it for each ring; do not combine it with `-p`. Each adapter
connects to its own complete ring. The server verifies every count before
starting; all opened ports close if startup fails. Existing `-p COM7` operation
is unchanged.

The example exposes one 24-slot instrument at `http://127.0.0.1:8765/`: slots
0–13 are `pan:0` through `pan:13`, and slots 14–23 are `drum:0` through
`drum:9`. The player and looper show ring/local-address labels. The player offers
an instrument selector for each ring: load an existing saved drum directly,
then home and play as usual. Home, recovery, tuning,
save, stop and cancel route to the selected global slots; fleet operations run
independently across adapters. Server-dispatched schedules and live current/mute
controls use one player. Duplicate mapped pitches still resolve to the first mapped slot;
use explicit slot-addressed events to strike both instruments on the same pitch.

Each ring has its own serial lock, latency estimates and playback worker, with
the same monotonic start time. A reply timeout on one ring does not hold up
another ring's notes. Chords retain the existing no-reply dispatch, including
chords spanning rings. USB/OS scheduling and mechanical variation still affect
actual impact alignment; this is not hardware clock synchronization.

Named-ring selections and shared controls are saved in
`instruments-rings-<layout-id>.json`. Each selector refers to an existing profile
in `instruments.json`; no combined profile is created. Pitch and trim edits save
back to the selected drum, preserving saved entries beyond its current ring
size. Select a different profile for each ring. Create/rename/delete profiles
using single-ring mode. A ring without a selected profile starts unassigned;
a compatible older `mapping-rings.json` is retained as a migration fallback.

The first selected drum supplies the initial shared master current, velocity
floor and compensation settings. Later ring selections retain those shared
controls. Changing them does not overwrite either drum's standalone settings.
Fallback routes are translated to global slots, with earlier rings taking
precedence on duplicate fallback pitches. Subsequent fallback edits are saved
for that combination of selected drums, leaving the original profiles' routes
intact. Exact mapped pitches take precedence over fallbacks as usual.

Ring order, name, port and expected count form the layout identity; changing
them starts a separate set of selections. Browser settings are isolated by
layout and selection. Reload other open players/loopers after changing profiles
or server configuration. Keep adapters assigned to the same physical
rings: equal-size swapped rings cannot be identified from enumeration alone.

For the two ten-mallet drums on COM29 and COM48:

```powershell
py scripts/ring_midi_server.py --ring jameson=COM29,10 --ring retuned=COM48,10
```

Load **Jameson Drum** for `jameson` and **retuned_drum** for `retuned`.
The selections persist across restarts. Slots 0–9 belong to Jameson and 10–19
to the retuned drum. All existing HTTPS/library options can be used with this
command, including `--tls-cert`, `--tls-key`, `--host`, and `--library-dir`.

Re-enumeration stops playback. A wrong count leaves that ring unavailable,
preserves all global slot positions, and blocks a new song until discovery
succeeds. A disconnected ring during playback reports transport failures while
other rings continue. Stop/cancel reports per-slot failures; an unreachable
adapter cannot acknowledge a stop. Firmware updates remain a single-ring
operation: close the multi-ring server and use `-p PORT` for the ring to update.

Software checks: `py -m unittest discover -s tests -p test_ring_fleet.py`
simulates 14+10 actuators and checks routing, HTTP pitch playback, concurrent
dispatch, failure isolation, live controls, cancellation and mapping identity.
`node --test tests/test_ring_fleet_ui.cjs` checks player/looper ring labels,
layout-specific browser settings, per-ring selection, and the firmware update
button in a DOM simulation. `test_fleet_instruments.py` checks selection
persistence, profile-local edits, shared controls, migration and stale writes.
These tests do not open serial ports or move motors. On 2026-10-02, the user
reported successful physical playback with the two-drum COM29/COM48 setup,
describing the nearly twenty-note range as working well and sounding beautiful,
and approved merging `multi-ring` into `main`. This is a user-reported listening
test of the prepared branch (`3c97cd1`); no quantitative cross-ring impact
timing measurements or failure-recovery results were supplied.

#### Two-drum hardware regression checklist

The successful playback trial above cleared the user's merge hold. Retain
this checklist for subsequent hardware regressions; the report does not claim
every item was exercised. Launch with the two-drum command above, then open
<http://localhost:8765/>.

1. Confirm ten actuators on each ring and the correct saved profile in each
   selector. Verify the adapters belong to the labelled drums. Check the
   shared current, velocity floor and compensation before homing or striking.
2. Home the drums. Test individual mallets on both rings, including global
   slots 9 and 10 at the ring boundary. Confirm the physical drum, local
   address, pitch assignment and saved trim agree with the player.
3. Play a short MIDI passage containing notes from both profiles, first
   alternating between drums and then with simultaneous cross-ring notes.
   Check for missed, delayed or misrouted strikes. Record an audio/video
   capture to assess impact alignment; software dispatch timing alone does
   not establish acoustic synchronization.
4. During playback, verify shared current/velocity controls, mute, and
   stop/cancel work on both drums. Restart the server and reload the page;
   confirm both selections and shared controls persist.
5. With playback stopped, check an unavailable adapter or wrong actuator
   count produces a clear error and prevents a new song. Restore the
   connections, re-enumerate and home before further playback.

Record the branch commit, launch command, adapter-to-drum assignments,
firmware versions, MIDI file, settings, results and any captures in the test
notes. Resolve observed regressions and record the hardware results for future
changes.

### Assign pitches with a microphone

Restart the MIDI server after installing this version, then reload the player.
Under **Pitch → Address Mapping**, open **Detect pitches with microphone**.
Select the mallet to measure, then choose:

- **Strike & detect**: allow microphone access and remain quiet while background
  sound is measured. The selected, homed mallet strikes once using the existing
  master current and its trim. There are no automatic repeat strikes.
- **Listen only**: after the prompt, tap that mallet's note by hand. This sends
  no motion commands and works before homing.
- **Detect all mallets**: scans the enumerated mallets in address order with one
  microphone session. The first strike uses the master/trim settings captured
  when the pass starts. If no new strike is heard within 1.8 seconds after the
  command is acknowledged, or sound produces no stable pitch within 3 seconds,
  it retries that same mallet once with up to 20% extra
  current, capped at 200 mA extra and 3000 mA total. This never changes saved
  master/trim settings; a zero-current setting remains zero. If the microphone
  overloads, the retry uses the original current instead of increasing it.

The scan pauses 0.6–1.5 seconds after a measurement, then measures a new local
background reference instead of waiting for silence. Each assignment still
requires a new rise above that reference and a stable pitch. Steady street noise
or continuing ringdown alone cannot trigger another reading.

Readiness is checked before every attempt. A healthy cached status up to two
seconds old is accepted; unavailable/stale status is checked up to three times,
200 ms apart, on the same mallet. This browser's background bus probe pauses
while the pitch dialog is open. A mallet becomes unresolved only after both
acoustic attempts fail; the scan then continues to the next mallet. This includes
noise that triggers an onset but never yields a usable pitch. Unavailable
readiness, unhomed/faulted motors, rejected commands, connection errors and
instrument-count changes stop the pass instead of silently skipping forward.
The affected row shows the actual failure reason. Missing command acknowledgment
never triggers another strike.

If microphone frames stop arriving, a wall-clock watchdog attempts to resume
audio once after 0.75 seconds and reports a microphone interruption after three
seconds without fresh audio. This also covers a frozen audio clock while the
browser still reports audio as running. It does not issue another strike while
the microphone is stalled. Audio-analysis exceptions are shown immediately.
These bounds require the browser's JavaScript timers to be running; hiding the
page still cancels the scan.

Use **Stop scan** to stop future strikes while retaining completed readings.
**Retry unresolved** measures only the remaining mallets, after you let the
instrument become quiet again. You can also select any mallet and use the
single-note controls to replace its proposed reading. Closing the dialog discards
unsaved proposals. Keeping the page in the foreground is required.

Review the results, including any duplicate-pitch warnings, then choose
**Save N detected pitches**. Only successful readings replace entries; unresolved
mallets and any mapping entries outside the enumerated ring are preserved.
The player checks the saved map before writing and reports confirmation failures
without claiming success. Use one operator at a time; this is not an atomic
cross-client calibration reservation.

The result shows the note/octave, frequency, MIDI number and cents relative to
A4 = 440 Hz. **Assign detected pitch** saves it to the existing shared mapping;
detection alone does not change assignments. Select another mallet to continue.
The microphone selector lists available inputs after permission has been granted.
All audio processing happens in the browser; audio is neither uploaded nor saved.

Keep the instrument quiet before each measurement and the page in the foreground.
Stop listening, closing the dialog, or hiding the page releases the microphone.
Stopping listening cannot retract a strike already sent. Normal keyboard strikes,
computer audition and playback in this page are blocked while the dialog is open.
Use one operator/browser during calibration; this first version does not reserve
the instrument against commands from other clients.

The detector covers 80–1500 Hz. It waits for a new sound above the background,
skips the impact transient, and requires agreement across at least six audio
windows. Weak, noisy or clipped signals may be rejected. Results more than 35
cents from the nearest semitone require a retry/tuning check before assignment.
Overtones, missing fundamentals and other ringing notes can still cause octave
errors: check against known notes during initial trials. Stability is not proof
of the correct fundamental. On 2026-09-20 the user reported perfect single-note
recognition using a mobile microphone across two different drums. Full-ring
trials then exposed excessive ringdown waits and cached-status skips. The revised
noise-adaptive timing, readiness handling and bounded stronger retry are covered
by simulated audio/workflow tests; the changes still need a physical scan trial.

Microphone-based volume normalization is a proposed next step, not part of this
release. A useful first version would hold microphone position and input gain
fixed, compare repeated strikes over the same post-impact recording window, then
verify proposed per-mallet trims with another pass. Browser audio processing must
be checked, since requested AGC/noise-suppression settings may not be available.
Measured recording level is relative, not calibrated sound-pressure level or
proof of equal perceived loudness. The current trims are browser-local; mobile
calibration intended for desktop playback would also require shared persistence.

#### Desktop and phone access

On the server PC, open `http://localhost:8765/` (adjust for `--http-port`).
Browsers allow microphone access on localhost without HTTPS. A phone opening
`http://192.168.x.x:8765/` needs trusted HTTPS instead; that phone's `localhost`
would refer to the phone, not the instrument server.
[Browser microphone requirements](https://developer.mozilla.org/en-US/docs/Web/API/MediaDevices/getUserMedia).

For a home-LAN trial, the server now supports direct HTTPS using
`--tls-cert` and `--tls-key`. Without these options, existing HTTP behavior is
unchanged. With them, the selected `--http-port` serves HTTPS only, including
both the page and API. Invalid or incomplete certificate options fail before
the serial port is opened. Certificate issuance/renewal is managed separately.

One development setup uses **mkcert**, installed from its official distribution.
Choose a stable LAN IP for the instrument computer. In the example below,
**replace `192.168.1.50` and `COM7` with your actual server address and serial port**:

```powershell
mkcert -install
New-Item -ItemType Directory -Force certs
mkcert -cert-file certs/player.pem -key-file certs/player-key.pem localhost 127.0.0.1 192.168.1.50
py scripts/ring_midi_server.py -p COM7 --host 192.168.1.50 --tls-cert certs/player.pem --tls-key certs/player-key.pem
```

Install and trust the `rootCA.pem` certificate from `mkcert -CAROOT` on the phone,
then visit `https://192.168.1.50:8765/` on the home Wi-Fi. On iOS, installing
the profile and enabling full certificate trust are separate steps. Android
certificate installation varies by version/browser. Verify the page opens
without a certificate warning before using the microphone. Transfer only the
public CA certificate, never `rootCA-key.pem` or the server's private key.
`certs/` is ignored by Git and is not exposed by the server's asset routes.
Allow the chosen port through the server firewall only from your trusted LAN.
[mkcert setup and mobile trust](https://github.com/FiloSottile/mkcert#mobile-devices).

Pi-hole can optionally resolve a local name such as `drum.home.arpa` to the
instrument computer. Include that exact name when generating the certificate.
Local DNS/Unbound does not itself supply certificate trust. No domain purchase
or VPS is required for this local approach. No trust-store or firewall changes
are made by the player or this code update.

An alternative that avoids manually trusting a certificate is **Tailscale Serve**:

1. Install Tailscale on the server computer and phone, and connect both to your
   private Tailscale network. The server can also be a Raspberry Pi.
2. Keep the MIDI server on its default `127.0.0.1` bind address and start it normally.
3. On that server computer, run `tailscale serve --bg http://127.0.0.1:8765`.
   Follow its setup link if HTTPS needs enabling.
4. Open the HTTPS address printed by Tailscale on the phone, allow its microphone,
   and keep the page open while measuring. Serve forwards both the page and API,
   so there is no mixed HTTP/HTTPS connection to configure.

This uses the phone's microphone while the server still owns the serial cable.
Serve makes the site reachable inside your Tailscale network; restrict access to
the intended operators because the player controls hardware and has no separate
login. No public hosting, purchased domain, or router port forwarding is needed.
Use **Serve**, not public Funnel. To remove this specific background listener,
use `tailscale serve --https=443 off` (check `tailscale serve status` first if
you already use Serve for other services).
[Tailscale Serve setup](https://tailscale.com/docs/features/tailscale-serve),
[command reference](https://tailscale.com/docs/reference/tailscale-cli/serve).

An existing **WireGuard VPS** can also proxy HTTPS to an instrument computer,
once that computer joins the VPN or is reachable through a configured VPN
gateway. Proxy both the page and API and keep player access limited to operators
on the VPN. A hostname you control with a publicly trusted certificate avoids
installing a private root on the phone; DNS-based certificate validation can
keep the web listener private. DNS-provider integration, VPN routing, and any
existing Pi-hole web-port use need checking before deployment. See
[Caddy automatic HTTPS and DNS validation](https://caddyserver.com/docs/automatic-https#dns-challenge).

Software checks: `node --test tests/test_pitch_detection.cjs tests/test_midi_transpose.cjs`
and `python -m unittest discover -s tests -p test_ring_playback.py`.
HTTPS and asset routing checks: `python -m unittest discover -s tests -p test_pitch_https.py`.
Generated-tone and simulated-browser tests do not establish acoustic reliability
on the drum or compatibility with a particular phone/microphone.

### Transpose an imported MIDI

Files open with their original pitches. Click **Transpose**, above the piano
roll, to choose and apply the best uniform shift for the pitches assigned to
the enumerated instrument. Each click starts from the original imported notes.

Candidates are ranked by fewest substituted or unplayable notes, then fewest
unplayable notes, most exact matches, least folding/substitution movement,
smallest absolute shift, and finally a downward shift to break equal ties.
Each criterion only breaks ties in the preceding criteria. Note timing,
same-mallet collisions, and playback speed do not affect the chosen shift;
restrike handling is applied separately when building the playback schedule.

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
