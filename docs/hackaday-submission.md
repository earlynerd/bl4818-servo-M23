# Hackaday.io page renovation kit

This is copy and a publishing plan for renovating the existing
[Tongue Drum Music Robot](https://hackaday.io/project/194584-tongue-drum-music-robot)
page, not for creating a competing second project. The existing page preserves
the 2024 first-generation videos, two project logs, its original GitHub link,
and the public history of the machine. The new material should turn that page
into the story of two complete generations.

The current brushless-servo repository remains a proven working project under
documentation, not a turn-key kit. The page should be impressive without
claiming that every replication detail has already been released.

## Recommended page identity

**Title:** Tongue Drum Music Robot: From Spring Mallets to Brushless Servos

**Shorter alternative:** Two Generations of a Tongue Drum Robot

**One-line description:** A ten-mallet spring-return prototype evolved into a
modular 14-node brushless servo instrument with closed-loop motion and
compensated timing.

**Suggested tags:** robotics, music, MIDI, brushless motor, servo, ESP32,
percussion, embedded, Cortex-M23, 3D printing, KiCad, Python

**Project type/status:** hardware, ongoing project

Keep the existing page and slug. Renaming the visible title is optional; the
two-generation story can live entirely in the short description and Details
if preserving the original title is preferable.

## Short description

This project follows two generations of a MIDI-controlled acoustic instrument.
The first used ten brushed DC motors, printed clocksprings, a central ESP32,
BLE MIDI, and microphone-based self-calibration. It worked—and bringing it to
job interviews helped seal the deal on two engineering jobs—but its passive
return mechanics, open-loop strokes, fixed wiring, and centralized controller
set hard limits. The second generation replaces every mallet with an
addressable brushless servo that measures current, velocity, and absolute
position, then joins as many as sixteen actuators on one serial ring. The
current instrument uses fourteen.

## Details field: paste-ready draft

### Results first

This machine has existed in two genuinely different generations.

The first generation proved that a compact array of motorized mallets could
turn MIDI into a real acoustic performance. It was also one of the best
portfolio pieces I have ever built: I carried it into interviews, put a working
electromechanical system on the table, and it helped seal the deal on two jobs.

But a successful prototype is also a very effective list of everything to fix.
The newer instrument is not a cosmetic revision. I redesigned the actuator,
electronics, control architecture, wiring, timing system, diagnostics, and
firmware-maintenance path around the limitations of the first machine.

The result is a network of compact brushless servo mallets. Roughly 35 of the
new actuators have been built, and the current handpan-style instrument uses
14 of them.

**Current performances:**

- [Full-ring performance and timing overview](https://www.instagram.com/sly.vester/reel/DYtgoHPu89_/)
- [Minecraft theme](https://www.instagram.com/sly.vester/reel/Db12a67uSeY/)
- [Performance on a Hamsa handpan](https://www.instagram.com/sly.vester/reel/DaqXpGyupEj/)
- [Overhead view of the synchronized mallets](https://www.instagram.com/sly.vester/reel/DaG4R6Ovw2a/)

### Generation one: the machine that proved the idea

The original instrument used ten brushed DC motors, one for each note. Each
motor drove a mallet through roughly a 70- to 80-degree arc against a
3D-printed clockspring. A single low-side MOSFET could push the motor toward the
drum or let it coast; the spring had to return the mallet to rest.

One ESP32 controlled all ten outputs and received BLE MIDI. An I2S MEMS
microphone detected impacts, measured strike-to-contact delay, identified the
played pitch with FFT-based analysis, and built per-mallet soft/hard timing
models. That calibration system let the machine compensate for a surprising
amount of mechanical variation.

It was a good machine and a strong demonstration of embedded software,
mechanical design, power electronics, signal processing, calibration, and
system integration. Its limitations came from the architecture rather than a
missing tuning constant:

- The motor drive was single-quadrant. It could accelerate toward the drum or
  coast, but it had no H-bridge, active reverse torque, or measured joint
  position.
- The printed clockspring owned the return stroke. Rebound, bump-stop impact,
  oscillation, and settling time limited how quickly a note could repeat.
- Strike force and timing were inferred from PWM and microphone measurements,
  not closed around motor current or mallet position.
- Ten motors were wired back to one controller with a fixed note-to-pin map.
  Adding or replacing a mallet was an instrument-level wiring and firmware job.
- The central controller owned actuation, audio capture, calibration, MIDI,
  timing, and diagnostics. One machine had to do everything at once.

The existing **Results first!** and **AI music on Robotic instrument** logs on
this page should remain as the first-generation record.

### What changed in generation two

| Area | Generation one | Brushless-servo generation |
| --- | --- | --- |
| Actuator | Brushed DC motor plus printed clockspring | BL4818 brushless motor and three-phase drive |
| Motor authority | Single low-side MOSFET: drive forward or coast | Bidirectional commutation with controlled drive, coast, and braking behavior |
| Feedback | Shared microphone measures impact and pitch during calibration | MT6701 absolute encoder, hall commutation, and amplified current sensing in every actuator |
| Motion control | Timed open-loop PWM profiles | Cascaded current, velocity, and position loops plus homing and strike state machines |
| Return stroke | Passive spring return with shaped rebound PWM | Position-aware return, coast thresholds, and velocity-zero braking at defined positions |
| Controller | One ESP32 directly controls ten mallets | One Cortex-M23 controller per actuator |
| Wiring and scale | Ten fixed motor channels return to one controller | Daisy-chained power/data ring with four-bit addresses and up to sixteen actuators |
| Timing | Central calibration model over BLE MIDI | Host-side measured impact latency by actuator/current range, with server-side scheduling |
| Diagnostics | Instrument-level status and microphone diagnostics | Per-node current, position, velocity, timing, fault, and link-health data |
| Maintenance | Reflash the central instrument controller | Addressed in-place application updates with permanent per-actuator recovery loaders |

The first generation treated the mallets as loads connected to an instrument.
The second treats each mallet as a small robot.

### Generation two: a robot at every note

Each new actuator begins with BL4818 integrated brushless-motor hardware. I
replace the original controller with a Nuvoton M2003 Cortex-M23, add amplified
current sensing, and close the position loop with an MT6701 absolute magnetic
encoder. The modified drive, encoder/interface board, connectors, and moving
mechanism fit inside a printed shell. The name **Mini-Bonk** is printed on the
current actuator bodies.

Every unit runs its own current, velocity, and position control plus homing,
normal-strike, and dead-strike state machines. It can report its measured
motion and electrical state, reject a command when it is not ready, preserve
calibration across boots, and stop locally on a detected fault.

The actuators are joined by a small binary ring protocol. Four-bit addresses
support as many as 16 units. A Python server owns the serial link, enumerates
the ring, maps MIDI notes to physical actuators, serves the browser player, and
keeps the musical schedule next to the hardware instead of depending on the
timing of a browser tab or Wi-Fi request.

### The trigger is not the note

The musically difficult part is that an electrical trigger is not the sound of
an impact. A mallet takes tens of milliseconds to move, and that delay changes
with drive current and mechanics.

The host measures trigger-to-impact latency for each actuator and current
range, predicts the next impact, and sends the command early. Chord commands
are transmitted farthest-first without waiting for individual replies, so
host round trips cannot spread a chord or swallow the notes immediately after
it. The complete 14-actuator instrument has demonstrated synchronized chords
and rapid layered passages with this architecture.

### Updating an instrument made of computers

Distributed control fixed the wiring and motion limitations, but it created a
new maintenance problem: a 14-note instrument now contains 14 independent
firmware targets.

Each actuator therefore has a permanent recovery loader. The host builds an
application image and manifest, deploys it through the same ring, verifies the
result, and returns the actuator to the application. The complete 14-actuator
ring has passed an in-place update through the instrument server.

### What is published

The two generations intentionally keep separate repositories:

- [Generation-one ESP32 instrument](https://github.com/earlynerd/RobotDrum)
- [Generation-two brushless servo system](https://github.com/earlynerd/bl4818-servo-M23)

The brushless repository contains bare-metal firmware, host tools, browser
players, PCB sources, mechanical CAD, STEP exports, protocol documentation,
tests, and retained bench/debug history.

### What still needs work

This is working hardware, not a rendering, but the second-generation repository
is not yet a turn-key kit for a new builder. It still needs a released BOM,
exact power and connector limits, annotated BL4818 modification photographs,
tested print exports/settings, and a fully photographed first-power procedure.
A project-wide license also needs to be selected. Those gaps are tracked
explicitly instead of being filled with guesses.

## Media and gallery plan

Use a native YouTube or Vimeo upload as the primary Hackaday video if possible.
Instagram is useful supporting proof, but it presents a login prompt to some
logged-out visitors and is less reliable as the only embedded project video.

Recommended page order:

1. **Open with generation two playing:** [full-ring performance and timing overview](https://www.instagram.com/sly.vester/reel/DYtgoHPu89_/)
2. **Show the earlier machine:** retain one of the original embedded videos
   from the **Results first!** log and label it clearly as generation one.
3. **Put the machines side by side:** one first-generation still, then a clean
   current 14-actuator still from a similar angle.
4. **Use a recognizable result:** [Minecraft theme](https://www.instagram.com/sly.vester/reel/Db12a67uSeY/)
5. **Show another installation:** [Hamsa handpan performance](https://www.instagram.com/sly.vester/reel/DaqXpGyupEj/)
6. **Show the mechanism:** [overhead synchronized mallets](https://www.instagram.com/sly.vester/reel/DaG4R6Ovw2a/)
7. **Provide an account-free fallback:** [56-second repository video](video/handpan_video_h264.mp4)

Recommended new gallery images:

1. A clean horizontal photograph of the current 14-actuator instrument.
2. The best surviving photograph of the ten-mallet spring-return instrument.
3. A side-by-side image labeled `GEN 1` and `GEN 2`.
4. `images/actuator/closeup_with_connectors.jpg` for one finished Mini-Bonk.
5. `images/actuator/9_actuators_in_a_row_topview.jpg` to show repeatability.
6. Internal photographs: brushed motor/clockspring/MOSFET from generation one,
   then stock BL4818 board, modified drive board, M2003/INA180 work,
   encoder/interface PCB, magnet/encoder air gap, and an exploded new actuator.
7. A browser-player screenshot and one measured timing/latency capture.

For a new primary video, open with the finished brushless instrument already
playing. In 75 to 120 seconds, show five to ten seconds of generation one,
state its core limitation, then reveal how the new actuator replaces the
spring-return/open-loop mechanism. End with an uninterrupted musical section.

## YouTube or Vimeo upload copy

**Title:** I Rebuilt My Robotic Drum with 14 Brushless Servos

**Description:**

I built this robotic percussion instrument twice.

Generation one used ten brushed DC motors, printed clocksprings, a central
ESP32, BLE MIDI, and microphone-based self-calibration. It worked well enough
to become a favorite job-interview demo, but passive spring return, open-loop
strokes, fixed wiring, and a centralized controller limited its speed and
scalability.

Generation two replaces every mallet with an addressable brushless servo. Each
actuator closes its own current, velocity, and position loops with an absolute
magnetic encoder. Fourteen actuators now share a serial ring, while the host
measures mechanical trigger-to-impact delay and schedules strikes so the
independent mallets land chords together.

Generation one:
https://github.com/earlynerd/RobotDrum

Brushless-servo generation:
https://github.com/earlynerd/bl4818-servo-M23

Project page:
https://hackaday.io/project/194584-tongue-drum-music-robot

**Thumbnail:** Use a clean split image of both generations, or the complete
brushless ring with several mallets in motion. Keep overlay text to four words,
for example `I BUILT IT TWICE` or `14 BRUSHLESS MALLETS`.

## Components sections

The existing page currently has no Components entries. Add two clearly named
groups. These are architectural lists, not released purchasing BOMs.

### Generation one

- 1 × ESP32 development board
- 10 × brushed DC motor mallet mechanisms
- 10 × single low-side N-channel MOSFET drive stages
- 10 × 3D-printed clocksprings, striker arms, and bump stops
- 1 × I2S MEMS microphone
- 1 × ten-note tongue drum
- 1 × power supply for the motor bank and controller

### Brushless-servo generation

- 1 × BL4818 integrated brushless motor/drive donor assembly per actuator
- 1 × Nuvoton M2003 Cortex-M23 microcontroller per actuator
- 1 × MT6701 absolute magnetic encoder per actuator
- 1 × INA180B2 current-shunt amplifier per actuator
- 1 × custom encoder/interface PCB per actuator
- 2 × XT30 2+2 power/data connectors per actuator
- 1 × printed actuator shell, mallet holder, and encoder-magnet holder
- 1 × mallet/linkage assembly
- 1 × USB serial interface per actuator ring
- 1 × current-limited power supply sized for the tested ring

Do not add unverified voltage, current, connector pinout, wire gauge, or part
substitution claims to the public components list.

## Project-log sequence

Keep the two existing logs. Add the newer story in this order:

1. **I built it twice.** A short personal introduction, the job-interview
   anecdote, and a side-by-side photograph.
2. **Generation one: ten motors, ten springs, one ESP32.** Brushed mallets,
   BLE MIDI, microphone/FFT calibration, and what the first machine proved.
3. **Where the spring-return machine hit its limits.** One-quadrant drive,
   rebound, settling time, open-loop variation, fixed wiring, and centralized
   diagnostics.
4. **A robot at every note.** The BL4818/M2003/MT6701 actuator and why
   distributed closed-loop control changes the instrument.
5. **Packaging Mini-Bonk.** Printed enclosure, magnet/encoder geometry, mallet
   holder, connectors, and the progression to roughly 35 actuators.
6. **One serial ring, fourteen independent servos.** Addressing, forwarding,
   enumeration, homing, diagnostics, and fault isolation.
7. **The trigger is not the note.** Measured trigger-to-impact delay and
   current-aware latency compensation.
8. **Making chords land together.** Server-side scheduling, farthest-first
   dispatch, and the 14-actuator validation that fixed dropped burst notes.
9. **Updating a whole instrument in place.** Application manifests, permanent
   recovery loaders, and the completed full-ring update.
10. **What is proven and what is still missing.** Working systems, public
    design files, and the remaining BOM/power/assembly/license gates.

The detailed second-generation evidence for these logs is in `Decisions.md`,
`DebugLog.md`, `docs/host-software.md`, `docs/firmware-update.md`,
`docs/hardware.md`, and `docs/replication-status.md`. The first-generation
architecture and code are in `C:\Users\mmsyl\Documents\RobotDrum` locally and
the public `earlynerd/RobotDrum` repository.

## Hackaday.com tip-line draft

**Subject:** I rebuilt my MIDI tongue-drum robot with 14 brushless servos

**Comment:**

I built the same robotic percussion instrument twice.

The first version used ten brushed DC motors, printed clocksprings, a central
ESP32, BLE MIDI, and microphone-based timing/pitch calibration. It worked well
enough to become a job-interview demo that helped me land two engineering jobs,
but the single-quadrant drive, passive return mechanics, fixed wiring, and
centralized controller put a ceiling on speed, consistency, and scale.

The second generation turns every mallet into an independent brushless servo.
Each printed actuator has its own Cortex-M23 firmware, current/velocity/position
loops, absolute encoder, homing and strike state machine, fault handling, and a
daisy-chained serial link. A Python/browser MIDI player measures mechanical
trigger-to-impact delay and schedules commands so fourteen independent mallets
land chords together on a handpan-style tongue drum.

Roughly 35 of the new actuators have been built. The fourteen-unit instrument
has played rapid layered pieces, and the complete ring has been updated in
place through permanent per-actuator recovery loaders. The two repositories
show the evolution from the first working prototype to the distributed servo
system, including firmware, host software, PCB/CAD sources, protocol docs,
tests, and bench history.

Project page:
https://hackaday.io/project/194584-tongue-drum-music-robot

Primary video:
https://www.instagram.com/sly.vester/reel/DYtgoHPu89_/

Minecraft performance:
https://www.instagram.com/sly.vester/reel/Db12a67uSeY/

Generation one:
https://github.com/earlynerd/RobotDrum

Brushless-servo generation:
https://github.com/earlynerd/bl4818-servo-M23

## Existing-page update checklist

### Must do

- Preserve the existing page, gallery, and two first-generation logs.
- Replace the one-line Details field with the two-generation draft above.
- Keep the `RobotDrum` external link and add the brushless-servo GitHub link.
- Add a generation-two performance near the top, preferably as a native
  YouTube or Vimeo embed.
- Add at least one first-generation and one second-generation image with clear
  captions.
- Populate the Components section with the two architectural lists.
- Add the **I built it twice** log before the more technical new logs.
- Choose and add a project-wide license for the new repository, with vendor
  exceptions called out, or remain explicit that it is source-visible but not
  yet open-source licensed.

### Strongly recommended

- Add one clean current 14-actuator hero photo and internal electronics photos
  from both generations.
- Name the known-good second-generation hardware revision and publish a
  reviewed BOM.
- Document verified supply limits and the connector pinout.
- Publish versioned fabrication files and tested STL/3MF print files.
- Turn the replication checklist into a photographed first-build sequence as
  the missing facts become available.
- Move or explain the unlabeled root CSV/PNG captures before directing a large
  audience into the repository.

### Submission sequence

1. Update the existing Hackaday.io page while leaving it public or temporarily
   switching it private, according to preference.
2. Check every media and repository link in a logged-out browser.
3. Submit the existing page through **Submit project to → Hackaday.com Tip Line**.
4. Optionally send the shorter pitch above to `tips@hackaday.com`, using the
   existing Hackaday.io page as the main link.
