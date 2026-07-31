# Repository map

This project combines the embedded actuator, the host controller, and hardware
design material in one repository. This page identifies the maintained source
of truth for each part.

## Firmware

| Path | Purpose |
| --- | --- |
| `src/main.c` | Clock, safety initialization, scheduler, and main loop |
| `src/motor.c` | Current, velocity, and position control |
| `src/commutation.c` | Hall-based BLDC commutation |
| `src/encoder.c` | MT6701 absolute encoder interface |
| `src/strike.c` | Homing, normal-strike, and dead-strike state machine |
| `src/protocol.c` | Ring packet parser, forwarding, commands, and replies |
| `src/persist.c` | Nonvolatile calibration/settings storage |
| `src/timing.c` | Control-loop and protocol timing instrumentation |
| `include/m2003_config.h` | Pin assignment, loop rates, limits, and default tuning |
| `Makefile` | Arm GCC build and firmware image generation |

`CMSIS/` and `Library/StdDriver/` are Nuvoton/platform support. `RTT/` is SEGGER
support. Treat these directories as vendored code unless a documented hardware
fix requires a focused change.

## Host software

| Path | Purpose |
| --- | --- |
| `scripts/ring_bus.py` | Reusable Python implementation of Ring Bus Protocol v2 |
| `scripts/ring_tool.py` | Main command-line diagnostics and calibration interface |
| `scripts/ring_midi_server.py` | Serial-to-HTTP bridge and playback scheduler |
| `scripts/tune_tool.py` | PID and strike step-response capture |
| `scripts/ring_measure.py` | Strike sweep analysis and calibration profiles |
| `scripts/ring_midi_drummer.py` | Direct playback of General MIDI percussion files |
| `scripts/chime_demo.py` | Small HTTP API integration example |
| `scripts/ring_welcome.py` | Network-presence-triggered walk-in chimes |
| `scripts/midi_to_motif.py` | Dependency-free Standard MIDI File parser/converter |

The browser applications in `player/` are static HTML files served directly by
`ring_midi_server.py`; there is no JavaScript build step.

## Hardware and references

| Path | Purpose |
| --- | --- |
| `pcb/modchip/flexcircuit/` | M2003 retrofit/flex electronics in KiCad |
| `pcb/interface/` | Interface electronics plus CAD and current fabrication outputs |
| `mechanical/` | Editable shell/mechanism CAD and ready-to-print exports |
| `docs/images/actuator/` | Photographs used by the hardware and assembly guides |
| `MS51-M2003_pinmap.txt` | Original BL4818-to-M2003 pin mapping notes |
| `docs/en-us--TRM_M2003_Series_EN_Rev1.00.pdf` | M2003 reference manual |
| `scripts/Nuvoton.NuMicroM23_DFP.1.0.7.pack` | Offline Nuvoton device-family pack |

The KiCad source files are the editable design source. Editor caches,
project-local state, auto-backups, and history directories are intentionally
ignored. Gerbers and other fabrication exports should be published as named,
versioned release artifacts once a hardware revision is declared reproducible.

## Engineering records and captured data

`Decisions.md` records architectural decisions, while `DebugLog.md` records
resolved bench failures. CSV and PNG files currently in the root and `scripts/`
are historical captures; they should be moved into named experiment folders
only after their hardware revision and test conditions are identified.
