# Replication status

The control software is proven on real hardware, but the repository is not yet
a self-contained build package for a new person. This checklist is the boundary
between what can be followed today and what still needs project-owner input.

## Present and usable

- Complete M2003 application source and linker/build files.
- Windows build and J-Link flash scripts with readback verification.
- Ring protocol implementation in firmware and Python.
- Browser MIDI player, HTTP bridge, looper, and integration API.
- Hardware-validated synchronized chord dispatch on the 14-actuator handpan,
  including rapid layered MIDI passages.
- Stable, maintenance-only Gen1 LDROM recovery loader plus successful
  protocol-3 update of the complete 14-actuator ring.
- Diagnostics, tuning, strike sweeps, and calibration-profile tooling.
- KiCad source for the current flex/modchip and interface designs.
- High-level description of the proven manual BL4818 retrofit and mechanical
  assembly sequence.
- SolidWorks 2026 actuator assembly/part sources and AP214 STEP exports for the
  base, encoder cover, and mallet holder.
- Photographs of completed actuator bodies, the connector end cap, and a ten-
  actuator tongue-drum installation.
- M2003 reference manual, Nuvoton device pack, and pin-mapping notes.
- Engineering decision and bench-debug history.

## Needed for an independent replica

- [ ] Name the known-good controller/flex and interface hardware revisions.
- [ ] Export and review a BOM with manufacturer part numbers and substitutions.
- [ ] Publish versioned fabrication archives generated from the known-good PCB
      sources, rather than an unlabeled live working directory.
- [ ] Document the exact BL4818 donor board model and every modification by
      reference designator, before/after connection, wire gauge, and photograph.
- [ ] Document supply voltage, current limit, peak demand, grounding, fusing,
      and reverse/transient protection.
- [ ] Publish connector part numbers, XT30 2+2 pin assignments, cable topology,
      wire gauge, and photographs of a correctly wired ring.
- [x] Add the initial editable SolidWorks assembly/part sources and STEP exports
      under `mechanical/`.
- [ ] Add or identify the magnet holder and any separate third shell component;
      verify assembly references; publish tested STL/3MF files, print settings,
      and a dimensioned assembly order.
- [ ] Specify the motor, encoder magnet, bearings/linkage, mallet, fasteners,
      and instrument-mounting hardware.
- [ ] Add SWD connection and first-flash photographs.
- [ ] Record a conservative first-power/first-motion/first-strike procedure with
      actual supply and current values.
- [ ] Export the known-good default/persisted tuning for at least one actuator
      and explain which values vary per instrument.
- [ ] Add a final acceptance test: enumeration, homing, repeated strikes,
      current/fault check, timing check, and power-cycle recovery.
- [ ] Choose a project-wide license and confirm that redistributed vendor files
      may remain in the repository under their respective terms.

## Cleanup candidates awaiting classification

The following types of files may be useful evidence, but they should not remain
unexplained at repository top level:

- root and `scripts/` CSV/PNG captures;
- KiCad automatic backup archives and editor caches already tracked in history;
- SolidWorks working files and temporary lock files;
- unlabeled Gerber/fabrication exports;
- old audit and bootloader planning notes;
- machine-specific assistant/editor configuration.

For each item, either move it into a named `docs/experiments/<date>-<topic>/`
folder with test context, publish it as a versioned hardware release artifact,
or remove it in a dedicated cleanup commit after confirming it is recoverable
from Git history. No such existing artifact was deleted during the initial
documentation pass.
