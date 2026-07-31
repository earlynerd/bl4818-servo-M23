# Replication guide

This is the staged path toward reproducing the instrument. It distinguishes
verified repository content from information that still has to be supplied for
a released hardware revision.

## 1. Choose a known hardware revision

Do not order parts from the live KiCad working tree without first declaring
which controller modification and interface revision are known to match the
current firmware. A release package should contain a revisioned schematic, PCB,
BOM, placement file if applicable, fabrication archive, and assembly drawing.

Current design sources are in:

- `pcb/modchip/flexcircuit/` for the M2003 retrofit electronics.
- `pcb/interface/` for the interface electronics and related CAD.
- `MS51-M2003_pinmap.txt` for the controller pin-remapping work.

The proven hardware is currently a manual retrofit, not a replacement motor-
drive PCB. It starts with the integrated BL4818 drive board and replaces its
microcontroller, adds an INA180B2 shunt amplifier, changes several resistor
links, and adds a second cable assembly. Both cable assemblies terminate at the
custom encoder/interface board. See `docs/hardware.md` for the known high-level
architecture and the exact details still required before attempting the mods.

## 2. Build one actuator mechanically

A complete release needs editable CAD and print-ready exports for the three-
piece shell, mallet holder, encoder magnet holder, rest position, travel stops,
instrument mounting, and cable strain relief. Add the maintained files under
`mechanical/` using the conventions in its README. Build and validate a single
actuator before producing a batch.

The firmware assumes a 10-pole motor (five pole pairs), hall commutation, and a
14-bit MT6701 absolute encoder. Changing those assumptions requires reviewing
`include/m2003_config.h` and retuning the control system.

The current physical assembly sequence is:

1. Bolt the motor to the motor-side shell piece.
2. Slide the mallet holder over the motor bell.
3. Fit the magnet holder to the otherwise internal motor shaft.
4. Plug the modified drive-board cable assemblies into the custom MT6701 board.
5. Install the board and fasten the screw-on end cap.

This is an architecture description, not yet a complete assembly instruction;
orientations, clearances, fasteners, print settings, and cable routing must be
captured alongside the CAD and photographs.

## 3. Wire power, programming, and the ring

Before applying power, the released hardware documentation must state:

- supply voltage range, normal draw, peak draw, and recommended current limit;
- connector manufacturer/part number, pin numbering, polarity, and wire gauge;
- fuse, reverse-polarity, transient, and emergency-disconnect strategy;
- SWD programming connector pinout;
- ring TX/RX direction and the correct first/last-device connection.

The data link is 250,000 baud and uses Ring Bus Protocol v2. The host enumerates
the physical chain and assigns addresses 0 through 15. See `protocol.md` for
packet forwarding and the exact enumeration sequence.

## 4. Build and flash firmware

Follow `docs/firmware.md`. Flash with motor power disconnected where the
hardware allows it, verify the image, and inspect the board for unexpected heat
or current before connecting the mechanism.

## 5. Bring up one actuator

Use a current-limited supply and the command-line tool:

```powershell
py scripts/ring_tool.py -p COM7 enumerate
py scripts/ring_tool.py -p COM7 status 0
py scripts/ring_tool.py -p COM7 timing-status 0
```

Confirm sensor direction, encoder validity, current offset, phase order, fault
handling, and physical travel before homing. Then establish safe starting
values for homing duty, home offset, coast distance, strike current, and dead-
strike parameters. Persist settings only after repeated power-cycle tests.

## 6. Assemble and validate a ring

Add actuators one at a time. After each addition, enumerate repeatedly, query
every address, and inspect bus timing/error counts. Confirm that a faulted or
unpowered device behaves as expected for the chosen topology. One ring can
contain at most 16 addresses in the current protocol.

## 7. Configure the player

Follow `docs/host-software.md`. In the browser, map each enumerated slot to the
MIDI note played by its physical instrument. Validate timing with isolated
notes, chords, and fast repeated notes before playing an unrestricted MIDI
file. Save the instrument's mapping and calibration as deployment data, not as
unexplained source-code defaults.

## 8. Record the replica

For each built instrument, record controller and interface revisions, firmware
commit, power supply, actuator-to-note mapping, persisted tuning parameters,
known mechanical deviations, and the final bench-test results. This record is
what makes later repairs and additional batches reproducible.
