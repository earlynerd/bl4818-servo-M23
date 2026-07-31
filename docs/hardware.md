# Hardware guide

## Current hardware generation

The actuators built so far use a manually modified BL4818 integrated motor-
drive PCB. This is the known working hardware generation. A future board may
replace the complete BL4818 drive PCB as a more drop-in design, but that board
is not the current replication target.

The present retrofit has four main parts:

1. The BL4818 motor and integrated three-phase drive PCB.
2. Manual drive-board modifications for the replacement microcontroller,
   amplified shunt-current measurement, signal routing, and added cabling.
3. A custom encoder/interface PCB carrying the MT6701, local capacitance,
   power/data connectors, and programming connector.
4. A printed three-piece enclosure plus mallet and magnet holders.

## Manual BL4818 drive-board modifications

At a high level, the working modification consists of:

- removing the original onboard microcontroller and installing a Nuvoton M2003;
- mounting an INA180B2 current-shunt amplifier as a bodge over the existing
  shunt area;
- removing selected resistors and shorting selected positions to reroute or
  adapt the relevant signals;
- installing an additional cable assembly alongside the original BL4818 cable.

This list deliberately does not identify pads or resistor designators yet. The
modification is fine-pitch, electrically consequential work on a motor power
stage; it should not be attempted from prose alone. The released instructions
need annotated before/after photographs, reference designators, wire endpoints,
wire gauge, insulation/strain-relief details, and continuity checks.

### Microcontroller

The replacement target is confirmed as the Nuvoton M2003 Cortex-M23. The
firmware, compiler target, linker setup, pin mapping, and hardware documentation
should all use M2003/Cortex-M23 consistently. The exact orderable suffix and
package should be recorded in the released BOM.

## Custom encoder/interface PCB

Both the original BL4818 cable assembly and the added cable assembly plug into
the custom board at the back of the actuator. The board currently contains:

- an MT6701 magnetic absolute encoder;
- ceramic decoupling/bulk capacitors;
- polymer electrolytic bulk capacitors;
- two XT30 2+2 power/data connectors; and
- an SWD programming port.

The pair of XT30 2+2 connectors supports the physical power/data chain through
the actuator. The released schematic and wiring guide still need to define the
exact pin assignments, mating connector orientation, input/output convention,
voltage/current rating, grounding, and behavior of an unpowered actuator.

Editable electronics live under `pcb/`. Once a known-good revision is chosen,
publish a revisioned BOM and fabrication package generated from that source.

![Connector end of an assembled actuator showing the RX, TX, and SWD openings](images/actuator/closeup_with_connectors.jpg)

The finished end cap exposes separate RX and TX XT30 2+2 connectors plus a
recessed SWD opening. Final documentation still needs the connector pin
assignments and the SWD adapter/cable mating details.

## Mechanical assembly

The electronics and motor install into a three-piece printed shell:

1. The BL4818 motor bolts to the motor-side shell piece.
2. The mallet holder slides over the motor bell.
3. A magnet holder fits onto the motor shaft. The shaft is internal rather than
   externally exposed, so the holder locates the encoder magnet inside the
   enclosure.
4. The original and added drive-board cables plug into the custom MT6701 board.
5. The custom board is installed behind the motor/magnet assembly.
6. The end cap screws onto the shell and closes the actuator.

The source CAD and printable exports belong under `mechanical/`. The associated
assembly guide should state print material and orientation, support strategy,
fasteners, tolerances, magnet dimensions/polarity, encoder air gap and
alignment, cable routing, mallet position, and installation clearances.

![Nine completed actuator bodies and mallets viewed from above](images/actuator/9_actuators_in_a_row_topview.jpg)

The current mechanical drop includes a SolidWorks 2026 top-level assembly,
native part models for the base, encoder cover, interface-PCB reference, and
mallet holder, plus AP214 STEP exports for the base, encoder cover, and mallet
holder. See `mechanical/README.md` for the file manifest.

## Photograph set needed

Place documentation images under `docs/images/actuator/`. The most useful set
for a new builder is:

- additional fully assembled actuator angles;
- three shell pieces and small mechanical parts laid out separately;
- exploded assembly in physical order;
- stock BL4818 drive board before modification;
- modified drive board, top and bottom, with every rework point annotated;
- close-up of the MCU replacement and INA180B2 bodge;
- original and added cable assemblies with connector orientation visible;
- custom encoder/interface PCB, front and back;
- magnet holder, magnet orientation, and installed encoder air gap;
- SWD connection during flashing; and
- two adjacent actuators showing the power/data daisy chain.

The repository now includes an assembled connector-end close-up, a row of nine
finished actuators, and ten actuators installed around a tongue drum:

![Ten actuators installed around a tongue drum](images/actuator/instrument_with_10_actuators.jpg)

Every image used as an instruction should identify the hardware revision and
viewing orientation. Avoid relying on wire color alone; labels and pin numbers
must agree with the schematic.

## Future direct-replacement drive PCB

The long-term hardware direction is a replacement for the BL4818 integrated
drive PCB that removes the involved donor-board rework. Keep that work in a
clearly named revision/project area and do not overwrite the Gen1 retrofit
instructions. The transition criteria should include equivalent motor-drive
behavior, current sensing, encoder interface, connectors, SWD access, thermal
performance, protection behavior, and a complete bench comparison against a
known-good modified actuator.
