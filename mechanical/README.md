# Mechanical design files

This directory is the maintained source for the actuator enclosure and moving
parts. The current files were created/exported with SolidWorks 2026. Keep native
CAD separate from neutral and print-ready exports so another builder can both
reproduce the proven design and modify it responsibly.

## Directory layout

- `source/`: native SolidWorks assembly/part files and linked components.
- `print-files/`: neutral STEP exports and, when available, revisioned STL or
  3MF files in the verified print orientation.
- Add `drawings/` if dimensioned PDFs, DXFs, or inspection drawings become
  available.

The expected part set includes the three-piece shell, the mallet holder that
slides over the motor bell, and the internal-shaft encoder magnet holder.

## Current file manifest

| File | Role |
| --- | --- |
| `source/actuator.SLDASM` | Top-level actuator assembly |
| `source/base.SLDPRT` | Main motor-side enclosure/base |
| `source/encodercover.SLDPRT` | Rear encoder/interface-board cover |
| `source/interface_pcb.SLDPRT` | Mechanical reference model for the custom PCB |
| `source/mallet_holder.SLDPRT` | Holder that slides over the motor bell |
| `print-files/base.STEP` | AP214 neutral export of the base |
| `print-files/encodercover.STEP` | AP214 neutral export of the encoder cover |
| `print-files/mallet_holder.STEP` | AP214 neutral export of the mallet holder |

The STEP headers identify SolidWorks 2026 and `SwSTEP 2.0`. Before calling this
a complete printable release, add or identify the magnet-holder and remaining
shell component if they are separate parts, verify that the assembly opens
without missing external references, and publish tested STL/3MF orientations or
explicit slicing instructions.

## Release information to include

For each known-good mechanical release, record:

- design/revision name and matching electronics revision;
- native CAD application and version;
- units and export tolerance;
- printer, material, nozzle, layer height, wall count, and infill;
- required supports and print orientation;
- heat-set inserts, screws, bearings, magnets, adhesives, and other hardware;
- critical fits, clearances, and post-processing;
- magnet dimensions, polarity, retention method, and MT6701 air gap;
- mallet/linkage installation and safe travel limits; and
- photographs of the completed assembly.

Do not replace an older known-good export in place. Add a new revision so the
files used for an existing actuator batch remain recoverable.
