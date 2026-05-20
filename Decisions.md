# Decision Log

Forward-facing, append-only record of architectural and behavioral decisions for this project. Read this file at the start of each session — it's more reliable than memory and supersedes any conflicting claims in CLAUDE.md, AGENTS.md, or README.

When a decision is reversed or superseded, append a new entry rather than rewriting the old one.

## 2026-05-14 — Rev2 PCB: switch bus from UART ring to RS485 multi-drop (tentative)

- **Decision:** Rev2 driver PCBA replaces the 5V UART ring (250 kbaud, cut-through forwarding) with RS485 half-duplex multi-drop. XT30 2+2 connectors retained; daisy-chain power + data through devices with a terminator plug on the last open connector. Addresses pre-programmed per device before bus placement and persisted to flash, replacing the ring-walk `ENTER_STORE_FORWARD` / `SET_ADDRESS` enumeration for rev2 nodes. Protocol shifts to master-polls with broadcast packets for command-rate traffic (extends the existing `BROADCAST_DUTY` slot-extraction pattern); feedback is lower rate. Baud target TBD, watch `uart_max` / `uart_rx_overflow_count` telemetry as it goes up from 250 kbaud.
- **Why:** Biggest concrete win is eliminating the cut-through echo write in `UART1_IRQHandler` (`src/app_uart.c:62-66`) — frees CPU on the 24 MHz M2003, which is already at the deadline edge. Differential signaling adds noise headroom (margin, not a current pain point) and removes the ring's downstream-break failure mode. CAN was considered but rejected: M2003 has no CAN peripheral, and rev2 keeps the M2003 because real-world firmware performance has been excellent.
- **Status:** Tentative. Rev2 schematic in progress. Gen1 ring-bus hardware (24 actuators) is in active use today and the gen1 protocol is unchanged on those units.
- **Supersedes:** (initial)
- **Affects:** rev2 PCB design; future `src/app_uart.c`, `src/protocol.c`, `protocol.md` updates for rev2 firmware path.

## 2026-05-14 — Rev2 PCB: RS485 transceiver control on a single GPIO (PB.14)

- **Decision:** PB.14 is the only spare GPIO and is tied to both DE and /RE for half-duplex control. Biased HIGH externally so during MCU reset/SWD the /RE input is deasserted and RO is tri-stated — frees PF.1 (shared with ICE_CLK) for the programmer. DE is also asserted in that state as a consequence. A ~10 kΩ pulldown on the PF.0 → DI net parks the driver at a defined recessive idle when PF.0 is high-Z, so a device under SWD drives a steady idle rather than spraying SWD edges or random levels onto the bus.
- **Why:** PF.0/PF.1 are shared between UART1 and ICE_DAT/ICE_CLK, so the transceiver's RO output cannot fight the programmer on PF.1. Biasing PB.14 high satisfies that constraint with the one available pin. Full tri-state of the transceiver during reset would need a second GPIO (none available) or a transceiver with a dedicated enable pin — not pursued.
- **Accepted tradeoff:** A device under SWD pauses bus traffic for the duration of the programming session (its transceiver dominates the differential pair driving recessive idle). Acceptable because (a) not safety-critical, (b) workflow programs devices off-bus before placement; no live-bus reflash use case.
- **Status:** Tentative, follows from the rev2 bus topology decision above.
- **Supersedes:** (initial)
- **Affects:** rev2 PCB schematic — RS485 transceiver wiring, PB.14 net, PF.0/DI pulldown.
