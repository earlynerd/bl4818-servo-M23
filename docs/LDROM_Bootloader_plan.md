❯ yes, lets make a more formalized plan for this work

● Let me dig into a few more specifics so the plan is grounded in the actual codebase.

  Read 3 files (ctrl+o to expand)

● Bash(arm-none-eabi-gcc --version 2>&1 | head -1; ls C:/Users/mmsyl/Documents/bl4818-servo-M23_2/build 2>&1 | head -20)
  ⎿  arm-none-eabi-gcc.exe (Arm GNU Toolchain 15.2.Rel1 (Build arm-15.86)) 15.2.1 20251203
     Library
     m2003-motor.bin
     … +4 lines (ctrl+o to expand)

● Bash(arm-none-eabi-size "C:/Users/mmsyl/Documents/bl4818-servo-M23_2/build/m2003-motor.elf")
  ⎿     text       data     bss     dec     hex filename
       25032         12    1148   26192    6650 C:/Users/mmsyl/Documents/bl4818-servo-M23_2/build/m2003-motor.elf

● Bash(arm-none-eabi-size -t "C:/Users/mmsyl/Documents/bl4818-servo-M23_2/build/src/crc16.o"
      "C:/Users/mmsyl/Documents/bl4818-servo-M23_2/build/src/startup_m2003.o" "C…)
  ⎿     text       data     bss     dec     hex filename
         596          0       0     596     254 C:/Users/mmsyl/Documents/bl4818-servo-M23_2/build/src/crc16.o
         404          0       0     404     194 C:/Users/mmsyl/Documents/bl4818-servo-M23_2/build/src/startup_m2003.o
     … +5 lines (ctrl+o to expand)

● Good — the size data confirms it fits. Reused primitives (CRC + startup + system init + UART + FMC) total ~3.96 KB as
  currently compiled, which is over budget for 4 KB LDROM by itself. So the plan has to trim those, not just glue them
  together. Here's the formal plan.

  ---
  Ring-Bus LDROM Bootloader — Implementation Plan

  1. Goals / Non-Goals

  Goals
  - Field firmware update over the existing ring bus, no SWD/J-Link required after factory provisioning.
  - Brick-proof: a failed update leaves the device still recoverable via the ring on next power-cycle.
  - Reuse the existing protocol framing, CRC, and forwarding model. No new wire.
  - Update all 16 ring devices in one host command sequence.

  Non-Goals (v1)
  - Cryptographic signing (CRC-only integrity).
  - A/B (dual-bank) APROM. There's no room — APROM is 32 KB and the app uses 25 KB already.
  - Compression / delta updates.

  2. Boot Model

  LDROM-always-first. Set CBS = 01 in user-config at production (one-time SWD operation). On every reset the LDROM
  bootloader runs first and decides whether to stay or jump to APROM.

  Entry decision sequence at LDROM startup:
  1. Read a .noinit RAM magic word (preserved across soft reset).
  2. If magic == STAY_IN_BOOT: clear magic, enter command loop forever.
  3. Else: open UART1 at 250k, enable cut-through echo, listen for N ms (default 100 ms) for an ENTER_BOOT frame.
  4. On ENTER_BOOT: enter command loop. Otherwise: FMC_SetVectorPageAddr(APROM_BASE), load SP/PC from APROM vectors,
  branch.

  App-side new sub-command SUBCMD_ENTER_BOOTLOADER writes the magic and calls NVIC_SystemReset(). Bench-injectable
  recovery path: power-cycle while host streams ENTER_BOOT frames.

  3. Memory Map & Size Budget

  Target LDROM image ≤ 3.8 KB (300 B headroom under 4096).

  ┌────────────────┬──────────────────────┬─────────────────────────────────────────────────────────────┬──────────┐
  │   Component    │        Source        │                          Strategy                           │  Est.    │
  │                │                      │                                                             │  bytes   │
  ├────────────────┼──────────────────────┼─────────────────────────────────────────────────────────────┼──────────┤
  │ Vectors +      │ trimmed              │ Reset_Handler + only used IRQs (UART1, SysTick)             │ ~250     │
  │ startup        │ startup_m2003.c      │                                                             │          │
  ├────────────────┼──────────────────────┼─────────────────────────────────────────────────────────────┼──────────┤
  │ Clock init     │ new bl_clock.c       │ HIRC-only, no PLL/timers                                    │ ~80      │
  ├────────────────┼──────────────────────┼─────────────────────────────────────────────────────────────┼──────────┤
  │ UART           │ trimmed app_uart.c   │ drop echo TX path, drop overflow counters, poll-mode TX,    │ ~500     │
  │                │                      │ smaller RX ring (64 B)                                      │          │
  ├────────────────┼──────────────────────┼─────────────────────────────────────────────────────────────┼──────────┤
  │ CRC-16         │ crc16.c (existing)   │ bitwise variant, no table                                   │ ~80      │
  ├────────────────┼──────────────────────┼─────────────────────────────────────────────────────────────┼──────────┤
  │ Frame parser   │ new bl_protocol.c    │ subset of protocol.c states, no sub-command dispatch table  │ ~400     │
  ├────────────────┼──────────────────────┼─────────────────────────────────────────────────────────────┼──────────┤
  │ Command        │ new bl_protocol.c    │ enumerate, ping, write_chunk, verify, run, read_uid         │ ~600     │
  │ handlers       │                      │                                                             │          │
  ├────────────────┼──────────────────────┼─────────────────────────────────────────────────────────────┼──────────┤
  │ FMC ops        │ inline subset        │ erase_page, program_word, set_vecmap, jump_aprom            │ ~250     │
  ├────────────────┼──────────────────────┼─────────────────────────────────────────────────────────────┼──────────┤
  │ Misc /         │ —                    │ —                                                           │ ~200     │
  │ alignment      │                      │                                                             │          │
  ├────────────────┼──────────────────────┼─────────────────────────────────────────────────────────────┼──────────┤
  │ Total          │                      │                                                             │ ~2.4 KB  │
  └────────────────┴──────────────────────┴─────────────────────────────────────────────────────────────┴──────────┘

  The Library/StdDriver/src/fmc.o is 1.5 KB because it includes every ISP variant. The bootloader inlines only the three
   operations it actually needs.

  Linker change: add a second target in m2003.ld (or split into m2003_app.ld and m2003_bl.ld) — bootloader links at
  ORIGIN = 0x00100000, LENGTH = 0x1000, app keeps the existing layout. Bootloader uses the same RAM region but smaller
  stack (256 B).

  4. Protocol Additions

  New addressed sub-commands (in the running app, used to enter bootloader):

  ┌─────────┬──────────────────┬──────┬────────────────────────────────┐
  │ Sub-Cmd │       Name       │ Data │            Purpose             │
  ├─────────┼──────────────────┼──────┼────────────────────────────────┤
  │ 0x30    │ ENTER_BOOTLOADER │ —    │ Write magic + NVIC_SystemReset │
  └─────────┴──────────────────┴──────┴────────────────────────────────┘

  Bootloader command set (runs only in LDROM; reuses the same 0xA5 0x5A framing, same CRC, same address-byte scheme):

  ┌───────────┬──────────────────────┬───────────────────────────┬─────────────────────────────────────────────────┐
  │ Type byte │         Name         │          Payload          │                    Direction                    │
  ├───────────┼──────────────────────┼───────────────────────────┼─────────────────────────────────────────────────┤
  │ 0x01      │ ENTER_STORE_FORWARD  │ —                         │ master → ring                                   │
  ├───────────┼──────────────────────┼───────────────────────────┼─────────────────────────────────────────────────┤
  │ 0x02      │ ENTER_CUT_THROUGH    │ —                         │ master → ring                                   │
  ├───────────┼──────────────────────┼───────────────────────────┼─────────────────────────────────────────────────┤
  │ 0x03      │ SET_ADDRESS          │ [counter]                 │ master → ring (S&F, same as app)                │
  ├───────────┼──────────────────────┼───────────────────────────┼─────────────────────────────────────────────────┤
  │ 0x60      │ BOOT_PING            │ —                         │ master → ring; every device emits BOOT_PONG in  │
  │           │ (broadcast)          │                           │ turn                                            │
  ├───────────┼──────────────────────┼───────────────────────────┼─────────────────────────────────────────────────┤
  │ 0x70+addr │ BOOT_CMD             │ [subcmd] [data…]          │ master → device                                 │
  ├───────────┼──────────────────────┼───────────────────────────┼─────────────────────────────────────────────────┤
  │ 0x80+addr │ BOOT_REPLY           │ [subcmd] [result]         │ device → master                                 │
  │           │                      │ [detail…]                 │                                                 │
  └───────────┴──────────────────────┴───────────────────────────┴─────────────────────────────────────────────────┘

  BOOT_CMD sub-commands:

  ┌──────┬─────────────┬────────────────────────────────────────┬───────────────────────────────────────────────────┐
  │ Sub  │    Name     │                  Data                  │                   Reply detail                    │
  ├──────┼─────────────┼────────────────────────────────────────┼───────────────────────────────────────────────────┤
  │ 0x01 │ READ_UID    │ —                                      │ 12 bytes UID                                      │
  ├──────┼─────────────┼────────────────────────────────────────┼───────────────────────────────────────────────────┤
  │ 0x02 │ READ_INFO   │ —                                      │ LDROM version, APROM size, page size, persist     │
  │      │             │                                        │ pages                                             │
  ├──────┼─────────────┼────────────────────────────────────────┼───────────────────────────────────────────────────┤
  │ 0x10 │ ERASE_PAGE  │ [page_idx]                             │ result                                            │
  ├──────┼─────────────┼────────────────────────────────────────┼───────────────────────────────────────────────────┤
  │ 0x11 │ WRITE_CHUNK │ [addr_hi] [addr_lo] [data…16B]         │ result, bytes_written                             │
  ├──────┼─────────────┼────────────────────────────────────────┼───────────────────────────────────────────────────┤
  │ 0x12 │ READ_CRC    │ [start_hi] [start_lo] [len_hi]         │ CRC16 over region                                 │
  │      │             │ [len_lo]                               │                                                   │
  ├──────┼─────────────┼────────────────────────────────────────┼───────────────────────────────────────────────────┤
  │ 0x20 │ RUN_APROM   │ —                                      │ (no reply, branches)                              │
  └──────┴─────────────┴────────────────────────────────────────┴───────────────────────────────────────────────────┘

  Type bytes 0x60/0x70/0x80 are deliberately outside the app's 0x40/0x50 reply range so host code can disambiguate
  "device is in app" vs "device is in bootloader" by reply-type prefix alone.

  WRITE_CHUNK payload is fixed at 16 data bytes + 2 addr bytes + 1 subcmd = 19 bytes, comfortably under the 34-byte
  protocol limit. At 250 kbaud and ~32 B framed, that's ~1 ms wire time + ~1 ms erase+program → effective ~16 B per 2 ms
   ≈ 8 KB/s per device. Full 30 KB image per device: ~4 s.

  5. Flash Sequencing

  v1 model: per-device, sequential. Host iterates addr=0..N-1. For each device:
  1. BOOT_PING → confirm in bootloader.
  2. For each page (0x00…0x7B, skipping persist pages 0x7C/0x7D by default):
    - ERASE_PAGE [n] → wait ACK.
    - For each 16-byte chunk in page: WRITE_CHUNK → wait ACK.
  3. READ_CRC [0, image_size] → compare against host's local CRC of the binary.
  4. RUN_APROM → device branches; host moves to next addr.

  Why not broadcast in v1: a single CRC mismatch on one device requires the host to be able to single out and retry that
   device. Sequential is simpler to reason about. Broadcast becomes a v2 optimization once v1 is proven (~16× throughput
   win for the homogeneous fleet).

  UART overflow during erase: erase is ~5 ms, FIFO+RX-ring is ~280 B, master is gated by ACK → no overflow as long as
  master waits. The bootloader runs from LDROM so it can keep servicing UART IRQs during APROM erase.

  Persist pages: skipped by default. ERASE_PAGE for pages 0x7C/0x7D is rejected unless an explicit --erase-persist flag
  is set on the host tool (separate sub-command or guarded with a magic argument).

  6. Files Changed / Added

  ldrom/
    bl_startup.c        # trimmed Reset_Handler, vectors (UART1, SysTick only)
    bl_main.c           # entry decision + command loop
    bl_uart.c           # minimal UART1 driver
    bl_protocol.c       # frame parser + command dispatch
    bl_fmc.c            # erase_page, program_word, vecmap, jump
    m2003_bl.ld         # LDROM linker script
  src/
    protocol.c          # add SUBCMD_ENTER_BOOTLOADER handler
    persist.h           # expose magic constant + .noinit declaration
  Makefile              # second target: bl-build → build/m2003-bl.bin
  scripts/
    ring_bootload.py    # new host tool (analogous to ring_tool.py)
    flash-jlink.ps1     # add --ldrom variant
  protocol.md           # document bootloader command set

  crc16.c is shared verbatim between app and bootloader; the bootloader links its own copy.

  7. Host Tool (scripts/ring_bootload.py)

  CLI shape, modeled on existing ring_tool.py:

  python scripts/ring_bootload.py --port COM7 --image build/m2003-motor.bin \
         [--addr N | --all] [--keep-persist | --erase-persist] [--dry-run] [-v]

  Phases:
  1. Send ENTER_BOOTLOADER to all addressed devices (or assume already in BL).
  2. Enumerate (same store-and-forward sequence as app).
  3. For each target, run the sequential program flow above with progress bar.
  4. Report per-device result + final CRC verification.

  8. Production Provisioning

  One-time SWD flow per board (or via JLink script):
  1. Flash LDROM image.
  2. Flash APROM image.
  3. Write user-config CBS=01 (boot from LDROM), CFG enables LDROM-update only via SWD (FMC_DISABLE_LD_UPDATE in the
  running app).
  4. Mass-erase enable bit set per fleet policy.

  scripts/flash-jlink.ps1 extended with a -Provision switch that performs all four steps.

  9. Testing

  Hardware-only; there's no committed unit framework.

  T1 — LDROM stub boots, jumps to APROM. Flash a no-op LDROM (just decide + jump). Boot. APROM runs. Verified by motor
  enumeration on the ring.

  T2 — Listen window catches ENTER_BOOT. Power on while host streams ENTER_BOOT. Device enters BL, responds to
  BOOT_PING.

  T3 — Soft-reset path. Send ENTER_BOOTLOADER to running app. Device responds, resets, comes up in BL. Magic + clear
  path verified.

  T4 — Single-device flash. Erase + reflash one device's APROM with a known-different image (e.g., toggle the indicator
  LED color). Verify behavior changes after RUN_APROM.

  T5 — Persist preservation. Re-run T4. Verify saved zero/PID/strike-cal still loads — persist pages untouched.

  T6 — Mid-write power loss. Pull power during a WRITE_CHUNK burst. On power-up, device should boot into BL (LDROM
  intact), accept retry, complete the update.

  T7 — Ring sequence. All 16 devices in ring, run host tool with --all. All flash to new image.

  T8 — Forward-while-program. Confirm devices in BL still forward (cut-through) frames addressed to other devices during
   their own idle moments.

  10. Risks / Open Questions

  ┌───────────────────────────────────────┬─────────────────────────────────────────────────────────────────────────┐
  │                 Risk                  │                               Mitigation                                │
  ├───────────────────────────────────────┼─────────────────────────────────────────────────────────────────────────┤
  │ LDROM size overflows 4 KB             │ Strict CI check on arm-none-eabi-size build/m2003-bl.elf; trim          │
  │                                       │ aggressively if it exceeds 3.8 KB.                                      │
  ├───────────────────────────────────────┼─────────────────────────────────────────────────────────────────────────┤
  │ .noinit RAM not actually preserved    │ Verify on bench in T3; fallback is a CRC-protected magic in a dedicated │
  │ across NVIC_SystemReset on M2003      │  LDROM page (but LDROM update is gated, so this is a bigger ask).       │
  ├───────────────────────────────────────┼─────────────────────────────────────────────────────────────────────────┤
  │ Forwarding behavior in BL when one    │ Bench-validate T8; if UART IRQ latency during erase is worse than       │
  │ device is mid-erase                   │ expected, fall back to enforced store-and-forward mode in BL.           │
  ├───────────────────────────────────────┼─────────────────────────────────────────────────────────────────────────┤
  │ Host tool race / partial enumeration  │ Listen window length is tunable; default 100 ms, increase if needed.    │
  │ on power-on                           │                                                                         │
  ├───────────────────────────────────────┼─────────────────────────────────────────────────────────────────────────┤
  │ LDUEN left enabled in field could     │ App calls FMC_DISABLE_LD_UPDATE() at boot. Verify in T1.                │
  │ allow rogue LDROM rewrite             │                                                                         │
  └───────────────────────────────────────┴─────────────────────────────────────────────────────────────────────────┘

  11. Milestones

  ┌───────┬────────────────────────────────────────────────────────────┬─────────┐
  │   #   │                         Milestone                          │ Effort  │
  ├───────┼────────────────────────────────────────────────────────────┼─────────┤
  │ M1    │ Bootloader skeleton: vectors, clock, UART, ping-pong       │ 0.5 d   │
  ├───────┼────────────────────────────────────────────────────────────┼─────────┤
  │ M2    │ FMC ops + jump-to-APROM, LDROM stub T1+T2                  │ 0.5 d   │
  ├───────┼────────────────────────────────────────────────────────────┼─────────┤
  │ M3    │ Frame parser + enumeration + READ_UID/READ_INFO            │ 1 d     │
  ├───────┼────────────────────────────────────────────────────────────┼─────────┤
  │ M4    │ ERASE_PAGE/WRITE_CHUNK/READ_CRC, T4                        │ 1 d     │
  ├───────┼────────────────────────────────────────────────────────────┼─────────┤
  │ M5    │ App-side ENTER_BOOTLOADER sub-command + magic plumbing, T3 │ 0.5 d   │
  ├───────┼────────────────────────────────────────────────────────────┼─────────┤
  │ M2    │ FMC ops + jump-to-APROM, LDROM stub T1+T2                  │ 0.5 d   │
  ├───────┼────────────────────────────────────────────────────────────┼─────────┤
  │ M3    │ Frame parser + enumeration + READ_UID/READ_INFO            │ 1 d     │
  ├───────┼────────────────────────────────────────────────────────────┼─────────┤
  │ M4    │ ERASE_PAGE/WRITE_CHUNK/READ_CRC, T4                        │ 1 d     │
  ├───────┼────────────────────────────────────────────────────────────┼─────────┤
  │ M5    │ App-side ENTER_BOOTLOADER sub-command + magic plumbing, T3 │ 0.5 d   │
  ├───────┼────────────────────────────────────────────────────────────┼─────────┤
  │ M6    │ ring_bootload.py host tool with progress, T4 end-to-end    │ 1 d     │
  ├───────┼────────────────────────────────────────────────────────────┼─────────┤
  │ M7    │ Multi-device ring testing T5–T8                            │ 1 d     │
  ├───────┼────────────────────────────────────────────────────────────┼─────────┤
  │ M8    │ Production provisioning script + docs                      │ 0.5 d   │
  ├───────┼────────────────────────────────────────────────────────────┼─────────┤
  │ Total │                                                            │ ~6 days │
  └───────┴────────────────────────────────────────────────────────────┴─────────┘

  Broadcast-update mode and signed images deferred to v2.

  ---
  Want me to:
  - (a) start on M1 (bootloader skeleton + size-validated build target), so we get an early read on the real LDROM image
   size before committing to the protocol details, or
  - (b) start on M5 (app-side ENTER_BOOTLOADER + magic plumbing), which is small and lets us test the soft-reset path
  end-to-end before the bootloader exists, or
  - (c) iterate on this plan first — anything in §2 / §4 / §5 you want changed?

✻ Sautéed for 3m 16s

※ recap: Goal is a ring-protocol LDROM bootloader for field firmware updates on the M2003 fleet. The formal plan is
  drafted and waiting on your call between starting the bootloader skeleton (M1), the app-side enter-bootloader command
  (M5), or iterating on the plan first. (disable recaps in /config)