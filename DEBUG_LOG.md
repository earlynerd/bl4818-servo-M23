# Debug Log

## 2026-08-01 — Standard flash command omitted the new LDROM loader

- **Observation:** An actuator was flashed with `scripts/flash-jlink.ps1`, but that familiar command installed only APROM and its manifest rather than the newly required LDROM loader.
- **Root cause:** Pre-fix `scripts/flash-jlink.ps1:41` selected the app-only build, and pre-fix `Makefile:95` defined `build-jlink` without the LDROM image.
- **Fix:** Collapsed J-Link support to one standard build/flash path that programs and byte-verifies APROM, manifest, and LDROM while leaving CONFIG unchanged.
- **Class:** misleading-tool-entrypoint
- **Recently-touched?** yes
- **Time to fix:** about 20 minutes

## 2026-08-01 — Generic J-Link provisioning is slow

- **Observation:** Programming the complete APROM, manifest, and LDROM image through J-Link Commander takes substantially longer than flashing the same device with a CMSIS-DAP probe.
- **Root cause:** `scripts/make_provision_jlink.py:91` expands every non-erased 32-bit word into five debugger commands because the installed J-Link device database has no native M2003 flash algorithm. The complete image produces about 39,500 command-file lines; byte-for-byte PowerShell verification contributes only about 50 ms.
- **Resolution:** Leave the reliable single J-Link path unchanged for now. Meaningful acceleration would require either adopting the OpenOCD NuMicro flash driver with CMSIS-DAP or adding a RAM-resident page-programming stub for generic J-Link.
- **Class:** debugger-flash-command-overhead
- **Recently-touched?** yes
- **Time to diagnose:** about 20 minutes

## 2026-08-02 — Software boot handoff required destination VECMAP

- **Observation:** Early LDROM builds visibly entered the loader but did not reliably reach APROM, and application-requested loader entry could not be distinguished from a watchdog return.
- **Root cause:** Pre-fix `ldrom/main.c:73` and `src/boot_control.c:55` changed only the runtime boot-source bit before system reset; the M2003 vendor sequence also maps the destination vector page. Application entry additionally relied on ISP remaining enabled even though `src/persist.c:194` can close it.
- **Fix:** Both handoffs now enable ISP as needed, issue and verify destination VECMAP, verify the matching boot source, and system-reset; loader protocol 2 exposes retained reset/FMC state.
- **Class:** incomplete-boot-handoff-sequence
- **Recently-touched?** yes
- **Time to fix:** one extended research-and-bench session

## 2026-08-02 — LDROM alias caused false J-Link APROM verification failure

- **Observation:** Standard flashing repeatedly reported `APROM verify failed at +0x4: expected 0x19, got 0xC5` even though programming appeared to complete.
- **Root cause:** Pre-fix `scripts/flash-jlink.ps1:115` compared APROM against a debugger read beginning at CPU address zero, which aliases the active LDROM vector page in LDROM-first mode.
- **Fix:** The standard wrapper now reconstructs APROM `0x0000..0x01FF` through physical FMC reads and joins it with direct reads above the alias window before byte comparison.
- **Class:** debugger-address-alias
- **Recently-touched?** yes
- **Time to fix:** about 30 minutes

## 2026-08-02 — Post-reset enumeration consumed a delayed mode echo

- **Observation:** `RUN_APROM` succeeded, but the updater's immediate application check failed with `unexpected enumerate reply: 01`; a fresh connection then enumerated and queried the healthy application normally.
- **Root cause:** Pre-fix `scripts/ring_bus.py:719` read exactly one frame after sending `SET_ADDRESS`, while the preceding `ENTER_STORE_FORWARD` cut-through echo could arrive after the fixed 10 ms flush interval.
- **Fix:** Enumeration now waits for each mode-command echo and skips stale control or seed echoes until a valid nonzero address count returns.
- **Class:** fixed-delay-protocol-race
- **Recently-touched?** no
- **Time to fix:** about 10 minutes

## 2026-08-02 — Manifest-last recovery passed staged power-cycle testing

- **Observation:** The restrained actuator was deliberately power-cycled with (1) an erased manifest and intact APROM, (2) page 0 erased with only its first 32 bytes restored, and (3) a fully written and CRC-verified APROM whose manifest commit was withheld.
- **Result:** Every cold boot remained in a responsive, enumerable LDROM loader with no committed image and cleared volatile update state. The partial image returned `CRC_MISMATCH`; `RUN_APROM` with an invalid manifest returned `BAD_STATE`.
- **Recovery:** A normal version-2 update rewrote 54 pages, verified CRC `0x349FA335`, committed the manifest, re-enumerated the application, and preserved strike settings `1024 / 300 / 100`. The same image also passed the one-device `--all` command path.
- **Class:** recovery-validation
- **Recently-touched?** yes
- **Time to validate:** about 15 minutes of staged bench testing
