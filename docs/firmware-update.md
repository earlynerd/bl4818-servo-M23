# Ring firmware update

## Status

This document is the operating, recovery, and validation record for the stable
Gen1 M2003 UART-ring updater. The loader and protocol-3 host path have passed
single-actuator boot/interruption recovery tests, one-node broadcast testing,
and a complete 14-actuator handpan-ring update. The Gen1 LDROM loader is now
maintenance-only: its protocol and flash layout are frozen unless a field defect
requires correction, and no additional loader features are planned.

The updater is intentionally a recovery system, not just an APROM write
command. A permanent loader runs from the M2003's 4 KB LDROM, while the motor
application and its update metadata live in APROM. The loader is never
rewritten over the ring in version 1.

## Goals and limits

- Update one or more Gen1 actuators over the existing 250 kbaud,
  one-direction UART ring after a one-time SWD provisioning pass.
- Recover through the same ring after reset or power loss at any point in an
  APROM update.
- Preserve the existing two-page settings journal by construction.
- Keep byte-level cut-through forwarding in both the application and loader.
- Use CRCs for transmission and image integrity. Version 1 does not provide
  authentication, signing, encryption, compression, delta updates, or A/B
  images.
- Keep verification, repair, manifest commit, and application restart
  device-specific. Protocol 3 may broadcast one shared image data phase, with
  the original sequential updater retained as the conservative fallback.

This work applies to the Gen1 UART ring. It does not change the tentative Rev2
RS485 design.

## Fixed memory map

| Range | Size | Owner | Update rule |
| --- | ---: | --- | --- |
| `0x00000000..0x000079FF` | 31,232 B | APROM application | Erased and programmed by the loader |
| `0x00007A00..0x00007BFF` | 512 B | committed-image manifest | Erased first; programmed last |
| `0x00007C00..0x00007FFF` | 1,024 B | persistent settings journal | Never accepted by updater commands |
| `0x00100000..0x00100FFF` | 4,096 B | permanent LDROM loader | SWD provisioning only in version 1 |
| `0x20000FF0..0x20000FFF` | 16 B | soft-reset mailbox | Excluded from both images' normal RAM |

`include/firmware_image.h` is the source of truth for the flash constants and
manifest layout. Linker scripts must fail rather than allow an image to cross
one of these boundaries.

The application currently fits below `0x7A00`. Reserving the manifest page
reduces its former maximum by 512 bytes but leaves several kilobytes of
headroom. The application `.bin` contains only application bytes; packaging
tools create and program the manifest separately.

## Commit record and integrity

The first 32 bytes of the manifest page hold eight naturally aligned words:

1. magic
2. manifest format version and header size
3. exact application byte length
4. CRC-32/ISO-HDLC of application bytes `[0, image_size)`
5. user-supplied image version
6. flags
7. reserved, written as zero
8. CRC-32/ISO-HDLC of the preceding 28 manifest bytes

Unused bytes in the manifest page remain erased (`0xFF`). The standard CRC-32
parameters match Python `zlib.crc32`: reflected polynomial `0xEDB88320`,
initial value `0xFFFFFFFF`, and final XOR `0xFFFFFFFF`.

The loader treats the application as bootable only when all of the following
hold:

- both manifest CRC and magic/version/header fields are valid;
- `image_size` is word-programmable and within the application region;
- the initial stack pointer is in the M2003 SRAM window and word-aligned;
- the reset vector is a Thumb address inside the application region; and
- the calculated application CRC matches the manifest.

These checks protect the boot boundary and flash ranges. They are not intended
to impose tight validation on motor tuning parameters, which remain the
application's responsibility.

## Boot and recovery contract

On every reset, LDROM performs the following sequence:

1. Mask and stop every PWM phase, then drive the retrofit's shared PB1
   CSn/status LED high and solid on before enabling other peripherals.
2. Start HIRC at 24 MHz and UART1 at 250 kbaud with cut-through forwarding.
3. Examine the fixed RAM mailbox. A valid one-shot request contains the former
   application ring address; the loader clears the request before continuing.
4. Validate the committed APROM image.
5. Stay in the loader when explicitly requested or when APROM is not valid.
6. Otherwise service a short boot-intercept window, issue and verify an APROM
   vector remap, select and verify APROM as the software boot source, and
   perform a system reset.

APROM therefore always starts through its real reset path rather than through
a direct call from LDROM. The transition follows the M2003 vendor ISP examples:
map the destination vectors, select the matching runtime boot source, then
system-reset. As the first application startup action, APROM selects LDROM as
the destination of the next reset without changing the active APROM vector map.
Watchdog, fault, and requested resets consequently return through the recovery
loader, while the one loader-to-application reset retains the temporary APROM
selection as defined by the M2003 reset contract.

The boot-intercept window is a last-resort recovery path for a valid but
non-responsive application. A host may repeatedly transmit the loader-enter
frame while power is applied. The current 1 second window deliberately keeps
the solid LDROM indicator visible during bench diagnosis and leaves ample time
for a recovery host to transmit `STAY_IN_BOOTLOADER`.

The normal application enters the loader through addressed subcommand
`ENTER_BOOTLOADER`. It first performs a coordinated stop, returns a compact
ACK, drains the UART transmitter, writes the mailbox, and requests a system
reset. Before selecting LDROM it enables the clock-gated Flash ISP controller,
issues and verifies an LDROM vector remap, and verifies the protected
boot-select bit took effect. Reset is deliberately deferred until after the ACK
has left the device. If vector remapping fails, the already-running application
watchdog forces a hardware reset that reloads the provisioned LDROM-first
configuration rather than attempting a direct branch.

Loader protocol 2 and 3 append raw entry snapshots of `SYS.RSTSTS`, `FMC.ISPCTL`,
and `FMC.ISPSTS` to `GET_INFO`. These values are captured before the loader
modifies the FMC controller and are the primary handoff diagnostic because SWD
and UART cannot be attached at the same time on Gen1.

The current bench retrofit lights the onboard LED when PB1 is high. LDROM holds
PB1 high continuously, while the normal application uses its usual heartbeat
and state patterns. This makes loader residence visible without UART or SWD.
The loader does not use the encoder, so holding the shared CSn pin is safe.

## Power-loss-safe update sequence

For each target, the host and loader use this state machine:

1. Read loader identity, geometry, and current manifest state.
2. `BEGIN_IMAGE(image_size, image_crc32, image_version)` validates the proposed
   range and erases the manifest page. From this point APROM cannot boot.
3. For each required application page, erase the page, send its data in small
   word-aligned chunks, and retry failed chunks without advancing.
4. Ask the loader to calculate the CRC of the exact proposed image length.
5. `COMMIT_IMAGE` succeeds only when that CRC matches the values accepted by
   `BEGIN_IMAGE`; it writes and verifies the manifest as the final flash
   operation.
6. Ask the loader to select and system-reset into APROM, then re-enumerate and
   query the application.

If reset or power loss occurs before step 5 completes, the erased or invalid
manifest keeps the device in LDROM. Repeating `BEGIN_IMAGE` and the full image
write is always safe. Resume-from-page is deferred until the simple full retry
has been proven reliable.

Protocol 3 optionally performs steps 2 and 3 once for all assigned loaders.
The command type encodes the farthest address, which alone returns the normal
reply after executing each command and therefore paces the next frame. The host
then performs step 4 individually, repeats the addressed data phase for any
CRC failure, and performs steps 5 and 6 individually. A stopped bulk transfer
can leave every participating actuator resident in LDROM, but rerunning either
bulk or sequential update is recovery-safe.

Every erase and write command rejects addresses at or above
`FIRMWARE_APP_LIMIT`; there is no `--erase-persist` escape hatch. Settings can
continue to be managed by the application's existing maintenance commands.

## Ring behavior

Both loader commands and replies travel in the existing physical direction.
The UART RX interrupt forwards each byte as soon as it arrives and also places
a copy in the local parser buffer. A loader therefore does not wait for a full
frame before forwarding it.

An application-requested loader receives its prior 4-bit ring address through
the RAM mailbox, so the host can continue addressing that device without a
second enumeration. A loader entered after power loss starts unassigned and
participates in the existing store-and-forward enumeration sequence. Running
applications and loaders can be enumerated in the same physical chain; their
normal command/reply type ranges remain distinct.

## One-time SWD provisioning

Each existing actuator requires one final manual connection:

1. Read and record all user configuration words before modifying them.
2. Program and verify LDROM.
3. Program and verify an APROM application plus a matching manifest.
4. Configure the supported LDROM-first IAP boot mode.
5. Power-cycle and prove that LDROM validates and starts APROM.
6. Prove addressed entry, recovery from an invalid manifest, and settings
   preservation. This validation was completed before fleet provisioning and
   must be repeated before deploying any corrective loader build.

SWD and the ring UART use the same two MCU pins on Gen1 and cannot be observed
simultaneously. Disconnect the UART adapter before attaching J-Link. After the
standard flash verifies, power-cycle the actuator, disconnect J-Link, and
reconnect the UART adapter for the ring test.

The first restrained actuator reported `CONFIG0=0xFFFFFFFF`,
`CONFIG1=0xFFFFFFFF`, `CONFIG2=0xFFFFFF5A`, and `ISPSTS=0x00000014` before the
change. CONFIG0.CBS was therefore 1, which the M2003 TRM defines as direct
APROM boot. Bench testing also showed that setting the runtime BS bit before a
system reset did not transfer this actuator to LDROM. The supported permanent
recovery configuration clears only CONFIG0 bit 7 (`0xFFFFFFFF` becomes
`0xFFFFFF7F` on this actuator), making LDROM the reset and cold-power entry
point while preserving every other configuration option.

The standard flash wrapper captures and validates all three words before any
write. It refuses automatic configuration changes if CONFIG0.LOCK is cleared
or CONFIG2 is not the documented unlocked `0xFFFFFF5A`. After the firmware
stack verifies, it follows the vendor CONFIG erase procedure, verifies the
erased `FFFFFFFF/FFFFFFFF/FFFFFF5A` state, restores the captured CONFIG1, then
programs CONFIG0 with CBS cleared as the final committed word and verifies all
three values again.

LDROM update permission is disabled during normal operation. Version 1 exposes
no ring command capable of erasing or programming LDROM or configuration
words.

## Verification record

### Off-hardware

- `make`: application builds below `FIRMWARE_APP_LIMIT`.
- `make ldrom`: loader builds at `0x00100000` and hard-fails above 4,096 bytes.
- Host tests cover frame CRC, manifest CRC, bounds, duplicate chunks, retry,
  and simulated reset before/during/after commit.
- Host tests cover CONFIG log parsing, security-state rejection,
  preserve-and-clear planning, erase/program separation, commit order, and
  exact erased/final verification.

Recorded on 2026-08-02 from a clean build:

- packaged application: 27,480 / 31,232 bytes (3,752 bytes free), CRC-32
  `0x349FA335` for image version zero;
- permanent loader: 4,092 / 4,096 bytes (4 bytes free), with the linker
  enforcing the limit;
- manifest: exactly 512 bytes;
- host tests cover the full firmware-update and playback host paths; Python
  and PowerShell entry points parse cleanly; and
- complete firmware, read-only CONFIG capture, and preservation-aware CONFIG
  transaction files generate without contacting hardware.

The 4-byte LDROM margin is intentional and explicit. Protocol 3 recovers 64
bytes by using a compact tableless CRC-32 routine, then spends almost all of
that space on tail-acknowledged bulk transfer while retaining loader-entry
diagnostics and every flash-safety check. It is not reserved for more features;
any future field-defect correction must preserve these protections and recover
measured code space if needed.

Generate the current programming and configuration-capture command files
without connecting to hardware:

```powershell
powershell -File scripts/build-jlink.ps1
```

This produces `build/m2003-motor.jlink`, which programs APROM,
its manifest, and LDROM, and `build/m2003-config-read.jlink`, which only reads
CONFIG0..2 plus ISPCTL/ISPSTS. Generation alone writes neither firmware nor
configuration. The preservation-aware CONFIG erase/program files are created
only after the standard flash command has captured and validated the attached
actuator.

The complete one-time provisioning pass is executed with the standard command:

```powershell
powershell -File scripts/flash-jlink.ps1
```

The wrapper verifies the application, manifest, and LDROM byte-for-byte in a
separate read-only pass, then selects and verifies LDROM-first boot while
preserving the captured CONFIG values. Because LDROM-first mode aliases CPU
address zero to the LDROM vector page, verification reads the first 512 APROM
bytes through physical FMC commands and uses direct debugger reads only above
that alias window. The generated SWD programming path invalidates the old
manifest before changing APROM and programs the new manifest magic word last.
Power-cycle the current bench actuator after J-Link exits before moving the
shared pins back to the UART adapter.

There is intentionally no separate app-only or provisioning command path.

The standalone updater remains the supported recovery and command-line path:

```powershell
# Validate an image without opening a serial port.
py scripts/ring_bootload.py build/m2003-motor.bin --addr 0 --dry-run

# Read loader identity and retained entry state without changing flash.
py scripts/ring_bootload.py build/m2003-motor.bin -p COM18 --addr 0 `
    --no-enter --info-only

# Update one addressed actuator.
py scripts/ring_bootload.py build/m2003-motor.bin -p COM7 --addr 0 --image-version 1

# Recovery: start this, then apply power during the five-second hold stream.
py scripts/ring_bootload.py build/m2003-motor.bin -p COM7 --addr 0 `
    --recover-seconds 5 --no-enter --image-version 1
```

`--all` replaces `--addr` for a sequential full-ring update. The standalone
updater owns the serial port for the entire operation; stop the MIDI server
before using that CLI. A normal update enumerates the ring before handoff.
Recovery also enumerates after the
boot-hold stream, so the requested address is the actuator's physical ring
position in that enumeration. After every `RUN_APROM`, the tool re-enumerates
the mixed ring and requires the updated application to answer `QUERY_STATUS`
before advancing.

Use the same prepared image and image-version value for every enumerated
actuator with:

```powershell
py scripts/ring_bootload.py build/m2003-motor.bin -p COM7 --all `
    --image-version 2
```

The command first enumerates the complete ring and confirms that every target
can enter and answer from LDROM. It then programs one physical ring address at
a time, CRC-verifies and commits that image, returns the device to APROM, and
requires the application status reply before advancing. If a run is
interrupted, rerun the same command and image version; page writes are
idempotent and an uncommitted target remains in its loader. Image versions are
operator-supplied labels in version 1, not an anti-rollback policy.

After every actuator has protocol-3 LDROM, the faster shared-data path is:

```powershell
py scripts/ring_bootload.py build/m2003-motor.bin -p COM7 --broadcast-all `
    --image-version 3
```

This broadcasts manifest invalidation, page erases, and chunks once, paced by
the farthest loader's single reply. It then CRC-verifies every device, repairs
any mismatch with the addressed transfer, commits every verified manifest
individually, and returns each actuator to APROM individually. `--all` remains
the sequential fallback and works with protocol 1, 2, or 3 loaders;
`--broadcast-all` refuses any loader older than protocol 3.

### MIDI server / browser updater

The browser player exposes the same protocol-3 path without opening a second
serial connection. **Build** runs the fixed `make` command in the server's
repository checkout, validates `build/m2003-motor.bin`, and freezes the prepared
bytes plus image size, CRC-32, version, Git commit, and dirty status in server
memory. **Update ring** requires a second explicit confirmation tied to the
displayed CRC, then uses the MIDI server's already-open `RingClientV2` instance.

The update transaction sets an exclusive maintenance flag before stopping the
playback worker. New strike, home, enumerate, status, tuning, and active bus
probe requests return HTTP 409 until every application has restarted or the
operation fails. The worker also holds the bridge ring lock for the entire
loader transaction. Build output and structured update progress remain
available through `GET /api/firmware`; the browser polls that endpoint because
normal ring status is intentionally unavailable during maintenance.

The first GUI version has no Git synchronization and no uploaded ELF/binary
path. Operators update the checkout outside the GUI, inspect the displayed
commit/dirty marker, build, then flash the frozen `.bin`. After success all
actuators are intentionally disabled/unhomed and must be homed again.

### Single restrained actuator validation

- Entering LDROM changes the normal application LED pattern to solid on.
- LDROM-first reset verifies the APROM vector map and boot selection before
  system-resetting into a valid application; APROM re-arms LDROM before normal
  initialization and leaves the bridge disabled.
- Loader `GET_INFO` protocol 2/3 reports the retained reset source and entry FMC
  state so the observed handoff can be attributed without simultaneous SWD.
- Addressed entry ACK is observed before reset; address survives the soft
  reset mailbox path.
- Boot interception can hold a valid image in the loader.
- A known image programs, verifies, commits, runs, and re-enumerates.
- Saved zero, PID, and strike settings survive the update.
- Cold-power recovery is exercised after manifest invalidation, after an APROM
  page erase plus partial chunk programming, and after full-image CRC
  verification with the commit deliberately withheld. Host tests separately
  prove that every partial manifest prefix remains invalid until the final
  magic word is written. These observable boundaries cover the recovery
  invariant without depending on manually timing a sub-millisecond flash-word
  operation.

Recorded restrained-device evidence:

| Evidence | Result |
| --- | --- |
| Target label / board revision | single restrained actuator; board revision not recorded |
| CONFIG0 / CONFIG1 / CONFIG2 before change | `FFFFFFFF / FFFFFFFF / FFFFFF5A` |
| ISPCTL / ISPSTS before change | `00000001 / 00000014` |
| Bench-approved CONFIG/CBS value and TRM reference | `FFFFFF7F / FFFFFFFF / FFFFFF5A`; clear CONFIG0[7]; TRM 6.4.6-6.4.7 and CONFIG0/FMC tables |
| Valid-image cold boot | **PASS 2026-08-02:** solid LDROM LED for one second, then normal APROM application |
| Addressed soft-reset handoff and ACK | **PASS 2026-08-02:** protocol-2 loader replied at inherited address 0; entry `RSTSTS=0000003B` has SYSRF set and WDTRF clear, `ISPCTL=0000000A` selects LDROM, and `ISPSTS=00100010` reports LDROM VECMAP |
| Full LDROM self-program | **PASS 2026-08-02:** 54 APROM pages erased/written over COM18 in about 17 seconds; CRC verified, manifest committed, and application re-enumerated and answered status |
| Protocol-3 broadcast self-program | **PASS 2026-08-02:** loader v3 was confirmed after J-Link provisioning; `--broadcast-all --image-version 3` sent all 54 pages through tail address 0, individually verified CRC `349FA335`, committed, and re-enumerated the application in about 19 seconds on the one-node ring |
| Invalid-manifest cold recovery | **PASS 2026-08-02:** `BEGIN_IMAGE` erased only the manifest and changed loader flags from committed `01` to update-active `02`; `RUN_APROM` returned `BAD_STATE`; after power removal the actuator enumerated in LDROM with flags `00` and an erased manifest. |
| Interrupted-update recovery points | **PASS 2026-08-02:** page 0 was erased and only its first 32 bytes rewritten; verification returned `CRC_MISMATCH`, and a cold boot returned to responsive LDROM. All 54 pages were then written and CRC-verified with commit deliberately withheld; another cold boot again returned to responsive LDROM. The manifest-prefix unit test proves commit remains invalid until magic is written last. |
| Settings journal preserved | **PASS 2026-08-02:** before/after `home_offset=1024`, `coast_distance=300`, and `homing_duty=100` matched exactly |
| UART log / test command / image version | UID `00200011046C70F100001DF8`; exercised addressed version 2, sequential `--all --image-version 2`, and protocol-3 `--broadcast-all --image-version 3` on the one-node ring; committed image valid, 27,480 bytes, CRC-32 `349FA335`, version 3; final application `IDLE`, `fault=NONE`, settings `1024 / 300 / 100` |

### Ring

- **PASS 2026-08-03 — complete 14-actuator handpan ring:** the instrument-server
  GUI built the application and completed the protocol-3 ring update without an
  update error. All actuators returned to application operation and the full
  instrument resumed playback after homing.
- This fleet run exercised the shared data phase through the physical tail and
  the individual verify, repair/commit, restart, and re-enumeration sequence
  across nearest, middle, and farthest ring positions.
- The earlier manifest-invalidation, partial-page, uncommitted-full-image, and
  cold-power tests remain the recovery evidence for interrupted transfers.
- A loader continues to forward ordinary application traffic byte-for-byte
  while idle; protocol-3 bulk transfer remains paced by only the farthest
  loader reply, with every device verified and committed individually.

## Release status

- [x] Architecture, flash boundaries, manifest contract, and validation plan
      documented.
- [x] Separate, size-enforced LDROM build and safe startup.
- [x] Loader framing, enumeration, identity, and boot decision.
- [x] APROM erase/write/verify/commit commands.
- [x] Addressed application handoff.
- [x] Host updater, retry/commit-loss handling, and interruption tests.
- [x] APROM/manifest/LDROM provisioning generator and read-only configuration capture.
- [x] Preservation-aware LDROM-first CONFIG transaction added to the standard provisioning path.
- [x] Single-device hardware validation.
- [x] Protocol-3 bulk transfer implementation, off-hardware tests, and one-node
      bench validation.
- [x] Full 14-actuator ring validation through the instrument-server GUI.

The Gen1 loader/update implementation is complete and maintenance-only. Its
4-byte LDROM margin is not a feature budget; future functionality belongs in a
later hardware generation. Gen1 changes should be limited to demonstrated
correctness or recovery defects.
