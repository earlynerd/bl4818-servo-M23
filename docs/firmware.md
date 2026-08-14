# Firmware guide

## Target

The firmware targets the Nuvoton M2003 Cortex-M23 on modified BL4818 controller
hardware. It runs without an RTOS from the internal 24 MHz oscillator. The
firmware uses a 20 kHz PWM/ADC path, a 6 kHz top-level control tick, nested
current/velocity/position loops, hall commutation, and an MT6701 14-bit absolute
encoder. Authoritative pin assignments and control constants live in
`include/m2003_config.h`.

## Toolchain

The firmware build supports Windows and POSIX hosts, including Raspbian, and
requires:

- GNU Make
- Arm GNU Toolchain (`arm-none-eabi-gcc`, `arm-none-eabi-objcopy`, and
  `arm-none-eabi-size` in `PATH`)
- Python 3 for generated packaging/provisioning files. The Makefile uses `py`
  on Windows and `python3` on POSIX; override `PYTHON=...` if needed.

PowerShell and SEGGER J-Link Software are additionally required for the Windows
SWD provisioning workflow. They are not required for an ordinary `make` build
on the instrument server.

Build the binary and ELF image:

```console
make
```

Build both the application and the separately linked 4 KB LDROM recovery
loader:

```console
make images
```

The additional artifacts are `build/m2003-ldrom.elf` and
`build/m2003-ldrom.bin`. The LDROM linker script hard-fails if the loader grows
beyond 4,096 bytes.

Build and generate the standard complete J-Link payload:

```powershell
powershell -File scripts/build-jlink.ps1
```

Generation does not connect to hardware. It creates
`build/m2003-motor.jlink` and the read-only
`build/m2003-config-read.jlink`. The programming file invalidates the old
manifest first, programs APROM and LDROM, writes the matching manifest magic
last, and saves all three readbacks.

Flash and verify:

```powershell
powershell -File scripts/flash-jlink.ps1
```

Use `-SkipBuild` only when the current contents of `build/` are known to match
the source. The standard flash programs the complete stack, then performs a
separate physical read-only verification of the application, manifest, and
LDROM. The APROM vector page is read through FMC because LDROM-first mode
aliases CPU address zero. The wrapper also reads CONFIG0..2, preserves every
existing option, and clears only CONFIG0.CBS to select LDROM-first boot; locked
or unexpected security configurations are rejected. There is no separate
app-only or provisioning command path. `build/m2003-config-read.jlink` remains
available for an explicit read-only capture.

The application linker layout is defined in `m2003.ld`; it reserves one
512-byte committed-image manifest page immediately before the existing
two-page settings journal. See [Ring firmware update](firmware-update.md) for
the stable LDROM loader, recovery contract, and completed hardware validation
record.
LDROM starts APROM only by verifying the APROM vector map and boot selection,
then requesting a system reset. APROM immediately re-arms LDROM for every later
watchdog, fault, or requested reset without changing its active vector map.
Debugger/OpenOCD support files are `jlink.cfg` and `numicro_M23.cfg`.

## Runtime architecture

Initialization is deliberately safety-first: brownout reset, clock, PWM with
masked phases, ADC, sensors, protocol, control modules, encoder polarity
detection, zero-current calibration, and finally the watchdog. The main loop
polls the serial protocol; SysTick runs the time-critical control scheduling.

| Layer | Nominal rate | Responsibility |
| --- | ---: | --- |
| PWM/ADC | 20 kHz | Phase output and current sampling |
| Current control | 6 kHz | Fast torque regulation and protection |
| Encoder/velocity | 1.5 kHz | Position sampling and velocity control |
| Position | 1 kHz | Outer position loop |
| Strike state machine | 500 Hz | Homing, acceleration, coast, rebound, and muting |
| Protocol timeout aging | 500 Hz | Incomplete-frame recovery |
| Indicator | 100 Hz | Status patterns |

The watchdog is only refreshed when both the main loop and SysTick continue to
run. Brownout, hard-fault, and NMI paths mask the bridge outputs before reset.

Homing and strike sequences also carry local mechanical bounds in the 500 Hz
state machine: cumulative encoder travel may not exceed one revolution, and
homing additionally may not exceed 5 seconds. A limit disables PWM, latches a
dedicated fault, and requires fault-clear plus re-home. Re-homing by 1,024
counts (22.5 degrees) or more from the previous known absolute-encoder
calibration raises an advisory status warning. Use `SAVE_SETTINGS` after a
known-good home to make that comparison reference survive reboot; firmware
safely pauses and resumes the idle position hold around the flash write.

## First-device validation

Before connecting a full ring:

1. Power the logic and motor stage from a current-limited supply using the
   verified voltage and polarity for the hardware revision.
2. Flash one board with the motor mechanically unloaded or safely restrained.
3. Connect the serial interface and enumerate it:

   ```powershell
   py scripts/ring_tool.py -p COM7 enumerate
   ```

4. Query status and confirm there is no fault:

   ```powershell
   py scripts/ring_tool.py -p COM7 status 0
   py scripts/ring_tool.py -p COM7 timing-status 0
   ```

5. Confirm encoder motion and motor direction before issuing a home or strike.
6. Home and test at low strike current, increasing only after the mechanism and
   current readings are known to be correct.

Exact supply, connector, and first-motion limits must be filled in for the
released hardware revision before this can be considered an independent
replication procedure.

## Settings and compatibility

Calibration and strike parameters can be stored in on-chip flash. The encoder
CS polarity is auto-detected from the MT6701 frame CRC and persisted. Protocol
changes must be kept synchronized across `src/protocol.c`, `include/protocol.h`,
`scripts/ring_bus.py`, and `protocol.md`.

The normal operator path for reading, changing, and saving motor/strike tuning
is the browser player's per-actuator configuration panel. Its target selector
can apply every ordinary tuning value to the selected actuator or the complete
enumerated fleet. See [parameters.md](parameters.md) for the full catalog,
including persistence, direction/polarity semantics, and compile-time safety
constants.
