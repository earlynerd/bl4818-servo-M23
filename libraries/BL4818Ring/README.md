# BL4818Ring

Arduino `Stream` client for the existing BL4818 application ring protocol.
No motor firmware changes are required. Arduino-Pico (Earle Philhower core)
supports Pico 2; the example uses its hardware `Serial1` UART.

## Install and connect

Copy this `BL4818Ring` folder into your Arduino sketchbook's `libraries`
directory, select **Raspberry Pi Pico 2** in the Arduino-Pico board package,
and open **File > Examples > BL4818Ring > PicoTurntable**.

The example uses GP0 TX and GP1 RX at 250000 baud, 8N1. Configure UART pins
and receive FIFO before `begin()`, as described in the
[Arduino-Pico serial documentation](https://arduino-pico.readthedocs.io/en/stable/serial.html).
Connect Pico TX through the appropriate ring interface to the first motor RX;
return the last motor TX through the interface to Pico RX. Use a common signal
ground. Even one motor needs the return path. The Pico must be the only master.

The ring connector is not automatically a 3.3 V GPIO connector: verify the
actual board/interface levels and pinout, and use level translation as needed.
Do not connect motor supply or a 5 V serial output to Pico GPIO.
Connect a pot between Pico 3.3 V and GND, with its wiper on GP26 (ADC0).
A wiper-to-ground resistor can bias a disconnected wiper toward stop; choose
its value with the pot's loading in mind. This example does not diagnose all
pot wiring failures.

## API

```cpp
#include <BL4818Ring.h>
BL4818Ring ring(Serial1, 100); // reply timeout in milliseconds
// In setup(), after configuring and beginning Serial1:
// uint8_t count;
// if (ring.enumerate(count) && count == 1 && ring.stop(0)) { ... }
// ring.set_velocity(0, 40);  // signed motor RPM; selects velocity mode and starts
// ring.stop(0);             // disables drive
```

- `enumerate(count)` assigns physical ring-order addresses 0..15 and waits
  for both forwarding-mode transitions. Count is zero on failure. Enumerate
  only while the installation is stopped; it changes addresses, not motion.
- `stop`, `clear_fault`, `set_mode`, `set_velocity`, `set_duty`,
  `set_torque_limit`, and `set_position` wait for a matching compact ACK.
- `query_status(address, status)` decodes the current 17-byte status layout.
  Output is unchanged on failure. RPM is motor RPM and position is encoder
  counts (16384/revolution). `set_torque_limit` limits current; it does not
  command torque mode. `set_velocity(0)` can actively hold zero speed;
  `stop()` disables drive.
- `command(address, subcommand, data, length)` provides ACK-based access to
  other non-query application commands using big-endian data (up to 6 bytes).
  It does not implement firmware updates, raw queries, or timed ACKs.
- A false return has `error()` of InvalidArgument, WriteFailed, Timeout, or
  Rejected. `last_ack()` carries device result/detail on success or rejection.
  ACK means accepted, not proof of motion or completion.

Calls are synchronous, use fixed buffers, and must run from one task/core.
The timeout bounds reply waiting, including unwanted traffic; `Stream::write`
itself follows the underlying driver's blocking policy. Echoes, wrong-address
replies and wrong-subcommand ACKs are ignored. CRC/length failures resynchronize
one byte at a time. Input already queued before a transaction is discarded.
There are no automatic command retries. The wire protocol has no transaction
IDs, so a very late reply to an identical command cannot be distinguished from
a new reply. After a timeout, stop the application, allow the link to drain and
recover deliberately rather than retrying motion automatically.

## Turntable example

The example expects exactly one motor, sends STOP, checks faults, applies a
500 mA current limit and requires the knob at zero before accepting motion.
It filters the pot, ramps motor RPM, and polls status. Set `MAX_MOTOR_RPM`,
`RPM_STEP`, and `TORQUE_LIMIT_MA` for the mechanism. The initial 100 motor RPM
cap is an example, not a qualified operating limit. Table RPM equals motor RPM
divided by the reduction ratio (100 / 20 = 5 RPM for a 20:1 reduction).

On a command/status failure it attempts STOP once and latches out until Pico
reset; it does not clear motor faults automatically. **The current motor
firmware has no host-link-loss stop timeout.** A disconnected or crashed Pico
can leave the motor running at its last command, and STOP cannot traverse a
broken link. Provide an independent way to remove drive power. Encoder-based
motor speed also does not detect belt slip at the table.

## Validation

Build the sketch with Arduino IDE or `pio run` in its example directory.
Host protocol tests live in `tests/arduino_ring` at the repository root.
Validated on 2026-09-06: host tests passed with MSVC 19.44; the Pico 2 example
built and linked with Arduino-Pico package `1.50601.0+sha.832f2c06`, platform
`1.20.0+sha.aa70b80` and ARM GCC 14.3.0 (60800 bytes flash, 9232 bytes RAM).
The existing motor application also passed a forced rebuild (`make -B`).
Bench qualification is still required: verify enumeration, direction, speed
ramp under load, zero-knob stop, motor fault handling and link interruption.
No motor is flashed or moved by installing/building this library.
