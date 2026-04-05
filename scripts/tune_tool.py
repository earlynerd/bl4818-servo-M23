#!/usr/bin/env python3
"""
tune_tool.py - Step response capture & plotting for M2003 motor PID tuning

Captures status telemetry before, during, and after a step command,
then plots velocity, angle, and current vs time.

Examples:
    # Velocity step to 500 RPM, default PID gains
    python tune_tool.py -p COM7 0 --velocity 500

    # Set PID gains, then step
    python tune_tool.py -p COM7 0 --velocity 500 --pid 256 32 0

    # Duty step (open-loop) for 1 second
    python tune_tool.py -p COM7 0 --duty 400 --duration 1

    # Save raw data to CSV (no plot)
    python tune_tool.py -p COM7 0 --velocity 500 --csv step.csv --no-plot

    # Longer capture with more pre-step baseline
    python tune_tool.py -p COM7 0 --velocity 500 --duration 5 --pre 1.0

Requires: pyserial, matplotlib (pip install pyserial matplotlib)
"""

from __future__ import annotations

import argparse
import csv
import os
import sys
import time

# Import the ring bus client from ring_tool.py in the same directory
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ring_tool import (
    RingClientV2,
    MotorStatus,
    RingError,
    RingTimeout,
    auto_detect_port,
    DEFAULT_BAUD,
)


# ── Data capture ────────────────────────────────────────────────────────────

def capture_loop(client: RingClientV2, address: int, duration: float) -> list[tuple[float, MotorStatus]]:
    """Poll status as fast as possible for `duration` seconds. Returns (timestamp, status) pairs."""
    samples: list[tuple[float, MotorStatus]] = []
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        try:
            s = client.query_status(address)
            samples.append((time.monotonic(), s))
        except RingTimeout:
            pass
    return samples


def run_step(
    client: RingClientV2,
    address: int,
    velocity: int | None,
    duty: int | None,
    pid: tuple[int, int, int] | None,
    ff_gain: int | None,
    torque_ma: int | None,
    pre_duration: float,
    step_duration: float,
) -> tuple[list[tuple[float, MotorStatus]], float]:
    """
    Execute a step response capture.

    Returns (samples, t_step) where t_step is the monotonic time of the step command.
    """
    # 1. Ensure clean state
    print(f"Stopping motor...")
    client.stop(address)
    time.sleep(0.05)

    # 2. Optionally set PID gains
    if pid is not None:
        kp, ki, kd = pid
        print(f"Setting PID: kp={kp} ki={ki} kd={kd}")
        client.set_pid(address, kp, ki, kd)

    # 3. Optionally set feedforward
    if ff_gain is not None:
        print(f"Setting feedforward: {ff_gain}")
        client.set_ff(address, ff_gain)

    # 4. Optionally set torque limit
    if torque_ma is not None:
        print(f"Setting torque limit: {torque_ma} mA")
        client.set_torque(address, torque_ma)

    all_samples: list[tuple[float, MotorStatus]] = []

    # 4. Pre-step baseline
    if pre_duration > 0:
        print(f"Capturing baseline ({pre_duration:.1f}s)...")
        all_samples.extend(capture_loop(client, address, pre_duration))

    # 5. Step command
    t_step = time.monotonic()
    if velocity is not None:
        print(f"STEP: velocity={velocity} RPM")
        s = client.set_velocity(address, velocity)
        all_samples.append((time.monotonic(), s))
    else:
        print(f"STEP: duty={duty}")
        s = client.set_duty(address, duty)
        all_samples.append((time.monotonic(), s))

    # 6. Capture during step
    print(f"Capturing response ({step_duration:.1f}s)...")
    all_samples.extend(capture_loop(client, address, step_duration))

    # 7. Stop
    print("Stopping motor...")
    s = client.stop(address)
    all_samples.append((time.monotonic(), s))

    # Brief post-step capture to see deceleration
    all_samples.extend(capture_loop(client, address, 0.3))

    rate = len(all_samples) / (pre_duration + step_duration + 0.3) if all_samples else 0
    print(f"Captured {len(all_samples)} samples ({rate:.0f} Hz effective)")

    return all_samples, t_step


# ── CSV export ──────────────────────────────────────────────────────────────

def save_csv(path: str, samples: list[tuple[float, MotorStatus]], t_step: float) -> None:
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_sec", "state", "fault", "mode", "current_ma", "hall",
                     "angle", "angle_deg", "velocity_rpm", "target"])
        for t, s in samples:
            w.writerow([
                f"{t - t_step:.4f}",
                s.state, s.fault, s.mode, s.current_ma, s.hall,
                s.angle, f"{s.angle_deg:.2f}", s.velocity, s.target,
            ])
    print(f"Saved {len(samples)} rows to {path}")


# ── Plotting ────────────────────────────────────────────────────────────────

def plot_step(
    samples: list[tuple[float, MotorStatus]],
    t_step: float,
    title: str,
) -> None:
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("ERROR: matplotlib is required for plotting. Install with: pip install matplotlib")
        return

    if not samples:
        print("No samples to plot.")
        return

    t = [s[0] - t_step for s in samples]  # time relative to step (seconds)
    vel = [s[1].velocity for s in samples]
    tgt = [s[1].target for s in samples]
    angle = [s[1].angle_deg for s in samples]
    current = [s[1].current_ma for s in samples]
    mode = samples[-1][1].mode  # use last sample's mode for label

    fig, axes = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
    fig.suptitle(title, fontsize=12)

    # Velocity
    ax = axes[0]
    ax.plot(t, vel, "b-", linewidth=1, label="measured")
    ax.plot(t, tgt, "r--", linewidth=1, alpha=0.7,
            label="target (rpm)" if mode == 1 else "target (duty)")
    ax.axvline(0, color="gray", linestyle=":", linewidth=0.8, label="step")
    ax.set_ylabel("Velocity (RPM)")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)

    # Angle
    ax = axes[1]
    ax.plot(t, angle, "g-", linewidth=1)
    ax.axvline(0, color="gray", linestyle=":", linewidth=0.8)
    ax.set_ylabel("Angle (deg)")
    ax.grid(True, alpha=0.3)

    # Current
    ax = axes[2]
    ax.plot(t, current, "m-", linewidth=1)
    ax.axvline(0, color="gray", linestyle=":", linewidth=0.8)
    ax.set_ylabel("Current (mA)")
    ax.set_xlabel("Time (s)")
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()


# ── CLI ─────────────────────────────────────────────────────────────────────

def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Step response capture & plotting for PID tuning",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "PID gains are Q8 fixed-point: value 256 = 1.0, 128 = 0.5, etc.\n"
            "Example: --pid 128 16 0  sets Kp=0.5, Ki=0.0625, Kd=0"
        ),
    )
    parser.add_argument("-p", "--port", help="Serial port (auto-detect if omitted)")
    parser.add_argument("-b", "--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--trace", action="store_true", help="Show raw TX/RX hex")

    parser.add_argument("address", type=int, help="Device address on the ring")

    step = parser.add_mutually_exclusive_group(required=True)
    step.add_argument("--velocity", type=int, metavar="RPM",
                      help="Velocity step target (signed RPM)")
    step.add_argument("--duty", type=int, metavar="DUTY",
                      help="Duty step target (signed, open-loop)")

    parser.add_argument("--pid", nargs=3, type=int, metavar=("KP", "KI", "KD"),
                        help="Set PID gains before step (Q8)")
    parser.add_argument("--ff", type=int, metavar="GAIN",
                        help="Set velocity feedforward gain (Q8: duty per RPM)")
    parser.add_argument("--torque", type=int, metavar="MA",
                        help="Set torque limit (mA) before step")
    parser.add_argument("--duration", type=float, default=2.0,
                        help="Step capture duration in seconds (default: 2.0)")
    parser.add_argument("--pre", type=float, default=0.3,
                        help="Pre-step baseline capture in seconds (default: 0.3)")
    parser.add_argument("--csv", metavar="FILE",
                        help="Save raw samples to CSV file")
    parser.add_argument("--no-plot", action="store_true",
                        help="Skip the matplotlib plot")

    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    port = args.port or auto_detect_port()
    if not port:
        print("ERROR: no serial port found. Use -p to specify.")
        return 1
    if args.port is None:
        print(f"Auto-detected port: {port}")

    client = RingClientV2(
        port=port,
        baudrate=args.baud,
        timeout_ms=150,
        settle_ms=250,
        trace=args.trace,
    )

    try:
        client.open()

        try:
            count = client.enumerate()
            print(f"Enumerated {count} device(s)")
        except RingError as exc:
            print(f"WARNING: enumerate failed ({exc}), proceeding anyway")

        pid = tuple(args.pid) if args.pid else None

        samples, t_step = run_step(
            client=client,
            address=args.address,
            velocity=args.velocity,
            duty=args.duty,
            pid=pid,
            ff_gain=args.ff,
            torque_ma=args.torque,
            pre_duration=args.pre,
            step_duration=args.duration,
        )

        if args.csv:
            save_csv(args.csv, samples, t_step)

        # Build a descriptive title
        if args.velocity is not None:
            step_desc = f"velocity={args.velocity} RPM"
        else:
            step_desc = f"duty={args.duty}"
        pid_desc = f"  Kp={pid[0]} Ki={pid[1]} Kd={pid[2]}" if pid else ""
        ff_desc = f"  Kff={args.ff}" if args.ff else ""
        title = f"Step Response: {step_desc}{pid_desc}{ff_desc}"

        if not args.no_plot:
            plot_step(samples, t_step, title)

        return 0

    except RingError as exc:
        print(f"ERROR: {exc}")
        return 1
    except KeyboardInterrupt:
        print("\nCancelled — stopping motor...")
        try:
            client.stop(args.address)
        except Exception:
            pass
        return 130
    finally:
        client.close()


if __name__ == "__main__":
    sys.exit(main())
