#!/usr/bin/env python3
"""
tune_tool.py - Step response capture & plotting for M2003 motor PID tuning

Captures status telemetry before, during, and after a step command,
then plots velocity, angle, and current vs time.

Examples:
    # Measure feedforward gain (run first!)
    python tune_tool.py -p COM7 0 --measure-ff

    # Velocity step to 500 RPM, default PID gains
    python tune_tool.py -p COM7 0 --velocity 500

    # Set PID gains, then step
    python tune_tool.py -p COM7 0 --velocity 500 --pid 256 32 0

    # Position step to 1/4 turn (4096 counts), with position PID
    python tune_tool.py -p COM7 0 --position 4096 --pos-pid 16 1 0

    # Position step with velocity PID and feedforward too
    python tune_tool.py -p COM7 0 --position 4096 --pos-pid 16 1 0 --pid 256 8 0 --ff 102

    # Duty step (open-loop) for 1 second
    python tune_tool.py -p COM7 0 --duty 400 --duration 1

    # Save raw data to CSV (no plot)
    python tune_tool.py -p COM7 0 --velocity 500 --csv step.csv --no-plot

Requires: pyserial, matplotlib (pip install pyserial matplotlib)
"""

from __future__ import annotations

import argparse
import csv
import os
import sys
import time

# Import the reusable ring bus client module from the same directory
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ring_bus import (
    RingClientV2,
    MotorStatus,
    StrikeStatus,
    RingError,
    RingTimeout,
    auto_detect_port,
    DEFAULT_BAUD,
    STRIKE_PARAM_HOME_OFFSET,
    STRIKE_PARAM_COAST_DISTANCE,
    STRIKE_PARAM_HOMING_DUTY,
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
    position: int | None,
    pid: tuple[int, int, int] | None,
    pos_pid: tuple[int, int, int] | None,
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
    print("Stopping motor...")
    client.stop(address)
    time.sleep(0.05)

    # 2. For position steps, zero the encoder first
    if position is not None:
        print("Zeroing encoder position...")
        client.zero_position(address)
        time.sleep(0.02)

    # 3. Optionally set velocity PID gains
    if pid is not None:
        kp, ki, kd = pid
        print(f"Setting vel PID: kp={kp} ki={ki} kd={kd}")
        client.set_pid(address, kp, ki, kd)

    # 4. Optionally set position PID gains
    if pos_pid is not None:
        kp, ki, kd = pos_pid
        print(f"Setting pos PID: kp={kp} ki={ki} kd={kd}")
        client.set_pos_pid(address, kp, ki, kd)

    # 5. Optionally set feedforward
    if ff_gain is not None:
        print(f"Setting feedforward: {ff_gain}")
        client.set_ff(address, ff_gain)

    # 6. Optionally set torque limit
    if torque_ma is not None:
        print(f"Setting torque limit: {torque_ma} mA")
        client.set_torque(address, torque_ma)

    all_samples: list[tuple[float, MotorStatus]] = []

    # 7. Pre-step baseline
    if pre_duration > 0:
        print(f"Capturing baseline ({pre_duration:.1f}s)...")
        all_samples.extend(capture_loop(client, address, pre_duration))

    # 8. Step command
    t_step = time.monotonic()
    if position is not None:
        print(f"STEP: position={position} counts")
        s = client.set_position(address, position)
        all_samples.append((time.monotonic(), s))
    elif velocity is not None:
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


# ── Feedforward measurement ─────────────────────────────────────────────────

def measure_ff(
    client: RingClientV2,
    address: int,
    duty_steps: list[int],
    settle_s: float,
    sample_s: float,
) -> int:
    """
    Measure motor Kv by spinning at several open-loop duty levels and
    capturing steady-state RPM.  Returns the Q8 feedforward gain.
    """
    print("Measuring feedforward (Kv)...")
    print(f"  Duty steps: {duty_steps}")
    print(f"  Settle: {settle_s:.1f}s, sample: {sample_s:.1f}s per step\n")

    # Disable PID — pure open-loop
    client.set_pid(address, 0, 0, 0)
    client.set_ff(address, 0)

    points: list[tuple[int, float]] = []  # (duty, avg_rpm)

    for duty in duty_steps:
        print(f"  duty={duty:+5d} ... ", end="", flush=True)
        client.set_duty(address, duty)
        time.sleep(settle_s)

        # Sample velocity over sample_s seconds
        samples = capture_loop(client, address, sample_s)
        if not samples:
            print("no data")
            continue

        velocities = [s[1].velocity for s in samples]
        avg_rpm = sum(velocities) / len(velocities)
        print(f"avg {avg_rpm:+7.1f} RPM  ({len(velocities)} samples)")
        points.append((duty, avg_rpm))

    client.stop(address)

    # Filter out points where motor didn't spin (stall / friction zone)
    valid = [(d, r) for d, r in points if abs(r) > 10]
    if len(valid) < 2:
        print("\nERROR: not enough valid data points. Try higher duty values.")
        return 0

    # Linear regression: duty = Kff_real * rpm + offset
    # Kff_real = sum(d*r) / sum(r*r)  (force through zero approx)
    # More robust: least-squares slope
    n = len(valid)
    sum_d = sum(d for d, _ in valid)
    sum_r = sum(r for _, r in valid)
    sum_dr = sum(d * r for d, r in valid)
    sum_rr = sum(r * r for _, r in valid)

    denom = n * sum_rr - sum_r * sum_r
    if abs(denom) < 1e-6:
        print("\nERROR: degenerate data (all same RPM?)")
        return 0

    slope = (n * sum_dr - sum_d * sum_r) / denom  # duty per RPM
    kff_q8 = int(round(slope * 256))

    print(f"\n  Kv = {slope:.4f} duty/RPM")
    print(f"  Kff (Q8) = {kff_q8}")
    print(f"\n  Use:  --ff {kff_q8}")

    # Optional: show the fit
    try:
        import matplotlib.pyplot as plt
        rpms = [r for _, r in valid]
        duties = [d for d, _ in valid]
        fit_r = [min(rpms) - 50, max(rpms) + 50]
        fit_d = [slope * r + (sum_d - slope * sum_r) / n for r in fit_r]

        plt.figure(figsize=(7, 4))
        plt.scatter(rpms, duties, c="blue", zorder=5, label="measured")
        plt.plot(fit_r, fit_d, "r--", label=f"fit: Kv={slope:.4f}")
        plt.xlabel("Velocity (RPM)")
        plt.ylabel("Duty")
        plt.title(f"Feedforward Measurement: Kff={kff_q8} (Q8)")
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.show()
    except ImportError:
        pass

    return kff_q8


# ── CSV export ──────────────────────────────────────────────────────────────

def save_csv(path: str, samples: list[tuple[float, MotorStatus]], t_step: float) -> None:
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_sec", "state", "fault", "mode", "current_ma", "hall",
                     "angle", "angle_deg", "velocity_rpm", "target", "position"])
        for t, s in samples:
            w.writerow([
                f"{t - t_step:.4f}",
                s.state, s.fault, s.mode, s.current_ma, s.hall,
                s.angle, f"{s.angle_deg:.2f}", s.velocity, s.target, s.position,
            ])
    print(f"Saved {len(samples)} rows to {path}")


# ── Plotting ────────────────────────────────────────────────────────────────

def plot_step(
    samples: list[tuple[float, MotorStatus]],
    t_step: float,
    title: str,
    position_target: int | None = None,
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
    pos = [s[1].position for s in samples]
    angle = [s[1].angle_deg for s in samples]
    current = [s[1].current_ma for s in samples]
    mode = samples[-1][1].mode  # use last sample's mode for label
    is_pos_mode = (mode == 2)

    fig, axes = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
    fig.suptitle(title, fontsize=12)

    if is_pos_mode:
        # Position (top plot for position mode)
        ax = axes[0]
        ax.plot(t, pos, "b-", linewidth=1, label="position")
        if position_target is not None:
            ax.axhline(position_target, color="r", linestyle="--",
                       linewidth=1, alpha=0.7, label=f"target ({position_target})")
        ax.axvline(0, color="gray", linestyle=":", linewidth=0.8, label="step")
        ax.set_ylabel("Position (counts)")
        ax.legend(loc="upper right", fontsize=8)
        ax.grid(True, alpha=0.3)

        # Velocity (shows the cascade's intermediate signal)
        ax = axes[1]
        ax.plot(t, vel, "b-", linewidth=1, label="measured")
        ax.plot(t, tgt, "r--", linewidth=1, alpha=0.7, label="vel setpoint")
        ax.axvline(0, color="gray", linestyle=":", linewidth=0.8)
        ax.set_ylabel("Velocity (RPM)")
        ax.legend(loc="upper right", fontsize=8)
        ax.grid(True, alpha=0.3)
    else:
        # Velocity (top plot for velocity/duty mode)
        ax = axes[0]
        ax.plot(t, vel, "b-", linewidth=1, label="measured")
        ax.plot(t, tgt, "r--", linewidth=1, alpha=0.7,
                label="target (rpm)" if mode == 1 else "target (duty)")
        ax.axvline(0, color="gray", linestyle=":", linewidth=0.8, label="step")
        ax.set_ylabel("Velocity (RPM)")
        ax.legend(loc="upper right", fontsize=8)
        ax.grid(True, alpha=0.3)

        # Position
        ax = axes[1]
        ax.plot(t, pos, "g-", linewidth=1)
        ax.axvline(0, color="gray", linestyle=":", linewidth=0.8)
        ax.set_ylabel("Position (counts)")
        ax.grid(True, alpha=0.3)

    # Current (always bottom)
    ax = axes[2]
    ax.plot(t, current, "m-", linewidth=1)
    ax.axvline(0, color="gray", linestyle=":", linewidth=0.8)
    ax.set_ylabel("Current (mA)")
    ax.set_xlabel("Time (s)")
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()


# ── Strike test ────────────────────────────────────────────────────────────

STRIKE_STATE_NAMES = {0: "IDLE", 1: "HOMING", 2: "DRIVING", 3: "COASTING", 4: "BRAKING", 5: "CATCHING"}


def wait_for_homed(client: RingClientV2, address: int, timeout: float = 15.0) -> StrikeStatus:
    """Poll strike status until homing is complete."""
    deadline = time.monotonic() + timeout
    last_state = -1
    while time.monotonic() < deadline:
        try:
            ss = client.query_strike(address)
            if ss.state != last_state:
                print(f"  strike state: {STRIKE_STATE_NAMES.get(ss.state, '?')}")
                last_state = ss.state
            if ss.homed and ss.state == 0:
                return ss
        except RingTimeout:
            pass
        time.sleep(0.05)
    raise RingError("homing timed out")


def wait_for_strike_idle(client: RingClientV2, address: int, timeout: float = 5.0) -> None:
    """Poll strike status until the strike cycle completes."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            ss = client.query_strike(address)
            if ss.state == 0:
                return
        except RingTimeout:
            pass
        time.sleep(0.02)


def run_strike(
    client: RingClientV2,
    address: int,
    strike_duty: int,
    home_offset: int | None,
    coast_distance: int | None,
    homing_duty: int | None,
    pre_duration: float,
    capture_duration: float,
) -> tuple[list[tuple[float, MotorStatus]], float, StrikeStatus]:
    """
    Run a full strike test: configure, home (if needed), capture telemetry through strike.

    Returns (samples, t_strike, strike_status).
    """
    # 1. Configure strike parameters
    if home_offset is not None:
        print(f"Setting home offset: {home_offset}")
        client.set_strike_param(address, STRIKE_PARAM_HOME_OFFSET, home_offset)
    if coast_distance is not None:
        print(f"Setting coast distance: {coast_distance}")
        client.set_strike_param(address, STRIKE_PARAM_COAST_DISTANCE, coast_distance)
    if homing_duty is not None:
        print(f"Setting homing duty: {homing_duty}")
        client.set_strike_param(address, STRIKE_PARAM_HOMING_DUTY, homing_duty)

    # 2. Check if already homed
    ss = client.query_strike(address)
    if not ss.homed:
        print("Running homing sequence...")
        client.strike_home(address)
        ss = wait_for_homed(client, address)
    print(f"Homed: drum={ss.drum_position}, home={ss.home_position}")

    all_samples: list[tuple[float, MotorStatus]] = []

    # 3. Pre-strike baseline
    if pre_duration > 0:
        print(f"Capturing baseline ({pre_duration:.1f}s)...")
        all_samples.extend(capture_loop(client, address, pre_duration))

    # 4. Fire strike
    t_strike = time.monotonic()
    print(f"STRIKE: duty={strike_duty}")
    s = client.strike(address, strike_duty)
    all_samples.append((time.monotonic(), s))

    # 5. Capture through the strike cycle
    print(f"Capturing strike cycle ({capture_duration:.1f}s)...")
    all_samples.extend(capture_loop(client, address, capture_duration))

    # 6. Wait for strike to complete (if not already)
    wait_for_strike_idle(client, address, timeout=2.0)

    # 7. Brief post-strike capture
    all_samples.extend(capture_loop(client, address, 0.3))

    rate = len(all_samples) / (pre_duration + capture_duration + 0.3) if all_samples else 0
    print(f"Captured {len(all_samples)} samples ({rate:.0f} Hz effective)")

    return all_samples, t_strike, ss


def plot_strike(
    samples: list[tuple[float, MotorStatus]],
    t_strike: float,
    strike_info: StrikeStatus,
    title: str,
) -> None:
    try:
        import matplotlib.pyplot as plt
        from matplotlib.patches import Patch
    except ImportError:
        print("ERROR: matplotlib required. Install with: pip install matplotlib")
        return

    if not samples:
        print("No samples to plot.")
        return

    t = [s[0] - t_strike for s in samples]
    pos = [s[1].position for s in samples]
    vel = [s[1].velocity for s in samples]
    current = [s[1].current_ma for s in samples]

    home_pos = strike_info.home_position
    drum_pos = strike_info.drum_position

    fig, axes = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
    fig.suptitle(title, fontsize=12)

    # Position
    ax = axes[0]
    ax.plot(t, pos, "b-", linewidth=1, label="position")
    ax.axhline(home_pos, color="green", linestyle="--", linewidth=1, alpha=0.7,
               label=f"home ({home_pos})")
    ax.axhline(drum_pos, color="red", linestyle="--", linewidth=1, alpha=0.7,
               label=f"drum ({drum_pos})")
    ax.axvline(0, color="gray", linestyle=":", linewidth=0.8, label="strike")
    ax.set_ylabel("Position (counts)")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)

    # Velocity
    ax = axes[1]
    ax.plot(t, vel, "b-", linewidth=1, label="velocity")
    ax.axhline(0, color="gray", linestyle="-", linewidth=0.5, alpha=0.5)
    ax.axvline(0, color="gray", linestyle=":", linewidth=0.8)
    # Mark the rebound (first velocity sign change after strike)
    for i in range(1, len(vel)):
        if t[i] > 0 and vel[i - 1] != 0 and vel[i] != 0:
            if (vel[i - 1] > 0) != (vel[i] > 0):
                ax.axvline(t[i], color="orange", linestyle="--", linewidth=1,
                           alpha=0.7, label=f"rebound ({t[i]:.3f}s)")
                break
    ax.set_ylabel("Velocity (RPM)")
    ax.legend(loc="upper right", fontsize=8)
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
    step.add_argument("--position", type=int, metavar="COUNTS",
                      help="Position step target (encoder counts, auto-zeros first)")
    step.add_argument("--measure-ff", action="store_true",
                      help="Measure feedforward gain (Kff) by open-loop sweep")
    step.add_argument("--strike", type=int, metavar="DUTY",
                      help="Strike test: home, strike at given duty, capture cycle")

    parser.add_argument("--pid", nargs=3, type=int, metavar=("KP", "KI", "KD"),
                        help="Set velocity PID gains before step (Q8)")
    parser.add_argument("--pos-pid", nargs=3, type=int, metavar=("KP", "KI", "KD"),
                        help="Set position PID gains before step (Q8, output=RPM)")
    parser.add_argument("--ff", type=int, metavar="GAIN",
                        help="Set velocity feedforward gain (Q8: duty per RPM)")
    parser.add_argument("--torque", type=int, metavar="MA",
                        help="Set torque limit (mA) before step")

    # Strike-specific options
    parser.add_argument("--home-offset", type=int, metavar="COUNTS",
                        help="Strike: encoder counts above drum for home position")
    parser.add_argument("--coast-distance", type=int, metavar="COUNTS",
                        help="Strike: cut power this many counts from drum")
    parser.add_argument("--homing-duty", type=int, metavar="DUTY",
                        help="Strike: duty for drum-sensing homing (sign = toward drum)")
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

        if args.measure_ff:
            # Sweep several duty levels in both directions
            duties = [150, 250, 350, 500, 700, -150, -250, -350, -500, -700]
            measure_ff(client, args.address, duties,
                       settle_s=1.0, sample_s=0.5)
            return 0

        if args.strike is not None:
            samples, t_strike, strike_info = run_strike(
                client=client,
                address=args.address,
                strike_duty=args.strike,
                home_offset=args.home_offset,
                coast_distance=args.coast_distance,
                homing_duty=args.homing_duty,
                pre_duration=args.pre,
                capture_duration=args.duration,
            )

            if args.csv:
                save_csv(args.csv, samples, t_strike)

            title = f"Strike Test: duty={args.strike}"
            if args.home_offset is not None:
                title += f"  home_offset={args.home_offset}"
            if args.coast_distance is not None:
                title += f"  coast={args.coast_distance}"

            if not args.no_plot:
                plot_strike(samples, t_strike, strike_info, title)
            return 0

        pid = tuple(args.pid) if args.pid else None
        pos_pid_arg = tuple(args.pos_pid) if args.pos_pid else None

        samples, t_step = run_step(
            client=client,
            address=args.address,
            velocity=args.velocity,
            duty=args.duty,
            position=args.position,
            pid=pid,
            pos_pid=pos_pid_arg,
            ff_gain=args.ff,
            torque_ma=args.torque,
            pre_duration=args.pre,
            step_duration=args.duration,
        )

        if args.csv:
            save_csv(args.csv, samples, t_step)

        # Build a descriptive title
        if args.position is not None:
            step_desc = f"position={args.position} counts"
        elif args.velocity is not None:
            step_desc = f"velocity={args.velocity} RPM"
        else:
            step_desc = f"duty={args.duty}"
        pid_desc = f"  vel_pid={pid}" if pid else ""
        pos_pid_desc = f"  pos_pid={pos_pid_arg}" if pos_pid_arg else ""
        ff_desc = f"  Kff={args.ff}" if args.ff else ""
        title = f"Step Response: {step_desc}{pid_desc}{pos_pid_desc}{ff_desc}"

        if not args.no_plot:
            plot_step(samples, t_step, title, position_target=args.position)

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
