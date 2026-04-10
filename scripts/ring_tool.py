#!/usr/bin/env python3
"""
ring_tool.py - CLI wrapper around the reusable ring bus modules.

Examples:
    python ring_tool.py -p COM7 enumerate
    python ring_tool.py -p COM7 status 0
    python ring_tool.py -p COM7 set-duty 0 400
    python ring_tool.py -p COM7 set-velocity 0 500
    python ring_tool.py -p COM7 zero-pos 0
    python ring_tool.py -p COM7 save-settings 0
    python ring_tool.py -p COM7 set-pid 0 128 16 0
    python ring_tool.py -p COM7 set-mode 0 1
    python ring_tool.py -p COM7 torque 0 2500
    python ring_tool.py -p COM7 stop 0
    python ring_tool.py -p COM7 broadcast 200 0 0
    python ring_tool.py -p COM7 monitor 0 --hz 10
    python ring_tool.py -p COM7 measure-strike-timing 0 --start 100 --stop 800 --step 100 --csv strike.csv
    python ring_tool.py -p COM7 measure-strike-timing 0 --sweep-param home-offset --start 1200 --stop 2200 --step 200 --strike-duty 500
"""

from __future__ import annotations

import argparse
import json
import sys
import time

import serial

from ring_bus import *  # noqa: F401,F403 - compatibility re-export
from ring_measure import *  # noqa: F401,F403 - compatibility re-export


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Ring Bus Protocol v2 client for M2003 motor boards"
    )
    parser.add_argument("-p", "--port", help="Serial port (auto-detect if omitted)")
    parser.add_argument(
        "-b",
        "--baud",
        type=int,
        default=DEFAULT_BAUD,
        help=f"Baud rate (default: {DEFAULT_BAUD})",
    )
    parser.add_argument(
        "--timeout-ms",
        type=int,
        default=DEFAULT_TIMEOUT_MS,
        help=f"Response timeout in ms (default: {DEFAULT_TIMEOUT_MS})",
    )
    parser.add_argument(
        "--settle-ms",
        type=int,
        default=DEFAULT_SETTLE_MS,
        help=f"Post-open settle delay in ms (default: {DEFAULT_SETTLE_MS})",
    )
    parser.add_argument("--trace", action="store_true", help="Print raw TX/RX hex for each frame")

    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("ports", help="List serial ports")
    sub.add_parser("enumerate", help="Enumerate devices on the ring")

    sp = sub.add_parser("status", help="Query status from a device")
    sp.add_argument("address", type=int)

    sp = sub.add_parser("set-duty", help="Set signed duty on a device")
    sp.add_argument("address", type=int)
    sp.add_argument("duty", type=int, help="Signed int16 duty")

    sp = sub.add_parser("torque", help="Set torque limit (mA)")
    sp.add_argument("address", type=int)
    sp.add_argument("milliamps", type=int)

    sp = sub.add_parser("stop", help="Stop a device")
    sp.add_argument("address", type=int)

    sp = sub.add_parser("clear-fault", help="Clear fault on a device")
    sp.add_argument("address", type=int)

    sp = sub.add_parser("set-position", help="Set position target (encoder counts, auto-sets POSITION mode)")
    sp.add_argument("address", type=int)
    sp.add_argument("counts", type=int, help="Signed int32 target in encoder counts")

    sp = sub.add_parser("set-pos-pid", help="Set position PID gains (Q8, output=RPM)")
    sp.add_argument("address", type=int)
    sp.add_argument("kp", type=int, help="Proportional gain (Q8)")
    sp.add_argument("ki", type=int, help="Integral gain (Q8)")
    sp.add_argument("kd", type=int, help="Derivative gain (Q8)")

    sp = sub.add_parser("zero-pos", help="Set the current absolute encoder angle as logical zero and save it")
    sp.add_argument("address", type=int)

    sp = sub.add_parser("save-settings", help="Save current runtime settings/calibration to flash")
    sp.add_argument("address", type=int)

    sp = sub.add_parser("clear-settings", help="Erase persisted settings from flash for next boot")
    sp.add_argument("address", type=int)

    sp = sub.add_parser("set-mode", help="Set control mode (0=DUTY, 1=VELOCITY, 2=POSITION)")
    sp.add_argument("address", type=int)
    sp.add_argument("mode", type=int, choices=[0, 1, 2], help="0=DUTY, 1=VELOCITY, 2=POSITION")

    sp = sub.add_parser("set-velocity", help="Set velocity target (RPM, auto-sets VELOCITY mode)")
    sp.add_argument("address", type=int)
    sp.add_argument("rpm", type=int, help="Signed int16 target RPM")

    sp = sub.add_parser("set-pid", help="Set velocity PID gains (Q8 fixed-point)")
    sp.add_argument("address", type=int)
    sp.add_argument("kp", type=int, help="Proportional gain (Q8: 256 = 1.0)")
    sp.add_argument("ki", type=int, help="Integral gain (Q8)")
    sp.add_argument("kd", type=int, help="Derivative gain (Q8)")

    sp = sub.add_parser("set-ff", help="Set velocity feedforward gain (Q8)")
    sp.add_argument("address", type=int)
    sp.add_argument("gain", type=int, help="Duty per RPM of back-EMF (Q8: 256 = 1.0)")

    sp = sub.add_parser("strike", help="Trigger a strike")
    sp.add_argument("address", type=int)
    sp.add_argument("duty", type=int, help="Signed strike duty (loudness)")

    sp = sub.add_parser("strike-home", help="Run strike homing sequence")
    sp.add_argument("address", type=int)

    sp = sub.add_parser("strike-cancel", help="Cancel strike/homing")
    sp.add_argument("address", type=int)

    sp = sub.add_parser("strike-status", help="Query strike status")
    sp.add_argument("address", type=int)

    sp = sub.add_parser("set-strike-param", help="Set strike parameter")
    sp.add_argument("address", type=int)
    sp.add_argument("param", choices=["home-offset", "coast-distance", "homing-duty"])
    sp.add_argument("value", type=int)

    sp = sub.add_parser("measure-strike-timing", help="Sweep strike timing versus duty or home-offset")
    sp.add_argument("address", type=int)
    sp.add_argument("--sweep-param", choices=["duty", "home-offset"], default="duty",
                    help="Parameter to sweep (default: duty)")
    sp.add_argument("--values", nargs="+", type=int,
                    help="Explicit sweep values; overrides --start/--stop/--step")
    sp.add_argument("--duties", nargs="+", type=int,
                    help="Explicit duty sweep values; duty sweep only, overrides --start/--stop/--step")
    sp.add_argument("--start", type=int, default=100,
                    help="Start value for generated sweep (default: 100)")
    sp.add_argument("--stop", type=int, default=1000,
                    help="Stop value for generated sweep, inclusive (default: 1000)")
    sp.add_argument("--step", type=int, default=100,
                    help="Step for generated sweep (default: 100)")
    sp.add_argument("--strike-duty", type=int, default=400,
                    help="Strike duty to use when sweeping non-duty parameters (default: 400)")
    sp.add_argument("--repeats", type=int, default=3,
                    help="Samples to collect per sweep value (default: 3)")
    sp.add_argument("--poll-ms", type=int, default=10,
                    help="Strike-status polling interval in ms (default: 10)")
    sp.add_argument("--idle-timeout-ms", type=int, default=3000,
                    help="Timeout waiting for idle before each strike (default: 3000)")
    sp.add_argument("--strike-timeout-ms", type=int, default=3000,
                    help="Timeout waiting for each strike to finish (default: 3000)")
    sp.add_argument("--pause-ms", type=int, default=100,
                    help="Pause after each completed strike sample (default: 100)")
    sp.add_argument("--csv", help="Write raw per-strike timing samples to CSV")
    sp.add_argument("--plot-out", help="Save the generated plot image to this path")
    sp.add_argument("--no-show", action="store_true", help="Do not open the plot window")
    sp.add_argument("--no-plot", action="store_true", help="Collect data without plotting")
    sp.add_argument("--title", help="Custom plot title")

    sp = sub.add_parser(
        "build-strike-calibration",
        help="Build a scheduler-ready JSON calibration profile from strike timing CSV sweeps",
    )
    sp.add_argument("--duty-csv", required=True, help="CSV produced by measure-strike-timing with --sweep-param=duty")
    sp.add_argument(
        "--home-offset-csv",
        help="Optional CSV produced by measure-strike-timing with --sweep-param=home-offset",
    )
    sp.add_argument("--out", required=True, help="Path to write the calibration profile JSON")
    sp.add_argument("--address", type=int, help="Optional ring address metadata to store in the profile")
    sp.add_argument("--midi-note", type=int, help="Optional MIDI note metadata to store in the profile")
    sp.add_argument("--name", help="Optional human-readable note or mallet name")
    sp.add_argument(
        "--preferred-home-offset",
        type=int,
        help="Override automatic home-offset selection by choosing a specific candidate from the home-offset sweep",
    )

    sp = sub.add_parser(
        "build-strike-calibration-grid",
        help="Build a multi-home-offset JSON calibration profile from a nested duty sweep CSV",
    )
    sp.add_argument(
        "--duty-csv",
        required=True,
        help="CSV produced by a multi-home-offset duty sweep; configured_home_offset must vary across rows",
    )
    sp.add_argument("--out", required=True, help="Path to write the calibration profile JSON")
    sp.add_argument("--address", type=int, help="Optional ring address metadata to store in the profile")
    sp.add_argument("--midi-note", type=int, help="Optional MIDI note metadata to store in the profile")
    sp.add_argument("--name", help="Optional human-readable note or mallet name")
    sp.add_argument(
        "--preferred-home-offset",
        type=int,
        help="Override automatic home-offset selection by choosing a specific offset profile",
    )

    sp = sub.add_parser(
        "measure-strike-calibration-grid",
        help="Sweep duty across multiple home offsets and emit a multi-offset calibration profile",
    )
    sp.add_argument("address", type=int)
    sp.add_argument(
        "--home-offsets",
        nargs="+",
        type=int,
        help="Explicit home offset values; overrides --home-offset-start/--stop/--step",
    )
    sp.add_argument("--home-offset-start", type=int, default=800, help="Generated home-offset sweep start (default: 800)")
    sp.add_argument("--home-offset-stop", type=int, default=1600, help="Generated home-offset sweep stop, inclusive (default: 1600)")
    sp.add_argument("--home-offset-step", type=int, default=200, help="Generated home-offset sweep step (default: 200)")
    sp.add_argument(
        "--duties",
        nargs="+",
        type=int,
        help="Explicit strike duty values; overrides --duty-start/--stop/--step",
    )
    sp.add_argument("--duty-start", type=int, default=100, help="Generated duty sweep start (default: 100)")
    sp.add_argument("--duty-stop", type=int, default=800, help="Generated duty sweep stop, inclusive (default: 800)")
    sp.add_argument("--duty-step", type=int, default=20, help="Generated duty sweep step (default: 20)")
    sp.add_argument("--repeats", type=int, default=3, help="Samples to collect per home-offset/duty point (default: 3)")
    sp.add_argument("--poll-ms", type=int, default=10, help="Strike-status polling interval in ms (default: 10)")
    sp.add_argument("--idle-timeout-ms", type=int, default=3000, help="Timeout waiting for idle before each strike (default: 3000)")
    sp.add_argument("--strike-timeout-ms", type=int, default=3000, help="Timeout waiting for each strike to finish (default: 3000)")
    sp.add_argument("--pause-ms", type=int, default=100, help="Pause after each completed strike sample (default: 100)")
    sp.add_argument("--csv", help="Write raw nested sweep samples to CSV")
    sp.add_argument("--out", required=True, help="Path to write the calibration profile JSON")
    sp.add_argument("--midi-note", type=int, help="Optional MIDI note metadata to store in the profile")
    sp.add_argument("--name", help="Optional human-readable note or mallet name")
    sp.add_argument(
        "--preferred-home-offset",
        type=int,
        help="Override automatic home-offset selection by choosing a specific offset profile",
    )

    sp = sub.add_parser("broadcast", help="Broadcast duty to all devices")
    sp.add_argument("duties", nargs="+", type=int, help="One signed duty per device")

    sp = sub.add_parser("monitor", help="Poll status continuously")
    sp.add_argument("address", type=int)
    sp.add_argument("--hz", type=float, default=10.0, help="Poll rate in Hz (default: 10)")

    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    if args.command == "ports":
        return list_ports()

    if args.command == "build-strike-calibration":
        duty_samples = load_strike_timing_csv(args.duty_csv)
        home_offset_samples = load_strike_timing_csv(args.home_offset_csv) if args.home_offset_csv else None
        profile = build_strike_calibration_profile(
            duty_samples=duty_samples,
            home_offset_samples=home_offset_samples,
            address=args.address,
            midi_note=args.midi_note,
            note_name=args.name,
            preferred_home_offset=args.preferred_home_offset,
        )
        with open(args.out, "w", encoding="ascii") as json_file:
            json.dump(profile, json_file, indent=2)
            json_file.write("\n")

        print(f"Calibration profile saved to {args.out}")
        print(f"lookup_home_offset={profile['lookup_home_offset']} suggested_home_offset={profile['suggested_home_offset']}")
        print(f"scheduler_ready={int(profile['scheduler_ready'])} lookup_entries={len(profile['midi_velocity_lookup'])}")
        for warning in profile["warnings"]:
            print(f"WARNING: {warning}")
        return 0

    if args.command == "build-strike-calibration-grid":
        duty_samples = load_strike_timing_csv(args.duty_csv)
        profile = build_strike_calibration_grid_profile(
            duty_samples=duty_samples,
            address=args.address,
            midi_note=args.midi_note,
            note_name=args.name,
            preferred_home_offset=args.preferred_home_offset,
        )
        with open(args.out, "w", encoding="ascii") as json_file:
            json.dump(profile, json_file, indent=2)
            json_file.write("\n")

        print(f"Calibration grid profile saved to {args.out}")
        print(f"suggested_home_offset={profile['suggested_home_offset']} profile_count={len(profile['home_offset_profiles'])}")
        print(f"scheduler_ready={int(profile['scheduler_ready'])}")
        for warning in profile["warnings"]:
            print(f"WARNING: {warning}")
        return 0

    port = args.port or auto_detect_port()
    if not port:
        print("ERROR: no serial port found. Use -p to specify.")
        return 1
    if args.port is None:
        print(f"Auto-detected port: {port}")

    client = RingClientV2(
        port=port,
        baudrate=args.baud,
        timeout_ms=args.timeout_ms,
        settle_ms=args.settle_ms,
        trace=args.trace,
    )

    try:
        client.open()

        if args.command == "enumerate":
            count = client.enumerate()
            print(f"devices={count}")
            return 0

        try:
            count = client.enumerate()
            print(f"devices={count}")
            if count == 0:
                print("WARNING: enumeration returned 0 devices")
        except RingError as exc:
            print(f"WARNING: enumerate failed ({exc}), proceeding anyway")

        if args.command == "status":
            print(format_status(client.query_status(args.address)))

        elif args.command == "set-duty":
            print(format_status(client.set_duty(args.address, args.duty)))

        elif args.command == "torque":
            print(format_status(client.set_torque(args.address, args.milliamps)))

        elif args.command == "stop":
            print(format_status(client.stop(args.address)))

        elif args.command == "clear-fault":
            print(format_status(client.clear_fault(args.address)))

        elif args.command == "set-position":
            print(format_status(client.set_position(args.address, args.counts)))

        elif args.command == "set-pos-pid":
            print(format_status(client.set_pos_pid(args.address, args.kp, args.ki, args.kd)))

        elif args.command == "zero-pos":
            print(format_status(client.zero_position(args.address)))

        elif args.command == "save-settings":
            print(format_status(client.save_settings(args.address)))

        elif args.command == "clear-settings":
            print(format_status(client.clear_settings(args.address)))

        elif args.command == "set-mode":
            print(format_status(client.set_mode(args.address, args.mode)))

        elif args.command == "set-velocity":
            print(format_status(client.set_velocity(args.address, args.rpm)))

        elif args.command == "set-pid":
            print(format_status(client.set_pid(args.address, args.kp, args.ki, args.kd)))

        elif args.command == "set-ff":
            print(format_status(client.set_ff(args.address, args.gain)))

        elif args.command == "strike":
            print(format_status(client.strike(args.address, args.duty)))

        elif args.command == "strike-home":
            print(format_status(client.strike_home(args.address)))

        elif args.command == "strike-cancel":
            print(format_status(client.strike_cancel(args.address)))

        elif args.command == "strike-status":
            status = client.query_strike(args.address)
            print(
                f"addr={status.address} strike={status.state_name} homed={status.homed} "
                f"seq={status.sequence} flags=0x{status.flags:02X} duty={status.last_duty} "
                f"coast_ms={status.trigger_to_coast_ms if status.coast_valid else -1} "
                f"rebound_ms={status.trigger_to_rebound_ms if status.rebound_valid else -1} "
                f"retrigger_ready_ms={status.trigger_to_retrigger_ready_ms if status.retrigger_ready_valid else -1} "
                f"ready_ms={status.trigger_to_ready_ms if status.ready_valid else -1} "
                f"strike_vel_dps={status.estimated_strike_velocity_dps if status.velocity_valid and status.estimated_strike_velocity_dps is not None else -1} "
                f"active={int(status.active)} retriggered={int(status.retriggered)} "
                f"retrigger_ready={int(status.retrigger_ready_valid)} "
                f"rebound_timeout={int(status.rebound_timeout)} "
                f"drum_pos={status.drum_position} home_pos={status.home_position} "
                f"home_offset={status.home_offset if status.home_offset is not None else 'n/a'} "
                f"coast_distance={status.coast_distance if status.coast_distance is not None else 'n/a'} "
                f"homing_duty={status.homing_duty if status.homing_duty is not None else 'n/a'}"
            )

        elif args.command == "set-strike-param":
            param_map = {
                "home-offset": STRIKE_PARAM_HOME_OFFSET,
                "coast-distance": STRIKE_PARAM_COAST_DISTANCE,
                "homing-duty": STRIKE_PARAM_HOMING_DUTY,
            }
            print(format_status(client.set_strike_param(args.address, param_map[args.param], args.value)))
            status = client.query_strike(args.address)
            print(
                f"strike params: home_offset={status.home_offset if status.home_offset is not None else 'n/a'} "
                f"coast_distance={status.coast_distance if status.coast_distance is not None else 'n/a'} "
                f"homing_duty={status.homing_duty if status.homing_duty is not None else 'n/a'}"
            )

        elif args.command == "measure-strike-timing":
            if args.repeats < 1:
                raise RingError("--repeats must be at least 1")
            if args.poll_ms < 0 or args.idle_timeout_ms < 1 or args.strike_timeout_ms < 1 or args.pause_ms < 0:
                raise RingError("poll/timeout/pause values must be non-negative, and timeouts must be at least 1 ms")
            if args.no_plot and args.plot_out:
                raise RingError("--plot-out cannot be used together with --no-plot")
            if args.sweep_param != "duty" and args.strike_duty == 0:
                raise RingError("--strike-duty must be non-zero when sweeping non-duty parameters")

            sweep_values = resolve_measurement_sweep_values(args)
            sweep_label = measurement_sweep_label(args.sweep_param)
            total_samples = len(sweep_values) * args.repeats
            samples: list[StrikeTimingSample] = []
            completed = 0

            print(
                f"Collecting strike timing for addr={args.address} "
                f"{sweep_label}s={sweep_values} repeats={args.repeats} "
                f"strike_duty={args.strike_duty if args.sweep_param != 'duty' else 'swept'}"
            )

            for sweep_value in sweep_values:
                for repeat_index in range(1, args.repeats + 1):
                    completed += 1
                    print(f"[{completed}/{total_samples}] {sweep_label}={sweep_value} repeat={repeat_index}")
                    sample = collect_strike_timing_sample(
                        client=client,
                        address=args.address,
                        sweep_param=args.sweep_param,
                        sweep_value=sweep_value,
                        strike_duty=args.strike_duty,
                        repeat_index=repeat_index,
                        poll_ms=args.poll_ms,
                        idle_timeout_ms=args.idle_timeout_ms,
                        strike_timeout_ms=args.strike_timeout_ms,
                    )
                    samples.append(sample)
                    coast_text = "n/a" if sample.coast_ms is None else str(sample.coast_ms)
                    rebound_text = "n/a" if sample.rebound_ms is None else str(sample.rebound_ms)
                    retrigger_ready_text = (
                        "n/a" if sample.retrigger_ready_ms is None else str(sample.retrigger_ready_ms)
                    )
                    ready_text = "n/a" if sample.ready_ms is None else str(sample.ready_ms)
                    velocity_text = "n/a" if sample.estimated_strike_velocity_dps is None else str(sample.estimated_strike_velocity_dps)
                    print(
                        f"  seq={sample.sequence} {sweep_label}={sample.sweep_value} "
                        f"requested_duty={sample.requested_duty} applied_duty={sample.applied_duty} "
                        f"home_offset={sample.configured_home_offset if sample.configured_home_offset is not None else 'n/a'} "
                        f"coast_ms={coast_text} rebound_ms={rebound_text} "
                        f"retrigger_ready_ms={retrigger_ready_text} ready_ms={ready_text} "
                        f"strike_vel_dps={velocity_text} "
                        f"rebound_timeout={int(sample.rebound_timeout)} retriggered={int(sample.retriggered)}"
                    )
                    if args.pause_ms > 0:
                        time.sleep(args.pause_ms / 1000.0)

            print_strike_timing_summary(samples)

            if args.csv:
                write_strike_timing_csv(samples, args.csv)
                print(f"CSV saved to {args.csv}")

            if not args.no_plot:
                plot_strike_timing_samples(
                    samples=samples,
                    title=args.title or f"Strike timing sweep addr {args.address} vs {sweep_label}",
                    plot_path=args.plot_out,
                    show_plot=not args.no_show,
                )
                if args.plot_out:
                    print(f"Plot saved to {args.plot_out}")

        elif args.command == "measure-strike-calibration-grid":
            if args.repeats < 1:
                raise RingError("--repeats must be at least 1")
            if args.poll_ms < 0 or args.idle_timeout_ms < 1 or args.strike_timeout_ms < 1 or args.pause_ms < 0:
                raise RingError("poll/timeout/pause values must be non-negative, and timeouts must be at least 1 ms")

            home_offsets = list(args.home_offsets) if args.home_offsets else build_sweep_values(
                args.home_offset_start,
                args.home_offset_stop,
                args.home_offset_step,
            )
            duties = list(args.duties) if args.duties else build_sweep_values(
                args.duty_start,
                args.duty_stop,
                args.duty_step,
            )
            if any(duty == 0 for duty in duties):
                raise RingError("duty sweep must not include 0 because STRIKE 0 is ignored")

            total_samples = len(home_offsets) * len(duties) * args.repeats
            samples: list[StrikeTimingSample] = []
            completed = 0

            print(
                f"Collecting strike calibration grid for addr={args.address} "
                f"home_offsets={home_offsets} duties={duties} repeats={args.repeats}"
            )

            for home_offset in home_offsets:
                status = ensure_home_offset(
                    client=client,
                    address=args.address,
                    home_offset=home_offset,
                    poll_ms=args.poll_ms,
                    idle_timeout_ms=args.idle_timeout_ms,
                )
                print(
                    f"Home offset {home_offset} ready "
                    f"(drum_pos={status.drum_position} home_pos={status.home_position})"
                )

                for duty in duties:
                    for repeat_index in range(1, args.repeats + 1):
                        completed += 1
                        print(
                            f"[{completed}/{total_samples}] "
                            f"home_offset={home_offset} duty={duty} repeat={repeat_index}"
                        )
                        sample = collect_strike_timing_sample(
                            client=client,
                            address=args.address,
                            sweep_param="duty",
                            sweep_value=duty,
                            strike_duty=0,
                            repeat_index=repeat_index,
                            poll_ms=args.poll_ms,
                            idle_timeout_ms=args.idle_timeout_ms,
                            strike_timeout_ms=args.strike_timeout_ms,
                        )
                        samples.append(sample)
                        coast_text = "n/a" if sample.coast_ms is None else str(sample.coast_ms)
                        rebound_text = "n/a" if sample.rebound_ms is None else str(sample.rebound_ms)
                        retrigger_ready_text = (
                            "n/a" if sample.retrigger_ready_ms is None else str(sample.retrigger_ready_ms)
                        )
                        ready_text = "n/a" if sample.ready_ms is None else str(sample.ready_ms)
                        velocity_text = (
                            "n/a" if sample.estimated_strike_velocity_dps is None
                            else str(sample.estimated_strike_velocity_dps)
                        )
                        print(
                            f"  seq={sample.sequence} home_offset={sample.configured_home_offset} "
                            f"requested_duty={sample.requested_duty} applied_duty={sample.applied_duty} "
                            f"coast_ms={coast_text} rebound_ms={rebound_text} "
                            f"retrigger_ready_ms={retrigger_ready_text} ready_ms={ready_text} "
                            f"strike_vel_dps={velocity_text}"
                        )
                        if args.pause_ms > 0:
                            time.sleep(args.pause_ms / 1000.0)

            if args.csv:
                write_strike_timing_csv(samples, args.csv)
                print(f"Raw CSV saved to {args.csv}")

            profile = build_strike_calibration_grid_profile(
                duty_samples=samples,
                address=args.address,
                midi_note=args.midi_note,
                note_name=args.name,
                preferred_home_offset=args.preferred_home_offset,
            )
            with open(args.out, "w", encoding="ascii") as json_file:
                json.dump(profile, json_file, indent=2)
                json_file.write("\n")

            print(f"Calibration grid profile saved to {args.out}")
            print(
                f"suggested_home_offset={profile['suggested_home_offset']} "
                f"profile_count={len(profile['home_offset_profiles'])} "
                f"scheduler_ready={int(profile['scheduler_ready'])}"
            )
            for summary in profile["home_offset_tradeoff"]:
                print(
                    f"  home_offset={summary['home_offset']} "
                    f"repeat_min_ms={summary['repeat_min_ms']} "
                    f"lead_min_ms={summary['lead_min_ms']} "
                    f"strength_max_dps={summary['strength_max_dps']}"
                )

        elif args.command == "broadcast":
            if client.device_count is not None and len(args.duties) != client.device_count:
                print(f"WARNING: sending {len(args.duties)} duties but {client.device_count} devices enumerated")
            client.broadcast_duty(args.duties)
            print("broadcast ok")

        elif args.command == "monitor":
            interval = 1.0 / args.hz if args.hz > 0 else 0.1
            print(f"Monitoring device {args.address} at {args.hz:.1f} Hz (Ctrl-C to stop)")
            while True:
                try:
                    print(format_status(client.query_status(args.address)))
                except RingTimeout:
                    print("  (timeout)")
                except RingCRCError as exc:
                    print(f"  (crc error: {exc})")
                time.sleep(interval)

        return 0

    except RingError as exc:
        print(f"ERROR: {exc}")
        return 1
    except serial.SerialException as exc:
        print(f"Serial error: {exc}")
        return 1
    except KeyboardInterrupt:
        print("\nCancelled.")
        return 130
    finally:
        client.close()


if __name__ == "__main__":
    sys.exit(main())
