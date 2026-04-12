#!/usr/bin/env python3
"""
Strike timing sweep and plotting helpers built on top of ring_bus.
"""

from __future__ import annotations

import argparse
import csv
import dataclasses
import datetime
import statistics
import time
from typing import Optional

from ring_bus import (
    REPLY_MODE_ACK,
    STRIKE_PARAM_HOME_OFFSET,
    CommandAck,
    RingClientV2,
    RingError,
    RingTimeout,
    StrikeStatus,
)


@dataclasses.dataclass
class StrikeTimingSample:
    sweep_param: str
    sweep_value: int
    requested_current_ma: int
    applied_current_ma: int
    configured_home_offset: Optional[int]
    repeat_index: int
    sequence: int
    coast_ms: Optional[int]
    rebound_ms: Optional[int]
    retrigger_ready_ms: Optional[int]
    ready_ms: Optional[int]
    estimated_strike_velocity_dps: Optional[int]
    retriggered: bool
    rebound_timeout: bool
    drum_position: int
    home_position: int


@dataclasses.dataclass
class StrikeTimingAggregate:
    sweep_param: str
    sweep_value: int
    sample_count: int
    requested_current_ma: Optional[float]
    applied_current_ma: Optional[float]
    configured_home_offset: Optional[int]
    coast_ms: Optional[float]
    rebound_ms: Optional[float]
    retrigger_ready_ms: Optional[float]
    ready_ms: Optional[float]
    estimated_strike_velocity_dps: Optional[float]
    retriggered_count: int
    rebound_timeout_count: int
    drum_position: Optional[int]
    home_position: Optional[int]


def parse_optional_int(value: str) -> Optional[int]:
    stripped = value.strip()
    if stripped == "":
        return None
    return int(stripped)


def parse_bool_flag(value: str) -> bool:
    return value.strip().lower() not in ("", "0", "false", "no")


def median_or_none(values: list[Optional[int]]) -> Optional[float]:
    filtered = [value for value in values if value is not None]
    if not filtered:
        return None
    return float(statistics.median(filtered))


def rounded_or_none(value: Optional[float], digits: int = 1) -> Optional[float]:
    if value is None:
        return None
    return round(value, digits)


def int_or_none(value: Optional[float]) -> Optional[int]:
    if value is None:
        return None
    return int(round(value))


def build_sweep_values(start: int, stop: int, step: int) -> list[int]:
    values: list[int] = []

    if step == 0:
        raise RingError("sweep step must be non-zero")

    if start != stop and (stop - start) * step < 0:
        raise RingError("sweep step sign does not move start toward stop")

    current = start
    if step > 0:
        while current <= stop:
            values.append(current)
            current += step
    else:
        while current >= stop:
            values.append(current)
            current += step

    if not values:
        raise RingError("no sweep values generated")

    return values


def measurement_sweep_label(sweep_param: str) -> str:
    if sweep_param == "home-offset":
        return "home_offset"
    return "current_ma"


def measurement_sweep_axis_label(sweep_param: str) -> str:
    if sweep_param == "home-offset":
        return "Home offset above drum (encoder counts)"
    return "Requested strike current (mA)"


def resolve_measurement_sweep_values(args: argparse.Namespace) -> list[int]:
    explicit_currents = getattr(args, "currents", None)

    if args.values and explicit_currents:
        raise RingError("--values and --currents cannot be used together")
    if args.sweep_param != "current" and explicit_currents:
        raise RingError("--currents can only be used when --sweep-param=current")

    values = list(args.values) if args.values else list(explicit_currents) if explicit_currents else build_sweep_values(
        args.start,
        args.stop,
        args.step,
    )

    if args.sweep_param == "current" and any(value == 0 for value in values):
        raise RingError("current sweep must not include 0 because STRIKE 0 is ignored")

    return values


def wait_for_strike_idle(
    client: RingClientV2,
    address: int,
    timeout_ms: int,
    poll_ms: int,
) -> StrikeStatus:
    deadline = time.monotonic() + timeout_ms / 1000.0
    last_status: Optional[StrikeStatus] = None
    poll_s = max(poll_ms, 0) / 1000.0

    while True:
        last_status = client.query_strike(address)
        if last_status.state == 0 and not last_status.active:
            return last_status
        if time.monotonic() >= deadline:
            break
        if poll_s > 0:
            time.sleep(poll_s)

    state_name = last_status.state_name if last_status is not None else "UNKNOWN"
    raise RingTimeout(f"timed out waiting for strike idle (last strike state: {state_name})")


def collect_strike_timing_sample(
    client: RingClientV2,
    address: int,
    sweep_param: str,
    sweep_value: int,
    strike_current_ma: int,
    repeat_index: int,
    poll_ms: int,
    idle_timeout_ms: int,
    strike_timeout_ms: int,
) -> StrikeTimingSample:
    baseline = wait_for_strike_idle(client, address, idle_timeout_ms, poll_ms)
    poll_s = max(poll_ms, 0) / 1000.0

    if not baseline.homed:
        raise RingError("strike actuator is not homed; run strike-home first")

    if sweep_param == "home-offset":
        response = client.set_strike_param(
            address,
            STRIKE_PARAM_HOME_OFFSET,
            sweep_value,
            reply_mode=REPLY_MODE_ACK,
        )
        if not isinstance(response, CommandAck) or not response.accepted:
            result_text = "no response" if response is None else response.result_name
            raise RingError(f"set home-offset was not accepted: {result_text}")
        baseline = wait_for_strike_idle(client, address, idle_timeout_ms, poll_ms)
        if baseline.home_offset is not None and baseline.home_offset != sweep_value:
            raise RingError(
                f"home_offset verification failed: expected {sweep_value}, got {baseline.home_offset}"
            )
        requested_current_ma = strike_current_ma
    else:
        requested_current_ma = sweep_value

    prev_sequence = baseline.sequence
    response = client.strike(address, requested_current_ma, reply_mode=REPLY_MODE_ACK)
    if not isinstance(response, CommandAck):
        raise RingError("strike did not return an acknowledgement")
    if not response.accepted:
        raise RingError(f"strike was not accepted: {response.result_name}")
    accepted_sequence = response.detail
    if accepted_sequence == prev_sequence:
        raise RingError(
            f"strike ack did not advance the sequence (still {accepted_sequence}); "
            "the command path is inconsistent with strike telemetry"
        )

    deadline = time.monotonic() + strike_timeout_ms / 1000.0
    final_status: Optional[StrikeStatus] = None

    # Don't hammer QUERY_STRIKE immediately after launch. The timing fields are
    # latched on-device, so slower polling does not reduce measurement accuracy,
    # but it does reduce protocol load during fast commutation and catch.
    if poll_s > 0:
        time.sleep(poll_s)

    while time.monotonic() < deadline:
        status = client.query_strike(address)
        if status.sequence == accepted_sequence and not status.active and status.ready_valid:
            return StrikeTimingSample(
                sweep_param=sweep_param,
                sweep_value=sweep_value,
                requested_current_ma=requested_current_ma,
                applied_current_ma=status.last_current_ma,
                configured_home_offset=status.home_offset,
                repeat_index=repeat_index,
                sequence=status.sequence,
                coast_ms=status.trigger_to_coast_ms if status.coast_valid else None,
                rebound_ms=status.trigger_to_rebound_ms if status.rebound_valid else None,
                retrigger_ready_ms=(
                    status.trigger_to_retrigger_ready_ms
                    if status.retrigger_ready_valid
                    else None
                ),
                ready_ms=status.trigger_to_ready_ms if status.ready_valid else None,
                estimated_strike_velocity_dps=(
                    status.estimated_strike_velocity_dps if status.velocity_valid else None
                ),
                retriggered=status.retriggered,
                rebound_timeout=status.rebound_timeout,
                drum_position=status.drum_position,
                home_position=status.home_position,
            )

        if status.sequence > accepted_sequence:
            raise RingError(
                f"strike sequence changed from {accepted_sequence} to {status.sequence} "
                "while waiting for completion"
            )

        final_status = status
        if poll_s > 0:
            time.sleep(poll_s)

    if final_status is None:
        raise RingTimeout(
            f"timed out waiting for strike sequence {accepted_sequence} to appear in strike telemetry"
        )

    raise RingTimeout(
        f"timed out waiting for strike sequence {accepted_sequence} to finish and report ready timing"
    )


def load_strike_timing_csv(path: str) -> list[StrikeTimingSample]:
    samples: list[StrikeTimingSample] = []

    with open(path, newline="") as csv_file:
        reader = csv.DictReader(csv_file)
        for row in reader:
            samples.append(
                StrikeTimingSample(
                    sweep_param=row["sweep_param"],
                    sweep_value=int(row["sweep_value"]),
                    requested_current_ma=int(row["requested_current_ma"]),
                    applied_current_ma=int(row["applied_current_ma"]),
                    configured_home_offset=parse_optional_int(row.get("configured_home_offset", "")),
                    repeat_index=int(row["repeat_index"]),
                    sequence=int(row["sequence"]),
                    coast_ms=parse_optional_int(row.get("coast_ms", "")),
                    rebound_ms=parse_optional_int(row.get("rebound_ms", "")),
                    retrigger_ready_ms=parse_optional_int(row.get("retrigger_ready_ms", "")),
                    ready_ms=parse_optional_int(row.get("ready_ms", "")),
                    estimated_strike_velocity_dps=parse_optional_int(
                        row.get("estimated_strike_velocity_dps", "")
                    ),
                    retriggered=parse_bool_flag(row.get("retriggered", "0")),
                    rebound_timeout=parse_bool_flag(row.get("rebound_timeout", "0")),
                    drum_position=int(row["drum_position"]),
                    home_position=int(row["home_position"]),
                )
            )

    if not samples:
        raise RingError(f"no strike timing samples found in {path}")

    return samples


def aggregate_strike_timing_samples(samples: list[StrikeTimingSample]) -> list[StrikeTimingAggregate]:
    grouped: dict[int, list[StrikeTimingSample]] = {}
    sweep_param = samples[0].sweep_param if samples else ""

    for sample in samples:
        if sample.sweep_param != sweep_param:
            raise RingError("CSV contains mixed sweep_param values")
        grouped.setdefault(sample.sweep_value, []).append(sample)

    aggregates: list[StrikeTimingAggregate] = []
    for sweep_value, group in grouped.items():
        aggregates.append(
            StrikeTimingAggregate(
                sweep_param=sweep_param,
                sweep_value=sweep_value,
                sample_count=len(group),
                requested_current_ma=median_or_none([sample.requested_current_ma for sample in group]),
                applied_current_ma=median_or_none([sample.applied_current_ma for sample in group]),
                configured_home_offset=int_or_none(
                    median_or_none([sample.configured_home_offset for sample in group])
                ),
                coast_ms=median_or_none([sample.coast_ms for sample in group]),
                rebound_ms=median_or_none([sample.rebound_ms for sample in group]),
                retrigger_ready_ms=median_or_none([sample.retrigger_ready_ms for sample in group]),
                ready_ms=median_or_none([sample.ready_ms for sample in group]),
                estimated_strike_velocity_dps=median_or_none(
                    [sample.estimated_strike_velocity_dps for sample in group]
                ),
                retriggered_count=sum(1 for sample in group if sample.retriggered),
                rebound_timeout_count=sum(1 for sample in group if sample.rebound_timeout),
                drum_position=int_or_none(median_or_none([sample.drum_position for sample in group])),
                home_position=int_or_none(median_or_none([sample.home_position for sample in group])),
            )
        )

    return sorted(aggregates, key=lambda aggregate: aggregate.sweep_value)


def choose_home_offset_candidate(
    aggregates: list[StrikeTimingAggregate],
    preferred_home_offset: Optional[int] = None,
) -> StrikeTimingAggregate:
    if not aggregates:
        raise RingError("home-offset sweep is empty")

    if preferred_home_offset is not None:
        for aggregate in aggregates:
            if aggregate.sweep_value == preferred_home_offset:
                return aggregate
        raise RingError(f"preferred home offset {preferred_home_offset} is not present in the sweep")

    usable = [
        aggregate
        for aggregate in aggregates
        if aggregate.retrigger_ready_ms is not None or aggregate.ready_ms is not None
    ]
    if not usable:
        raise RingError("home-offset sweep does not contain any usable timing data")

    return min(
        usable,
        key=lambda aggregate: (
            aggregate.retrigger_ready_ms is None,
            aggregate.retrigger_ready_ms if aggregate.retrigger_ready_ms is not None else float("inf"),
            aggregate.ready_ms is None,
            aggregate.ready_ms if aggregate.ready_ms is not None else float("inf"),
            -(
                aggregate.estimated_strike_velocity_dps
                if aggregate.estimated_strike_velocity_dps is not None
                else float("-inf")
            ),
            aggregate.sweep_value,
        ),
    )


def interpolate_curve(x: float, points: list[tuple[float, float]]) -> float:
    if not points:
        raise RingError("interpolation requires at least one point")
    if len(points) == 1:
        return points[0][1]
    if x <= points[0][0]:
        return points[0][1]

    for index in range(1, len(points)):
        x0, y0 = points[index - 1]
        x1, y1 = points[index]
        if x <= x1:
            if x1 == x0:
                return y1
            blend = (x - x0) / (x1 - x0)
            return y0 + (y1 - y0) * blend

    return points[-1][1]


def build_lookup_curve_points(
    current_aggregates: list[StrikeTimingAggregate],
) -> list[dict[str, float]]:
    usable = [aggregate for aggregate in current_aggregates if aggregate.applied_current_ma is not None]
    if not usable:
        raise RingError("current sweep does not contain any applied strike-current data")

    curve_points: list[dict[str, float]] = []
    last_strength: Optional[float] = None

    for aggregate in sorted(
        usable,
        key=lambda item: item.applied_current_ma if item.applied_current_ma is not None else item.sweep_value,
    ):
        current_ma = float(
            aggregate.applied_current_ma if aggregate.applied_current_ma is not None else aggregate.sweep_value
        )
        raw_strength = aggregate.estimated_strike_velocity_dps
        strength = float(raw_strength if raw_strength is not None else current_ma)
        if last_strength is not None and strength < last_strength:
            strength = last_strength

        point = {
            "current_ma": current_ma,
            "strength": strength,
            "rebound_ms": float(aggregate.rebound_ms if aggregate.rebound_ms is not None else 0.0),
            "retrigger_ready_ms": float(
                aggregate.retrigger_ready_ms
                if aggregate.retrigger_ready_ms is not None
                else aggregate.ready_ms
                if aggregate.ready_ms is not None
                else 0.0
            ),
            "ready_ms": float(aggregate.ready_ms if aggregate.ready_ms is not None else 0.0),
        }

        if curve_points and abs(point["strength"] - curve_points[-1]["strength"]) < 1e-9:
            curve_points[-1] = point
        else:
            curve_points.append(point)
        last_strength = point["strength"]

    return curve_points


def build_midi_velocity_lookup(
    current_aggregates: list[StrikeTimingAggregate],
) -> list[dict[str, int]]:
    curve_points = build_lookup_curve_points(current_aggregates)
    strength_points = [(point["strength"], point["current_ma"]) for point in curve_points]
    current_to_rebound = [(point["current_ma"], point["rebound_ms"]) for point in curve_points]
    current_to_retrigger_ready = [(point["current_ma"], point["retrigger_ready_ms"]) for point in curve_points]
    current_to_ready = [(point["current_ma"], point["ready_ms"]) for point in curve_points]

    min_strength = curve_points[0]["strength"]
    max_strength = curve_points[-1]["strength"]
    lookup: list[dict[str, int]] = []

    for midi_velocity in range(1, 128):
        if max_strength == min_strength:
            target_strength = min_strength
        else:
            blend = (midi_velocity - 1) / 126.0
            target_strength = min_strength + (max_strength - min_strength) * blend

        current_ma = interpolate_curve(target_strength, strength_points)
        rebound_ms = interpolate_curve(current_ma, current_to_rebound)
        retrigger_ready_ms = interpolate_curve(current_ma, current_to_retrigger_ready)
        ready_ms = interpolate_curve(current_ma, current_to_ready)

        lookup.append(
            {
                "midi_velocity": midi_velocity,
                "target_strength_dps": int(round(target_strength)),
                "current_ma": int(round(current_ma)),
                "lead_ms": int(round(rebound_ms)),
                "repeat_ms": int(round(retrigger_ready_ms)),
                "settle_ms": int(round(ready_ms)),
            }
        )

    return lookup


def aggregate_to_profile_dict(aggregate: StrikeTimingAggregate) -> dict[str, object]:
    return {
        "sweep_value": aggregate.sweep_value,
        "sample_count": aggregate.sample_count,
        "requested_current_ma": rounded_or_none(aggregate.requested_current_ma),
        "applied_current_ma": rounded_or_none(aggregate.applied_current_ma),
        "configured_home_offset": aggregate.configured_home_offset,
        "coast_ms": rounded_or_none(aggregate.coast_ms),
        "rebound_ms": rounded_or_none(aggregate.rebound_ms),
        "retrigger_ready_ms": rounded_or_none(aggregate.retrigger_ready_ms),
        "ready_ms": rounded_or_none(aggregate.ready_ms),
        "estimated_strike_velocity_dps": rounded_or_none(aggregate.estimated_strike_velocity_dps),
        "retriggered_count": aggregate.retriggered_count,
        "rebound_timeout_count": aggregate.rebound_timeout_count,
        "drum_position": aggregate.drum_position,
        "home_position": aggregate.home_position,
    }


def build_strike_calibration_profile(
    current_samples: list[StrikeTimingSample],
    home_offset_samples: Optional[list[StrikeTimingSample]] = None,
    address: Optional[int] = None,
    midi_note: Optional[int] = None,
    note_name: Optional[str] = None,
    preferred_home_offset: Optional[int] = None,
) -> dict[str, object]:
    if not current_samples:
        raise RingError("current sweep is empty")
    if current_samples[0].sweep_param != "current":
        raise RingError("current sweep CSV must come from --sweep-param=current")

    current_aggregates = aggregate_strike_timing_samples(current_samples)
    current_curve_home_offset = current_aggregates[0].configured_home_offset
    if any(aggregate.configured_home_offset != current_curve_home_offset for aggregate in current_aggregates):
        raise RingError("current sweep CSV mixes multiple configured home offsets")

    suggested_home_offset = current_curve_home_offset
    home_offset_candidates: list[dict[str, object]] = []
    selection_reason = "current sweep configured_home_offset"

    if home_offset_samples:
        if home_offset_samples[0].sweep_param != "home-offset":
            raise RingError("home-offset sweep CSV must come from --sweep-param=home-offset")
        home_offset_aggregates = aggregate_strike_timing_samples(home_offset_samples)
        chosen_home_offset = choose_home_offset_candidate(
            home_offset_aggregates,
            preferred_home_offset=preferred_home_offset,
        )
        suggested_home_offset = chosen_home_offset.sweep_value
        home_offset_candidates = [
            aggregate_to_profile_dict(aggregate) for aggregate in home_offset_aggregates
        ]
        selection_reason = (
            "minimum retrigger_ready_ms, then minimum ready_ms, then maximum strike velocity"
            if preferred_home_offset is None
            else "user-selected preferred_home_offset"
        )

    warnings: list[str] = []
    scheduler_ready = True
    if (
        suggested_home_offset is not None
        and current_curve_home_offset is not None
        and suggested_home_offset != current_curve_home_offset
    ):
        scheduler_ready = False
        warnings.append(
            "Suggested home_offset from the home-offset sweep does not match the home_offset used for the current sweep. "
            "Retake the current sweep at the suggested home_offset before using this profile for production scheduling."
        )

    identity: dict[str, object] = {}
    if address is not None:
        identity["address"] = address
    if midi_note is not None:
        identity["midi_note"] = midi_note
    if note_name:
        identity["name"] = note_name

    return {
        "schema_version": 2,
        "generated_at": datetime.datetime.now(datetime.timezone.utc).isoformat().replace("+00:00", "Z"),
        "scheduler_ready": scheduler_ready,
        "warnings": warnings,
        "identity": identity,
        "lookup_basis": {
            "lead_time_field": "rebound_ms",
            "repeat_time_field": "retrigger_ready_ms",
            "settle_time_field": "ready_ms",
            "velocity_proxy_field": "estimated_strike_velocity_dps",
            "impact_time_note": "rebound_ms is an inferred impact proxy from rebound detection, not a contact sensor",
        },
        "lookup_home_offset": current_curve_home_offset,
        "suggested_home_offset": suggested_home_offset,
        "home_offset_selection_reason": selection_reason,
        "current_curve": [aggregate_to_profile_dict(aggregate) for aggregate in current_aggregates],
        "home_offset_candidates": home_offset_candidates,
        "midi_velocity_lookup": build_midi_velocity_lookup(current_aggregates),
    }


def ensure_home_offset(
    client: RingClientV2,
    address: int,
    home_offset: int,
    poll_ms: int,
    idle_timeout_ms: int,
) -> StrikeStatus:
    baseline = wait_for_strike_idle(client, address, idle_timeout_ms, poll_ms)

    if not baseline.homed:
        raise RingError("strike actuator is not homed; run strike-home first")

    if baseline.home_offset == home_offset:
        return baseline

    response = client.set_strike_param(
        address,
        STRIKE_PARAM_HOME_OFFSET,
        home_offset,
        reply_mode=REPLY_MODE_ACK,
    )
    if not isinstance(response, CommandAck) or not response.accepted:
        result_text = "no response" if response is None else response.result_name
        raise RingError(f"set home-offset was not accepted: {result_text}")

    status = wait_for_strike_idle(client, address, idle_timeout_ms, poll_ms)
    if status.home_offset is not None and status.home_offset != home_offset:
        raise RingError(
            f"home_offset verification failed: expected {home_offset}, got {status.home_offset}"
        )
    return status


def build_home_offset_profile(
    home_offset: int,
    current_aggregates: list[StrikeTimingAggregate],
) -> dict[str, object]:
    if not current_aggregates:
        raise RingError(f"home_offset {home_offset} has no current aggregate data")

    lookup = build_midi_velocity_lookup(current_aggregates)
    strength_values = [entry["target_strength_dps"] for entry in lookup]
    current_values = [entry["current_ma"] for entry in lookup]
    lead_values = [entry["lead_ms"] for entry in lookup]
    repeat_values = [entry["repeat_ms"] for entry in lookup]
    settle_values = [entry["settle_ms"] for entry in lookup]

    return {
        "home_offset": home_offset,
        "summary": {
            "sample_points": len(current_aggregates),
            "strength_min_dps": min(strength_values),
            "strength_max_dps": max(strength_values),
            "current_ma_min": min(current_values),
            "current_ma_max": max(current_values),
            "lead_min_ms": min(lead_values),
            "lead_max_ms": max(lead_values),
            "repeat_min_ms": min(repeat_values),
            "repeat_max_ms": max(repeat_values),
            "settle_min_ms": min(settle_values),
            "settle_max_ms": max(settle_values),
            "drum_position": current_aggregates[0].drum_position,
            "home_position": current_aggregates[0].home_position,
        },
        "current_curve": [aggregate_to_profile_dict(aggregate) for aggregate in current_aggregates],
        "midi_velocity_lookup": lookup,
    }


def choose_home_offset_profile(
    profiles: list[dict[str, object]],
    preferred_home_offset: Optional[int] = None,
) -> dict[str, object]:
    if not profiles:
        raise RingError("no home_offset profiles are available")

    if preferred_home_offset is not None:
        for profile in profiles:
            if profile["home_offset"] == preferred_home_offset:
                return profile
        raise RingError(f"preferred home offset {preferred_home_offset} is not present in the profile")

    return min(
        profiles,
        key=lambda profile: (
            profile["summary"]["repeat_min_ms"],
            profile["summary"]["settle_min_ms"],
            -profile["summary"]["strength_max_dps"],
            profile["home_offset"],
        ),
    )


def build_strike_calibration_grid_profile(
    current_samples: list[StrikeTimingSample],
    address: Optional[int] = None,
    midi_note: Optional[int] = None,
    note_name: Optional[str] = None,
    preferred_home_offset: Optional[int] = None,
) -> dict[str, object]:
    if not current_samples:
        raise RingError("current sweep is empty")

    if current_samples[0].sweep_param != "current":
        raise RingError("calibration grid requires current sweep samples")

    grouped: dict[int, list[StrikeTimingSample]] = {}
    for sample in current_samples:
        if sample.sweep_param != "current":
            raise RingError("calibration grid CSV contains non-current sweep samples")
        if sample.configured_home_offset is None:
            raise RingError("calibration grid CSV is missing configured_home_offset values")
        grouped.setdefault(sample.configured_home_offset, []).append(sample)

    profiles = [
        build_home_offset_profile(home_offset, aggregate_strike_timing_samples(group))
        for home_offset, group in sorted(grouped.items())
    ]
    suggested_profile = choose_home_offset_profile(
        profiles,
        preferred_home_offset=preferred_home_offset,
    )

    identity: dict[str, object] = {}
    if address is not None:
        identity["address"] = address
    if midi_note is not None:
        identity["midi_note"] = midi_note
    if note_name:
        identity["name"] = note_name

    return {
        "schema_version": 3,
        "generated_at": datetime.datetime.now(datetime.timezone.utc).isoformat().replace("+00:00", "Z"),
        "scheduler_ready": True,
        "warnings": [],
        "identity": identity,
        "lookup_basis": {
            "lead_time_field": "rebound_ms",
            "repeat_time_field": "retrigger_ready_ms",
            "settle_time_field": "ready_ms",
            "velocity_proxy_field": "estimated_strike_velocity_dps",
            "impact_time_note": "rebound_ms is an inferred impact proxy from rebound detection, not a contact sensor",
        },
        "suggested_home_offset": suggested_profile["home_offset"],
        "home_offset_selection_reason": (
            "minimum repeat_min_ms, then minimum settle_min_ms, then maximum strength_max_dps"
            if preferred_home_offset is None
            else "user-selected preferred_home_offset"
        ),
        "home_offset_tradeoff": [
            {
                "home_offset": profile["home_offset"],
                **profile["summary"],
            }
            for profile in profiles
        ],
        "home_offset_profiles": profiles,
    }


def write_strike_timing_csv(samples: list[StrikeTimingSample], path: str) -> None:
    fieldnames = [
        "sweep_param",
        "sweep_value",
        "requested_current_ma",
        "applied_current_ma",
        "configured_home_offset",
        "repeat_index",
        "sequence",
        "coast_ms",
        "rebound_ms",
        "retrigger_ready_ms",
        "ready_ms",
        "estimated_strike_velocity_dps",
        "retriggered",
        "rebound_timeout",
        "drum_position",
        "home_position",
    ]

    with open(path, "w", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        for sample in samples:
            writer.writerow(
                {
                    "sweep_param": sample.sweep_param,
                    "sweep_value": sample.sweep_value,
                    "requested_current_ma": sample.requested_current_ma,
                    "applied_current_ma": sample.applied_current_ma,
                    "configured_home_offset": (
                        "" if sample.configured_home_offset is None else sample.configured_home_offset
                    ),
                    "repeat_index": sample.repeat_index,
                    "sequence": sample.sequence,
                    "coast_ms": "" if sample.coast_ms is None else sample.coast_ms,
                    "rebound_ms": "" if sample.rebound_ms is None else sample.rebound_ms,
                    "retrigger_ready_ms": (
                        "" if sample.retrigger_ready_ms is None else sample.retrigger_ready_ms
                    ),
                    "ready_ms": "" if sample.ready_ms is None else sample.ready_ms,
                    "estimated_strike_velocity_dps": (
                        "" if sample.estimated_strike_velocity_dps is None else sample.estimated_strike_velocity_dps
                    ),
                    "retriggered": int(sample.retriggered),
                    "rebound_timeout": int(sample.rebound_timeout),
                    "drum_position": sample.drum_position,
                    "home_position": sample.home_position,
                }
            )


def mean_or_none(values: list[Optional[int]]) -> Optional[float]:
    filtered = [value for value in values if value is not None]
    if not filtered:
        return None
    return sum(filtered) / len(filtered)


def print_strike_timing_summary(samples: list[StrikeTimingSample]) -> None:
    grouped: dict[int, list[StrikeTimingSample]] = {}
    sweep_param = samples[0].sweep_param if samples else "current"
    value_label = measurement_sweep_label(sweep_param)

    for sample in samples:
        grouped.setdefault(sample.sweep_value, []).append(sample)

    print("Strike timing summary:")
    for sweep_value, sweep_samples in grouped.items():
        coast_mean = mean_or_none([sample.coast_ms for sample in sweep_samples])
        rebound_mean = mean_or_none([sample.rebound_ms for sample in sweep_samples])
        retrigger_ready_mean = mean_or_none([sample.retrigger_ready_ms for sample in sweep_samples])
        ready_mean = mean_or_none([sample.ready_ms for sample in sweep_samples])
        velocity_mean = mean_or_none([sample.estimated_strike_velocity_dps for sample in sweep_samples])
        timeout_count = sum(1 for sample in sweep_samples if sample.rebound_timeout)
        retrigger_count = sum(1 for sample in sweep_samples if sample.retriggered)
        current_mean = mean_or_none([sample.requested_current_ma for sample in sweep_samples])

        coast_text = "n/a" if coast_mean is None else f"{coast_mean:.1f}"
        rebound_text = "n/a" if rebound_mean is None else f"{rebound_mean:.1f}"
        retrigger_ready_text = "n/a" if retrigger_ready_mean is None else f"{retrigger_ready_mean:.1f}"
        ready_text = "n/a" if ready_mean is None else f"{ready_mean:.1f}"
        velocity_text = "n/a" if velocity_mean is None else f"{velocity_mean:.1f}"
        current_text = "n/a" if current_mean is None else f"{current_mean:.1f}"

        print(
            f"  {value_label}={sweep_value} n={len(sweep_samples)} "
            f"strike_current_ma_mean={current_text} "
            f"coast_mean_ms={coast_text} rebound_mean_ms={rebound_text} "
            f"retrigger_ready_mean_ms={retrigger_ready_text} "
            f"ready_mean_ms={ready_text} strike_vel_mean_dps={velocity_text} "
            f"rebound_timeouts={timeout_count} "
            f"retriggered={retrigger_count}"
        )


def plot_strike_timing_samples(
    samples: list[StrikeTimingSample],
    title: str,
    plot_path: Optional[str],
    show_plot: bool,
) -> None:
    try:
        if not show_plot:
            import matplotlib
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise RingError("matplotlib is required for strike timing plots. Install with: pip install matplotlib") from exc

    grouped: dict[int, list[StrikeTimingSample]] = {}
    sweep_param = samples[0].sweep_param if samples else "current"
    for sample in samples:
        grouped.setdefault(sample.sweep_value, []).append(sample)

    sweep_values = sorted(grouped.keys())
    time_series = [
        ("coast_ms", "Trigger to coast", "tab:blue"),
        ("rebound_ms", "Trigger to rebound", "tab:orange"),
        ("retrigger_ready_ms", "Trigger to retrigger-ready", "tab:purple"),
        ("ready_ms", "Trigger to ready", "tab:green"),
    ]
    velocity_series = ("estimated_strike_velocity_dps", "Strike velocity", "tab:red")

    fig, ax = plt.subplots(figsize=(9, 5))

    for attr_name, label, color in time_series:
        raw_x: list[int] = []
        raw_y: list[int] = []
        mean_x: list[int] = []
        mean_y: list[float] = []

        for sweep_value in sweep_values:
            values = [
                getattr(sample, attr_name)
                for sample in grouped[sweep_value]
                if getattr(sample, attr_name) is not None
            ]
            raw_x.extend([sweep_value] * len(values))
            raw_y.extend(values)
            if values:
                mean_x.append(sweep_value)
                mean_y.append(sum(values) / len(values))

        if raw_y:
            ax.scatter(raw_x, raw_y, s=24, alpha=0.35, color=color)
            ax.plot(mean_x, mean_y, marker="o", linewidth=2, label=label, color=color)

    velocity_attr_name, velocity_label, velocity_color = velocity_series
    velocity_raw_x: list[int] = []
    velocity_raw_y: list[int] = []
    velocity_mean_x: list[int] = []
    velocity_mean_y: list[float] = []

    for sweep_value in sweep_values:
        values = [
            getattr(sample, velocity_attr_name)
            for sample in grouped[sweep_value]
            if getattr(sample, velocity_attr_name) is not None
        ]
        velocity_raw_x.extend([sweep_value] * len(values))
        velocity_raw_y.extend(values)
        if values:
            velocity_mean_x.append(sweep_value)
            velocity_mean_y.append(sum(values) / len(values))

    velocity_ax = None
    if velocity_raw_y:
        velocity_ax = ax.twinx()
        velocity_ax.scatter(velocity_raw_x, velocity_raw_y, s=24, alpha=0.35, color=velocity_color)
        velocity_ax.plot(
            velocity_mean_x,
            velocity_mean_y,
            marker="s",
            linewidth=2,
            label=velocity_label,
            color=velocity_color,
        )
        velocity_ax.set_ylabel("Degrees per second", color=velocity_color)
        velocity_ax.tick_params(axis="y", labelcolor=velocity_color)

    ax.set_title(title)
    ax.set_xlabel(measurement_sweep_axis_label(sweep_param))
    ax.set_ylabel("Milliseconds")
    ax.grid(True, alpha=0.3)
    handles, labels = ax.get_legend_handles_labels()
    if velocity_ax is not None:
        velocity_handles, velocity_labels = velocity_ax.get_legend_handles_labels()
        handles += velocity_handles
        labels += velocity_labels
    ax.legend(handles, labels)
    fig.tight_layout()

    if plot_path:
        fig.savefig(plot_path, dpi=160)

    if show_plot:
        plt.show()
    else:
        plt.close(fig)


__all__ = [
    "StrikeTimingAggregate",
    "StrikeTimingSample",
    "aggregate_strike_timing_samples",
    "aggregate_to_profile_dict",
    "build_midi_velocity_lookup",
    "build_home_offset_profile",
    "build_strike_calibration_profile",
    "build_strike_calibration_grid_profile",
    "build_sweep_values",
    "choose_home_offset_candidate",
    "choose_home_offset_profile",
    "collect_strike_timing_sample",
    "ensure_home_offset",
    "int_or_none",
    "interpolate_curve",
    "load_strike_timing_csv",
    "mean_or_none",
    "median_or_none",
    "measurement_sweep_axis_label",
    "measurement_sweep_label",
    "parse_bool_flag",
    "parse_optional_int",
    "plot_strike_timing_samples",
    "print_strike_timing_summary",
    "rounded_or_none",
    "resolve_measurement_sweep_values",
    "wait_for_strike_idle",
    "write_strike_timing_csv",
]
