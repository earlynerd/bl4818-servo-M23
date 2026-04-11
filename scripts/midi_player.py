#!/usr/bin/env python3
"""
Headless MIDI scheduler and serial playback prototype for the BL4818 ring bus.
"""

from __future__ import annotations

import argparse
import collections
import dataclasses
import json
import queue
import sys
import threading
import time
from pathlib import Path
from typing import Any, Optional

try:
    import mido
except ModuleNotFoundError as exc:  # pragma: no cover - import guard
    raise SystemExit(
        "mido is required. Install it with: py -m pip install mido"
    ) from exc

try:
    from ring_bus import CommandAck, REPLY_MODE_ACK, RingClientV2, auto_detect_port
except ModuleNotFoundError as exc:  # pragma: no cover - import guard
    raise SystemExit(
        "pyserial is required. Install it with: py -m pip install pyserial"
    ) from exc


DEFAULT_LOOKAHEAD_MS = 100
DEFAULT_POLL_MS = 10
DEFAULT_SCALE_STEP_MS = 500
DEFAULT_SCALE_DURATION_MS = 250
DEFAULT_SCALE_VELOCITY = 96
DEFAULT_START_MARGIN_MS = 50
ESTIMATED_STRIKE_TRANSACTION_MS = 1.24

NOTE_NAMES = ("C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B")


class MidiPlayerError(Exception):
    pass


@dataclasses.dataclass(frozen=True)
class InputNote:
    midi_note: int
    velocity: int
    start_ms: float
    duration_ms: float
    channel: int


@dataclasses.dataclass(frozen=True)
class MidiSource:
    name: str
    notes: list[InputNote]
    format_type: Optional[int] = None
    track_count: Optional[int] = None
    ticks_per_beat: Optional[int] = None
    tempo_change_count: int = 0
    total_duration_ms: float = 0.0


@dataclasses.dataclass(frozen=True)
class LookupEntry:
    midi_velocity: int
    target_strength_dps: int
    duty: int
    lead_ms: int
    repeat_ms: int
    settle_ms: int


@dataclasses.dataclass
class MalletProfile:
    name: str
    address: int
    midi_note: int
    source_label: str
    lookup_entries: list[LookupEntry]
    scheduler_ready: bool
    warnings: list[str]
    selected_home_offset: Optional[int]

    def lookup_for_velocity(self, velocity: int) -> LookupEntry:
        clamped = max(1, min(127, int(velocity)))
        index = max(0, min(len(self.lookup_entries) - 1, clamped - 1))
        entry = self.lookup_entries[index]
        if entry.midi_velocity == clamped:
            return entry
        for candidate in self.lookup_entries:
            if candidate.midi_velocity >= clamped:
                return candidate
        return self.lookup_entries[-1]


@dataclasses.dataclass(frozen=True)
class ScheduledStrike:
    sequence_index: int
    source_index: int
    impact_time_ms: float
    send_deadline_ms: float
    wall_send_ms: float
    wall_impact_ms: float
    duration_ms: float
    address: int
    midi_note: int
    midi_velocity: int
    duty: int
    lead_ms: int
    repeat_ms: int
    settle_ms: int
    mallet_name: str
    source_label: str


@dataclasses.dataclass(frozen=True)
class PlaybackLogEntry:
    event: ScheduledStrike
    sent_offset_ms: float
    ack_offset_ms: float
    lateness_ms: float
    ack_result_name: str
    accepted: bool
    detail: int


@dataclasses.dataclass
class PerformancePlan:
    source: MidiSource
    scheduled: list[ScheduledStrike]
    issues: list[str]
    profile_warnings: list[str]
    scheduler_preroll_ms: float
    skipped_unmapped: int
    skipped_conflicts: int
    total_source_notes: int
    max_chord_size: int
    estimated_chord_bus_ms: float
    estimated_total_bus_ms: float
    last_wall_impact_ms: float
    last_settle_complete_ms: float


def format_note_name(midi_note: int) -> str:
    octave = (midi_note // 12) - 1
    return f"{NOTE_NAMES[midi_note % 12]}{octave}"


def load_midi_source(path: Path) -> MidiSource:
    midi = mido.MidiFile(path)
    pending: dict[tuple[int, int], collections.deque[tuple[float, int]]] = collections.defaultdict(collections.deque)
    notes: list[InputNote] = []
    current_s = 0.0

    for message in midi:
        current_s += float(message.time)
        if not hasattr(message, "type"):
            continue
        if message.type == "note_on" and getattr(message, "velocity", 0) > 0:
            pending[(message.channel, message.note)].append((current_s, int(message.velocity)))
            continue
        if message.type not in ("note_off", "note_on"):
            continue
        if message.type == "note_on" and getattr(message, "velocity", 0) != 0:
            continue
        key = (message.channel, message.note)
        if not pending[key]:
            continue
        start_s, velocity = pending[key].popleft()
        notes.append(
            InputNote(
                midi_note=int(message.note),
                velocity=velocity,
                start_ms=start_s * 1000.0,
                duration_ms=max(0.0, (current_s - start_s) * 1000.0),
                channel=int(message.channel),
            )
        )

    tempo_change_count = sum(
        1
        for track in midi.tracks
        for message in track
        if getattr(message, "type", None) == "set_tempo"
    )

    return MidiSource(
        name=path.stem,
        notes=sorted(notes, key=lambda note: (note.start_ms, note.channel, note.midi_note)),
        format_type=midi.type,
        track_count=len(midi.tracks),
        ticks_per_beat=midi.ticks_per_beat,
        tempo_change_count=tempo_change_count,
        total_duration_ms=current_s * 1000.0,
    )


def build_scale_source(
    profiles: list[MalletProfile],
    step_ms: int,
    duration_ms: int,
    velocity: int,
) -> MidiSource:
    notes: list[InputNote] = []
    for index, profile in enumerate(sorted(profiles, key=lambda item: (item.midi_note, item.address))):
        notes.append(
            InputNote(
                midi_note=profile.midi_note,
                velocity=max(1, min(127, velocity)),
                start_ms=index * step_ms,
                duration_ms=max(0, duration_ms),
                channel=0,
            )
        )
    total_duration_ms = max((note.start_ms + note.duration_ms) for note in notes) if notes else 0.0
    return MidiSource(name="generated-scale", notes=notes, total_duration_ms=total_duration_ms)


def require_json_object(value: Any, context: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise MidiPlayerError(f"{context} must be a JSON object")
    return value


def parse_lookup_entries(raw_entries: Any, context: str) -> list[LookupEntry]:
    if not isinstance(raw_entries, list) or not raw_entries:
        raise MidiPlayerError(f"{context} must contain a non-empty midi_velocity_lookup list")

    entries: list[LookupEntry] = []
    for index, raw_entry in enumerate(raw_entries):
        entry = require_json_object(raw_entry, f"{context}[{index}]")
        entries.append(
            LookupEntry(
                midi_velocity=int(entry["midi_velocity"]),
                target_strength_dps=int(entry.get("target_strength_dps", entry.get("duty", 0))),
                duty=int(entry["duty"]),
                lead_ms=int(entry["lead_ms"]),
                repeat_ms=int(entry.get("repeat_ms", entry.get("settle_ms", 0))),
                settle_ms=int(entry.get("settle_ms", entry.get("repeat_ms", 0))),
            )
        )
    entries.sort(key=lambda item: item.midi_velocity)
    return entries


def choose_lookup(
    profile_data: dict[str, Any],
    context: str,
    requested_home_offset: Optional[int],
) -> tuple[list[LookupEntry], Optional[int]]:
    if "midi_velocity_lookup" in profile_data:
        return (
            parse_lookup_entries(profile_data["midi_velocity_lookup"], f"{context}.midi_velocity_lookup"),
            int(profile_data["lookup_home_offset"]) if "lookup_home_offset" in profile_data else (
                int(profile_data["suggested_home_offset"]) if "suggested_home_offset" in profile_data else None
            ),
        )

    if "home_offset_profiles" not in profile_data:
        raise MidiPlayerError(f"{context} is missing midi_velocity_lookup or home_offset_profiles")

    raw_profiles = profile_data["home_offset_profiles"]
    if not isinstance(raw_profiles, list) or not raw_profiles:
        raise MidiPlayerError(f"{context}.home_offset_profiles must be a non-empty list")

    chosen_profile: Optional[dict[str, Any]] = None
    chosen_offset: Optional[int] = None
    target_offset = requested_home_offset
    if target_offset is None and "suggested_home_offset" in profile_data:
        target_offset = int(profile_data["suggested_home_offset"])

    if target_offset is not None:
        for raw_profile in raw_profiles:
            profile = require_json_object(raw_profile, f"{context}.home_offset_profiles[]")
            if int(profile["home_offset"]) == target_offset:
                chosen_profile = profile
                chosen_offset = target_offset
                break
        if chosen_profile is None:
            raise MidiPlayerError(f"{context} does not contain home_offset {target_offset}")

    if chosen_profile is None:
        chosen_profile = require_json_object(raw_profiles[0], f"{context}.home_offset_profiles[0]")
        chosen_offset = int(chosen_profile["home_offset"])

    return (
        parse_lookup_entries(chosen_profile["midi_velocity_lookup"], f"{context}.home_offset_profile"),
        chosen_offset,
    )


def build_mallet_profile(
    profile_data: dict[str, Any],
    source_path: Path,
    context: str,
    overrides: Optional[dict[str, Any]] = None,
) -> MalletProfile:
    overrides = overrides or {}
    identity = profile_data.get("identity", {})
    if identity is None:
        identity = {}
    if not isinstance(identity, dict):
        raise MidiPlayerError(f"{context}.identity must be an object")

    address = overrides.get("address", identity.get("address"))
    midi_note = overrides.get("midi_note", identity.get("midi_note"))
    name = overrides.get("name", identity.get("name"))
    home_offset = overrides.get("home_offset")

    if address is None:
        raise MidiPlayerError(f"{context} is missing identity.address")
    if midi_note is None:
        raise MidiPlayerError(f"{context} is missing identity.midi_note")

    lookup_entries, selected_home_offset = choose_lookup(
        profile_data,
        context=context,
        requested_home_offset=None if home_offset is None else int(home_offset),
    )
    warning_list = profile_data.get("warnings", [])
    warnings = [str(item) for item in warning_list] if isinstance(warning_list, list) else []

    return MalletProfile(
        name=str(name) if name else format_note_name(int(midi_note)),
        address=int(address),
        midi_note=int(midi_note),
        source_label=source_path.name,
        lookup_entries=lookup_entries,
        scheduler_ready=bool(profile_data.get("scheduler_ready", True)),
        warnings=warnings,
        selected_home_offset=selected_home_offset,
    )


def extract_mallet_profiles(
    raw_data: Any,
    source_path: Path,
    context: str,
    inherited_overrides: Optional[dict[str, Any]] = None,
) -> list[MalletProfile]:
    inherited_overrides = inherited_overrides or {}

    if isinstance(raw_data, list):
        profiles: list[MalletProfile] = []
        for index, item in enumerate(raw_data):
            profiles.extend(
                extract_mallet_profiles(
                    item,
                    source_path,
                    f"{context}[{index}]",
                    inherited_overrides=inherited_overrides,
                )
            )
        return profiles

    raw_dict = require_json_object(raw_data, context)

    if "mallets" in raw_dict:
        mallets = raw_dict["mallets"]
        if not isinstance(mallets, list):
            raise MidiPlayerError(f"{context}.mallets must be a list")
        profiles: list[MalletProfile] = []
        for index, mallet in enumerate(mallets):
            profiles.extend(
                extract_mallet_profiles(
                    mallet,
                    source_path,
                    f"{context}.mallets[{index}]",
                    inherited_overrides=inherited_overrides,
                )
            )
        return profiles

    current_overrides = dict(inherited_overrides)
    for key in ("address", "midi_note", "name", "home_offset"):
        if key in raw_dict:
            current_overrides[key] = raw_dict[key]

    if "profile_path" in raw_dict:
        profile_path = source_path.parent / str(raw_dict["profile_path"])
        nested_data = json.loads(profile_path.read_text(encoding="utf-8"))
        return extract_mallet_profiles(
            nested_data,
            profile_path,
            f"{context}.profile_path",
            inherited_overrides=current_overrides,
        )

    if "profile" in raw_dict:
        return extract_mallet_profiles(
            raw_dict["profile"],
            source_path,
            f"{context}.profile",
            inherited_overrides=current_overrides,
        )

    return [build_mallet_profile(raw_dict, source_path, context, overrides=current_overrides)]


def load_profiles(profile_paths: list[Path]) -> list[MalletProfile]:
    profiles: list[MalletProfile] = []
    for profile_path in profile_paths:
        raw_data = json.loads(profile_path.read_text(encoding="utf-8"))
        profiles.extend(extract_mallet_profiles(raw_data, profile_path, profile_path.name))
    if not profiles:
        raise MidiPlayerError("no mallet profiles were loaded")

    midi_note_map: dict[int, MalletProfile] = {}
    address_map: dict[int, MalletProfile] = {}
    for profile in profiles:
        if profile.midi_note in midi_note_map:
            other = midi_note_map[profile.midi_note]
            raise MidiPlayerError(
                f"duplicate midi_note {profile.midi_note} in {other.source_label} and {profile.source_label}"
            )
        if profile.address in address_map:
            other = address_map[profile.address]
            raise MidiPlayerError(
                f"duplicate address {profile.address} in {other.source_label} and {profile.source_label}"
            )
        midi_note_map[profile.midi_note] = profile
        address_map[profile.address] = profile

    return sorted(profiles, key=lambda item: (item.midi_note, item.address))


def build_performance_plan(
    source: MidiSource,
    profiles: list[MalletProfile],
    conflict_policy: str,
) -> PerformancePlan:
    profile_by_note = {profile.midi_note: profile for profile in profiles}
    issues: list[str] = []
    profile_warnings: list[str] = []
    scheduled_raw: list[tuple[ScheduledStrike, float]] = []
    skipped_unmapped = 0
    skipped_conflicts = 0
    next_ready_time_by_address: dict[int, float] = {}
    sequence_index = 0

    for profile in profiles:
        if not profile.scheduler_ready:
            profile_warnings.append(
                f"profile {profile.source_label} for {profile.name} is not scheduler_ready"
            )
        for warning in profile.warnings:
            profile_warnings.append(f"profile {profile.source_label} for {profile.name}: {warning}")

    for source_index, note in enumerate(source.notes):
        profile = profile_by_note.get(note.midi_note)
        if profile is None:
            skipped_unmapped += 1
            issues.append(
                f"unmapped note {format_note_name(note.midi_note)} velocity={note.velocity} "
                f"at {note.start_ms:.1f} ms"
            )
            continue

        lookup = profile.lookup_for_velocity(note.velocity)
        impact_time_ms = note.start_ms
        next_ready = next_ready_time_by_address.get(profile.address)
        if next_ready is not None and impact_time_ms < next_ready:
            warning = (
                f"retrigger conflict on {profile.name} addr={profile.address}: "
                f"impact {impact_time_ms:.1f} ms is earlier than ready {next_ready:.1f} ms"
            )
            if conflict_policy == "error":
                raise MidiPlayerError(warning)
            issues.append(warning)
            if conflict_policy == "drop":
                skipped_conflicts += 1
                continue

        scheduled_raw.append(
            (
                ScheduledStrike(
                    sequence_index=sequence_index,
                    source_index=source_index,
                    impact_time_ms=impact_time_ms,
                    send_deadline_ms=impact_time_ms - lookup.lead_ms,
                    wall_send_ms=0.0,
                    wall_impact_ms=0.0,
                    duration_ms=note.duration_ms,
                    address=profile.address,
                    midi_note=note.midi_note,
                    midi_velocity=note.velocity,
                    duty=lookup.duty,
                    lead_ms=lookup.lead_ms,
                    repeat_ms=lookup.repeat_ms,
                    settle_ms=lookup.settle_ms,
                    mallet_name=profile.name,
                    source_label=profile.source_label,
                ),
                lookup.settle_ms,
            )
        )
        next_ready_time_by_address[profile.address] = impact_time_ms + lookup.repeat_ms
        sequence_index += 1

    min_deadline_ms = min((event.send_deadline_ms for event, _ in scheduled_raw), default=0.0)
    preroll_ms = max(DEFAULT_START_MARGIN_MS, -min_deadline_ms + DEFAULT_START_MARGIN_MS)

    scheduled: list[ScheduledStrike] = []
    last_wall_impact_ms = 0.0
    last_settle_complete_ms = 0.0
    for event, settle_ms in scheduled_raw:
        scheduled_event = dataclasses.replace(
            event,
            wall_send_ms=preroll_ms + event.send_deadline_ms,
            wall_impact_ms=preroll_ms + event.impact_time_ms,
        )
        scheduled.append(scheduled_event)
        last_wall_impact_ms = max(last_wall_impact_ms, scheduled_event.wall_impact_ms)
        last_settle_complete_ms = max(last_settle_complete_ms, scheduled_event.wall_impact_ms + settle_ms)

    chord_counts: dict[float, int] = {}
    for event in scheduled:
        key = round(event.impact_time_ms, 3)
        chord_counts[key] = chord_counts.get(key, 0) + 1

    max_chord_size = max(chord_counts.values(), default=0)
    return PerformancePlan(
        source=source,
        scheduled=scheduled,
        issues=issues,
        profile_warnings=profile_warnings,
        scheduler_preroll_ms=preroll_ms,
        skipped_unmapped=skipped_unmapped,
        skipped_conflicts=skipped_conflicts,
        total_source_notes=len(source.notes),
        max_chord_size=max_chord_size,
        estimated_chord_bus_ms=max_chord_size * ESTIMATED_STRIKE_TRANSACTION_MS,
        estimated_total_bus_ms=len(scheduled) * ESTIMATED_STRIKE_TRANSACTION_MS,
        last_wall_impact_ms=last_wall_impact_ms,
        last_settle_complete_ms=last_settle_complete_ms,
    )


def plan_to_jsonable(plan: PerformancePlan) -> dict[str, Any]:
    return {
        "source": {
            "name": plan.source.name,
            "note_count": len(plan.source.notes),
            "format_type": plan.source.format_type,
            "track_count": plan.source.track_count,
            "ticks_per_beat": plan.source.ticks_per_beat,
            "tempo_change_count": plan.source.tempo_change_count,
            "total_duration_ms": round(plan.source.total_duration_ms, 3),
        },
        "summary": {
            "scheduled_count": len(plan.scheduled),
            "total_source_notes": plan.total_source_notes,
            "skipped_unmapped": plan.skipped_unmapped,
            "skipped_conflicts": plan.skipped_conflicts,
            "scheduler_preroll_ms": round(plan.scheduler_preroll_ms, 3),
            "max_chord_size": plan.max_chord_size,
            "estimated_chord_bus_ms": round(plan.estimated_chord_bus_ms, 3),
            "estimated_total_bus_ms": round(plan.estimated_total_bus_ms, 3),
            "last_wall_impact_ms": round(plan.last_wall_impact_ms, 3),
            "last_settle_complete_ms": round(plan.last_settle_complete_ms, 3),
        },
        "profile_warnings": plan.profile_warnings,
        "issues": plan.issues,
        "events": [
            {
                "sequence_index": event.sequence_index,
                "source_index": event.source_index,
                "impact_time_ms": round(event.impact_time_ms, 3),
                "send_deadline_ms": round(event.send_deadline_ms, 3),
                "wall_send_ms": round(event.wall_send_ms, 3),
                "wall_impact_ms": round(event.wall_impact_ms, 3),
                "duration_ms": round(event.duration_ms, 3),
                "address": event.address,
                "midi_note": event.midi_note,
                "midi_note_name": format_note_name(event.midi_note),
                "midi_velocity": event.midi_velocity,
                "duty": event.duty,
                "lead_ms": event.lead_ms,
                "repeat_ms": event.repeat_ms,
                "settle_ms": event.settle_ms,
                "mallet_name": event.mallet_name,
                "source_label": event.source_label,
            }
            for event in plan.scheduled
        ],
    }


def print_plan_summary(plan: PerformancePlan, show_events: int) -> None:
    source = plan.source
    print(f"Source: {source.name}")
    if source.format_type is not None:
        print(
            f"  midi_format={source.format_type} tracks={source.track_count} "
            f"ticks_per_beat={source.ticks_per_beat} tempo_changes={source.tempo_change_count}"
        )
    print(
        f"  source_notes={plan.total_source_notes} scheduled={len(plan.scheduled)} "
        f"skipped_unmapped={plan.skipped_unmapped} skipped_conflicts={plan.skipped_conflicts}"
    )
    print(
        f"  preroll_ms={plan.scheduler_preroll_ms:.1f} "
        f"max_chord={plan.max_chord_size} est_chord_bus_ms={plan.estimated_chord_bus_ms:.2f} "
        f"est_total_bus_ms={plan.estimated_total_bus_ms:.2f}"
    )
    if plan.profile_warnings:
        print("Profile warnings:")
        for warning in plan.profile_warnings:
            print(f"  - {warning}")
    if plan.issues:
        print("Schedule issues:")
        for warning in plan.issues[:20]:
            print(f"  - {warning}")
        if len(plan.issues) > 20:
            print(f"  - ... {len(plan.issues) - 20} more")
    if show_events <= 0:
        return
    print("Scheduled events:")
    for event in plan.scheduled[:show_events]:
        print(
            f"  #{event.sequence_index:03d} impact={event.impact_time_ms:8.1f} ms "
            f"deadline={event.send_deadline_ms:8.1f} ms addr={event.address:02d} "
            f"note={format_note_name(event.midi_note):>3} vel={event.midi_velocity:3d} "
            f"duty={event.duty:4d} lead={event.lead_ms:3d} "
            f"repeat={event.repeat_ms:3d} settle={event.settle_ms:3d}"
        )
    if len(plan.scheduled) > show_events:
        print(f"  ... {len(plan.scheduled) - show_events} more events")


def sleep_until(target_time: float) -> None:
    while True:
        remaining = target_time - time.monotonic()
        if remaining <= 0:
            return
        if remaining > 0.050:
            time.sleep(0.020)
        elif remaining > 0.005:
            time.sleep(0.002)
        else:
            time.sleep(0.0005)


def wait_for_idle_and_homed(client: RingClientV2, address: int, timeout_ms: int, poll_ms: int) -> None:
    deadline = time.monotonic() + timeout_ms / 1000.0
    poll_s = max(0, poll_ms) / 1000.0
    last_status = None
    while True:
        last_status = client.query_strike(address)
        if last_status.homed and last_status.state == 0 and not last_status.active:
            return
        if time.monotonic() >= deadline:
            break
        if poll_s > 0:
            time.sleep(poll_s)
    homed_text = "unknown" if last_status is None else str(last_status.homed)
    state_text = "unknown" if last_status is None else last_status.state_name
    raise MidiPlayerError(
        f"timed out waiting for addr {address} to become homed+idle "
        f"(homed={homed_text} state={state_text})"
    )


def verify_or_home_mallets(
    client: RingClientV2,
    addresses: list[int],
    home_all: bool,
    timeout_ms: int,
    poll_ms: int,
) -> None:
    if home_all:
        for address in addresses:
            reply = client.strike_home(address, reply_mode=REPLY_MODE_ACK)
            if not isinstance(reply, CommandAck) or not reply.accepted:
                result = "no-ack" if reply is None else reply.result_name
                raise MidiPlayerError(f"strike_home addr {address} was not accepted: {result}")
        for address in addresses:
            wait_for_idle_and_homed(client, address, timeout_ms=timeout_ms, poll_ms=poll_ms)
        return

    for address in addresses:
        status = client.query_strike(address)
        if not status.homed:
            raise MidiPlayerError(
                f"addr {address} is not homed; rerun with --home-all or home it manually first"
            )


class SerialStrikeWorker(threading.Thread):
    def __init__(
        self,
        client: RingClientV2,
        start_time: float,
        event_queue: queue.Queue[Optional[ScheduledStrike]],
    ):
        super().__init__(daemon=True)
        self.client = client
        self.start_time = start_time
        self.event_queue = event_queue
        self.results: list[PlaybackLogEntry] = []
        self.error: Optional[BaseException] = None

    def run(self) -> None:
        try:
            while True:
                event = self.event_queue.get()
                if event is None:
                    return
                target_time = self.start_time + event.wall_send_ms / 1000.0
                sleep_until(target_time)
                send_time = time.monotonic()
                reply = self.client.strike(event.address, event.duty, reply_mode=REPLY_MODE_ACK)
                ack_time = time.monotonic()
                if not isinstance(reply, CommandAck):
                    raise MidiPlayerError(
                        f"addr {event.address} strike did not return an acknowledgement"
                    )
                self.results.append(
                    PlaybackLogEntry(
                        event=event,
                        sent_offset_ms=(send_time - self.start_time) * 1000.0,
                        ack_offset_ms=(ack_time - self.start_time) * 1000.0,
                        lateness_ms=(send_time - target_time) * 1000.0,
                        ack_result_name=reply.result_name,
                        accepted=reply.accepted,
                        detail=reply.detail,
                    )
                )
        except BaseException as exc:  # noqa: BLE001
            self.error = exc


def execute_playback(
    plan: PerformancePlan,
    client: RingClientV2,
    lookahead_ms: int,
) -> list[PlaybackLogEntry]:
    start_time = time.monotonic()
    event_queue: queue.Queue[Optional[ScheduledStrike]] = queue.Queue()
    worker = SerialStrikeWorker(client=client, start_time=start_time, event_queue=event_queue)
    worker.start()

    for event in plan.scheduled:
        queue_target = start_time + max(0.0, (event.wall_send_ms - lookahead_ms)) / 1000.0
        sleep_until(queue_target)
        event_queue.put(event)
        if worker.error is not None:
            break

    event_queue.put(None)
    worker.join()
    if worker.error is not None:
        raise MidiPlayerError(str(worker.error))
    return worker.results


def print_playback_summary(results: list[PlaybackLogEntry]) -> None:
    accepted = sum(1 for item in results if item.accepted)
    rejected = len(results) - accepted
    max_late_ms = max((item.lateness_ms for item in results), default=0.0)
    print(
        f"Playback summary: events={len(results)} accepted={accepted} "
        f"rejected={rejected} max_lateness_ms={max_late_ms:.3f}"
    )
    for item in results[:20]:
        print(
            f"  #{item.event.sequence_index:03d} addr={item.event.address:02d} "
            f"note={format_note_name(item.event.midi_note):>3} duty={item.event.duty:4d} "
            f"sent={item.sent_offset_ms:8.2f} ms late={item.lateness_ms:7.3f} ms "
            f"ack={item.ack_result_name}"
        )
    if len(results) > 20:
        print(f"  ... {len(results) - 20} more results")


def add_profile_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--profile",
        action="append",
        required=True,
        help="Profile JSON path. Repeat to merge multiple per-note calibration files.",
    )
    parser.add_argument(
        "--conflict-policy",
        choices=("drop", "keep", "error"),
        default="drop",
        help="How to handle repeat_ms conflicts on the same mallet (default: drop)",
    )


def add_serial_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("-p", "--port", help="Serial port (auto-detect if omitted)")
    parser.add_argument(
        "-b",
        "--baud",
        type=int,
        default=250000,
        help="Ring bus baud rate (default: 250000)",
    )
    parser.add_argument(
        "--timeout-ms",
        type=int,
        default=200,
        help="Serial reply timeout in ms (default: 200)",
    )
    parser.add_argument(
        "--settle-ms",
        type=int,
        default=250,
        help="Post-open settle delay in ms (default: 250)",
    )
    parser.add_argument(
        "--lookahead-ms",
        type=int,
        default=DEFAULT_LOOKAHEAD_MS,
        help=f"Scheduler lookahead window in ms (default: {DEFAULT_LOOKAHEAD_MS})",
    )
    parser.add_argument(
        "--expect-devices",
        type=int,
        help="Optional exact ring enumeration count to require before playback",
    )
    parser.add_argument(
        "--home-all",
        action="store_true",
        help="Run strike_home on every used mallet before playback",
    )
    parser.add_argument(
        "--poll-ms",
        type=int,
        default=DEFAULT_POLL_MS,
        help=f"Poll interval used while waiting for homing in ms (default: {DEFAULT_POLL_MS})",
    )
    parser.add_argument("--trace", action="store_true", help="Enable raw frame tracing")


def open_ring_client(args: argparse.Namespace) -> RingClientV2:
    port = args.port or auto_detect_port()
    if not port:
        raise MidiPlayerError("no serial port found; pass --port explicitly")
    client = RingClientV2(
        port=port,
        baudrate=args.baud,
        timeout_ms=args.timeout_ms,
        settle_ms=args.settle_ms,
        trace=args.trace,
    )
    client.open()
    count = client.enumerate()
    print(f"Connected on {port}; devices={count}")
    if args.expect_devices is not None and count != args.expect_devices:
        raise MidiPlayerError(f"expected {args.expect_devices} devices but enumerated {count}")
    return client


def build_plan_from_args(args: argparse.Namespace) -> tuple[list[MalletProfile], PerformancePlan]:
    profiles = load_profiles([Path(item) for item in args.profile])
    if args.command in ("schedule-midi", "play-midi"):
        source = load_midi_source(Path(args.midi))
    elif args.command == "play-scale":
        source = build_scale_source(
            profiles=profiles,
            step_ms=args.step_ms,
            duration_ms=args.duration_ms,
            velocity=args.velocity,
        )
    else:
        raise MidiPlayerError(f"unsupported command {args.command}")
    plan = build_performance_plan(source=source, profiles=profiles, conflict_policy=args.conflict_policy)
    if args.command in ("play-midi", "play-scale") and not plan.scheduled:
        raise MidiPlayerError("the current inputs produced an empty playback schedule")
    return profiles, plan


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Headless MIDI scheduler and serial playback prototype for the BL4818 ring bus"
    )
    sub = parser.add_subparsers(dest="command", required=True)

    schedule = sub.add_parser("schedule-midi", help="Load MIDI plus profile JSON and print a dry-run schedule")
    add_profile_args(schedule)
    schedule.add_argument("--midi", required=True, help="Path to the MIDI file")
    schedule.add_argument(
        "--show-events",
        type=int,
        default=20,
        help="How many scheduled events to print (default: 20)",
    )
    schedule.add_argument("--json-out", help="Optional path to write the expanded schedule JSON")

    play_midi = sub.add_parser("play-midi", help="Play a MIDI file over the serial ring bus")
    add_profile_args(play_midi)
    add_serial_args(play_midi)
    play_midi.add_argument("--midi", required=True, help="Path to the MIDI file")
    play_midi.add_argument(
        "--show-events",
        type=int,
        default=10,
        help="How many scheduled events to print before playback (default: 10)",
    )

    play_scale = sub.add_parser("play-scale", help="Generate and play a simple scale across the loaded mallets")
    add_profile_args(play_scale)
    add_serial_args(play_scale)
    play_scale.add_argument(
        "--step-ms",
        type=int,
        default=DEFAULT_SCALE_STEP_MS,
        help=f"Gap between scale notes in ms (default: {DEFAULT_SCALE_STEP_MS})",
    )
    play_scale.add_argument(
        "--duration-ms",
        type=int,
        default=DEFAULT_SCALE_DURATION_MS,
        help=f"Nominal note duration in ms for the generated scale (default: {DEFAULT_SCALE_DURATION_MS})",
    )
    play_scale.add_argument(
        "--velocity",
        type=int,
        default=DEFAULT_SCALE_VELOCITY,
        help=f"MIDI velocity for generated scale notes (default: {DEFAULT_SCALE_VELOCITY})",
    )
    play_scale.add_argument(
        "--show-events",
        type=int,
        default=20,
        help="How many scheduled events to print before playback (default: 20)",
    )

    return parser


def main(argv: Optional[list[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    try:
        _, plan = build_plan_from_args(args)
        print_plan_summary(plan, show_events=args.show_events)

        if args.command == "schedule-midi":
            if args.json_out:
                out_path = Path(args.json_out)
                out_path.write_text(json.dumps(plan_to_jsonable(plan), indent=2) + "\n", encoding="ascii")
                print(f"Schedule JSON saved to {out_path}")
            return 0

        client = open_ring_client(args)
        try:
            used_addresses = sorted({event.address for event in plan.scheduled})
            verify_or_home_mallets(
                client=client,
                addresses=used_addresses,
                home_all=args.home_all,
                timeout_ms=max(args.timeout_ms, 3000),
                poll_ms=args.poll_ms,
            )
            results = execute_playback(plan=plan, client=client, lookahead_ms=args.lookahead_ms)
            print_playback_summary(results)
            return 0
        finally:
            client.close()

    except MidiPlayerError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("\nCancelled.", file=sys.stderr)
        return 130


if __name__ == "__main__":
    sys.exit(main())
