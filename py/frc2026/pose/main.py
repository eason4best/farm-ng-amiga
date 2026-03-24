"""Drive a 10m x 4m rectangle, pausing 10 s at every corner — one lap."""
# Copyright (c) farm-ng, inc.
#
# Licensed under the Amiga Development Kit License (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://github.com/farm-ng/amiga-dev-kit/blob/main/LICENSE
from __future__ import annotations

import argparse
import asyncio
from math import copysign
from math import radians
from pathlib import Path

from farm_ng.core.event_client import EventClient
from farm_ng.core.event_service_pb2 import EventServiceConfigList
from farm_ng.core.event_service_pb2 import SubscribeRequest
from farm_ng.core.events_file_reader import proto_from_json_file
from farm_ng.core.uri_pb2 import Uri
from farm_ng.filter.filter_pb2 import FilterState
from farm_ng.track.track_pb2 import Track
from farm_ng.track.track_pb2 import TrackFollowerState
from farm_ng.track.track_pb2 import TrackFollowRequest
from farm_ng_core_pybind import Isometry3F64
from farm_ng_core_pybind import Pose3F64
from farm_ng_core_pybind import Rotation3F64
from google.protobuf.empty_pb2 import Empty

# ---------------------------------------------------------------------------
# TrackStatusEnum values (from track.proto)
# ---------------------------------------------------------------------------
TRACK_FOLLOWING: int = 3
TRACK_PAUSED: int = 4
TRACK_COMPLETE: int = 5

# ---------------------------------------------------------------------------
# Rectangle configuration
# ---------------------------------------------------------------------------
LONG_SIDE: float = 10.0          # metres  (Side 1 and Side 3)
SHORT_SIDE: float = 4.0          # metres  (Side 2 and Side 4)
SIDE_LENGTHS: list[float] = [LONG_SIDE, SHORT_SIDE, LONG_SIDE, SHORT_SIDE]

CORNER_PAUSE_SECONDS: float = 10.0   # pause at each corner


# ---------------------------------------------------------------------------
# Service helpers
# ---------------------------------------------------------------------------

async def get_pose(clients: dict[str, EventClient]) -> Pose3F64:
    """Get the robot's current pose from the filter service."""
    state: FilterState = await clients["filter"].request_reply(
        "/get_state", Empty(), decode=True
    )
    print(f"Current filter state:\n{state}")
    return Pose3F64.from_proto(state.pose)


async def set_track(clients: dict[str, EventClient], track: Track) -> None:
    """Upload a track to the track_follower service."""
    print(f"Setting track:\n{track}")
    await clients["track_follower"].request_reply(
        "/set_track", TrackFollowRequest(track=track)
    )


async def start(clients: dict[str, EventClient]) -> None:
    """Command the track_follower to start."""
    print("Starting track follower...")
    await clients["track_follower"].request_reply("/start", Empty())


async def pause(clients: dict[str, EventClient]) -> None:
    """Pause the track follower — robot stops but remembers progress."""
    print("Sending /pause to track follower...")
    await clients["track_follower"].request_reply("/pause", Empty())


async def resume(clients: dict[str, EventClient]) -> None:
    """Resume from where /pause was called."""
    print("Sending /resume to track follower...")
    await clients["track_follower"].request_reply("/resume", Empty())


# ---------------------------------------------------------------------------
# State monitoring helpers
# ---------------------------------------------------------------------------

async def wait_for_status(
    clients: dict[str, EventClient],
    target_status: int,
    label: str = "",
) -> None:
    """Block until track_follower status.track_status == target_status."""
    print(f"Waiting for status: {label} (code={target_status})...")
    async for _, message in clients["track_follower"].subscribe(
        SubscribeRequest(uri=Uri(path="/state"))
    ):
        current = message.status.track_status
        remaining = message.progress.distance_remaining
        print(f"  track_status={current}  |  distance_remaining={remaining:.2f} m")
        if current == target_status:
            print(f"  ✓ Status reached: {label}")
            break


async def wait_for_waypoint_index(
    clients: dict[str, EventClient],
    target_index: int,
) -> None:
    """Block until closest_waypoint_index >= target_index."""
    print(f"Waiting for waypoint index >= {target_index} ...")
    async for _, message in clients["track_follower"].subscribe(
        SubscribeRequest(uri=Uri(path="/state"))
    ):
        idx = message.progress.closest_waypoint_index
        remaining = message.progress.distance_remaining
        track_status = message.status.track_status
        print(
            f"  waypoint={idx}/{message.progress.track_size - 1}"
            f"  |  remaining={remaining:.2f} m"
            f"  |  status={track_status}"
        )
        if track_status == TRACK_COMPLETE or idx >= target_index:
            print(f"  ✓ Corner waypoint reached at index {idx}.")
            break


# ---------------------------------------------------------------------------
# Waypoint builders
# ---------------------------------------------------------------------------

def create_straight_segment(
    previous_pose: Pose3F64,
    next_frame_b: str,
    distance: float,
    spacing: float = 0.1,
) -> list[Pose3F64]:
    """Straight waypoints spaced `spacing` metres apart."""
    segment_poses: list[Pose3F64] = [previous_pose]
    counter = 0
    remaining = distance

    while abs(remaining) > 0.01:
        seg_dist = copysign(min(abs(remaining), spacing), distance)
        segment_poses.append(
            segment_poses[-1] * Pose3F64(
                a_from_b=Isometry3F64([seg_dist, 0, 0], Rotation3F64.Rz(0)),
                frame_a=segment_poses[-1].frame_b,
                frame_b=f"{next_frame_b}_{counter}",
            )
        )
        counter += 1
        remaining -= seg_dist

    segment_poses[-1].frame_b = next_frame_b
    return segment_poses


def create_turn_segment(
    previous_pose: Pose3F64,
    next_frame_b: str,
    angle: float,
    spacing: float = 0.1,
) -> list[Pose3F64]:
    """In-place turn waypoints spaced `spacing` radians apart."""
    segment_poses: list[Pose3F64] = [previous_pose]
    counter = 0
    remaining = angle

    while abs(remaining) > 0.01:
        seg_angle = copysign(min(abs(remaining), spacing), angle)
        segment_poses.append(
            segment_poses[-1] * Pose3F64(
                a_from_b=Isometry3F64.Rz(seg_angle),
                frame_a=segment_poses[-1].frame_b,
                frame_b=f"{next_frame_b}_{counter}",
            )
        )
        counter += 1
        remaining -= seg_angle

    segment_poses[-1].frame_b = next_frame_b
    return segment_poses


def format_track(waypoints: list[Pose3F64]) -> Track:
    """Pack waypoints into a Track proto."""
    return Track(waypoints=[p.to_proto() for p in waypoints])


# ---------------------------------------------------------------------------
# Build the full rectangle and record corner waypoint indices
# ---------------------------------------------------------------------------

async def build_rectangle(
    clients: dict[str, EventClient],
    turn_angle: float,
) -> tuple[Track, list[int]]:
    """Build the full 10m x 4m rectangle as one continuous track.

    Path:
        Side 1: 10 m forward  → turn 90°  (Corner 1)
        Side 2:  4 m forward  → turn 90°  (Corner 2)
        Side 3: 10 m forward  → turn 90°  (Corner 3)
        Side 4:  4 m forward  → turn 90°  (Corner 4 / back to start)

    Returns:
        (Track, corner_indices) — corner_indices[i] is the waypoint index
        at the END of the i-th turn (i.e. exactly at the corner).
    """
    world_pose_robot = await get_pose(clients)

    anchor = world_pose_robot * Pose3F64(
        a_from_b=Isometry3F64(),
        frame_a="robot",
        frame_b="rect_start",
    )

    all_waypoints: list[Pose3F64] = [anchor]
    corner_indices: list[int] = []

    for side_num, side_len in enumerate(SIDE_LENGTHS, start=1):
        label = f"side{side_num}"

        # Straight section
        straight = create_straight_segment(
            all_waypoints[-1], f"{label}_str", side_len
        )
        all_waypoints.extend(straight[1:])

        # Turn section
        turn = create_turn_segment(
            all_waypoints[-1], f"{label}_turn", turn_angle
        )
        all_waypoints.extend(turn[1:])

        # Index of the last waypoint of this turn = corner position
        corner_indices.append(len(all_waypoints) - 1)

        print(
            f"  Side {side_num}: {side_len} m  |  "
            f"corner at waypoint index {corner_indices[-1]}"
        )

    print(
        f"\nRectangle built: {len(all_waypoints)} total waypoints. "
        f"Corner indices: {corner_indices}"
    )
    return format_track(all_waypoints), corner_indices


# ---------------------------------------------------------------------------
# Main driving logic
# ---------------------------------------------------------------------------

async def drive_rectangle(
    clients: dict[str, EventClient],
    clockwise: bool,
) -> None:
    """Drive the 10 m x 4 m rectangle once, pausing 10 s at each corner.

    Flow
    ----
    1. Build the full rectangle as ONE track.
    2. Upload → /start.
    3. Watch closest_waypoint_index.
    4. At each corner (1-3):
           /pause → wait 10 s → /resume
    5. At corner 4 (back to start): wait for TRACK_COMPLETE.

    Args:
        clients:   EventClient dictionary.
        clockwise: True → right-hand turns; False → left-hand turns.
    """
    turn_angle = radians(-90) if clockwise else radians(90)
    direction = "clockwise" if clockwise else "counter-clockwise"

    print(f"\n{'='*60}")
    print(f"Rectangle: {LONG_SIDE} m × {SHORT_SIDE} m  |  {direction}")
    print(f"Pause: {CORNER_PAUSE_SECONDS:.0f} s at each corner  |  1 lap")
    print(f"{'='*60}\n")

    # ── 1. Build and upload ────────────────────────────────────────────────
    track, corner_indices = await build_rectangle(clients, turn_angle)
    await set_track(clients, track)
    await start(clients)

    # ── 2. Handle each corner ─────────────────────────────────────────────
    side_labels = [
        f"Side 1 ({LONG_SIDE} m)",
        f"Side 2 ({SHORT_SIDE} m)",
        f"Side 3 ({LONG_SIDE} m)",
        f"Side 4 ({SHORT_SIDE} m)",
    ]

    for corner_num, (corner_idx, side_label) in enumerate(
        zip(corner_indices, side_labels), start=1
    ):
        print(f"\n--- After {side_label} → Corner {corner_num} ---")

        # Last corner: just wait for the track to finish
        if corner_num == 4:
            print("Final side — waiting for TRACK_COMPLETE...")
            await wait_for_status(clients, TRACK_COMPLETE, "TRACK_COMPLETE")
            print("✓ Rectangle complete — robot back at start.")
            break

        # Wait until robot reaches this corner
        await wait_for_waypoint_index(clients, corner_idx)

        # /pause
        await pause(clients)
        await wait_for_status(clients, TRACK_PAUSED, "TRACK_PAUSED")
        print(f"✓ Robot paused at Corner {corner_num}.")

        # 10-second countdown
        print(f"\n*** PAUSING at Corner {corner_num} for {CORNER_PAUSE_SECONDS:.0f} s ***")
        elapsed = 0.0
        while elapsed < CORNER_PAUSE_SECONDS:
            sleep_for = min(5.0, CORNER_PAUSE_SECONDS - elapsed)
            await asyncio.sleep(sleep_for)
            elapsed += sleep_for
            remaining = max(CORNER_PAUSE_SECONDS - elapsed, 0.0)
            print(f"    {elapsed:.0f} s elapsed, {remaining:.0f} s remaining")
        print(f"*** Resuming after Corner {corner_num} ***\n")

        # /resume
        await resume(clients)
        await wait_for_status(clients, TRACK_FOLLOWING, "TRACK_FOLLOWING")
        print(f"✓ Robot resumed — driving {side_labels[corner_num]}.")

    print("\nDone.")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

async def run(args) -> None:
    clients: dict[str, EventClient] = {}
    expected_configs = ["track_follower", "filter"]

    config_list = proto_from_json_file(args.service_config, EventServiceConfigList())
    for config in config_list.configs:
        if config.name in expected_configs:
            clients[config.name] = EventClient(config)

    for config in expected_configs:
        if config not in clients:
            raise RuntimeError(f"No {config} service config in {args.service_config}")

    await drive_rectangle(clients, args.clockwise)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        prog="python main.py",
        description="Drive a 10 m x 4 m rectangle with 10 s pauses at each corner.",
    )
    parser.add_argument(
        "--service-config", type=Path, required=True,
        help="Path to the service config JSON file.",
    )
    parser.add_argument(
        "--clockwise", action="store_true",
        help="Drive clockwise (right-hand turns). Default is counter-clockwise.",
    )
    args = parser.parse_args()

    loop = asyncio.get_event_loop()
    loop.run_until_complete(run(args))