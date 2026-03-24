"""Drive 10 m forward.
- Pause 5 s at every 1 m mark (1m, 2m, 3m ... 10m).
- Collect GPS data every second throughout the entire drive.
- Save all GPS readings to a CSV file with timestamp, north, east, and status.
"""
from __future__ import annotations

import argparse
import asyncio
import csv
from datetime import datetime
from pathlib import Path

from farm_ng.core.event_client import EventClient
from farm_ng.core.event_service_pb2 import EventServiceConfigList
from farm_ng.core.event_service_pb2 import SubscribeRequest
from farm_ng.core.events_file_reader import proto_from_json_file
from farm_ng.core.uri_pb2 import Uri
from farm_ng.filter.filter_pb2 import FilterState
from farm_ng.gps import gps_pb2
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
# Configuration
# ---------------------------------------------------------------------------
TOTAL_DISTANCE: float = 10.0      # total metres to drive
PAUSE_INTERVAL: float = 1.0       # pause every this many metres
PAUSE_DURATION: float = 5.0       # seconds to pause at each interval
WAYPOINT_SPACING: float = 0.1     # metres between waypoints
GPS_OUTPUT_FILE: str = "gps_data.csv"

# Waypoints per metre  (1.0 / 0.1 = 10)
WAYPOINTS_PER_METRE: int = int(PAUSE_INTERVAL / WAYPOINT_SPACING)

# Pause waypoint indices: 10, 20, 30 ... 100  (every 1 m)
# We subtract 1 because indices are 0-based and the anchor is index 0
PAUSE_INDICES: list[int] = [
    int(WAYPOINTS_PER_METRE * i)
    for i in range(1, int(TOTAL_DISTANCE / PAUSE_INTERVAL) + 1)
]


# ---------------------------------------------------------------------------
# Shared GPS state  (updated by background GPS task)
# ---------------------------------------------------------------------------
class GpsState:
    """Holds the latest GPS reading, shared between tasks."""
    north: float = 0.0
    east: float = 0.0
    updated: bool = False


gps_state = GpsState()


# ---------------------------------------------------------------------------
# Service helpers
# ---------------------------------------------------------------------------

async def get_pose(clients: dict[str, EventClient]) -> Pose3F64:
    state: FilterState = await clients["filter"].request_reply(
        "/get_state", Empty(), decode=True
    )
    print(f"Current filter state:\n{state}")
    return Pose3F64.from_proto(state.pose)


async def set_track(clients: dict[str, EventClient], track: Track) -> None:
    await clients["track_follower"].request_reply(
        "/set_track", TrackFollowRequest(track=track)
    )


async def start(clients: dict[str, EventClient]) -> None:
    print("Starting track follower...")
    await clients["track_follower"].request_reply("/start", Empty())


async def pause_robot(clients: dict[str, EventClient]) -> None:
    print("Sending /pause...")
    await clients["track_follower"].request_reply("/pause", Empty())


async def resume_robot(clients: dict[str, EventClient]) -> None:
    print("Sending /resume...")
    await clients["track_follower"].request_reply("/resume", Empty())


# ---------------------------------------------------------------------------
# State monitoring
# ---------------------------------------------------------------------------

async def wait_for_status(
    clients: dict[str, EventClient],
    target_status: int,
    label: str = "",
) -> None:
    """Block until track_follower status.track_status == target_status."""
    print(f"Waiting for: {label} ...")
    async for _, message in clients["track_follower"].subscribe(
        SubscribeRequest(uri=Uri(path="/state"))
    ):
        current = message.status.track_status
        if current == target_status:
            print(f"  ✓ {label}")
            break


async def wait_for_waypoint_index(
    clients: dict[str, EventClient],
    target_index: int,
    metre_mark: float,
) -> None:
    """Block until closest_waypoint_index >= target_index."""
    print(f"Driving to {metre_mark:.0f} m mark (waypoint index ~{target_index})...")
    async for _, message in clients["track_follower"].subscribe(
        SubscribeRequest(uri=Uri(path="/state"))
    ):
        idx = message.progress.closest_waypoint_index
        remaining = message.progress.distance_remaining
        status = message.status.track_status
        print(f"  waypoint={idx}  |  remaining={remaining:.2f} m  |  status={status}")
        if status == TRACK_COMPLETE or idx >= target_index:
            break


# ---------------------------------------------------------------------------
# Waypoint builder
# ---------------------------------------------------------------------------

def create_straight_segment(
    previous_pose: Pose3F64,
    next_frame_b: str,
    distance: float,
    spacing: float = WAYPOINT_SPACING,
) -> list[Pose3F64]:
    """Straight waypoints spaced `spacing` metres apart."""
    segment_poses: list[Pose3F64] = [previous_pose]
    counter = 0
    remaining = distance

    while abs(remaining) > 0.001:
        seg_dist = min(abs(remaining), spacing) * (1 if distance >= 0 else -1)
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


def format_track(waypoints: list[Pose3F64]) -> Track:
    return Track(waypoints=[p.to_proto() for p in waypoints])


# ---------------------------------------------------------------------------
# GPS background task — runs continuously, saves every second
# ---------------------------------------------------------------------------

async def gps_logger_task(
    clients: dict[str, EventClient],
    stop_event: asyncio.Event,
) -> None:
    """Background task: reads GPS every second and saves to CSV.

    Columns saved:
        timestamp               — ISO format datetime
        relative_pose_north_m   — north position in metres
        relative_pose_east_m    — east position in metres
        robot_status            — 'DRIVING' or 'PAUSED'

    The status column is updated from the shared `gps_state` object.
    """
    csv_path = Path(GPS_OUTPUT_FILE)

    # Write CSV header
    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "timestamp",
            "relative_pose_north_m",
            "relative_pose_east_m",
            "robot_status",
        ])

    print(f"GPS logger started — saving to {csv_path.resolve()}")

    async for event, msg in clients["gps"].subscribe(
        clients["gps"].config.subscriptions[0]
    ):
        if stop_event.is_set():
            break

        if not isinstance(msg, gps_pb2.RelativePositionFrame):
            continue

        # Update shared state
        gps_state.north = msg.relative_pose_north
        gps_state.east = msg.relative_pose_east
        gps_state.updated = True

        timestamp = datetime.now().isoformat()
        status = "PAUSED" if gps_state.robot_paused else "DRIVING"

        print(
            f"  [GPS] {timestamp}  |  "
            f"north={gps_state.north:.4f} m  |  "
            f"east={gps_state.east:.4f} m  |  "
            f"status={status}"
        )

        with open(csv_path, "a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([timestamp, gps_state.north, gps_state.east, status])

        # Wait 1 second before next reading
        await asyncio.sleep(1.0)


# ---------------------------------------------------------------------------
# Main driving logic
# ---------------------------------------------------------------------------

async def drive_with_interval_pauses(clients: dict[str, EventClient]) -> None:
    """Drive 10 m, pausing 5 s at every 1 m mark while logging GPS every second.

    Flow
    ----
    1. Build the full 10 m track (100 waypoints at 0.1 m spacing).
    2. Start GPS logger as a background task.
    3. Upload track → /start.
    4. For each 1 m mark (indices 10, 20, 30 ... 100):
           a. Wait until robot reaches that waypoint.
           b. /pause → set status = PAUSED.
           c. Wait 5 s (GPS logger records during this time).
           d. /resume → set status = DRIVING.
    5. Wait for TRACK_COMPLETE.
    6. Stop GPS logger.
    """
    print(f"\n{'='*60}")
    print(f"Drive {TOTAL_DISTANCE} m  |  pause {PAUSE_DURATION:.0f} s every {PAUSE_INTERVAL:.0f} m")
    print(f"GPS logged every second → {GPS_OUTPUT_FILE}")
    print(f"Pause waypoint indices: {PAUSE_INDICES}")
    print(f"{'='*60}\n")

    # Add robot_paused flag to gps_state
    gps_state.robot_paused = False

    # ── 1. Build the full 10 m straight track ─────────────────────────────
    world_pose_robot = await get_pose(clients)

    anchor = world_pose_robot * Pose3F64(
        a_from_b=Isometry3F64(),
        frame_a="robot",
        frame_b="drive_start",
    )

    all_waypoints: list[Pose3F64] = [anchor]
    straight = create_straight_segment(all_waypoints[-1], "drive_end", TOTAL_DISTANCE)
    all_waypoints.extend(straight[1:])
    print(f"Track built: {len(all_waypoints)} waypoints.")

    # ── 2. Start GPS background logger ────────────────────────────────────
    stop_gps = asyncio.Event()
    gps_task = asyncio.create_task(gps_logger_task(clients, stop_gps))

    # ── 3. Upload and start ────────────────────────────────────────────────
    await set_track(clients, format_track(all_waypoints))
    await start(clients)
    gps_state.robot_paused = False

    # ── 4. Pause at every 1 m mark ────────────────────────────────────────
    for i, pause_idx in enumerate(PAUSE_INDICES):
        metre_mark = PAUSE_INTERVAL * (i + 1)

        # Last waypoint index = end of track → just wait for COMPLETE
        if metre_mark >= TOTAL_DISTANCE:
            print(f"\nFinal metre ({metre_mark:.0f} m) — waiting for TRACK_COMPLETE...")
            await wait_for_status(clients, TRACK_COMPLETE, "TRACK_COMPLETE")
            print(f"✓ Reached {metre_mark:.0f} m — drive complete.")
            break

        # Wait until robot reaches this 1 m mark
        await wait_for_waypoint_index(clients, pause_idx, metre_mark)

        # /pause
        await pause_robot(clients)
        await wait_for_status(clients, TRACK_PAUSED, "TRACK_PAUSED")
        gps_state.robot_paused = True
        print(f"\n*** PAUSED at {metre_mark:.0f} m — holding for {PAUSE_DURATION:.0f} s ***")

        # Wait PAUSE_DURATION seconds (GPS logger keeps running)
        elapsed = 0.0
        while elapsed < PAUSE_DURATION:
            await asyncio.sleep(1.0)
            elapsed += 1.0
            print(f"    Pause: {elapsed:.0f} / {PAUSE_DURATION:.0f} s")

        print(f"*** Resuming from {metre_mark:.0f} m ***\n")

        # /resume
        gps_state.robot_paused = False
        await resume_robot(clients)
        await wait_for_status(clients, TRACK_FOLLOWING, "TRACK_FOLLOWING")

    # ── 5. Stop GPS logger ─────────────────────────────────────────────────
    stop_gps.set()
    gps_task.cancel()
    try:
        await gps_task
    except asyncio.CancelledError:
        pass

    print(f"\n✓ All done. GPS data saved to: {Path(GPS_OUTPUT_FILE).resolve()}")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

async def run(args) -> None:
    clients: dict[str, EventClient] = {}
    expected_configs = ["track_follower", "filter", "gps"]

    config_list = proto_from_json_file(args.service_config, EventServiceConfigList())
    for config in config_list.configs:
        if config.name in expected_configs:
            clients[config.name] = EventClient(config)

    for name in expected_configs:
        if name not in clients:
            raise RuntimeError(
                f"No '{name}' service config found in {args.service_config}"
            )

    await drive_with_interval_pauses(clients)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        prog="python main.py",
        description=(
            "Drive 10 m forward, pausing 5 s at every 1 m mark. "
            "GPS data is logged every second to a CSV file."
        ),
    )
    parser.add_argument(
        "--service-config", type=Path, required=True,
        help="Service config JSON (must include track_follower, filter, gps).",
    )
    args = parser.parse_args()

    loop = asyncio.get_event_loop()
    loop.run_until_complete(run(args))