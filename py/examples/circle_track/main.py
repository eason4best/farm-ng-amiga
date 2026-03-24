"""Example using the track_follower service to drive a circle."""
# Copyright (c) farm-ng, inc.
#
# Licensed under the Amiga Development Kit License (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://github.com/farm-ng/amiga-dev-kit/blob/main/LICENSE
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
from __future__ import annotations

import argparse
import asyncio
from math import copysign
from math import cos
from math import pi
from math import sin
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


async def get_pose(clients: dict[str, EventClient]) -> Pose3F64:
    """Get the current pose of the robot in the world frame, from the filter service.

    Args:
        clients (dict[str, EventClient]): A dictionary of EventClients.
    """
    state: FilterState = await clients["filter"].request_reply("/get_state", Empty(), decode=True)
    print(f"Current filter state:\n{state}")
    return Pose3F64.from_proto(state.pose)


async def set_track(clients: dict[str, EventClient], track: Track) -> None:
    """Set the track of the track_follower.

    Args:
        clients (dict[str, EventClient]): A dictionary of EventClients.
        track (Track): The track for the track_follower to follow.
    """
    print(f"Setting track:\n{track}")
    await clients["track_follower"].request_reply("/set_track", TrackFollowRequest(track=track))


async def start(clients: dict[str, EventClient]) -> None:
    """Request to start following the track.

    Args:
        clients (dict[str, EventClient]): A dictionary of EventClients.
    """
    print("Sending request to start following the track...")
    await clients["track_follower"].request_reply("/start", Empty())


def create_arc_segment(
    previous_pose: Pose3F64,
    next_frame_b: str,
    radius: float,
    total_angle: float,
    spacing: float = 0.1,
) -> list[Pose3F64]:
    """Compute a circular arc segment as a list of waypoint poses.

    The arc is computed by chaining small incremental pose steps.
    Each step moves the robot forward along the circle and rotates
    it by a small angle, matching the true arc geometry.

    Arc geometry per step (angle = segment_angle, signed):
        dx  = radius * sin(|segment_angle|)         -- always forward (+x)
        dy  = sign(segment_angle) * radius * (1 - cos(|segment_angle|))
                                                     -- left (+y) for CCW, right (-y) for CW
        dθ  = segment_angle                          -- heading change

    Args:
        previous_pose (Pose3F64): The pose at the start of this arc.
        next_frame_b (str): Child frame name for the final waypoint of this arc.
        radius (float): Radius of the circle, in meters.
        total_angle (float): Total arc angle in radians.
                             Positive = counter-clockwise, negative = clockwise.
        spacing (float): Angular step size per waypoint, in radians.

    Returns:
        list[Pose3F64]: Ordered list of poses along the arc (includes previous_pose).
    """
    segment_poses: list[Pose3F64] = [previous_pose]
    counter: int = 0
    remaining_angle: float = total_angle

    while abs(remaining_angle) > 1e-4:
        # Signed angular step — capped by spacing
        segment_angle: float = copysign(min(abs(remaining_angle), spacing), total_angle)
        abs_angle: float = abs(segment_angle)

        # Exact arc-chord geometry in the robot's current frame
        dx: float = radius * sin(abs_angle)
        dy: float = copysign(radius * (1.0 - cos(abs_angle)), segment_angle)

        arc_step = Pose3F64(
            a_from_b=Isometry3F64([dx, dy, 0.0], Rotation3F64.Rz(segment_angle)),
            frame_a=segment_poses[-1].frame_b,
            frame_b=f"{next_frame_b}_{counter}",
        )
        segment_poses.append(segment_poses[-1] * arc_step)

        counter += 1
        remaining_angle -= segment_angle

    # Give the last waypoint the desired frame name
    segment_poses[-1].frame_b = next_frame_b
    return segment_poses


async def build_circle(
    clients: dict[str, EventClient],
    radius: float,
    clockwise: bool,
    num_laps: int = 1,
) -> Track:
    """Build a circular track starting from the robot's current pose.

    The robot begins at its current position (which becomes the first
    waypoint on the circle's perimeter) and traces one or more full
    loops before returning to the start.

    Geometry
    --------
    For a counter-clockwise circle of radius *r* the circle centre is
    offset *r* metres to the **left** of the robot's current heading.
    For clockwise the centre is *r* metres to the **right**.

    Args:
        clients (dict[str, EventClient]): A dictionary of EventClients.
        radius (float): Radius of the circle, in metres. Must be > 0.
        clockwise (bool): Drive clockwise (right-hand turns) when True;
                          counter-clockwise (left-hand turns) when False.
        num_laps (int): Number of complete loops to include in the track.

    Returns:
        Track: A Track proto message containing the computed waypoints.
    """
    if radius <= 0:
        raise ValueError(f"radius must be positive, got {radius}")
    if num_laps < 1:
        raise ValueError(f"num_laps must be >= 1, got {num_laps}")

    # Query the filter for the robot's current pose in the world frame
    world_pose_robot: Pose3F64 = await get_pose(clients)

    # Signed total arc angle for one full revolution
    #   positive → counter-clockwise,  negative → clockwise
    full_circle: float = -2.0 * pi if clockwise else 2.0 * pi

    track_waypoints: list[Pose3F64] = []

    # First waypoint: robot's current pose (the circle start/end point)
    world_pose_goal0 = world_pose_robot * Pose3F64(
        a_from_b=Isometry3F64(),
        frame_a="robot",
        frame_b="goal0",
    )
    track_waypoints.append(world_pose_goal0)

    # Generate waypoints for each requested lap
    for lap in range(num_laps):
        goal_label = f"lap{lap + 1}_end"
        arc_poses = create_arc_segment(
            previous_pose=track_waypoints[-1],
            next_frame_b=goal_label,
            radius=radius,
            total_angle=full_circle,
        )
        # arc_poses[0] duplicates the previous pose — skip it
        track_waypoints.extend(arc_poses[1:])

    print(
        f"Circle track built: radius={radius} m, "
        f"{'clockwise' if clockwise else 'counter-clockwise'}, "
        f"{num_laps} lap(s), {len(track_waypoints)} waypoints total."
    )
    return format_track(track_waypoints)


def format_track(track_waypoints: list[Pose3F64]) -> Track:
    """Pack the track waypoints into a Track proto message.

    Args:
        track_waypoints (list[Pose3F64]): The ordered track waypoints.

    Returns:
        Track: The serialised Track proto.
    """
    return Track(waypoints=[pose.to_proto() for pose in track_waypoints])


async def start_track(
    clients: dict[str, EventClient],
    radius: float,
    clockwise: bool,
    num_laps: int,
) -> None:
    """Build the circle track, upload it, and start the track follower.

    Args:
        clients (dict[str, EventClient]): A dictionary of EventClients.
        radius (float): Circle radius in metres.
        clockwise (bool): True for clockwise, False for counter-clockwise.
        num_laps (int): Number of complete laps to drive.
    """
    track: Track = await build_circle(clients, radius, clockwise, num_laps)
    await set_track(clients, track)
    await start(clients)


async def stream_track_state(clients: dict[str, EventClient]) -> None:
    """Stream and print the track_follower state.

    Args:
        clients (dict[str, EventClient]): A dictionary of EventClients.
    """
    # Brief pause so the initial track upload is visible in the log
    await asyncio.sleep(1.0)

    message: TrackFollowerState
    async for _, message in clients["track_follower"].subscribe(
        SubscribeRequest(uri=Uri(path="/state"))
    ):
        print("###################")
        print(message)


async def run(args) -> None:
    """Entry point: create clients and launch asyncio tasks."""
    clients: dict[str, EventClient] = {}
    expected_configs = ["track_follower", "filter"]

    config_list = proto_from_json_file(args.service_config, EventServiceConfigList())
    for config in config_list.configs:
        if config.name in expected_configs:
            clients[config.name] = EventClient(config)

    for config in expected_configs:
        if config not in clients:
            raise RuntimeError(f"No {config} service config in {args.service_config}")

    tasks: list[asyncio.Task] = [
        asyncio.create_task(
            start_track(clients, args.radius, args.clockwise, args.num_laps)
        ),
        asyncio.create_task(stream_track_state(clients)),
    ]
    await asyncio.gather(*tasks)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        prog="python main.py",
        description="Amiga track_follower circle example.",
    )
    parser.add_argument(
        "--service-config",
        type=Path,
        required=True,
        help="Path to the service config JSON file.",
    )
    parser.add_argument(
        "--radius",
        type=float,
        default=2.0,
        help="Radius of the circle in metres (default: 2.0).",
    )
    parser.add_argument(
        "--clockwise",
        action="store_true",
        help="Drive clockwise (right-hand turns). Default is counter-clockwise.",
    )
    parser.add_argument(
        "--num-laps",
        type=int,
        default=1,
        help="Number of complete laps to drive (default: 1).",
    )
    args = parser.parse_args()

    loop = asyncio.get_event_loop()
    loop.run_until_complete(run(args))