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

import asyncio

import numpy as np
from end_of_row_maneuver import build_row_end_maneuver
from farm_ng.core.event_client import EventClient
from farm_ng.filter.filter_pb2 import FilterState
from farm_ng.track.track_pb2 import Track
from farm_ng_core_pybind import Isometry3F64
from farm_ng_core_pybind import Pose3F64
from google.protobuf.empty_pb2 import Empty
from track_planner import TrackBuilder


async def get_current_pose(client: EventClient | None = None, timeout: float = 0.5) -> Pose3F64:
    """Get the current pose for the track.

    Args:
        client: A EventClient for the required service (filter)
    Returns:
        The start pose (Pose3F64)
    """
    print("Creating start pose...")

    zero_tangent = np.zeros((6, 1), dtype=np.float64)
    start: Pose3F64 = Pose3F64(
        a_from_b=Isometry3F64(), frame_a="world", frame_b="robot", tangent_of_b_in_a=zero_tangent
    )
    if client is not None:
        try:
            # Get the current state of the filter
            state: FilterState = await asyncio.wait_for(
                client.request_reply("/get_state", Empty(), decode=True), timeout=timeout
            )
            start = Pose3F64.from_proto(state.pose)
        except asyncio.TimeoutError:
            print("Timeout while getting filter state. Using default start pose.")
        except Exception as e:
            print(f"Error getting filter state: {e}. Using default start pose.")

    return start


async def create_track_to_next_waypoint(goal_pose: Pose3F64, client: EventClient | None = None) -> Track:
    """Builds a custom track for the Amiga to follow.

    Args:
        client: A EventClient for the required service (filter)
        goal_pose: The pose to navigate to
    Returns:
        The track
    """

    if client is None:
        raise RuntimeError("EventClient cannot be None")

    print("Building track...")

    current_pose: Pose3F64 = await get_current_pose(client)
    track_builder = TrackBuilder(start=current_pose)

    # Drive forward
    track_builder.create_ab_segment(next_frame_b="goal_pose", final_pose=goal_pose, spacing=0.1)

    # Print the number of waypoints in the track
    print(f" Track created with {len(track_builder.track_waypoints)} waypoints")

    # Plot the track
    return track_builder.track


async def create_track_to_next_row_waypoint(
    goal_pose: Pose3F64,
    buffer_distance: float = 2.5,
    row_spacing: float = 6.0,
    direction: str = "left",
    client: EventClient | None = None,
) -> Track:
    """Builds a custom track for the Amiga to follow to the next row waypoint.

    Args:
        client: A EventClient for the required service (filter)
        goal_pose: The pose to navigate to
        row_spacing: The distance between rows (in meters)
        buffer_distance: The distance to drive forward before turning (in meters)
        direction: The direction to turn at the end of the row, either "left" or "right"
    Returns:
        The track
    """

    if client is None:
        raise RuntimeError("EventClient cannot be None")

    print("Building track...")

    # Create track that represents the maneuver to the next row waypoint
    track: Track = await build_row_end_maneuver(
        client, buffer_distance=buffer_distance, row_spacing=row_spacing, direction=direction
    )

    track_builder = TrackBuilder(start=None)
    track_builder.track = track

    # Drive to the first waypoint in this next row
    track_builder.create_ab_segment(next_frame_b="goal_pose", final_pose=goal_pose, spacing=0.1)

    # Print the number of waypoints in the track
    print(f" Track created with {len(track_builder.track_waypoints)} waypoints")

    # Plot the track
    return track_builder.track
