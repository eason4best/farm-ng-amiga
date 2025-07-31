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
from math import radians
from pathlib import Path
from typing import Dict
from typing import Optional
from typing import Tuple

from farm_ng.core.event_client import EventClient
from farm_ng.core.events_file_reader import proto_from_json_file
from farm_ng.filter.filter_pb2 import FilterState
from farm_ng.track.track_pb2 import Track
from farm_ng_core_pybind import Pose3F64
from google.protobuf.empty_pb2 import Empty
from track_planner import TrackBuilder


async def get_current_pose(client: EventClient | None = None, timeout: float = 5.0) -> Optional[Pose3F64]:
    """Get the current pose for the track.

    Args:
        client: A EventClient for the required service (filter)
    Returns:
        The current pose (Pose3F64) if available, otherwise None.
    """

    if client is not None:
        try:
            # Get the current state of the filter
            state: FilterState = await asyncio.wait_for(
                client.request_reply("/get_state", Empty(), decode=True), timeout=timeout
            )
            return Pose3F64.from_proto(state.pose)
        except asyncio.TimeoutError:
            print("Timeout while getting filter state. Using default start pose.")
        except Exception as e:
            print(f"Error getting filter state: {e}. Using default start pose.")

    return None


class MotionPlanner:
    """A class to handle motion planning for the Amiga."""

    def __init__(
        self,
        client: EventClient,
        waypoints_path: Path | str,
        last_row_waypoint_index: int,
        turn_direction: str,
        row_spacing: float,
        headland_buffer: float,
    ):
        self.client = client
        self.waypoints: Dict[int, Pose3F64] = {}
        self.last_row_waypoint_index = last_row_waypoint_index
        self.row_spacing = row_spacing
        self.headland_buffer = headland_buffer
        self.current_waypoint_index = 0
        self.current_pose: Optional[Pose3F64] = None
        self.pose_query_task: asyncio.Task | None = None
        self.should_poll: bool = True
        self.row_end_segment_index: int = 1  # Track if we have finished all row end maneuvers (total of 5)
        if turn_direction not in ["left", "right"]:
            raise ValueError("turn_direction must be either 'left' or 'right'")
        self.turn_angle_sign: float = 1.0 if turn_direction == "left" else -1.0

        if not isinstance(waypoints_path, Path):
            waypoints_path = Path(waypoints_path)
        try:
            # Load the Track protobuf from JSON
            track: Track = proto_from_json_file(waypoints_path, Track())

            # Convert to dictionary with integer keys
            waypoints_dict = {}
            for i, waypoint_proto in enumerate(track.waypoints, 1):
                waypoint_pose = Pose3F64.from_proto(waypoint_proto)
                waypoints_dict[i] = waypoint_pose
        except Exception as e:
            raise RuntimeError(f"Failed to load waypoints from {waypoints_path}: {e}")

        self.waypoints = waypoints_dict

        self.pose_query_task = asyncio.create_task(self._update_current_pose())

    async def _update_current_pose(self):
        """Update the current pose from the filter."""
        if self.client is None:
            raise RuntimeError("EventClient cannot be None")

        while self.should_poll:
            try:
                maybe_current_pose = await get_current_pose(self.client)
                if maybe_current_pose is not None:
                    self.current_pose = maybe_current_pose
                else:
                    print("Current pose is None, ensure your filter is running.")
            except Exception as e:
                print(f"Error updating current pose: {e}")
                return None

    async def _get_current_pose(self) -> Pose3F64:
        """Get the current pose of the Amiga.

        NOTE: This will block until the pose is available.
        Returns:
            The current pose (Pose3F64)
        """
        current_pose = None
        while current_pose is None:
            current_pose = self.current_pose  # should be updated by the background task
            await asyncio.sleep(0.5)  # Wait for the pose to be updated

        return current_pose

    async def _create_ab_segment_to_next_waypoint(self) -> Track:
        """Create an AB segment to the next waypoint.

        Returns:
            The track segment to the next waypoint (Track)
        """
        # 1. Ensure we have the current pose
        current_pose = await self._get_current_pose()

        # 2. Create the track (AB) segment to the next waypoint
        track_builder = TrackBuilder(start=current_pose)
        self.current_waypoint_index += 1
        track_builder.create_ab_segment(
            next_frame_b=f"waypoint_{self.current_waypoint_index}",
            final_pose=self.waypoints[self.current_waypoint_index],
            spacing=0.1,
        )
        return track_builder.track

    async def _row_end_maneuver(self, index: int) -> Track:
        """Create a row end maneuver segment based on the index.

        Args:
            index: The index of the row end maneuver (1 to 5)
        Returns:
            The track segment for the row end maneuver (Track)
        """
        if index < 1 or index > 5:
            raise ValueError("index must be between 1 and 5")

        # Create a turn segment based on the index
        current_pose = await self._get_current_pose()
        track_builder = TrackBuilder(start=current_pose)
        track_segment: Track
        next_frame_b = f"row_end_{index}"
        if index == 1 or index == 5:
            # Drive forward – move away from the last hole into a buffer zone,
            # or re-enter the row, ending parallel to the last hole in row 1.
            track_builder.create_straight_segment(next_frame_b=next_frame_b, distance=self.headland_buffer, spacing=0.1)
            track_segment = track_builder.track
        elif index == 2 or index == 4:
            # Turn 90° – reorient the robot toward the next row.
            track_builder.create_turn_segment(next_frame_b=next_frame_b, angle=radians(90 * self.turn_angle_sign))
            track_segment = track_builder.track
        else:
            # Drive forward – cross the row spacing gap.
            track_builder.create_straight_segment(next_frame_b=next_frame_b, distance=self.row_spacing, spacing=0.1)
            track_segment = track_builder.track

        return track_segment

    async def _shutdown(self):
        """Shutdown the motion planner."""
        if self.pose_query_task is not None:
            self.should_poll = False
            await self.pose_query_task
            self.pose_query_task = None

    async def next_track_segment(self) -> Tuple[Optional[Track], Optional[str]]:
        """Get the next track segment to navigate to.

        Returns:
            The next track segment (Track)
        """
        if self.current_waypoint_index >= len(self.waypoints):
            print("No more waypoints to navigate to.")
            asyncio.create_task(self._shutdown())
            return (None, None)

        # Check if we're switching to the next row or just moving to the next waypoint
        if self.current_waypoint_index != self.last_row_waypoint_index:
            # We're not transitioning to a new row, we will just create an AB segment to the next waypoint
            curr_index = self.current_waypoint_index
            track = await self._create_ab_segment_to_next_waypoint()
            next_index = self.current_waypoint_index
            seg_name = f"waypoint_{curr_index}_to_{next_index}"
            return (track, seg_name)

        # We're switching to the next row
        # 1. Check if we have finished all row end maneuvers
        if self.row_end_segment_index >= 5:
            print("Finished all row end maneuvers, moving to the next row.")
            seg_name = f"row_end_5_to_waypoint_{self.current_waypoint_index + 1}"
            return (await self._create_ab_segment_to_next_waypoint(), seg_name)
        else:
            # We need to return a segment from the row end maneuver
            track_segment = await self._row_end_maneuver(self.row_end_segment_index)
            self.row_end_segment_index += 1
            return (track_segment, f"row_end_{self.row_end_segment_index}")
