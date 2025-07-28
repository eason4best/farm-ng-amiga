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
from math import atan2
from math import degrees
from math import pi
from math import sqrt
from pathlib import Path
from typing import Dict
from typing import Tuple

import matplotlib.pyplot as plt
import numpy as np
from farm_ng.core.event_client import EventClient
from farm_ng.core.event_service_pb2 import EventServiceConfig
from farm_ng.core.events_file_reader import proto_from_json_file
from farm_ng.filter.filter_pb2 import FilterState
from farm_ng.track.track_pb2 import Track
from farm_ng_core_pybind import Isometry3F64
from farm_ng_core_pybind import Pose3F64
from google.protobuf.empty_pb2 import Empty
from track_planner import TrackBuilder


class WaypointNavigator:
    """A class for navigating between waypoints loaded from a Track protobuf."""

    def __init__(self, waypoints_file: Path):
        """Initialize the WaypointNavigator with waypoints from Track JSON file.

        Args:
            waypoints_file: Path to JSON file containing Track protobuf data
        """
        self.waypoints_dict = self.load_waypoints_from_track(waypoints_file)
        self.current_waypoint_index = 1  # Start with waypoint 1

    def load_waypoints_from_track(self, waypoints_file: Path) -> Dict[int, Pose3F64]:
        """Load waypoints from Track JSON file and convert to dictionary.

        Args:
            waypoints_file: Path to the JSON file containing Track data

        Returns:
            Dictionary with integer keys (1, 2, 3, ...) and Pose3F64 values
        """
        try:
            # Load the Track protobuf from JSON
            track: Track = proto_from_json_file(waypoints_file, Track())

            # Convert to dictionary with integer keys
            waypoints_dict = {}
            for i, waypoint_proto in enumerate(track.waypoints, 1):
                waypoint_pose = Pose3F64.from_proto(waypoint_proto)
                waypoints_dict[i] = waypoint_pose

            print(f"Loaded {len(waypoints_dict)} waypoints from {waypoints_file}")
            for key, pose in waypoints_dict.items():
                x, y = pose.a_from_b.translation[0], pose.a_from_b.translation[1]
                heading = pose.a_from_b.rotation.log()[-1]
                # Convert heading to compass direction for clarity
                heading_deg = degrees(heading)
                if heading_deg < 0:
                    heading_deg += 360
                print(f"  Waypoint {key}: x={x:.2f}m (North), y={y:.2f}m (West), heading={heading_deg:.1f}°")

            return waypoints_dict

        except FileNotFoundError:
            print(f"Error: Waypoints file {waypoints_file} not found")
            return {}
        except Exception as e:
            print(f"Error loading waypoints: {e}")
            return {}

    def get_waypoint(self, index: int) -> Pose3F64 | None:
        """Get a specific waypoint by index.

        Args:
            index: Waypoint index (1, 2, 3, ...)

        Returns:
            Pose3F64 object or None if not found
        """
        return self.waypoints_dict.get(index)

    def get_next_waypoint(self) -> Tuple[int, Pose3F64] | None:
        """Get the next waypoint to visit.

        Returns:
            Tuple of (waypoint_index, waypoint_pose) or None if no more waypoints
        """
        if self.current_waypoint_index in self.waypoints_dict:
            return (self.current_waypoint_index, self.waypoints_dict[self.current_waypoint_index])
        return None

    def advance_to_next_waypoint(self) -> None:
        """Advance to the next waypoint in the sequence."""
        self.current_waypoint_index += 1

    def get_total_waypoints(self) -> int:
        """Get the total number of waypoints."""
        return len(self.waypoints_dict)


def calculate_distance_and_heading(start_pose: Pose3F64, target_pose: Pose3F64) -> Tuple[float, float]:
    """Calculate distance and required heading between two poses.

    Coordinate system:
    - X-axis: Forward/North (positive X = North)
    - Y-axis: Horizontal (positive Y = West, negative Y = East)
    - Heading: 0 = North, π/2 = West, π = South, -π/2 = East

    Args:
        start_pose: Starting pose
        target_pose: Target pose

    Returns:
        Tuple of (distance, heading_change_needed)
    """
    start_pos = start_pose.a_from_b.translation
    target_pos = target_pose.a_from_b.translation

    # Calculate distance
    dx = target_pos[0] - start_pos[0]  # North-South difference
    dy = target_pos[1] - start_pos[1]  # West-East difference
    distance = sqrt(dx**2 + dy**2)

    # Calculate heading to target
    # In our coordinate system: atan2(y, x) where x=North, y=West
    # This gives: 0=North, π/2=West, π=South, -π/2=East
    heading_to_target = atan2(dy, dx)

    # Get current heading
    current_heading = start_pose.a_from_b.rotation.log()[-1]

    # Calculate heading change needed
    heading_change = heading_to_target - current_heading

    # Normalize heading change to [-pi, pi]
    while heading_change > pi:
        heading_change -= 2 * pi
    while heading_change < -pi:
        heading_change += 2 * pi

    return distance, heading_change


def create_straight_track_to_waypoint(
    start_pose: Pose3F64, target_pose: Pose3F64, waypoint_index: int, spacing: float = 0.1
) -> Track:
    """Create a straight track from current pose to target waypoint.

    Args:
        start_pose: Current robot pose
        target_pose: Target waypoint pose
        waypoint_index: Index of the target waypoint
        spacing: Spacing between track points

    Returns:
        Track object representing the straight path to the waypoint
    """
    # Calculate distance and heading change
    distance, heading_change = calculate_distance_and_heading(start_pose, target_pose)

    print(f"Creating straight track to waypoint {waypoint_index}:")
    print(f"  Distance: {distance:.2f}m")
    print(f"  Heading change: {degrees(heading_change):.1f}°")
    print(
        f"  Direction: dx={target_pose.a_from_b.translation[0] - start_pose.a_from_b.translation[0]:.2f}m (North), "
        f"dy={target_pose.a_from_b.translation[1] - start_pose.a_from_b.translation[1]:.2f}m (West)"
    )

    # Create track builder
    track_builder = TrackBuilder(start=start_pose)

    # First, turn to face the target if needed (threshold of 5 degrees)
    if abs(heading_change) > np.radians(5):
        track_builder.create_turn_segment(
            next_frame_b=f"turn_to_waypoint_{waypoint_index}",
            angle=heading_change,
            spacing=0.05,  # Smaller spacing for turns
        )

    # Then drive straight to the target
    track_builder.create_straight_segment(next_frame_b=f"waypoint_{waypoint_index}", distance=distance, spacing=spacing)

    # Optionally, turn to match the target pose's heading
    target_heading = target_pose.a_from_b.rotation.log()[-1]
    current_heading_after_approach = start_pose.a_from_b.rotation.log()[-1] + heading_change
    final_turn = target_heading - current_heading_after_approach

    # Normalize final turn
    while final_turn > pi:
        final_turn -= 2 * pi
    while final_turn < -pi:
        final_turn += 2 * pi

    if abs(final_turn) > np.radians(5):
        track_builder.create_turn_segment(
            next_frame_b=f"waypoint_{waypoint_index}_final", angle=final_turn, spacing=0.05
        )

    return track_builder.track


def plot_waypoints_and_track(
    waypoints_dict: Dict[int, Pose3F64], track: Track = None, current_waypoint: int = None
) -> None:
    """Plot waypoints and optionally a track.

    Args:
        waypoints_dict: Dictionary of waypoints
        track: Optional track to plot
        current_waypoint: Index of current target waypoint to highlight
    """
    plt.figure(figsize=(12, 10))

    # Plot waypoints
    for i, (index, pose) in enumerate(waypoints_dict.items()):
        x, y = pose.a_from_b.translation[0], pose.a_from_b.translation[1]
        heading = pose.a_from_b.rotation.log()[-1]

        # Choose color based on whether this is the current target
        color = 'red' if index == current_waypoint else 'blue'
        size = 150 if index == current_waypoint else 100

        # Plot waypoint as a circle
        plt.scatter(
            x,
            y,
            s=size,
            c=color,
            marker='o',
            label=(
                'Current Target'
                if index == current_waypoint and i == 0
                else 'Waypoints'
                if index != current_waypoint and i == 0
                else ""
            ),
        )

        # Plot heading arrow
        arrow_length = 1.0
        dx = arrow_length * np.cos(heading)
        dy = arrow_length * np.sin(heading)
        plt.arrow(x, y, dx, dy, head_width=0.3, head_length=0.2, fc=color, ec=color, alpha=0.7)

        # Label the waypoint
        plt.annotate(
            f'WP{index}',
            (x, y),
            xytext=(8, 8),
            textcoords='offset points',
            fontweight='bold' if index == current_waypoint else 'normal',
        )

    # Plot track if provided
    if track is not None:
        track_waypoints = [Pose3F64.from_proto(pose) for pose in track.waypoints]
        x_track = [pose.a_from_b.translation[0] for pose in track_waypoints]
        y_track = [pose.a_from_b.translation[1] for pose in track_waypoints]
        plt.plot(x_track, y_track, 'g-', linewidth=3, label='Planned Track', alpha=0.8)

        # Mark start and end of track
        if len(x_track) > 0:
            plt.scatter(
                x_track[0],
                y_track[0],
                s=200,
                c='green',
                marker='s',
                label='Track Start',
                edgecolors='black',
                linewidth=2,
            )
            plt.scatter(
                x_track[-1],
                y_track[-1],
                s=200,
                c='orange',
                marker='^',
                label='Track End',
                edgecolors='black',
                linewidth=2,
            )

    plt.axis('equal')
    plt.grid(True, alpha=0.3)
    plt.xlabel('X (m) - North →')
    plt.ylabel('Y (m) - West →')
    plt.title('Waypoint Navigation (X=North, Y=West)')
    plt.legend()
    plt.show()


def plot_track(waypoints: list[list[float]]) -> None:
    """Original plot function for compatibility."""
    x = waypoints[0]
    y = waypoints[1]
    headings = waypoints[2]

    # Calculate the arrow directions
    U = np.cos(headings)
    V = np.sin(headings)

    # Parameters for arrow plotting
    arrow_interval = 20  # Adjust this to change the frequency of arrows
    turn_threshold = np.radians(10)  # Threshold in radians for when to skip plotting

    plt.figure(figsize=(8, 8))
    plt.plot(x, y, color='orange', linewidth=1.0)

    for i in range(0, len(x), arrow_interval):
        # Calculate the heading change
        if i > 0:
            heading_change = np.abs(headings[i] - headings[i - 1])
        else:
            heading_change = 0

        # Plot the arrow if the heading change is below the threshold
        if heading_change < turn_threshold:
            plt.quiver(x[i], y[i], U[i], V[i], angles='xy', scale_units='xy', scale=3.5, color='blue')

    plt.plot(x[0], y[0], marker="o", markersize=5, color='red')
    plt.axis("equal")
    legend_elements = [
        plt.Line2D([0], [0], color='orange', lw=2, label='Track'),
        plt.Line2D([0], [0], color='blue', lw=2, label='Heading'),
        plt.scatter([], [], color='red', marker='o', s=30, label='Start'),
    ]
    plt.legend(handles=legend_elements)
    plt.show()


async def create_start_pose(client: EventClient | None = None, timeout: float = 0.5) -> Pose3F64:
    """Create a start pose for the track.

    Args:
        client: A EventClient for the required service (filter)
    Returns:
        The start pose (Pose3F64)
    """
    print("Getting current robot position...")

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
            x, y = start.a_from_b.translation[0], start.a_from_b.translation[1]
            heading = start.a_from_b.rotation.log()[-1]
            heading_deg = degrees(heading)
            if heading_deg < 0:
                heading_deg += 360
            print(f"Current position: x={x:.2f}m (North), y={y:.2f}m (West), heading={heading_deg:.1f}°")
        except asyncio.TimeoutError:
            print("Timeout while getting filter state. Using default start pose.")
        except Exception as e:
            print(f"Error getting filter state: {e}. Using default start pose.")

    return start


async def navigate_waypoints(
    waypoints_file: Path, client: EventClient | None = None, save_tracks: bool = False, visualize: bool = True
) -> None:
    """Navigate through all waypoints in sequence.

    Args:
        waypoints_file: Path to JSON file containing Track with waypoints
        client: EventClient for getting robot position
        save_tracks: Whether to save tracks to files
        visualize: Whether to show visualization
    """
    # Initialize waypoint navigator
    navigator = WaypointNavigator(waypoints_file)

    if not navigator.waypoints_dict:
        print("No waypoints loaded. Exiting.")
        return

    # Get current robot position
    current_pose = await create_start_pose(client)

    # Visualize all waypoints
    if visualize:
        plot_waypoints_and_track(navigator.waypoints_dict)

    # Process each waypoint
    while True:
        next_waypoint = navigator.get_next_waypoint()
        if next_waypoint is None:
            print("\n🎉 All waypoints completed!")
            break

        waypoint_index, target_pose = next_waypoint
        print(f"\n--- Processing waypoint {waypoint_index}/{navigator.get_total_waypoints()} ---")

        # Create straight track to this waypoint
        track = create_straight_track_to_waypoint(current_pose, target_pose, waypoint_index)

        # Save track if requested
        if save_tracks:
            track_file = Path(f"track_to_waypoint_{waypoint_index}.json")
            track_builder = TrackBuilder()
            track_builder.track = track
            track_builder.save_track(track_file)

        # Visualize this specific track
        if visualize:
            plot_waypoints_and_track(navigator.waypoints_dict, track, waypoint_index)

        # Update current pose to the target waypoint pose
        # (In real implementation, you would wait for the robot to reach the waypoint)
        current_pose = target_pose

        # Advance to next waypoint
        navigator.advance_to_next_waypoint()

        # For demo purposes, ask user to continue
        # In real implementation, you would send the track to the robot and wait
        try:
            input("Press Enter to continue to next waypoint (or Ctrl+C to stop)...")
        except KeyboardInterrupt:
            print("\n\n🛑 Navigation stopped by user.")
            break


async def run(args) -> None:
    """Main function."""
    client: EventClient | None = None

    if args.service_config is not None:
        client = EventClient(proto_from_json_file(args.service_config, EventServiceConfig()))
        if client is None:
            raise RuntimeError(f"No filter service config in {args.service_config}")
        if client.config.name != "filter":
            raise RuntimeError(f"Expected filter service in {args.service_config}, got {client.config.name}")

    if args.waypoints is not None:
        if not args.waypoints.exists():
            raise RuntimeError(f"Waypoints file {args.waypoints} does not exist.")

        await navigate_waypoints(
            waypoints_file=args.waypoints, client=client, save_tracks=args.save_tracks, visualize=not args.no_plot
        )
    else:
        print("Please provide a waypoints file using --waypoints argument.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        prog="python waypoint_navigation.py", description="Navigate through waypoints from Track protobuf."
    )
    parser.add_argument(
        "--waypoints", type=Path, required=True, help="Path to JSON file containing Track with waypoints"
    )
    parser.add_argument("--save-tracks", action='store_true', help="Save individual tracks to files")
    parser.add_argument("--no-plot", action='store_true', help="Disable visualization")
    parser.add_argument("--service-config", type=Path, help="Path to the service config file")

    args = parser.parse_args()

    # Create the asyncio event loop and run the main function
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run(args))
