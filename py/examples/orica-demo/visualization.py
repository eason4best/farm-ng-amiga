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

from typing import Optional

import matplotlib.pyplot as plt
import numpy as np
from farm_ng.track.track_pb2 import Track
from farm_ng_core_pybind import Pose3F64


def unpack_track(track: Track) -> tuple[list[float], list[float], list[float]]:
    """Unpack x and y coordinates and heading from the waypoints for plotting.

    Args:
        track: Track object containing waypoints
    Returns:
        tuple[list[float], list[float], list[float]]: The x, y, and heading coordinates of the track waypoints.
    """

    x: list[float] = []
    y: list[float] = []
    heading: list[float] = []
    track_waypoints = [Pose3F64.from_proto(pose) for pose in track.waypoints]
    for pose in track_waypoints:
        x.append(pose.a_from_b.translation[0])
        y.append(pose.a_from_b.translation[1])
        heading.append(pose.a_from_b.rotation.log()[-1])
    print(f"Unpacked track with {len(x)} waypoints.")
    return (x, y, heading)


def plot_track(track: Track, current_pose: Optional[list[float]] = None) -> None:
    """Plot the track with optional current pose visualization.

    Args:
        track: Track object containing waypoints
        current_pose: Optional list [x, y, heading] representing current position
    """
    waypoints = unpack_track(track=track)
    x = waypoints[0]
    y = waypoints[1]
    headings = waypoints[2]

    # Calculate the arrow directions
    U = np.cos(headings)
    V = np.sin(headings)

    # Parameters for arrow plotting
    arrow_interval = 5  # Adjust this to change the frequency of arrows
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

    # Plot track start point
    plt.plot(x[0], y[0], marker="o", markersize=5, color='red')

    # Plot current pose if provided
    if current_pose is not None:
        print(f"Current pose for plotting: {current_pose}")
        current_x, current_y, current_heading = current_pose

        # Plot current position as a larger green circle
        plt.plot(current_x, current_y, marker="o", markersize=8, color='green')

        # Plot current heading as a larger arrow
        current_u = np.cos(current_heading)
        current_v = np.sin(current_heading)
        plt.quiver(
            current_x,
            current_y,
            current_u,
            current_v,
            angles='xy',
            scale_units='xy',
            scale=2.5,
            color='green',
            width=0.005,
        )

    plt.axis("equal")

    # Create legend elements
    legend_elements = [
        plt.Line2D([0], [0], color='orange', lw=2, label='Track'),
        plt.Line2D([0], [0], color='blue', lw=2, label='Track Heading'),
        plt.scatter([], [], color='red', marker='o', s=30, label='Start'),
    ]

    # Add current pose to legend if it exists
    # if current_pose is not None:
    #     legend_elements.append(plt.scatter([], [], color='green', marker='o', s=50, label='Current Pose'))

    plt.legend(handles=legend_elements)
    plt.show()
