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
import math
import time
from pathlib import Path

from farm_ng.canbus.canbus_pb2 import Twist2d
from farm_ng.core.event_client import EventClient
from farm_ng.core.event_service_pb2 import EventServiceConfig
from farm_ng.core.events_file_reader import proto_from_json_file
from numpy import clip

# Constants
SERVICE_CONFIG_PATH = Path(__file__).parent / "service_config.json"
MAX_LINEAR_VELOCITY_MPS = 5.0  # m/s
LINEAR_VELOCITY = 1.0  # m/s
"""
Challenge: Robot Field Operations Sequence

Your task is to create a script that will control a farm robot to:
1. Drive forward 1 meter without any tool actions
2. Drive forward 10 meters without tool action
3. Execute a turn around maneuver (180 degrees) with a radius of 1 meters

The template below provides the basic structure and helper methods.
You will need to implement the missing functionality and create the
sequence in the main() function.

Tips:
- Start by understanding each helper function and what it does
- Pay attention to how you calculate the time needed to complete each action
- For the arc turn, remember that angular velocity = linear velocity / radius
"""


async def send_twist_command(client: EventClient, linear_velocity: float, angular_velocity: float = 0.0) -> None:
    """Send a twist command to control robot movement.

    Args:
        client: The event client connected to the canbus service.
        linear_velocity: The linear velocity in m/s.
        angular_velocity: The angular velocity in rad/s (default: 0.0).
    """
    # TODO: Implement this function
    # 1. Create a Twist2d message
    # 2. Set the linear and angular velocities (make sure to clip the linear velocity)
    # 3. Send the command to the robot using the client
    pass


async def drive_distance(client: EventClient, distance: float, velocity: float) -> None:
    """Drive the robot a specific distance at a given velocity.

    Args:
        client: The event client connected to the canbus service.
        distance: The distance to drive in meters.
        velocity: The velocity to drive at in m/s.
    """
    # TODO: Implement this function
    # 1. Calculate the time needed to travel the specified distance
    # 2. Send continuous movement commands for the calculated duration
    # 3. Stop the robot when the distance has been covered
    pass


async def drive_arc(
    client: EventClient, radius: float, angle_radians: float, linear_velocity: float, direction: int = 1
) -> None:
    """Drive the robot in an arc with specified radius and angle.

    Args:
        client: The event client connected to the canbus service.
        radius: The radius of the arc in meters.
        angle_radians: The angle to turn in radians.
        linear_velocity: The linear velocity to drive at in m/s.
        direction: 1 for counterclockwise, -1 for clockwise.
    """

    # TODO: Implement this function
    # 1. Calculate the angular velocity based on the radius and linear velocity (w = v/r)
    # 2. Calculate the arc length (arc_length = radius * angle_radians)
    # 3. Calculate the time needed to travel the arc (time = arc_length / linear_velocity)
    # 4. Send continuous movement commands with both linear and angular velocities
    # 5. Stop the robot when the arc has been completed
    pass


async def main() -> None:
    """Run the combined sequence of robot movement and tool operation.

    This is the main function that will execute the complete sequence:
    1. Drive forward 1 meter (line up robot)
    2. Drive forward 10 meters
    3. Execute a pi turn with 1 meter radius
    """
    # Create a client to the canbus service
    config: EventServiceConfig = proto_from_json_file(SERVICE_CONFIG_PATH, EventServiceConfig())
    client: EventClient = EventClient(config)

    # TODO: Implement the sequence
    # Step 1: Drive forward 1 meter without tool action

    # Step 2: Drive forward 10 meters

    # Step 3: Make a turn around maneuver with radius of 1 meter

    print("\n--- Sequence completed ---")


if __name__ == "__main__":
    asyncio.run(main())
