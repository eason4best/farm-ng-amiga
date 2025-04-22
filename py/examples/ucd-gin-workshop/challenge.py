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
from farm_ng.canbus.tool_control_pb2 import ActuatorCommands
from farm_ng.canbus.tool_control_pb2 import HBridgeCommand
from farm_ng.canbus.tool_control_pb2 import HBridgeCommandType
from farm_ng.core.event_client import EventClient
from farm_ng.core.event_service_pb2 import EventServiceConfig
from farm_ng.core.events_file_reader import proto_from_json_file
from numpy import clip

# Constants
SERVICE_CONFIG_PATH = Path(__file__).parent / "service_config.json"
MAX_LINEAR_VELOCITY_MPS = 5.0  # m/s
LINEAR_VELOCITY = 1.0  # m/s
HBRIDGE_ID = 0

"""
Challenge: Robot Field Operations Sequence

Your task is to create a script that will control a farm robot to:
1. Drive forward 1 meter without any tool actions
2. Stop and operate a tool in REVERSE direction for 10 seconds
3. Drive forward 10 meters without tool action
4. Stop and operate the tool in FORWARD direction for 10 seconds
5. BIG STRETCH: Execute a turn around maneuver (180 degrees) with a radius of 2 meters

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


async def send_tool_command(client: EventClient, direction: str) -> None:
    """Send a tool control command.

    Args:
        client: The event client connected to the canbus service.
        direction: The direction to move the tool ("FORWARD" or "REVERSE").
    """
    # TODO: Implement this function
    # 1. Convert the direction string to the appropriate HBridgeCommandType
    # 2. Create an ActuatorCommands message with the appropriate HBridgeCommand
    # 3. Send the command to the tool controller using the client
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


async def operate_tool(client: EventClient, direction: str, duration: float) -> None:
    """Operate the tool in a specific direction for a duration.

    Args:
        client: The event client connected to the canbus service.
        direction: The direction to move the tool ("FORWARD" or "REVERSE").
        duration: The duration to operate the tool in seconds.
    """

    # TODO: Implement this function
    # 1. Send continuous tool control commands for the specified duration
    # 2. Stop the tool when the duration has elapsed
    pass


async def main() -> None:
    """Run the combined sequence of robot movement and tool operation.

    This is the main function that will execute the complete sequence:
    1. Drive forward 1 meter
    2. Operate tool in REVERSE for 10 seconds
    3. Drive forward 10 meters
    4. Operate tool in FORWARD for 10 seconds
    5. Execute a pi turn with 2 meter radius
    """
    # Create a client to the canbus service
    config: EventServiceConfig = proto_from_json_file(SERVICE_CONFIG_PATH, EventServiceConfig())
    client: EventClient = EventClient(config)

    # TODO: Implement the sequence
    # Step 1: Drive forward 1 meter without tool action

    # Step 2: Stop robot and operate tool in REVERSE for 10 seconds

    # Step 3: Drive forward 10 meters without tool action

    # Step 4: Stop robot and operate tool in FORWARD for 10 seconds

    # Step 5 (stretch): Make a turn around maneuver with radius of 2 meters

    print("\n--- Sequence completed ---")


if __name__ == "__main__":
    asyncio.run(main())
