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


async def send_twist_command(client: EventClient, linear_velocity: float, angular_velocity: float = 0.0) -> None:
    """Send a twist command to control robot movement.

    Args:
        client: The event client connected to the canbus service.
        linear_velocity: The linear velocity in m/s.
        angular_velocity: The angular velocity in rad/s (default: 0.0).
    """
    # Create and clip the twist command
    twist = Twist2d()
    twist.linear_velocity_x = clip(linear_velocity, -MAX_LINEAR_VELOCITY_MPS, MAX_LINEAR_VELOCITY_MPS)
    twist.angular_velocity = angular_velocity

    print(f"Sending linear velocity: {twist.linear_velocity_x:.3f}, angular velocity: {twist.angular_velocity:.3f}")
    await client.request_reply("/twist", twist)


async def send_tool_command(client: EventClient, direction: str) -> None:
    """Send a tool control command.

    Args:
        client: The event client connected to the canbus service.
        direction: The direction to move the tool ("FORWARD" or "REVERSE").
    """
    if direction == "FORWARD":
        command_type = HBridgeCommandType.HBRIDGE_FORWARD
    elif direction == "REVERSE":
        command_type = HBridgeCommandType.HBRIDGE_REVERSE
    elif direction == "STOP":
        command_type = HBridgeCommandType.HBRIDGE_STOPPED
    else:
        raise ValueError(f"Invalid tool direction: {direction}. Use 'FORWARD', 'REVERSE', or 'STOP'.")

    command = ActuatorCommands()
    command.hbridges.append(HBridgeCommand(id=HBRIDGE_ID, command=command_type))

    print(f"Sending tool control command: {command}")
    await client.request_reply("/control_tools", command)


async def drive_distance(client: EventClient, distance: float, velocity: float) -> None:
    """Drive the robot a specific distance at a given velocity.

    Args:
        client: The event client connected to the canbus service.
        distance: The distance to drive in meters.
        velocity: The velocity to drive at in m/s.
    """
    # Calculate the time needed to travel the distance
    time_to_travel = abs(distance / velocity)

    start_time = time.time()

    while time.time() - start_time < time_to_travel:
        await send_twist_command(client, velocity)
        await asyncio.sleep(0.05)

    # Stop the robot
    await send_twist_command(client, 0.0)
    print(f"Drove {distance:.2f} meters")


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
    # Calculate the angular velocity based on the linear velocity and radius
    # w = v/r
    angular_velocity = direction * linear_velocity / radius

    # Calculate the arc length
    arc_length = abs(radius * angle_radians)

    # Calculate the time needed to travel the arc
    time_to_travel = arc_length / abs(linear_velocity)

    print(
        f"Driving arc: radius={radius}m, angle={angle_radians:.2f}rad,"
        f"linear_vel={linear_velocity}m/s, angular_vel={angular_velocity:.3f}rad/s"
    )
    print(f"Arc length: {arc_length:.2f}m, estimated time: {time_to_travel:.2f}s")

    start_time = time.time()

    while time.time() - start_time < time_to_travel:
        await send_twist_command(client, linear_velocity, angular_velocity)
        await asyncio.sleep(0.05)

    # Stop the robot
    await send_twist_command(client, 0.0)
    print(f"Completed arc turn of {angle_radians:.2f} radians with radius {radius:.2f} meters")


async def operate_tool(client: EventClient, direction: str, duration: float) -> None:
    """Operate the tool in a specific direction for a duration.

    Args:
        client: The event client connected to the canbus service.
        direction: The direction to move the tool ("FORWARD" or "REVERSE").
        duration: The duration to operate the tool in seconds.
    """
    start_time = time.time()

    while time.time() - start_time < duration:
        await send_tool_command(client, direction)
        await asyncio.sleep(0.1)

    # Stop the tool
    await send_tool_command(client, "STOP")
    print(f"Operated tool in {direction} direction for {duration:.2f} seconds")


async def main() -> None:
    """Run the combined sequence of robot movement and tool operation."""
    # Create a client to the canbus service
    config: EventServiceConfig = proto_from_json_file(SERVICE_CONFIG_PATH, EventServiceConfig())
    client: EventClient = EventClient(config)

    # Step 1: Drive forward 1 meter without tool action
    print("\n--- Step 1: Driving forward 1 meter ---")
    await drive_distance(client, 1.0, LINEAR_VELOCITY)

    # Step 2: Stop robot and operate tool in REVERSE for 10 seconds
    print("\n--- Step 2: Operating tool in REVERSE for 10 seconds ---")
    await operate_tool(client, "REVERSE", 10.0)

    # Step 3: Drive forward 10 meters without tool action
    print("\n--- Step 3: Driving forward 10 meters ---")
    await drive_distance(client, 10.0, LINEAR_VELOCITY)

    # Step 4: Stop robot and operate tool in FORWARD for 10 seconds
    print("\n--- Step 4: Operating tool in FORWARD for 10 seconds ---")
    await operate_tool(client, "FORWARD", 10.0)

    # Step 5: Drive a pi turn with radius of 2 meters
    print("\n--- Step 5: Driving a pi turn with 2 meter radius ---")
    # Pi radians = 180 degrees, counterclockwise turn
    await drive_arc(client, radius=2.0, angle_radians=math.pi, linear_velocity=LINEAR_VELOCITY, direction=1)

    print("\n--- Sequence completed ---")


if __name__ == "__main__":
    asyncio.run(main())
