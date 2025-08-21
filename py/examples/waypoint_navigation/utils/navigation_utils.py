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
import logging
import time
from pathlib import Path
from typing import Optional

from farm_ng.canbus.canbus_pb2 import Twist2d
from farm_ng.core.event_client import EventClient
from farm_ng.core.event_service_pb2 import EventServiceConfig
from farm_ng.core.events_file_reader import proto_from_json_file
from google.protobuf.empty_pb2 import Empty


logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("Utils")


def get_config(config_name: str) -> EventServiceConfig:
    """Get the configuration for a given service."""
    config_name = f"{config_name}_config.json"
    og_path = Path(f"../configs/{config_name}")
    alternative_path = Path(f"./configs/{config_name}")
    config_path = og_path if og_path.exists() else alternative_path
    return proto_from_json_file(config_path, EventServiceConfig())


async def move_robot_forward(time_goal: float = 1.5) -> None:
    """Util function to move the robot forward in case it gets stuck.

    Args:
        service_config_path (Path): The path to the canbus service config.
    """
    # Initialize the command to send
    twist = Twist2d(linear_velocity_x=0.7)

    # create a client to the canbus service
    config = get_config("canbus")
    client: EventClient = EventClient(config)
    start = time.monotonic()
    # Hold the loop for the duration
    while time.monotonic() - start < time_goal:
        # Update and send the twist command
        logger.info(
            f"Sending linear velocity: {twist.linear_velocity_x:.3f}, angular velocity: {twist.angular_velocity:.3f}"
        )
        await client.request_reply("/twist", twist)

        # Sleep to maintain a constant rate
        await asyncio.sleep(0.1)


async def stop_robot() -> None:
    """Util function to stop the robot."""

    MAX_RETRIES = 5

    # 1. Ensure we cancel any existing tracks
    controller_config: EventServiceConfig = get_config("controller")
    controller_client: EventClient = EventClient(controller_config)
    controller_success: Optional[Empty] = None

    for _ in range(MAX_RETRIES):
        try:
            # This will return None if there are no tracks set,
            # so we will try this for a maximum number of retries
            controller_success = await controller_client.request_reply("/cancel", Empty())
        except Exception as e:
            logger.error(f"Error cancelling controller: {e}")
        if controller_success is not None:
            logger.info("Potential tracks cancelled successfully.")
            break
        await asyncio.sleep(0.1)
    if controller_success is None:
        logger.error("Failed to cancel the controller. Maybe there were no tracks set?")

    # 2. Send a zero twist command to the can service to stop the robot
    twist = Twist2d(linear_velocity_x=0.0, angular_velocity=0.0)
    can_config = get_config("canbus")
    can_client: EventClient = EventClient(can_config)
    twist_success: Optional[Empty] = None
    while twist_success is None:
        # This doesn't depend on anything other than the can socket,
        # so we can just keep trying until we get a response
        twist_success = await can_client.request_reply("/twist", twist)
        logger.info("Sending twist stop command")
        await asyncio.sleep(0.1)

    logger.info("Robot stopped successfully.")


async def e_stop():
    """Util function to emergency stop the robot."""
    # 1. Try to "break" the robot
    MAX_RETRIES = 5
    twist = Twist2d(linear_velocity_x=0.0, angular_velocity=0.0)

    can_config = get_config("canbus")

    can_client: EventClient = EventClient(can_config)
    twist_success: Optional[Empty] = None
    for _ in range(MAX_RETRIES):
        twist_success = await can_client.request_reply("/twist", twist)
        if twist_success is not None:
            break
        logger.info("Sending twist stop command")
        await asyncio.sleep(0.1)

    # 2. E-stop the robot
    await can_client.request_reply("/estop", Empty())
    logger.info("Robot e-stopped successfully.")


if __name__ == "__main__":
    asyncio.run(e_stop())
