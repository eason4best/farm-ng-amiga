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


async def move_robot_forward(time_goal: float = 1.5) -> None:
    """Util function to move the robot forward in case it gets stuck.

    Args:
        service_config_path (Path): The path to the canbus service config.
    """
    # Initialize the command to send
    twist = Twist2d(linear_velocity_x=0.7)

    # create a client to the canbus service
    service_config_path = Path("./configs/canbus_config.json")
    config: EventServiceConfig = proto_from_json_file(service_config_path, EventServiceConfig())
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
    controller_config_path = Path("./configs/controller_config.json")
    controller_config: EventServiceConfig = proto_from_json_file(controller_config_path, EventServiceConfig())
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
            break
        await asyncio.sleep(0.1)
    if controller_success is None:
        logger.error("Failed to cancel the controller. Maybe there were no tracks set?")

    # 2. Send a zero twist command to the can service to stop the robot
    twist = Twist2d(linear_velocity_x=0.0, angular_velocity=0.0)
    canbus_config_path = Path("./configs/canbus_config.json")
    can_config: EventServiceConfig = proto_from_json_file(canbus_config_path, EventServiceConfig())
    can_client: EventClient = EventClient(can_config)
    twist_success: Optional[Empty] = None
    while twist_success is None:
        # This doesn't depend on anything other than the can socket,
        # so we can just keep trying until we get a response
        twist_success = await can_client.request_reply("/twist", twist)
        logger.info("Sending twist stop command")
        await asyncio.sleep(0.1)

    logger.info("Robot stopped successfully.")
