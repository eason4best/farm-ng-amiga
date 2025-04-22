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
SIMULATION_TIME = 5.0  # seconds


async def main() -> None:
    """Run the canbus service client.

    Args:
        service_config_path (Path): The path to the canbus service config.
    """
    # Initialize the command to send
    twist = Twist2d()
    twist.linear_velocity_x = LINEAR_VELOCITY
    twist.linear_velocity_x = clip(twist.linear_velocity_x, -MAX_LINEAR_VELOCITY_MPS, MAX_LINEAR_VELOCITY_MPS)
    twist.angular_velocity = 0.0

    # create a client to the canbus service
    config: EventServiceConfig = proto_from_json_file(SERVICE_CONFIG_PATH, EventServiceConfig())
    client: EventClient = EventClient(config)

    start_time = time.time()

    while time.time() - start_time < SIMULATION_TIME:
        # Send the twist command
        print(f"Sending linear velocity: {twist.linear_velocity_x:.3f}, angular velocity: {twist.angular_velocity:.3f}")
        await client.request_reply("/twist", twist)

        # Sleep to maintain a constant rate
        await asyncio.sleep(0.05)


if __name__ == "__main__":
    asyncio.run(main())
