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

from farm_ng.canbus.tool_control_pb2 import ActuatorCommands
from farm_ng.canbus.tool_control_pb2 import HBridgeCommand
from farm_ng.canbus.tool_control_pb2 import HBridgeCommandType
from farm_ng.core.event_client import EventClient
from farm_ng.core.event_service_pb2 import EventServiceConfig
from farm_ng.core.events_file_reader import proto_from_json_file

# Constants
SIMULATION_TIME: float = 5.0  # seconds
TOOL_DIRECTION: str = "FORWARD"  # "FORWARD", "REVERSE", or "STOP"
HBRIDGE_ID: int = 0
SERVICE_CONFIG_PATH: Path = Path(__file__).parent / "service_config.json"


async def main():
    if TOOL_DIRECTION == "FORWARD":
        direction = HBridgeCommandType.HBRIDGE_FORWARD
    elif TOOL_DIRECTION == "REVERSE":
        direction = HBridgeCommandType.HBRIDGE_REVERSE
    elif TOOL_DIRECTION == "STOP":
        direction = HBridgeCommandType.HBRIDGE_STOPPED
    else:
        raise ValueError(f"Invalid tool direction: {TOOL_DIRECTION}. Use 'FORWARD', 'REVERSE', or 'STOP'.")

    command: ActuatorCommands = ActuatorCommands()
    command.hbridges.append(HBridgeCommand(id=HBRIDGE_ID, command=direction))

    # Control the tool
    config: EventServiceConfig = proto_from_json_file(SERVICE_CONFIG_PATH, EventServiceConfig())
    client: EventClient = EventClient(config)

    start_time = time.time()
    while time.time() - start_time < SIMULATION_TIME:
        # Send the tool control command
        print(f"Sending tool control command: {command}")
        await client.request_reply("/control_tools", command)

        # Sleep to maintain a constant rate
        await asyncio.sleep(0.1)


if __name__ == "__main__":
    asyncio.run(main())
