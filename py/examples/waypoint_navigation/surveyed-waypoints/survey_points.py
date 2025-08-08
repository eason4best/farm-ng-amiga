"""Example of a state estimation filter service client with waypoint saving."""
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
import sys
import termios
import time
import tty
from pathlib import Path
from typing import Optional

from farm_ng.core.event_client import EventClient
from farm_ng.core.event_service_pb2 import EventServiceConfig
from farm_ng.core.events_file_reader import proto_from_json_file
from farm_ng.core.events_file_writer import proto_to_json_file
from farm_ng.filter.filter_pb2 import DivergenceCriteria
from farm_ng.track.track_pb2 import Track
from farm_ng_core_pybind import Pose3F64

logger = logging.getLogger("Waypoint Surveying")


class WaypointCollector:
    def __init__(self):
        self.current_pose: Pose3F64 | None = None
        self.waypoints: Track = Track()
        self.waypoint_counter = 1
        self.file_name: Optional[Path] = None

    async def filter_listener_task(self, config: EventServiceConfig):
        """Task to listen for filter messages and update current state."""
        async for _, message in EventClient(config).subscribe(config.subscriptions[0], decode=True):
            divergence_criteria: list[DivergenceCriteria] = [
                DivergenceCriteria.Name(criteria) for criteria in message.divergence_criteria
            ]
            pose: Pose3F64 = Pose3F64.from_proto(message.pose)

            if not message.has_converged:
                self.current_pose = None
                logger.error(f"Filter diverged due to: {divergence_criteria}")

            else:
                self.current_pose = pose

    def get_char(self):
        """Get a single character from stdin without pressing enter."""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    async def keyboard_input_task(self):
        """Task to handle keyboard input."""
        loop = asyncio.get_event_loop()
        logger.info("Press 'a' to add waypoint, 's' to save, 'q' to quit")

        while True:
            try:
                # Run the blocking get_char in a thread executor
                char = await loop.run_in_executor(None, self.get_char)

                if char.lower() == 'a':
                    await self.add_waypoint()
                elif char.lower() == 's':
                    await self.save_waypoints()
                elif char.lower() == 'q':
                    logger.info("\nSaving waypoints and quitting...")
                    break

            except Exception as e:
                logger.error(f"Error in keyboard input: {e}")
                break

    async def add_waypoint(self):
        """Add current filter state as a waypoint."""
        if self.current_pose is None:
            logger.warning("No filter state available yet. Wait for first message.")
            return

        self.waypoints.waypoints.append(self.current_pose.to_proto())

        logger.info(f"\n NEW WAYPOINT ADDED: Total Waypoints: {self.waypoint_counter}")

        self.waypoint_counter += 1

    async def save_waypoints(self):
        """Save waypoints dictionary to JSON file."""
        if not self.waypoints.waypoints:
            logger.error("No waypoints to save.")
            return

        # Create file name if it doesn't exist
        # If it exists (meaning we have already saved stuff, we will just overwrite it)
        if self.file_name is None:
            date_str = time.strftime("%Y-%m-%d_%H-%M-%S")
            filename = f"./{date_str}_waypoints.json"
            self.file_name = Path(filename)

        try:
            proto_to_json_file(self.file_name, self.waypoints)
            logger.info(f"Waypoints saved to '{self.file_name}'")
        except Exception as e:
            logger.error(f"Error saving waypoints: {e}")

    async def run(self, service_config_path: Path):
        """Run the waypoint collector with both filter listener and keyboard input."""
        config: EventServiceConfig = proto_from_json_file(service_config_path, EventServiceConfig())

        logger.info("Starting waypoint collector...")
        logger.info("Commands:")
        logger.info("  'a' - Add current position as waypoint")
        logger.info("  's' - Save waypoints to JSON file")
        logger.info("  'q' - Quit")
        logger.info("\nWaiting for filter messages...")

        # Create tasks for both filter listening and keyboard input
        filter_task = asyncio.create_task(self.filter_listener_task(config))
        keyboard_task = asyncio.create_task(self.keyboard_input_task())

        try:
            # Wait for either task to complete (keyboard task will complete on 'q')
            _, pending = await asyncio.wait([filter_task, keyboard_task], return_when=asyncio.FIRST_COMPLETED)

            # Cancel remaining tasks
            for task in pending:
                task.cancel()
                try:
                    await task
                except asyncio.CancelledError:
                    pass

        except KeyboardInterrupt:
            logger.warning("\nReceived Ctrl+C, shutting down...")
        finally:
            # Save waypoints before exiting if any were collected
            if self.waypoints:
                await self.save_waypoints()


async def main() -> None:
    """Run the filter service client with waypoint collection.

    Args:
        service_config_path (Path): The path to the filter service config.
    """
    collector = WaypointCollector()
    service_config_path = Path("../configs/survey_config.json")
    await collector.run(service_config_path)


if __name__ == "__main__":
    asyncio.run(main())
