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
import signal
import sys
from pathlib import Path
from typing import Optional
from typing import Tuple

from farm_ng.core.event_client import EventClient
from farm_ng.core.event_service_pb2 import EventServiceConfig
from farm_ng.core.events_file_reader import proto_from_json_file
from farm_ng.track.track_pb2 import Track
from farm_ng.track.track_pb2 import TrackFollowerState
from farm_ng.track.track_pb2 import TrackFollowRequest
from farm_ng.track.track_pb2 import TrackStatusEnum
from google.protobuf.empty_pb2 import Empty
from motion_planner import MotionPlanner


class NavigationManager:
    """Orchestrates waypoint navigation using MotionPlanner and track_follower service."""

    def __init__(self, filter_client: EventClient, controller_client: EventClient, motion_planner: MotionPlanner):
        self.filter_client = filter_client
        self.controller_client = controller_client
        self.motion_planner = motion_planner
        self.current_track_status: Optional[TrackStatusEnum] = None
        self.track_complete_event = asyncio.Event()
        self.track_failed_event = asyncio.Event()
        self.shutdown_requested = False

    async def set_track(self, track: Track) -> None:
        """Set the track for the track_follower to follow.

        Args:
            track: The track to follow
        """
        print(f"📤 Setting track with {len(track.waypoints)} waypoints...")
        try:
            await self.controller_client.request_reply("/set_track", TrackFollowRequest(track=track))
            print("✅ Track set successfully")
        except Exception as e:
            print(f"❌ Failed to set track: {e}")
            raise

    async def start_following(self) -> None:
        """Start following the currently set track."""
        print("🚀 Starting track following...")
        try:
            await self.controller_client.request_reply("/start", Empty())
            print("✅ Track following started")
        except Exception as e:
            print(f"❌ Failed to start track following: {e}")
            raise

    async def monitor_track_state(self) -> None:
        """Monitor the track_follower state and set events based on status."""
        print("👁️  Starting track state monitoring...")

        try:
            config = self.controller_client.config
            subscription = config.subscriptions[0] if config.subscriptions else "/state"

            async for event, message in self.controller_client.subscribe(subscription, decode=True):
                if self.shutdown_requested:
                    break

                if isinstance(message, TrackFollowerState):
                    await self._process_track_state(message)

        except Exception as e:
            print(f"❌ Error monitoring track state: {e}")
            self.track_failed_event.set()

    async def _process_track_state(self, state: TrackFollowerState) -> None:
        """Process incoming track follower state messages.

        Args:
            state: The TrackFollowerState message
        """
        track_status = state.status.track_status
        robot_controllable = state.status.robot_status.controllable

        # Update current status
        prev_status = self.current_track_status
        self.current_track_status = track_status

        # Log status changes
        if prev_status != track_status:
            status_name = TrackStatusEnum.Name(track_status)
            print(f"📊 Track status changed: {status_name}")

        # Check for completion or failure
        if track_status == TrackStatusEnum.TRACK_COMPLETE:
            print("🎉 Track completed successfully!")
            self.track_complete_event.set()

        elif track_status in [
            TrackStatusEnum.TRACK_FAILED,
            TrackStatusEnum.TRACK_ABORTED,
            TrackStatusEnum.TRACK_CANCELLED,
        ]:
            status_name = TrackStatusEnum.Name(track_status)
            print(f"💥 Track failed with status: {status_name}")
            if not robot_controllable:
                failure_modes = [mode.name for mode in state.status.robot_status.failure_modes]
                print(f"Robot not controllable. Failure modes: {failure_modes}")
            self.track_failed_event.set()

        # Log cross-track error if available
        if (
            hasattr(state, 'progress')
            and state.progress
            and hasattr(state.progress, 'cross_track_error')
            and state.progress.cross_track_error
        ):
            error = state.progress.cross_track_error
            if error.total_distance > 0.5:  # Only log if significant error
                print(
                    f"⚠️  Cross-track error: {error.total_distance:.2f}m "
                    f"(lateral: {error.lateral_distance:.2f}m, "
                    f"longitudinal: {error.longitudinal_distance:.2f}m)"
                )

    async def wait_for_track_completion(self, timeout: float = 300.0) -> bool:
        """Wait for track to complete or fail.

        Args:
            timeout: Maximum time to wait in seconds

        Returns:
            True if track completed successfully, False if failed or timed out
        """
        print(f"⏳ Waiting for track completion (timeout: {timeout}s)...")

        try:
            # Wait for either completion or failure
            done, pending = await asyncio.wait(
                [
                    asyncio.create_task(self.track_complete_event.wait()),
                    asyncio.create_task(self.track_failed_event.wait()),
                ],
                timeout=timeout,
                return_when=asyncio.FIRST_COMPLETED,
            )

            # Cancel pending tasks
            for task in pending:
                task.cancel()

            if not done:
                print("⏰ Timeout waiting for track completion")
                return False

            # Check which event was set
            if self.track_complete_event.is_set():
                return True
            elif self.track_failed_event.is_set():
                return False

        except Exception as e:
            print(f"❌ Error waiting for track completion: {e}")
            return False

        return False

    async def execute_single_track(self, track: Track, timeout: float = 300.0) -> bool:
        """Execute a single track segment and wait for completion.

        Args:
            track: The track to execute
            timeout: Maximum time to wait for completion

        Returns:
            True if successful, False otherwise
        """
        # Reset events
        self.track_complete_event.clear()
        self.track_failed_event.clear()

        try:
            # Set and start the track
            await self.set_track(track)
            await self.start_following()

            # Wait for completion
            success = await self.wait_for_track_completion(timeout)

            if success:
                print("✅ Track segment completed successfully")
            else:
                print("❌ Track segment failed or timed out")

            return success

        except Exception as e:
            print(f"❌ Error executing track: {e}")
            return False

    async def run_navigation(self) -> None:
        """Run the complete waypoint navigation sequence."""
        print("🚁 Starting waypoint navigation...")

        # Start monitoring track state
        monitor_task = asyncio.create_task(self.monitor_track_state())

        try:
            segment_count = 0

            while not self.shutdown_requested:
                # Get next track segment
                print(f"\n--- Segment {segment_count + 1} ---")
                track_segment = await self.motion_planner.next_track_segment()

                if track_segment is None:
                    print("🏁 No more track segments. Navigation complete!")
                    break

                segment_count += 1
                print(f"📍 Executing track segment {segment_count} with {len(track_segment.waypoints)} waypoints")

                # Execute the track segment
                success = await self.execute_single_track(track_segment)

                if not success:
                    print(f"💥 Failed to execute segment {segment_count}. Stopping navigation.")
                    break

                # Brief pause between segments
                await asyncio.sleep(1.0)

            print(f"🎯 Navigation completed after {segment_count} segments")

        except KeyboardInterrupt:
            print("\n🛑 Navigation interrupted by user")
        except Exception as e:
            print(f"💥 Navigation failed with error: {e}")
        finally:
            # Cleanup
            self.shutdown_requested = True
            monitor_task.cancel()
            await self.motion_planner._shutdown()

            try:
                await monitor_task
            except asyncio.CancelledError:
                pass


async def setup_clients(filter_config_path: Path, controller_config_path: Path) -> Tuple[EventClient, EventClient]:
    """Setup EventClients for filter and controller services.

    Args:
        filter_config_path: Path to filter service config
        controller_config_path: Path to controller service config

    Returns:
        Tuple of (filter_client, controller_client)
    """
    print("🔧 Setting up service clients...")

    # Load filter service config
    filter_config = proto_from_json_file(filter_config_path, EventServiceConfig())
    if filter_config.name != "filter":
        raise RuntimeError(f"Expected filter service config, got {filter_config.name}")
    filter_client = EventClient(filter_config)

    # Load controller service config
    controller_config = proto_from_json_file(controller_config_path, EventServiceConfig())
    if controller_config.name != "track_follower":
        raise RuntimeError(f"Expected track_follower service config, got {controller_config.name}")
    controller_client = EventClient(controller_config)

    print(f"✅ Filter client: {filter_config.name}")
    print(f"✅ Controller client: {controller_config.name}")

    return filter_client, controller_client


def setup_signal_handlers():
    """Setup signal handlers for graceful shutdown."""

    def signal_handler(signum, frame):
        print(f"\n🛑 Received signal {signum}, shutting down gracefully...")
        # Let the main loop handle the shutdown
        return

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)


async def main(args) -> None:
    """Main function to orchestrate waypoint navigation."""

    setup_signal_handlers()

    try:
        # Setup clients
        filter_client, controller_client = await setup_clients(args.filter_config, args.controller_config)

        # Initialize motion planner
        print("🗺️  Initializing motion planner...")
        motion_planner = MotionPlanner(
            client=filter_client,
            waypoints_path=args.waypoints_path,
            last_row_waypoint_index=args.last_row_waypoint_index,
            turn_direction=args.turn_direction,
            row_spacing=args.row_spacing,
            headland_buffer=args.headland_buffer,
        )

        # Create orchestrator
        orchestrator = NavigationManager(
            filter_client=filter_client, controller_client=controller_client, motion_planner=motion_planner
        )

        # Run navigation
        await orchestrator.run_navigation()

    except Exception as e:
        print(f"💥 Fatal error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        prog="python main.py", description="Waypoint navigation using MotionPlanner and track_follower service"
    )

    # Required arguments
    parser.add_argument("--filter-config", type=Path, required=True, help="Path to filter service config JSON file")
    parser.add_argument("--waypoints-path", type=Path, required=True, help="Path to waypoints JSON file (Track format)")
    parser.add_argument(
        "--controller-config", type=Path, required=True, help="Path to track_follower service config JSON file"
    )

    # MotionPlanner configuration
    parser.add_argument(
        "--last-row-waypoint-index",
        type=int,
        default=6,
        help="Index of the last waypoint in the current row (default: 6)",
    )
    parser.add_argument(
        "--turn-direction",
        choices=["left", "right"],
        default="left",
        help="Direction to turn at row ends (default: left)",
    )
    parser.add_argument("--row-spacing", type=float, default=3.0, help="Spacing between rows in meters (default: 3.0)")
    parser.add_argument(
        "--headland-buffer",
        type=float,
        default=2.0,
        help="Buffer distance for headland maneuvers in meters (default: 2.0)",
    )

    args = parser.parse_args()

    # Validate file paths
    for path_arg in [args.filter_config, args.waypoints_path, args.controller_config]:
        if not path_arg.exists():
            print(f"❌ File not found: {path_arg}")
            sys.exit(1)

    # Run the main function
    try:
        asyncio.run(main(args))
    except KeyboardInterrupt:
        print("\n🛑 Interrupted by user")
        sys.exit(0)
