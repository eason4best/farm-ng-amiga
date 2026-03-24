import argparse
import asyncio
from math import copysign, radians
from pathlib import Path

from farm_ng.core.event_client import EventClient
from farm_ng.core.event_service_pb2 import EventServiceConfigList, SubscribeRequest
from farm_ng.core.events_file_reader import proto_from_json_file
from farm_ng.core.uri_pb2 import Uri
from farm_ng.filter.filter_pb2 import FilterState
from farm_ng.gps import gps_pb2
from farm_ng.track.track_pb2 import (
    Track, 
    TrackFollowRequest, 
    TRACK_FOLLOWING,
    DIRECTION_FORWARD
)
from farm_ng_core_pybind import Isometry3F64, Pose3F64, Rotation3F64
from google.protobuf.empty_pb2 import Empty

class AmigaRobotController:
    def __init__(self, service_config_path: Path):
        self.clients = {}
        config_list = proto_from_json_file(service_config_path, EventServiceConfigList())
        for config in config_list.configs:
            self.clients[config.name] = EventClient(config)
        
        self.current_gps = {"lat": 0.0, "lon": 0.0}

    async def update_gps_task(self):
        async for _, msg in self.clients["gps"].subscribe(self.clients["gps"].config.subscriptions[0]):
            if isinstance(msg, gps_pb2.GpsFrame):
                self.current_gps["lat"] = msg.latitude
                self.current_gps["lon"] = msg.longitude

    async def get_pose(self) -> Pose3F64:
        state: FilterState = await self.clients["filter"].request_reply("/get_state", Empty(), decode=True)
        return Pose3F64.from_proto(state.pose)

    async def wait_for_track_completion(self):
        sub_request = SubscribeRequest(uri=Uri(path="/state"))
        
        await asyncio.sleep(0.3)

        async for _, message in self.clients["track_follower"].subscribe(sub_request):
            status = message.status.track_status
            print(f"Track status: {status}")

            if status != TRACK_FOLLOWING:
                print(f"Segment finished with status: {status}")
                break

        await asyncio.sleep(0.05)

    async def drive_segment(self, distance: float, is_turn: bool = False, angle_rad: float = 0.0):
        current_pose = await self.get_pose()
        waypoints = [current_pose]

        if is_turn:            
            step_size = radians(15)
            steps = int(abs(angle_rad) / step_size)
            actual_step = copysign(step_size, angle_rad)
            
            last_pose = current_pose
            for i in range(steps):
                last_pose = last_pose * Pose3F64(
                    a_from_b=Isometry3F64.Rz(actual_step),
                    frame_a=last_pose.frame_b,
                    frame_b=f"turn_{i}"
                )
                waypoints.append(last_pose)

            remaining_angle = angle_rad - (actual_step * steps)
            if abs(remaining_angle) > 0.001:
                final_pose = last_pose * Pose3F64(
                    a_from_b=Isometry3F64.Rz(remaining_angle),
                    frame_a=last_pose.frame_b,
                    frame_b="turn_final"
                )
                waypoints.append(final_pose)
        else:
            straight_pose = current_pose * Pose3F64(
                a_from_b=Isometry3F64([distance, 0, 0], Rotation3F64.Rz(0)),
                frame_a=current_pose.frame_b,
                frame_b="straight_goal"
            )
            waypoints.append(straight_pose)

        track_msg = Track(waypoints=[pose.to_proto() for pose in waypoints])
        
        request = TrackFollowRequest(
            track=track_msg,
            driving_direction=DIRECTION_FORWARD
        )
        
        await self.clients["track_follower"].request_reply("/set_track", request)
        await asyncio.sleep(0.1) 
        await self.clients["track_follower"].request_reply("/start", Empty())

        await self.wait_for_track_completion()

    async def run_mission(self):
        print("Starting mission: Drive 10m forward, turn 180 degrees, then drive 10m back to starting point.")

        for i in range(1, 6):
            print(f"--- Driving Segment {i} (Total: {i*2}m) ---")
            await self.drive_segment(distance=2.0)
            
            print(f"Resting for 1 second. Current Position - Latitude: {self.current_gps['lat']}, Longitude: {self.current_gps['lon']}")
            await asyncio.sleep(1.0)

        print("Reached 10m, preparing to turn around...")
        await self.drive_segment(distance=0, is_turn=True, angle_rad=radians(180))
        await asyncio.sleep(1.0)

        for i in range(1, 6):
            print(f"--- Returning Segment {i} (Total: {i*2}m) ---")
            
            await self.drive_segment(distance=2.0)
            
            print(f"Resting for 1 second. Current Position - Latitude: {self.current_gps['lat']}, Longitude: {self.current_gps['lon']}")
            await asyncio.sleep(1.0)

        print("Mission complete, returned to starting point.")

async def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--service-config", type=Path, required=True)
    args = parser.parse_args()

    controller = AmigaRobotController(args.service_config)

    try:
        await asyncio.gather(
            controller.update_gps_task(),
            controller.run_mission()
        )
    except asyncio.CancelledError:
        pass

if __name__ == "__main__":
    asyncio.run(main())