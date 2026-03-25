from math import copysign, radians
from pathlib import Path
from typing import List

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

        async for _, message in self.clients["track_follower"].subscribe(sub_request):
            status = message.status.track_status

            if status != TRACK_FOLLOWING:
                print(f"Segment finished with status: {status}")
                break

    async def navigate(self, distance: float, is_turn: bool = False, angle: float = 0.0):
        if is_turn:            
            waypoints = await self.turn(angle)
        else:
            waypoints = await self.go_straight(distance)

        track_msg = Track(waypoints=[pose.to_proto() for pose in waypoints])
        
        request = TrackFollowRequest(
            track=track_msg,
            driving_direction=DIRECTION_FORWARD
        )
        
        await self.clients["track_follower"].request_reply("/set_track", request)
        await self.clients["track_follower"].request_reply("/start", Empty())

        await self.wait_for_track_completion()
        
    async def go_straight(self, distance: float, step_size: float = 1.0) -> List[Pose3F64]:
        current_pose = await self.get_pose()
        poses = [current_pose]
        steps = int(distance / step_size)
        
        for i in range(1, steps + 1):
            intermediate_dist = i * step_size
            step_pose = current_pose * Pose3F64(
                a_from_b=Isometry3F64([intermediate_dist, 0, 0], Rotation3F64.Rz(0)),
                frame_a=current_pose.frame_b,
                frame_b=f"straight_{i}"
            )
            poses.append(step_pose)

        if distance % step_size > 0.001:
            final_pose = current_pose * Pose3F64(
                a_from_b=Isometry3F64([distance, 0, 0], Rotation3F64.Rz(0)),
                frame_a=current_pose.frame_b,
                frame_b="straight_final"
            )
            poses.append(final_pose)
            
        return poses
        
    async def turn(self, angle: float, step_size: float = radians(15)) -> List[Pose3F64]:
        current_pose = await self.get_pose()
        poses = [current_pose]
        steps = int(abs(angle) / step_size)
        actual_step = copysign(step_size, angle)
        
        for i in range(1, steps + 1):
            cumulative_angle = i * actual_step
            step_pose = current_pose * Pose3F64(
                a_from_b=Isometry3F64.Rz(cumulative_angle), 
                frame_a=current_pose.frame_b,
                frame_b=f"turn_{i}"
            )
            poses.append(step_pose)

        remaining_angle = angle - (actual_step * steps)
        
        if abs(remaining_angle) > 0.001:
            final_pose = current_pose * Pose3F64(
                a_from_b=Isometry3F64.Rz(angle),
                frame_a=current_pose.frame_b,
                frame_b="turn_final"
            )
            poses.append(final_pose)

        return poses

    async def start_mission(self):
        print("Starting mission: Drive 10m forward, turn 180 degrees, then drive 10m back to starting point.")

        await self.navigate(distance=10)

        print("Reached 10m, preparing to turn around...")
        await self.navigate(distance=0, is_turn=True, angle=radians(180))
        
        await self.navigate(distance=10)
        
        print("Backed to starting point, preparing to turn around...")
        await self.navigate(distance=0, is_turn=True, angle=radians(180))

        print("Mission complete, returned to starting point.")