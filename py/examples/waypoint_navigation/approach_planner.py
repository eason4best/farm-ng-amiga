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

import logging
from enum import Enum
from pathlib import Path
from typing import List

import numpy as np
from farm_ng.core.events_file_writer import proto_to_json_file
from farm_ng.track.track_pb2 import Track
from farm_ng_core_pybind import Isometry3F64
from farm_ng_core_pybind import Pose3F64
from farm_ng_core_pybind import Rotation3F64


class FirstApproachStrategy(Enum):
    """Strategy for approaching the first waypoint."""

    DIRECT_AB = "direct_ab"
    HEADING_CORRECTION_ONLY = "heading_correction_only"
    LATERAL_CORRECTION = "lateral_correction"
    BACKUP_THEN_LATERAL = "backup_then_lateral"
    BACKUP_THEN_HEADING_CORRECTION = "backup_then_heading_correction"
    REPOSITIONING_NEEDED = "repositioning_needed"


logger = logging.getLogger("Motion Planner")


class ApproachPlanner:
    """Plans the multi-segment approach to the first waypoint."""

    def __init__(self, current_pose: Pose3F64, goal_pose: Pose3F64):
        self.current_pose = current_pose
        self.goal_pose = goal_pose

        # Thresholds
        self.LATERAL_THRESHOLD = 0.1  # 10 cm
        self.MIN_LONGITUDINAL_DISTANCE = 9.0  # 9 m target distance
        self.HEADING_THRESHOLD = np.radians(3)  # 3 degrees
        self.MIN_BACKUP_DISTANCE = 2.0  # Minimum distance before doing lateral correction

        self.delta_x: float = 0.0
        self.delta_y: float = 0.0
        self.delta_heading: float = 0.0
        self.current_heading: float = 0.0
        self.goal_heading: float = 0.0
        self.is_behind_goal: bool = False
        self.robot_in_goal_frame: np.ndarray | None = None
        self._robot_in_goal_frame()
        self.strategy = self._determine_strategy()

    def _robot_in_goal_frame(self) -> None:
        """Calculate the robot position in the goal's coordinate system."""

        # Transform robot position to goal's coordinate system
        goal_from_robot = self.goal_pose.inverse() * self.current_pose
        robot_in_goal_frame = goal_from_robot.a_from_b.translation

        # In goal's coordinate system:
        # - Negative X = robot behind goal
        # - Positive Y = robot to left of goal
        self.is_behind_goal = robot_in_goal_frame[0] < 0.0
        self.robot_in_goal_frame = robot_in_goal_frame

        # Set deltas in goal's coordinate system
        self.delta_x = -robot_in_goal_frame[0]  # Distance to goal (positive = goal ahead)
        self.delta_y = -robot_in_goal_frame[1]  # Lateral offset (positive = goal to left)

        # Heading calculation needs to be separate
        self.current_heading = self.current_pose.a_from_b.rotation.log()[-1]
        self.goal_heading = self.goal_pose.a_from_b.rotation.log()[-1]
        self.delta_heading = self.goal_heading - self.current_heading

        # Normalize heading difference to [-π, π]
        while self.delta_heading > np.pi:
            self.delta_heading -= 2 * np.pi
        while self.delta_heading < -np.pi:
            self.delta_heading += 2 * np.pi

    def _determine_strategy(self) -> FirstApproachStrategy:
        """Determine the approach strategy based on the deltas."""

        # Robot must be behind goal
        if not self.is_behind_goal:
            return FirstApproachStrategy.REPOSITIONING_NEEDED

        # Direct AB or HEADING CORRECTION: Small lateral offset compared to longitudinal offset
        if abs(self.delta_y) <= self.LATERAL_THRESHOLD and abs(self.delta_x) >= self.MIN_BACKUP_DISTANCE:
            if abs(self.delta_heading) < self.HEADING_THRESHOLD:
                return FirstApproachStrategy.DIRECT_AB
            return FirstApproachStrategy.HEADING_CORRECTION_ONLY

        # At this point, we need to do some lateral correction
        if abs(self.delta_y) <= self.LATERAL_THRESHOLD:
            # This means our delta x is small, but the delta y is also small, so we just need to drive backwards
            # (away from the goal and then drive towards the goal)
            return FirstApproachStrategy.BACKUP_THEN_HEADING_CORRECTION
        # If everything failed up until this point, this means, our delta y is large,
        # in this case, we need to devise a strategy for driving laterally
        elif (
            abs(self.delta_x) >= self.MIN_LONGITUDINAL_DISTANCE
        ):  # we are still fairly distant to the goal, driving laterally only will suffice
            return FirstApproachStrategy.LATERAL_CORRECTION
        return (
            FirstApproachStrategy.BACKUP_THEN_LATERAL
        )  # we're too close to simply drive laterally, we will back up a bit more and then drive laterally

    def _get_heading_correction_poses(self) -> List[Pose3F64]:
        """Get poses for heading correction only strategy."""
        poses = []

        # 1. Turn in place to goal heading
        turn_pose = Pose3F64(
            a_from_b=Isometry3F64(
                translation=self.current_pose.a_from_b.translation,
                rotation=self.goal_pose.a_from_b.rotation,  # Use goal's absolute rotation
            ),
            frame_a="world",
            frame_b="robot",
        )
        poses.append(turn_pose)

        # 2. Goal pose
        poses.append(self.goal_pose)

        return poses

    def _get_lateral_correction_poses(self) -> List[Pose3F64]:
        """Get poses for lateral correction strategy."""
        poses = []
        current_pos = self.current_pose.a_from_b.translation

        # 1. Turn to face perpendicular to goal (towards goal direction)
        perpendicular_rotation = self._get_perpendicular_rotation_towards_goal()
        turn1_pose = Pose3F64(
            a_from_b=Isometry3F64(translation=current_pos, rotation=perpendicular_rotation),
            frame_a="world",
            frame_b="robot",
        )
        poses.append(turn1_pose)

        # 2. Drive to be aligned laterally with the goal (maintain current longitudinal distance)
        # Use current longitudinal distance (abs of robot_in_goal_frame[0]) to stay at same distance behind goal
        current_longitudinal_distance = abs(self.robot_in_goal_frame[0])
        target_position = self._get_position_behind_goal(current_longitudinal_distance)
        lateral_pose = Pose3F64(
            a_from_b=Isometry3F64(translation=target_position, rotation=perpendicular_rotation),
            frame_a="world",
            frame_b="robot",
        )
        poses.append(lateral_pose)

        # 3. Turn to goal heading
        turn2_pose = Pose3F64(
            a_from_b=Isometry3F64(
                translation=target_position, rotation=self.goal_pose.a_from_b.rotation  # Goal's absolute rotation
            ),
            frame_a="world",
            frame_b="robot",
        )
        poses.append(turn2_pose)

        # 4. Goal pose
        poses.append(self.goal_pose)

        return poses

    def _get_backup_then_heading_poses(self) -> List[Pose3F64]:
        """Get poses for backup then heading correction strategy."""
        poses = []
        current_pos = self.current_pose.a_from_b.translation

        # 1. Turn to face opposite of goal (180° from goal heading)
        opposite_rotation = self._get_opposite_rotation()
        turn1_pose = Pose3F64(
            a_from_b=Isometry3F64(translation=current_pos, rotation=opposite_rotation), frame_a="world", frame_b="robot"
        )
        poses.append(turn1_pose)

        # 2. Drive to be 5m behind goal
        backup_distance = self.MIN_LONGITUDINAL_DISTANCE - abs(self.robot_in_goal_frame[0])
        backup_pose = self._get_backup_pose(turn1_pose, backup_distance)

        poses.append(backup_pose)
        backup_x = backup_pose.a_from_b.translation[0]
        backup_y = backup_pose.a_from_b.translation[1]

        # 3. Turn to goal heading
        turn2_pose = Pose3F64(
            a_from_b=Isometry3F64(
                translation=np.array([backup_x, backup_y, current_pos[2]]), rotation=self.goal_pose.a_from_b.rotation
            ),
            frame_a="world",
            frame_b="robot",
        )
        poses.append(turn2_pose)

        # 4. Goal pose
        poses.append(self.goal_pose)

        return poses

    def _get_backup_then_lateral_poses(self) -> List[Pose3F64]:
        """Get poses for backup then lateral correction strategy."""
        poses = []
        current_pos = self.current_pose.a_from_b.translation

        # 1. Turn to face opposite of goal
        opposite_rotation = self._get_opposite_rotation()
        turn1_pose = Pose3F64(
            a_from_b=Isometry3F64(translation=current_pos, rotation=opposite_rotation), frame_a="world", frame_b="robot"
        )
        poses.append(turn1_pose)

        # 2. Drive to be 5m behind goal
        backup_distance = self.MIN_LONGITUDINAL_DISTANCE - abs(self.robot_in_goal_frame[0])
        backup_pose = self._get_backup_pose(turn1_pose, backup_distance)

        poses.append(backup_pose)
        backup_x = backup_pose.a_from_b.translation[0]
        backup_y = backup_pose.a_from_b.translation[1]

        # 3. Turn to face perpendicular to goal (towards goal)
        perpendicular_rotation = self._get_perpendicular_rotation_towards_goal()
        turn2_pose = Pose3F64(
            a_from_b=Isometry3F64(
                translation=np.array([backup_x, backup_y, current_pos[2]]), rotation=perpendicular_rotation
            ),
            frame_a="world",
            frame_b="robot",
        )
        poses.append(turn2_pose)

        # 4. Drive to be properly aligned behind the goal (lateral correction)
        target_position = self._get_position_behind_goal(self.MIN_LONGITUDINAL_DISTANCE)
        lateral_pose = Pose3F64(
            a_from_b=Isometry3F64(translation=target_position, rotation=perpendicular_rotation),
            frame_a="world",
            frame_b="robot",
        )
        poses.append(lateral_pose)

        # 5. Turn to goal heading
        turn3_pose = Pose3F64(
            a_from_b=Isometry3F64(translation=target_position, rotation=self.goal_pose.a_from_b.rotation),
            frame_a="world",
            frame_b="robot",
        )
        poses.append(turn3_pose)

        # 6. Goal pose
        poses.append(self.goal_pose)

        return poses

    def _get_perpendicular_rotation_towards_goal(self) -> Rotation3F64:
        """Get the perpendicular rotation that points towards the goal."""
        # Get goal's rotation matrix and add 90° or -90°
        goal_rotation = self.goal_pose.a_from_b.rotation

        if self.delta_y > 0:
            # Goal is to the left, face left (add +90°)
            perpendicular_rotation = goal_rotation * Rotation3F64.Rz(np.pi / 2)
        else:
            # Goal is to the right, face right (add -90°)
            perpendicular_rotation = goal_rotation * Rotation3F64.Rz(-np.pi / 2)

        return perpendicular_rotation

    def _get_opposite_rotation(self) -> Rotation3F64:
        """Get rotation opposite to goal (180° from goal)."""
        goal_rotation = self.goal_pose.a_from_b.rotation
        return goal_rotation * Rotation3F64.Rz(np.pi)

    def _get_backup_pose(self, current_pose: Pose3F64, distance: float) -> Pose3F64:
        """Calculate backup pose by moving backwards in robot's current frame."""

        # Create relative movement: move forwards (X) in robot frame
        relative_movement = Pose3F64(
            a_from_b=Isometry3F64(translation=np.array([distance, 0.0, 0.0]), rotation=Rotation3F64()),
            frame_a=current_pose.frame_b,
            frame_b="backup_target",
        )

        # Transform to world frame
        return current_pose * relative_movement

    def _get_position_behind_goal(self, distance: float) -> np.ndarray:
        """Calculate a position that is 'distance' meters behind the goal in the goal's coordinate frame."""
        # Create a pose that is 'distance' meters behind the goal in goal's local frame
        behind_goal_local = Pose3F64(
            a_from_b=Isometry3F64(
                translation=np.array([-distance, 0.0, 0.0]),  # Negative X = behind in goal's frame
                rotation=Rotation3F64(),
            ),
            frame_a="robot",
            frame_b="world",
        )

        # Transform to world coordinates
        behind_goal_world = self.goal_pose * behind_goal_local
        return behind_goal_world.a_from_b.translation

    def get_final_poses(self) -> List[Pose3F64]:
        """Get the final poses for each segment based on the strategy."""

        final_poses: List[Pose3F64] = []

        if self.strategy == FirstApproachStrategy.DIRECT_AB:
            final_poses = [self.goal_pose]

        elif self.strategy == FirstApproachStrategy.HEADING_CORRECTION_ONLY:
            final_poses = self._get_heading_correction_poses()

        elif self.strategy == FirstApproachStrategy.LATERAL_CORRECTION:
            final_poses = self._get_lateral_correction_poses()

        elif self.strategy == FirstApproachStrategy.BACKUP_THEN_HEADING_CORRECTION:
            final_poses = self._get_backup_then_heading_poses()

        elif self.strategy == FirstApproachStrategy.BACKUP_THEN_LATERAL:
            final_poses = self._get_backup_then_lateral_poses()

        elif self.strategy == FirstApproachStrategy.REPOSITIONING_NEEDED:
            raise RuntimeError("Robot is not behind the goal, repositioning needed")

        else:
            raise ValueError(f"Unknown strategy: {self.strategy}")

        # For troubleshooting
        track = Track()
        track.waypoints.append(self.current_pose.to_proto())
        for i, pose in enumerate(final_poses):
            track.waypoints.append(pose.to_proto())
            self.print_log(pose, f"Approach Segment {i}")

        track.waypoints.append(self.goal_pose.to_proto())

        filename = Path("../track_plotter/approach_track.json")
        proto_to_json_file(filename, track)

        return final_poses

    def print_log(self, pose: Pose3F64, pose_name: str):
        # Always print goal pose first
        goal_pose = self.goal_pose
        goal_translation = goal_pose.a_from_b.translation
        goal_heading = goal_pose.a_from_b.rotation.log()[-1]
        print(f"Goal | Pose: x={goal_translation[0]:.3f}, y={goal_translation[1]:.3f}, heading: {goal_heading:.3f}")

        # Always print current pose
        current_pose = self.current_pose
        current_translation = current_pose.a_from_b.translation
        current_heading = current_pose.a_from_b.rotation.log()[-1]
        print(
            f"Current | Pose: x={current_translation[0]:.3f}, y={current_translation[1]:.3f}, "
            f"heading: {current_heading:.3f}"
        )

        translation = pose.a_from_b.translation
        heading = pose.a_from_b.rotation.log()[-1]
        print(f"{pose_name} | Pose: x={translation[0]:.3f}, y={translation[1]:.3f}, heading: {heading:.3f}")


if __name__ == "__main__":
    zero_tangent = np.zeros((6, 1), dtype=np.float64)
    goal_pose: Pose3F64 = Pose3F64(
        a_from_b=Isometry3F64(), frame_a="world", frame_b="robot", tangent_of_b_in_a=zero_tangent
    )
    robot_pose = Pose3F64(
        a_from_b=Isometry3F64(
            translation=np.array([-0.2, -5.0, 0]), rotation=Rotation3F64.Rz(-np.pi / 4)
        ),  # 45 degrees
        frame_a="world",
        frame_b="robot",
    )

    planner = ApproachPlanner(goal_pose=goal_pose, current_pose=robot_pose)
    poses: List[Pose3F64] = planner.get_final_poses()
    print(f"Strategy: {planner.strategy}")
    print(f"Got: {len(poses)} poses")
    track = Track()
    track.waypoints.append(robot_pose.to_proto())
    for pose in poses:
        track.waypoints.append(pose.to_proto())

    filename = Path("../track_plotter/approach_track.json")
    proto_to_json_file(filename, track)
