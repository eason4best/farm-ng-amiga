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
from pathlib import Path

import numpy as np
from approach_planner import ApproachPlanner
from approach_planner import FirstApproachStrategy
from farm_ng.core.events_file_writer import proto_to_json_file
from farm_ng.track.track_pb2 import Track
from farm_ng_core_pybind import Isometry3F64
from farm_ng_core_pybind import Pose3F64
from farm_ng_core_pybind import Rotation3F64


def create_goal_pose(x=0.0, y=0.0, heading_deg=0.0):
    """Create a goal pose with given position and heading."""
    return Pose3F64(
        a_from_b=Isometry3F64(translation=np.array([x, y, 0.0]), rotation=Rotation3F64.Rz(np.radians(heading_deg))),
        frame_a="world",
        frame_b="robot",
    )


def create_robot_pose(x, y, heading_deg):
    """Create a robot pose with given position and heading."""
    return Pose3F64(
        a_from_b=Isometry3F64(translation=np.array([x, y, 0.0]), rotation=Rotation3F64.Rz(np.radians(heading_deg))),
        frame_a="world",
        frame_b="robot",
    )


def run_test_case(case_name, goal_pose, robot_pose, expected_strategy=None):
    """Run a single test case and print results."""
    print(f"\n{'='*60}")
    print(f"TEST CASE: {case_name}")
    print("{'='*60}")

    # Extract positions and headings for display
    goal_pos = goal_pose.a_from_b.translation
    goal_heading = np.degrees(goal_pose.a_from_b.rotation.log()[-1])
    robot_pos = robot_pose.a_from_b.translation
    robot_heading = np.degrees(robot_pose.a_from_b.rotation.log()[-1])

    print(f"Goal:  ({goal_pos[0]:.1f}, {goal_pos[1]:.1f}) heading: {goal_heading:.0f}°")
    print(f"Robot: ({robot_pos[0]:.1f}, {robot_pos[1]:.1f}) heading: {robot_heading:.0f}°")

    try:
        planner = ApproachPlanner(robot_pose, goal_pose)

        print("\nAnalysis:")
        print(f"  World deltas - X: {planner.delta_x:.2f}m, Y: {planner.delta_y:.2f}m")
        print(f"  Robot in goal frame: ({planner.robot_in_goal_frame[0]:.2f}, {planner.robot_in_goal_frame[1]:.2f})")
        print(f"  Is behind goal: {planner.is_behind_goal}")
        print(f"  Heading difference: {np.degrees(planner.delta_heading):.1f}°")
        print(f"  Selected strategy: {planner.strategy.value}")

        if expected_strategy:
            status = "✅" if planner.strategy == expected_strategy else "❌"
            print(f"  Expected strategy: {expected_strategy.value} {status}")

        # Handle REPOSITIONING_NEEDED case
        if planner.strategy == FirstApproachStrategy.REPOSITIONING_NEEDED:
            print("  ⚠️  Strategy is REPOSITIONING_NEEDED - no poses generated")
            print("  Robot needs to be manually repositioned behind the goal")
            return planner.strategy

        poses = planner.get_final_poses()
        print(f"  Generated {len(poses)} poses")

        # Save track for visualization
        track = Track()
        track.waypoints.append(robot_pose.to_proto())
        for pose in poses:
            track.waypoints.append(pose.to_proto())

        filename = Path(f"../track_plotter/{case_name.lower().replace(' ', '_')}.json")
        proto_to_json_file(filename, track)
        print(f"  Saved track to: {filename}")

        return planner.strategy

    except RuntimeError as e:
        # Handle expected RuntimeErrors (like repositioning needed)
        print(f"⚠️  EXPECTED ERROR: {e}")
        if "repositioning" in str(e).lower():
            return FirstApproachStrategy.REPOSITIONING_NEEDED
        return None

    except Exception as e:
        print(f"❌ UNEXPECTED ERROR: {e}")
        return None


def run_all_tests():
    """Run comprehensive test suite for ApproachPlanner."""

    print("🧪 APPROACH PLANNER TEST SUITE")
    print("=" * 60)

    test_results = []

    # Test Case 1: Current working case - BACKUP_THEN_LATERAL
    test_results.append(
        run_test_case(
            "Case 1 - Current Working",
            goal_pose=create_goal_pose(0, 0, 0),
            robot_pose=create_robot_pose(-0.2, -5.0, -45),
            expected_strategy=FirstApproachStrategy.BACKUP_THEN_LATERAL,
        )
    )

    # Test Case 2: Goal facing South - robot should be behind when X > goal_X
    test_results.append(
        run_test_case(
            "Case 2 - Goal South Facing",
            goal_pose=create_goal_pose(0, 0, 180),
            robot_pose=create_robot_pose(0.2, -5.0, 45),
            expected_strategy=FirstApproachStrategy.BACKUP_THEN_LATERAL,
        )
    )

    # Test Case 3: Perfect alignment - robot behind goal with same heading
    test_results.append(
        run_test_case(
            "Case 3 - Perfect Alignment",
            goal_pose=create_goal_pose(0, 0, 90),
            robot_pose=create_robot_pose(0, -2.0, 90),
            expected_strategy=FirstApproachStrategy.DIRECT_AB,
        )
    )

    # Test Case 4: Goal facing East
    test_results.append(
        run_test_case(
            "Case 4 - Goal East Facing",
            goal_pose=create_goal_pose(0, 0, -90),
            robot_pose=create_robot_pose(0.2, 2.0, 0),
            expected_strategy=FirstApproachStrategy.BACKUP_THEN_LATERAL,
        )
    )

    # Test Case 5: DIRECT_AB scenario - close with good alignment
    test_results.append(
        run_test_case(
            "Case 5 - Direct AB",
            goal_pose=create_goal_pose(0, 0, 0),
            robot_pose=create_robot_pose(-3.0, 0.05, 2),  # Far enough, small lateral, small heading
            expected_strategy=FirstApproachStrategy.DIRECT_AB,
        )
    )

    # Test Case 6: HEADING_CORRECTION_ONLY - good position, bad heading
    test_results.append(
        run_test_case(
            "Case 6 - Heading Correction Only",
            goal_pose=create_goal_pose(0, 0, 0),
            robot_pose=create_robot_pose(-3.0, 0.05, 45),  # Far enough, small lateral, large heading
            expected_strategy=FirstApproachStrategy.HEADING_CORRECTION_ONLY,
        )
    )

    # Test Case 7: LATERAL_CORRECTION - far enough for lateral movement
    test_results.append(
        run_test_case(
            "Case 7 - Lateral Correction",
            goal_pose=create_goal_pose(0, 0, 0),
            robot_pose=create_robot_pose(-6.0, 2.0, 0),  # Far enough, large lateral, good heading
            expected_strategy=FirstApproachStrategy.LATERAL_CORRECTION,
        )
    )

    # Test Case 8: BACKUP_THEN_HEADING - close with small lateral offset
    test_results.append(
        run_test_case(
            "Case 8 - Backup Then Heading",
            goal_pose=create_goal_pose(0, 0, 0),
            robot_pose=create_robot_pose(-1.0, 0.05, 45),  # Close, small lateral, bad heading
            expected_strategy=FirstApproachStrategy.BACKUP_THEN_HEADING_CORRECTION,
        )
    )

    # Test Case 9: REPOSITIONING_NEEDED - robot ahead of goal
    test_results.append(
        run_test_case(
            "Case 9 - Repositioning Needed",
            goal_pose=create_goal_pose(0, 0, 0),
            robot_pose=create_robot_pose(2.0, 1.0, 180),  # Ahead of goal
            expected_strategy=FirstApproachStrategy.REPOSITIONING_NEEDED,
        )
    )

    # Test Case 10: Goal at different position - not at origin
    test_results.append(
        run_test_case(
            "Case 10 - Goal Not At Origin",
            goal_pose=create_goal_pose(10, 5, 45),
            robot_pose=create_robot_pose(8, 3, 0),
            expected_strategy=None,  # Let's see what it picks
        )
    )

    # Test Case 11: Large negative coordinates
    test_results.append(
        run_test_case(
            "Case 11 - Large Negative Coords",
            goal_pose=create_goal_pose(-20, -15, -135),
            robot_pose=create_robot_pose(-25, -18, 90),
            expected_strategy=None,
        )
    )

    # Test Case 12: Edge case - exactly at thresholds
    test_results.append(
        run_test_case(
            "Case 12 - Threshold Edge Case",
            goal_pose=create_goal_pose(0, 0, 0),
            robot_pose=create_robot_pose(-2.0, 0.10, 3),  # Exactly at MIN_BACKUP_DISTANCE and LATERAL_THRESHOLD
            expected_strategy=None,
        )
    )

    # Summary
    print(f"\n{'='*60}")
    print("TEST SUMMARY")
    print(f"{'='*60}")

    success_count = sum(1 for result in test_results if result is not None)
    total_count = len(test_results)

    print(f"Completed: {success_count}/{total_count} tests")

    # Count strategy distribution
    strategy_counts = {}
    for result in test_results:
        if result:
            strategy_counts[result] = strategy_counts.get(result, 0) + 1

    print("\nStrategy Distribution:")
    for strategy, count in strategy_counts.items():
        print(f"  {strategy.value}: {count}")


if __name__ == "__main__":
    run_all_tests()
