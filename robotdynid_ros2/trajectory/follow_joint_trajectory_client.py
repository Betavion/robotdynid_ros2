"""Send a CSV trajectory through FollowJointTrajectory."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint


def _duration_from_seconds(seconds: float):
    from builtin_interfaces.msg import Duration

    whole = int(seconds)
    return Duration(sec=whole, nanosec=int((seconds - whole) * 1e9))


def _load_csv_trajectory(path: Path) -> tuple[list[str], list[JointTrajectoryPoint]]:
    with path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None or "time_from_start" not in reader.fieldnames:
            raise ValueError("Trajectory CSV must contain a time_from_start column.")
        joint_names = [name for name in reader.fieldnames if name != "time_from_start"]
        points: list[JointTrajectoryPoint] = []
        for row in reader:
            point = JointTrajectoryPoint()
            point.positions = [float(row[name]) for name in joint_names]
            point.time_from_start = _duration_from_seconds(float(row["time_from_start"]))
            points.append(point)
    if not points:
        raise ValueError("Trajectory CSV contains no points.")
    return joint_names, points


class FollowJointTrajectoryCsvClient(Node):
    def __init__(self, action_name: str) -> None:
        super().__init__("robotdynid_trajectory_sender")
        self._client = ActionClient(self, FollowJointTrajectory, action_name)

    def send(self, joint_names: list[str], points: list[JointTrajectoryPoint]) -> bool:
        if not self._client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("FollowJointTrajectory action server is not available.")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joint_names
        goal.trajectory.points = points
        future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if handle is None or not handle.accepted:
            return False
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        return result is not None and result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trajectory", required=True)
    parser.add_argument("--action-name", default="/joint_trajectory_controller/follow_joint_trajectory")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    joint_names, points = _load_csv_trajectory(Path(args.trajectory))
    rclpy.init()
    node = FollowJointTrajectoryCsvClient(args.action_name)
    try:
        if not node.send(joint_names, points):
            raise RuntimeError("Trajectory goal was rejected or failed.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
