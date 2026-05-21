"""Send a CSV trajectory through FollowJointTrajectory."""

from __future__ import annotations

import argparse
from pathlib import Path

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint

from robotdynid_ros2.config import read_config, trajectory_config
from robotdynid_ros2.trajectory.schema import TrajectoryData, read_trajectory_csv


def _duration_from_seconds(seconds: float):
    from builtin_interfaces.msg import Duration

    nanoseconds = int(round(seconds * 1e9))
    return Duration(sec=nanoseconds // 1_000_000_000, nanosec=nanoseconds % 1_000_000_000)


def trajectory_points_from_data(data: TrajectoryData) -> list[JointTrajectoryPoint]:
    points: list[JointTrajectoryPoint] = []
    for index, time_from_start in enumerate(data.time):
        point = JointTrajectoryPoint()
        point.positions = [float(value) for value in data.position[index]]
        point.velocities = [float(value) for value in data.velocity[index]]
        point.accelerations = [float(value) for value in data.acceleration[index]]
        point.time_from_start = _duration_from_seconds(float(time_from_start))
        points.append(point)
    return points


def _load_csv_trajectory(path: Path) -> tuple[list[str], list[JointTrajectoryPoint]]:
    data = read_trajectory_csv(path)
    return list(data.joint_names), trajectory_points_from_data(data)


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
    parser.add_argument("--config", default="", help="Unified robotdynid_ros2 config file.")
    parser.add_argument("--trajectory", default="")
    parser.add_argument("--action-name", default="")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    config = trajectory_config(read_config(args.config))
    trajectory = args.trajectory or config["csv_path"] or config["output"]
    action_name = args.action_name or config["action_name"]
    if not trajectory:
        raise ValueError("--trajectory is required unless trajectory.csv_path is configured.")
    joint_names, points = _load_csv_trajectory(Path(trajectory))
    rclpy.init()
    node = FollowJointTrajectoryCsvClient(action_name)
    try:
        if not node.send(joint_names, points):
            raise RuntimeError("Trajectory goal was rejected or failed.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
