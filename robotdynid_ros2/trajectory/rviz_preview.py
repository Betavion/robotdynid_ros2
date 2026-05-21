"""Publish trajectory CSVs as MoveIt RViz display trajectories."""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import rclpy
from builtin_interfaces.msg import Time
from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from robotdynid_ros2.config import read_config, trajectory_config
from robotdynid_ros2.trajectory.follow_joint_trajectory_client import trajectory_points_from_data
from robotdynid_ros2.trajectory.schema import TrajectoryData, read_trajectory_csv


DEFAULT_DISPLAY_TOPIC = "/display_planned_path"


def display_trajectory_from_data(
    data: TrajectoryData,
    *,
    model_id: str = "",
    frame_id: str = "",
    stamp: Time | None = None,
) -> DisplayTrajectory:
    """Build a MoveIt DisplayTrajectory message without touching robot state topics."""

    msg = DisplayTrajectory()
    msg.model_id = model_id
    msg.trajectory_start.joint_state.name = list(data.joint_names)
    msg.trajectory_start.joint_state.position = [float(value) for value in data.position[0]]
    msg.trajectory_start.joint_state.velocity = [float(value) for value in data.velocity[0]]
    msg.trajectory_start.joint_state.header.frame_id = frame_id

    trajectory = RobotTrajectory()
    trajectory.joint_trajectory.header.frame_id = frame_id
    trajectory.joint_trajectory.joint_names = list(data.joint_names)
    trajectory.joint_trajectory.points = trajectory_points_from_data(data)

    if stamp is not None:
        msg.trajectory_start.joint_state.header.stamp = stamp
        trajectory.joint_trajectory.header.stamp = stamp

    msg.trajectory.append(trajectory)
    return msg


def _display_qos() -> QoSProfile:
    return QoSProfile(
        depth=1,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        reliability=QoSReliabilityPolicy.RELIABLE,
    )


def publish_display_trajectory(
    node: Node,
    data: TrajectoryData,
    *,
    topic: str = DEFAULT_DISPLAY_TOPIC,
    model_id: str = "",
    frame_id: str = "",
    publish_seconds: float = 3.0,
    publish_period: float = 0.5,
) -> int:
    """Publish a display trajectory long enough for RViz to receive it."""

    publisher = node.create_publisher(DisplayTrajectory, topic, _display_qos())
    deadline = time.monotonic() + max(0.0, publish_seconds)
    period = max(0.05, publish_period)
    publish_count = 0

    while rclpy.ok() and (publish_count == 0 or time.monotonic() < deadline):
        stamp = node.get_clock().now().to_msg()
        msg = display_trajectory_from_data(data, model_id=model_id, frame_id=frame_id, stamp=stamp)
        publisher.publish(msg)
        publish_count += 1
        rclpy.spin_once(node, timeout_sec=period)
    return publish_count


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="", help="Unified robotdynid_ros2 config file.")
    parser.add_argument("--trajectory", default="", help="Trajectory CSV to preview.")
    parser.add_argument("--topic", default=DEFAULT_DISPLAY_TOPIC, help="MoveIt DisplayTrajectory topic for RViz.")
    parser.add_argument("--model-id", default="", help="Optional DisplayTrajectory model_id.")
    parser.add_argument("--frame-id", default="", help="Optional trajectory header frame_id.")
    parser.add_argument("--publish-seconds", type=float, default=3.0, help="Seconds to repeat the latched RViz preview.")
    parser.add_argument("--publish-period", type=float, default=0.5, help="Seconds between preview publications.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    config = trajectory_config(read_config(args.config))
    trajectory = args.trajectory or config["csv_path"] or config["output"]
    if not trajectory:
        raise ValueError("--trajectory is required unless trajectory.csv_path is configured.")
    expected_joints = config["joint_names"] or None
    data = read_trajectory_csv(Path(trajectory), expected_joint_names=expected_joints)

    rclpy.init()
    node = rclpy.create_node("robotdynid_rviz_trajectory_preview")
    try:
        count = publish_display_trajectory(
            node,
            data,
            topic=args.topic,
            model_id=args.model_id,
            frame_id=args.frame_id,
            publish_seconds=args.publish_seconds,
            publish_period=args.publish_period,
        )
        node.get_logger().info(f"Published {count} DisplayTrajectory message(s) to {args.topic}.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
