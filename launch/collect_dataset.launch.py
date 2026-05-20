"""Launch the robot dynamics dataset recorder."""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    output_root = LaunchConfiguration("output_root")
    run_name = LaunchConfiguration("run_name")
    joint_names = LaunchConfiguration("joint_names")
    joint_state_topic = LaunchConfiguration("joint_state_topic")
    estimate_joint_state_topic = LaunchConfiguration("estimate_joint_state_topic")
    urdf_path = LaunchConfiguration("urdf_path")
    duration_sec = LaunchConfiguration("duration_sec")
    min_sample_period_sec = LaunchConfiguration("min_sample_period_sec")
    flush_period_sec = LaunchConfiguration("flush_period_sec")
    queue_max_samples = LaunchConfiguration("queue_max_samples")
    shutdown_on_finish = LaunchConfiguration("shutdown_on_finish")
    allow_missing_effort = LaunchConfiguration("allow_missing_effort")

    return LaunchDescription(
        [
            DeclareLaunchArgument("output_root", default_value="runs"),
            DeclareLaunchArgument("run_name", default_value=""),
            DeclareLaunchArgument("joint_names", default_value="[]"),
            DeclareLaunchArgument("joint_state_topic", default_value="/joint_states"),
            DeclareLaunchArgument("estimate_joint_state_topic", default_value=""),
            DeclareLaunchArgument("urdf_path", default_value=""),
            DeclareLaunchArgument("duration_sec", default_value="0.0"),
            DeclareLaunchArgument("min_sample_period_sec", default_value="0.0"),
            DeclareLaunchArgument("flush_period_sec", default_value="0.01"),
            DeclareLaunchArgument("queue_max_samples", default_value="0"),
            DeclareLaunchArgument("shutdown_on_finish", default_value="false"),
            DeclareLaunchArgument("allow_missing_effort", default_value="false"),
            Node(
                package="robotdynid_ros2",
                executable="dataset_recorder",
                name="robotdynid_dataset_recorder",
                output="screen",
                parameters=[
                    {
                        "output_root": output_root,
                        "run_name": run_name,
                        "joint_names": joint_names,
                        "joint_state_topic": joint_state_topic,
                        "estimate_joint_state_topic": estimate_joint_state_topic,
                        "urdf_path": urdf_path,
                        "duration_sec": duration_sec,
                        "min_sample_period_sec": min_sample_period_sec,
                        "flush_period_sec": flush_period_sec,
                        "queue_max_samples": queue_max_samples,
                        "shutdown_on_finish": shutdown_on_finish,
                        "allow_missing_effort": allow_missing_effort,
                        "auto_start": True,
                    }
                ],
            ),
        ]
    )
