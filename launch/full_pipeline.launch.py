"""Collect a dataset and then run offline identification/codegen."""

from __future__ import annotations

from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    default_run_name = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_root = LaunchConfiguration("output_root")
    run_name = LaunchConfiguration("run_name")
    manifest = PathJoinSubstitution([output_root, run_name, "manifest.yaml"])

    recorder = Node(
        package="robotdynid_ros2",
        executable="dataset_recorder",
        name="robotdynid_dataset_recorder",
        output="screen",
        parameters=[
            {
                "output_root": output_root,
                "run_name": run_name,
                "joint_names": LaunchConfiguration("joint_names"),
                "joint_state_topic": LaunchConfiguration("joint_state_topic"),
                "estimate_joint_state_topic": LaunchConfiguration("estimate_joint_state_topic"),
                "urdf_path": LaunchConfiguration("urdf_path"),
                "duration_sec": LaunchConfiguration("duration_sec"),
                "flush_period_sec": LaunchConfiguration("flush_period_sec"),
                "queue_max_samples": LaunchConfiguration("queue_max_samples"),
                "shutdown_on_finish": True,
                "auto_start": True,
            }
        ],
    )

    identify = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "robotdynid_ros2",
            "robotdynid-identify-codegen",
            "--manifest",
            manifest,
            "--export-code",
            "--codegen-languages",
            LaunchConfiguration("codegen_languages"),
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("output_root", default_value="runs"),
            DeclareLaunchArgument("run_name", default_value=default_run_name),
            DeclareLaunchArgument("joint_names", default_value="[]"),
            DeclareLaunchArgument("joint_state_topic", default_value="/joint_states"),
            DeclareLaunchArgument("estimate_joint_state_topic", default_value=""),
            DeclareLaunchArgument("urdf_path", default_value=""),
            DeclareLaunchArgument("duration_sec", default_value="30.0"),
            DeclareLaunchArgument("flush_period_sec", default_value="0.01"),
            DeclareLaunchArgument("queue_max_samples", default_value="0"),
            DeclareLaunchArgument("codegen_languages", default_value="c,cpp"),
            recorder,
            RegisterEventHandler(OnProcessExit(target_action=recorder, on_exit=[identify])),
        ]
    )
