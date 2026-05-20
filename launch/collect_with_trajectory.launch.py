"""Record a dataset while sending a FollowJointTrajectory CSV excitation."""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    recorder = Node(
        package="robotdynid_ros2",
        executable="dataset_recorder",
        name="robotdynid_dataset_recorder",
        output="screen",
        parameters=[
            {
                "output_root": LaunchConfiguration("output_root"),
                "run_name": LaunchConfiguration("run_name"),
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
    trajectory_sender = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "robotdynid_ros2",
            "robotdynid-send-trajectory",
            "--trajectory",
            LaunchConfiguration("trajectory_csv"),
            "--action-name",
            LaunchConfiguration("action_name"),
        ],
        output="screen",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("output_root", default_value="runs"),
            DeclareLaunchArgument("run_name", default_value=""),
            DeclareLaunchArgument("joint_names", default_value="[]"),
            DeclareLaunchArgument("joint_state_topic", default_value="/joint_states"),
            DeclareLaunchArgument("estimate_joint_state_topic", default_value=""),
            DeclareLaunchArgument("urdf_path", default_value=""),
            DeclareLaunchArgument("duration_sec", default_value="30.0"),
            DeclareLaunchArgument("flush_period_sec", default_value="0.01"),
            DeclareLaunchArgument("queue_max_samples", default_value="0"),
            DeclareLaunchArgument("trajectory_csv", default_value=""),
            DeclareLaunchArgument("action_name", default_value="/joint_trajectory_controller/follow_joint_trajectory"),
            recorder,
            TimerAction(period=1.0, actions=[trajectory_sender]),
        ]
    )
