"""Record a dataset while optionally sending a configured trajectory."""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from robotdynid_ros2.config import as_float, as_int, as_list, read_config, recorder_params_from_config, trajectory_config


def _arg(context, name: str) -> str:
    return LaunchConfiguration(name).perform(context)


def _setup(context, *args, **kwargs):  # noqa: ANN001
    del args, kwargs
    config = read_config(_arg(context, "config"))
    params = recorder_params_from_config(config)
    trajectory = trajectory_config(config)

    for key in ("output_root", "run_name", "joint_state_topic", "estimate_joint_state_topic", "urdf_path"):
        raw = _arg(context, key)
        if raw != "":
            params[key] = raw
    if _arg(context, "joint_names"):
        params["joint_names"] = as_list(_arg(context, "joint_names"))
    for key in ("duration_sec", "flush_period_sec"):
        raw = _arg(context, key)
        if raw != "":
            params[key] = as_float(raw)
    if _arg(context, "queue_max_samples"):
        params["queue_max_samples"] = as_int(_arg(context, "queue_max_samples"))

    trajectory_override = _arg(context, "trajectory_csv")
    trajectory_csv = trajectory_override or trajectory["csv_path"]
    action_name = _arg(context, "action_name") or trajectory["action_name"]
    send_delay_sec = as_float(_arg(context, "send_delay_sec") or trajectory["send_delay_sec"], 1.0)
    if trajectory_csv:
        params["commanded_trajectory_csv"] = trajectory_csv

    recorder = Node(
        package="robotdynid_ros2",
        executable="dataset_recorder",
        name="robotdynid_dataset_recorder",
        output="screen",
        parameters=[params],
    )
    actions = [recorder]
    send_actions = []
    if trajectory_csv and (trajectory_override or trajectory["enabled"]):
        send_actions.append(
            TimerAction(
                period=send_delay_sec,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "ros2",
                            "run",
                            "robotdynid_ros2",
                            "robotdynid-send-trajectory",
                            "--config",
                            _arg(context, "config"),
                            "--trajectory",
                            trajectory_csv,
                            "--action-name",
                            action_name,
                        ],
                        output="screen",
                    )
                ],
            )
        )
    generate_before_recording = _arg(context, "generate_trajectory").lower() == "true" or trajectory["generate_on_start"]
    if generate_before_recording:
        if not trajectory_csv:
            raise ValueError("trajectory.csv_path or trajectory_csv launch argument is required when generate_trajectory is true.")
        generate_cmd = [
            "ros2",
            "run",
            "robotdynid_ros2",
            "robotdynid-generate-excitation",
            "--config",
            _arg(context, "config"),
            "--output",
            trajectory_csv,
        ]
        if trajectory["validate_enabled"]:
            generate_cmd.append("--validate")
        generator = ExecuteProcess(cmd=generate_cmd, output="screen")
        actions = [generator, RegisterEventHandler(OnProcessExit(target_action=generator, on_exit=[recorder] + send_actions))]
    else:
        actions.extend(send_actions)
    return actions


def generate_launch_description() -> LaunchDescription:
    default_config = PathJoinSubstitution([FindPackageShare("robotdynid_ros2"), "config", "generic_joint_state.yaml"])
    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=default_config),
            DeclareLaunchArgument("output_root", default_value=""),
            DeclareLaunchArgument("run_name", default_value=""),
            DeclareLaunchArgument("joint_names", default_value=""),
            DeclareLaunchArgument("joint_state_topic", default_value=""),
            DeclareLaunchArgument("estimate_joint_state_topic", default_value=""),
            DeclareLaunchArgument("urdf_path", default_value=""),
            DeclareLaunchArgument("duration_sec", default_value=""),
            DeclareLaunchArgument("flush_period_sec", default_value=""),
            DeclareLaunchArgument("queue_max_samples", default_value=""),
            DeclareLaunchArgument("trajectory_csv", default_value=""),
            DeclareLaunchArgument("generate_trajectory", default_value="false"),
            DeclareLaunchArgument("action_name", default_value=""),
            DeclareLaunchArgument("send_delay_sec", default_value=""),
            OpaqueFunction(function=_setup),
        ]
    )
