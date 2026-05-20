"""Collect a dataset and then run configured offline identification/codegen."""

from __future__ import annotations

from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from robotdynid_ros2.config import as_float, as_list, identification_config, read_config, recorder_params_from_config, trajectory_config
from robotdynid_ros2.paths import timestamped_run_dir


def _arg(context, name: str) -> str:
    return LaunchConfiguration(name).perform(context)


def _setup(context, *args, **kwargs):  # noqa: ANN001
    del args, kwargs
    config_path = _arg(context, "config")
    config = read_config(config_path)
    params = recorder_params_from_config(config)
    identification = identification_config(config)
    trajectory = trajectory_config(config)

    if _arg(context, "output_root"):
        params["output_root"] = _arg(context, "output_root")
    if _arg(context, "run_name"):
        params["run_name"] = _arg(context, "run_name")
    elif not params["run_name"]:
        params["run_name"] = datetime.now().strftime("%Y%m%d_%H%M%S")
    for key in ("joint_state_topic", "estimate_joint_state_topic", "urdf_path"):
        raw = _arg(context, key)
        if raw:
            params[key] = raw
    if _arg(context, "joint_names"):
        params["joint_names"] = as_list(_arg(context, "joint_names"))
    if _arg(context, "duration_sec"):
        params["duration_sec"] = as_float(_arg(context, "duration_sec"))

    run_dir = timestamped_run_dir(str(params["output_root"]), str(params["run_name"]))
    params["output_root"] = str(run_dir.parent)
    params["run_name"] = run_dir.name
    manifest = str(run_dir / "manifest.yaml")
    codegen_languages = _arg(context, "codegen_languages") or identification["codegen_languages"]

    recorder = Node(
        package="robotdynid_ros2",
        executable="dataset_recorder",
        name="robotdynid_dataset_recorder",
        output="screen",
        parameters=[{**params, "shutdown_on_finish": True, "auto_start": True}],
    )
    identify_cmd = [
        "ros2",
        "run",
        "robotdynid_ros2",
        "robotdynid-identify-codegen",
        "--config",
        config_path,
        "--manifest",
        manifest,
        "--codegen-languages",
        codegen_languages,
    ]
    if identification["export_code"]:
        identify_cmd.append("--export-code")

    actions = [
        recorder,
        RegisterEventHandler(
            OnProcessExit(
                target_action=recorder,
                on_exit=[ExecuteProcess(cmd=identify_cmd, output="screen")],
            )
        ),
    ]
    trajectory_override = _arg(context, "trajectory_csv")
    trajectory_csv = trajectory_override or trajectory["csv_path"]
    if trajectory_csv and (trajectory_override or trajectory["enabled"]):
        actions.append(
            TimerAction(
                period=as_float(_arg(context, "send_delay_sec") or trajectory["send_delay_sec"], 1.0),
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "ros2",
                            "run",
                            "robotdynid_ros2",
                            "robotdynid-send-trajectory",
                            "--config",
                            config_path,
                            "--trajectory",
                            trajectory_csv,
                        ],
                        output="screen",
                    )
                ],
            )
        )
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
            DeclareLaunchArgument("codegen_languages", default_value=""),
            DeclareLaunchArgument("trajectory_csv", default_value=""),
            DeclareLaunchArgument("send_delay_sec", default_value=""),
            OpaqueFunction(function=_setup),
        ]
    )
