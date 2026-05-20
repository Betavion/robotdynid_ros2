"""Launch the robot dynamics dataset recorder from a unified config file."""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from robotdynid_ros2.config import as_bool, as_float, as_int, as_list, read_config, recorder_params_from_config


def _arg(context, name: str) -> str:
    return LaunchConfiguration(name).perform(context)


def _override(params: dict[str, object], context, name: str, parser=str) -> None:  # noqa: ANN001
    raw = _arg(context, name)
    if raw != "":
        params[name] = parser(raw)


def _setup(context, *args, **kwargs):  # noqa: ANN001
    del args, kwargs
    params = recorder_params_from_config(read_config(_arg(context, "config")))
    _override(params, context, "output_root")
    _override(params, context, "run_name")
    _override(params, context, "joint_names", as_list)
    _override(params, context, "joint_state_topic")
    _override(params, context, "estimate_joint_state_topic")
    _override(params, context, "urdf_path")
    _override(params, context, "duration_sec", as_float)
    _override(params, context, "min_sample_period_sec", as_float)
    _override(params, context, "flush_period_sec", as_float)
    _override(params, context, "queue_max_samples", as_int)
    _override(params, context, "shutdown_on_finish", as_bool)
    _override(params, context, "allow_missing_effort", as_bool)

    return [
        Node(
            package="robotdynid_ros2",
            executable="dataset_recorder",
            name="robotdynid_dataset_recorder",
            output="screen",
            parameters=[params],
        )
    ]


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
            DeclareLaunchArgument("min_sample_period_sec", default_value=""),
            DeclareLaunchArgument("flush_period_sec", default_value=""),
            DeclareLaunchArgument("queue_max_samples", default_value=""),
            DeclareLaunchArgument("shutdown_on_finish", default_value=""),
            DeclareLaunchArgument("allow_missing_effort", default_value=""),
            OpaqueFunction(function=_setup),
        ]
    )
