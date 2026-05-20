"""Launch offline identification/codegen from a unified config file."""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from robotdynid_ros2.config import identification_config, read_config


def _arg(context, name: str) -> str:
    return LaunchConfiguration(name).perform(context)


def _setup(context, *args, **kwargs):  # noqa: ANN001
    del args, kwargs
    config_path = _arg(context, "config")
    identification = identification_config(read_config(config_path))
    cmd = [
        "ros2",
        "run",
        "robotdynid_ros2",
        "robotdynid-identify-codegen",
        "--config",
        config_path,
    ]
    manifest = _arg(context, "manifest") or str(identification.get("manifest", ""))
    if manifest:
        cmd.extend(["--manifest", manifest])
    languages = _arg(context, "codegen_languages") or identification["codegen_languages"]
    if languages:
        cmd.extend(["--codegen-languages", languages])
    if identification["export_code"]:
        cmd.append("--export-code")
    return [ExecuteProcess(cmd=cmd, output="screen")]


def generate_launch_description() -> LaunchDescription:
    default_config = PathJoinSubstitution([FindPackageShare("robotdynid_ros2"), "config", "generic_joint_state.yaml"])
    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=default_config),
            DeclareLaunchArgument("manifest", default_value=""),
            DeclareLaunchArgument("codegen_languages", default_value=""),
            OpaqueFunction(function=_setup),
        ]
    )
