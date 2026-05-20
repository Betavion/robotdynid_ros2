"""Launch offline identification/codegen as a process."""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    manifest = LaunchConfiguration("manifest")
    languages = LaunchConfiguration("codegen_languages")

    return LaunchDescription(
        [
            DeclareLaunchArgument("manifest", default_value=""),
            DeclareLaunchArgument("codegen_languages", default_value="c,cpp"),
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "run",
                    "robotdynid_ros2",
                    "robotdynid-identify-codegen",
                    "--manifest",
                    manifest,
                    "--export-code",
                    "--codegen-languages",
                    languages,
                ],
                output="screen",
            ),
        ]
    )
