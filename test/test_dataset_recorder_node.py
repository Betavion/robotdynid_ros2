from __future__ import annotations

from pathlib import Path

import rclpy

from robotdynid_ros2.nodes.dataset_recorder_node import DatasetRecorderNode


def test_dataset_recorder_accepts_string_array_joint_names(tmp_path: Path) -> None:
    params = tmp_path / "params.yaml"
    params.write_text(
        """
robotdynid_dataset_recorder:
  ros__parameters:
    joint_names:
    - joint1
    - joint2
    auto_start: false
""",
        encoding="utf-8",
    )

    if rclpy.ok():
        rclpy.shutdown()
    rclpy.init(args=["--ros-args", "--params-file", str(params)])
    node = None
    try:
        node = DatasetRecorderNode()
        assert node._joint_names == ["joint1", "joint2"]
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
