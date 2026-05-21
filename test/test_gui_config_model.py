from __future__ import annotations

from pathlib import Path

from robotdynid_ros2.gui.config_model import GuiConfigModel


def test_config_model_load_update_and_save(tmp_path: Path) -> None:
    config = tmp_path / "config.yaml"
    config.write_text(
        """
robot:
  urdf_path: robot.urdf
  dof: 2
  joint_names: [joint1, joint2]
run:
  output_root: runs
trajectory:
  action_name: /joint_trajectory_controller/follow_joint_trajectory
  generation:
    profile: composite
recording:
  joint_state_topic: /joint_states
""",
        encoding="utf-8",
    )
    (tmp_path / "robot.urdf").write_text("<robot name='r' />", encoding="utf-8")

    model = GuiConfigModel(config)
    summary = model.robot_summary()
    assert summary.dof == 2
    assert summary.joint_names == ("joint1", "joint2")
    assert summary.urdf_path == str((tmp_path / "robot.urdf").resolve())

    model.set_nested_value("trajectory", "generation", "duration", 12.0)
    model.set_value("run", "output_root", "runs2")
    saved = model.save()

    reloaded = GuiConfigModel(saved)
    assert reloaded.robot_summary().output_root == "runs2"
    assert reloaded.trajectory_values()["duration"] == 12.0
    assert "__config_dir" not in saved.read_text(encoding="utf-8")

