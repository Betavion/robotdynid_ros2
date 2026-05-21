from pathlib import Path

from robotdynid_ros2.config import identification_config, read_config, recorder_params_from_config, trajectory_config


def test_unified_config_drives_recorder_and_identification(tmp_path: Path) -> None:
    path = tmp_path / "config.yaml"
    path.write_text(
        """
robot:
  urdf_path: robot.urdf
  dof: 2
  joint_names:
    - j1
    - j2
run:
  output_root: runs
recording:
  joint_state_topic: /measured
  estimate_joint_state_topic: /estimated
identification:
  stride: 5
codegen:
  export_code: true
  languages: [c, cpp]
trajectory:
  enabled: true
  csv_path: trajectory.csv
  generation:
    position_lower: [-0.5, -0.6]
    position_upper: [0.5, 0.6]
    friction_speed_levels: 4
    gravity_pose_count: 6
""",
        encoding="utf-8",
    )

    config = read_config(path)
    recorder = recorder_params_from_config(config)
    identification = identification_config(config)
    trajectory = trajectory_config(config)

    assert recorder["joint_names"] == ["j1", "j2"]
    assert recorder["joint_state_topic"] == "/measured"
    assert identification["dof"] == 2
    assert identification["stride"] == 5
    assert identification["prediction_plot_stride"] == 0
    assert identification["prediction_plot_rate_hz"] == 10.0
    assert identification["export_code"] is True
    assert identification["codegen_languages"] == "c,cpp"
    assert trajectory["csv_path"] == "trajectory.csv"
    assert trajectory["enabled"] is True
    assert trajectory["joint_names"] == ["j1", "j2"]
    assert trajectory["position_lower"] == [-0.5, -0.6]
    assert trajectory["position_upper"] == [0.5, 0.6]
    assert trajectory["friction_speed_levels"] == 4
    assert trajectory["gravity_pose_count"] == 6
