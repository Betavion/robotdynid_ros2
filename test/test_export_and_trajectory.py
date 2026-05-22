from pathlib import Path

import numpy as np

from robotdynid_ros2.cli.export_runtime import _read_vector, export_runtime
from robotdynid_ros2.trajectory.excitation import (
    ExcitationSettings,
    _candidate_score,
    generate_excitation_trajectory,
    generate_sine_trajectory,
)
from robotdynid_ros2.trajectory.follow_joint_trajectory_client import trajectory_points_from_data
from robotdynid_ros2.trajectory.rviz_preview import display_trajectory_from_data
from robotdynid_ros2.trajectory.schema import read_trajectory_csv, write_trajectory_csv


def test_read_vector_accepts_named_csv(tmp_path: Path) -> None:
    path = tmp_path / "identified_linear_parameters.csv"
    path.write_text("name,value\nbip01,1.5\nbip02,-2.0\n", encoding="utf-8")

    assert _read_vector(path) == [1.5, -2.0]


def test_export_runtime_uses_ros2_package_layout(tmp_path: Path) -> None:
    run_dir = tmp_path / "run" / "identify"
    codegen_dir = run_dir / "codegen" / "cpp"
    codegen_dir.mkdir(parents=True)
    (codegen_dir / "predict_tau.hpp").write_text("#pragma once\n", encoding="utf-8")
    (codegen_dir / "predict_tau.cpp").write_text('#include "predict_tau.hpp"\n', encoding="utf-8")
    (codegen_dir / "predict_tau.json").write_text('{"dof": 2}\n', encoding="utf-8")
    (run_dir / "identified_linear_parameters.csv").write_text("name,value\nbip01,1.0\n", encoding="utf-8")
    (run_dir / "identified_stribeck_parameters.csv").write_text("name,value\nstribeck1,0.1\n", encoding="utf-8")

    target = tmp_path / "sia_controllers"
    target.mkdir()
    (target / "package.xml").write_text("<package><name>sia_controllers</name></package>\n", encoding="utf-8")

    source, header, linear_params, stribeck_params, manifest = export_runtime(
        run_dir=run_dir,
        target_root=target,
    )

    assert source == target / "src/generated/robotdynid/predict_tau.cpp"
    assert header == target / "include/sia_controllers/generated/robotdynid/predict_tau.hpp"
    assert linear_params == target / "runtime/robotdynid/identified_linear_parameters.csv"
    assert stribeck_params == target / "runtime/robotdynid/identified_stribeck_parameters.csv"
    assert manifest == target / "src/generated/robotdynid/runtime_manifest.json"
    assert '#include "sia_controllers/generated/robotdynid/predict_tau.hpp"' in source.read_text(encoding="utf-8")
    assert linear_params.read_text(encoding="utf-8") == "name,value\nbip01,1.0\n"
    assert stribeck_params.read_text(encoding="utf-8") == "name,value\nstribeck1,0.1\n"


def test_generate_sine_trajectory_includes_all_joints() -> None:
    rows = generate_sine_trajectory(
        joint_names=["j1", "j2"],
        duration=0.1,
        sample_period=0.05,
        amplitude=0.2,
        frequency=0.5,
        center=[0.0, 1.0],
    )

    assert len(rows) == 3
    assert set(rows[0]) == {
        "time_from_start",
        "j1_position",
        "j1_velocity",
        "j1_acceleration",
        "j2_position",
        "j2_velocity",
        "j2_acceleration",
    }
    assert rows[-1]["time_from_start"] == 0.1


def test_trajectory_schema_roundtrip_and_message_fields(tmp_path: Path) -> None:
    rows = generate_sine_trajectory(
        joint_names=["j1", "j2"],
        duration=0.1,
        sample_period=0.05,
        amplitude=0.2,
        frequency=0.5,
        center=[0.0, 1.0],
    )
    path = tmp_path / "excitation.csv"
    path.write_text(
        ",".join(rows[0].keys()) + "\n" + "\n".join(",".join(str(value) for value in row.values()) for row in rows) + "\n",
        encoding="utf-8",
    )

    data = read_trajectory_csv(path, expected_joint_names=["j1", "j2"])
    copied = write_trajectory_csv(tmp_path / "copy.csv", data)
    points = trajectory_points_from_data(read_trajectory_csv(copied))

    assert data.position.shape == (3, 2)
    assert points[0].positions
    assert points[0].velocities
    assert points[0].accelerations


def test_display_trajectory_preview_message_is_visualization_only(tmp_path: Path) -> None:
    rows = generate_sine_trajectory(
        joint_names=["j1", "j2"],
        duration=0.1,
        sample_period=0.05,
        amplitude=0.2,
        frequency=0.5,
        center=[0.0, 1.0],
    )
    path = tmp_path / "excitation.csv"
    path.write_text(
        ",".join(rows[0].keys()) + "\n" + "\n".join(",".join(str(value) for value in row.values()) for row in rows) + "\n",
        encoding="utf-8",
    )

    data = read_trajectory_csv(path)
    msg = display_trajectory_from_data(data, frame_id="base_link")

    assert msg.trajectory_start.joint_state.name == ["j1", "j2"]
    assert msg.trajectory_start.joint_state.header.frame_id == "base_link"
    assert len(msg.trajectory) == 1
    assert msg.trajectory[0].joint_trajectory.joint_names == ["j1", "j2"]
    assert np.allclose(msg.trajectory[0].joint_trajectory.points[0].positions, data.position[0])


def test_composite_excitation_is_smooth_at_segment_boundaries(tmp_path: Path) -> None:
    urdf = tmp_path / "robot.urdf"
    urdf.write_text(
        """
<robot name="test_robot">
  <joint name="j1" type="revolute"><limit lower="-1.5" upper="1.5" velocity="2.0" effort="10.0" /></joint>
  <joint name="j2" type="revolute"><limit lower="-1.5" upper="1.5" velocity="2.0" effort="10.0" /></joint>
</robot>
""",
        encoding="utf-8",
    )
    settings = ExcitationSettings(
        joint_names=("j1", "j2"),
        urdf_path=urdf,
        profile="composite",
        duration=8.0,
        sample_period=0.01,
        harmonics=3,
        base_frequency=0.1,
        position_margin_ratio=0.2,
        velocity_scale=0.4,
        acceleration_scale=0.4,
        low_speed_ratio=0.2,
        search_candidates=3,
        random_seed=3,
        transition_duration=0.5,
        center=(0.0, 0.0),
        home_position=(0.0, 0.0),
        position_lower=None,
        position_upper=None,
        acceleration_limits=(2.0, 2.0),
        score_regressor=False,
        score_sample_limit=20,
    )

    data, report = generate_excitation_trajectory(settings)
    friction_report = next(segment for segment in report["segments"] if segment["name"] == "friction_sweep")
    gravity_report = next(segment for segment in report["segments"] if segment["name"] == "gravity_sweep")

    assert friction_report["profile"]["speed_level_count"] == 3
    assert gravity_report["profile"]["pose_count"] == 5
    elapsed = 0.0
    boundaries = []
    for segment in report["segments"]:
        duration = segment.get("duration")
        if duration is not None:
            elapsed += float(duration)
            boundaries.append(elapsed)
    for boundary in boundaries:
        index = int(np.argmin(np.abs(data.time - boundary)))
        assert np.allclose(data.position[index], [0.0, 0.0], atol=1e-9)
        assert np.allclose(data.velocity[index], [0.0, 0.0], atol=1e-9)
        assert np.allclose(data.acceleration[index], [0.0, 0.0], atol=1e-9)


def test_excitation_position_workspace_bounds_are_applied(tmp_path: Path) -> None:
    urdf = tmp_path / "robot.urdf"
    urdf.write_text(
        """
<robot name="test_robot">
  <joint name="j1" type="revolute"><limit lower="-3.0" upper="3.0" velocity="2.0" effort="10.0" /></joint>
  <joint name="j2" type="revolute"><limit lower="-3.0" upper="3.0" velocity="2.0" effort="10.0" /></joint>
</robot>
""",
        encoding="utf-8",
    )
    settings = ExcitationSettings(
        joint_names=("j1", "j2"),
        urdf_path=urdf,
        profile="safe_multisine",
        duration=2.0,
        sample_period=0.05,
        harmonics=3,
        base_frequency=0.2,
        position_margin_ratio=0.0,
        velocity_scale=0.5,
        acceleration_scale=0.5,
        low_speed_ratio=0.25,
        search_candidates=3,
        random_seed=7,
        transition_duration=1.0,
        center=(0.0, 0.0),
        home_position=(0.0, 0.0),
        position_lower=(-0.4, -0.3),
        position_upper=(0.5, 0.6),
        acceleration_limits=(4.0, 4.0),
        score_regressor=False,
        score_sample_limit=20,
    )

    data, report = generate_excitation_trajectory(settings)

    assert np.all(data.position[:, 0] >= -0.4 - 1e-12)
    assert np.all(data.position[:, 0] <= 0.5 + 1e-12)
    assert np.all(data.position[:, 1] >= -0.3 - 1e-12)
    assert np.all(data.position[:, 1] <= 0.6 + 1e-12)
    assert report["position_bounds"]["lower"] == [-0.4, -0.3]
    assert report["position_bounds"]["urdf_lower"] == [-3.0, -3.0]
    assert 0.0 < report["limits"]["dynamic_utilization_score"] <= 1.0
    assert 0.0 < report["limits"]["speed_utilization_score"] <= 1.0
    assert 0.0 < report["limits"]["inertial_utilization_score"] <= 1.0
    assert 0.0 < report["limits"]["aperiodicity_score"] <= 1.0
    assert len(report["limits"]["velocity_utilization"]) == 2
    candidates = report["segments"][0]["candidates"]
    assert candidates[0]["coefficients"]["frequency_mode"] == "common"
    assert candidates[1]["coefficients"]["frequency_mode"] == "detuned"
    assert candidates[1]["coefficients"]["phase_strategy"] == "schroeder"
    assert candidates[2]["coefficients"]["frequency_mode"] == "adaptive"
    assert len(candidates[1]["coefficients"]["effective_base_frequency"]) == 2


def test_multisine_candidate_score_prefers_speed_utilization() -> None:
    base = {
        "regressor": {"enabled": True, "rank": 12, "condition_number": 100.0},
        "limits": {
            "min_position_margin": [0.1, 0.2],
            "low_speed_ratio": 0.9,
            "inertial_utilization_score": 0.8,
            "aperiodicity_score": 0.6,
        },
    }
    slow = {
        **base,
        "limits": {
            **base["limits"],
            "dynamic_utilization_score": 0.4,
            "speed_utilization_score": 0.2,
        },
    }
    fast = {
        **base,
        "limits": {
            **base["limits"],
            "dynamic_utilization_score": 0.7,
            "speed_utilization_score": 0.7,
        },
    }

    assert _candidate_score(fast) > _candidate_score(slow)
