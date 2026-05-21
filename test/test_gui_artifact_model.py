from __future__ import annotations

import json
from pathlib import Path

from robotdynid_ros2.data.manifest import build_collection_manifest, write_manifest
from robotdynid_ros2.gui.artifact_model import inspect_run, scan_runs


def test_inspect_run_detects_artifacts(tmp_path: Path) -> None:
    run_dir = tmp_path / "20260521_120000"
    data_dir = run_dir / "data"
    data_dir.mkdir(parents=True)
    motion = data_dir / "motion.csv"
    torque = data_dir / "torque_measure_data.csv"
    trajectory = run_dir / "excitation.csv"
    motion.write_text("timestamp,joint1_position\n0,0\n", encoding="utf-8")
    torque.write_text("timestamp,joint1_torque\n0,0\n", encoding="utf-8")
    trajectory.write_text("time_from_start,joint1_position,joint1_velocity,joint1_acceleration\n0,0,0,0\n", encoding="utf-8")
    (run_dir / "excitation_report.json").write_text("{}", encoding="utf-8")
    (run_dir / "excitation_validation.json").write_text("{}", encoding="utf-8")
    manifest = build_collection_manifest(
        run_dir=run_dir,
        data_dir=data_dir,
        motion_csv=motion,
        torque_csv=torque,
        joint_names=["joint1"],
        joint_state_topic="/joint_states",
        sample_count=10,
        trajectory_csv=trajectory,
    )
    write_manifest(run_dir / "manifest.yaml", manifest)

    identify = run_dir / "identify"
    identify.mkdir()
    (identify / "identify_result.json").write_text(
        json.dumps({"sample_count": 10, "rmse_history": [[1.0], [0.25]]}),
        encoding="utf-8",
    )
    (identify / "prediction.png").write_bytes(b"png")
    (identify / "codegen").mkdir()

    artifacts = inspect_run(run_dir)

    assert artifacts.has_manifest
    assert artifacts.has_trajectory
    assert artifacts.has_identification
    assert artifacts.has_codegen
    assert artifacts.sample_count == 10
    assert artifacts.rmse == (0.25,)
    assert artifacts.status_labels() == ["trajectory", "validated", "recorded", "identified", "codegen"]


def test_scan_runs_skips_empty_directories(tmp_path: Path) -> None:
    (tmp_path / "empty").mkdir()
    run_dir = tmp_path / "with_result"
    run_dir.mkdir()
    (run_dir / "identify_result.json").write_text(json.dumps({"rmse_history": [[0.1]]}), encoding="utf-8")

    runs = scan_runs(tmp_path)

    assert [run.run_dir.name for run in runs] == ["with_result"]
