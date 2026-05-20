from pathlib import Path

import pytest

from robotdynid_ros2.data.csv_writer import SplitDatasetCsvWriter
from robotdynid_ros2.data.dataset_schema import motion_columns, torque_columns, validate_columns
from robotdynid_ros2.data.manifest import build_collection_manifest, read_manifest, write_manifest


def test_split_csv_writer_uses_robotdynid_column_schema(tmp_path: Path) -> None:
    with SplitDatasetCsvWriter(tmp_path / "data", dof=2) as writer:
        writer.append(1.25, [0.1, 0.2], [0.3, 0.4], [1.0, 2.0])

    motion = validate_columns(tmp_path / "data" / "motion.csv", motion_columns(2))
    torque = validate_columns(tmp_path / "data" / "torque_measure_data.csv", torque_columns(2))

    assert motion.row_count == 1
    assert torque.row_count == 1


def test_split_csv_writer_rejects_wrong_vector_lengths(tmp_path: Path) -> None:
    with SplitDatasetCsvWriter(tmp_path / "data", dof=2) as writer:
        with pytest.raises(ValueError, match="length 2"):
            writer.append(1.0, [0.1], [0.2, 0.3], [1.0, 2.0])


def test_manifest_roundtrip_records_absolute_paths(tmp_path: Path) -> None:
    manifest = build_collection_manifest(
        run_dir=tmp_path,
        data_dir=tmp_path / "data",
        motion_csv=tmp_path / "data" / "motion.csv",
        torque_csv=tmp_path / "data" / "torque_measure_data.csv",
        joint_names=["j1", "j2"],
        joint_state_topic="/joint_states",
        sample_count=12,
    )

    manifest_path = write_manifest(tmp_path / "manifest.yaml", manifest)
    loaded = read_manifest(manifest_path)

    assert loaded["robot"]["dof"] == 2
    assert loaded["collection"]["sample_count"] == 12
    assert Path(loaded["data"]["motion_csv"]).is_absolute()
