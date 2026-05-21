from pathlib import Path

from robotdynid_ros2.cli.export_runtime import _params_header, _read_vector
from robotdynid_ros2.trajectory.excitation import generate_sine_trajectory
from robotdynid_ros2.trajectory.follow_joint_trajectory_client import trajectory_points_from_data
from robotdynid_ros2.trajectory.schema import read_trajectory_csv, write_trajectory_csv


def test_params_header_uses_namespace_and_array_sizes() -> None:
    header = _params_header("robotdynid::generated", [1.0, 2.0], [0.1])

    assert "namespace robotdynid {" in header
    assert "namespace generated {" in header
    assert "std::array<double, 2> kLinearParameters" in header
    assert "std::array<double, 1> kStribeckParameters" in header


def test_read_vector_accepts_named_csv(tmp_path: Path) -> None:
    path = tmp_path / "identified_linear_parameters.csv"
    path.write_text("name,value\nbip01,1.5\nbip02,-2.0\n", encoding="utf-8")

    assert _read_vector(path) == [1.5, -2.0]


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
