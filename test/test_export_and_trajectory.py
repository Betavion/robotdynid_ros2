from pathlib import Path

from robotdynid_ros2.cli.export_runtime import _params_header, _read_vector
from robotdynid_ros2.trajectory.excitation import generate_sine_trajectory


def test_params_header_uses_namespace_and_array_sizes() -> None:
    header = _params_header("robotdynid::generated", [1.0, 2.0], [0.1])

    assert "namespace robotdynid {" in header
    assert "namespace generated {" in header
    assert "std::array<double, 2> kThetaLin" in header
    assert "std::array<double, 1> kQds" in header


def test_read_vector_accepts_named_csv(tmp_path: Path) -> None:
    path = tmp_path / "theta_lin.csv"
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
    assert set(rows[0]) == {"time_from_start", "j1", "j2"}
    assert rows[-1]["time_from_start"] == 0.1
