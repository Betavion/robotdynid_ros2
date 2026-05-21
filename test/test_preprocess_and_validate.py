from pathlib import Path

import numpy as np

from robotdynid_ros2.config import preprocessing_config
from robotdynid_ros2.data.csv_writer import SplitDatasetCsvWriter
from robotdynid_ros2.preprocessing.acceleration import preprocess_split_dataset
from robotdynid_ros2.trajectory.excitation import generate_excitation_trajectory
from robotdynid_ros2.trajectory.schema import write_trajectory_csv
from robotdynid_ros2.trajectory.urdf_limits import parse_urdf_joint_limits
from robotdynid_ros2.trajectory.validate import validate_trajectory_data


def _write_two_joint_urdf(path: Path) -> None:
    path.write_text(
        """
<robot name="two_joint">
  <link name="base"/>
  <link name="l1"/>
  <link name="l2"/>
  <joint name="j1" type="revolute">
    <parent link="base"/>
    <child link="l1"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.0" upper="1.0" velocity="2.0" effort="20.0"/>
  </joint>
  <joint name="j2" type="revolute">
    <parent link="l1"/>
    <child link="l2"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.5" upper="1.5" velocity="2.0" effort="20.0"/>
  </joint>
</robot>
""",
        encoding="utf-8",
    )


def test_urdf_scaled_excitation_validates(tmp_path: Path) -> None:
    from robotdynid_ros2.trajectory.excitation import ExcitationSettings

    urdf = tmp_path / "robot.urdf"
    _write_two_joint_urdf(urdf)
    settings = ExcitationSettings(
        joint_names=("j1", "j2"),
        urdf_path=urdf,
        profile="safe_multisine",
        duration=2.0,
        sample_period=0.05,
        harmonics=3,
        base_frequency=0.2,
        position_margin_ratio=0.1,
        velocity_scale=0.5,
        acceleration_scale=0.5,
        low_speed_ratio=0.25,
        search_candidates=2,
        random_seed=1,
        transition_duration=1.0,
        center=(0.0, 0.0),
        home_position=(0.0, 0.0),
        acceleration_limits=(4.0, 4.0),
        score_regressor=False,
        score_sample_limit=20,
    )

    data, report = generate_excitation_trajectory(settings)
    validation = validate_trajectory_data(
        data,
        {
            "robot": {"urdf_path": str(urdf), "joint_names": ["j1", "j2"]},
            "trajectory": {
                "generation": {
                    "velocity_scale": 0.5,
                    "acceleration_scale": 0.5,
                    "acceleration_limits": [4.0, 4.0],
                }
            },
        },
    )

    assert data.dof == 2
    assert report["sample_count"] == data.sample_count
    assert validation["valid"] is True
    assert [limit.name for limit in parse_urdf_joint_limits(urdf)] == ["j1", "j2"]


def test_preprocess_uses_fitted_command_acceleration(tmp_path: Path) -> None:
    from robotdynid_ros2.trajectory.excitation import generate_sine_trajectory
    from robotdynid_ros2.trajectory.schema import read_trajectory_csv

    rows = generate_sine_trajectory(
        joint_names=["j1", "j2"],
        duration=1.0,
        sample_period=0.02,
        amplitude=0.2,
        frequency=0.5,
        center=[0.0, 0.0],
    )
    trajectory_path = tmp_path / "trajectory.csv"
    trajectory_path.write_text(
        ",".join(rows[0].keys()) + "\n" + "\n".join(",".join(str(value) for value in row.values()) for row in rows) + "\n",
        encoding="utf-8",
    )
    command = read_trajectory_csv(trajectory_path)
    with SplitDatasetCsvWriter(tmp_path / "raw", dof=2) as writer:
        for index, time_value in enumerate(command.time):
            writer.append(time_value, command.position[index], command.velocity[index], [1.0, 2.0])

    report = preprocess_split_dataset(
        motion_csv=tmp_path / "raw" / "motion.csv",
        torque_csv=tmp_path / "raw" / "torque_measure_data.csv",
        dof=2,
        output_dir=tmp_path / "preprocess",
        config=preprocessing_config(
            {
                "preprocessing": {
                    "acceleration_source": "fitted",
                    "time_alignment": {"max_offset_sec": 0.05, "offset_grid_count": 11},
                    "fitting": {"max_position_rmse": 1e-9},
                }
            }
        ),
        trajectory_csv=trajectory_path,
    )

    motion = np.genfromtxt(report["output"]["motion_csv"], delimiter=",", names=True)
    assert report["method"]["method"] == "generated_profile_affine_fit"
    assert "joint1_acceleration" in motion.dtype.names
