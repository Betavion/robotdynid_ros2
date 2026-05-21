"""CSV schema helpers for dynamics-aware joint trajectories."""

from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path

import numpy as np


TIME_COLUMN = "time_from_start"
TIME_UNIT = "s"
POSITION_SUFFIX = "position"
VELOCITY_SUFFIX = "velocity"
ACCELERATION_SUFFIX = "acceleration"
TRAJECTORY_SUFFIXES = (POSITION_SUFFIX, VELOCITY_SUFFIX, ACCELERATION_SUFFIX)
TRAJECTORY_UNITS = {
    POSITION_SUFFIX: "rad",
    VELOCITY_SUFFIX: "rad/s",
    ACCELERATION_SUFFIX: "rad/s^2",
}


@dataclass(frozen=True)
class TrajectoryData:
    """Joint trajectory samples with position, velocity, and acceleration."""

    joint_names: tuple[str, ...]
    time: np.ndarray
    position: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray

    @property
    def sample_count(self) -> int:
        return int(self.time.shape[0])

    @property
    def dof(self) -> int:
        return len(self.joint_names)


def trajectory_column(joint_name: str, suffix: str) -> str:
    return f"{joint_name}_{suffix}"


def trajectory_columns(joint_names: list[str] | tuple[str, ...]) -> list[str]:
    return [TIME_COLUMN] + [
        trajectory_column(joint_name, suffix)
        for joint_name in joint_names
        for suffix in TRAJECTORY_SUFFIXES
    ]


def _split_trajectory_column(column: str) -> tuple[str, str] | None:
    for suffix in TRAJECTORY_SUFFIXES:
        marker = f"_{suffix}"
        if column.endswith(marker) and len(column) > len(marker):
            return column[: -len(marker)], suffix
    return None


def infer_joint_names(fieldnames: list[str]) -> tuple[str, ...]:
    """Infer joint order from a strict trajectory CSV header."""

    if not fieldnames or fieldnames[0] != TIME_COLUMN:
        raise ValueError(f"Trajectory CSV must start with '{TIME_COLUMN}'.")
    grouped: dict[str, set[str]] = {}
    joint_order: list[str] = []
    for column in fieldnames[1:]:
        parsed = _split_trajectory_column(column)
        if parsed is None:
            raise ValueError(
                "Trajectory CSV must use '<joint>_position', '<joint>_velocity' "
                "and '<joint>_acceleration' columns only."
            )
        joint_name, suffix = parsed
        if joint_name not in grouped:
            grouped[joint_name] = set()
            joint_order.append(joint_name)
        grouped[joint_name].add(suffix)

    missing = {
        joint_name: sorted(set(TRAJECTORY_SUFFIXES) - suffixes)
        for joint_name, suffixes in grouped.items()
        if set(TRAJECTORY_SUFFIXES) != suffixes
    }
    if missing:
        raise ValueError(f"Trajectory CSV has incomplete joint columns: {missing}")
    expected = trajectory_columns(joint_order)
    if fieldnames != expected:
        raise ValueError(f"Trajectory CSV columns must be ordered as {expected}.")
    return tuple(joint_order)


def read_trajectory_csv(path: str | Path, expected_joint_names: list[str] | tuple[str, ...] | None = None) -> TrajectoryData:
    trajectory_path = Path(path)
    with trajectory_path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise ValueError(f"{trajectory_path} is empty.")
        joint_names = infer_joint_names(reader.fieldnames)
        if expected_joint_names and tuple(expected_joint_names) != joint_names:
            raise ValueError(f"Trajectory joints {joint_names} do not match expected joints {tuple(expected_joint_names)}.")

        times: list[float] = []
        position: list[list[float]] = []
        velocity: list[list[float]] = []
        acceleration: list[list[float]] = []
        for row in reader:
            times.append(float(row[TIME_COLUMN]))
            position.append([float(row[trajectory_column(joint, POSITION_SUFFIX)]) for joint in joint_names])
            velocity.append([float(row[trajectory_column(joint, VELOCITY_SUFFIX)]) for joint in joint_names])
            acceleration.append([float(row[trajectory_column(joint, ACCELERATION_SUFFIX)]) for joint in joint_names])

    if not times:
        raise ValueError(f"{trajectory_path} contains no trajectory points.")
    time_array = np.asarray(times, dtype=float)
    if not np.isclose(time_array[0], 0.0, atol=1e-12):
        raise ValueError("Trajectory time_from_start must start at 0.0.")
    if np.any(np.diff(time_array) <= 0.0):
        raise ValueError("Trajectory time_from_start must be strictly increasing.")
    return TrajectoryData(
        joint_names=joint_names,
        time=time_array,
        position=np.asarray(position, dtype=float),
        velocity=np.asarray(velocity, dtype=float),
        acceleration=np.asarray(acceleration, dtype=float),
    )


def write_trajectory_csv(path: str | Path, data: TrajectoryData) -> Path:
    trajectory_path = Path(path)
    trajectory_path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = trajectory_columns(data.joint_names)
    with trajectory_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row_index, time_value in enumerate(data.time):
            row: dict[str, float] = {TIME_COLUMN: float(time_value)}
            for joint_index, joint_name in enumerate(data.joint_names):
                row[trajectory_column(joint_name, POSITION_SUFFIX)] = float(data.position[row_index, joint_index])
                row[trajectory_column(joint_name, VELOCITY_SUFFIX)] = float(data.velocity[row_index, joint_index])
                row[trajectory_column(joint_name, ACCELERATION_SUFFIX)] = float(data.acceleration[row_index, joint_index])
            writer.writerow(row)
    return trajectory_path
