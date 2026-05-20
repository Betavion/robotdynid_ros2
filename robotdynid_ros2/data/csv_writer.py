"""CSV writers for split motion and torque identification datasets."""

from __future__ import annotations

import csv
from pathlib import Path
from typing import Iterable

from .dataset_schema import motion_columns, torque_columns


class SplitDatasetCsvWriter:
    """Write motion and torque samples using robotdynid's split CSV schema."""

    def __init__(self, output_dir: str | Path, dof: int) -> None:
        self.dof = dof
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.motion_path = self.output_dir / "motion.csv"
        self.torque_path = self.output_dir / "torque_measure_data.csv"
        self._motion_handle = self.motion_path.open("w", encoding="utf-8", newline="")
        self._torque_handle = self.torque_path.open("w", encoding="utf-8", newline="")
        self._motion_writer = csv.writer(self._motion_handle)
        self._torque_writer = csv.writer(self._torque_handle)
        self._motion_writer.writerow(motion_columns(dof))
        self._torque_writer.writerow(torque_columns(dof))
        self.sample_count = 0

    def append(self, timestamp: float, position: Iterable[float], velocity: Iterable[float], effort: Iterable[float]) -> None:
        position_values = tuple(position)
        velocity_values = tuple(velocity)
        effort_values = tuple(effort)
        if len(position_values) != self.dof or len(velocity_values) != self.dof or len(effort_values) != self.dof:
            raise ValueError(f"position, velocity and effort must all have length {self.dof}.")
        motion_row: list[float] = [timestamp]
        for pos_value, vel_value in zip(position_values, velocity_values, strict=True):
            motion_row.extend((float(pos_value), float(vel_value)))
        torque_row = [timestamp] + [float(value) for value in effort_values]
        self._motion_writer.writerow(motion_row)
        self._torque_writer.writerow(torque_row)
        self.sample_count += 1

    def close(self) -> None:
        self._motion_handle.close()
        self._torque_handle.close()

    def __enter__(self) -> "SplitDatasetCsvWriter":
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:  # noqa: ANN001
        self.close()
