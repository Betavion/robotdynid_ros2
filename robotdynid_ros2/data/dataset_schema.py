"""CSV schema definitions for robot dynamics identification data."""

from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path


TIMESTAMP_COLUMN = "timestamp"
POSITION_TEMPLATE = "joint{index}_position"
VELOCITY_TEMPLATE = "joint{index}_velocity"
ACCELERATION_TEMPLATE = "joint{index}_acceleration"
TORQUE_TEMPLATE = "joint{index}_measure"


@dataclass(frozen=True)
class DatasetSummary:
    path: Path
    row_count: int
    columns: tuple[str, ...]


def motion_columns(dof: int, include_acceleration: bool = False) -> list[str]:
    per_joint = (POSITION_TEMPLATE, VELOCITY_TEMPLATE, ACCELERATION_TEMPLATE) if include_acceleration else (
        POSITION_TEMPLATE,
        VELOCITY_TEMPLATE,
    )
    return [TIMESTAMP_COLUMN] + [
        name
        for index in range(1, dof + 1)
        for template in per_joint
        for name in (template.format(index=index),)
    ]


def torque_columns(dof: int) -> list[str]:
    return [TIMESTAMP_COLUMN] + [TORQUE_TEMPLATE.format(index=index) for index in range(1, dof + 1)]


def estimate_columns(joint_names: list[str]) -> list[str]:
    return [TIMESTAMP_COLUMN] + [f"{joint_name}_estimate" for joint_name in joint_names]


def one_file_columns(dof: int) -> list[str]:
    return (
        [TIMESTAMP_COLUMN]
        + [f"pos{index}" for index in range(1, dof + 1)]
        + [f"vel{index}" for index in range(1, dof + 1)]
        + [f"acc{index}" for index in range(1, dof + 1)]
        + [f"torque{index}" for index in range(1, dof + 1)]
    )


def summarize_csv(path: str | Path) -> DatasetSummary:
    csv_path = Path(path)
    with csv_path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.reader(handle)
        try:
            columns = tuple(next(reader))
        except StopIteration:
            return DatasetSummary(path=csv_path, row_count=0, columns=())
        row_count = sum(1 for _ in reader)
    return DatasetSummary(path=csv_path, row_count=row_count, columns=columns)


def validate_columns(path: str | Path, required_columns: list[str]) -> DatasetSummary:
    summary = summarize_csv(path)
    missing = [column for column in required_columns if column not in summary.columns]
    if missing:
        raise ValueError(f"{summary.path} is missing required columns: {missing}")
    return summary
