"""CSV schema definitions for robot dynamics identification data."""

from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path


TIMESTAMP_COLUMN = "timestamp"
POSITION_TEMPLATE = "joint{index}_position"
VELOCITY_TEMPLATE = "joint{index}_velocity"
TORQUE_TEMPLATE = "joint{index}_measure"


@dataclass(frozen=True)
class DatasetSummary:
    path: Path
    row_count: int
    columns: tuple[str, ...]


def motion_columns(dof: int) -> list[str]:
    return [TIMESTAMP_COLUMN] + [
        name
        for index in range(1, dof + 1)
        for name in (POSITION_TEMPLATE.format(index=index), VELOCITY_TEMPLATE.format(index=index))
    ]


def torque_columns(dof: int) -> list[str]:
    return [TIMESTAMP_COLUMN] + [TORQUE_TEMPLATE.format(index=index) for index in range(1, dof + 1)]


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
