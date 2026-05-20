"""Validate robot dynamics identification CSV files."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from robotdynid_ros2.data.dataset_schema import motion_columns, one_file_columns, torque_columns, validate_columns
from robotdynid_ros2.data.manifest import resolve_manifest_data_paths


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", default="")
    parser.add_argument("--dof", type=int, default=0)
    parser.add_argument("--csv", default="")
    parser.add_argument("--motion-csv", default="")
    parser.add_argument("--torque-csv", default="")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.manifest:
        motion_csv, torque_csv, _, dof = resolve_manifest_data_paths(args.manifest)
        summaries = [
            validate_columns(motion_csv, motion_columns(dof)),
            validate_columns(torque_csv, torque_columns(dof)),
        ]
    else:
        if args.dof <= 0:
            raise ValueError("--dof is required when --manifest is not used.")
        if args.csv:
            summaries = [validate_columns(Path(args.csv), one_file_columns(args.dof))]
        else:
            summaries = [
                validate_columns(Path(args.motion_csv), motion_columns(args.dof)),
                validate_columns(Path(args.torque_csv), torque_columns(args.dof)),
            ]
    print(
        json.dumps(
            {
                "ok": True,
                "files": [
                    {"path": str(summary.path), "row_count": summary.row_count, "columns": list(summary.columns)}
                    for summary in summaries
                ],
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
