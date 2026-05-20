"""Validate robot dynamics identification CSV files."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from robotdynid_ros2.config import read_config, validation_config
from robotdynid_ros2.data.dataset_schema import motion_columns, one_file_columns, torque_columns, validate_columns
from robotdynid_ros2.data.manifest import resolve_manifest_data_paths


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="", help="Unified robotdynid_ros2 config file.")
    parser.add_argument("--manifest", default="")
    parser.add_argument("--dof", type=int, default=0)
    parser.add_argument("--csv", default="")
    parser.add_argument("--motion-csv", default="")
    parser.add_argument("--torque-csv", default="")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    config = validation_config(read_config(args.config))
    manifest = args.manifest or config["manifest"]
    dof = args.dof or int(config["dof"])
    csv_path = args.csv or config["csv"]
    motion_csv_arg = args.motion_csv or config["motion_csv"]
    torque_csv_arg = args.torque_csv or config["torque_csv"]
    if manifest:
        motion_csv, torque_csv, _, dof = resolve_manifest_data_paths(manifest)
        summaries = [
            validate_columns(motion_csv, motion_columns(dof)),
            validate_columns(torque_csv, torque_columns(dof)),
        ]
    else:
        if dof <= 0:
            raise ValueError("--dof is required when --manifest is not used.")
        if csv_path:
            summaries = [validate_columns(Path(csv_path), one_file_columns(dof))]
        else:
            summaries = [
                validate_columns(Path(motion_csv_arg), motion_columns(dof)),
                validate_columns(Path(torque_csv_arg), torque_columns(dof)),
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
