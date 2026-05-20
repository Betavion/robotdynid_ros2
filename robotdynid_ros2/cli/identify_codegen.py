"""Run offline robot dynamics identification and code generation."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from robotdynid_ros2.config import identification_config, read_config
from robotdynid_ros2.data.manifest import resolve_manifest_data_paths
from robotdynid_ros2.robotdynid_loader import ensure_robotdynid_available


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="", help="Unified robotdynid_ros2 config file.")
    parser.add_argument("--manifest", default=None, help="Run manifest written by dataset_recorder.")
    parser.add_argument("--urdf", default=None, help="URDF path. Required unless provided by --manifest.")
    parser.add_argument("--dof", type=int, default=None, help="Robot DOF. Required unless provided by --manifest.")
    parser.add_argument("--csv", default=None, help="One-file CSV dataset path.")
    parser.add_argument("--motion-csv", default=None, help="Motion CSV path for split datasets.")
    parser.add_argument("--torque-csv", default=None, help="Torque CSV path for split datasets.")
    parser.add_argument("--stride", type=int, default=None, help="Keep every Nth sample from the CSV.")
    parser.add_argument("--max-samples", type=int, default=None, help="Maximum sample count after stride.")
    parser.add_argument("--selection-samples", type=int, default=None, help="Sample count used for base-parameter selection.")
    parser.add_argument(
        "--selection-source",
        choices=("model", "data"),
        default=None,
        help="Use model random sampling or actual dataset states for base-parameter selection.",
    )
    parser.add_argument("--selection-random-seed", type=int, default=None)
    parser.add_argument("--selection-velocity-scale", type=float, default=None)
    parser.add_argument("--selection-acceleration-scale", type=float, default=None)
    parser.add_argument("--qds-init", default=None, help="Comma-separated Stribeck velocity initial guess.")
    parser.add_argument("--max-iterations", type=int, default=None)
    parser.add_argument("--chunk-size", type=int, default=None)
    parser.add_argument("--output-dir", default=None, help="Output directory. Defaults to manifest run_dir/identify or runs/<timestamp>.")
    parser.add_argument("--export-code", action="store_true", default=None)
    parser.add_argument("--codegen-languages", default=None)
    parser.add_argument("--codegen-output-subdir", default=None)
    parser.add_argument("--codegen-namespace", default=None)
    parser.add_argument("--codegen-class-name", default=None)
    parser.add_argument("--prediction-plot-stride", type=int, default=None)
    parser.add_argument("--no-plot", action="store_true")
    return parser.parse_args()


def _parse_qds_init(raw: str) -> np.ndarray | None:
    if not raw:
        return None
    return np.asarray([float(part.strip()) for part in raw.split(",") if part.strip()], dtype=float)


def _parse_codegen_languages(raw: str) -> tuple[str, ...]:
    return tuple(part.strip() for part in raw.split(",") if part.strip())


def _pick(value, default):  # noqa: ANN001
    return default if value is None else value


def _pick_text(value: str | None, default: str) -> str:
    return default if value is None else value


def _resolve_inputs(
    args: argparse.Namespace,
    config_values: dict[str, object],
) -> tuple[Path, int, Path | None, Path | None, Path | None, Path | None]:
    manifest_raw = _pick_text(args.manifest, str(config_values["manifest"]))
    manifest_path = Path(manifest_raw).expanduser() if manifest_raw else None
    if manifest_path is not None:
        motion_csv, torque_csv, manifest_urdf, dof = resolve_manifest_data_paths(manifest_path)
        urdf_raw = _pick_text(args.urdf, str(config_values["urdf"]))
        urdf_path = Path(urdf_raw).expanduser() if urdf_raw else manifest_urdf
        output_dir_raw = _pick_text(args.output_dir, str(config_values["output_dir"]))
        output_dir = Path(output_dir_raw).expanduser() if output_dir_raw else manifest_path.parent / "identify"
        return urdf_path, dof, None, motion_csv, torque_csv, output_dir

    urdf_raw = _pick_text(args.urdf, str(config_values["urdf"]))
    dof = int(_pick(args.dof, config_values["dof"]))
    if not urdf_raw:
        raise ValueError("--urdf is required when --manifest is not used.")
    if dof <= 0:
        raise ValueError("--dof must be provided when --manifest is not used.")
    output_dir_raw = _pick_text(args.output_dir, str(config_values["output_dir"]))
    csv_raw = _pick_text(args.csv, str(config_values["csv"]))
    motion_raw = _pick_text(args.motion_csv, str(config_values["motion_csv"]))
    torque_raw = _pick_text(args.torque_csv, str(config_values["torque_csv"]))
    output_dir = Path(output_dir_raw).expanduser() if output_dir_raw else None
    csv_path = Path(csv_raw).expanduser() if csv_raw else None
    motion_csv = Path(motion_raw).expanduser() if motion_raw else None
    torque_csv = Path(torque_raw).expanduser() if torque_raw else None
    return Path(urdf_raw).expanduser(), dof, csv_path, motion_csv, torque_csv, output_dir


def main() -> None:
    args = parse_args()
    config_values = identification_config(read_config(args.config))
    ensure_robotdynid_available()
    from robotdynid.workflow import IdentificationWorkflowConfig, run_identification_workflow

    urdf_path, dof, csv_path, motion_csv, torque_csv, output_dir = _resolve_inputs(args, config_values)
    if urdf_path is None:
        raise ValueError("A URDF path is required for identification.")

    export_code = bool(config_values["export_code"]) if args.export_code is None else args.export_code
    save_prediction_plot = bool(config_values["save_prediction_plot"]) and not args.no_plot
    payload = run_identification_workflow(
        IdentificationWorkflowConfig(
            urdf_path=urdf_path,
            dof=dof,
            csv_path=csv_path,
            motion_csv_path=motion_csv,
            torque_csv_path=torque_csv,
            stride=int(_pick(args.stride, config_values["stride"])),
            max_samples=int(_pick(args.max_samples, config_values["max_samples"])),
            selection_samples=int(_pick(args.selection_samples, config_values["selection_samples"])),
            selection_source=str(_pick(args.selection_source, config_values["selection_source"])),
            selection_random_seed=int(_pick(args.selection_random_seed, config_values["selection_random_seed"])),
            selection_velocity_scale=float(_pick(args.selection_velocity_scale, config_values["selection_velocity_scale"])),
            selection_acceleration_scale=float(_pick(args.selection_acceleration_scale, config_values["selection_acceleration_scale"])),
            qds_init=_parse_qds_init(str(_pick_text(args.qds_init, str(config_values["qds_init"])))),
            max_iterations=int(_pick(args.max_iterations, config_values["max_iterations"])),
            chunk_size=int(_pick(args.chunk_size, config_values["chunk_size"])) or None,
            output_dir=output_dir,
            export_code=export_code,
            codegen_languages=_parse_codegen_languages(_pick_text(args.codegen_languages, str(config_values["codegen_languages"]))),
            codegen_output_subdir=_pick_text(args.codegen_output_subdir, str(config_values["codegen_output_subdir"])),
            codegen_namespace=_pick_text(args.codegen_namespace, str(config_values["codegen_namespace"])),
            codegen_class_name=_pick_text(args.codegen_class_name, str(config_values["codegen_class_name"])),
            prediction_plot_stride=int(_pick(args.prediction_plot_stride, config_values["prediction_plot_stride"])),
            save_prediction_plot=save_prediction_plot,
        )
    )
    print(json.dumps(payload, indent=2))


if __name__ == "__main__":
    main()
