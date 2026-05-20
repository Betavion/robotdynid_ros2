"""Run offline robot dynamics identification and code generation."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from robotdynid_ros2.data.manifest import resolve_manifest_data_paths
from robotdynid_ros2.robotdynid_loader import ensure_robotdynid_available


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", default="", help="Run manifest written by dataset_recorder.")
    parser.add_argument("--urdf", default="", help="URDF path. Required unless provided by --manifest.")
    parser.add_argument("--dof", type=int, default=0, help="Robot DOF. Required unless provided by --manifest.")
    parser.add_argument("--csv", default="", help="One-file CSV dataset path.")
    parser.add_argument("--motion-csv", default="", help="Motion CSV path for split datasets.")
    parser.add_argument("--torque-csv", default="", help="Torque CSV path for split datasets.")
    parser.add_argument("--stride", type=int, default=100, help="Keep every Nth sample from the CSV.")
    parser.add_argument("--max-samples", type=int, default=3000, help="Maximum sample count after stride.")
    parser.add_argument("--selection-samples", type=int, default=800, help="Sample count used for base-parameter selection.")
    parser.add_argument(
        "--selection-source",
        choices=("model", "data"),
        default="model",
        help="Use model random sampling or actual dataset states for base-parameter selection.",
    )
    parser.add_argument("--selection-random-seed", type=int, default=42)
    parser.add_argument("--selection-velocity-scale", type=float, default=0.5)
    parser.add_argument("--selection-acceleration-scale", type=float, default=0.5)
    parser.add_argument("--qds-init", default="", help="Comma-separated Stribeck velocity initial guess.")
    parser.add_argument("--max-iterations", type=int, default=8)
    parser.add_argument("--chunk-size", type=int, default=0)
    parser.add_argument("--output-dir", default="", help="Output directory. Defaults to manifest run_dir/identify or runs/<timestamp>.")
    parser.add_argument("--export-code", action="store_true")
    parser.add_argument("--codegen-languages", default="c,cpp")
    parser.add_argument("--codegen-output-subdir", default="codegen")
    parser.add_argument("--codegen-namespace", default="robotdynid::generated")
    parser.add_argument("--codegen-class-name", default="RegressorKernel")
    parser.add_argument("--prediction-plot-stride", type=int, default=50)
    parser.add_argument("--no-plot", action="store_true")
    return parser.parse_args()


def _parse_qds_init(raw: str) -> np.ndarray | None:
    if not raw:
        return None
    return np.asarray([float(part.strip()) for part in raw.split(",") if part.strip()], dtype=float)


def _parse_codegen_languages(raw: str) -> tuple[str, ...]:
    return tuple(part.strip() for part in raw.split(",") if part.strip())


def _resolve_inputs(args: argparse.Namespace) -> tuple[Path, int, Path | None, Path | None, Path | None, Path | None]:
    manifest_path = Path(args.manifest).expanduser() if args.manifest else None
    if manifest_path is not None:
        motion_csv, torque_csv, manifest_urdf, dof = resolve_manifest_data_paths(manifest_path)
        urdf_path = Path(args.urdf).expanduser() if args.urdf else manifest_urdf
        output_dir = Path(args.output_dir).expanduser() if args.output_dir else manifest_path.parent / "identify"
        return urdf_path, dof, None, motion_csv, torque_csv, output_dir

    if not args.urdf:
        raise ValueError("--urdf is required when --manifest is not used.")
    if args.dof <= 0:
        raise ValueError("--dof must be provided when --manifest is not used.")
    output_dir = Path(args.output_dir).expanduser() if args.output_dir else None
    csv_path = Path(args.csv).expanduser() if args.csv else None
    motion_csv = Path(args.motion_csv).expanduser() if args.motion_csv else None
    torque_csv = Path(args.torque_csv).expanduser() if args.torque_csv else None
    return Path(args.urdf).expanduser(), args.dof, csv_path, motion_csv, torque_csv, output_dir


def main() -> None:
    args = parse_args()
    ensure_robotdynid_available()
    from robotdynid.workflow import IdentificationWorkflowConfig, run_identification_workflow

    urdf_path, dof, csv_path, motion_csv, torque_csv, output_dir = _resolve_inputs(args)
    if urdf_path is None:
        raise ValueError("A URDF path is required for identification.")

    payload = run_identification_workflow(
        IdentificationWorkflowConfig(
            urdf_path=urdf_path,
            dof=dof,
            csv_path=csv_path,
            motion_csv_path=motion_csv,
            torque_csv_path=torque_csv,
            stride=args.stride,
            max_samples=args.max_samples,
            selection_samples=args.selection_samples,
            selection_source=args.selection_source,
            selection_random_seed=args.selection_random_seed,
            selection_velocity_scale=args.selection_velocity_scale,
            selection_acceleration_scale=args.selection_acceleration_scale,
            qds_init=_parse_qds_init(args.qds_init),
            max_iterations=args.max_iterations,
            chunk_size=args.chunk_size or None,
            output_dir=output_dir,
            export_code=args.export_code,
            codegen_languages=_parse_codegen_languages(args.codegen_languages),
            codegen_output_subdir=args.codegen_output_subdir,
            codegen_namespace=args.codegen_namespace,
            codegen_class_name=args.codegen_class_name,
            prediction_plot_stride=args.prediction_plot_stride,
            save_prediction_plot=not args.no_plot,
        )
    )
    print(json.dumps(payload, indent=2))


if __name__ == "__main__":
    main()
