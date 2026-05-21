"""Trajectory-aware acceleration preprocessing for split datasets."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd
from scipy.signal import savgol_filter

from robotdynid_ros2.config import preprocessing_config, read_config
from robotdynid_ros2.data.csv_writer import SplitDatasetCsvWriter
from robotdynid_ros2.data.dataset_schema import ACCELERATION_TEMPLATE, POSITION_TEMPLATE, TIMESTAMP_COLUMN, TORQUE_TEMPLATE, VELOCITY_TEMPLATE
from robotdynid_ros2.data.manifest import read_manifest, resolve_manifest_data_paths
from robotdynid_ros2.trajectory.schema import read_trajectory_csv


def _template_names(template: str, dof: int) -> list[str]:
    return [template.format(index=index) for index in range(1, dof + 1)]


def _require_columns(dataframe: pd.DataFrame, columns: list[str], label: str) -> None:
    missing = [column for column in columns if column not in dataframe.columns]
    if missing:
        raise ValueError(f"{label} is missing required columns: {missing}")


def _aligned_motion_and_torque(motion_csv: str | Path, torque_csv: str | Path, dof: int) -> tuple[pd.DataFrame, pd.DataFrame]:
    motion = pd.read_csv(motion_csv)
    torque = pd.read_csv(torque_csv)
    _require_columns(
        motion,
        [TIMESTAMP_COLUMN] + _template_names(POSITION_TEMPLATE, dof) + _template_names(VELOCITY_TEMPLATE, dof),
        "Motion CSV",
    )
    _require_columns(torque, [TIMESTAMP_COLUMN] + _template_names(TORQUE_TEMPLATE, dof), "Torque CSV")
    motion_ts = motion[TIMESTAMP_COLUMN].to_numpy(dtype=float)
    torque_ts = torque[TIMESTAMP_COLUMN].to_numpy(dtype=float)
    if motion_ts.shape != torque_ts.shape or not np.allclose(motion_ts, torque_ts, atol=1e-9, rtol=0.0):
        raise ValueError("Motion and torque timestamps must align before preprocessing.")
    return motion, torque


def _odd_window(sample_period: float, window_sec: float, sample_count: int, poly_order: int) -> int:
    window = max(poly_order + 2, int(round(window_sec / sample_period)))
    if window % 2 == 0:
        window += 1
    if window > sample_count:
        window = sample_count if sample_count % 2 == 1 else sample_count - 1
    return max(window, poly_order + 2 + (poly_order + 2) % 2)


def _savgol_motion(
    timestamp: np.ndarray,
    position: np.ndarray,
    *,
    window_sec: float,
    poly_order: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, dict[str, Any]]:
    if len(timestamp) < poly_order + 3:
        qd = np.gradient(position, timestamp, axis=0, edge_order=1)
        qdd = np.gradient(qd, timestamp, axis=0, edge_order=1)
        return position.copy(), qd, qdd, {"method": "gradient", "reason": "too few samples for savgol"}
    sample_period = float(np.median(np.diff(timestamp)))
    window = _odd_window(sample_period, window_sec, len(timestamp), poly_order)
    q = savgol_filter(position, window_length=window, polyorder=poly_order, axis=0, mode="interp")
    qd = savgol_filter(position, window_length=window, polyorder=poly_order, deriv=1, delta=sample_period, axis=0, mode="interp")
    qdd = savgol_filter(position, window_length=window, polyorder=poly_order, deriv=2, delta=sample_period, axis=0, mode="interp")
    return q, qd, qdd, {"method": "savgol", "window_samples": int(window), "sample_period": sample_period}


def _interpolate_matrix(source_time: np.ndarray, values: np.ndarray, target_time: np.ndarray) -> np.ndarray:
    result = np.empty((len(target_time), values.shape[1]), dtype=float)
    for joint_index in range(values.shape[1]):
        result[:, joint_index] = np.interp(target_time, source_time, values[:, joint_index])
    return result


def _estimate_time_offset(
    measured_time: np.ndarray,
    measured_position: np.ndarray,
    command_time: np.ndarray,
    command_position: np.ndarray,
    *,
    max_offset_sec: float,
    grid_count: int,
) -> tuple[float, float]:
    rel_time = measured_time - measured_time[0]
    offsets = np.linspace(-max_offset_sec, max_offset_sec, max(3, grid_count))
    best_offset = 0.0
    best_rmse = float("inf")
    for offset in offsets:
        command_eval_time = rel_time - offset
        mask = (command_eval_time >= command_time[0]) & (command_eval_time <= command_time[-1])
        if np.count_nonzero(mask) < max(4, measured_position.shape[1] + 1):
            continue
        command = _interpolate_matrix(command_time, command_position, command_eval_time[mask])
        rmse = float(np.sqrt(np.mean((measured_position[mask] - command) ** 2)))
        if rmse < best_rmse:
            best_rmse = rmse
            best_offset = float(offset)
    return best_offset, best_rmse


def _fit_to_command(
    timestamp: np.ndarray,
    position: np.ndarray,
    velocity: np.ndarray,
    *,
    trajectory_csv: str | Path,
    config: dict[str, Any],
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, dict[str, Any]]:
    command = read_trajectory_csv(trajectory_csv)
    offset, command_rmse = _estimate_time_offset(
        timestamp,
        position,
        command.time,
        command.position,
        max_offset_sec=float(config["max_offset_sec"]),
        grid_count=int(config["offset_grid_count"]),
    )
    rel_time = timestamp - timestamp[0]
    command_eval_time = rel_time - offset
    mask = (command_eval_time >= command.time[0]) & (command_eval_time <= command.time[-1])
    discard_start = float(config["discard_start_sec"])
    discard_end = float(config["discard_end_sec"])
    if discard_start > 0.0:
        mask &= command_eval_time >= command.time[0] + discard_start
    if discard_end > 0.0:
        mask &= command_eval_time <= command.time[-1] - discard_end
    if np.count_nonzero(mask) < max(4, position.shape[1] + 1):
        raise ValueError("Too few samples remain after trajectory alignment and trimming.")

    cmd_q = _interpolate_matrix(command.time, command.position, command_eval_time[mask])
    cmd_qd = _interpolate_matrix(command.time, command.velocity, command_eval_time[mask])
    cmd_qdd = _interpolate_matrix(command.time, command.acceleration, command_eval_time[mask])

    fitted_q = np.empty_like(cmd_q)
    fitted_qd = np.empty_like(cmd_qd)
    fitted_qdd = np.empty_like(cmd_qdd)
    gains: list[float] = []
    offsets: list[float] = []
    position_rmse: list[float] = []
    velocity_rmse: list[float] = []
    for joint_index in range(position.shape[1]):
        basis = np.column_stack([np.ones(cmd_q.shape[0]), cmd_q[:, joint_index]])
        offset_value, gain_value = np.linalg.lstsq(basis, position[mask, joint_index], rcond=None)[0]
        fitted_q[:, joint_index] = offset_value + gain_value * cmd_q[:, joint_index]
        fitted_qd[:, joint_index] = gain_value * cmd_qd[:, joint_index]
        fitted_qdd[:, joint_index] = gain_value * cmd_qdd[:, joint_index]
        gains.append(float(gain_value))
        offsets.append(float(offset_value))
        position_rmse.append(float(np.sqrt(np.mean((fitted_q[:, joint_index] - position[mask, joint_index]) ** 2))))
        velocity_rmse.append(float(np.sqrt(np.mean((fitted_qd[:, joint_index] - velocity[mask, joint_index]) ** 2))))

    report = {
        "method": "generated_profile_affine_fit",
        "trajectory_csv": str(Path(trajectory_csv).expanduser().resolve()),
        "time_offset_sec": offset,
        "command_position_rmse_before_fit": command_rmse,
        "gain": gains,
        "offset": offsets,
        "position_rmse": position_rmse,
        "velocity_rmse": velocity_rmse,
        "sample_count": int(np.count_nonzero(mask)),
    }
    max_rmse = float(config["max_position_rmse"])
    if max_rmse > 0.0 and max(position_rmse) > max_rmse:
        raise ValueError(f"trajectory fit position RMSE {max(position_rmse):.6g} exceeds threshold {max_rmse:.6g}")
    return fitted_q, fitted_qd, fitted_qdd, mask, report


def _write_preprocessed_split(
    output_dir: str | Path,
    timestamp: np.ndarray,
    q: np.ndarray,
    qd: np.ndarray,
    qdd: np.ndarray,
    torque: np.ndarray,
) -> tuple[Path, Path]:
    with SplitDatasetCsvWriter(output_dir, dof=q.shape[1], include_acceleration=True) as writer:
        for index, time_value in enumerate(timestamp):
            writer.append(time_value, q[index], qd[index], torque[index], acceleration=qdd[index])
    return writer.motion_path, writer.torque_path


def preprocess_split_dataset(
    *,
    motion_csv: str | Path,
    torque_csv: str | Path,
    dof: int,
    output_dir: str | Path,
    config: dict[str, Any],
    trajectory_csv: str | Path | None = None,
) -> dict[str, Any]:
    motion, torque = _aligned_motion_and_torque(motion_csv, torque_csv, dof)
    timestamp = motion[TIMESTAMP_COLUMN].to_numpy(dtype=float)
    q = motion[_template_names(POSITION_TEMPLATE, dof)].to_numpy(dtype=float)
    qd = motion[_template_names(VELOCITY_TEMPLATE, dof)].to_numpy(dtype=float)
    tau = torque[_template_names(TORQUE_TEMPLATE, dof)].to_numpy(dtype=float)
    acceleration_columns = _template_names(ACCELERATION_TEMPLATE, dof)
    source = str(config["acceleration_source"])

    if source == "measured" and all(column in motion.columns for column in acceleration_columns):
        qdd = motion[acceleration_columns].to_numpy(dtype=float)
        mask = np.ones(len(timestamp), dtype=bool)
        method_report = {"method": "measured_columns"}
        actual_source = "measured"
    elif source == "fitted" and trajectory_csv:
        try:
            q, qd, qdd, mask, method_report = _fit_to_command(timestamp, q, qd, trajectory_csv=trajectory_csv, config=config)
            actual_source = "fitted"
        except Exception as exc:
            if not config["fallback_to_filter"]:
                raise
            q, qd, qdd, filter_report = _savgol_motion(
                timestamp,
                q,
                window_sec=float(config["window_sec"]),
                poly_order=int(config["poly_order"]),
            )
            mask = np.ones(len(timestamp), dtype=bool)
            method_report = {"method": "fallback_filtered", "fit_error": str(exc), "filter": filter_report}
            actual_source = "filtered"
    else:
        q, qd, qdd, filter_report = _savgol_motion(
            timestamp,
            q,
            window_sec=float(config["window_sec"]),
            poly_order=int(config["poly_order"]),
        )
        mask = np.ones(len(timestamp), dtype=bool)
        method_report = {"method": "filtered", "filter": filter_report}
        actual_source = "filtered"

    timestamp_out = timestamp[mask]
    tau_out = tau[mask]
    motion_path, torque_path = _write_preprocessed_split(output_dir, timestamp_out, q, qd, qdd, tau_out)
    report = {
        "schema_version": 1,
        "requested_acceleration_source": source,
        "actual_acceleration_source": actual_source,
        "method": method_report,
        "input": {
            "motion_csv": str(Path(motion_csv).expanduser().resolve()),
            "torque_csv": str(Path(torque_csv).expanduser().resolve()),
            "trajectory_csv": str(Path(trajectory_csv).expanduser().resolve()) if trajectory_csv else "",
            "sample_count": int(len(timestamp)),
        },
        "output": {
            "motion_csv": str(motion_path.resolve()),
            "torque_csv": str(torque_path.resolve()),
            "sample_count": int(len(timestamp_out)),
            "dropped_sample_count": int(len(timestamp) - len(timestamp_out)),
            "max_abs_acceleration": np.max(np.abs(qdd), axis=0).tolist(),
            "rms_acceleration": np.sqrt(np.mean(qdd**2, axis=0)).tolist(),
        },
    }
    Path(output_dir).mkdir(parents=True, exist_ok=True)
    report_path = Path(output_dir) / "preprocess_report.json"
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    report["report_path"] = str(report_path.resolve())
    return report


def _trajectory_from_manifest(manifest_path: str | Path) -> str:
    manifest = read_manifest(manifest_path)
    return str(manifest.get("collection", {}).get("commanded_trajectory_csv", ""))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="", help="Unified robotdynid_ros2 config file.")
    parser.add_argument("--manifest", default="", help="Run manifest with split dataset paths.")
    parser.add_argument("--motion-csv", default="")
    parser.add_argument("--torque-csv", default="")
    parser.add_argument("--dof", type=int, default=0)
    parser.add_argument("--trajectory", default="")
    parser.add_argument("--output-dir", required=True)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    config = preprocessing_config(read_config(args.config))
    trajectory = args.trajectory
    if args.manifest:
        motion_csv, torque_csv, _, dof = resolve_manifest_data_paths(args.manifest)
        trajectory = trajectory or config["trajectory_csv"] or _trajectory_from_manifest(args.manifest)
    else:
        if not args.motion_csv or not args.torque_csv or args.dof <= 0:
            raise ValueError("--motion-csv, --torque-csv and --dof are required when --manifest is not used.")
        motion_csv = Path(args.motion_csv)
        torque_csv = Path(args.torque_csv)
        dof = args.dof
        trajectory = trajectory or config["trajectory_csv"]
    report = preprocess_split_dataset(
        motion_csv=motion_csv,
        torque_csv=torque_csv,
        dof=dof,
        output_dir=args.output_dir,
        config=config,
        trajectory_csv=trajectory or None,
    )
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
