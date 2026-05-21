"""Generate dynamics-aware excitation trajectories."""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from robotdynid_ros2.config import read_config, trajectory_config
from robotdynid_ros2.paths import timestamped_run_dir
from robotdynid_ros2.robotdynid_loader import ensure_robotdynid_available
from robotdynid_ros2.trajectory.schema import TrajectoryData, trajectory_column, write_trajectory_csv
from robotdynid_ros2.trajectory.urdf_limits import (
    JointLimit,
    centers_from_limits,
    finite_or_default,
    parse_urdf_joint_limits,
    resolve_vector,
)


@dataclass(frozen=True)
class ExcitationSettings:
    joint_names: tuple[str, ...]
    urdf_path: Path | None
    profile: str
    duration: float
    sample_period: float
    harmonics: int
    base_frequency: float
    position_margin_ratio: float
    velocity_scale: float
    acceleration_scale: float
    low_speed_ratio: float
    search_candidates: int
    random_seed: int
    transition_duration: float
    center: tuple[float, ...] | None
    home_position: tuple[float, ...] | None
    acceleration_limits: tuple[float, ...] | None
    score_regressor: bool
    score_sample_limit: int


def _time_grid(duration: float, sample_period: float) -> np.ndarray:
    if duration <= 0.0:
        raise ValueError("duration must be positive.")
    if sample_period <= 0.0:
        raise ValueError("sample_period must be positive.")
    steps = max(1, int(math.ceil(duration / sample_period)))
    return np.linspace(0.0, duration, steps + 1)


def _smootherstep(time: np.ndarray, duration: float, ramp_ratio: float = 0.12) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    ramp = max(min(duration * ramp_ratio, duration * 0.5), 1e-9)
    envelope = np.ones_like(time)
    envelope_dot = np.zeros_like(time)
    envelope_ddot = np.zeros_like(time)

    def fill(mask: np.ndarray, local_time: np.ndarray, sign: float) -> None:
        u = np.clip(local_time / ramp, 0.0, 1.0)
        value = 6.0 * u**5 - 15.0 * u**4 + 10.0 * u**3
        first = (30.0 * u**4 - 60.0 * u**3 + 30.0 * u**2) / ramp
        second = (120.0 * u**3 - 180.0 * u**2 + 60.0 * u) / (ramp**2)
        if sign < 0.0:
            value = 1.0 - value
            first = -first
            second = -second
        envelope[mask] = value
        envelope_dot[mask] = first
        envelope_ddot[mask] = second

    start_mask = time < ramp
    fill(start_mask, time[start_mask], 1.0)
    end_mask = time > duration - ramp
    fill(end_mask, duration - time[end_mask], -1.0)
    return envelope, envelope_dot, envelope_ddot


def _quintic_segment(
    *,
    joint_names: tuple[str, ...],
    start: np.ndarray,
    end: np.ndarray,
    duration: float,
    sample_period: float,
) -> TrajectoryData:
    time = _time_grid(duration, sample_period)
    u = time / duration
    s = 10.0 * u**3 - 15.0 * u**4 + 6.0 * u**5
    sd = (30.0 * u**2 - 60.0 * u**3 + 30.0 * u**4) / duration
    sdd = (60.0 * u - 180.0 * u**2 + 120.0 * u**3) / (duration**2)
    delta = end - start
    return TrajectoryData(
        joint_names=joint_names,
        time=time,
        position=start[None, :] + s[:, None] * delta[None, :],
        velocity=sd[:, None] * delta[None, :],
        acceleration=sdd[:, None] * delta[None, :],
    )


def _available_acceleration_limits(settings: ExcitationSettings, limits: list[JointLimit]) -> np.ndarray:
    if settings.acceleration_limits is not None:
        return np.asarray(settings.acceleration_limits, dtype=float) * settings.acceleration_scale
    velocity = np.asarray([finite_or_default(limit.velocity, 1.0) for limit in limits], dtype=float)
    return velocity * max(settings.acceleration_scale, 1e-6) * 2.0


def _scale_raw_profile(
    raw_position: np.ndarray,
    raw_velocity: np.ndarray,
    raw_acceleration: np.ndarray,
    *,
    center: np.ndarray,
    limits: list[JointLimit],
    settings: ExcitationSettings,
) -> tuple[np.ndarray, dict[str, list[float]]]:
    velocity_limits = np.asarray([finite_or_default(limit.velocity, 1.0) for limit in limits], dtype=float) * settings.velocity_scale
    acceleration_limits = _available_acceleration_limits(settings, limits)
    position_radii = np.asarray(
        [limit.position_radius(float(center[index]), settings.position_margin_ratio) for index, limit in enumerate(limits)],
        dtype=float,
    )

    raw_position_max = np.maximum(np.max(np.abs(raw_position), axis=0), 1e-12)
    raw_velocity_max = np.maximum(np.max(np.abs(raw_velocity), axis=0), 1e-12)
    raw_acceleration_max = np.maximum(np.max(np.abs(raw_acceleration), axis=0), 1e-12)
    scales = np.minimum.reduce(
        [
            position_radii / raw_position_max,
            velocity_limits / raw_velocity_max,
            acceleration_limits / raw_acceleration_max,
        ]
    )
    scales = np.maximum(scales, 0.0)
    return scales, {
        "position_radius": position_radii.tolist(),
        "velocity_limit": velocity_limits.tolist(),
        "acceleration_limit": acceleration_limits.tolist(),
        "scale": scales.tolist(),
    }


def _multisine_raw(
    *,
    time: np.ndarray,
    dof: int,
    harmonics: int,
    base_frequency: float,
    rng: np.random.Generator,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, dict[str, Any]]:
    position = np.zeros((len(time), dof), dtype=float)
    velocity = np.zeros_like(position)
    acceleration = np.zeros_like(position)
    phases = rng.uniform(0.0, 2.0 * math.pi, size=(dof, harmonics))
    weights = rng.uniform(0.35, 1.0, size=(dof, harmonics))
    signs = rng.choice(np.asarray([-1.0, 1.0]), size=(dof, harmonics))
    weights = signs * weights / np.arange(1, harmonics + 1)[None, :]
    for joint_index in range(dof):
        for harmonic_index in range(harmonics):
            omega = 2.0 * math.pi * base_frequency * float(harmonic_index + 1)
            phase = phases[joint_index, harmonic_index]
            weight = weights[joint_index, harmonic_index]
            angle = omega * time + phase
            position[:, joint_index] += weight * np.sin(angle)
            velocity[:, joint_index] += weight * omega * np.cos(angle)
            acceleration[:, joint_index] -= weight * omega**2 * np.sin(angle)
    envelope, envelope_dot, envelope_ddot = _smootherstep(time, float(time[-1]))
    shaped_position = envelope[:, None] * position
    shaped_velocity = envelope_dot[:, None] * position + envelope[:, None] * velocity
    shaped_acceleration = (
        envelope_ddot[:, None] * position + 2.0 * envelope_dot[:, None] * velocity + envelope[:, None] * acceleration
    )
    return shaped_position, shaped_velocity, shaped_acceleration, {
        "phases": phases.tolist(),
        "weights": weights.tolist(),
    }


def _single_sine_raw(
    *,
    time: np.ndarray,
    dof: int,
    cycles: float,
    phases: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    duration = float(time[-1])
    omega = 2.0 * math.pi * cycles / duration
    angle = omega * time[:, None] + phases[None, :]
    position = np.sin(angle)
    velocity = omega * np.cos(angle)
    acceleration = -(omega**2) * np.sin(angle)
    envelope, envelope_dot, envelope_ddot = _smootherstep(time, duration)
    shaped_position = envelope[:, None] * position
    shaped_velocity = envelope_dot[:, None] * position + envelope[:, None] * velocity
    shaped_acceleration = (
        envelope_ddot[:, None] * position + 2.0 * envelope_dot[:, None] * velocity + envelope[:, None] * acceleration
    )
    return shaped_position, shaped_velocity, shaped_acceleration


def _make_scaled_segment(
    *,
    joint_names: tuple[str, ...],
    time: np.ndarray,
    raw_position: np.ndarray,
    raw_velocity: np.ndarray,
    raw_acceleration: np.ndarray,
    center: np.ndarray,
    limits: list[JointLimit],
    settings: ExcitationSettings,
    amplitude_ratio: float = 1.0,
) -> tuple[TrajectoryData, dict[str, Any]]:
    scales, scale_report = _scale_raw_profile(raw_position, raw_velocity, raw_acceleration, center=center, limits=limits, settings=settings)
    scales = scales * amplitude_ratio
    return (
        TrajectoryData(
            joint_names=joint_names,
            time=time,
            position=center[None, :] + raw_position * scales[None, :],
            velocity=raw_velocity * scales[None, :],
            acceleration=raw_acceleration * scales[None, :],
        ),
        scale_report,
    )


def _limit_metrics(data: TrajectoryData, limits: list[JointLimit], settings: ExcitationSettings) -> dict[str, Any]:
    max_position = np.max(np.abs(data.position), axis=0)
    max_velocity = np.max(np.abs(data.velocity), axis=0)
    max_acceleration = np.max(np.abs(data.acceleration), axis=0)
    velocity_limits = np.asarray([finite_or_default(limit.velocity, np.inf) for limit in limits], dtype=float) * settings.velocity_scale
    acceleration_limits = _available_acceleration_limits(settings, limits)
    position_margin: list[float | None] = []
    for joint_index, limit in enumerate(limits):
        if limit.lower is None or limit.upper is None:
            position_margin.append(None)
        else:
            lower_margin = float(np.min(data.position[:, joint_index] - limit.lower))
            upper_margin = float(np.min(limit.upper - data.position[:, joint_index]))
            position_margin.append(min(lower_margin, upper_margin))
    low_speed_threshold = np.maximum(velocity_limits * settings.low_speed_ratio, 1e-6)
    low_speed_ratio = float(np.mean(np.abs(data.velocity) <= low_speed_threshold[None, :]))
    return {
        "max_abs_position": max_position.tolist(),
        "max_abs_velocity": max_velocity.tolist(),
        "max_abs_acceleration": max_acceleration.tolist(),
        "velocity_limit": velocity_limits.tolist(),
        "acceleration_limit": acceleration_limits.tolist(),
        "min_position_margin": position_margin,
        "low_speed_ratio": low_speed_ratio,
    }


def _regressor_metrics(data: TrajectoryData, settings: ExcitationSettings) -> dict[str, Any]:
    if not settings.score_regressor or settings.urdf_path is None:
        return {"enabled": False}
    try:
        ensure_robotdynid_available()
        from robotdynid.identify import IdentificationDataset, stack_regression_problem
        from robotdynid.numeric import build_pinocchio_model, build_pinocchio_regressor_evaluator
    except Exception as exc:  # pragma: no cover - environment dependent fallback
        return {"enabled": True, "error": str(exc)}

    sample_count = min(settings.score_sample_limit, data.sample_count)
    indices = np.linspace(0, data.sample_count - 1, sample_count, dtype=int)
    dataset = IdentificationDataset(
        q=data.position[indices],
        qd=data.velocity[indices],
        qdd=data.acceleration[indices],
        tau=np.zeros((sample_count, data.dof), dtype=float),
    )
    pin_bundle = build_pinocchio_model(settings.urdf_path)
    evaluator = build_pinocchio_regressor_evaluator(pin_bundle, enabled_joint_dynamics_groups=tuple())
    matrix, _ = stack_regression_problem(dataset, evaluator)
    singular_values = np.linalg.svd(matrix, compute_uv=False)
    tolerance = np.finfo(float).eps * max(matrix.shape) * (singular_values[0] if singular_values.size else 0.0)
    rank = int(np.sum(singular_values > tolerance))
    smallest = float(singular_values[rank - 1]) if rank > 0 else 0.0
    condition = float(singular_values[0] / smallest) if smallest > 0.0 else float("inf")
    return {
        "enabled": True,
        "rank": rank,
        "column_count": int(matrix.shape[1]),
        "condition_number": condition,
        "min_singular_value": smallest,
    }


def _candidate_score(metrics: dict[str, Any]) -> tuple[int, float, float, float]:
    regressor = metrics.get("regressor", {})
    rank = int(regressor.get("rank", 0)) if regressor.get("enabled") else 0
    condition = float(regressor.get("condition_number", 1e12)) if regressor.get("enabled") else 1e12
    low_speed = float(metrics["limits"]["low_speed_ratio"])
    min_margin_values = [value for value in metrics["limits"]["min_position_margin"] if value is not None]
    min_margin = min(min_margin_values) if min_margin_values else 0.0
    return (rank, -math.log10(max(condition, 1.0)), low_speed, min_margin)


def _best_multisine_segment(
    settings: ExcitationSettings,
    limits: list[JointLimit],
    center: np.ndarray,
    duration: float,
) -> tuple[TrajectoryData, dict[str, Any]]:
    time = _time_grid(duration, settings.sample_period)
    candidates: list[dict[str, Any]] = []
    best_data: TrajectoryData | None = None
    best_score: tuple[int, float, float, float] | None = None
    best_index = 0

    for candidate_index in range(max(1, settings.search_candidates)):
        rng = np.random.default_rng(settings.random_seed + candidate_index)
        raw_q, raw_qd, raw_qdd, coefficients = _multisine_raw(
            time=time,
            dof=len(settings.joint_names),
            harmonics=settings.harmonics,
            base_frequency=settings.base_frequency,
            rng=rng,
        )
        data, scale_report = _make_scaled_segment(
            joint_names=settings.joint_names,
            time=time,
            raw_position=raw_q,
            raw_velocity=raw_qd,
            raw_acceleration=raw_qdd,
            center=center,
            limits=limits,
            settings=settings,
        )
        metrics = {
            "candidate_index": candidate_index,
            "scale": scale_report,
            "limits": _limit_metrics(data, limits, settings),
            "regressor": _regressor_metrics(data, settings),
            "coefficients": coefficients,
        }
        score = _candidate_score(metrics)
        metrics["score"] = list(score)
        candidates.append(metrics)
        if best_score is None or score > best_score:
            best_score = score
            best_data = data
            best_index = candidate_index

    assert best_data is not None
    return best_data, {"best_candidate_index": best_index, "candidates": candidates}


def _friction_segment(settings: ExcitationSettings, limits: list[JointLimit], center: np.ndarray, duration: float) -> tuple[TrajectoryData, dict[str, Any]]:
    time = _time_grid(duration, settings.sample_period)
    phases = np.linspace(0.0, math.pi, len(settings.joint_names), endpoint=False)
    raw_q, raw_qd, raw_qdd = _single_sine_raw(time=time, dof=len(settings.joint_names), cycles=3.0, phases=phases)
    data, scale_report = _make_scaled_segment(
        joint_names=settings.joint_names,
        time=time,
        raw_position=raw_q,
        raw_velocity=raw_qd,
        raw_acceleration=raw_qdd,
        center=center,
        limits=limits,
        settings=settings,
        amplitude_ratio=max(0.05, min(settings.low_speed_ratio, 0.35)),
    )
    return data, {"scale": scale_report, "limits": _limit_metrics(data, limits, settings)}


def _gravity_segment(settings: ExcitationSettings, limits: list[JointLimit], center: np.ndarray, duration: float) -> tuple[TrajectoryData, dict[str, Any]]:
    time = _time_grid(duration, settings.sample_period)
    phases = np.linspace(0.0, 2.0 * math.pi, len(settings.joint_names), endpoint=False)
    raw_q, raw_qd, raw_qdd = _single_sine_raw(time=time, dof=len(settings.joint_names), cycles=0.75, phases=phases)
    data, scale_report = _make_scaled_segment(
        joint_names=settings.joint_names,
        time=time,
        raw_position=raw_q,
        raw_velocity=raw_qd,
        raw_acceleration=raw_qdd,
        center=center,
        limits=limits,
        settings=settings,
        amplitude_ratio=0.55,
    )
    return data, {"scale": scale_report, "limits": _limit_metrics(data, limits, settings)}


def _concatenate_segments(segments: list[TrajectoryData]) -> TrajectoryData:
    if not segments:
        raise ValueError("At least one trajectory segment is required.")
    joint_names = segments[0].joint_names
    times: list[np.ndarray] = []
    positions: list[np.ndarray] = []
    velocities: list[np.ndarray] = []
    accelerations: list[np.ndarray] = []
    offset = 0.0
    for index, segment in enumerate(segments):
        if segment.joint_names != joint_names:
            raise ValueError("All trajectory segments must use the same joint order.")
        start = 1 if index > 0 else 0
        times.append(segment.time[start:] + offset)
        positions.append(segment.position[start:])
        velocities.append(segment.velocity[start:])
        accelerations.append(segment.acceleration[start:])
        offset += float(segment.time[-1])
    return TrajectoryData(
        joint_names=joint_names,
        time=np.concatenate(times),
        position=np.vstack(positions),
        velocity=np.vstack(velocities),
        acceleration=np.vstack(accelerations),
    )


def generate_excitation_trajectory(settings: ExcitationSettings) -> tuple[TrajectoryData, dict[str, Any]]:
    if settings.urdf_path is None:
        raise ValueError("trajectory generation requires robot.urdf_path.")
    limits = parse_urdf_joint_limits(settings.urdf_path, settings.joint_names)
    configured_center = list(settings.center) if settings.center is not None else None
    home_position = list(settings.home_position) if settings.home_position is not None else None
    center = centers_from_limits(limits, configured_center, home_position)
    home = np.asarray(home_position if home_position is not None else center, dtype=float)

    segments: list[TrajectoryData] = []
    segment_reports: list[dict[str, Any]] = []
    profile = settings.profile
    if profile == "safe_multisine":
        multisine, report = _best_multisine_segment(settings, limits, center, settings.duration)
        segments.append(multisine)
        segment_reports.append({"name": "safe_multisine", **report})
    elif profile == "friction_sweep":
        segment, report = _friction_segment(settings, limits, center, settings.duration)
        segments.append(segment)
        segment_reports.append({"name": "friction_sweep", **report})
    elif profile == "gravity_sweep":
        segment, report = _gravity_segment(settings, limits, center, settings.duration)
        segments.append(segment)
        segment_reports.append({"name": "gravity_sweep", **report})
    elif profile == "composite":
        friction_duration = max(2.0, settings.duration * 0.25)
        multisine_duration = max(2.0, settings.duration * 0.55)
        gravity_duration = max(2.0, settings.duration - friction_duration - multisine_duration)
        if np.linalg.norm(home - center, ord=np.inf) > 1e-9:
            segments.append(
                _quintic_segment(
                    joint_names=settings.joint_names,
                    start=home,
                    end=center,
                    duration=settings.transition_duration,
                    sample_period=settings.sample_period,
                )
            )
            segment_reports.append({"name": "move_to_start", "duration": settings.transition_duration})
        friction, friction_report = _friction_segment(settings, limits, center, friction_duration)
        multisine, multisine_report = _best_multisine_segment(settings, limits, center, multisine_duration)
        gravity, gravity_report = _gravity_segment(settings, limits, center, gravity_duration)
        segments.extend([friction, multisine, gravity])
        segment_reports.extend(
            [
                {"name": "friction_sweep", **friction_report},
                {"name": "safe_multisine", **multisine_report},
                {"name": "gravity_sweep", **gravity_report},
            ]
        )
        if np.linalg.norm(home - center, ord=np.inf) > 1e-9:
            segments.append(
                _quintic_segment(
                    joint_names=settings.joint_names,
                    start=center,
                    end=home,
                    duration=settings.transition_duration,
                    sample_period=settings.sample_period,
                )
            )
            segment_reports.append({"name": "return_home", "duration": settings.transition_duration})
    else:
        raise ValueError("trajectory profile must be safe_multisine, friction_sweep, gravity_sweep, or composite.")

    data = _concatenate_segments(segments)
    report = {
        "schema_version": 1,
        "profile": profile,
        "joint_names": list(settings.joint_names),
        "urdf_path": str(settings.urdf_path),
        "duration": float(data.time[-1]),
        "sample_period": settings.sample_period,
        "sample_count": data.sample_count,
        "center": center.tolist(),
        "home_position": home.tolist(),
        "limits": _limit_metrics(data, limits, settings),
        "segments": segment_reports,
    }
    return data, report


def generate_sine_trajectory(
    *,
    joint_names: list[str],
    duration: float,
    sample_period: float,
    amplitude: float,
    frequency: float,
    center: list[float],
) -> list[dict[str, float]]:
    """Compatibility helper that emits the new trajectory CSV schema."""

    time = _time_grid(duration, sample_period)
    rows: list[dict[str, float]] = []
    for time_value in time:
        row: dict[str, float] = {"time_from_start": float(time_value)}
        for index, joint in enumerate(joint_names):
            phase = index * math.pi / max(len(joint_names), 1)
            angle = 2.0 * math.pi * frequency * time_value + phase
            row[trajectory_column(joint, "position")] = center[index] + amplitude * math.sin(angle)
            row[trajectory_column(joint, "velocity")] = amplitude * 2.0 * math.pi * frequency * math.cos(angle)
            row[trajectory_column(joint, "acceleration")] = -amplitude * (2.0 * math.pi * frequency) ** 2 * math.sin(angle)
        rows.append(row)
    return rows


def _settings_from_config(config_values: dict[str, Any], args: argparse.Namespace) -> ExcitationSettings:
    joint_names = tuple(args.joint_names.split(",")) if args.joint_names else tuple(config_values["joint_names"])
    joint_names = tuple(name.strip() for name in joint_names if name.strip())
    if not joint_names:
        raise ValueError("robot.joint_names or --joint-names must be configured.")
    urdf_raw = str(args.urdf or config_values["urdf_path"])
    urdf_path = Path(urdf_raw).expanduser() if urdf_raw else None
    center = resolve_vector(args.center or config_values["center"], len(joint_names))
    home_position = resolve_vector(args.home_position or config_values["home_position"], len(joint_names))
    acceleration_limits = resolve_vector(config_values["acceleration_limits"], len(joint_names))
    return ExcitationSettings(
        joint_names=joint_names,
        urdf_path=urdf_path,
        profile=str(args.profile or config_values["profile"]),
        duration=float(args.duration if args.duration is not None else config_values["duration"]),
        sample_period=float(args.sample_period if args.sample_period is not None else config_values["sample_period"]),
        harmonics=int(args.harmonics if args.harmonics is not None else config_values["harmonics"]),
        base_frequency=float(config_values["base_frequency"]),
        position_margin_ratio=float(config_values["position_margin_ratio"]),
        velocity_scale=float(config_values["velocity_scale"]),
        acceleration_scale=float(config_values["acceleration_scale"]),
        low_speed_ratio=float(config_values["low_speed_ratio"]),
        search_candidates=int(args.search_candidates if args.search_candidates is not None else config_values["search_candidates"]),
        random_seed=int(args.random_seed if args.random_seed is not None else config_values["random_seed"]),
        transition_duration=float(config_values["transition_duration"]),
        center=tuple(center) if center is not None else None,
        home_position=tuple(home_position) if home_position is not None else None,
        acceleration_limits=tuple(acceleration_limits) if acceleration_limits is not None else None,
        score_regressor=bool(config_values["score_regressor"]),
        score_sample_limit=int(config_values["score_sample_limit"]),
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="", help="Unified robotdynid_ros2 config file.")
    parser.add_argument("--joint-names", default="", help="Comma-separated joint names.")
    parser.add_argument("--urdf", default="", help="URDF path. Defaults to robot.urdf_path from config.")
    parser.add_argument("--output", default="")
    parser.add_argument("--report", default="")
    parser.add_argument("--profile", choices=("safe_multisine", "friction_sweep", "gravity_sweep", "composite"), default=None)
    parser.add_argument("--duration", type=float, default=None)
    parser.add_argument("--sample-period", type=float, default=None)
    parser.add_argument("--harmonics", type=int, default=None)
    parser.add_argument("--search-candidates", type=int, default=None)
    parser.add_argument("--random-seed", type=int, default=None)
    parser.add_argument("--center", default="", help="Comma-separated center positions.")
    parser.add_argument("--home-position", default="", help="Comma-separated home positions.")
    parser.add_argument("--validate", action="store_true", help="Run offline trajectory validation after generation.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    config_values = trajectory_config(read_config(args.config))
    settings = _settings_from_config(config_values, args)
    data, report = generate_excitation_trajectory(settings)

    output_raw = args.output or config_values["output"]
    if not output_raw:
        output = timestamped_run_dir(config_values["output_root"]) / "excitation.csv"
    else:
        output = Path(output_raw).expanduser()
    report_raw = args.report or config_values["report"]
    report_path = Path(report_raw).expanduser() if report_raw else output.with_name("excitation_report.json")

    write_trajectory_csv(output, data)
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report["trajectory_csv"] = str(output.resolve())
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")

    if args.validate or config_values["validate_enabled"]:
        from robotdynid_ros2.trajectory.validate import validate_trajectory_file

        validation = validate_trajectory_file(output, config=read_config(args.config))
        validation_path = output.with_name("excitation_validation.json")
        validation_path.write_text(json.dumps(validation, indent=2) + "\n", encoding="utf-8")
        if not validation["valid"]:
            raise RuntimeError(f"Generated trajectory failed validation. See {validation_path}")

    print(output)


if __name__ == "__main__":
    main()
