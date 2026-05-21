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
from robotdynid_ros2.trajectory.schema import TIME_COLUMN, TIME_UNIT, TRAJECTORY_UNITS, TrajectoryData, trajectory_column, write_trajectory_csv
from robotdynid_ros2.trajectory.urdf_limits import (
    JointLimit,
    apply_position_bounds,
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
    position_lower: tuple[float, ...] | None
    position_upper: tuple[float, ...] | None
    acceleration_limits: tuple[float, ...] | None
    score_regressor: bool
    score_sample_limit: int
    friction_speed_levels: int = 3
    gravity_pose_count: int = 5


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
    fill(end_mask, time[end_mask] - (duration - ramp), -1.0)
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


def _quintic_basis(u: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    s = 10.0 * u**3 - 15.0 * u**4 + 6.0 * u**5
    ds = 30.0 * u**2 - 60.0 * u**3 + 30.0 * u**4
    dds = 60.0 * u - 180.0 * u**2 + 120.0 * u**3
    return s, ds, dds


def _available_acceleration_limits(settings: ExcitationSettings, limits: list[JointLimit]) -> np.ndarray:
    if settings.acceleration_limits is not None:
        return np.asarray(settings.acceleration_limits, dtype=float) * settings.acceleration_scale
    acceleration = np.asarray(
        [finite_or_default(limit.acceleration, finite_or_default(limit.velocity, 1.0) * 2.0) for limit in limits],
        dtype=float,
    )
    return acceleration * max(settings.acceleration_scale, 1e-6)


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


def _multisine_phases(weights: np.ndarray, phase_strategy: str, rng: np.random.Generator) -> np.ndarray:
    if phase_strategy == "schroeder":
        harmonics = weights.shape[1]
        harmonic_numbers = np.arange(harmonics, dtype=float)
        phases = -math.pi * harmonic_numbers * (harmonic_numbers - 1.0) / max(float(harmonics), 1.0)
        phases = phases[None, :] + rng.uniform(0.0, 2.0 * math.pi, size=(weights.shape[0], 1))
        phases = np.where(weights < 0.0, phases + math.pi, phases)
        return np.mod(phases, 2.0 * math.pi)
    return rng.uniform(0.0, 2.0 * math.pi, size=weights.shape)


def _multisine_raw(
    *,
    time: np.ndarray,
    dof: int,
    harmonics: int,
    base_frequencies: np.ndarray,
    frequency_scale: float,
    harmonic_decay: float,
    frequency_mode: str,
    frequency_jitter: float,
    phase_strategy: str,
    rng: np.random.Generator,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, dict[str, Any]]:
    position = np.zeros((len(time), dof), dtype=float)
    velocity = np.zeros_like(position)
    acceleration = np.zeros_like(position)
    weights = rng.uniform(0.35, 1.0, size=(dof, harmonics))
    signs = rng.choice(np.asarray([-1.0, 1.0]), size=(dof, harmonics))
    harmonic_numbers = np.arange(1, harmonics + 1, dtype=float)
    weights = signs * weights / harmonic_numbers[None, :] ** harmonic_decay
    phases = _multisine_phases(weights, phase_strategy, rng)
    joint_base_frequencies = np.asarray(base_frequencies, dtype=float) * frequency_scale
    detune = np.ones((dof, harmonics), dtype=float)
    if frequency_jitter > 0.0:
        detune += rng.uniform(-frequency_jitter, frequency_jitter, size=(dof, harmonics))
    frequencies = joint_base_frequencies[:, None] * harmonic_numbers[None, :] * detune
    angular_frequencies = 2.0 * math.pi * frequencies
    for joint_index in range(dof):
        for harmonic_index in range(harmonics):
            omega = float(angular_frequencies[joint_index, harmonic_index])
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
        "frequency_mode": frequency_mode,
        "frequency_scale": float(frequency_scale),
        "harmonic_decay": float(harmonic_decay),
        "frequency_jitter": float(frequency_jitter),
        "phase_strategy": phase_strategy,
        "effective_base_frequency": joint_base_frequencies.tolist(),
        "frequencies": frequencies.tolist(),
        "angular_frequencies": angular_frequencies.tolist(),
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


def _speed_level_grid(count: int) -> np.ndarray:
    count = max(1, int(count))
    anchors_x = np.asarray([0.0, 0.5, 1.0], dtype=float)
    anchors_y = np.asarray([0.10, 0.35, 0.80], dtype=float)
    return np.interp(np.linspace(0.0, 1.0, count), anchors_x, anchors_y)


def _friction_velocity_sweep_raw(
    time: np.ndarray,
    dof: int,
    speed_level_count: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, dict[str, Any]]:
    duration = float(time[-1])
    speed_levels = _speed_level_grid(speed_level_count)
    level_values = [0.0]
    for speed in speed_levels:
        level_values.extend([float(speed), float(speed), -float(speed), -float(speed)])
    level_values.append(0.0)
    base_levels = np.asarray(level_values, dtype=float)
    knots = np.linspace(0.0, duration, len(base_levels))
    velocity = np.zeros((len(time), dof), dtype=float)
    acceleration = np.zeros_like(velocity)
    inner = base_levels[1:-1]

    for joint_index in range(dof):
        rolled = np.roll(inner, 2 * joint_index)
        sign = -1.0 if joint_index % 2 else 1.0
        levels = np.concatenate(([0.0], sign * rolled, [0.0]))
        for segment_index in range(len(levels) - 1):
            start = knots[segment_index]
            end = knots[segment_index + 1]
            mask = (time >= start) & (time <= end if segment_index == len(levels) - 2 else time < end)
            if not np.any(mask):
                continue
            interval = max(end - start, 1e-12)
            u = np.clip((time[mask] - start) / interval, 0.0, 1.0)
            s, ds, _ = _quintic_basis(u)
            delta = levels[segment_index + 1] - levels[segment_index]
            velocity[mask, joint_index] = levels[segment_index] + delta * s
            acceleration[mask, joint_index] = delta * ds / interval

    position = np.zeros_like(velocity)
    dt = np.diff(time)
    position[1:] = np.cumsum(0.5 * (velocity[1:] + velocity[:-1]) * dt[:, None], axis=0)
    if duration > 0.0:
        u = np.clip(time / duration, 0.0, 1.0)
        s, ds, dds = _quintic_basis(u)
        drift = position[-1].copy()
        position -= s[:, None] * drift[None, :]
        velocity -= (ds / duration)[:, None] * drift[None, :]
        acceleration -= (dds / duration**2)[:, None] * drift[None, :]
    return position, velocity, acceleration, {
        "speed_level_count": int(speed_level_count),
        "speed_levels": speed_levels.tolist(),
        "velocity_levels": base_levels.tolist(),
        "segment_count": max(0, len(base_levels) - 1),
    }


def _gravity_pose_sweep_raw(
    time: np.ndarray,
    dof: int,
    pose_count: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, dict[str, Any]]:
    duration = float(time[-1])
    joint_indices = np.arange(1, dof + 1, dtype=float)
    waypoints = [np.zeros(dof, dtype=float)]
    pose_count = max(1, int(pose_count))
    for waypoint_index in range(pose_count):
        raw = (
            np.sin(0.85 * (waypoint_index + 1) * joint_indices + 0.35 * waypoint_index)
            + 0.45 * np.cos(1.55 * (waypoint_index + 2) * joint_indices)
        )
        raw /= max(float(np.max(np.abs(raw))), 1e-12)
        waypoints.append(raw)
    waypoints.append(np.zeros(dof, dtype=float))

    position = np.zeros((len(time), dof), dtype=float)
    velocity = np.zeros_like(position)
    acceleration = np.zeros_like(position)
    segment_duration = duration / max(len(waypoints) - 1, 1)
    move_fraction = 0.72
    for segment_index in range(len(waypoints) - 1):
        start_time = segment_index * segment_duration
        end_time = duration if segment_index == len(waypoints) - 2 else (segment_index + 1) * segment_duration
        move_duration = max((end_time - start_time) * move_fraction, 1e-12)
        mask = (time >= start_time) & (time <= end_time if segment_index == len(waypoints) - 2 else time < end_time)
        if not np.any(mask):
            continue
        local = time[mask] - start_time
        moving = local <= move_duration
        delta = waypoints[segment_index + 1] - waypoints[segment_index]
        segment_position = np.repeat(waypoints[segment_index + 1][None, :], int(np.sum(mask)), axis=0)
        segment_velocity = np.zeros_like(segment_position)
        segment_acceleration = np.zeros_like(segment_position)
        if np.any(moving):
            u = np.clip(local[moving] / move_duration, 0.0, 1.0)
            s, ds, dds = _quintic_basis(u)
            segment_position[moving] = waypoints[segment_index][None, :] + s[:, None] * delta[None, :]
            segment_velocity[moving] = (ds / move_duration)[:, None] * delta[None, :]
            segment_acceleration[moving] = (dds / move_duration**2)[:, None] * delta[None, :]
        position[mask] = segment_position
        velocity[mask] = segment_velocity
        acceleration[mask] = segment_acceleration
    return position, velocity, acceleration, {
        "pose_count": int(pose_count),
        "segment_count": max(0, len(waypoints) - 1),
        "waypoints": [waypoint.tolist() for waypoint in waypoints],
    }


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
    scales, scale_report = _scale_raw_profile(
        raw_position,
        raw_velocity,
        raw_acceleration,
        center=center,
        limits=limits,
        settings=settings,
    )
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
    velocity_utilization = np.divide(
        max_velocity,
        velocity_limits,
        out=np.zeros_like(max_velocity),
        where=np.isfinite(velocity_limits) & (velocity_limits > 0.0),
    )
    acceleration_utilization = np.divide(
        max_acceleration,
        acceleration_limits,
        out=np.zeros_like(max_acceleration),
        where=np.isfinite(acceleration_limits) & (acceleration_limits > 0.0),
    )
    velocity_utilization = np.clip(velocity_utilization, 0.0, 1.0)
    acceleration_utilization = np.clip(acceleration_utilization, 0.0, 1.0)
    speed_utilization = float(0.75 * np.mean(velocity_utilization) + 0.25 * np.min(velocity_utilization))
    inertial_utilization = float(0.75 * np.mean(acceleration_utilization) + 0.25 * np.min(acceleration_utilization))
    aperiodicity = _aperiodicity_score(data.velocity)
    dynamic_utilization = float(0.60 * speed_utilization + 0.30 * inertial_utilization + 0.10 * aperiodicity)
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
        "velocity_utilization": velocity_utilization.tolist(),
        "acceleration_utilization": acceleration_utilization.tolist(),
        "speed_utilization_score": speed_utilization,
        "inertial_utilization_score": inertial_utilization,
        "aperiodicity_score": aperiodicity,
        "dynamic_utilization_score": dynamic_utilization,
        "min_position_margin": position_margin,
        "low_speed_ratio": low_speed_ratio,
    }


def _aperiodicity_score(samples: np.ndarray) -> float:
    if samples.shape[0] < 16:
        return 0.0
    stride = max(1, int(math.ceil(samples.shape[0] / 1600)))
    values = samples[::stride].astype(float, copy=False)
    values = values - np.mean(values, axis=0, keepdims=True)
    rms = np.sqrt(np.mean(values**2, axis=0))
    active = rms > 1e-9
    if not np.any(active):
        return 0.0
    values = values[:, active] / rms[active][None, :]
    sample_count = values.shape[0]
    min_lag = max(2, int(sample_count * 0.04))
    max_lag = max(min_lag, int(sample_count * 0.75))
    lags = np.unique(np.linspace(min_lag, max_lag, 80, dtype=int))
    peak_correlation = 0.0
    for lag in lags:
        if lag >= sample_count - 1:
            continue
        left = values[:-lag]
        right = values[lag:]
        numerator = np.sum(left * right, axis=0)
        denominator = np.sqrt(np.sum(left**2, axis=0) * np.sum(right**2, axis=0))
        correlation = np.divide(numerator, denominator, out=np.zeros_like(numerator), where=denominator > 1e-12)
        peak_correlation = max(peak_correlation, float(np.mean(np.abs(correlation))))
    return float(np.clip(1.0 - peak_correlation, 0.0, 1.0))


def _check_positions_inside_bounds(label: str, values: np.ndarray, limits: list[JointLimit]) -> None:
    for index, limit in enumerate(limits):
        value = float(values[index])
        if limit.lower is not None and value < limit.lower:
            raise ValueError(f"{label} for {limit.name} is below configured lower position bound {limit.lower}.")
        if limit.upper is not None and value > limit.upper:
            raise ValueError(f"{label} for {limit.name} is above configured upper position bound {limit.upper}.")


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


def _candidate_score(metrics: dict[str, Any]) -> tuple[int, float, float, float, float, float, float, float]:
    regressor = metrics.get("regressor", {})
    rank = int(regressor.get("rank", 0)) if regressor.get("enabled") else 0
    condition = float(regressor.get("condition_number", 1e12)) if regressor.get("enabled") else 1e12
    condition_score = -math.log10(max(condition, 1.0))
    dynamic_score = float(metrics["limits"].get("dynamic_utilization_score", 0.0))
    speed_score = float(metrics["limits"].get("speed_utilization_score", 0.0))
    inertial_score = float(metrics["limits"].get("inertial_utilization_score", 0.0))
    aperiodicity_score = float(metrics["limits"].get("aperiodicity_score", 0.0))
    min_margin_values = [value for value in metrics["limits"]["min_position_margin"] if value is not None]
    min_margin = min(min_margin_values) if min_margin_values else 0.0
    return (rank, dynamic_score, speed_score, aperiodicity_score, inertial_score, condition_score, min_margin, -float(condition))


def _adaptive_multisine_base_frequencies(
    settings: ExcitationSettings,
    limits: list[JointLimit],
    center: np.ndarray,
) -> np.ndarray:
    """Estimate per-joint base frequencies that balance position and acceleration limits."""

    acceleration_limits = _available_acceleration_limits(settings, limits)
    position_radii = np.asarray(
        [limit.position_radius(float(center[index]), settings.position_margin_ratio) for index, limit in enumerate(limits)],
        dtype=float,
    )
    configured = max(float(settings.base_frequency), 1e-6)
    optimal = np.divide(
        np.sqrt(np.maximum(acceleration_limits, 1e-12) / np.maximum(position_radii, 1e-12)),
        2.0 * math.pi,
    )
    lower = configured * 0.25
    upper = configured * 3.0
    return np.clip(optimal, lower, upper)


def _multisine_spectral_parameters(candidate_index: int) -> tuple[str, float, float, float, str]:
    """Return frequency and phase shaping parameters for a candidate."""

    if candidate_index == 0:
        return "common", 1.0, 1.0, 0.0, "random"
    spectral_specs = (
        ("detuned", 0.85, 2.8, 0.12, "schroeder"),
        ("adaptive", 0.85, 2.8, 0.0, "schroeder"),
        ("detuned", 0.70, 2.4, 0.16, "schroeder"),
        ("adaptive", 0.70, 2.4, 0.0, "schroeder"),
        ("detuned", 0.55, 1.7, 0.18, "random"),
        ("adaptive", 0.55, 1.7, 0.0, "random"),
        ("detuned", 0.45, 1.35, 0.20, "random"),
        ("adaptive", 0.45, 1.35, 0.0, "random"),
        ("common", 1.0, 2.8, 0.0, "schroeder"),
        ("detuned", 1.0, 2.8, 0.10, "schroeder"),
        ("common", 0.85, 2.4, 0.0, "schroeder"),
        ("detuned", 0.85, 2.4, 0.14, "schroeder"),
        ("common", 0.70, 2.0, 0.0, "random"),
        ("detuned", 0.70, 2.0, 0.16, "random"),
        ("common", 1.15, 2.8, 0.0, "schroeder"),
        ("detuned", 1.15, 2.8, 0.10, "schroeder"),
        ("common", 1.0, 1.0, 0.0, "random"),
        ("adaptive", 1.0, 2.4, 0.0, "schroeder"),
    )
    return spectral_specs[(candidate_index - 1) % len(spectral_specs)]


def _best_multisine_segment(
    settings: ExcitationSettings,
    limits: list[JointLimit],
    center: np.ndarray,
    duration: float,
    *,
    seed_offset: int = 0,
) -> tuple[TrajectoryData, dict[str, Any]]:
    time = _time_grid(duration, settings.sample_period)
    candidates: list[dict[str, Any]] = []
    best_data: TrajectoryData | None = None
    best_score: tuple[int, float, float, float, float, float, float, float] | None = None
    best_index = 0
    common_base_frequencies = np.full(len(settings.joint_names), float(settings.base_frequency), dtype=float)
    adaptive_base_frequencies = _adaptive_multisine_base_frequencies(settings, limits, center)

    for candidate_index in range(max(1, settings.search_candidates)):
        rng = np.random.default_rng(settings.random_seed + seed_offset + candidate_index)
        frequency_mode, frequency_scale, harmonic_decay, frequency_jitter, phase_strategy = _multisine_spectral_parameters(
            candidate_index
        )
        base_frequencies = adaptive_base_frequencies if frequency_mode in {"adaptive", "detuned"} else common_base_frequencies
        raw_q, raw_qd, raw_qdd, coefficients = _multisine_raw(
            time=time,
            dof=len(settings.joint_names),
            harmonics=settings.harmonics,
            base_frequencies=base_frequencies,
            frequency_scale=frequency_scale,
            harmonic_decay=harmonic_decay,
            frequency_mode=frequency_mode,
            frequency_jitter=frequency_jitter if frequency_mode == "detuned" else 0.0,
            phase_strategy=phase_strategy,
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
    return best_data, {"best_candidate_index": best_index, "seed_offset": seed_offset, "candidates": candidates}


def _friction_segment(settings: ExcitationSettings, limits: list[JointLimit], center: np.ndarray, duration: float) -> tuple[TrajectoryData, dict[str, Any]]:
    time = _time_grid(duration, settings.sample_period)
    raw_q, raw_qd, raw_qdd, profile = _friction_velocity_sweep_raw(
        time=time,
        dof=len(settings.joint_names),
        speed_level_count=settings.friction_speed_levels,
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
        amplitude_ratio=max(0.35, min(settings.low_speed_ratio * 2.2, 0.65)),
    )
    return data, {"profile": profile, "scale": scale_report, "limits": _limit_metrics(data, limits, settings)}


def _gravity_segment(settings: ExcitationSettings, limits: list[JointLimit], center: np.ndarray, duration: float) -> tuple[TrajectoryData, dict[str, Any]]:
    time = _time_grid(duration, settings.sample_period)
    raw_q, raw_qd, raw_qdd, profile = _gravity_pose_sweep_raw(
        time=time,
        dof=len(settings.joint_names),
        pose_count=settings.gravity_pose_count,
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
        amplitude_ratio=0.70,
    )
    return data, {"profile": profile, "scale": scale_report, "limits": _limit_metrics(data, limits, settings)}


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
    urdf_limits = parse_urdf_joint_limits(settings.urdf_path, settings.joint_names)
    limits = apply_position_bounds(urdf_limits, settings.position_lower, settings.position_upper)
    configured_center = list(settings.center) if settings.center is not None else None
    home_position = list(settings.home_position) if settings.home_position is not None else None
    center = centers_from_limits(limits, configured_center, home_position)
    home = np.asarray(home_position if home_position is not None else center, dtype=float)
    _check_positions_inside_bounds("center", center, limits)
    _check_positions_inside_bounds("home_position", home, limits)

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
        friction_duration = max(2.0, settings.duration * 0.20)
        gravity_duration = max(2.0, settings.duration * 0.15)
        reserved_duration = friction_duration + gravity_duration
        if reserved_duration > settings.duration * 0.5:
            scale = settings.duration * 0.5 / reserved_duration
            friction_duration *= scale
            gravity_duration *= scale
        multisine_duration = max(settings.duration - friction_duration - gravity_duration, settings.sample_period)
        first_multisine_duration = multisine_duration * 0.55
        second_multisine_duration = multisine_duration - first_multisine_duration
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
        multisine_a, multisine_a_report = _best_multisine_segment(
            settings,
            limits,
            center,
            first_multisine_duration,
            seed_offset=0,
        )
        gravity, gravity_report = _gravity_segment(settings, limits, center, gravity_duration)
        multisine_b, multisine_b_report = _best_multisine_segment(
            settings,
            limits,
            center,
            second_multisine_duration,
            seed_offset=1009,
        )
        segments.extend([friction, multisine_a, gravity, multisine_b])
        segment_reports.extend(
            [
                {"name": "friction_sweep", "duration": friction_duration, **friction_report},
                {"name": "safe_multisine", "duration": first_multisine_duration, **multisine_a_report},
                {"name": "gravity_sweep", "duration": gravity_duration, **gravity_report},
                {"name": "safe_multisine", "duration": second_multisine_duration, **multisine_b_report},
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
        "units": {TIME_COLUMN: TIME_UNIT, **TRAJECTORY_UNITS},
        "joint_names": list(settings.joint_names),
        "urdf_path": str(settings.urdf_path),
        "duration": float(data.time[-1]),
        "sample_period": settings.sample_period,
        "sample_count": data.sample_count,
        "center": center.tolist(),
        "home_position": home.tolist(),
        "position_bounds": {
            "lower": [limit.lower for limit in limits],
            "upper": [limit.upper for limit in limits],
            "urdf_lower": [limit.lower for limit in urdf_limits],
            "urdf_upper": [limit.upper for limit in urdf_limits],
        },
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
    position_lower = resolve_vector(args.position_lower or config_values["position_lower"], len(joint_names))
    position_upper = resolve_vector(args.position_upper or config_values["position_upper"], len(joint_names))
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
        friction_speed_levels=int(config_values["friction_speed_levels"]),
        gravity_pose_count=int(config_values["gravity_pose_count"]),
        random_seed=int(args.random_seed if args.random_seed is not None else config_values["random_seed"]),
        transition_duration=float(config_values["transition_duration"]),
        center=tuple(center) if center is not None else None,
        home_position=tuple(home_position) if home_position is not None else None,
        position_lower=tuple(position_lower) if position_lower is not None else None,
        position_upper=tuple(position_upper) if position_upper is not None else None,
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
    parser.add_argument("--position-lower", default="", help="Comma-separated lower excitation workspace positions.")
    parser.add_argument("--position-upper", default="", help="Comma-separated upper excitation workspace positions.")
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
