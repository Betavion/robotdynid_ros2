"""Validate generated excitation trajectories before execution."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

import numpy as np

from robotdynid_ros2.config import read_config, trajectory_config
from robotdynid_ros2.trajectory.schema import TIME_COLUMN, TIME_UNIT, TRAJECTORY_UNITS, TrajectoryData, read_trajectory_csv
from robotdynid_ros2.trajectory.urdf_limits import (
    JointLimit,
    apply_position_bounds,
    finite_or_default,
    parse_urdf_joint_limits,
    resolve_vector,
)


def _trajectory_validation_config(config: dict[str, Any]) -> dict[str, Any]:
    values = trajectory_config(config)
    trajectory = config.get("trajectory", {}) if isinstance(config.get("trajectory", {}), dict) else {}
    validate = trajectory.get("validate", {}) if isinstance(trajectory.get("validate", {}), dict) else {}
    return {
        **values,
        "collision": bool(validate.get("collision", False)),
        "require_collision": bool(validate.get("require_collision", False)),
        "state_validity_service": str(validate.get("state_validity_service", "/check_state_validity")),
        "moveit_group": str(validate.get("moveit_group", "")),
        "collision_sample_limit": int(validate.get("collision_sample_limit", 200)),
        "collision_timeout_sec": float(validate.get("collision_timeout_sec", 2.0)),
        "torque": bool(validate.get("torque", False)),
        "max_torque_scale": float(validate.get("max_torque_scale", 1.0)),
        "torque_sample_limit": int(validate.get("torque_sample_limit", 200)),
        "position_lower": values["position_lower"],
        "position_upper": values["position_upper"],
    }


def _check_time(data: TrajectoryData) -> dict[str, Any]:
    dt = np.diff(data.time)
    return {
        "start_time": float(data.time[0]),
        "end_time": float(data.time[-1]),
        "sample_count": data.sample_count,
        "min_sample_period": float(np.min(dt)) if len(dt) else 0.0,
        "max_sample_period": float(np.max(dt)) if len(dt) else 0.0,
        "median_sample_period": float(np.median(dt)) if len(dt) else 0.0,
    }


def _check_limits(data: TrajectoryData, limits: list[JointLimit], config: dict[str, Any]) -> tuple[dict[str, Any], list[str]]:
    errors: list[str] = []
    acceleration_limits = resolve_vector(config["acceleration_limits"], data.dof)
    if acceleration_limits is None:
        acceleration_limits = [finite_or_default(limit.acceleration, finite_or_default(limit.velocity, 1.0) * 2.0) for limit in limits]
    velocity_scale = float(config["velocity_scale"])
    acceleration_scale = float(config["acceleration_scale"])
    joint_reports: dict[str, dict[str, float | None]] = {}
    for index, limit in enumerate(limits):
        position = data.position[:, index]
        velocity = data.velocity[:, index]
        acceleration = data.acceleration[:, index]
        max_abs_velocity = float(np.max(np.abs(velocity)))
        max_abs_acceleration = float(np.max(np.abs(acceleration)))
        velocity_limit = finite_or_default(limit.velocity, math.inf) * velocity_scale
        acceleration_limit = float(acceleration_limits[index]) * acceleration_scale
        report = {
            "min_position": float(np.min(position)),
            "max_position": float(np.max(position)),
            "max_abs_velocity": max_abs_velocity,
            "max_abs_acceleration": max_abs_acceleration,
            "lower": limit.lower,
            "upper": limit.upper,
            "velocity_limit": velocity_limit,
            "acceleration_limit": acceleration_limit,
        }
        if limit.lower is not None and float(np.min(position)) < limit.lower:
            errors.append(f"{limit.name} position goes below lower limit {limit.lower}.")
        if limit.upper is not None and float(np.max(position)) > limit.upper:
            errors.append(f"{limit.name} position goes above upper limit {limit.upper}.")
        if max_abs_velocity > velocity_limit + 1e-9:
            errors.append(f"{limit.name} velocity {max_abs_velocity:.6g} exceeds scaled limit {velocity_limit:.6g}.")
        if max_abs_acceleration > acceleration_limit + 1e-9:
            errors.append(f"{limit.name} acceleration {max_abs_acceleration:.6g} exceeds scaled limit {acceleration_limit:.6g}.")
        joint_reports[limit.name] = report
    return joint_reports, errors


def _check_torque(data: TrajectoryData, limits: list[JointLimit], config: dict[str, Any]) -> tuple[dict[str, Any], list[str]]:
    if not config["torque"]:
        return {"enabled": False}, []
    if not config["urdf_path"]:
        return {"enabled": True, "error": "robot.urdf_path is not configured"}, ["Torque validation requires robot.urdf_path."]
    try:
        import pinocchio as pin
    except ModuleNotFoundError as exc:  # pragma: no cover - environment dependent fallback
        return {"enabled": True, "error": str(exc)}, ["Torque validation requires pinocchio."]

    model = pin.buildModelFromUrdf(str(config["urdf_path"]))
    pin_data = model.createData()
    sample_count = min(int(config["torque_sample_limit"]), data.sample_count)
    indices = np.linspace(0, data.sample_count - 1, sample_count, dtype=int)
    tau_values = []
    for index in indices:
        tau = pin.rnea(model, pin_data, data.position[index], data.velocity[index], data.acceleration[index])
        tau_values.append(np.asarray(tau, dtype=float)[: data.dof])
    tau_matrix = np.vstack(tau_values)
    max_abs_tau = np.max(np.abs(tau_matrix), axis=0)
    errors: list[str] = []
    effort_limits = np.asarray([finite_or_default(limit.effort, math.inf) for limit in limits], dtype=float)
    scaled_effort = effort_limits * float(config["max_torque_scale"])
    for joint_index, limit in enumerate(limits):
        if max_abs_tau[joint_index] > scaled_effort[joint_index] + 1e-9:
            errors.append(
                f"{limit.name} model torque {max_abs_tau[joint_index]:.6g} exceeds scaled effort {scaled_effort[joint_index]:.6g}."
            )
    return {
        "enabled": True,
        "sample_count": int(sample_count),
        "max_abs_torque": max_abs_tau.tolist(),
        "scaled_effort_limit": scaled_effort.tolist(),
    }, errors


def _check_moveit_collision(data: TrajectoryData, config: dict[str, Any]) -> tuple[dict[str, Any], list[str], list[str]]:
    if not config["collision"]:
        return {"enabled": False}, [], []
    if not config["moveit_group"]:
        message = "MoveIt collision validation requires trajectory.validate.moveit_group."
        return {"enabled": True, "available": False, "error": message}, ([message] if config["require_collision"] else []), [message]

    try:
        import rclpy
        from moveit_msgs.msg import RobotState
        from moveit_msgs.srv import GetStateValidity
        from sensor_msgs.msg import JointState
    except ModuleNotFoundError as exc:  # pragma: no cover - ROS environment dependent
        message = f"MoveIt collision validation is unavailable: {exc}"
        return {"enabled": True, "available": False, "error": message}, ([message] if config["require_collision"] else []), [message]

    if not rclpy.ok():
        rclpy.init()
        shutdown_after = True
    else:
        shutdown_after = False
    node = rclpy.create_node("robotdynid_validate_excitation")
    try:
        client = node.create_client(GetStateValidity, config["state_validity_service"])
        if not client.wait_for_service(timeout_sec=float(config["collision_timeout_sec"])):
            message = f"MoveIt state-validity service {config['state_validity_service']} is not available."
            return {"enabled": True, "available": False, "error": message}, ([message] if config["require_collision"] else []), [message]

        sample_count = min(int(config["collision_sample_limit"]), data.sample_count)
        indices = np.linspace(0, data.sample_count - 1, sample_count, dtype=int)
        invalid: list[dict[str, Any]] = []
        for index in indices:
            request = GetStateValidity.Request()
            request.group_name = config["moveit_group"]
            request.robot_state = RobotState()
            request.robot_state.joint_state = JointState()
            request.robot_state.joint_state.name = list(data.joint_names)
            request.robot_state.joint_state.position = [float(value) for value in data.position[index]]
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=float(config["collision_timeout_sec"]))
            response = future.result()
            if response is None:
                invalid.append({"time_from_start": float(data.time[index]), "reason": "service timeout"})
            elif not response.valid:
                invalid.append({"time_from_start": float(data.time[index]), "reason": "collision_or_invalid_state"})
        errors = [f"MoveIt collision/state validation failed for {len(invalid)} sampled states."] if invalid else []
        return {
            "enabled": True,
            "available": True,
            "sample_count": int(sample_count),
            "invalid_count": len(invalid),
            "invalid_samples": invalid[:20],
        }, errors, []
    finally:
        node.destroy_node()
        if shutdown_after:
            rclpy.shutdown()


def validate_trajectory_data(data: TrajectoryData, config: dict[str, Any]) -> dict[str, Any]:
    values = _trajectory_validation_config(config)
    expected = tuple(values["joint_names"])
    errors: list[str] = []
    warnings: list[str] = []
    if expected and data.joint_names != expected:
        errors.append(f"Trajectory joint order {data.joint_names} does not match config joint order {expected}.")
    if not values["urdf_path"]:
        errors.append("robot.urdf_path is required for trajectory validation.")
        limits: list[JointLimit] = []
    else:
        urdf_limits = parse_urdf_joint_limits(values["urdf_path"], data.joint_names)
        position_lower = resolve_vector(values["position_lower"], data.dof)
        position_upper = resolve_vector(values["position_upper"], data.dof)
        limits = apply_position_bounds(urdf_limits, position_lower, position_upper)

    limit_report: dict[str, Any] = {}
    torque_report: dict[str, Any] = {"enabled": False}
    collision_report: dict[str, Any] = {"enabled": False}
    if limits:
        limit_report, limit_errors = _check_limits(data, limits, values)
        torque_report, torque_errors = _check_torque(data, limits, values)
        collision_report, collision_errors, collision_warnings = _check_moveit_collision(data, values)
        errors.extend(limit_errors)
        errors.extend(torque_errors)
        errors.extend(collision_errors)
        warnings.extend(collision_warnings)

    return {
        "schema_version": 1,
        "valid": not errors,
        "errors": errors,
        "warnings": warnings,
        "units": {TIME_COLUMN: TIME_UNIT, **TRAJECTORY_UNITS},
        "trajectory": {
            "joint_names": list(data.joint_names),
            "time": _check_time(data),
        },
        "limits": limit_report,
        "torque": torque_report,
        "collision": collision_report,
    }


def validate_trajectory_file(path: str | Path, config: dict[str, Any]) -> dict[str, Any]:
    values = _trajectory_validation_config(config)
    expected = tuple(values["joint_names"]) if values["joint_names"] else None
    data = read_trajectory_csv(path, expected_joint_names=expected)
    report = validate_trajectory_data(data, config)
    report["trajectory_csv"] = str(Path(path).expanduser().resolve())
    return report


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="", help="Unified robotdynid_ros2 config file.")
    parser.add_argument("--trajectory", required=True, help="Trajectory CSV to validate.")
    parser.add_argument("--output", default="", help="Validation report path. Defaults to <trajectory>_validation.json.")
    parser.add_argument("--require-collision", action="store_true", help="Fail when MoveIt collision service is unavailable.")
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Explicitly send the trajectory to the configured FollowJointTrajectory action after validation.",
    )
    parser.add_argument("--action-name", default="", help="FollowJointTrajectory action name for explicit --dry-run.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    config = read_config(args.config)
    if args.require_collision:
        config.setdefault("trajectory", {}).setdefault("validate", {})["require_collision"] = True
        config["trajectory"]["validate"]["collision"] = True
    report = validate_trajectory_file(args.trajectory, config=config)
    values = _trajectory_validation_config(config)
    if report["valid"] and args.dry_run:
        from robotdynid_ros2.trajectory.follow_joint_trajectory_client import FollowJointTrajectoryCsvClient, _load_csv_trajectory

        import rclpy

        action_name = args.action_name or values["action_name"]
        joint_names, points = _load_csv_trajectory(Path(args.trajectory))
        rclpy.init()
        node = FollowJointTrajectoryCsvClient(action_name)
        try:
            report["dry_run"] = {"enabled": True, "action_name": action_name, "accepted_and_successful": node.send(joint_names, points)}
        finally:
            node.destroy_node()
            rclpy.shutdown()
        if not report["dry_run"]["accepted_and_successful"]:
            report["valid"] = False
            report["errors"].append("Trajectory dry-run action failed.")
    else:
        report["dry_run"] = {"enabled": False}
    output = Path(args.output).expanduser() if args.output else Path(args.trajectory).with_name("excitation_validation.json")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(output)
    if not report["valid"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
