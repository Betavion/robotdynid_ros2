"""Configuration loading and normalization for robotdynid ROS 2 tools."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any


CONFIG_DIR_KEY = "__config_dir"


def _parse_scalar(raw: str) -> Any:
    value = raw.strip()
    if value == "":
        return ""
    if value in {"[]", "{}"}:
        return [] if value == "[]" else {}
    if value.startswith("[") and value.endswith("]"):
        inner = value[1:-1].strip()
        if not inner:
            return []
        return [_parse_scalar(part) for part in inner.split(",")]
    lower = value.lower()
    if lower in {"true", "false"}:
        return lower == "true"
    if lower in {"null", "none"}:
        return None
    if (value.startswith('"') and value.endswith('"')) or (value.startswith("'") and value.endswith("'")):
        return value[1:-1]
    try:
        return int(value)
    except ValueError:
        try:
            return float(value)
        except ValueError:
            return value


def _yaml_lines(raw: str) -> list[tuple[int, str]]:
    lines: list[tuple[int, str]] = []
    for line in raw.splitlines():
        stripped = line.split("#", 1)[0].rstrip()
        if stripped.strip():
            lines.append((len(stripped) - len(stripped.lstrip(" ")), stripped.lstrip(" ")))
    return lines


def _parse_yaml_block(lines: list[tuple[int, str]], index: int, indent: int) -> tuple[Any, int]:
    if index >= len(lines):
        return {}, index
    is_list = lines[index][1].startswith("- ")
    if is_list:
        values: list[Any] = []
        while index < len(lines):
            line_indent, content = lines[index]
            if line_indent < indent or not content.startswith("- "):
                break
            item = content[2:].strip()
            if item:
                values.append(_parse_scalar(item))
                index += 1
            else:
                child, index = _parse_yaml_block(lines, index + 1, line_indent + 2)
                values.append(child)
        return values, index

    values: dict[str, Any] = {}
    while index < len(lines):
        line_indent, content = lines[index]
        if line_indent < indent or content.startswith("- "):
            break
        if ":" not in content:
            raise ValueError(f"Invalid config line: {content}")
        key, raw_value = content.split(":", 1)
        key = key.strip()
        raw_value = raw_value.strip()
        if raw_value:
            values[key] = _parse_scalar(raw_value)
            index += 1
        else:
            child, index = _parse_yaml_block(lines, index + 1, line_indent + 2)
            values[key] = child
    return values, index


def _parse_simple_yaml(raw: str) -> dict[str, Any]:
    parsed, index = _parse_yaml_block(_yaml_lines(raw), 0, 0)
    if index != len(_yaml_lines(raw)) or not isinstance(parsed, dict):
        raise ValueError("Config must contain a mapping.")
    return parsed


def read_config(path: str | Path | None) -> dict[str, Any]:
    """Read a JSON/YAML configuration file.

    JSON is accepted first because it has no optional dependency. Standard YAML
    requires PyYAML, which is available in normal ROS 2 environments.
    """

    if path is None or str(path) == "":
        return {}
    config_path = Path(path).expanduser()
    raw = config_path.read_text(encoding="utf-8")
    try:
        data = json.loads(raw)
    except json.JSONDecodeError:
        try:
            import yaml  # type: ignore[import-not-found]
        except ModuleNotFoundError as exc:
            try:
                data = _parse_simple_yaml(raw)
            except ValueError:
                raise ValueError(f"Config {config_path} is not JSON and PyYAML is not installed.") from exc
        else:
            data = yaml.safe_load(raw)
    if data is None:
        return {}
    if not isinstance(data, dict):
        raise ValueError(f"Config {config_path} must contain a mapping.")
    data[CONFIG_DIR_KEY] = str(config_path.parent.resolve())
    return data


def section(config: dict[str, Any], name: str) -> dict[str, Any]:
    value = config.get(name, {})
    if value is None:
        return {}
    if not isinstance(value, dict):
        raise ValueError(f"Config section '{name}' must be a mapping.")
    return value


def resolve_config_path(config: dict[str, Any], raw_path: Any) -> str:
    raw = str(raw_path or "")
    if not raw:
        return ""
    path = Path(raw).expanduser()
    if path.is_absolute():
        return str(path)
    config_dir = Path(str(config.get(CONFIG_DIR_KEY, "")))
    if config_dir:
        candidates = (config_dir / path, config_dir.parent / path)
        for candidate in candidates:
            if candidate.exists():
                return str(candidate.resolve())
    return raw


def as_bool(value: Any, default: bool = False) -> bool:
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "on"}
    return bool(value)


def as_int(value: Any, default: int = 0) -> int:
    if value is None or value == "":
        return default
    return int(value)


def as_float(value: Any, default: float = 0.0) -> float:
    if value is None or value == "":
        return default
    return float(value)


def as_list(value: Any) -> list[Any]:
    if value is None or value == "":
        return []
    if isinstance(value, str):
        stripped = value.strip()
        if not stripped:
            return []
        if stripped.startswith("[") and stripped.endswith("]"):
            stripped = stripped[1:-1]
        return [part.strip().strip("'\"") for part in stripped.split(",") if part.strip()]
    return list(value)


def as_csv(value: Any) -> str:
    values = as_list(value)
    return ",".join(str(item) for item in values) if values else str(value or "")


def recorder_params_from_config(config: dict[str, Any]) -> dict[str, Any]:
    robot = section(config, "robot")
    run = section(config, "run")
    recording = section(config, "recording")
    trajectory = section(config, "trajectory")
    return {
        "output_root": str(run.get("output_root", "runs")),
        "run_name": str(run.get("run_name", "")),
        "joint_names": [str(name) for name in as_list(robot.get("joint_names", []))],
        "joint_state_topic": str(recording.get("joint_state_topic", "/joint_states")),
        "estimate_joint_state_topic": str(recording.get("estimate_joint_state_topic", "")),
        "commanded_trajectory_csv": str(recording.get("commanded_trajectory_csv", trajectory.get("csv_path", ""))),
        "urdf_path": resolve_config_path(config, robot.get("urdf_path", "")),
        "duration_sec": as_float(recording.get("duration_sec"), 0.0),
        "min_sample_period_sec": as_float(recording.get("min_sample_period_sec"), 0.0),
        "flush_period_sec": as_float(recording.get("flush_period_sec"), 0.01),
        "queue_max_samples": as_int(recording.get("queue_max_samples"), 0),
        "auto_start": as_bool(recording.get("auto_start"), True),
        "shutdown_on_finish": as_bool(recording.get("shutdown_on_finish"), False),
        "allow_missing_effort": as_bool(recording.get("allow_missing_effort"), False),
    }


def identification_config(config: dict[str, Any]) -> dict[str, Any]:
    robot = section(config, "robot")
    identification = section(config, "identification")
    codegen = section(config, "codegen")
    return {
        "manifest": str(identification.get("manifest", "")),
        "urdf": resolve_config_path(config, robot.get("urdf_path", "")),
        "dof": as_int(robot.get("dof"), len(as_list(robot.get("joint_names", [])))),
        "csv": str(identification.get("csv", "")),
        "motion_csv": str(identification.get("motion_csv", "")),
        "torque_csv": str(identification.get("torque_csv", "")),
        "stride": as_int(identification.get("stride"), 100),
        "max_samples": as_int(identification.get("max_samples"), 3000),
        "selection_samples": as_int(identification.get("selection_samples"), 800),
        "selection_source": str(identification.get("selection_source", "model")),
        "selection_random_seed": as_int(identification.get("selection_random_seed"), 42),
        "selection_velocity_scale": as_float(identification.get("selection_velocity_scale"), 0.5),
        "selection_acceleration_scale": as_float(identification.get("selection_acceleration_scale"), 0.5),
        "stribeck_init": as_csv(identification.get("stribeck_init", "")),
        "max_iterations": as_int(identification.get("max_iterations"), 8),
        "chunk_size": as_int(identification.get("chunk_size"), 0),
        "output_dir": str(identification.get("output_dir", "")),
        "prediction_plot_stride": as_int(identification.get("prediction_plot_stride"), 50),
        "save_prediction_plot": as_bool(identification.get("save_prediction_plot"), True),
        "export_code": as_bool(codegen.get("export_code"), False),
        "codegen_languages": as_csv(codegen.get("languages", "c")),
        "codegen_output_subdir": str(codegen.get("output_subdir", "codegen")),
        "codegen_namespace": str(codegen.get("namespace", "robotdynid::generated")),
        "codegen_class_name": str(codegen.get("class_name", "RegressorKernel")),
    }


def trajectory_config(config: dict[str, Any]) -> dict[str, Any]:
    robot = section(config, "robot")
    run = section(config, "run")
    trajectory = section(config, "trajectory")
    generation = section(trajectory, "generation") if trajectory else {}
    validate = section(trajectory, "validate") if trajectory else {}
    return {
        "enabled": as_bool(trajectory.get("enabled"), False),
        "action_name": str(trajectory.get("action_name", "/joint_trajectory_controller/follow_joint_trajectory")),
        "csv_path": str(trajectory.get("csv_path", "")),
        "send_delay_sec": as_float(trajectory.get("send_delay_sec"), 1.0),
        "generate_on_start": as_bool(trajectory.get("generate_on_start"), False),
        "output_root": str(run.get("output_root", "runs")),
        "urdf_path": resolve_config_path(config, generation.get("urdf_path", robot.get("urdf_path", ""))),
        "joint_names": [str(name) for name in as_list(generation.get("joint_names", robot.get("joint_names", [])))],
        "output": str(generation.get("output", trajectory.get("csv_path", ""))),
        "report": str(generation.get("report", "")),
        "profile": str(generation.get("profile", "composite")),
        "duration": as_float(generation.get("duration"), 60.0),
        "sample_period": as_float(generation.get("sample_period"), 0.01),
        "harmonics": as_int(generation.get("harmonics"), 5),
        "base_frequency": as_float(generation.get("base_frequency"), 0.05),
        "position_margin_ratio": as_float(generation.get("position_margin_ratio"), 0.15),
        "velocity_scale": as_float(generation.get("velocity_scale"), 0.3),
        "acceleration_scale": as_float(generation.get("acceleration_scale"), 0.2),
        "low_speed_ratio": as_float(generation.get("low_speed_ratio"), 0.25),
        "search_candidates": as_int(generation.get("search_candidates"), 8),
        "random_seed": as_int(generation.get("random_seed"), 42),
        "transition_duration": as_float(generation.get("transition_duration"), 4.0),
        "center": generation.get("center", []),
        "home_position": generation.get("home_position", []),
        "acceleration_limits": generation.get("acceleration_limits", []),
        "score_regressor": as_bool(generation.get("score_regressor"), True),
        "score_sample_limit": as_int(generation.get("score_sample_limit"), 300),
        "validate_enabled": as_bool(validate.get("enabled"), False),
        "amplitude": as_float(generation.get("amplitude"), 0.2),
        "frequency": as_float(generation.get("frequency"), 0.1),
    }


def preprocessing_config(config: dict[str, Any]) -> dict[str, Any]:
    preprocessing = section(config, "preprocessing")
    alignment = section(preprocessing, "time_alignment") if preprocessing else {}
    fitting = section(preprocessing, "fitting") if preprocessing else {}
    filtering = section(preprocessing, "filtering") if preprocessing else {}
    return {
        "enabled": as_bool(preprocessing.get("enabled"), True),
        "acceleration_source": str(preprocessing.get("acceleration_source", "fitted")),
        "trajectory_csv": resolve_config_path(config, preprocessing.get("trajectory_csv", "")),
        "max_offset_sec": as_float(alignment.get("max_offset_sec"), 0.2),
        "offset_grid_count": as_int(alignment.get("offset_grid_count"), 81),
        "method": str(fitting.get("method", "generated_profile")),
        "discard_start_sec": as_float(fitting.get("discard_start_sec"), 0.0),
        "discard_end_sec": as_float(fitting.get("discard_end_sec"), 0.0),
        "max_position_rmse": as_float(fitting.get("max_position_rmse"), 0.05),
        "fallback_to_filter": as_bool(fitting.get("fallback_to_filter"), True),
        "filter_method": str(filtering.get("method", "savgol")),
        "window_sec": as_float(filtering.get("window_sec"), 0.15),
        "poly_order": as_int(filtering.get("poly_order"), 3),
    }


def export_runtime_config(config: dict[str, Any]) -> dict[str, Any]:
    export = section(config, "export_runtime")
    codegen = section(config, "codegen")
    return {
        "run_dir": str(export.get("run_dir", "")),
        "target_root": str(export.get("target_root", "")),
        "include_subdir": str(export.get("include_subdir", "include/robotdynid_ros2/generated")),
        "source_subdir": str(export.get("source_subdir", "src/generated")),
        "namespace": str(export.get("namespace", codegen.get("namespace", "robotdynid::generated"))),
        "class_name": str(export.get("class_name", codegen.get("class_name", "RegressorKernel"))),
    }


def bag_to_csv_config(config: dict[str, Any]) -> dict[str, Any]:
    robot = section(config, "robot")
    run = section(config, "run")
    recording = section(config, "recording")
    bag = section(config, "bag_to_csv")
    return {
        "bag": str(bag.get("bag", "")),
        "joint_state_topic": str(bag.get("joint_state_topic", recording.get("joint_state_topic", "/joint_states"))),
        "joint_names": as_csv(bag.get("joint_names", robot.get("joint_names", []))),
        "output_root": str(bag.get("output_root", run.get("output_root", "runs"))),
        "run_name": str(bag.get("run_name", run.get("run_name", ""))),
        "urdf": resolve_config_path(config, bag.get("urdf", robot.get("urdf_path", ""))),
        "storage_id": str(bag.get("storage_id", "sqlite3")),
        "allow_missing_effort": as_bool(bag.get("allow_missing_effort", recording.get("allow_missing_effort")), False),
    }


def validation_config(config: dict[str, Any]) -> dict[str, Any]:
    robot = section(config, "robot")
    identification = section(config, "identification")
    validation = section(config, "validation")
    return {
        "manifest": str(validation.get("manifest", identification.get("manifest", ""))),
        "dof": as_int(validation.get("dof", robot.get("dof")), len(as_list(robot.get("joint_names", [])))),
        "csv": str(validation.get("csv", identification.get("csv", ""))),
        "motion_csv": str(validation.get("motion_csv", identification.get("motion_csv", ""))),
        "torque_csv": str(validation.get("torque_csv", identification.get("torque_csv", ""))),
    }
