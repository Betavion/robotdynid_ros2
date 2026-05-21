"""Configuration model used by the GUI."""

from __future__ import annotations

import copy
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from robotdynid_ros2.config import (
    CONFIG_DIR_KEY,
    export_runtime_config,
    identification_config,
    preprocessing_config,
    read_config,
    recorder_params_from_config,
    resolve_config_path,
    section,
    trajectory_config,
)


def _drop_internal_keys(value: Any) -> Any:
    if isinstance(value, dict):
        return {key: _drop_internal_keys(item) for key, item in value.items() if key != CONFIG_DIR_KEY}
    if isinstance(value, list):
        return [_drop_internal_keys(item) for item in value]
    return value


def _write_mapping(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    try:
        import yaml  # type: ignore[import-not-found]
    except ModuleNotFoundError:
        path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    else:
        path.write_text(yaml.safe_dump(payload, sort_keys=False, allow_unicode=False), encoding="utf-8")


@dataclass(frozen=True)
class RobotSummary:
    urdf_path: str
    dof: int
    joint_names: tuple[str, ...]
    output_root: str


class GuiConfigModel:
    """Small wrapper around the existing YAML config helpers.

    The GUI intentionally keeps the existing config file as the source of
    truth. This class only provides safe accessors and canonical save behavior.
    """

    def __init__(self, path: str | Path | None = None) -> None:
        self.path: Path | None = None
        self.data: dict[str, Any] = {}
        if path:
            self.load(path)

    def load(self, path: str | Path) -> None:
        self.path = Path(path).expanduser()
        self.data = read_config(self.path)

    def save(self, path: str | Path | None = None) -> Path:
        target = Path(path).expanduser() if path else self.path
        if target is None:
            raise ValueError("A config path is required before saving.")
        payload = _drop_internal_keys(copy.deepcopy(self.data))
        _write_mapping(target, payload)
        self.load(target)
        return target

    def ensure_section(self, name: str) -> dict[str, Any]:
        value = self.data.setdefault(name, {})
        if not isinstance(value, dict):
            raise ValueError(f"Config section '{name}' must be a mapping.")
        return value

    def set_value(self, section_name: str, key: str, value: Any) -> None:
        self.ensure_section(section_name)[key] = value

    def set_nested_value(self, section_name: str, nested_name: str, key: str, value: Any) -> None:
        parent = self.ensure_section(section_name)
        nested = parent.setdefault(nested_name, {})
        if not isinstance(nested, dict):
            raise ValueError(f"Config section '{section_name}.{nested_name}' must be a mapping.")
        nested[key] = value

    def remove_nested_value(self, section_name: str, nested_name: str, key: str) -> None:
        parent = self.ensure_section(section_name)
        nested = parent.get(nested_name, {})
        if isinstance(nested, dict):
            nested.pop(key, None)

    def robot_summary(self) -> RobotSummary:
        robot = section(self.data, "robot")
        run = section(self.data, "run")
        joints = tuple(str(name) for name in robot.get("joint_names", []) or [])
        dof = int(robot.get("dof") or len(joints))
        return RobotSummary(
            urdf_path=resolve_config_path(self.data, robot.get("urdf_path", "")),
            dof=dof,
            joint_names=joints,
            output_root=str(run.get("output_root", "runs")),
        )

    def recorder_values(self) -> dict[str, Any]:
        return recorder_params_from_config(self.data)

    def trajectory_values(self) -> dict[str, Any]:
        return trajectory_config(self.data)

    def preprocessing_values(self) -> dict[str, Any]:
        return preprocessing_config(self.data)

    def identification_values(self) -> dict[str, Any]:
        return identification_config(self.data)

    def export_values(self) -> dict[str, Any]:
        return export_runtime_config(self.data)

    def config_path_text(self) -> str:
        return str(self.path) if self.path else ""

    def is_loaded(self) -> bool:
        return self.path is not None and bool(self.data)
