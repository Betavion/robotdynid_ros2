"""URDF joint-limit helpers used by excitation generation and validation."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np


@dataclass(frozen=True)
class JointLimit:
    name: str
    joint_type: str
    lower: float | None
    upper: float | None
    velocity: float | None
    effort: float | None

    @property
    def has_position_limits(self) -> bool:
        return self.lower is not None and self.upper is not None

    @property
    def center(self) -> float:
        if self.lower is None or self.upper is None:
            return 0.0
        return 0.5 * (self.lower + self.upper)

    def position_radius(self, center: float, margin_ratio: float) -> float:
        if self.lower is None or self.upper is None:
            return np.pi * max(0.0, 1.0 - margin_ratio)
        raw_radius = min(center - self.lower, self.upper - center)
        return max(0.0, raw_radius * max(0.0, 1.0 - margin_ratio))


def parse_urdf_joint_limits(urdf_path: str | Path, joint_names: list[str] | tuple[str, ...] | None = None) -> list[JointLimit]:
    """Parse movable-joint limits from a URDF file."""

    root = ET.parse(Path(urdf_path).expanduser()).getroot()
    requested = tuple(joint_names or ())
    requested_set = set(requested)
    found: dict[str, JointLimit] = {}

    for joint in root.findall("joint"):
        joint_type = str(joint.attrib.get("type", ""))
        name = str(joint.attrib.get("name", ""))
        if not name or joint_type == "fixed":
            continue
        if requested_set and name not in requested_set:
            continue
        limit = joint.find("limit")
        lower = upper = velocity = effort = None
        if limit is not None:
            lower = float(limit.attrib["lower"]) if "lower" in limit.attrib else None
            upper = float(limit.attrib["upper"]) if "upper" in limit.attrib else None
            velocity = float(limit.attrib["velocity"]) if "velocity" in limit.attrib else None
            effort = float(limit.attrib["effort"]) if "effort" in limit.attrib else None
        if joint_type == "continuous":
            lower = None
            upper = None
        found[name] = JointLimit(name=name, joint_type=joint_type, lower=lower, upper=upper, velocity=velocity, effort=effort)

    if requested:
        missing = [name for name in requested if name not in found]
        if missing:
            raise ValueError(f"URDF is missing movable joints: {missing}")
        return [found[name] for name in requested]
    return list(found.values())


def resolve_vector(raw: object, size: int, *, default: float | None = None) -> list[float] | None:
    if raw is None or raw == "":
        if default is None:
            return None
        return [float(default)] * size
    if isinstance(raw, str):
        stripped = raw.strip()
        if stripped.startswith("[") and stripped.endswith("]"):
            stripped = stripped[1:-1]
        values = [float(part.strip()) for part in stripped.split(",") if part.strip()]
    else:
        values = [float(value) for value in raw]  # type: ignore[arg-type]
    if not values and default is None:
        return None
    if len(values) != size:
        raise ValueError(f"Expected {size} values, got {len(values)}.")
    return values


def centers_from_limits(limits: list[JointLimit], configured_center: list[float] | None, home_position: list[float] | None) -> np.ndarray:
    if configured_center is not None:
        return np.asarray(configured_center, dtype=float)
    if home_position is not None:
        return np.asarray(home_position, dtype=float)
    return np.asarray([limit.center for limit in limits], dtype=float)


def finite_or_default(value: float | None, default: float) -> float:
    return default if value is None or not np.isfinite(value) or value <= 0.0 else float(value)
