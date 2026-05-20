"""Helpers for converting ROS joint state messages into ordered samples."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence


@dataclass(frozen=True)
class OrderedJointSample:
    timestamp: float
    position: tuple[float, ...]
    velocity: tuple[float, ...]
    effort: tuple[float, ...]


def stamp_to_seconds(stamp: object) -> float:
    sec = getattr(stamp, "sec", 0)
    nanosec = getattr(stamp, "nanosec", 0)
    return float(sec) + float(nanosec) * 1e-9


def ordered_joint_state_sample(
    msg: object,
    joint_names: Sequence[str],
    *,
    fallback_timestamp: float,
    allow_missing_effort: bool = False,
) -> OrderedJointSample:
    """Extract and reorder a JointState-like message by configured joint names."""

    message_names = list(getattr(msg, "name", []))
    index_by_name = {name: index for index, name in enumerate(message_names)}
    missing = [name for name in joint_names if name not in index_by_name]
    if missing:
        raise ValueError(f"JointState is missing configured joints: {missing}")

    position = list(getattr(msg, "position", []))
    velocity = list(getattr(msg, "velocity", []))
    effort = list(getattr(msg, "effort", []))
    if len(position) < len(message_names):
        raise ValueError("JointState.position must contain one value per named joint.")
    if len(velocity) < len(message_names):
        raise ValueError("JointState.velocity must contain one value per named joint.")
    if len(effort) < len(message_names):
        if not allow_missing_effort:
            raise ValueError("JointState.effort is required for torque identification.")
        effort = [0.0] * len(message_names)

    header = getattr(msg, "header", None)
    stamp = getattr(header, "stamp", None)
    timestamp = stamp_to_seconds(stamp) if stamp is not None else fallback_timestamp
    if timestamp == 0.0:
        timestamp = fallback_timestamp

    indices = [index_by_name[name] for name in joint_names]
    return OrderedJointSample(
        timestamp=timestamp,
        position=tuple(float(position[index]) for index in indices),
        velocity=tuple(float(velocity[index]) for index in indices),
        effort=tuple(float(effort[index]) for index in indices),
    )
