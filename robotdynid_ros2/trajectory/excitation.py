"""Generate simple smooth excitation trajectories."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

from robotdynid_ros2.config import read_config, trajectory_config


def generate_sine_trajectory(
    *,
    joint_names: list[str],
    duration: float,
    sample_period: float,
    amplitude: float,
    frequency: float,
    center: list[float],
) -> list[dict[str, float]]:
    steps = int(duration / sample_period) + 1
    rows: list[dict[str, float]] = []
    for step in range(steps):
        t = min(step * sample_period, duration)
        row: dict[str, float] = {"time_from_start": t}
        for index, joint in enumerate(joint_names):
            phase = index * math.pi / max(len(joint_names), 1)
            row[joint] = center[index] + amplitude * math.sin(2.0 * math.pi * frequency * t + phase)
        rows.append(row)
    return rows


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="", help="Unified robotdynid_ros2 config file.")
    parser.add_argument("--joint-names", default="", help="Comma-separated joint names.")
    parser.add_argument("--output", default="")
    parser.add_argument("--duration", type=float, default=None)
    parser.add_argument("--sample-period", type=float, default=None)
    parser.add_argument("--amplitude", type=float, default=None)
    parser.add_argument("--frequency", type=float, default=None)
    parser.add_argument("--center", default="", help="Comma-separated center positions. Defaults to zeros.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    config = trajectory_config(read_config(args.config))
    joint_names_raw = args.joint_names or config["joint_names"]
    joint_names = [part.strip() for part in joint_names_raw.split(",") if part.strip()]
    if not joint_names:
        raise ValueError("--joint-names must not be empty.")
    center = [0.0] * len(joint_names)
    center_raw = args.center or config["center"]
    if center_raw:
        center = [float(part.strip()) for part in center_raw.split(",") if part.strip()]
        if len(center) != len(joint_names):
            raise ValueError("--center length must match --joint-names.")
    output_raw = args.output or config["output"]
    if not output_raw:
        raise ValueError("--output is required unless trajectory.generation.output is configured.")
    rows = generate_sine_trajectory(
        joint_names=joint_names,
        duration=args.duration if args.duration is not None else float(config["duration"]),
        sample_period=args.sample_period if args.sample_period is not None else float(config["sample_period"]),
        amplitude=args.amplitude if args.amplitude is not None else float(config["amplitude"]),
        frequency=args.frequency if args.frequency is not None else float(config["frequency"]),
        center=center,
    )
    output = Path(output_raw)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=["time_from_start"] + joint_names)
        writer.writeheader()
        writer.writerows(rows)
    print(output)


if __name__ == "__main__":
    main()
