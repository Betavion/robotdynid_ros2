"""Generate simple smooth excitation trajectories."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path


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
    parser.add_argument("--joint-names", required=True, help="Comma-separated joint names.")
    parser.add_argument("--output", required=True)
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--sample-period", type=float, default=0.05)
    parser.add_argument("--amplitude", type=float, default=0.2)
    parser.add_argument("--frequency", type=float, default=0.1)
    parser.add_argument("--center", default="", help="Comma-separated center positions. Defaults to zeros.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    joint_names = [part.strip() for part in args.joint_names.split(",") if part.strip()]
    if not joint_names:
        raise ValueError("--joint-names must not be empty.")
    center = [0.0] * len(joint_names)
    if args.center:
        center = [float(part.strip()) for part in args.center.split(",") if part.strip()]
        if len(center) != len(joint_names):
            raise ValueError("--center length must match --joint-names.")
    rows = generate_sine_trajectory(
        joint_names=joint_names,
        duration=args.duration,
        sample_period=args.sample_period,
        amplitude=args.amplitude,
        frequency=args.frequency,
        center=center,
    )
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=["time_from_start"] + joint_names)
        writer.writeheader()
        writer.writerows(rows)
    print(output)


if __name__ == "__main__":
    main()
