"""Export generated runtime code into a controller package."""

from __future__ import annotations

import argparse
import csv
import shutil
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", required=True, help="Identification output directory.")
    parser.add_argument("--target-root", required=True, help="Controller package root.")
    parser.add_argument("--include-subdir", default="include/robotdynid_ros2/generated")
    parser.add_argument("--source-subdir", default="src/generated")
    parser.add_argument("--namespace", default="robotdynid::generated")
    parser.add_argument("--class-name", default="RegressorKernel")
    return parser.parse_args()


def _read_vector(path: Path) -> list[float]:
    values: list[float] = []
    with path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.reader(handle)
        for row in reader:
            if not row:
                continue
            try:
                values.append(float(row[-1]))
            except ValueError:
                continue
    return values


def _find_prediction_files(run_dir: Path) -> tuple[Path, Path]:
    candidates = [
        run_dir / "codegen" / "cpp",
        run_dir / "identify" / "codegen" / "cpp",
    ]
    for directory in candidates:
        source = directory / "predict_tau.cpp"
        header = directory / "predict_tau.hpp"
        if source.exists() and header.exists():
            return source, header
    raise FileNotFoundError(f"Could not find codegen/cpp/predict_tau.cpp/.hpp under {run_dir}")


def _params_header(namespace: str, theta: list[float], qds: list[float]) -> str:
    namespace_open = "\n".join(f"namespace {part} {{" for part in namespace.split("::") if part)
    namespace_close = "\n".join("}" for part in reversed([part for part in namespace.split("::") if part]))
    theta_values = ", ".join(f"{value:.17g}" for value in theta)
    qds_values = ", ".join(f"{value:.17g}" for value in qds)
    return f"""#pragma once

#include <array>

{namespace_open}

inline constexpr std::array<double, {len(theta)}> kThetaLin = {{{theta_values}}};
inline constexpr std::array<double, {len(qds)}> kQds = {{{qds_values}}};

{namespace_close}
"""


def main() -> None:
    args = parse_args()
    run_dir = Path(args.run_dir).expanduser()
    target_root = Path(args.target_root).expanduser()
    source_dir = target_root / args.source_subdir
    include_dir = target_root / args.include_subdir
    source_dir.mkdir(parents=True, exist_ok=True)
    include_dir.mkdir(parents=True, exist_ok=True)

    source, header = _find_prediction_files(run_dir)
    shutil.copy2(source, source_dir / source.name)
    shutil.copy2(header, include_dir / header.name)

    theta = _read_vector(run_dir / "theta_lin.csv")
    qds = _read_vector(run_dir / "qds_star.csv")
    (include_dir / "identified_params.hpp").write_text(_params_header(args.namespace, theta, qds), encoding="utf-8")

    print(source_dir / source.name)
    print(include_dir / header.name)
    print(include_dir / "identified_params.hpp")


if __name__ == "__main__":
    main()
