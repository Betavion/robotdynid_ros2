"""Export generated runtime code into a controller package."""

from __future__ import annotations

import argparse
import csv
import shutil
from pathlib import Path

from robotdynid_ros2.config import export_runtime_config, read_config


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="", help="Unified robotdynid_ros2 config file.")
    parser.add_argument("--run-dir", default="", help="Identification output directory.")
    parser.add_argument("--target-root", default="", help="Controller package root.")
    parser.add_argument("--include-subdir", default="")
    parser.add_argument("--source-subdir", default="")
    parser.add_argument("--namespace", default="")
    parser.add_argument("--class-name", default="")
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


def _params_header(namespace: str, linear_parameters: list[float], stribeck_parameters: list[float]) -> str:
    namespace_open = "\n".join(f"namespace {part} {{" for part in namespace.split("::") if part)
    namespace_close = "\n".join("}" for part in reversed([part for part in namespace.split("::") if part]))
    linear_values = ", ".join(f"{value:.17g}" for value in linear_parameters)
    stribeck_values = ", ".join(f"{value:.17g}" for value in stribeck_parameters)
    return f"""#pragma once

#include <array>

{namespace_open}

inline constexpr std::array<double, {len(linear_parameters)}> kLinearParameters = {{{linear_values}}};
inline constexpr std::array<double, {len(stribeck_parameters)}> kStribeckParameters = {{{stribeck_values}}};

{namespace_close}
"""


def main() -> None:
    args = parse_args()
    config = export_runtime_config(read_config(args.config))
    run_dir_raw = args.run_dir or config["run_dir"]
    target_root_raw = args.target_root or config["target_root"]
    if not run_dir_raw:
        raise ValueError("--run-dir is required unless export_runtime.run_dir is configured.")
    if not target_root_raw:
        raise ValueError("--target-root is required unless export_runtime.target_root is configured.")
    run_dir = Path(run_dir_raw).expanduser()
    target_root = Path(target_root_raw).expanduser()
    source_dir = target_root / (args.source_subdir or config["source_subdir"])
    include_dir = target_root / (args.include_subdir or config["include_subdir"])
    source_dir.mkdir(parents=True, exist_ok=True)
    include_dir.mkdir(parents=True, exist_ok=True)

    source, header = _find_prediction_files(run_dir)
    shutil.copy2(source, source_dir / source.name)
    shutil.copy2(header, include_dir / header.name)

    linear_parameters = _read_vector(run_dir / "identified_linear_parameters.csv")
    stribeck_parameters = _read_vector(run_dir / "identified_stribeck_parameters.csv")
    namespace = args.namespace or config["namespace"]
    (include_dir / "identified_params.hpp").write_text(
        _params_header(namespace, linear_parameters, stribeck_parameters),
        encoding="utf-8",
    )

    print(source_dir / source.name)
    print(include_dir / header.name)
    print(include_dir / "identified_params.hpp")


if __name__ == "__main__":
    main()
