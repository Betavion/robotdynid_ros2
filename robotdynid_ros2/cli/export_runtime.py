"""Export generated runtime code into a controller package."""

from __future__ import annotations

import argparse
import csv
import json
import shutil
from pathlib import Path
import xml.etree.ElementTree as ET

from robotdynid_ros2.config import export_runtime_config, read_config


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="", help="Unified robotdynid_ros2 config file.")
    parser.add_argument("--run-dir", default="", help="Identification output directory.")
    parser.add_argument("--target-root", default="", help="Controller package root.")
    parser.add_argument("--include-subdir", default="")
    parser.add_argument("--source-subdir", default="")
    parser.add_argument("--runtime-subdir", default="")
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


def _find_prediction_metadata(run_dir: Path) -> Path | None:
    candidates = [
        run_dir / "codegen" / "cpp" / "predict_tau.json",
        run_dir / "identify" / "codegen" / "cpp" / "predict_tau.json",
    ]
    return next((path for path in candidates if path.exists()), None)


def _package_name(target_root: Path) -> str:
    package_xml = target_root / "package.xml"
    if not package_xml.exists():
        return target_root.name
    root = ET.parse(package_xml).getroot()
    name = root.findtext("name")
    return name.strip() if name and name.strip() else target_root.name


def _default_include_subdir(target_root: Path) -> str:
    return f"include/{_package_name(target_root)}/generated/robotdynid"


def _default_source_subdir() -> str:
    return "src/generated/robotdynid"


def _default_runtime_subdir() -> str:
    return "runtime/robotdynid"


def _include_path(include_subdir: str, header_name: str) -> str:
    path = Path(include_subdir)
    parts = path.parts
    if parts and parts[0] == "include":
        return str(Path(*parts[1:]) / header_name)
    return str(path / header_name)


def _copy_prediction_source(source: Path, destination: Path, header_include: str) -> None:
    text = source.read_text(encoding="utf-8")
    text = text.replace(f'#include "{source.with_suffix(".hpp").name}"', f'#include "{header_include}"', 1)
    destination.write_text(text, encoding="utf-8")


def _workspace_root(target_root: Path) -> Path | None:
    resolved = target_root.resolve()
    if resolved.parent.name == "src":
        return resolved.parent.parent
    return None


def _mirror_runtime_dirs(target_root: Path, runtime_subdir: str) -> list[Path]:
    package = _package_name(target_root)
    workspace = _workspace_root(target_root)
    if workspace is None:
        return []
    return [
        workspace / runtime_subdir,
        workspace / "install" / package / "share" / package / runtime_subdir,
    ]


def _manifest_path(path: Path, bases: list[Path | None]) -> str:
    resolved = path.resolve()
    for base in bases:
        if base is None:
            continue
        try:
            return str(resolved.relative_to(base.resolve()))
        except ValueError:
            continue
    return str(path)


def _runtime_manifest(
    *,
    target_root: Path,
    run_dir: Path,
    source_path: Path,
    header_path: Path,
    linear_path: Path,
    stribeck_path: Path,
    mirrored_runtime_dirs: list[Path],
    metadata_path: Path | None,
    linear_parameters: list[float],
    stribeck_parameters: list[float],
) -> str:
    metadata: dict[str, object] = {}
    if metadata_path is not None:
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    workspace = _workspace_root(target_root)
    payload = {
        "schema_version": 1,
        "run_dir": _manifest_path(run_dir, [workspace]),
        "source": _manifest_path(source_path, [target_root]),
        "header": _manifest_path(header_path, [target_root]),
        "linear_parameters": _manifest_path(linear_path, [target_root]),
        "stribeck_parameters": _manifest_path(stribeck_path, [target_root]),
        "mirrored_runtime_dirs": [_manifest_path(path, [workspace]) for path in mirrored_runtime_dirs],
        "metadata": _manifest_path(metadata_path, [workspace]) if metadata_path is not None else "",
        "dof": metadata.get("dof"),
        "linear_parameter_count": len(linear_parameters),
        "stribeck_parameter_count": len(stribeck_parameters),
    }
    return json.dumps(payload, indent=2) + "\n"


def export_runtime(
    *,
    run_dir: Path,
    target_root: Path,
    include_subdir: str = "",
    source_subdir: str = "",
    runtime_subdir: str = "",
) -> tuple[Path, Path, Path, Path, Path]:
    if not target_root.exists():
        raise FileNotFoundError(f"Target package root does not exist: {target_root}")

    source_dir = target_root / (source_subdir or _default_source_subdir())
    include_dir = target_root / (include_subdir or _default_include_subdir(target_root))
    runtime_dir = target_root / (runtime_subdir or _default_runtime_subdir())
    source_dir.mkdir(parents=True, exist_ok=True)
    include_dir.mkdir(parents=True, exist_ok=True)
    runtime_dir.mkdir(parents=True, exist_ok=True)

    source, header = _find_prediction_files(run_dir)
    source_path = source_dir / source.name
    header_path = include_dir / header.name
    manifest_path = source_dir / "runtime_manifest.json"
    linear_path = runtime_dir / "identified_linear_parameters.csv"
    stribeck_path = runtime_dir / "identified_stribeck_parameters.csv"

    shutil.copy2(header, header_path)
    _copy_prediction_source(source, source_path, _include_path(str(include_dir.relative_to(target_root)), header.name))
    shutil.copy2(run_dir / "identified_linear_parameters.csv", linear_path)
    shutil.copy2(run_dir / "identified_stribeck_parameters.csv", stribeck_path)

    linear_parameters = _read_vector(run_dir / "identified_linear_parameters.csv")
    stribeck_parameters = _read_vector(run_dir / "identified_stribeck_parameters.csv")
    mirrored_runtime_dirs: list[Path] = []
    for mirror_dir in _mirror_runtime_dirs(target_root, runtime_subdir or _default_runtime_subdir()):
        mirror_dir.mkdir(parents=True, exist_ok=True)
        shutil.copy2(linear_path, mirror_dir / linear_path.name)
        shutil.copy2(stribeck_path, mirror_dir / stribeck_path.name)
        mirrored_runtime_dirs.append(mirror_dir)
    manifest_path.write_text(
        _runtime_manifest(
            target_root=target_root,
            run_dir=run_dir,
            source_path=source_path,
            header_path=header_path,
            linear_path=linear_path,
            stribeck_path=stribeck_path,
            mirrored_runtime_dirs=mirrored_runtime_dirs,
            metadata_path=_find_prediction_metadata(run_dir),
            linear_parameters=linear_parameters,
            stribeck_parameters=stribeck_parameters,
        ),
        encoding="utf-8",
    )
    return source_path, header_path, linear_path, stribeck_path, manifest_path


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
    exported = export_runtime(
        run_dir=run_dir,
        target_root=target_root,
        include_subdir=args.include_subdir or str(config["include_subdir"]),
        source_subdir=args.source_subdir or str(config["source_subdir"]),
        runtime_subdir=args.runtime_subdir or str(config["runtime_subdir"]),
    )
    for path in exported:
        print(path)


if __name__ == "__main__":
    main()
