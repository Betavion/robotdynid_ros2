"""Run manifest helpers."""

from __future__ import annotations

import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


MANIFEST_NAME = "manifest.yaml"


def utc_timestamp() -> str:
    return datetime.now(timezone.utc).isoformat()


def write_manifest(path: str | Path, data: dict[str, Any]) -> Path:
    manifest_path = Path(path)
    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    manifest_path.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
    return manifest_path


def read_manifest(path: str | Path) -> dict[str, Any]:
    raw = Path(path).read_text(encoding="utf-8")
    try:
        data = json.loads(raw)
    except json.JSONDecodeError:
        try:
            import yaml  # type: ignore[import-not-found]
        except ModuleNotFoundError as exc:
            raise ValueError(f"Manifest {path} is not JSON and PyYAML is not installed.") from exc
        data = yaml.safe_load(raw)
    if not isinstance(data, dict):
        raise ValueError(f"Manifest {path} must contain a mapping.")
    return data


def build_collection_manifest(
    *,
    run_dir: str | Path,
    data_dir: str | Path,
    motion_csv: str | Path,
    torque_csv: str | Path,
    joint_names: list[str],
    joint_state_topic: str,
    sample_count: int,
    urdf_path: str | Path | None = None,
    qdd_source: str = "estimated_from_velocity",
    estimate_joint_state_topic: str = "",
    estimate_csv: str | Path | None = None,
    estimate_sample_count: int = 0,
    dropped_motion_torque_sample_count: int = 0,
    dropped_estimate_sample_count: int = 0,
) -> dict[str, Any]:
    run_path = Path(run_dir).resolve()
    data_path = Path(data_dir).resolve()
    motion_path = Path(motion_csv).resolve()
    torque_path = Path(torque_csv).resolve()
    estimate_path = Path(estimate_csv).resolve() if estimate_csv else None
    urdf_resolved = Path(urdf_path).expanduser().resolve() if urdf_path else None
    return {
        "schema_version": 1,
        "created_at": utc_timestamp(),
        "run_dir": str(run_path),
        "robot": {
            "urdf_path": str(urdf_resolved) if urdf_resolved else "",
            "joint_names": list(joint_names),
            "dof": len(joint_names),
        },
        "collection": {
            "joint_state_topic": joint_state_topic,
            "estimate_joint_state_topic": estimate_joint_state_topic,
            "torque_source": "sensor_msgs/JointState.effort",
            "qdd_source": qdd_source,
            "sample_count": int(sample_count),
            "estimate_sample_count": int(estimate_sample_count),
            "dropped_motion_torque_sample_count": int(dropped_motion_torque_sample_count),
            "dropped_estimate_sample_count": int(dropped_estimate_sample_count),
            "writer_mode": "buffered_timer_flush",
        },
        "data": {
            "data_dir": str(data_path),
            "motion_csv": str(motion_path),
            "torque_csv": str(torque_path),
            **({"estimate_csv": str(estimate_path)} if estimate_path else {}),
        },
    }


def resolve_manifest_data_paths(manifest_path: str | Path) -> tuple[Path, Path, Path | None, int]:
    manifest_file = Path(manifest_path)
    manifest = read_manifest(manifest_file)
    base_dir = manifest_file.parent
    robot = manifest.get("robot", {})
    data = manifest.get("data", {})
    dof = int(robot.get("dof") or len(robot.get("joint_names", [])))
    if dof <= 0:
        raise ValueError("Manifest must contain robot.dof or robot.joint_names.")

    def resolve(raw: str) -> Path:
        value = Path(raw)
        return value if value.is_absolute() else (base_dir / value).resolve()

    motion_csv = resolve(str(data["motion_csv"]))
    torque_csv = resolve(str(data["torque_csv"]))
    urdf_raw = str(robot.get("urdf_path", ""))
    urdf_path = resolve(urdf_raw) if urdf_raw else None
    return motion_csv, torque_csv, urdf_path, dof
