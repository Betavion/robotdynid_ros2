"""Inspect robotdynid run artifacts for GUI presentation."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from robotdynid_ros2.data.manifest import MANIFEST_NAME, read_manifest


def _read_json(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {}
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError:
        return {}
    return data if isinstance(data, dict) else {}


def _last_rmse(result: dict[str, Any]) -> list[float]:
    history = result.get("rmse_history", [])
    if not isinstance(history, list) or not history:
        return []
    last = history[-1]
    if not isinstance(last, list):
        return []
    values: list[float] = []
    for item in last:
        try:
            values.append(float(item))
        except (TypeError, ValueError):
            continue
    return values


@dataclass(frozen=True)
class RunArtifacts:
    run_dir: Path
    manifest_path: Path | None
    trajectory_csv: Path | None
    excitation_report_path: Path | None
    validation_report_path: Path | None
    identify_dir: Path | None
    identify_result_path: Path | None
    prediction_plot_path: Path | None
    codegen_dir: Path | None
    sample_count: int
    rmse: tuple[float, ...]

    @property
    def has_manifest(self) -> bool:
        return self.manifest_path is not None

    @property
    def has_trajectory(self) -> bool:
        return self.trajectory_csv is not None

    @property
    def has_identification(self) -> bool:
        return self.identify_result_path is not None

    @property
    def has_codegen(self) -> bool:
        return self.codegen_dir is not None

    def status_labels(self) -> list[str]:
        labels: list[str] = []
        if self.has_trajectory:
            labels.append("trajectory")
        if self.validation_report_path is not None:
            labels.append("validated")
        if self.has_manifest:
            labels.append("recorded")
        if self.has_identification:
            labels.append("identified")
        if self.has_codegen:
            labels.append("codegen")
        return labels


def inspect_run(run_dir: str | Path) -> RunArtifacts:
    root = Path(run_dir).expanduser()
    manifest_path = root / MANIFEST_NAME
    manifest: dict[str, Any] = {}
    if manifest_path.exists():
        manifest = read_manifest(manifest_path)
    else:
        manifest_path = None  # type: ignore[assignment]

    collection = manifest.get("collection", {}) if manifest else {}
    trajectory_raw = str(collection.get("commanded_trajectory_csv", "")) if isinstance(collection, dict) else ""
    trajectory_csv = Path(trajectory_raw) if trajectory_raw else None
    if trajectory_csv is not None and not trajectory_csv.is_absolute():
        trajectory_csv = (root / trajectory_csv).resolve()
    if trajectory_csv is None:
        for candidate in (root / "excitation.csv", root / "sia_excitation.csv"):
            if candidate.exists():
                trajectory_csv = candidate
                break
    if trajectory_csv is not None and not trajectory_csv.exists():
        trajectory_csv = None

    excitation_report = None
    validation_report = None
    if trajectory_csv is not None:
        report_candidate = trajectory_csv.with_name("excitation_report.json")
        validation_candidate = trajectory_csv.with_name("excitation_validation.json")
        excitation_report = report_candidate if report_candidate.exists() else None
        validation_report = validation_candidate if validation_candidate.exists() else None
    if excitation_report is None and (root / "excitation_report.json").exists():
        excitation_report = root / "excitation_report.json"
    if validation_report is None and (root / "excitation_validation.json").exists():
        validation_report = root / "excitation_validation.json"

    identify_dir = root / "identify" if (root / "identify").exists() else root
    identify_result = identify_dir / "identify_result.json"
    if not identify_result.exists():
        identify_result = None  # type: ignore[assignment]
        identify_dir = None  # type: ignore[assignment]

    prediction_plot = identify_dir / "prediction.png" if identify_dir is not None else None
    if prediction_plot is not None and not prediction_plot.exists():
        prediction_plot = None
    codegen_dir = identify_dir / "codegen" if identify_dir is not None else None
    if codegen_dir is not None and not codegen_dir.exists():
        codegen_dir = None

    result_payload = _read_json(identify_result) if identify_result is not None else {}
    sample_count = int(collection.get("sample_count", 0)) if isinstance(collection, dict) else 0
    if not sample_count and result_payload:
        try:
            sample_count = int(result_payload.get("sample_count", 0))
        except (TypeError, ValueError):
            sample_count = 0

    return RunArtifacts(
        run_dir=root,
        manifest_path=manifest_path,
        trajectory_csv=trajectory_csv,
        excitation_report_path=excitation_report,
        validation_report_path=validation_report,
        identify_dir=identify_dir,
        identify_result_path=identify_result,
        prediction_plot_path=prediction_plot,
        codegen_dir=codegen_dir,
        sample_count=sample_count,
        rmse=tuple(_last_rmse(result_payload)),
    )


def scan_runs(output_root: str | Path) -> list[RunArtifacts]:
    root = Path(output_root).expanduser()
    if not root.exists():
        return []
    runs: list[RunArtifacts] = []
    for child in sorted(root.iterdir(), reverse=True):
        if child.is_dir():
            artifacts = inspect_run(child)
            if any(
                (
                    artifacts.has_manifest,
                    artifacts.has_trajectory,
                    artifacts.has_identification,
                    artifacts.has_codegen,
                )
            ):
                runs.append(artifacts)
    return runs

