"""Build commands executed by the GUI."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class CommandSpec:
    program: str
    args: tuple[str, ...]
    label: str

    def argv(self) -> list[str]:
        return [self.program, *self.args]

    def display(self) -> str:
        return " ".join(self.argv())


def _path(value: str | Path) -> str:
    return str(Path(value).expanduser())


def generate_excitation(config: str | Path, output: str | Path, *, validate: bool = False) -> CommandSpec:
    args = [
        "run",
        "robotdynid_ros2",
        "robotdynid-generate-excitation",
        "--config",
        _path(config),
        "--output",
        _path(output),
    ]
    if validate:
        args.append("--validate")
    return CommandSpec("ros2", tuple(args), "Generate excitation")


def validate_excitation(
    config: str | Path,
    trajectory: str | Path,
    *,
    dry_run: bool = False,
    require_collision: bool = False,
    action_name: str = "",
) -> CommandSpec:
    args = [
        "run",
        "robotdynid_ros2",
        "robotdynid-validate-excitation",
        "--config",
        _path(config),
        "--trajectory",
        _path(trajectory),
    ]
    if dry_run:
        args.append("--dry-run")
    if require_collision:
        args.append("--require-collision")
    if dry_run and action_name:
        args.extend(["--action-name", action_name])
    return CommandSpec("ros2", tuple(args), "Validate excitation")


def preview_trajectory(
    config: str | Path,
    trajectory: str | Path,
    *,
    topic: str = "/display_planned_path",
) -> CommandSpec:
    return CommandSpec(
        "ros2",
        (
            "run",
            "robotdynid_ros2",
            "robotdynid-preview-trajectory",
            "--config",
            _path(config),
            "--trajectory",
            _path(trajectory),
            "--topic",
            topic,
        ),
        "RViz trajectory preview",
    )


def cancel_trajectory(action_name: str, *, timeout_sec: float = 2.0) -> CommandSpec:
    return CommandSpec(
        "ros2",
        (
            "run",
            "robotdynid_ros2",
            "robotdynid-cancel-trajectory",
            "--action-name",
            action_name,
            "--timeout-sec",
            f"{timeout_sec:g}",
        ),
        "Cancel trajectory",
    )


def collect_with_trajectory(
    config: str | Path,
    *,
    trajectory: str | Path | None = None,
    generate_trajectory: bool = False,
) -> CommandSpec:
    args = [
        "launch",
        "robotdynid_ros2",
        "collect_with_trajectory.launch.py",
        f"config:={_path(config)}",
    ]
    if trajectory:
        args.append(f"trajectory_csv:={_path(trajectory)}")
    if generate_trajectory:
        args.append("generate_trajectory:=true")
    return CommandSpec("ros2", tuple(args), "Collect dataset")


def full_pipeline(config: str | Path, *, generate_trajectory: bool = True) -> CommandSpec:
    args = [
        "launch",
        "robotdynid_ros2",
        "full_pipeline.launch.py",
        f"config:={_path(config)}",
    ]
    if generate_trajectory:
        args.append("generate_trajectory:=true")
    return CommandSpec("ros2", tuple(args), "Full pipeline")


def identify_codegen(
    config: str | Path,
    *,
    manifest: str | Path | None = None,
    export_code: bool = False,
) -> CommandSpec:
    args = [
        "run",
        "robotdynid_ros2",
        "robotdynid-identify-codegen",
        "--config",
        _path(config),
    ]
    if manifest:
        args.extend(["--manifest", _path(manifest)])
    if export_code:
        args.append("--export-code")
    return CommandSpec("ros2", tuple(args), "Identify and codegen")


def export_runtime(
    config: str | Path,
    *,
    run_dir: str | Path,
    target_root: str | Path,
) -> CommandSpec:
    return CommandSpec(
        "ros2",
        (
            "run",
            "robotdynid_ros2",
            "robotdynid-export-runtime",
            "--config",
            _path(config),
            "--run-dir",
            _path(run_dir),
            "--target-root",
            _path(target_root),
        ),
        "Export runtime",
    )
