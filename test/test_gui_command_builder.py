from __future__ import annotations

from robotdynid_ros2.gui import command_builder


def test_generate_excitation_command() -> None:
    spec = command_builder.generate_excitation("config.yaml", "runs/excitation.csv", validate=True)

    assert spec.program == "ros2"
    assert spec.argv() == [
        "ros2",
        "run",
        "robotdynid_ros2",
        "robotdynid-generate-excitation",
        "--config",
        "config.yaml",
        "--output",
        "runs/excitation.csv",
        "--validate",
    ]


def test_validate_excitation_collision_command_does_not_send_motion() -> None:
    spec = command_builder.validate_excitation(
        "config.yaml",
        "runs/excitation.csv",
        require_collision=True,
        action_name="/controller/follow_joint_trajectory",
    )

    assert "--require-collision" in spec.args
    assert "--dry-run" not in spec.args
    assert "--action-name" not in spec.args


def test_validate_excitation_explicit_dry_run_command() -> None:
    spec = command_builder.validate_excitation(
        "config.yaml",
        "runs/excitation.csv",
        dry_run=True,
        action_name="/controller/follow_joint_trajectory",
    )

    assert "--dry-run" in spec.args
    assert spec.args[-2:] == ("--action-name", "/controller/follow_joint_trajectory")


def test_preview_trajectory_command_is_rviz_only() -> None:
    spec = command_builder.preview_trajectory("config.yaml", "runs/excitation.csv")

    assert spec.argv() == [
        "ros2",
        "run",
        "robotdynid_ros2",
        "robotdynid-preview-trajectory",
        "--config",
        "config.yaml",
        "--trajectory",
        "runs/excitation.csv",
        "--topic",
        "/display_planned_path",
    ]
    assert "--dry-run" not in spec.args
    assert "--action-name" not in spec.args


def test_cancel_trajectory_command() -> None:
    spec = command_builder.cancel_trajectory("/controller/follow_joint_trajectory", timeout_sec=1.5)

    assert spec.argv() == [
        "ros2",
        "run",
        "robotdynid_ros2",
        "robotdynid-cancel-trajectory",
        "--action-name",
        "/controller/follow_joint_trajectory",
        "--timeout-sec",
        "1.5",
    ]


def test_collect_and_identify_commands() -> None:
    collect = command_builder.collect_with_trajectory("config.yaml", trajectory="runs/excitation.csv")
    identify = command_builder.identify_codegen("config.yaml", manifest="runs/001/manifest.yaml", export_code=True)

    assert collect.argv() == [
        "ros2",
        "launch",
        "robotdynid_ros2",
        "collect_with_trajectory.launch.py",
        "config:=config.yaml",
        "trajectory_csv:=runs/excitation.csv",
    ]
    assert identify.argv()[-3:] == ["--manifest", "runs/001/manifest.yaml", "--export-code"]


def test_export_runtime_command() -> None:
    spec = command_builder.export_runtime("config.yaml", run_dir="runs/001/identify", target_root="/tmp/controller")

    assert spec.argv() == [
        "ros2",
        "run",
        "robotdynid_ros2",
        "robotdynid-export-runtime",
        "--config",
        "config.yaml",
        "--run-dir",
        "runs/001/identify",
        "--target-root",
        "/tmp/controller",
    ]
