from __future__ import annotations

import os
from pathlib import Path

import pytest


def test_main_window_loads_config_offscreen(tmp_path: Path) -> None:
    pytest.importorskip("PySide6")
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

    from PySide6.QtWidgets import QApplication

    from robotdynid_ros2.gui.app import parse_args
    from robotdynid_ros2.gui.main_window import MainWindow

    app = QApplication.instance() or QApplication([])
    package_root = Path(__file__).resolve().parents[1]
    workspace_root = package_root.parents[1]
    config = package_root / "config" / "sia_example.yaml"
    window = MainWindow(
        parse_args(
            [
                "--config",
                str(config),
                "--workspace",
                str(workspace_root),
                "--no-ros-monitor",
            ]
        )
    )
    try:
        assert window.windowTitle() == "robotdynid_ros2 Studio"
        assert window.robot_page.dof.value() == 6
        assert "joint1" in window.robot_page.joint_names.text()
        assert window.robot_page.limits_table.rowCount() == 6
        assert window.robot_page.limits_table.item(1, 4).text() == "2.0"
        assert window.robot_page.limits_table.item(1, 5).text() == "20.0"
        assert window.excitation_page.moveit_group.text() == "sia_arm"
        assert window.collect_page.trajectory_csv.text().endswith("runs/sia_excitation.csv")
    finally:
        window.close()
        app.processEvents()


def test_excitation_validation_controls_round_trip(tmp_path: Path) -> None:
    pytest.importorskip("PySide6")
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

    from PySide6.QtWidgets import QApplication

    from robotdynid_ros2.gui.app import parse_args
    from robotdynid_ros2.gui.main_window import MainWindow

    app = QApplication.instance() or QApplication([])
    config = tmp_path / "config.yaml"
    urdf = tmp_path / "robot.urdf"
    urdf.write_text("<robot name='r' />", encoding="utf-8")
    config.write_text(
        f"""
robot:
  urdf_path: {urdf}
  dof: 1
  joint_names: [joint1]
trajectory:
  csv_path: runs/test.csv
  generation:
    position_lower: [-0.3]
    position_upper: [0.4]
  validate:
    enabled: true
    collision: true
    require_collision: true
    moveit_group: sia_arm
    state_validity_service: /check_state_validity
    collision_sample_limit: 1234
    torque: true
    torque_sample_limit: 321
    max_torque_scale: 0.7
    dry_run: true
""",
        encoding="utf-8",
    )

    window = MainWindow(
        parse_args(
            [
                "--config",
                str(config),
                "--workspace",
                str(tmp_path),
                "--no-ros-monitor",
            ]
        )
    )
    try:
        assert window.excitation_page.collision_enabled.isChecked()
        assert window.excitation_page.position_lower.text() == "-0.3"
        assert window.excitation_page.position_upper.text() == "0.4"
        assert window.excitation_page.friction_speed_levels.value() == 3
        assert window.excitation_page.gravity_pose_count.value() == 5
        assert window.collect_page.trajectory_csv.text() == "runs/test.csv"
        assert window.excitation_page.require_collision.isChecked()
        assert window.excitation_page.collision_sample_limit.value() == 1234
        assert window.excitation_page.torque_enabled.isChecked()
        window.excitation_page.position_lower.setText("-0.2")
        window.excitation_page.position_upper.setText("0.3")
        window.excitation_page.friction_speed_levels.setValue(4)
        window.excitation_page.gravity_pose_count.setValue(6)
        window.collect_page.trajectory_csv.setText("runs/manual_excitation.csv")
        window.identify_page.manifest.setText("runs/manual/manifest.yaml")
        window.excitation_page.collision_sample_limit.setValue(5678)
        window._save_config()
    finally:
        window.close()
        app.processEvents()

    saved = config.read_text(encoding="utf-8")
    assert "collision_sample_limit: 5678" in saved
    assert "position_lower:" in saved
    assert "friction_speed_levels: 4" in saved
    assert "gravity_pose_count: 6" in saved
    assert "commanded_trajectory_csv: runs/manual_excitation.csv" in saved
    assert "manifest:" not in saved
    assert "- -0.2" in saved
    assert "- 0.3" in saved
    assert "dry_run:" not in saved


def test_main_window_language_switch_updates_workflow_labels(tmp_path: Path) -> None:
    pytest.importorskip("PySide6")
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

    from PySide6.QtWidgets import QApplication

    from robotdynid_ros2.gui.app import parse_args
    from robotdynid_ros2.gui.main_window import MainWindow

    app = QApplication.instance() or QApplication([])
    package_root = Path(__file__).resolve().parents[1]
    workspace_root = package_root.parents[1]
    config = package_root / "config" / "sia_example.yaml"
    window = MainWindow(
        parse_args(
            [
                "--config",
                str(config),
                "--workspace",
                str(workspace_root),
                "--language",
                "zh",
                "--no-ros-monitor",
            ]
        )
    )
    try:
        assert window.windowTitle() == "robotdynid_ros2 工作台"
        assert "模型" in window.nav.item(0).text()
        assert window.excitation_page.generate_button.text() == "生成"
        assert window.excitation_page.collision_check_button.text() == "碰撞校验"
        assert "不发布 /joint_states" in window.excitation_page.collision_check_button.toolTip()
        assert window.excitation_page.trajectory_preview_button.text() == "RViz 预览"
        assert "DisplayTrajectory" in window.excitation_page.trajectory_preview_button.toolTip()
        assert "不发送 FollowJointTrajectory" in window.excitation_page.trajectory_preview_button.toolTip()
        assert window.identify_page.stride_label.text() == "降采样步长"
        assert "每隔 N 个样本" in window.identify_page.stride.toolTip()
        assert window.collect_page.trajectory_csv_label.text() == "轨迹 CSV"
        assert "默认使用激励轨迹生成器" in window.collect_page.trajectory_csv.toolTip()
        assert "rad/s^2" in window.excitation_page.output.toolTip()
        assert "工作区" in window.excitation_page.position_lower.toolTip()
        english_index = window.language_combo.findData("en")
        window.language_combo.setCurrentIndex(english_index)
        assert window.windowTitle() == "robotdynid_ros2 Studio"
        assert "Robot" in window.nav.item(0).text()
        assert window.excitation_page.generate_button.text() == "Generate"
        assert window.excitation_page.collision_check_button.text() == "Collision Check"
        assert "does not publish /joint_states" in window.excitation_page.collision_check_button.toolTip()
        assert window.excitation_page.trajectory_preview_button.text() == "RViz Preview"
        assert "DisplayTrajectory" in window.excitation_page.trajectory_preview_button.toolTip()
        assert "does not publish /joint_states" in window.excitation_page.trajectory_preview_button.toolTip()
        assert window.identify_page.stride_label.text() == "Downsample stride"
        assert "Keep every Nth sample" in window.identify_page.stride.toolTip()
        assert window.collect_page.trajectory_csv_label.text() == "Trajectory CSV"
        assert "trajectory generator output" in window.collect_page.trajectory_csv.toolTip()
    finally:
        window.close()
        app.processEvents()


def test_collect_uses_selected_collection_trajectory(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    pytest.importorskip("PySide6")
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

    from PySide6.QtWidgets import QApplication

    from robotdynid_ros2.gui.app import parse_args
    from robotdynid_ros2.gui.command_builder import CommandSpec
    from robotdynid_ros2.gui.main_window import MainWindow

    app = QApplication.instance() or QApplication([])
    config = tmp_path / "config.yaml"
    urdf = tmp_path / "robot.urdf"
    urdf.write_text("<robot name='r' />", encoding="utf-8")
    config.write_text(
        f"""
robot:
  urdf_path: {urdf}
  dof: 1
  joint_names: [joint1]
trajectory:
  csv_path: runs/generated.csv
recording:
  joint_state_topic: /joint_states
""",
        encoding="utf-8",
    )
    window = MainWindow(
        parse_args(
            [
                "--config",
                str(config),
                "--workspace",
                str(tmp_path),
                "--no-ros-monitor",
            ]
        )
    )
    commands: list[CommandSpec] = []
    monkeypatch.setattr(window.runner, "run", commands.append)
    try:
        window.collect_page.trajectory_csv.setText("runs/manual.csv")
        window._collect_with_trajectory()
    finally:
        window.close()
        app.processEvents()

    assert commands
    assert "trajectory_csv:=runs/manual.csv" in commands[0].argv()


def test_stop_cancels_active_follow_joint_trajectory(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    pytest.importorskip("PySide6")
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

    from PySide6.QtWidgets import QApplication

    from robotdynid_ros2.gui import main_window as main_window_module
    from robotdynid_ros2.gui.app import parse_args
    from robotdynid_ros2.gui.main_window import MainWindow

    app = QApplication.instance() or QApplication([])
    config = tmp_path / "config.yaml"
    urdf = tmp_path / "robot.urdf"
    urdf.write_text("<robot name='r' />", encoding="utf-8")
    config.write_text(
        f"""
robot:
  urdf_path: {urdf}
  dof: 1
  joint_names: [joint1]
trajectory:
  action_name: /controller/follow_joint_trajectory
  csv_path: runs/generated.csv
recording:
  joint_state_topic: /joint_states
""",
        encoding="utf-8",
    )
    window = MainWindow(
        parse_args(
            [
                "--config",
                str(config),
                "--workspace",
                str(tmp_path),
                "--no-ros-monitor",
            ]
        )
    )
    detached: list[tuple[str, list[str], str]] = []
    stopped: list[bool] = []
    monkeypatch.setattr(
        main_window_module.QProcess,
        "startDetached",
        staticmethod(lambda program, args, cwd: detached.append((program, list(args), cwd)) or True),
    )
    monkeypatch.setattr(window.runner, "stop", lambda: stopped.append(True))
    try:
        window._stop_active_process()
    finally:
        window.close()
        app.processEvents()

    assert stopped
    assert detached
    assert detached[0][0] == "ros2"
    assert detached[0][1] == [
        "run",
        "robotdynid_ros2",
        "robotdynid-cancel-trajectory",
        "--action-name",
        "/controller/follow_joint_trajectory",
        "--timeout-sec",
        "1.5",
    ]


def test_collect_finish_loads_manifest_and_motion_preview(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    pytest.importorskip("PySide6")
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

    from PySide6.QtWidgets import QApplication

    from robotdynid_ros2.data.manifest import build_collection_manifest, write_manifest
    from robotdynid_ros2.gui.app import parse_args
    from robotdynid_ros2.gui.main_window import MainWindow
    from robotdynid_ros2.gui.process_runner import ProcessRecord

    app = QApplication.instance() or QApplication([])
    config = tmp_path / "config.yaml"
    urdf = tmp_path / "robot.urdf"
    urdf.write_text("<robot name='r' />", encoding="utf-8")
    config.write_text(
        f"""
robot:
  urdf_path: {urdf}
  dof: 2
  joint_names: [joint1, joint2]
run:
  output_root: runs
trajectory:
  csv_path: runs/generated.csv
recording:
  joint_state_topic: /joint_states
""",
        encoding="utf-8",
    )
    run_dir = tmp_path / "runs" / "20260521_221500"
    data_dir = run_dir / "data"
    data_dir.mkdir(parents=True)
    motion = data_dir / "motion.csv"
    torque = data_dir / "torque_measure_data.csv"
    motion.write_text(
        "timestamp,joint1_position,joint1_velocity,joint2_position,joint2_velocity\n0,0,0,1,0\n0.1,0.2,2,1.1,1\n",
        encoding="utf-8",
    )
    torque.write_text("timestamp,joint1_measure,joint2_measure\n0,0,0\n0.1,1,2\n", encoding="utf-8")
    manifest_path = write_manifest(
        run_dir / "manifest.yaml",
        build_collection_manifest(
            run_dir=run_dir,
            data_dir=data_dir,
            motion_csv=motion,
            torque_csv=torque,
            joint_names=["joint1", "joint2"],
            joint_state_topic="/joint_states",
            sample_count=2,
        ),
    )
    window = MainWindow(
        parse_args(
            [
                "--config",
                str(config),
                "--workspace",
                str(tmp_path),
                "--no-ros-monitor",
            ]
        )
    )
    plotted: list[tuple[str, list[str]]] = []
    monkeypatch.setattr(window.collect_page.preview, "plot_columns", lambda path, columns: plotted.append((str(path), columns)))
    try:
        window._append_log("[dataset_recorder-1] Recording robot dynamics dataset into runs/20260521_221500\n")
        window._on_process_finished(ProcessRecord("Collect dataset", "cmd", exit_code=0))
    finally:
        window.close()
        app.processEvents()

    assert window.identify_page.manifest.text() == str(manifest_path)
    assert window.collect_page.preview_path.text() == str(motion)
    assert plotted == [(str(motion), ["joint1_position", "joint2_position"])]


def test_collect_finish_selects_reported_run_not_sorted_history(tmp_path: Path) -> None:
    pytest.importorskip("PySide6")
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

    from PySide6.QtWidgets import QApplication

    from robotdynid_ros2.data.manifest import build_collection_manifest, write_manifest
    from robotdynid_ros2.gui.app import parse_args
    from robotdynid_ros2.gui.main_window import MainWindow
    from robotdynid_ros2.gui.process_runner import ProcessRecord

    def write_run_manifest(name: str) -> Path:
        run_dir = tmp_path / "runs" / name
        data_dir = run_dir / "data"
        data_dir.mkdir(parents=True)
        motion = data_dir / "motion.csv"
        torque = data_dir / "torque_measure_data.csv"
        motion.write_text("timestamp,joint1_position\n0,0\n", encoding="utf-8")
        torque.write_text("timestamp,joint1_measure\n0,0\n", encoding="utf-8")
        return write_manifest(
            run_dir / "manifest.yaml",
            build_collection_manifest(
                run_dir=run_dir,
                data_dir=data_dir,
                motion_csv=motion,
                torque_csv=torque,
                joint_names=["joint1"],
                joint_state_topic="/joint_states",
                sample_count=1,
            ),
        )

    stale_manifest = write_run_manifest("test_run")
    reported_manifest = write_run_manifest("20260522_174854")
    config = tmp_path / "config.yaml"
    urdf = tmp_path / "robot.urdf"
    urdf.write_text("<robot name='r' />", encoding="utf-8")
    config.write_text(
        f"""
robot:
  urdf_path: {urdf}
  dof: 1
  joint_names: [joint1]
run:
  output_root: runs
trajectory:
  csv_path: runs/generated.csv
recording:
  joint_state_topic: /joint_states
""",
        encoding="utf-8",
    )

    app = QApplication.instance() or QApplication([])
    window = MainWindow(
        parse_args(
            [
                "--config",
                str(config),
                "--workspace",
                str(tmp_path),
                "--no-ros-monitor",
            ]
        )
    )
    try:
        window._append_log("[dataset_recorder-1] Recording robot dynamics dataset into runs/20260522_174854\n")
        window._on_process_finished(ProcessRecord("Collect dataset", "cmd", exit_code=-15))
    finally:
        window.close()
        app.processEvents()

    assert stale_manifest.exists()
    assert window.identify_page.manifest.text() == str(reported_manifest)
    assert window.active_run_field.text() == "20260522_174854"


def test_identify_finish_loads_prediction_plot(tmp_path: Path) -> None:
    pytest.importorskip("PySide6")
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

    from PySide6.QtWidgets import QApplication

    from robotdynid_ros2.data.manifest import build_collection_manifest, write_manifest
    from robotdynid_ros2.gui.app import parse_args
    from robotdynid_ros2.gui.main_window import MainWindow
    from robotdynid_ros2.gui.process_runner import ProcessRecord

    app = QApplication.instance() or QApplication([])
    config = tmp_path / "config.yaml"
    urdf = tmp_path / "robot.urdf"
    urdf.write_text("<robot name='r' />", encoding="utf-8")
    config.write_text(
        f"""
robot:
  urdf_path: {urdf}
  dof: 1
  joint_names: [joint1]
run:
  output_root: runs
trajectory:
  csv_path: runs/generated.csv
recording:
  joint_state_topic: /joint_states
""",
        encoding="utf-8",
    )
    run_dir = tmp_path / "runs" / "20260521_221600"
    data_dir = run_dir / "data"
    data_dir.mkdir(parents=True)
    motion = data_dir / "motion.csv"
    torque = data_dir / "torque_measure_data.csv"
    motion.write_text("timestamp,joint1_position\n0,0\n", encoding="utf-8")
    torque.write_text("timestamp,joint1_measure\n0,0\n", encoding="utf-8")
    manifest_path = write_manifest(
        run_dir / "manifest.yaml",
        build_collection_manifest(
            run_dir=run_dir,
            data_dir=data_dir,
            motion_csv=motion,
            torque_csv=torque,
            joint_names=["joint1"],
            joint_state_topic="/joint_states",
            sample_count=1,
        ),
    )
    identify_dir = run_dir / "identify"
    identify_dir.mkdir()
    (identify_dir / "identify_result.json").write_text('{"sample_count": 1, "rmse_history": [[0.1]]}', encoding="utf-8")
    prediction = identify_dir / "prediction.png"
    prediction.write_bytes(b"not a real png but path exists")
    window = MainWindow(
        parse_args(
            [
                "--config",
                str(config),
                "--workspace",
                str(tmp_path),
                "--no-ros-monitor",
            ]
        )
    )
    try:
        window.identify_page.manifest.setText(str(manifest_path))
        window._on_process_finished(ProcessRecord("Identify and codegen", "cmd", exit_code=0))
    finally:
        window.close()
        app.processEvents()

    assert window.export_page.run_dir.text() == str(identify_dir)
    assert window.identify_page.result_summary.text() == str(prediction)


def test_main_window_can_fit_1080p_height(tmp_path: Path) -> None:
    pytest.importorskip("PySide6")
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

    from PySide6.QtWidgets import QApplication

    from robotdynid_ros2.gui.app import parse_args
    from robotdynid_ros2.gui.main_window import MainWindow

    app = QApplication.instance() or QApplication([])
    package_root = Path(__file__).resolve().parents[1]
    workspace_root = package_root.parents[1]
    config = package_root / "config" / "sia_example.yaml"
    window = MainWindow(
        parse_args(
            [
                "--config",
                str(config),
                "--workspace",
                str(workspace_root),
                "--language",
                "zh",
                "--no-ros-monitor",
            ]
        )
    )
    try:
        window.nav.setCurrentRow(1)
        window.resize(1440, 840)
        window.show()
        app.processEvents()
        assert window.width() <= 1440
        assert window.height() <= 840
        assert 220 <= window.excitation_page.position_plot.maximumHeight() <= 280
        assert 220 <= window.collect_page.preview.maximumHeight() <= 280
        assert window.excitation_page.position_plot.fit_x_button.text() == "Fit X"
        assert window.excitation_page.position_plot.zoom_y_button.isChecked()
        assert window.workflow_scroll.horizontalScrollBar().maximum() == 0
        assert window.workflow_scroll.verticalScrollBar().maximum() > 0
    finally:
        window.close()
        app.processEvents()


def test_trajectory_plot_uses_standard_axis_units(tmp_path: Path) -> None:
    pytest.importorskip("PySide6")
    pytest.importorskip("pyqtgraph")
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

    import numpy as np
    from PySide6.QtWidgets import QApplication

    from robotdynid_ros2.gui.plot_widgets import TrajectoryPlot
    from robotdynid_ros2.trajectory.schema import TrajectoryData, write_trajectory_csv

    app = QApplication.instance() or QApplication([])
    path = write_trajectory_csv(
        tmp_path / "excitation.csv",
        TrajectoryData(
            joint_names=("joint1",),
            time=np.asarray([0.0, 0.1, 0.2]),
            position=np.asarray([[0.0], [1e-4], [0.0]]),
            velocity=np.asarray([[0.0], [1e-3], [0.0]]),
            acceleration=np.asarray([[0.0], [1e-2], [0.0]]),
        ),
    )
    plot = TrajectoryPlot()
    try:
        plot.plot_trajectory(path, "acceleration")
        left_axis = plot._plot.getAxis("left")
        bottom_axis = plot._plot.getAxis("bottom")
        assert left_axis.labelText == "acceleration"
        assert left_axis.labelUnits == "rad/s^2"
        assert bottom_axis.labelText == "time"
        assert bottom_axis.labelUnits == "s"
        assert left_axis.autoSIPrefix is False
        assert bottom_axis.autoSIPrefix is False
    finally:
        plot.close()
        app.processEvents()


def test_ros_monitor_action_query_uses_rclpy_action_fallback(monkeypatch: pytest.MonkeyPatch) -> None:
    pytest.importorskip("PySide6")
    rclpy_action = pytest.importorskip("rclpy.action")

    from robotdynid_ros2.gui.ros_monitor import _action_names_from_graph

    class LegacyNode:
        pass

    monkeypatch.setattr(
        rclpy_action,
        "get_action_names_and_types",
        lambda node: [
            ("/joint_trajectory_controller/follow_joint_trajectory", ["control_msgs/action/FollowJointTrajectory"]),
            ("/move_action", ["moveit_msgs/action/MoveGroup"]),
        ],
    )

    assert _action_names_from_graph(LegacyNode()) == (
        "/joint_trajectory_controller/follow_joint_trajectory",
        "/move_action",
    )
