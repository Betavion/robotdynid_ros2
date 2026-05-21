"""Main PySide6 window for robotdynid_ros2."""

from __future__ import annotations

import argparse
from pathlib import Path

from PySide6.QtCore import QProcess, QSize, QUrl, Qt
from PySide6.QtGui import QDesktopServices, QTextCursor
from PySide6.QtWidgets import (
    QComboBox,
    QFileDialog,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QListWidget,
    QListWidgetItem,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QPlainTextEdit,
    QScrollArea,
    QSizePolicy,
    QStackedWidget,
    QVBoxLayout,
    QWidget,
)

from robotdynid_ros2.data.manifest import read_manifest
from robotdynid_ros2.gui import command_builder
from robotdynid_ros2.gui.artifact_model import RunArtifacts, inspect_run, scan_runs
from robotdynid_ros2.gui.config_model import GuiConfigModel
from robotdynid_ros2.gui.i18n import LANGUAGES, normalize_language, tr
from robotdynid_ros2.gui.pages.collect_page import CollectPage
from robotdynid_ros2.gui.pages.excitation_page import ExcitationPage
from robotdynid_ros2.gui.pages.export_page import ExportPage
from robotdynid_ros2.gui.pages.identify_page import IdentifyPage
from robotdynid_ros2.gui.pages.robot_page import RobotPage
from robotdynid_ros2.gui.pages.runs_page import RunsPage
from robotdynid_ros2.gui.process_runner import ProcessRecord, ProcessRunner
from robotdynid_ros2.gui.ros_monitor import RosGraphMonitor, RosGraphState
from robotdynid_ros2.gui.widgets import LogoMark, set_tooltip


def _default_config_path(workspace: Path) -> Path | None:
    candidates = [
        workspace / "src" / "robotdynid_ros2" / "config" / "sia_example.yaml",
        workspace / "config" / "sia_example.yaml",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return None


class MainWindow(QMainWindow):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__()
        self.workspace = Path(args.workspace or Path.cwd()).expanduser()
        self.model = GuiConfigModel()
        self.runner = ProcessRunner(self.workspace, self)
        self.monitor = RosGraphMonitor() if not args.no_ros_monitor else None
        self.language = normalize_language(getattr(args, "language", "en"))
        self._active_run: Path | None = None
        self._last_trajectory: Path | None = None
        self._summary_status_keys: dict[str, str] = {}

        self.setWindowTitle(tr(self.language, "app_title"))
        self.resize(1440, 840)
        self.setMinimumSize(1180, 720)
        self._build_ui()
        self._connect()
        self._apply_language()

        initial_config = self._workspace_path(args.config) if args.config else _default_config_path(self.workspace)
        if initial_config is not None and initial_config.exists():
            self.robot_page.config_path.setText(str(initial_config))
            self._load_config(initial_config)

        if self.monitor is not None:
            self.monitor.state_changed.connect(self._on_ros_state)
            self.monitor.start()

    def _build_ui(self) -> None:
        central = QWidget()
        root = QVBoxLayout(central)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)
        root.addWidget(self._build_topbar())

        body = QHBoxLayout()
        body.setContentsMargins(0, 0, 0, 0)
        body.setSpacing(0)
        self.nav = QListWidget()
        self.nav.setObjectName("NavRail")
        self.nav.setFixedWidth(118)
        self._nav_items = (
            ("R", "nav_robot"),
            ("E", "nav_excitation"),
            ("C", "nav_collect"),
            ("I", "nav_identify"),
            ("X", "nav_export"),
            ("N", "nav_runs"),
        )
        for prefix, key in self._nav_items:
            item = QListWidgetItem(f"{prefix}   {tr(self.language, key)}")
            item.setData(Qt.ItemDataRole.UserRole, (prefix, key))
            item.setSizeHint(QSize(102, 52))
            self.nav.addItem(item)
        self.nav.setCurrentRow(0)
        body.addWidget(self.nav)

        center_wrap = QWidget()
        center_wrap.setObjectName("MainCanvas")
        center_layout = QVBoxLayout(center_wrap)
        center_layout.setContentsMargins(10, 10, 10, 8)
        center_layout.setSpacing(8)
        self.stack = QStackedWidget()
        self.stack.setMinimumWidth(0)
        self.stack.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.robot_page = RobotPage()
        self.excitation_page = ExcitationPage()
        self.collect_page = CollectPage()
        self.identify_page = IdentifyPage()
        self.export_page = ExportPage()
        self.runs_page = RunsPage()
        for page in (
            self.robot_page,
            self.excitation_page,
            self.collect_page,
            self.identify_page,
            self.export_page,
            self.runs_page,
        ):
            page.setMinimumWidth(0)
            page.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Expanding)
            self.stack.addWidget(page)
        self.workflow_scroll = QScrollArea()
        self.workflow_scroll.setObjectName("WorkflowScroll")
        self.workflow_scroll.setWidgetResizable(True)
        self.workflow_scroll.setFrameShape(QFrame.Shape.NoFrame)
        self.workflow_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.workflow_scroll.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        self.workflow_scroll.setWidget(self.stack)
        center_layout.addWidget(self.workflow_scroll, 1)
        center_layout.addWidget(self._build_artifact_bar())
        body.addWidget(center_wrap, 1)
        body.addWidget(self._build_summary(), 0)
        root.addLayout(body, 1)
        self.setCentralWidget(central)

    def _build_topbar(self) -> QFrame:
        frame = QFrame()
        frame.setObjectName("TopBar")
        frame.setFrameShape(QFrame.Shape.StyledPanel)
        layout = QHBoxLayout(frame)
        layout.setContentsMargins(12, 8, 12, 8)
        layout.setSpacing(8)
        mark = LogoMark()
        brand = QWidget()
        brand.setObjectName("BrandBlock")
        brand.setMaximumWidth(168)
        brand_layout = QVBoxLayout(brand)
        brand_layout.setContentsMargins(0, 0, 0, 0)
        brand_layout.setSpacing(0)
        self.brand_title = QLabel()
        self.brand_title.setObjectName("BrandTitle")
        self.brand_subtitle = QLabel()
        self.brand_subtitle.setObjectName("BrandSubtitle")
        brand_layout.addWidget(self.brand_title)
        brand_layout.addWidget(self.brand_subtitle)
        self.config_field = QLineEdit()
        self.config_field.setPlaceholderText("config/sia_example.yaml")
        self.config_field.setMinimumWidth(220)
        self.active_run_field = QLineEdit()
        self.active_run_field.setReadOnly(True)
        self.active_run_field.setFixedWidth(140)
        self.config_caption = QLabel()
        self.config_caption.setObjectName("TopbarCaption")
        self.active_run_caption = QLabel()
        self.active_run_caption.setObjectName("TopbarCaption")
        self.language_caption = QLabel()
        self.language_caption.setObjectName("TopbarCaption")
        self.browse_config_button = QPushButton()
        self.browse_config_button.clicked.connect(self._browse_config)
        self.load_config_button = QPushButton()
        self.load_config_button.clicked.connect(self._load_config_from_toolbar)
        self.save_config_button = QPushButton()
        self.save_config_button.clicked.connect(self._save_config)
        self.ros_status = QLabel()
        self.ros_status.setObjectName("StatusDotPending")
        self.hardware_status = QLabel()
        self.hardware_status.setObjectName("StatusDotIdle")
        self.open_run_button = QPushButton()
        self.open_run_button.clicked.connect(self._open_active_run)
        self.language_combo = QComboBox()
        self.language_combo.setObjectName("LanguageCombo")
        for code, name in LANGUAGES:
            self.language_combo.addItem(name, code)
        index = self.language_combo.findData(self.language)
        self.language_combo.setCurrentIndex(max(index, 0))
        layout.addWidget(mark)
        layout.addWidget(brand)
        layout.addSpacing(8)
        layout.addWidget(self.config_caption)
        layout.addWidget(self.config_field, 1)
        layout.addWidget(self.browse_config_button)
        layout.addWidget(self.load_config_button)
        layout.addSpacing(8)
        layout.addWidget(self.active_run_caption)
        layout.addWidget(self.active_run_field)
        layout.addSpacing(8)
        layout.addWidget(self.ros_status)
        layout.addWidget(self.hardware_status)
        layout.addSpacing(8)
        layout.addWidget(self.language_caption)
        layout.addWidget(self.language_combo)
        layout.addWidget(self.save_config_button)
        layout.addWidget(self.open_run_button)
        return frame

    def _build_summary(self) -> QFrame:
        frame = QFrame()
        frame.setObjectName("SummaryPanel")
        frame.setFrameShape(QFrame.Shape.StyledPanel)
        frame.setFixedWidth(300)
        layout = QVBoxLayout(frame)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(8)
        self.summary_title = QLabel()
        self.summary_title.setObjectName("SectionTitle")
        self.summary_labels: dict[str, QLabel] = {}
        layout.addWidget(self.summary_title)
        grid = QGridLayout()
        grid.setHorizontalSpacing(10)
        grid.setVerticalSpacing(8)
        self.summary_name_labels: dict[str, QLabel] = {}
        for row, key in enumerate(("urdf", "trajectory", "validation", "recording", "identify", "codegen")):
            label = QLabel()
            label.setObjectName("Muted")
            badge = QLabel("Pending")
            badge.setAlignment(Qt.AlignmentFlag.AlignCenter)
            badge.setObjectName("StatusDotPending")
            self.summary_name_labels[key] = label
            self.summary_labels[key] = badge
            grid.addWidget(label, row, 0)
            grid.addWidget(badge, row, 1)
        layout.addLayout(grid)
        self.summary_labels["run"] = QLabel("No active run")
        self.summary_labels["run"].setWordWrap(True)
        self.summary_labels["run"].setObjectName("Muted")
        layout.addWidget(self.summary_labels["run"])
        export_row = QHBoxLayout()
        self.summary_name_labels["export"] = QLabel()
        export_row.addWidget(self.summary_name_labels["export"])
        export_row.addStretch(1)
        self.summary_labels["export"] = QLabel("Pending")
        self.summary_labels["export"].setObjectName("StatusDotPending")
        export_row.addWidget(self.summary_labels["export"])
        layout.addLayout(export_row)
        self.log_title = QLabel()
        layout.addWidget(self.log_title)
        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumBlockCount(1500)
        self.log_view.setMinimumHeight(120)
        layout.addWidget(self.log_view, 1)
        self.stop_button = QPushButton()
        self.stop_button.setObjectName("DangerButton")
        self.stop_button.clicked.connect(self._stop_active_process)
        layout.addWidget(self.stop_button)
        return frame

    def _build_artifact_bar(self) -> QFrame:
        frame = QFrame()
        frame.setObjectName("ArtifactBar")
        frame.setFrameShape(QFrame.Shape.StyledPanel)
        layout = QHBoxLayout(frame)
        layout.setContentsMargins(12, 8, 12, 8)
        layout.setSpacing(12)
        self.artifacts_title = QLabel()
        layout.addWidget(self.artifacts_title)
        self.artifact_run = QLabel()
        self.artifact_run.setObjectName("Muted")
        self.artifact_trajectory = QLabel("excitation.csv")
        self.artifact_trajectory.setObjectName("Muted")
        self.artifact_report = QLabel("excitation_report.json")
        self.artifact_report.setObjectName("Muted")
        layout.addWidget(self.artifact_run, 1)
        layout.addWidget(self.artifact_trajectory)
        layout.addWidget(self.artifact_report)
        return frame

    def _connect(self) -> None:
        self.nav.currentRowChanged.connect(self.stack.setCurrentIndex)
        self.language_combo.currentIndexChanged.connect(self._change_language)
        self.robot_page.load_requested.connect(self._load_config_from_robot_page)
        self.robot_page.save_requested.connect(self._save_config)
        self.robot_page.browse_config_requested.connect(self._browse_config)
        self.robot_page.browse_urdf_requested.connect(self._browse_urdf)
        self.excitation_page.generate_requested.connect(self._generate_excitation)
        self.excitation_page.validate_requested.connect(lambda: self._validate_excitation(require_collision=False))
        self.excitation_page.collision_check_requested.connect(lambda: self._validate_excitation(require_collision=True))
        self.excitation_page.trajectory_preview_requested.connect(self._preview_trajectory)
        self.collect_page.browse_trajectory_requested.connect(self._browse_collection_trajectory)
        self.collect_page.collect_requested.connect(self._collect_with_trajectory)
        self.collect_page.stop_requested.connect(self._stop_active_process)
        self.identify_page.identify_requested.connect(self._identify_codegen)
        self.identify_page.browse_manifest_requested.connect(self._browse_manifest)
        self.export_page.export_requested.connect(self._export_runtime)
        self.runs_page.run_selected.connect(self._on_run_selected)
        self.runner.started.connect(self._on_process_started)
        self.runner.output.connect(self._append_log)
        self.runner.finished.connect(self._on_process_finished)

    def _workspace_path(self, path: str | Path) -> Path:
        value = Path(path).expanduser()
        return value if value.is_absolute() else self.workspace / value

    def _config_path(self) -> Path:
        raw = self.config_field.text().strip() or self.robot_page.config_path.text().strip()
        if not raw:
            raise ValueError("Load or choose a config file first.")
        return self._workspace_path(raw)

    def _load_config_from_toolbar(self) -> None:
        self._load_config(self._config_path())

    def _load_config_from_robot_page(self) -> None:
        self._load_config(self._workspace_path(self.robot_page.config_path.text().strip()))

    def _load_config(self, path: str | Path) -> None:
        try:
            self.model.load(path)
        except Exception as exc:  # noqa: BLE001
            QMessageBox.critical(self, self._t("load_config_failed"), str(exc))
            return
        self.config_field.setText(self.model.config_path_text())
        self.robot_page.load_from_model(self.model)
        self.excitation_page.load_from_model(self.model)
        self.collect_page.load_from_model(self.model)
        self.identify_page.load_from_model(self.model)
        self.export_page.load_from_model(self.model)
        self.runs_page.refresh(str(self._workspace_path(self.model.robot_summary().output_root)))
        self._update_summary_from_model()
        self._append_log(f"Loaded config: {self.model.config_path_text()}\n")

    def _apply_pages_to_model(self) -> None:
        if not self.model.is_loaded():
            self.model.data = {}
        self.robot_page.apply_to_model(self.model)
        self.excitation_page.apply_to_model(self.model)
        self.collect_page.apply_to_model(self.model)
        self.identify_page.apply_to_model(self.model)
        self.export_page.apply_to_model(self.model)

    def _save_config(self) -> None:
        try:
            self._apply_pages_to_model()
            target = self.model.save(self._config_path())
        except Exception as exc:  # noqa: BLE001
            QMessageBox.critical(self, self._t("save_config_failed"), str(exc))
            return
        self._load_config(target)

    def _save_before_command(self) -> Path | None:
        try:
            self._apply_pages_to_model()
            return self.model.save(self._config_path())
        except Exception as exc:  # noqa: BLE001
            QMessageBox.critical(self, self._t("config_error"), str(exc))
            return None

    def _browse_config(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self,
            self._t("open_config_title"),
            str(self.workspace),
            "Config files (*.yaml *.yml *.json)",
        )
        if path:
            self.config_field.setText(path)
            self.robot_page.config_path.setText(path)

    def _browse_urdf(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, self._t("open_urdf_title"), str(self.workspace), "URDF files (*.urdf *.xml)")
        if path:
            self.robot_page.urdf_path.setText(path)
            self.robot_page.refresh_limits()

    def _browse_manifest(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self,
            self._t("open_manifest_title"),
            str(self.workspace),
            "Manifest files (*.yaml *.json)",
        )
        if path:
            self.identify_page.manifest.setText(path)

    def _browse_collection_trajectory(self) -> None:
        current = self.collect_page.trajectory_path()
        directory = self._workspace_path(current).parent if current else self.workspace
        path, _ = QFileDialog.getOpenFileName(
            self,
            self._t("open_trajectory_title"),
            str(directory),
            "Trajectory CSV files (*.csv)",
        )
        if path:
            self.collect_page.set_trajectory_path(path)

    def _generate_excitation(self) -> None:
        config = self._save_before_command()
        if config is None:
            return
        trajectory = self.excitation_page.trajectory_path()
        self._last_trajectory = self._workspace_path(trajectory)
        self.collect_page.set_trajectory_path(trajectory)
        spec = command_builder.generate_excitation(config, trajectory)
        self.runner.run(spec)

    def _validate_excitation(self, *, require_collision: bool) -> None:
        config = self._save_before_command()
        if config is None:
            return
        trajectory = self.excitation_page.trajectory_path()
        self._last_trajectory = self._workspace_path(trajectory)
        spec = command_builder.validate_excitation(
            config,
            trajectory,
            require_collision=require_collision or self.excitation_page.require_collision.isChecked(),
        )
        self.runner.run(spec)

    def _preview_trajectory(self) -> None:
        config = self._save_before_command()
        if config is None:
            return
        trajectory = self.excitation_page.trajectory_path()
        self._last_trajectory = self._workspace_path(trajectory)
        spec = command_builder.preview_trajectory(
            config,
            trajectory,
        )
        self.runner.run(spec)

    def _stop_active_process(self) -> None:
        self._cancel_active_trajectory()
        self.runner.stop()

    def _cancel_active_trajectory(self) -> None:
        action_name = self.robot_page.action_name.text().strip()
        if not action_name:
            return
        spec = command_builder.cancel_trajectory(action_name, timeout_sec=1.5)
        self._append_log(f"$ {spec.display()}\n")
        result = QProcess.startDetached(spec.program, list(spec.args), str(self.workspace))
        started = result[0] if isinstance(result, tuple) else bool(result)
        if not started:
            self._append_log("Failed to start trajectory cancel command.\n")

    def _collect_with_trajectory(self) -> None:
        config = self._save_before_command()
        if config is None:
            return
        trajectory = self.collect_page.trajectory_path() or self.excitation_page.trajectory_path()
        self._last_trajectory = self._workspace_path(trajectory)
        spec = command_builder.collect_with_trajectory(config, trajectory=trajectory)
        self.runner.run(spec)

    def _identify_codegen(self) -> None:
        config = self._save_before_command()
        if config is None:
            return
        manifest = self.identify_page.manifest_path()
        spec = command_builder.identify_codegen(config, manifest=manifest, export_code=self.identify_page.export_code.isChecked())
        self.runner.run(spec)

    def _export_runtime(self) -> None:
        config = self._save_before_command()
        if config is None:
            return
        if not self.export_page.run_dir.text().strip() or not self.export_page.target_root.text().strip():
            QMessageBox.warning(self, self._t("export_runtime_title"), self._t("export_runtime_missing"))
            return
        run_dir = self.export_page.run_dir_path()
        target_root = self.export_page.target_root_path()
        self.runner.run(command_builder.export_runtime(config, run_dir=run_dir, target_root=target_root))

    def _on_process_started(self, record: ProcessRecord) -> None:
        self.summary_labels["run"].setText(f"Running: {record.label}")
        self.active_run_field.setText(record.label)

    def _on_process_finished(self, record: ProcessRecord) -> None:
        if record.exit_code not in (0, None) or record.failed_to_start:
            self.summary_labels["run"].setText(f"Failed: {record.label}")
            return
        self.summary_labels["run"].setText(f"Done: {record.label}")
        self.active_run_field.setText(record.label)
        if record.label == "Generate excitation" and self._last_trajectory is not None and self._last_trajectory.exists():
            self.excitation_page.show_trajectory(self._last_trajectory)
            self.collect_page.set_trajectory_path(self.excitation_page.trajectory_path())
            self._set_status_key("trajectory", "status_ready", "ok")
            self.artifact_trajectory.setText(str(self._last_trajectory.name))
        if record.label == "Validate excitation" and self._last_trajectory is not None:
            report = self._last_trajectory.with_name("excitation_validation.json")
            self._set_status_key("validation", "status_ok" if report.exists() else "status_done", "ok")
            self.artifact_report.setText(str(report.name))
        output_root = self.model.robot_summary().output_root if self.model.is_loaded() else "runs"
        if record.label == "Collect dataset":
            self._select_latest_collected_run(output_root)
        if record.label == "Identify and codegen":
            self._select_identified_run()
        self.runs_page.refresh(str(self._workspace_path(output_root)))

    def _select_latest_collected_run(self, output_root: str) -> None:
        for run in scan_runs(self._workspace_path(output_root)):
            if run.manifest_path is not None:
                self._on_run_selected(run)
                return

    def _select_identified_run(self) -> None:
        manifest = self.identify_page.manifest_path()
        if manifest is not None and manifest.exists():
            self._on_run_selected(inspect_run(manifest.parent))

    def _on_run_selected(self, run: RunArtifacts) -> None:
        self._active_run = run.run_dir
        self.summary_labels["run"].setText(str(run.run_dir))
        self.active_run_field.setText(run.run_dir.name)
        self._set_status("recording", f"{run.sample_count} samples" if run.sample_count else self._t("status_idle"), "ok" if run.sample_count else "idle")
        self._set_status_key("identify", "status_done" if run.has_identification else "status_pending", "ok" if run.has_identification else "pending")
        self._set_status_key("codegen", "status_done" if run.has_codegen else "status_pending", "ok" if run.has_codegen else "pending")
        self.artifact_run.setText(str(run.run_dir))
        if run.manifest_path is not None:
            self.identify_page.manifest.setText(str(run.manifest_path))
            try:
                manifest = read_manifest(run.manifest_path)
                motion_csv = str(manifest.get("data", {}).get("motion_csv", ""))
            except Exception:  # noqa: BLE001
                motion_csv = ""
            if motion_csv:
                self.collect_page.preview_motion_csv(motion_csv)
        if run.identify_dir is not None:
            self.export_page.run_dir.setText(str(run.identify_dir))
        self.identify_page.show_prediction_plot(run.prediction_plot_path)

    def _on_ros_state(self, state: RosGraphState) -> None:
        if not state.available:
            self.ros_status.setText(self._t("ros_unavailable"))
            self.ros_status.setObjectName("StatusDotError")
            self._refresh_style(self.ros_status)
            return
        self.ros_status.setText(f"ROS: {len(state.topics)} topics")
        self.ros_status.setObjectName("StatusDotOk")
        self._refresh_style(self.ros_status)
        action_name = self.robot_page.action_name.text().strip()
        if action_name and state.has_action(action_name):
            self.hardware_status.setText(self._t("controller_ready"))
            self.hardware_status.setObjectName("StatusDotOk")
        else:
            self.hardware_status.setText(self._t("controller_missing"))
            self.hardware_status.setObjectName("StatusDotIdle")
        self._refresh_style(self.hardware_status)

    def _update_summary_from_model(self) -> None:
        summary = self.model.robot_summary()
        self._set_status_key("urdf", "status_loaded" if summary.urdf_path else "status_missing", "ok" if summary.urdf_path else "error")
        self._set_status_key("trajectory", "status_configured" if self.excitation_page.output.text().strip() else "status_pending", "idle")
        self._set_status_key("validation", "status_pending", "pending")
        self._set_status_key("recording", "status_idle", "idle")
        self._set_status_key("identify", "status_pending", "pending")
        self._set_status_key("codegen", "status_pending", "pending")
        self._set_status_key("export", "status_pending", "pending")
        self.summary_labels["run"].setText(self._t("no_active_run"))
        self.active_run_field.clear()
        self.artifact_run.setText(summary.output_root)
        self.artifact_trajectory.setText(Path(self.excitation_page.output.text().strip() or "excitation.csv").name)

    def _t(self, key: str) -> str:
        return tr(self.language, key)

    def _change_language(self, _index: int = -1) -> None:
        self.language = normalize_language(self.language_combo.currentData())
        self._apply_language()

    def _apply_language(self) -> None:
        self.setWindowTitle(self._t("app_title"))
        self.brand_title.setText(self._t("brand_title"))
        self.brand_subtitle.setText(self._t("brand_subtitle"))
        self.config_caption.setText(self._t("config"))
        self.active_run_caption.setText(self._t("active_run"))
        self.language_caption.setText(self._t("language"))
        self.active_run_field.setPlaceholderText(self._t("active_run"))
        self.browse_config_button.setText(self._t("open"))
        self.load_config_button.setText(self._t("load"))
        self.save_config_button.setText(self._t("save_config"))
        self.open_run_button.setText(self._t("open_run_dir"))
        set_tooltip(
            self._t("config_tip"),
            self.config_caption,
            self.config_field,
            self.browse_config_button,
            self.load_config_button,
            self.save_config_button,
        )
        set_tooltip(self._t("active_run_tip"), self.active_run_caption, self.active_run_field, self.open_run_button)
        set_tooltip(self._t("language_tip"), self.language_caption, self.language_combo)
        if not self.ros_status.text() or self.ros_status.text() in {tr("en", "ros_unknown"), tr("zh", "ros_unknown")}:
            self.ros_status.setText(self._t("ros_unknown"))
        if not self.hardware_status.text() or self.hardware_status.text() in {tr("en", "hardware_unknown"), tr("zh", "hardware_unknown")}:
            self.hardware_status.setText(self._t("hardware_unknown"))
        for row in range(self.nav.count()):
            item = self.nav.item(row)
            prefix, key = item.data(Qt.ItemDataRole.UserRole)
            item.setText(f"{prefix}   {self._t(key)}")
        self.summary_title.setText(self._t("run_summary"))
        for key, label in self.summary_name_labels.items():
            label.setText(self._t(f"summary_{key}"))
        self.log_title.setText(self._t("log"))
        self.stop_button.setText(self._t("stop_process"))
        set_tooltip(self._t("stop_process_tip"), self.stop_button)
        self.artifacts_title.setText(f"{self._t('artifacts')}:")
        if self.artifact_run.text() in {"", tr("en", "no_run_selected"), tr("zh", "no_run_selected")}:
            self.artifact_run.setText(self._t("no_run_selected"))
        if self.summary_labels["run"].text() in {tr("en", "no_active_run"), tr("zh", "no_active_run"), ""}:
            self.summary_labels["run"].setText(self._t("no_active_run"))
        for key, text_key in self._summary_status_keys.items():
            if key in self.summary_labels:
                self.summary_labels[key].setText(self._t(text_key))
        for key in ("urdf", "trajectory", "validation", "recording", "identify", "codegen", "export"):
            label = self.summary_labels[key]
            if key not in self._summary_status_keys and label.text() in {"", tr("en", "status_pending"), tr("zh", "status_pending")}:
                label.setText(self._t("status_pending"))
        for page in (
            self.robot_page,
            self.excitation_page,
            self.collect_page,
            self.identify_page,
            self.export_page,
            self.runs_page,
        ):
            page.set_language(self.language)

    def _set_status(self, key: str, text: str, state: str) -> None:
        label = self.summary_labels[key]
        label.setText(text)
        object_name = {
            "ok": "StatusDotOk",
            "idle": "StatusDotIdle",
            "error": "StatusDotError",
        }.get(state, "StatusDotPending")
        label.setObjectName(object_name)
        self._refresh_style(label)

    def _set_status_key(self, key: str, text_key: str, state: str) -> None:
        self._summary_status_keys[key] = text_key
        self._set_status(key, self._t(text_key), state)

    def _refresh_style(self, widget: QWidget) -> None:
        widget.style().unpolish(widget)
        widget.style().polish(widget)

    def _append_log(self, text: str) -> None:
        self.log_view.moveCursor(QTextCursor.MoveOperation.End)
        self.log_view.insertPlainText(text)
        self.log_view.moveCursor(QTextCursor.MoveOperation.End)

    def _open_active_run(self) -> None:
        path = self._active_run
        if path is None and self.model.is_loaded():
            path = self._workspace_path(self.model.robot_summary().output_root)
        if path is not None:
            QDesktopServices.openUrl(QUrl.fromLocalFile(str(path)))

    def closeEvent(self, event) -> None:  # noqa: ANN001
        self.runner.stop()
        if self.monitor is not None:
            self.monitor.stop()
        super().closeEvent(event)
