"""Collection workflow page."""

from __future__ import annotations

import csv
from pathlib import Path

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QDoubleSpinBox,
    QFormLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)

from robotdynid_ros2.gui.config_model import GuiConfigModel
from robotdynid_ros2.gui.i18n import tr
from robotdynid_ros2.gui.plot_widgets import CsvPreviewPlot
from robotdynid_ros2.gui.widgets import PageHeader, make_panel, set_tooltip


class CollectPage(QWidget):
    browse_trajectory_requested = Signal()
    collect_requested = Signal()
    stop_requested = Signal()

    def __init__(self) -> None:
        super().__init__()
        self._language = "en"
        layout = QVBoxLayout(self)
        self.header = PageHeader("", "")
        layout.addWidget(self.header)

        panel, panel_layout, self.recording_panel_title = make_panel("", "")
        form = QFormLayout()
        self.trajectory_csv = QLineEdit()
        self.browse_trajectory_button = QPushButton()
        self.browse_trajectory_button.setMinimumWidth(90)
        self.browse_trajectory_button.clicked.connect(self.browse_trajectory_requested)
        trajectory_row = QWidget()
        trajectory_layout = QHBoxLayout(trajectory_row)
        trajectory_layout.setContentsMargins(0, 0, 0, 0)
        trajectory_layout.setSpacing(8)
        trajectory_layout.addWidget(self.trajectory_csv, 1)
        trajectory_layout.addWidget(self.browse_trajectory_button)
        self.joint_state_topic = QLineEdit()
        self.estimate_topic = QLineEdit()
        self.duration = QDoubleSpinBox()
        self.duration.setRange(0.0, 7200.0)
        self.duration.setDecimals(2)
        self.flush_period = QDoubleSpinBox()
        self.flush_period.setRange(0.001, 5.0)
        self.flush_period.setDecimals(4)
        self.output_root = QLineEdit("runs")
        self.output_root.setReadOnly(True)
        self.trajectory_csv_label = QLabel()
        self.joint_state_topic_label = QLabel()
        self.estimate_topic_label = QLabel()
        self.duration_label = QLabel()
        self.flush_period_label = QLabel()
        self.output_root_label = QLabel()
        form.addRow(self.trajectory_csv_label, trajectory_row)
        form.addRow(self.joint_state_topic_label, self.joint_state_topic)
        form.addRow(self.estimate_topic_label, self.estimate_topic)
        form.addRow(self.duration_label, self.duration)
        form.addRow(self.flush_period_label, self.flush_period)
        form.addRow(self.output_root_label, self.output_root)
        panel_layout.addLayout(form)
        row = QHBoxLayout()
        self.collect_button = QPushButton()
        self.collect_button.setObjectName("PrimaryButton")
        self.stop_button = QPushButton()
        self.stop_button.setObjectName("DangerButton")
        self.collect_button.clicked.connect(self.collect_requested)
        self.stop_button.clicked.connect(self.stop_requested)
        row.addWidget(self.collect_button)
        row.addWidget(self.stop_button)
        row.addStretch(1)
        panel_layout.addLayout(row)
        layout.addWidget(panel)

        preview_panel, preview_layout, self.preview_panel_title = make_panel("", "")
        preview_panel.setMaximumHeight(390)
        preview_panel.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.preview_path = QLineEdit()
        self.preview_path.setPlaceholderText("motion.csv path")
        self.preview = CsvPreviewPlot()
        preview_layout.addWidget(self.preview_path)
        preview_layout.addWidget(self.preview)
        layout.addWidget(preview_panel)
        layout.addStretch(1)
        self.set_language(self._language)

    def load_from_model(self, model: GuiConfigModel) -> None:
        values = model.recorder_values()
        self.trajectory_csv.setText(str(values["commanded_trajectory_csv"]))
        self.joint_state_topic.setText(str(values["joint_state_topic"]))
        self.estimate_topic.setText(str(values["estimate_joint_state_topic"]))
        self.duration.setValue(float(values["duration_sec"]))
        self.flush_period.setValue(float(values["flush_period_sec"]))
        self.output_root.setText(str(values["output_root"]))

    def apply_to_model(self, model: GuiConfigModel) -> None:
        model.set_value("recording", "commanded_trajectory_csv", self.trajectory_csv.text().strip())
        model.set_value("recording", "joint_state_topic", self.joint_state_topic.text().strip())
        model.set_value("recording", "estimate_joint_state_topic", self.estimate_topic.text().strip())
        model.set_value("recording", "duration_sec", self.duration.value())
        model.set_value("recording", "flush_period_sec", self.flush_period.value())

    def set_trajectory_path(self, path: str | Path) -> None:
        self.trajectory_csv.setText(str(path))

    def trajectory_path(self) -> Path | None:
        raw = self.trajectory_csv.text().strip()
        return Path(raw) if raw else None

    def preview_motion_csv(self, path: str) -> None:
        self.preview_path.setText(path)
        self.preview.plot_columns(path, _preview_columns(path))

    def set_language(self, language: str) -> None:
        self._language = language
        self.header.set_text(tr(language, "collect_title"), tr(language, "collect_subtitle"))
        self.recording_panel_title.set_text(tr(language, "recording"), tr(language, "recording_subtitle"))
        self.preview_panel_title.set_text(tr(language, "recorded_preview"), tr(language, "recorded_preview_subtitle"))
        self.trajectory_csv_label.setText(tr(language, "commanded_trajectory"))
        self.browse_trajectory_button.setText(tr(language, "browse"))
        self.joint_state_topic_label.setText(tr(language, "joint_state_topic"))
        self.estimate_topic_label.setText(tr(language, "estimate_topic"))
        self.duration_label.setText(tr(language, "duration"))
        self.flush_period_label.setText(tr(language, "flush_period"))
        self.output_root_label.setText(tr(language, "output_root"))
        self.collect_button.setText(tr(language, "send_record"))
        self.stop_button.setText(tr(language, "stop_process"))
        self._set_field_tooltips(language)

    def _set_field_tooltips(self, language: str) -> None:
        set_tooltip(
            tr(language, "commanded_trajectory_tip"),
            self.trajectory_csv_label,
            self.trajectory_csv,
            self.browse_trajectory_button,
        )
        set_tooltip(tr(language, "joint_state_topic_tip"), self.joint_state_topic_label, self.joint_state_topic)
        set_tooltip(tr(language, "estimate_topic_tip"), self.estimate_topic_label, self.estimate_topic)
        set_tooltip(tr(language, "recording_duration_tip"), self.duration_label, self.duration)
        set_tooltip(tr(language, "flush_period_tip"), self.flush_period_label, self.flush_period)
        set_tooltip(tr(language, "output_root_tip"), self.output_root_label, self.output_root)
        set_tooltip(tr(language, "stop_process_tip"), self.stop_button)


def _preview_columns(path: str | Path) -> list[str]:
    csv_path = Path(path).expanduser()
    if not csv_path.exists():
        return []
    with csv_path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        fieldnames = reader.fieldnames or []
    position_columns = [name for name in fieldnames if name.endswith("_position")]
    if position_columns:
        return position_columns
    return [name for name in fieldnames if name != "timestamp"][:6]
