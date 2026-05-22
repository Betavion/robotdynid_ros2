"""Runtime export page."""

from __future__ import annotations

from pathlib import Path

from PySide6.QtCore import Signal
from PySide6.QtWidgets import QFormLayout, QHBoxLayout, QLabel, QLineEdit, QPushButton, QVBoxLayout, QWidget

from robotdynid_ros2.gui.config_model import GuiConfigModel
from robotdynid_ros2.gui.i18n import tr
from robotdynid_ros2.gui.widgets import PageHeader, make_panel, set_tooltip


class ExportPage(QWidget):
    export_requested = Signal()
    browse_run_dir_requested = Signal()
    browse_target_root_requested = Signal()

    def __init__(self) -> None:
        super().__init__()
        self._language = "en"
        layout = QVBoxLayout(self)
        self.header = PageHeader("", "")
        layout.addWidget(self.header)

        panel, panel_layout, self.runtime_panel_title = make_panel("", "")
        form = QFormLayout()
        self.run_dir = QLineEdit()
        self.target_root = QLineEdit()
        self.browse_run_dir_button = QPushButton()
        self.browse_target_root_button = QPushButton()
        self.browse_run_dir_button.setMinimumWidth(90)
        self.browse_target_root_button.setMinimumWidth(90)
        self.browse_run_dir_button.clicked.connect(self.browse_run_dir_requested)
        self.browse_target_root_button.clicked.connect(self.browse_target_root_requested)
        run_dir_row = QWidget()
        run_dir_layout = QHBoxLayout(run_dir_row)
        run_dir_layout.setContentsMargins(0, 0, 0, 0)
        run_dir_layout.setSpacing(8)
        run_dir_layout.addWidget(self.run_dir, 1)
        run_dir_layout.addWidget(self.browse_run_dir_button)
        target_root_row = QWidget()
        target_root_layout = QHBoxLayout(target_root_row)
        target_root_layout.setContentsMargins(0, 0, 0, 0)
        target_root_layout.setSpacing(8)
        target_root_layout.addWidget(self.target_root, 1)
        target_root_layout.addWidget(self.browse_target_root_button)
        self.run_dir_label = QLabel()
        self.target_root_label = QLabel()
        form.addRow(self.run_dir_label, run_dir_row)
        form.addRow(self.target_root_label, target_root_row)
        panel_layout.addLayout(form)
        self.export_button = QPushButton()
        self.export_button.setObjectName("PrimaryButton")
        self.export_button.clicked.connect(self.export_requested)
        panel_layout.addWidget(self.export_button)
        self.summary = QLabel()
        self.summary.setObjectName("Muted")
        panel_layout.addWidget(self.summary)
        layout.addWidget(panel)
        layout.addStretch(1)
        self.set_language(self._language)

    def load_from_model(self, model: GuiConfigModel) -> None:
        values = model.export_values()
        self.run_dir.setText(str(values["run_dir"]))
        self.target_root.setText(str(values["target_root"]))

    def apply_to_model(self, model: GuiConfigModel) -> None:
        model.set_value("export_runtime", "run_dir", self.run_dir.text().strip())
        model.set_value("export_runtime", "target_root", self.target_root.text().strip())

    def run_dir_path(self) -> Path:
        return Path(self.run_dir.text().strip()).expanduser()

    def target_root_path(self) -> Path:
        return Path(self.target_root.text().strip()).expanduser()

    def set_language(self, language: str) -> None:
        self._language = language
        self.header.set_text(tr(language, "export_title"), tr(language, "export_subtitle"))
        self.runtime_panel_title.set_text(tr(language, "runtime_kernel"), tr(language, "runtime_kernel_subtitle"))
        self.run_dir_label.setText(tr(language, "identify_dir"))
        self.target_root_label.setText(tr(language, "target_root"))
        self.browse_run_dir_button.setText(tr(language, "browse"))
        self.browse_target_root_button.setText(tr(language, "browse"))
        self.export_button.setText(tr(language, "export_runtime"))
        self.summary.setText(tr(language, "export_summary"))
        self._set_field_tooltips(language)

    def _set_field_tooltips(self, language: str) -> None:
        set_tooltip(tr(language, "identify_dir_tip"), self.run_dir_label, self.run_dir, self.browse_run_dir_button)
        set_tooltip(tr(language, "target_root_tip"), self.target_root_label, self.target_root, self.browse_target_root_button)
