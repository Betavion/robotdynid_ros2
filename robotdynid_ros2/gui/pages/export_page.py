"""Runtime export page."""

from __future__ import annotations

from pathlib import Path

from PySide6.QtCore import Signal
from PySide6.QtWidgets import QFormLayout, QLabel, QLineEdit, QPushButton, QVBoxLayout, QWidget

from robotdynid_ros2.gui.config_model import GuiConfigModel
from robotdynid_ros2.gui.i18n import tr
from robotdynid_ros2.gui.widgets import PageHeader, make_panel, set_tooltip


class ExportPage(QWidget):
    export_requested = Signal()

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
        self.namespace = QLineEdit()
        self.class_name = QLineEdit()
        self.run_dir_label = QLabel()
        self.target_root_label = QLabel()
        self.namespace_label = QLabel()
        self.class_name_label = QLabel()
        form.addRow(self.run_dir_label, self.run_dir)
        form.addRow(self.target_root_label, self.target_root)
        form.addRow(self.namespace_label, self.namespace)
        form.addRow(self.class_name_label, self.class_name)
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
        self.namespace.setText(str(values["namespace"]))
        self.class_name.setText(str(values["class_name"]))

    def apply_to_model(self, model: GuiConfigModel) -> None:
        model.set_value("export_runtime", "run_dir", self.run_dir.text().strip())
        model.set_value("export_runtime", "target_root", self.target_root.text().strip())
        model.set_value("export_runtime", "namespace", self.namespace.text().strip())
        model.set_value("export_runtime", "class_name", self.class_name.text().strip())

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
        self.namespace_label.setText(tr(language, "namespace"))
        self.class_name_label.setText(tr(language, "class_name"))
        self.export_button.setText(tr(language, "export_runtime"))
        self.summary.setText(tr(language, "export_summary"))
        self._set_field_tooltips(language)

    def _set_field_tooltips(self, language: str) -> None:
        set_tooltip(tr(language, "identify_dir_tip"), self.run_dir_label, self.run_dir)
        set_tooltip(tr(language, "target_root_tip"), self.target_root_label, self.target_root)
        set_tooltip(tr(language, "namespace_tip"), self.namespace_label, self.namespace)
        set_tooltip(tr(language, "class_name_tip"), self.class_name_label, self.class_name)
