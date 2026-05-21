"""Identification and codegen page."""

from __future__ import annotations

from pathlib import Path

from PySide6.QtCore import Signal
from PySide6.QtGui import QPixmap
from PySide6.QtWidgets import QCheckBox, QFormLayout, QHBoxLayout, QLabel, QLineEdit, QPushButton, QSpinBox, QVBoxLayout, QWidget

from robotdynid_ros2.gui.config_model import GuiConfigModel
from robotdynid_ros2.gui.i18n import tr
from robotdynid_ros2.gui.widgets import PageHeader, make_panel, set_tooltip


class IdentifyPage(QWidget):
    identify_requested = Signal()
    browse_manifest_requested = Signal()

    def __init__(self) -> None:
        super().__init__()
        self._language = "en"
        layout = QVBoxLayout(self)
        self.header = PageHeader("", "")
        layout.addWidget(self.header)

        panel, panel_layout, self.identification_panel_title = make_panel("", "")
        form = QFormLayout()
        self.manifest = QLineEdit()
        self.browse_button = QPushButton()
        self.browse_button.clicked.connect(self.browse_manifest_requested)
        manifest_row = QHBoxLayout()
        manifest_row.addWidget(self.manifest, 1)
        manifest_row.addWidget(self.browse_button)
        self.stride = QSpinBox()
        self.stride.setRange(1, 100000)
        self.max_samples = QSpinBox()
        self.max_samples.setRange(1, 1000000)
        self.selection_samples = QSpinBox()
        self.selection_samples.setRange(1, 1000000)
        self.chunk_size = QSpinBox()
        self.chunk_size.setRange(0, 1000000)
        self.export_code = QCheckBox()
        self.manifest_label = QLabel()
        self.stride_label = QLabel()
        self.max_samples_label = QLabel()
        self.selection_samples_label = QLabel()
        self.chunk_size_label = QLabel()
        form.addRow(self.manifest_label, manifest_row)
        form.addRow(self.stride_label, self.stride)
        form.addRow(self.max_samples_label, self.max_samples)
        form.addRow(self.selection_samples_label, self.selection_samples)
        form.addRow(self.chunk_size_label, self.chunk_size)
        form.addRow("", self.export_code)
        panel_layout.addLayout(form)
        self.run_button = QPushButton()
        self.run_button.setObjectName("PrimaryButton")
        self.run_button.clicked.connect(self.identify_requested)
        panel_layout.addWidget(self.run_button)
        layout.addWidget(panel)

        result_panel, result_layout, self.result_panel_title = make_panel("", "")
        self.result_summary = QLabel()
        self.result_summary.setObjectName("Muted")
        self.prediction_image = QLabel()
        self.prediction_image.setMinimumHeight(260)
        self.prediction_image.setScaledContents(False)
        result_layout.addWidget(self.result_summary)
        result_layout.addWidget(self.prediction_image, 1)
        layout.addWidget(result_panel, 1)
        self.set_language(self._language)

    def load_from_model(self, model: GuiConfigModel) -> None:
        values = model.identification_values()
        self.manifest.clear()
        self.stride.setValue(max(1, int(values["stride"])))
        self.max_samples.setValue(max(1, int(values["max_samples"])))
        self.selection_samples.setValue(max(1, int(values["selection_samples"])))
        self.chunk_size.setValue(max(0, int(values["chunk_size"])))
        self.export_code.setChecked(bool(values["export_code"]))

    def apply_to_model(self, model: GuiConfigModel) -> None:
        model.ensure_section("identification").pop("manifest", None)
        model.set_value("identification", "stride", self.stride.value())
        model.set_value("identification", "max_samples", self.max_samples.value())
        model.set_value("identification", "selection_samples", self.selection_samples.value())
        model.set_value("identification", "chunk_size", self.chunk_size.value())
        model.set_value("codegen", "export_code", self.export_code.isChecked())

    def manifest_path(self) -> Path | None:
        raw = self.manifest.text().strip()
        return Path(raw).expanduser() if raw else None

    def show_prediction_plot(self, path: str | Path | None) -> None:
        if path is None:
            self.prediction_image.clear()
            self.result_summary.setText(tr(self._language, "no_result_loaded"))
            return
        image_path = Path(path).expanduser()
        if not image_path.exists():
            self.prediction_image.clear()
            self.result_summary.setText(tr(self._language, "no_result_loaded"))
            return
        pixmap = QPixmap(str(image_path))
        self.prediction_image.setPixmap(pixmap.scaledToWidth(760))
        self.result_summary.setText(str(image_path))

    def set_language(self, language: str) -> None:
        self._language = language
        self.header.set_text(tr(language, "identify_title"), tr(language, "identify_subtitle"))
        self.identification_panel_title.set_text(tr(language, "identification"), tr(language, "identification_subtitle"))
        self.result_panel_title.set_text(tr(language, "prediction_plot"), tr(language, "prediction_plot_subtitle"))
        self.browse_button.setText(tr(language, "browse"))
        self.manifest_label.setText(tr(language, "manifest"))
        self.stride_label.setText(tr(language, "stride"))
        self.max_samples_label.setText(tr(language, "max_samples"))
        self.selection_samples_label.setText(tr(language, "selection_samples"))
        self.chunk_size_label.setText(tr(language, "chunk_size"))
        self.export_code.setText(tr(language, "export_code"))
        self.run_button.setText(tr(language, "run_identification"))
        self._set_field_tooltips(language)
        if self.result_summary.text() in {"", tr("en", "no_result_loaded"), tr("zh", "no_result_loaded")}:
            self.result_summary.setText(tr(language, "no_result_loaded"))

    def _set_field_tooltips(self, language: str) -> None:
        set_tooltip(tr(language, "manifest_tip"), self.manifest_label, self.manifest, self.browse_button)
        set_tooltip(tr(language, "stride_tip"), self.stride_label, self.stride)
        set_tooltip(tr(language, "max_samples_tip"), self.max_samples_label, self.max_samples)
        set_tooltip(tr(language, "selection_samples_tip"), self.selection_samples_label, self.selection_samples)
        set_tooltip(tr(language, "chunk_size_tip"), self.chunk_size_label, self.chunk_size)
        set_tooltip(tr(language, "export_code_tip"), self.export_code)
