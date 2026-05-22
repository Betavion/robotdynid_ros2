"""Identification and codegen page."""

from __future__ import annotations

from pathlib import Path

from PySide6.QtCore import Signal
from PySide6.QtGui import QPixmap
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from robotdynid_ros2.gui.config_model import GuiConfigModel
from robotdynid_ros2.gui.i18n import tr
from robotdynid_ros2.gui.widgets import PageHeader, make_panel, set_tooltip


def _csv_to_list(raw: str) -> list[float]:
    return [float(part.strip()) for part in raw.split(",") if part.strip()]


def _set_combo(combo: QComboBox, value: str) -> None:
    index = combo.findText(value)
    combo.setCurrentIndex(index if index >= 0 else 0)


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
        self.torque_weighting = QComboBox()
        self.torque_weighting.addItems(["torque_std", "none"])
        self.measurement_torque_std = QLineEdit()
        self.linear_regularization_strength = QDoubleSpinBox()
        self.linear_regularization_strength.setRange(0.0, 1.0e9)
        self.linear_regularization_strength.setDecimals(6)
        self.linear_regularization_strength.setSingleStep(0.001)
        self.linear_regularization_prior_source = QComboBox()
        self.linear_regularization_prior_source.addItems(["zero", "urdf"])
        self.linear_regularization_prior_std = QLineEdit()
        self.robust_loss = QComboBox()
        self.robust_loss.addItems(["linear", "soft_l1", "huber", "cauchy", "arctan"])
        self.robust_f_scale = QDoubleSpinBox()
        self.robust_f_scale.setRange(1.0e-9, 1.0e9)
        self.robust_f_scale.setDecimals(6)
        self.robust_f_scale.setSingleStep(0.1)
        self.robust_max_iterations = QSpinBox()
        self.robust_max_iterations.setRange(1, 100)
        self.export_code = QCheckBox()
        self.manifest_label = QLabel()
        self.stride_label = QLabel()
        self.max_samples_label = QLabel()
        self.selection_samples_label = QLabel()
        self.chunk_size_label = QLabel()
        self.torque_weighting_label = QLabel()
        self.measurement_torque_std_label = QLabel()
        self.linear_regularization_strength_label = QLabel()
        self.linear_regularization_prior_source_label = QLabel()
        self.linear_regularization_prior_std_label = QLabel()
        self.robust_loss_label = QLabel()
        self.robust_f_scale_label = QLabel()
        self.robust_max_iterations_label = QLabel()
        form.addRow(self.manifest_label, manifest_row)
        form.addRow(self.stride_label, self.stride)
        form.addRow(self.max_samples_label, self.max_samples)
        form.addRow(self.selection_samples_label, self.selection_samples)
        form.addRow(self.chunk_size_label, self.chunk_size)
        form.addRow(self.torque_weighting_label, self.torque_weighting)
        form.addRow(self.measurement_torque_std_label, self.measurement_torque_std)
        form.addRow(self.linear_regularization_strength_label, self.linear_regularization_strength)
        form.addRow(self.linear_regularization_prior_source_label, self.linear_regularization_prior_source)
        form.addRow(self.linear_regularization_prior_std_label, self.linear_regularization_prior_std)
        form.addRow(self.robust_loss_label, self.robust_loss)
        form.addRow(self.robust_f_scale_label, self.robust_f_scale)
        form.addRow(self.robust_max_iterations_label, self.robust_max_iterations)
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
        _set_combo(self.torque_weighting, str(values["torque_weighting"]))
        self.measurement_torque_std.setText(str(values["measurement_torque_std"]))
        self.linear_regularization_strength.setValue(max(0.0, float(values["linear_regularization_strength"])))
        _set_combo(self.linear_regularization_prior_source, str(values["linear_regularization_prior_source"]))
        self.linear_regularization_prior_std.setText(str(values["linear_regularization_prior_std"]))
        _set_combo(self.robust_loss, str(values["robust_loss"]))
        self.robust_f_scale.setValue(max(1.0e-9, float(values["robust_f_scale"])))
        self.robust_max_iterations.setValue(max(1, int(values["robust_max_iterations"])))
        self.export_code.setChecked(bool(values["export_code"]))

    def apply_to_model(self, model: GuiConfigModel) -> None:
        model.ensure_section("identification").pop("manifest", None)
        model.set_value("identification", "stride", self.stride.value())
        model.set_value("identification", "max_samples", self.max_samples.value())
        model.set_value("identification", "selection_samples", self.selection_samples.value())
        model.set_value("identification", "chunk_size", self.chunk_size.value())
        model.set_value("identification", "torque_weighting", self.torque_weighting.currentText())
        model.set_value("identification", "measurement_torque_std", _csv_to_list(self.measurement_torque_std.text()))
        model.set_value(
            "identification",
            "linear_regularization_strength",
            self.linear_regularization_strength.value(),
        )
        model.set_value(
            "identification",
            "linear_regularization_prior_source",
            self.linear_regularization_prior_source.currentText(),
        )
        model.set_value(
            "identification",
            "linear_regularization_prior_std",
            _csv_to_list(self.linear_regularization_prior_std.text()),
        )
        model.set_value("identification", "robust_loss", self.robust_loss.currentText())
        model.set_value("identification", "robust_f_scale", self.robust_f_scale.value())
        model.set_value("identification", "robust_max_iterations", self.robust_max_iterations.value())
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
        self.torque_weighting_label.setText(tr(language, "torque_weighting"))
        self.measurement_torque_std_label.setText(tr(language, "measurement_torque_std"))
        self.linear_regularization_strength_label.setText(tr(language, "linear_regularization_strength"))
        self.linear_regularization_prior_source_label.setText(tr(language, "linear_regularization_prior_source"))
        self.linear_regularization_prior_std_label.setText(tr(language, "linear_regularization_prior_std"))
        self.robust_loss_label.setText(tr(language, "robust_loss"))
        self.robust_f_scale_label.setText(tr(language, "robust_f_scale"))
        self.robust_max_iterations_label.setText(tr(language, "robust_max_iterations"))
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
        set_tooltip(tr(language, "torque_weighting_tip"), self.torque_weighting_label, self.torque_weighting)
        set_tooltip(tr(language, "measurement_torque_std_tip"), self.measurement_torque_std_label, self.measurement_torque_std)
        set_tooltip(
            tr(language, "linear_regularization_strength_tip"),
            self.linear_regularization_strength_label,
            self.linear_regularization_strength,
        )
        set_tooltip(
            tr(language, "linear_regularization_prior_source_tip"),
            self.linear_regularization_prior_source_label,
            self.linear_regularization_prior_source,
        )
        set_tooltip(
            tr(language, "linear_regularization_prior_std_tip"),
            self.linear_regularization_prior_std_label,
            self.linear_regularization_prior_std,
        )
        set_tooltip(tr(language, "robust_loss_tip"), self.robust_loss_label, self.robust_loss)
        set_tooltip(tr(language, "robust_f_scale_tip"), self.robust_f_scale_label, self.robust_f_scale)
        set_tooltip(
            tr(language, "robust_max_iterations_tip"),
            self.robust_max_iterations_label,
            self.robust_max_iterations,
        )
        set_tooltip(tr(language, "export_code_tip"), self.export_code)
