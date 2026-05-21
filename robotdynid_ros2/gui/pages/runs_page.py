"""Historical runs page."""

from __future__ import annotations

from PySide6.QtCore import Signal
from PySide6.QtWidgets import QListWidget, QListWidgetItem, QPushButton, QVBoxLayout, QWidget

from robotdynid_ros2.gui.i18n import tr
from robotdynid_ros2.gui.artifact_model import RunArtifacts, scan_runs
from robotdynid_ros2.gui.widgets import PageHeader, make_panel


class RunsPage(QWidget):
    run_selected = Signal(object)

    def __init__(self) -> None:
        super().__init__()
        self._runs: list[RunArtifacts] = []
        self._language = "en"
        layout = QVBoxLayout(self)
        self.header = PageHeader("", "")
        layout.addWidget(self.header)
        panel, panel_layout, self.history_panel_title = make_panel("", "")
        self.refresh_button = QPushButton()
        self.refresh_button.clicked.connect(lambda: self.refresh(self._output_root))
        self._output_root = "runs"
        self.list_widget = QListWidget()
        self.list_widget.currentRowChanged.connect(self._on_row_changed)
        panel_layout.addWidget(self.refresh_button)
        panel_layout.addWidget(self.list_widget)
        layout.addWidget(panel, 1)
        self.set_language(self._language)

    def refresh(self, output_root: str) -> None:
        self._output_root = output_root or "runs"
        self._runs = scan_runs(self._output_root)
        self.list_widget.clear()
        for run in self._runs:
            labels = ", ".join(run.status_labels()) or "empty"
            rmse = f" | rmse {sum(run.rmse) / len(run.rmse):.4g}" if run.rmse else ""
            item = QListWidgetItem(f"{run.run_dir.name} [{labels}] samples {run.sample_count}{rmse}")
            self.list_widget.addItem(item)

    def _on_row_changed(self, row: int) -> None:
        if 0 <= row < len(self._runs):
            self.run_selected.emit(self._runs[row])

    def set_language(self, language: str) -> None:
        self._language = language
        self.header.set_text(tr(language, "runs_title"), tr(language, "runs_subtitle"))
        self.history_panel_title.set_text(tr(language, "history"), tr(language, "history_subtitle"))
        self.refresh_button.setText(tr(language, "refresh"))
