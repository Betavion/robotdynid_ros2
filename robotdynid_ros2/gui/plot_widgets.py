"""Plot widgets for trajectory and dataset previews."""

from __future__ import annotations

import csv
from pathlib import Path

import numpy as np
from PySide6.QtCore import Qt
from PySide6.QtWidgets import QHBoxLayout, QLabel, QSizePolicy, QToolButton, QVBoxLayout, QWidget


PLOT_CANVAS_HEIGHT = 220
PLOT_WIDGET_HEIGHT = 258


def _finite_range(values: np.ndarray) -> tuple[float, float] | None:
    finite = values[np.isfinite(values)]
    if finite.size == 0:
        return None
    lower = float(np.min(finite))
    upper = float(np.max(finite))
    if lower == upper:
        delta = max(abs(lower) * 0.05, 0.5)
        return lower - delta, upper + delta
    return lower, upper


class _PlotShell(QWidget):
    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._x_range: tuple[float, float] | None = None
        self._y_range: tuple[float, float] | None = None
        self._plot = None
        self.setMinimumHeight(PLOT_WIDGET_HEIGHT)
        self.setMaximumHeight(PLOT_WIDGET_HEIGHT)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(5)
        tools = QHBoxLayout()
        tools.setContentsMargins(0, 0, 0, 0)
        tools.setSpacing(5)
        self.fit_all_button = self._tool_button("Fit All", "Fit both axes to all plotted data")
        self.fit_x_button = self._tool_button("Fit X", "Fit the horizontal axis")
        self.fit_y_button = self._tool_button("Fit Y", "Fit the vertical axis")
        self.zoom_x_button = self._tool_button("Zoom X", "Enable mouse zoom/pan on the horizontal axis", checkable=True)
        self.zoom_y_button = self._tool_button("Zoom Y", "Enable mouse zoom/pan on the vertical axis", checkable=True)
        self.zoom_x_button.setChecked(True)
        self.zoom_y_button.setChecked(True)
        for button in (self.fit_all_button, self.fit_x_button, self.fit_y_button, self.zoom_x_button, self.zoom_y_button):
            tools.addWidget(button)
        tools.addStretch(1)
        layout.addLayout(tools)

        self.fit_all_button.clicked.connect(self.fit_all)
        self.fit_x_button.clicked.connect(self.fit_x)
        self.fit_y_button.clicked.connect(self.fit_y)
        self.zoom_x_button.toggled.connect(self._update_mouse_mode)
        self.zoom_y_button.toggled.connect(self._update_mouse_mode)

        try:
            import pyqtgraph as pg
        except ModuleNotFoundError:
            label = QLabel("pyqtgraph is not installed.")
            label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            label.setMinimumHeight(PLOT_CANVAS_HEIGHT)
            label.setMaximumHeight(PLOT_CANVAS_HEIGHT)
            layout.addWidget(label)
        else:
            pg.setConfigOptions(antialias=True, background="#ffffff", foreground="#475467")
            self._plot = pg.PlotWidget()
            self._plot.setMinimumHeight(PLOT_CANVAS_HEIGHT)
            self._plot.setMaximumHeight(PLOT_CANVAS_HEIGHT)
            self._plot.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
            self._plot.setBackground("#ffffff")
            self._plot.showGrid(x=True, y=True, alpha=0.25)
            self._plot.addLegend(offset=(8, 8))
            self._plot.setMouseEnabled(x=True, y=True)
            for axis_name in ("bottom", "left"):
                axis = self._plot.getAxis(axis_name)
                if hasattr(axis, "enableAutoSIPrefix"):
                    axis.enableAutoSIPrefix(False)
            layout.addWidget(self._plot)

    def _tool_button(self, text: str, tooltip: str, *, checkable: bool = False) -> QToolButton:
        button = QToolButton()
        button.setObjectName("PlotToolButton")
        button.setText(text)
        button.setToolTip(tooltip)
        button.setCheckable(checkable)
        button.setAutoRaise(True)
        return button

    def _update_ranges(self, x: np.ndarray, y: np.ndarray) -> None:
        self._x_range = _finite_range(np.asarray(x, dtype=float).reshape(-1))
        self._y_range = _finite_range(np.asarray(y, dtype=float).reshape(-1))

    def _update_mouse_mode(self) -> None:
        if self._plot is not None:
            self._plot.setMouseEnabled(x=self.zoom_x_button.isChecked(), y=self.zoom_y_button.isChecked())

    def clear(self) -> None:
        if self._plot is not None:
            self._plot.clear()
            self._plot.addLegend(offset=(8, 8))

    def fit_all(self) -> None:
        if self._plot is None:
            return
        self._plot.enableAutoRange(axis="xy", enable=True)
        self._plot.autoRange(padding=0.03)

    def fit_x(self) -> None:
        if self._plot is not None and self._x_range is not None:
            self._plot.enableAutoRange(axis="x", enable=False)
            self._plot.setXRange(*self._x_range, padding=0.02)

    def fit_y(self) -> None:
        if self._plot is not None and self._y_range is not None:
            self._plot.enableAutoRange(axis="y", enable=False)
            self._plot.setYRange(*self._y_range, padding=0.05)


class TrajectoryPlot(_PlotShell):
    """Plot trajectory CSV position, velocity, and acceleration traces."""

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)

    def plot_trajectory(self, csv_path: str | Path, kind: str = "position", max_points: int = 2500) -> None:
        if self._plot is None:
            return
        from robotdynid_ros2.trajectory.schema import TIME_UNIT, TRAJECTORY_UNITS, read_trajectory_csv

        data = read_trajectory_csv(csv_path)
        values = {
            "position": data.position,
            "velocity": data.velocity,
            "acceleration": data.acceleration,
        }[kind]
        indices = np.arange(data.sample_count)
        if data.sample_count > max_points:
            indices = np.linspace(0, data.sample_count - 1, max_points, dtype=int)
        self.clear()
        self._update_ranges(data.time[indices], values[indices, :])
        colors = ["#2563eb", "#16a34a", "#dc2626", "#9333ea", "#f59e0b", "#0891b2", "#475569"]
        for joint_index, joint_name in enumerate(data.joint_names):
            self._plot.plot(
                data.time[indices],
                values[indices, joint_index],
                pen=colors[joint_index % len(colors)],
                name=joint_name,
            )
        self._plot.setLabel("bottom", "time", units=TIME_UNIT)
        self._plot.setLabel("left", kind, units=TRAJECTORY_UNITS[kind])
        self.fit_all()


class CsvPreviewPlot(_PlotShell):
    """Lightweight CSV preview plot used by collection and result pages."""

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)

    def plot_columns(self, path: str | Path, columns: list[str], *, x_column: str = "timestamp", max_points: int = 2500) -> None:
        if self._plot is None:
            return
        csv_path = Path(path).expanduser()
        if not csv_path.exists():
            return
        with csv_path.open("r", encoding="utf-8", newline="") as handle:
            reader = csv.DictReader(handle)
            rows = list(reader)
        if not rows:
            return
        x = np.asarray([float(row.get(x_column, index)) for index, row in enumerate(rows)], dtype=float)
        indices = np.arange(len(rows))
        if len(rows) > max_points:
            indices = np.linspace(0, len(rows) - 1, max_points, dtype=int)
        self._plot.clear()
        self._plot.addLegend(offset=(8, 8))
        colors = ["#2563eb", "#16a34a", "#dc2626", "#9333ea", "#f59e0b", "#0891b2"]
        plotted_values = []
        for column_index, column in enumerate(columns):
            try:
                y = np.asarray([float(row[column]) for row in rows], dtype=float)
            except (KeyError, ValueError):
                continue
            plotted_values.append(y[indices])
            self._plot.plot(x[indices], y[indices], pen=colors[column_index % len(colors)], name=column)
        self._plot.setLabel("bottom", x_column)
        if plotted_values:
            self._update_ranges(x[indices], np.vstack(plotted_values))
            self.fit_all()
