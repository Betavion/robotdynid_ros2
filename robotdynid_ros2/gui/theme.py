"""Qt stylesheet helpers for the GUI."""

from __future__ import annotations

import tempfile
from pathlib import Path


def _chevron_down_icon() -> str:
    path = Path(tempfile.gettempdir()) / "robotdynid_ros2_chevron_down.svg"
    if not path.exists():
        path.write_text(
            """<svg xmlns="http://www.w3.org/2000/svg" width="16" height="16" viewBox="0 0 16 16">
<path d="M4 6l4 4 4-4" fill="none" stroke="#64748b" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round"/>
</svg>
""",
            encoding="utf-8",
        )
    return path.as_posix()


def stylesheet() -> str:
    chevron_down = _chevron_down_icon()
    return """
    QWidget {
        background: #eef3f8;
        color: #172033;
        font-size: 13px;
    }
    QLabel {
        background: transparent;
    }
    QMainWindow, QDialog {
        background: #eef3f8;
    }
    QWidget#MainCanvas, QScrollArea#WorkflowScroll, QScrollArea#WorkflowScroll > QWidget > QWidget {
        background: #f4f7fb;
    }
    QWidget#BrandBlock {
        background: transparent;
    }
    QFrame#TopBar {
        background: #ffffff;
        border: none;
        border-bottom: 1px solid #d7dfe9;
        border-radius: 0px;
    }
    QFrame#SummaryPanel {
        background: #ffffff;
        border: none;
        border-left: 1px solid #d7dfe9;
    }
    QFrame#ArtifactBar {
        background: #ffffff;
        border: 1px solid #dbe5f1;
        border-radius: 8px;
    }
    QFrame#Panel {
        background: #ffffff;
        border: 1px solid #cfdae8;
        border-radius: 8px;
    }
    QFrame#FlatPanel {
        background: #ffffff;
        border: 1px solid #dbe5f1;
        border-radius: 8px;
    }
    QLabel#PageTitle {
        font-size: 21px;
        font-weight: 700;
        color: #101828;
    }
    QLabel#PageSubtitle, QLabel#PanelSubtitle {
        color: #667085;
        line-height: 1.25;
    }
    QLabel#SectionTitle {
        font-size: 15px;
        font-weight: 700;
        color: #101828;
    }
    QLabel#Muted {
        color: #667085;
    }
    QLabel#BrandTitle {
        font-size: 19px;
        font-weight: 800;
        color: #101828;
    }
    QLabel#BrandSubtitle {
        color: #667085;
        font-size: 12px;
    }
    QLabel#TopbarCaption {
        color: #667085;
        font-weight: 600;
    }
    QLabel#StatusDotOk {
        background: #dff7ea;
        color: #047857;
        border: 1px solid #a7f3d0;
        border-radius: 11px;
        padding: 3px 8px;
        font-weight: 600;
    }
    QLabel#StatusDotIdle {
        background: #e7f5ff;
        color: #0369a1;
        border: 1px solid #bae6fd;
        border-radius: 11px;
        padding: 3px 8px;
        font-weight: 600;
    }
    QLabel#StatusDotPending {
        background: #f5f7fb;
        color: #475467;
        border: 1px solid #d7dee9;
        border-radius: 11px;
        padding: 3px 8px;
        font-weight: 600;
    }
    QLabel#StatusDotError {
        background: #fef3f2;
        color: #b42318;
        border: 1px solid #fecdca;
        border-radius: 11px;
        padding: 3px 8px;
        font-weight: 600;
    }
    QLabel#Chip {
        background: #f0f9ff;
        color: #075985;
        border: 1px solid #bae6fd;
        border-radius: 6px;
        padding: 3px 7px;
    }
    QPushButton {
        background: #ffffff;
        border: 1px solid #cbd7e6;
        border-radius: 6px;
        padding: 6px 11px;
        min-width: 54px;
        color: #1d4ed8;
        font-weight: 600;
    }
    QPushButton:hover {
        background: #f8fbff;
        border-color: #2563eb;
    }
    QPushButton:pressed {
        background: #e0efff;
    }
    QPushButton#PrimaryButton {
        background: #2563eb;
        color: #ffffff;
        border-color: #2563eb;
        font-weight: 600;
    }
    QPushButton#PrimaryButton:hover {
        background: #1d4ed8;
        border-color: #1d4ed8;
    }
    QPushButton#DangerButton {
        background: #ffffff;
        color: #b42318;
        border-color: #f4b7ae;
    }
    QToolButton#PlotToolButton {
        background: #ffffff;
        border: 1px solid #d6e0ec;
        border-radius: 5px;
        padding: 4px 7px;
        color: #315075;
        font-size: 12px;
        font-weight: 600;
    }
    QToolButton#PlotToolButton:hover {
        background: #f7fbff;
        border-color: #93b9f2;
        color: #1d4ed8;
    }
    QToolButton#PlotToolButton:checked {
        background: #e0efff;
        border-color: #93b9f2;
        color: #1d4ed8;
    }
    QListWidget#NavRail, QListWidget#NavRail::viewport {
        background: #0f172a;
        border: none;
        outline: none;
        padding-top: 10px;
    }
    QListWidget#NavRail::item {
        padding: 11px 9px;
        border-radius: 6px;
        margin: 4px 8px;
        color: #cbd5e1;
    }
    QListWidget#NavRail::item:hover {
        background: #172554;
        color: #ffffff;
    }
    QListWidget#NavRail::item:selected {
        background: #1d4ed8;
        color: #ffffff;
        border-left: 3px solid #67e8f9;
        font-weight: 700;
    }
    QLineEdit, QComboBox, QSpinBox, QDoubleSpinBox {
        background: #ffffff;
        border: 1px solid #cbd7e6;
        border-radius: 6px;
        padding: 6px 9px;
        min-height: 20px;
        selection-background-color: #d1e9ff;
    }
    QComboBox {
        padding-right: 28px;
    }
    QComboBox::drop-down {
        subcontrol-origin: padding;
        subcontrol-position: top right;
        width: 28px;
        border: none;
        background: transparent;
    }
    QComboBox::down-arrow {
        image: url("__CHEVRON_DOWN__");
        width: 16px;
        height: 16px;
        margin-right: 10px;
    }
    QComboBox::down-arrow:on {
        top: 1px;
    }
    QComboBox QAbstractItemView {
        background: #ffffff;
        border: 1px solid #cbd7e6;
        border-radius: 6px;
        padding: 4px;
        outline: none;
        selection-background-color: #e0efff;
        selection-color: #1d4ed8;
    }
    QComboBox QAbstractItemView::item {
        min-height: 24px;
        padding: 5px 8px;
        border-radius: 5px;
    }
    QComboBox QAbstractItemView::item:hover {
        background: #f0f7ff;
    }
    QCheckBox {
        background: transparent;
        color: #172033;
        spacing: 7px;
    }
    QCheckBox::indicator {
        width: 15px;
        height: 15px;
    }
    QLineEdit:focus, QComboBox:focus, QSpinBox:focus, QDoubleSpinBox:focus {
        border: 1px solid #2563eb;
    }
    QComboBox#LanguageCombo {
        min-width: 94px;
    }
    QAbstractSpinBox::up-button, QAbstractSpinBox::down-button {
        width: 0px;
        border: none;
    }
    QLineEdit:read-only {
        background: #f8fafc;
        color: #667085;
    }
    QTableWidget {
        background: #ffffff;
        border: 1px solid #dbe5f1;
        border-radius: 6px;
        gridline-color: #edf1f6;
    }
    QHeaderView::section {
        background: #f5f8fb;
        color: #344054;
        border: none;
        padding: 6px;
        font-weight: 600;
    }
    QTextEdit, QPlainTextEdit {
        background: #0f172a;
        color: #d9f3ff;
        border: 1px solid #172554;
        border-radius: 6px;
        font-family: monospace;
        font-size: 12px;
    }
    QTabWidget::pane {
        border: 1px solid #dbe5f1;
        border-radius: 6px;
        background: #ffffff;
        top: -1px;
    }
    QTabBar::tab {
        background: #ffffff;
        border: none;
        border-bottom: 2px solid transparent;
        padding: 8px 14px;
        color: #344054;
    }
    QTabBar::tab:selected {
        background: #ffffff;
        color: #2563eb;
        border-bottom: 2px solid #2563eb;
        font-weight: 600;
    }
    QScrollBar:vertical {
        background: transparent;
        width: 10px;
        margin: 2px;
    }
    QScrollBar::handle:vertical {
        background: #cbd5e1;
        border-radius: 5px;
        min-height: 32px;
    }
    QScrollBar::handle:vertical:hover {
        background: #94a3b8;
    }
    QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
        height: 0px;
        background: transparent;
        border: none;
    }
    QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical {
        background: transparent;
    }
    QScrollBar:horizontal {
        background: transparent;
        height: 10px;
        margin: 2px;
    }
    QScrollBar::handle:horizontal {
        background: #cbd5e1;
        border-radius: 5px;
        min-width: 32px;
    }
    QScrollBar::handle:horizontal:hover {
        background: #94a3b8;
    }
    QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal {
        width: 0px;
        background: transparent;
        border: none;
    }
    QScrollBar::add-page:horizontal, QScrollBar::sub-page:horizontal {
        background: transparent;
    }
    """.replace("__CHEVRON_DOWN__", chevron_down)
