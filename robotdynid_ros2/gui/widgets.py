"""Shared GUI widgets."""

from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtGui import QColor, QLinearGradient, QPainter, QPen
from PySide6.QtWidgets import QFrame, QLabel, QVBoxLayout, QWidget


class LogoMark(QWidget):
    """Compact vector mark used in the top bar."""

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setFixedSize(44, 44)

    def paintEvent(self, event) -> None:  # noqa: ANN001
        del event
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        rect = self.rect().adjusted(2, 2, -2, -2)
        gradient = QLinearGradient(rect.topLeft(), rect.bottomRight())
        gradient.setColorAt(0.0, QColor("#2563eb"))
        gradient.setColorAt(0.55, QColor("#0891b2"))
        gradient.setColorAt(1.0, QColor("#10b981"))
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(gradient)
        painter.drawRoundedRect(rect, 12, 12)

        painter.setBrush(Qt.BrushStyle.NoBrush)
        painter.setPen(QPen(QColor(255, 255, 255, 205), 2.2))
        painter.drawEllipse(12, 11, 20, 20)
        painter.drawArc(8, 16, 28, 14, 20 * 16, 145 * 16)
        painter.drawArc(8, 16, 28, 14, 200 * 16, 120 * 16)

        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QColor("#ffffff"))
        painter.drawEllipse(20, 19, 5, 5)
        painter.setBrush(QColor("#cffafe"))
        painter.drawEllipse(30, 10, 5, 5)


class PageHeader(QWidget):
    """Consistent page title block."""

    def __init__(self, title: str, subtitle: str = "", parent: QWidget | None = None) -> None:
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 2)
        layout.setSpacing(3)
        self.title = QLabel(title)
        self.title.setObjectName("PageTitle")
        self.subtitle = QLabel(subtitle)
        self.subtitle.setObjectName("PageSubtitle")
        self.subtitle.setWordWrap(True)
        layout.addWidget(self.title)
        layout.addWidget(self.subtitle)

    def set_text(self, title: str, subtitle: str) -> None:
        self.title.setText(title)
        self.subtitle.setText(subtitle)


class PanelTitle:
    def __init__(self, title: QLabel, subtitle: QLabel | None) -> None:
        self.title = title
        self.subtitle = subtitle

    def set_text(self, title: str, subtitle: str = "") -> None:
        self.title.setText(title)
        if self.subtitle is not None:
            self.subtitle.setText(subtitle)
            self.subtitle.setVisible(bool(subtitle))


def make_panel(title: str, subtitle: str = "") -> tuple[QFrame, QVBoxLayout, PanelTitle]:
    frame = QFrame()
    frame.setObjectName("Panel")
    frame.setFrameShape(QFrame.Shape.StyledPanel)
    layout = QVBoxLayout(frame)
    layout.setContentsMargins(16, 16, 16, 16)
    layout.setSpacing(11)
    title_label = QLabel(title)
    title_label.setObjectName("SectionTitle")
    subtitle_label = QLabel(subtitle)
    subtitle_label.setObjectName("PanelSubtitle")
    subtitle_label.setWordWrap(True)
    subtitle_label.setVisible(bool(subtitle))
    layout.addWidget(title_label)
    layout.addWidget(subtitle_label)
    return frame, layout, PanelTitle(title_label, subtitle_label)


def set_tooltip(text: str, *widgets: QWidget) -> None:
    """Apply the same explanatory tooltip to a field label and its editor."""

    for widget in widgets:
        widget.setToolTip(text)
