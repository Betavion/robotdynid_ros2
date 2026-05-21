"""Robot and config page."""

from __future__ import annotations

from pathlib import Path

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QFormLayout,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QLineEdit,
    QPushButton,
    QSpinBox,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from robotdynid_ros2.gui.config_model import GuiConfigModel
from robotdynid_ros2.gui.i18n import tr
from robotdynid_ros2.gui.widgets import PageHeader, make_panel, set_tooltip
from robotdynid_ros2.trajectory.urdf_limits import parse_urdf_joint_limits


class RobotPage(QWidget):
    load_requested = Signal()
    save_requested = Signal()
    browse_config_requested = Signal()
    browse_urdf_requested = Signal()

    def __init__(self) -> None:
        super().__init__()
        self._language = "en"
        self._resolved_urdf_path: Path | None = None
        layout = QVBoxLayout(self)
        self.header = PageHeader("", "")
        layout.addWidget(self.header)

        config_panel, config_layout, self.config_panel_title = make_panel("", "")
        self.config_path = QLineEdit()
        self.browse_config_button = QPushButton()
        self.browse_config_button.setMinimumWidth(78)
        self.browse_config_button.clicked.connect(self.browse_config_requested)
        self.load_button = QPushButton()
        self.load_button.setObjectName("PrimaryButton")
        self.load_button.clicked.connect(self.load_requested)
        self.save_button = QPushButton()
        self.save_button.clicked.connect(self.save_requested)
        row = QHBoxLayout()
        row.addWidget(self.config_path, 1)
        row.addWidget(self.browse_config_button)
        row.addWidget(self.load_button)
        row.addWidget(self.save_button)
        config_layout.addLayout(row)
        layout.addWidget(config_panel)

        robot_panel, robot_layout, self.robot_panel_title = make_panel("", "")
        form = QFormLayout()
        self.urdf_path = QLineEdit()
        self.browse_urdf_button = QPushButton()
        self.browse_urdf_button.setMinimumWidth(78)
        self.browse_urdf_button.clicked.connect(self.browse_urdf_requested)
        urdf_row = QHBoxLayout()
        urdf_row.addWidget(self.urdf_path, 1)
        urdf_row.addWidget(self.browse_urdf_button)
        self.urdf_label = QLabel()
        form.addRow(self.urdf_label, urdf_row)
        self.output_root = QLineEdit("runs")
        self.dof = QSpinBox()
        self.dof.setRange(1, 128)
        self.joint_names = QLineEdit()
        self.output_root_label = QLabel()
        self.dof_label = QLabel()
        self.joint_names_label = QLabel()
        form.addRow(self.output_root_label, self.output_root)
        form.addRow(self.dof_label, self.dof)
        form.addRow(self.joint_names_label, self.joint_names)
        robot_layout.addLayout(form)
        layout.addWidget(robot_panel)

        topics_panel, topics_layout, self.topics_panel_title = make_panel("", "")
        topic_form = QFormLayout()
        self.joint_state_topic = QLineEdit("/joint_states")
        self.estimate_topic = QLineEdit()
        self.action_name = QLineEdit("/joint_trajectory_controller/follow_joint_trajectory")
        self.joint_state_topic.setReadOnly(True)
        self.estimate_topic.setReadOnly(True)
        self.joint_state_topic_label = QLabel()
        self.estimate_topic_label = QLabel()
        self.action_name_label = QLabel()
        topic_form.addRow(self.joint_state_topic_label, self.joint_state_topic)
        topic_form.addRow(self.estimate_topic_label, self.estimate_topic)
        topic_form.addRow(self.action_name_label, self.action_name)
        topics_layout.addLayout(topic_form)
        layout.addWidget(topics_panel)

        limits_panel, limits_layout, self.limits_panel_title = make_panel("", "")
        self.limits_table = QTableWidget(0, 6)
        self.limits_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.limits_table.horizontalHeader().setFixedHeight(30)
        self.limits_table.verticalHeader().setDefaultSectionSize(24)
        self.limits_table.setMinimumHeight(190)
        limits_layout.addWidget(self.limits_table)
        layout.addWidget(limits_panel, 1)
        self.set_language(self._language)

    def load_from_model(self, model: GuiConfigModel) -> None:
        summary = model.robot_summary()
        recorder = model.recorder_values()
        trajectory = model.trajectory_values()
        robot = model.data.get("robot", {})
        robot = robot if isinstance(robot, dict) else {}
        self._resolved_urdf_path = Path(summary.urdf_path).expanduser() if summary.urdf_path else None
        self.config_path.setText(model.config_path_text())
        self.urdf_path.setText(str(robot.get("urdf_path", summary.urdf_path)))
        self.output_root.setText(summary.output_root)
        self.dof.setValue(max(summary.dof, 1))
        self.joint_names.setText(", ".join(summary.joint_names))
        self.joint_state_topic.setText(str(recorder["joint_state_topic"]))
        self.estimate_topic.setText(str(recorder["estimate_joint_state_topic"]))
        self.action_name.setText(str(trajectory["action_name"]))
        self.refresh_limits()

    def apply_to_model(self, model: GuiConfigModel) -> None:
        joints = [part.strip() for part in self.joint_names.text().split(",") if part.strip()]
        model.set_value("robot", "urdf_path", self.urdf_path.text().strip())
        model.set_value("robot", "dof", self.dof.value())
        model.set_value("robot", "joint_names", joints)
        model.set_value("run", "output_root", self.output_root.text().strip() or "runs")
        model.set_value("trajectory", "action_name", self.action_name.text().strip())

    def refresh_limits(self) -> None:
        self.limits_table.setRowCount(0)
        path = self._current_urdf_path()
        joints = [part.strip() for part in self.joint_names.text().split(",") if part.strip()]
        if not path.exists() or not joints:
            return
        try:
            limits = parse_urdf_joint_limits(path, tuple(joints))
        except Exception:  # noqa: BLE001
            return
        self.limits_table.setRowCount(len(limits))
        for row, limit in enumerate(limits):
            values = [limit.name, limit.lower, limit.upper, limit.velocity, limit.acceleration, limit.effort]
            for column, value in enumerate(values):
                self.limits_table.setItem(row, column, QTableWidgetItem("" if value is None else str(value)))

    def _current_urdf_path(self) -> Path:
        raw = self.urdf_path.text().strip()
        if not raw:
            return Path()
        path = Path(raw).expanduser()
        if path.is_absolute():
            return path
        if self._resolved_urdf_path is not None and self._resolved_urdf_path.exists():
            return self._resolved_urdf_path
        return path

    def set_language(self, language: str) -> None:
        self._language = language
        self.header.set_text(tr(language, "robot_title"), tr(language, "robot_subtitle"))
        self.config_panel_title.set_text(tr(language, "project_config"), tr(language, "project_config_subtitle"))
        self.robot_panel_title.set_text(tr(language, "robot"), tr(language, "robot_subpanel"))
        self.topics_panel_title.set_text(tr(language, "ros_interfaces"), tr(language, "ros_interfaces_subtitle"))
        self.limits_panel_title.set_text(tr(language, "joint_limits"), tr(language, "joint_limits_subtitle"))
        self.browse_config_button.setText(tr(language, "browse"))
        self.browse_urdf_button.setText(tr(language, "browse"))
        self.load_button.setText(tr(language, "load"))
        self.save_button.setText(tr(language, "save"))
        self.urdf_label.setText(tr(language, "urdf"))
        self.output_root_label.setText(tr(language, "output_root"))
        self.dof_label.setText(tr(language, "dof"))
        self.joint_names_label.setText(tr(language, "joint_order"))
        self.joint_state_topic_label.setText(tr(language, "joint_state_topic"))
        self.estimate_topic_label.setText(tr(language, "estimate_topic"))
        self.action_name_label.setText(tr(language, "trajectory_action"))
        self.limits_table.setHorizontalHeaderLabels(["joint", "lower", "upper", "velocity", "acceleration", "effort"])
        self._set_field_tooltips(language)

    def _set_field_tooltips(self, language: str) -> None:
        set_tooltip(tr(language, "config_tip"), self.config_path, self.browse_config_button, self.load_button, self.save_button)
        set_tooltip(tr(language, "urdf_tip"), self.urdf_label, self.urdf_path, self.browse_urdf_button)
        set_tooltip(tr(language, "output_root_tip"), self.output_root_label, self.output_root)
        set_tooltip(tr(language, "dof_tip"), self.dof_label, self.dof)
        set_tooltip(tr(language, "joint_order_tip"), self.joint_names_label, self.joint_names)
        set_tooltip(tr(language, "joint_state_topic_tip"), self.joint_state_topic_label, self.joint_state_topic)
        set_tooltip(tr(language, "estimate_topic_tip"), self.estimate_topic_label, self.estimate_topic)
        set_tooltip(tr(language, "trajectory_action_tip"), self.action_name_label, self.action_name)
