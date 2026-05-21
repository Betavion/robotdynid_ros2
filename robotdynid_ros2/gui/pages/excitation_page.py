"""Excitation generation and validation page."""

from __future__ import annotations

from pathlib import Path

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QSizePolicy,
    QSpinBox,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from robotdynid_ros2.gui.config_model import GuiConfigModel
from robotdynid_ros2.gui.i18n import tr
from robotdynid_ros2.gui.plot_widgets import TrajectoryPlot
from robotdynid_ros2.gui.widgets import PageHeader, make_panel, set_tooltip


def _format_vector(value: object) -> str:
    if not value:
        return ""
    if isinstance(value, str):
        return value
    return ", ".join(f"{float(item):.6g}" for item in value)  # type: ignore[arg-type]


def _parse_vector_text(value: str) -> list[float] | None:
    stripped = value.strip()
    if not stripped:
        return None
    return [float(part.strip()) for part in stripped.split(",") if part.strip()]


class ExcitationPage(QWidget):
    generate_requested = Signal()
    validate_requested = Signal()
    collision_check_requested = Signal()
    trajectory_preview_requested = Signal()

    def __init__(self) -> None:
        super().__init__()
        self._language = "en"
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(10)
        self.header = PageHeader("", "")
        layout.addWidget(self.header)

        top_row = QHBoxLayout()
        top_row.setSpacing(10)
        robot_panel, robot_layout, self.robot_panel_title = make_panel("", "")
        self.robot_urdf = QLineEdit()
        self.robot_urdf.setReadOnly(True)
        self.robot_dof = QLabel("DOF 0")
        self.robot_dof.setObjectName("StatusDotPending")
        self.robot_joints = QLabel()
        self.robot_joints.setObjectName("Chip")
        self.robot_joints.setWordWrap(True)
        self.robot_urdf_label = QLabel()
        self.robot_joint_order_label = QLabel()
        robot_layout.addWidget(self.robot_urdf_label)
        robot_layout.addWidget(self.robot_urdf)
        robot_layout.addWidget(self.robot_dof)
        robot_layout.addWidget(self.robot_joint_order_label)
        robot_layout.addWidget(self.robot_joints)
        top_row.addWidget(robot_panel, 1)

        settings_panel, settings_layout, self.settings_panel_title = make_panel("", "")
        self.output = QLineEdit("runs/sia_excitation.csv")
        self.profile = QComboBox()
        self.profile.addItems(["composite", "safe_multisine", "friction_sweep", "gravity_sweep"])
        self.duration = QDoubleSpinBox()
        self.duration.setRange(0.1, 3600.0)
        self.duration.setDecimals(3)
        self.duration.setValue(60.0)
        self.sample_period = QDoubleSpinBox()
        self.sample_period.setRange(0.001, 10.0)
        self.sample_period.setDecimals(4)
        self.sample_period.setValue(0.01)
        self.harmonics = QSpinBox()
        self.harmonics.setRange(1, 32)
        self.search_candidates = QSpinBox()
        self.search_candidates.setRange(1, 256)
        self.friction_speed_levels = QSpinBox()
        self.friction_speed_levels.setRange(1, 12)
        self.gravity_pose_count = QSpinBox()
        self.gravity_pose_count.setRange(1, 20)
        self.velocity_scale = QDoubleSpinBox()
        self.velocity_scale.setRange(0.001, 1.0)
        self.velocity_scale.setDecimals(3)
        self.acceleration_scale = QDoubleSpinBox()
        self.acceleration_scale.setRange(0.001, 1.0)
        self.acceleration_scale.setDecimals(3)
        self.position_lower = QLineEdit()
        self.position_upper = QLineEdit()
        grid = QGridLayout()
        grid.setHorizontalSpacing(18)
        grid.setVerticalSpacing(10)
        self.output_label = QLabel()
        self.profile_label = QLabel()
        self.harmonics_label = QLabel()
        self.duration_label = QLabel()
        self.velocity_scale_label = QLabel()
        self.sample_period_label = QLabel()
        self.acceleration_scale_label = QLabel()
        self.search_candidates_label = QLabel()
        self.friction_speed_levels_label = QLabel()
        self.gravity_pose_count_label = QLabel()
        self.position_lower_label = QLabel()
        self.position_upper_label = QLabel()
        grid.addWidget(self.output_label, 0, 0)
        grid.addWidget(self.output, 0, 1, 1, 3)
        grid.addWidget(self.profile_label, 1, 0)
        grid.addWidget(self.profile, 1, 1)
        grid.addWidget(self.harmonics_label, 1, 2)
        grid.addWidget(self.harmonics, 1, 3)
        grid.addWidget(self.duration_label, 2, 0)
        grid.addWidget(self.duration, 2, 1)
        grid.addWidget(self.velocity_scale_label, 2, 2)
        grid.addWidget(self.velocity_scale, 2, 3)
        grid.addWidget(self.sample_period_label, 3, 0)
        grid.addWidget(self.sample_period, 3, 1)
        grid.addWidget(self.acceleration_scale_label, 3, 2)
        grid.addWidget(self.acceleration_scale, 3, 3)
        grid.addWidget(self.position_lower_label, 4, 0)
        grid.addWidget(self.position_lower, 4, 1)
        grid.addWidget(self.position_upper_label, 4, 2)
        grid.addWidget(self.position_upper, 4, 3)
        grid.addWidget(self.search_candidates_label, 5, 0)
        grid.addWidget(self.search_candidates, 5, 1)
        grid.addWidget(self.friction_speed_levels_label, 5, 2)
        grid.addWidget(self.friction_speed_levels, 5, 3)
        grid.addWidget(self.gravity_pose_count_label, 6, 0)
        grid.addWidget(self.gravity_pose_count, 6, 1)
        settings_layout.addLayout(grid)
        button_row = QHBoxLayout()
        self.generate_button = QPushButton()
        self.generate_button.setObjectName("PrimaryButton")
        self.validate_button = QPushButton()
        self.collision_check_button = QPushButton()
        self.trajectory_preview_button = QPushButton()
        self.generate_button.clicked.connect(self.generate_requested)
        self.validate_button.clicked.connect(self.validate_requested)
        self.collision_check_button.clicked.connect(self.collision_check_requested)
        self.trajectory_preview_button.clicked.connect(self.trajectory_preview_requested)
        for button in (
            self.generate_button,
            self.validate_button,
            self.collision_check_button,
            self.trajectory_preview_button,
        ):
            button_row.addWidget(button)
        button_row.addStretch(1)
        settings_layout.addLayout(button_row)
        top_row.addWidget(settings_panel, 2)
        layout.addLayout(top_row)

        safety_panel, safety_layout, self.safety_panel_title = make_panel("", "")
        self.validate_enabled = QCheckBox()
        self.collision_enabled = QCheckBox()
        self.require_collision = QCheckBox()
        self.torque_enabled = QCheckBox()
        self.moveit_group = QLineEdit("sia_arm")
        self.state_validity_service = QLineEdit("/check_state_validity")
        self.collision_sample_limit = QSpinBox()
        self.collision_sample_limit.setRange(1, 1000000)
        self.torque_sample_limit = QSpinBox()
        self.torque_sample_limit.setRange(1, 1000000)
        self.max_torque_scale = QDoubleSpinBox()
        self.max_torque_scale.setRange(0.001, 1000.0)
        self.max_torque_scale.setDecimals(3)
        safety_grid = QGridLayout()
        safety_grid.setHorizontalSpacing(18)
        safety_grid.setVerticalSpacing(8)
        safety_grid.addWidget(self.validate_enabled, 0, 0)
        safety_grid.addWidget(self.collision_enabled, 0, 1)
        safety_grid.addWidget(self.require_collision, 0, 2)
        safety_grid.addWidget(self.torque_enabled, 0, 3)
        self.moveit_group_label = QLabel()
        self.state_validity_service_label = QLabel()
        self.collision_sample_limit_label = QLabel()
        self.torque_sample_limit_label = QLabel()
        self.max_torque_scale_label = QLabel()
        safety_grid.addWidget(self.moveit_group_label, 1, 0)
        safety_grid.addWidget(self.moveit_group, 1, 1)
        safety_grid.addWidget(self.state_validity_service_label, 1, 2)
        safety_grid.addWidget(self.state_validity_service, 1, 3, 1, 2)
        safety_grid.addWidget(self.collision_sample_limit_label, 2, 0)
        safety_grid.addWidget(self.collision_sample_limit, 2, 1)
        safety_grid.addWidget(self.torque_sample_limit_label, 2, 2)
        safety_grid.addWidget(self.torque_sample_limit, 2, 3)
        safety_grid.addWidget(self.max_torque_scale_label, 2, 4)
        safety_grid.addWidget(self.max_torque_scale, 2, 5)
        safety_layout.addLayout(safety_grid)
        layout.addWidget(safety_panel)

        preview_panel, preview_layout, self.preview_panel_title = make_panel("", "")
        preview_panel.setMaximumHeight(355)
        preview_panel.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.tabs = QTabWidget()
        self.tabs.setMaximumHeight(300)
        self.tabs.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.position_plot = TrajectoryPlot()
        self.velocity_plot = TrajectoryPlot()
        self.acceleration_plot = TrajectoryPlot()
        self.tabs.addTab(self.position_plot, "")
        self.tabs.addTab(self.velocity_plot, "")
        self.tabs.addTab(self.acceleration_plot, "")
        preview_layout.addWidget(self.tabs)
        layout.addWidget(preview_panel)
        self.set_language(self._language)

    def load_from_model(self, model: GuiConfigModel) -> None:
        summary = model.robot_summary()
        self.robot_urdf.setText(summary.urdf_path)
        self.robot_dof.setText(f"{tr(self._language, 'dof')} {summary.dof}")
        self.robot_dof.setObjectName("StatusDotOk" if summary.dof else "StatusDotPending")
        self.robot_dof.style().unpolish(self.robot_dof)
        self.robot_dof.style().polish(self.robot_dof)
        self.robot_joints.setText("  ".join(summary.joint_names) if summary.joint_names else tr(self._language, "no_joints_loaded"))
        values = model.trajectory_values()
        self.output.setText(str(values["csv_path"] or values["output"] or "runs/sia_excitation.csv"))
        index = self.profile.findText(str(values["profile"]))
        self.profile.setCurrentIndex(max(index, 0))
        self.duration.setValue(float(values["duration"]))
        self.sample_period.setValue(float(values["sample_period"]))
        self.harmonics.setValue(int(values["harmonics"]))
        self.search_candidates.setValue(max(1, int(values["search_candidates"])))
        self.friction_speed_levels.setValue(max(1, int(values["friction_speed_levels"])))
        self.gravity_pose_count.setValue(max(1, int(values["gravity_pose_count"])))
        self.velocity_scale.setValue(float(values["velocity_scale"]))
        self.acceleration_scale.setValue(float(values["acceleration_scale"]))
        self.position_lower.setText(_format_vector(values["position_lower"]))
        self.position_upper.setText(_format_vector(values["position_upper"]))
        validate = model.data.get("trajectory", {}).get("validate", {})
        validate = validate if isinstance(validate, dict) else {}
        self.validate_enabled.setChecked(bool(validate.get("enabled", values["validate_enabled"])))
        self.collision_enabled.setChecked(bool(validate.get("collision", False)))
        self.require_collision.setChecked(bool(validate.get("require_collision", False)))
        self.torque_enabled.setChecked(bool(validate.get("torque", False)))
        self.moveit_group.setText(str(validate.get("moveit_group", "")))
        self.state_validity_service.setText(str(validate.get("state_validity_service", "/check_state_validity")))
        self.collision_sample_limit.setValue(int(validate.get("collision_sample_limit", 200)))
        self.torque_sample_limit.setValue(int(validate.get("torque_sample_limit", 200)))
        self.max_torque_scale.setValue(float(validate.get("max_torque_scale", 1.0)))

    def apply_to_model(self, model: GuiConfigModel) -> None:
        output = self.output.text().strip()
        model.set_value("trajectory", "csv_path", output)
        model.set_nested_value("trajectory", "generation", "profile", self.profile.currentText())
        model.set_nested_value("trajectory", "generation", "duration", self.duration.value())
        model.set_nested_value("trajectory", "generation", "sample_period", self.sample_period.value())
        model.set_nested_value("trajectory", "generation", "harmonics", self.harmonics.value())
        model.set_nested_value("trajectory", "generation", "search_candidates", self.search_candidates.value())
        model.set_nested_value("trajectory", "generation", "friction_speed_levels", self.friction_speed_levels.value())
        model.set_nested_value("trajectory", "generation", "gravity_pose_count", self.gravity_pose_count.value())
        model.set_nested_value("trajectory", "generation", "velocity_scale", self.velocity_scale.value())
        model.set_nested_value("trajectory", "generation", "acceleration_scale", self.acceleration_scale.value())
        position_lower = _parse_vector_text(self.position_lower.text())
        position_upper = _parse_vector_text(self.position_upper.text())
        if position_lower is None:
            model.remove_nested_value("trajectory", "generation", "position_lower")
        else:
            model.set_nested_value("trajectory", "generation", "position_lower", position_lower)
        if position_upper is None:
            model.remove_nested_value("trajectory", "generation", "position_upper")
        else:
            model.set_nested_value("trajectory", "generation", "position_upper", position_upper)
        model.set_nested_value("trajectory", "validate", "enabled", self.validate_enabled.isChecked())
        model.set_nested_value("trajectory", "validate", "collision", self.collision_enabled.isChecked())
        model.set_nested_value("trajectory", "validate", "require_collision", self.require_collision.isChecked())
        model.set_nested_value("trajectory", "validate", "moveit_group", self.moveit_group.text().strip())
        model.set_nested_value(
            "trajectory",
            "validate",
            "state_validity_service",
            self.state_validity_service.text().strip() or "/check_state_validity",
        )
        model.set_nested_value("trajectory", "validate", "collision_sample_limit", self.collision_sample_limit.value())
        model.set_nested_value("trajectory", "validate", "torque", self.torque_enabled.isChecked())
        model.set_nested_value("trajectory", "validate", "torque_sample_limit", self.torque_sample_limit.value())
        model.set_nested_value("trajectory", "validate", "max_torque_scale", self.max_torque_scale.value())
        model.remove_nested_value("trajectory", "validate", "dry_run")

    def trajectory_path(self) -> Path:
        return Path(self.output.text().strip()).expanduser()

    def show_trajectory(self, path: str | Path) -> None:
        self.position_plot.plot_trajectory(path, "position")
        self.velocity_plot.plot_trajectory(path, "velocity")
        self.acceleration_plot.plot_trajectory(path, "acceleration")

    def set_language(self, language: str) -> None:
        self._language = language
        self.header.set_text(tr(language, "excitation_title"), tr(language, "excitation_subtitle"))
        self.robot_panel_title.set_text(tr(language, "robot_model"), tr(language, "robot_model_subtitle"))
        self.settings_panel_title.set_text(tr(language, "trajectory_generator"), tr(language, "trajectory_generator_subtitle"))
        self.safety_panel_title.set_text(tr(language, "validation_safety"), tr(language, "validation_safety_subtitle"))
        self.preview_panel_title.set_text(tr(language, "preview"), tr(language, "preview_subtitle"))
        self.robot_urdf_label.setText(tr(language, "urdf_path"))
        self.robot_joint_order_label.setText(tr(language, "joint_order"))
        if self.robot_joints.text() in {"", tr("en", "no_joints_loaded"), tr("zh", "no_joints_loaded")}:
            self.robot_joints.setText(tr(language, "no_joints_loaded"))
        if self.robot_dof.text().startswith("DOF") or self.robot_dof.text().startswith("自由度"):
            value = self.robot_dof.text().split()[-1]
            self.robot_dof.setText(f"{tr(language, 'dof')} {value}")
        self.output_label.setText(tr(language, "output_csv"))
        self.profile_label.setText(tr(language, "profile"))
        self.harmonics_label.setText(tr(language, "harmonics"))
        self.duration_label.setText(tr(language, "duration"))
        self.velocity_scale_label.setText(tr(language, "velocity_scale"))
        self.sample_period_label.setText(tr(language, "sample_period"))
        self.acceleration_scale_label.setText(tr(language, "acceleration_scale"))
        self.search_candidates_label.setText(tr(language, "search_candidates"))
        self.friction_speed_levels_label.setText(tr(language, "friction_speed_levels"))
        self.gravity_pose_count_label.setText(tr(language, "gravity_pose_count"))
        self.position_lower_label.setText(tr(language, "position_lower"))
        self.position_upper_label.setText(tr(language, "position_upper"))
        self.generate_button.setText(tr(language, "generate"))
        self.validate_button.setText(tr(language, "validate"))
        self.collision_check_button.setText(tr(language, "collision_check"))
        self.trajectory_preview_button.setText(tr(language, "trajectory_preview"))
        self.validate_enabled.setText(tr(language, "enable_validation"))
        self.collision_enabled.setText(tr(language, "moveit_collision"))
        self.require_collision.setText(tr(language, "require_collision_service"))
        self.torque_enabled.setText(tr(language, "pinocchio_torque"))
        self.moveit_group_label.setText(tr(language, "moveit_group"))
        self.state_validity_service_label.setText(tr(language, "state_validity_service"))
        self.collision_sample_limit_label.setText(tr(language, "collision_samples"))
        self.torque_sample_limit_label.setText(tr(language, "torque_samples"))
        self.max_torque_scale_label.setText(tr(language, "torque_scale"))
        self.tabs.setTabText(0, tr(language, "position"))
        self.tabs.setTabText(1, tr(language, "velocity"))
        self.tabs.setTabText(2, tr(language, "acceleration"))
        self._set_field_tooltips(language)

    def _set_field_tooltips(self, language: str) -> None:
        set_tooltip(tr(language, "urdf_tip"), self.robot_urdf_label, self.robot_urdf)
        set_tooltip(tr(language, "joint_order_tip"), self.robot_joint_order_label, self.robot_joints)
        set_tooltip(tr(language, "output_csv_tip"), self.output_label, self.output)
        set_tooltip(tr(language, "profile_tip"), self.profile_label, self.profile)
        set_tooltip(tr(language, "harmonics_tip"), self.harmonics_label, self.harmonics)
        set_tooltip(tr(language, "trajectory_duration_tip"), self.duration_label, self.duration)
        set_tooltip(tr(language, "velocity_scale_tip"), self.velocity_scale_label, self.velocity_scale)
        set_tooltip(tr(language, "sample_period_tip"), self.sample_period_label, self.sample_period)
        set_tooltip(tr(language, "acceleration_scale_tip"), self.acceleration_scale_label, self.acceleration_scale)
        set_tooltip(tr(language, "search_candidates_tip"), self.search_candidates_label, self.search_candidates)
        set_tooltip(tr(language, "friction_speed_levels_tip"), self.friction_speed_levels_label, self.friction_speed_levels)
        set_tooltip(tr(language, "gravity_pose_count_tip"), self.gravity_pose_count_label, self.gravity_pose_count)
        set_tooltip(tr(language, "position_lower_tip"), self.position_lower_label, self.position_lower)
        set_tooltip(tr(language, "position_upper_tip"), self.position_upper_label, self.position_upper)
        set_tooltip(tr(language, "enable_validation_tip"), self.validate_enabled)
        set_tooltip(tr(language, "moveit_collision_tip"), self.collision_enabled)
        set_tooltip(tr(language, "require_collision_service_tip"), self.require_collision)
        set_tooltip(tr(language, "pinocchio_torque_tip"), self.torque_enabled)
        set_tooltip(tr(language, "collision_check_tip"), self.collision_check_button)
        set_tooltip(tr(language, "trajectory_preview_tip"), self.trajectory_preview_button)
        set_tooltip(tr(language, "moveit_group_tip"), self.moveit_group_label, self.moveit_group)
        set_tooltip(tr(language, "state_validity_service_tip"), self.state_validity_service_label, self.state_validity_service)
        set_tooltip(tr(language, "collision_samples_tip"), self.collision_sample_limit_label, self.collision_sample_limit)
        set_tooltip(tr(language, "torque_samples_tip"), self.torque_sample_limit_label, self.torque_sample_limit)
        set_tooltip(tr(language, "torque_scale_tip"), self.max_torque_scale_label, self.max_torque_scale)
