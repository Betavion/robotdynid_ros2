"""Record JointState data into robotdynid split CSV files."""

from __future__ import annotations

from collections import deque
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from robotdynid_ros2.data.csv_writer import SplitDatasetCsvWriter, TorqueEstimateCsvWriter
from robotdynid_ros2.data.manifest import MANIFEST_NAME, build_collection_manifest, write_manifest
from robotdynid_ros2.data.ros_extractors import OrderedJointEffortSample, OrderedJointSample
from robotdynid_ros2.data.ros_extractors import ordered_joint_effort_sample, ordered_joint_state_sample
from robotdynid_ros2.paths import timestamped_run_dir


def _parse_joint_names(raw: object) -> list[str]:
    if isinstance(raw, str):
        stripped = raw.strip()
        if not stripped:
            return []
        if stripped.startswith("["):
            stripped = stripped.strip("[]")
            return [part.strip().strip("'\"") for part in stripped.split(",") if part.strip()]
        return [part.strip() for part in stripped.split(",") if part.strip()]
    return [str(name) for name in raw]


def _parse_bool(raw: object) -> bool:
    if isinstance(raw, str):
        return raw.strip().lower() in {"1", "true", "yes", "on"}
    return bool(raw)


class DatasetRecorderNode(Node):
    """A lightweight recorder for dynamics-identification datasets."""

    def __init__(self) -> None:
        super().__init__("robotdynid_dataset_recorder")

        self.declare_parameter("output_root", "runs")
        self.declare_parameter("run_name", "")
        self.declare_parameter("joint_names", [])
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("estimate_joint_state_topic", "")
        self.declare_parameter("urdf_path", "")
        self.declare_parameter("duration_sec", 0.0)
        self.declare_parameter("min_sample_period_sec", 0.0)
        self.declare_parameter("flush_period_sec", 0.01)
        self.declare_parameter("queue_max_samples", 0)
        self.declare_parameter("auto_start", True)
        self.declare_parameter("shutdown_on_finish", False)
        self.declare_parameter("allow_missing_effort", False)

        self._joint_names = _parse_joint_names(self.get_parameter("joint_names").value)
        if not self._joint_names:
            raise ValueError("Parameter 'joint_names' must be a non-empty string array.")

        self._topic = str(self.get_parameter("joint_state_topic").value)
        self._estimate_topic = str(self.get_parameter("estimate_joint_state_topic").value)
        self._urdf_path = str(self.get_parameter("urdf_path").value)
        self._min_sample_period = float(self.get_parameter("min_sample_period_sec").value)
        self._flush_period = float(self.get_parameter("flush_period_sec").value)
        self._queue_max_samples = int(self.get_parameter("queue_max_samples").value)
        self._duration = float(self.get_parameter("duration_sec").value)
        self._shutdown_on_finish = _parse_bool(self.get_parameter("shutdown_on_finish").value)
        self._allow_missing_effort = _parse_bool(self.get_parameter("allow_missing_effort").value)

        self._writer: SplitDatasetCsvWriter | None = None
        self._estimate_writer: TorqueEstimateCsvWriter | None = None
        self._run_dir: Path | None = None
        self._started_time = None
        self._last_sample_timestamp: float | None = None
        self._is_recording = False
        self._pending_samples: deque[OrderedJointSample] = deque()
        self._pending_estimates: deque[OrderedJointEffortSample] = deque()
        self._dropped_sample_count = 0
        self._dropped_estimate_count = 0

        self.create_subscription(JointState, self._topic, self._on_joint_state, 100)
        if self._estimate_topic:
            self.create_subscription(JointState, self._estimate_topic, self._on_estimate_joint_state, 100)
        self.create_service(Trigger, "start_recording", self._on_start_recording)
        self.create_service(Trigger, "stop_recording", self._on_stop_recording)
        if self._flush_period <= 0.0:
            raise ValueError("Parameter 'flush_period_sec' must be positive.")
        self.create_timer(self._flush_period, self._flush_pending_samples)
        if self._duration > 0.0:
            self.create_timer(0.1, self._check_duration)

        if _parse_bool(self.get_parameter("auto_start").value):
            self.start_recording()

    @property
    def run_dir(self) -> Path | None:
        return self._run_dir

    def start_recording(self) -> Path:
        if self._is_recording:
            assert self._run_dir is not None
            return self._run_dir

        output_root = str(self.get_parameter("output_root").value)
        run_name = str(self.get_parameter("run_name").value) or None
        self._run_dir = timestamped_run_dir(output_root, run_name)
        data_dir = self._run_dir / "data"
        self._writer = SplitDatasetCsvWriter(data_dir, len(self._joint_names))
        self._estimate_writer = TorqueEstimateCsvWriter(data_dir, self._joint_names) if self._estimate_topic else None
        self._started_time = self.get_clock().now()
        self._last_sample_timestamp = None
        self._pending_samples.clear()
        self._pending_estimates.clear()
        self._dropped_sample_count = 0
        self._dropped_estimate_count = 0
        self._is_recording = True
        self.get_logger().info(f"Recording robot dynamics dataset into {self._run_dir}")
        return self._run_dir

    def stop_recording(self) -> Path | None:
        if not self._is_recording:
            return self._run_dir
        assert self._writer is not None
        assert self._run_dir is not None

        self._flush_pending_samples()
        self._writer.close()
        estimate_path = self._estimate_writer.path if self._estimate_writer is not None else None
        estimate_count = self._estimate_writer.sample_count if self._estimate_writer is not None else 0
        if self._estimate_writer is not None:
            self._estimate_writer.close()
        manifest = build_collection_manifest(
            run_dir=self._run_dir,
            data_dir=self._writer.output_dir,
            motion_csv=self._writer.motion_path,
            torque_csv=self._writer.torque_path,
            joint_names=self._joint_names,
            joint_state_topic=self._topic,
            sample_count=self._writer.sample_count,
            urdf_path=self._urdf_path,
            estimate_joint_state_topic=self._estimate_topic,
            estimate_csv=estimate_path,
            estimate_sample_count=estimate_count,
            dropped_motion_torque_sample_count=self._dropped_sample_count,
            dropped_estimate_sample_count=self._dropped_estimate_count,
        )
        write_manifest(self._run_dir / MANIFEST_NAME, manifest)
        self.get_logger().info(f"Stopped recording {self._writer.sample_count} samples into {self._run_dir}")

        self._writer = None
        self._estimate_writer = None
        self._is_recording = False
        if self._shutdown_on_finish:
            rclpy.shutdown()
        return self._run_dir

    def _on_start_recording(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        run_dir = self.start_recording()
        response.success = True
        response.message = str(run_dir)
        return response

    def _on_stop_recording(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        run_dir = self.stop_recording()
        response.success = True
        response.message = str(run_dir or "")
        return response

    def _on_joint_state(self, msg: JointState) -> None:
        if not self._is_recording or self._writer is None:
            return
        fallback_timestamp = self.get_clock().now().nanoseconds * 1e-9
        try:
            sample = ordered_joint_state_sample(
                msg,
                self._joint_names,
                fallback_timestamp=fallback_timestamp,
                allow_missing_effort=self._allow_missing_effort,
            )
        except ValueError as exc:
            self.get_logger().warn(str(exc), throttle_duration_sec=2.0)
            return

        if self._last_sample_timestamp is not None and self._min_sample_period > 0.0:
            if sample.timestamp - self._last_sample_timestamp < self._min_sample_period:
                return
        self._enqueue_sample(sample)
        self._last_sample_timestamp = sample.timestamp

    def _on_estimate_joint_state(self, msg: JointState) -> None:
        if not self._is_recording or self._estimate_writer is None:
            return
        fallback_timestamp = self.get_clock().now().nanoseconds * 1e-9
        try:
            sample = ordered_joint_effort_sample(msg, self._joint_names, fallback_timestamp=fallback_timestamp)
        except ValueError as exc:
            self.get_logger().warn(str(exc), throttle_duration_sec=2.0)
            return
        self._enqueue_estimate(sample)

    def _enqueue_sample(self, sample: OrderedJointSample) -> None:
        if self._queue_max_samples > 0 and len(self._pending_samples) >= self._queue_max_samples:
            self._pending_samples.popleft()
            self._dropped_sample_count += 1
        self._pending_samples.append(sample)

    def _enqueue_estimate(self, sample: OrderedJointEffortSample) -> None:
        if self._queue_max_samples > 0 and len(self._pending_estimates) >= self._queue_max_samples:
            self._pending_estimates.popleft()
            self._dropped_estimate_count += 1
        self._pending_estimates.append(sample)

    def _flush_pending_samples(self) -> None:
        if not self._is_recording:
            return
        if self._writer is not None:
            while self._pending_samples:
                sample = self._pending_samples.popleft()
                self._writer.append(sample.timestamp, sample.position, sample.velocity, sample.effort)
            self._writer.flush()
        if self._estimate_writer is not None:
            while self._pending_estimates:
                sample = self._pending_estimates.popleft()
                self._estimate_writer.append(sample.timestamp, sample.effort)
            self._estimate_writer.flush()

    def _check_duration(self) -> None:
        if not self._is_recording or self._started_time is None or self._duration <= 0.0:
            return
        elapsed = (self.get_clock().now() - self._started_time).nanoseconds * 1e-9
        if elapsed >= self._duration:
            self.stop_recording()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = DatasetRecorderNode()
    try:
        rclpy.spin(node)
    finally:
        node.stop_recording()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
