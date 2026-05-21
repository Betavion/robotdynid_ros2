"""ROS graph monitoring for the GUI."""

from __future__ import annotations

import time
from dataclasses import dataclass

from PySide6.QtCore import QThread, Signal


@dataclass(frozen=True)
class RosGraphState:
    available: bool
    topics: tuple[str, ...] = ()
    services: tuple[str, ...] = ()
    actions: tuple[str, ...] = ()
    error: str = ""

    def has_topic(self, name: str) -> bool:
        return name in self.topics

    def has_service(self, name: str) -> bool:
        return name in self.services

    def has_action(self, name: str) -> bool:
        return name in self.actions


def _action_names_from_graph(node: object) -> tuple[str, ...]:
    getter = getattr(node, "get_action_names_and_types", None)
    if getter is None:
        try:
            from rclpy.action import get_action_names_and_types
        except (ImportError, AttributeError):
            return ()
        names_and_types = get_action_names_and_types(node)
    else:
        names_and_types = getter()
    return tuple(sorted(name for name, _types in names_and_types))


class RosGraphMonitor(QThread):
    """Poll the ROS graph in a background thread."""

    state_changed = Signal(object)

    def __init__(self, poll_period_sec: float = 1.0) -> None:
        super().__init__()
        self._poll_period_sec = poll_period_sec
        self._running = True

    def stop(self) -> None:
        self._running = False
        self.wait(2000)

    def run(self) -> None:  # pragma: no cover - depends on a live ROS graph
        try:
            import rclpy
        except ModuleNotFoundError as exc:
            self.state_changed.emit(RosGraphState(False, error=str(exc)))
            return

        shutdown_after = False
        try:
            if not rclpy.ok():
                rclpy.init()
                shutdown_after = True
            node = rclpy.create_node("robotdynid_gui_monitor")
        except Exception as exc:  # noqa: BLE001
            self.state_changed.emit(RosGraphState(False, error=str(exc)))
            return

        try:
            while self._running:
                try:
                    rclpy.spin_once(node, timeout_sec=0.0)
                    topics = tuple(sorted(name for name, _types in node.get_topic_names_and_types()))
                    services = tuple(sorted(name for name, _types in node.get_service_names_and_types()))
                    actions = _action_names_from_graph(node)
                    self.state_changed.emit(RosGraphState(True, topics=topics, services=services, actions=actions))
                except Exception as exc:  # noqa: BLE001
                    self.state_changed.emit(RosGraphState(False, error=str(exc)))
                time.sleep(self._poll_period_sec)
        finally:
            node.destroy_node()
            if shutdown_after and rclpy.ok():
                rclpy.shutdown()
