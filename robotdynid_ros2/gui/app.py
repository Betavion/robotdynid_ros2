"""Application entry point for robotdynid_ros2 Studio."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="", help="Initial robotdynid_ros2 YAML config.")
    parser.add_argument("--workspace", default=str(Path.cwd()), help="ROS workspace root used as process working directory.")
    parser.add_argument("--language", choices=("en", "zh"), default="en", help="Initial GUI language.")
    parser.add_argument("--no-ros-monitor", action="store_true", help="Disable background ROS graph polling.")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        from PySide6.QtWidgets import QApplication
    except ModuleNotFoundError as exc:
        raise SystemExit("PySide6 is required for robotdynid-gui. Install the GUI requirements first.") from exc

    from robotdynid_ros2.gui.main_window import MainWindow
    from robotdynid_ros2.gui.theme import stylesheet

    app = QApplication(sys.argv[:1])
    app.setApplicationName("robotdynid_ros2 Studio")
    app.setStyleSheet(stylesheet())
    window = MainWindow(args)
    window.show()
    return int(app.exec())


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
