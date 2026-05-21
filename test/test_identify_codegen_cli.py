from __future__ import annotations

import argparse

from robotdynid_ros2.cli.identify_codegen import _resolve_prediction_plot_stride


def test_prediction_plot_stride_auto_targets_ten_hz_after_identification_stride(tmp_path: Path) -> None:
    motion = tmp_path / "motion.csv"
    motion.write_text(
        "timestamp,joint1_position\n0.00,0\n0.01,0\n0.02,0\n0.03,0\n",
        encoding="utf-8",
    )

    stride = _resolve_prediction_plot_stride(
        args=argparse.Namespace(prediction_plot_stride=None),
        config_values={"prediction_plot_stride": 0, "prediction_plot_rate_hz": 10.0},
        motion_csv=motion,
        identification_stride=5,
    )

    assert stride == 2


def test_prediction_plot_stride_explicit_value_wins(tmp_path: Path) -> None:
    motion = tmp_path / "motion.csv"
    motion.write_text("timestamp,joint1_position\n0.0,0\n0.01,0\n", encoding="utf-8")

    stride = _resolve_prediction_plot_stride(
        args=argparse.Namespace(prediction_plot_stride=7),
        config_values={"prediction_plot_stride": 0, "prediction_plot_rate_hz": 10.0},
        motion_csv=motion,
        identification_stride=5,
    )

    assert stride == 7
