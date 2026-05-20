from types import SimpleNamespace

import pytest

from robotdynid_ros2.data.ros_extractors import ordered_joint_effort_sample, ordered_joint_state_sample


def _msg(**kwargs):
    return SimpleNamespace(**kwargs)


def test_ordered_joint_state_sample_reorders_by_name() -> None:
    msg = _msg(
        header=_msg(stamp=_msg(sec=2, nanosec=500_000_000)),
        name=["b", "a"],
        position=[2.0, 1.0],
        velocity=[20.0, 10.0],
        effort=[200.0, 100.0],
    )

    sample = ordered_joint_state_sample(msg, ["a", "b"], fallback_timestamp=0.0)

    assert sample.timestamp == 2.5
    assert sample.position == (1.0, 2.0)
    assert sample.velocity == (10.0, 20.0)
    assert sample.effort == (100.0, 200.0)


def test_ordered_joint_state_sample_requires_effort_by_default() -> None:
    msg = _msg(
        header=_msg(stamp=_msg(sec=0, nanosec=0)),
        name=["j1"],
        position=[1.0],
        velocity=[2.0],
        effort=[],
    )

    with pytest.raises(ValueError, match="effort"):
        ordered_joint_state_sample(msg, ["j1"], fallback_timestamp=1.0)

    sample = ordered_joint_state_sample(msg, ["j1"], fallback_timestamp=1.0, allow_missing_effort=True)
    assert sample.timestamp == 1.0
    assert sample.effort == (0.0,)


def test_ordered_joint_effort_sample_does_not_require_motion_fields() -> None:
    msg = _msg(
        header=_msg(stamp=_msg(sec=3, nanosec=0)),
        name=["j2", "j1"],
        effort=[2.0, 1.0],
    )

    sample = ordered_joint_effort_sample(msg, ["j1", "j2"], fallback_timestamp=0.0)

    assert sample.timestamp == 3.0
    assert sample.effort == (1.0, 2.0)
