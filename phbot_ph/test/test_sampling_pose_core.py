"""Unit tests for the pH sample position snapshot (no ROS installation required)."""
from types import SimpleNamespace

import math
import pytest

from phbot_ph.ph_simulation_core import SoilPhModel
from phbot_ph.sampling_pose_core import capture_sampling_pose


def make_odom(x=0.34, y=-0.28, frame='odom', stamp_ns=10_000_000_000):
    return SimpleNamespace(
        header=SimpleNamespace(
            frame_id=frame,
            stamp=SimpleNamespace(
                sec=stamp_ns // 1_000_000_000,
                nanosec=stamp_ns % 1_000_000_000)),
        pose=SimpleNamespace(
            pose=SimpleNamespace(position=SimpleNamespace(x=x, y=y))),
    )


def capture(odom=None, **overrides):
    args = dict(
        received_monotonic_s=49.9,
        now_monotonic_s=50.0,
        ros_now_ns=10_100_000_000,
        expected_frame='odom',
        odom_timeout_s=0.5,
        target_x_m=0.3,
        target_y_m=-0.3,
        max_target_offset_m=0.15,
        in_bounds=SoilPhModel().in_bounds,
    )
    args.update(overrides)
    return capture_sampling_pose(odom or make_odom(),
                                 args.pop('received_monotonic_s'), **args)


def test_sampling_uses_ekf_position_not_target():
    pose = capture()
    assert pose.x_m == pytest.approx(0.34)
    assert pose.y_m == pytest.approx(-0.28)
    assert pose.target_error_m == pytest.approx(math.hypot(0.04, 0.02))


def test_sampling_accepts_negative_map_coordinates():
    assert capture(make_odom(x=0.6, y=-0.3),
                   target_x_m=0.6).y_m == pytest.approx(-0.3)


@pytest.mark.parametrize('odom,extra,reason', [
    (None, {'received_monotonic_s': None}, 'odometry_unavailable'),
    (make_odom(frame='map'), {}, 'odometry_frame_mismatch'),
    (make_odom(), {'received_monotonic_s': 48.0}, 'odometry_receipt_stale'),
    (make_odom(stamp_ns=9_000_000_000), {},
     'odometry_header_stamp_stale'),
    (make_odom(stamp_ns=0), {}, 'odometry_header_stamp_missing'),
    (make_odom(stamp_ns=11_000_000_000), {},
     'odometry_header_stamp_stale'),
    (make_odom(x=float('nan')), {}, 'nonfinite_sample_position'),
    (make_odom(x=-2.0), {}, 'sample_pose_outside_ph_field'),
    (make_odom(x=0.8), {}, 'sample_pose_outside_target_tolerance'),
])
def test_sampling_rejects_invalid_pose(odom, extra, reason):
    with pytest.raises(ValueError, match=reason):
        if odom is None:
            capture_sampling_pose(None, extra['received_monotonic_s'],
                                  now_monotonic_s=50.0,
                                  ros_now_ns=10_100_000_000,
                                  expected_frame='odom',
                                  odom_timeout_s=0.5,
                                  target_x_m=0.3, target_y_m=-0.3,
                                  max_target_offset_m=0.15,
                                  in_bounds=SoilPhModel().in_bounds)
        else:
            capture(odom, **extra)


def test_sampling_pose_snapshot_is_independent_of_future_odometry():
    odom = make_odom()
    frozen = capture(odom)
    odom.pose.pose.position.x = 0.8
    assert frozen.x_m == pytest.approx(0.34)
