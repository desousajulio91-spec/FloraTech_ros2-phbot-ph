"""Pure validation and snapshot of the EKF pose used for a simulated sample.

No ROS imports: tests can supply a nav_msgs/Odometry-like object.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class SamplingPose:
    x_m: float
    y_m: float
    target_error_m: float


def capture_sampling_pose(odom, received_monotonic_s, *,
                          now_monotonic_s, ros_now_ns, expected_frame,
                          odom_timeout_s, target_x_m, target_y_m,
                          max_target_offset_m, in_bounds):
    """Return one fresh EKF pose near the intended site, or fail closed."""
    if odom is None or received_monotonic_s is None:
        raise ValueError('odometry_unavailable')
    if odom.header.frame_id != expected_frame:
        raise ValueError('odometry_frame_mismatch')
    if (not math.isfinite(received_monotonic_s)
            or not math.isfinite(now_monotonic_s)
            or not 0.0 <= now_monotonic_s - received_monotonic_s
            <= odom_timeout_s):
        raise ValueError('odometry_receipt_stale')
    stamp_ns = (int(odom.header.stamp.sec) * 1_000_000_000
                + int(odom.header.stamp.nanosec))
    if stamp_ns <= 0:
        raise ValueError('odometry_header_stamp_missing')
    age_s = (ros_now_ns - stamp_ns) / 1e9
    if not -0.2 <= age_s <= odom_timeout_s:
        raise ValueError('odometry_header_stamp_stale')

    x = float(odom.pose.pose.position.x)
    y = float(odom.pose.pose.position.y)
    if not all(math.isfinite(v) for v in (x, y, target_x_m, target_y_m)):
        raise ValueError('nonfinite_sample_position')
    if not in_bounds(x, y):
        raise ValueError('sample_pose_outside_ph_field')
    error = math.hypot(x - target_x_m, y - target_y_m)
    if error > max_target_offset_m:
        raise ValueError('sample_pose_outside_target_tolerance')
    return SamplingPose(x_m=x, y_m=y, target_error_m=error)
