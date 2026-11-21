"""Cancellable ROS 2 action server for synthetic pH measurements at sampling sites."""

import math
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from nav_msgs.msg import Odometry

from phbot_interfaces.action import SamplePh
from phbot_interfaces.msg import PhMeasurement

from .ph_simulation_core import SoilPhModel
from .sampling_pose_core import capture_sampling_pose
from .mission_csv_core import MissionCsvRecorder


_CSV_FIELDS = (
    'timestamp_utc', 'sample_id', 'x_m', 'y_m',
    'target_x_m', 'target_y_m', 'target_error_m',
    'true_ph', 'measured_ph', 'temperature_c',
    'electrode_mv', 'adc_voltage',
)


class PHSimulationNode(Node):
    """Accept one sample at a time and publish only completed measurements."""

    def __init__(self):
        super().__init__('ph_simulation_node')
        self.declare_parameter('field_origin_x_m', -1.0)
        self.declare_parameter('field_origin_y_m', -1.0)
        self.declare_parameter('field_width_m', 6.0)
        self.declare_parameter('field_height_m', 6.0)
        self.declare_parameter('sampling_dwell_s', 35.0)
        self.declare_parameter('noise_std_mv', 2.0)
        self.declare_parameter('adc_bits', 12)
        self.declare_parameter('random_seed', 42)
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('odom_timeout_s', 0.5)
        self.declare_parameter('max_target_offset_m', 0.15)
        # Keep the former CSV untouched: its x/y columns mean planned sites.
        self.declare_parameter(
            'csv_path', '~/.ros/phbot/rover_ph_pose_telemetry_log.csv')

        param = lambda name: self.get_parameter(name).value
        self.model = SoilPhModel(
            width_m=float(param('field_width_m')),
            height_m=float(param('field_height_m')),
            origin_x_m=float(param('field_origin_x_m')),
            origin_y_m=float(param('field_origin_y_m')),
            noise_std_mv=float(param('noise_std_mv')),
            adc_bits=int(param('adc_bits')),
            seed=int(param('random_seed')),
        )
        self.dwell_s = float(param('sampling_dwell_s'))
        self.frame_id = str(param('frame_id'))
        self.odom_timeout = float(param('odom_timeout_s'))
        self.max_target_offset = float(param('max_target_offset_m'))
        self.csv_path = Path(str(param('csv_path'))).expanduser()
        self.csv_recorder = MissionCsvRecorder(self.csv_path, _CSV_FIELDS)
        if not math.isfinite(self.dwell_s) or self.dwell_s < 0:
            raise ValueError('sampling_dwell_s must be finite and nonnegative')
        if (not math.isfinite(self.odom_timeout) or self.odom_timeout <= 0
                or not math.isfinite(self.max_target_offset)
                or self.max_target_offset <= 0):
            raise ValueError('Invalid sampling odometry parameters')
        if not self.frame_id or not str(self.csv_path):
            raise ValueError('frame_id and csv_path must not be empty')

        self._goal_lock = threading.Lock()
        self._odom_lock = threading.Lock()
        self._latest_odom = None
        self._odom_received_at = None
        self._busy = False
        self.create_subscription(
            Odometry, '/localization/odom', self.on_odom, 10)
        self.measurement_pub = self.create_publisher(
            PhMeasurement, '/ph/measurement', 10)
        self._action_group = ReentrantCallbackGroup()
        self._server = ActionServer(
            self, SamplePh, '/ph/sample',
            execute_callback=self.execute_sample,
            goal_callback=self.accept_goal,
            cancel_callback=self.accept_cancel,
            callback_group=self._action_group,
        )
        self.get_logger().info('Synthetic soil pH action server ready on /ph/sample')

    def on_odom(self, msg):
        # The action worker and odometry callback can run concurrently.
        with self._odom_lock:
            self._latest_odom = msg
            self._odom_received_at = time.monotonic()

    def sampling_pose(self, goal):
        # Snapshot the EKF pose ONCE, before the dwell. Subsequent EKF
        # corrections must not change the spatial location of this sample.
        with self._odom_lock:
            odom = self._latest_odom
            received_at = self._odom_received_at
        return capture_sampling_pose(
            odom, received_at,
            now_monotonic_s=time.monotonic(),
            ros_now_ns=self.get_clock().now().nanoseconds,
            expected_frame=self.frame_id,
            odom_timeout_s=self.odom_timeout,
            target_x_m=float(goal.x_coord),
            target_y_m=float(goal.y_coord),
            max_target_offset_m=self.max_target_offset,
            in_bounds=self.model.in_bounds,
        )

    def accept_goal(self, goal):
        """Atomically reserve the sampler before accepting a goal."""
        if goal.sample_id < 0 or not self.model.in_bounds(goal.x_coord, goal.y_coord):
            self.get_logger().warning('Rejected invalid sample ID or coordinates')
            return GoalResponse.REJECT
        with self._goal_lock:
            if self._busy:
                return GoalResponse.REJECT
            self._busy = True
        return GoalResponse.ACCEPT

    def accept_cancel(self, _goal_handle):
        return CancelResponse.ACCEPT

    def execute_sample(self, goal_handle):
        """Simulate dwell in an executor worker; leave other workers responsive."""
        goal = goal_handle.request
        result = SamplePh.Result()
        start = time.monotonic()
        try:
            sampling_pose = self.sampling_pose(goal)
            self.get_logger().info(
                f'Starting pH sample {goal.sample_id}: '
                f'EKF=({sampling_pose.x_m:.3f},{sampling_pose.y_m:.3f}) m, '
                f'target=({goal.x_coord:.3f},{goal.y_coord:.3f}) m, '
                f'offset={sampling_pose.target_error_m:.3f} m')
            while True:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.success = False
                    result.message = 'sampling_cancelled'
                    return result

                elapsed = time.monotonic() - start
                feedback = SamplePh.Feedback()
                feedback.elapsed_seconds = float(min(elapsed, self.dwell_s))
                goal_handle.publish_feedback(feedback)
                remaining = self.dwell_s - elapsed
                if remaining <= 0:
                    break
                time.sleep(min(0.5, remaining))

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = 'sampling_cancelled'
                return result

            # Evaluate synthetic soil at the captured EKF sampling position,
            # never at the requested target coordinates.
            reading = self.model.measure(
                sampling_pose.x_m, sampling_pose.y_m)
            self.write_csv(
                goal.mission_id, goal.sample_id,
                goal.x_coord, goal.y_coord, sampling_pose, reading)

            msg = PhMeasurement()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            msg.sample_id = goal.sample_id
            msg.x_coord = sampling_pose.x_m
            msg.y_coord = sampling_pose.y_m
            msg.measured_ph = reading.measured_ph
            msg.temperature_c = reading.temperature_c
            self.measurement_pub.publish(msg)

            result.success = True
            result.measured_ph = reading.measured_ph
            result.message = 'sampling_complete'
            goal_handle.succeed()
            self.get_logger().info(
                f'Sampled site {goal.sample_id}: pH={reading.measured_ph:.3f}')
            return result
        except Exception as error:
            self.get_logger().error(f'Sampling failed: {error}')
            goal_handle.abort()
            result.success = False
            result.message = f'sampling_failed: {error}'
            return result
        finally:
            with self._goal_lock:
                self._busy = False

    def write_csv(self, mission_id, sample_id, target_x, target_y,
                  pose, reading):
        """Persist one completed measurement to its mission-specific CSV."""
        path = self.csv_recorder.write(mission_id, {
            'timestamp_utc': datetime.now(timezone.utc).isoformat(),
            'sample_id': sample_id,
            # The existing x/y columns remain EKF-estimated sampling positions.
            'x_m': pose.x_m,
            'y_m': pose.y_m,
            'target_x_m': target_x,
            'target_y_m': target_y,
            'target_error_m': pose.target_error_m,
            'true_ph': reading.true_ph,
            'measured_ph': reading.measured_ph,
            'temperature_c': reading.temperature_c,
            'electrode_mv': reading.electrode_mv,
            'adc_voltage': reading.adc_voltage,
        })
        self.get_logger().info(
            f'Persisted pH sample {sample_id} to {path}')


    def destroy_node(self):
        self._server.destroy()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PHSimulationNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
