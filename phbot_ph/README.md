# Synthetic soil pH sampling — ROS 2 Jazzy

This package exposes a cancellable action at `/ph/sample` of type
`phbot_interfaces/action/SamplePh`. Goals specify a zero-based site ID and *planned target* x/y coordinates in
the configured local frame (default `odom`). The action server subscribes to
`/localization/odom`, snapshots a fresh EKF-estimated x/y position when the
sampling action begins (before the 35-second dwell), and evaluates the synthetic
soil at **that position**. The planned target is retained for diagnostics.
Completed measurements publish to `/ph/measurement` of type
`phbot_interfaces/msg/PhMeasurement`, whose `x_coord`/`y_coord` now
represent the EKF-estimated **sampling** location, not the planned target.

Build after sourcing ROS 2 Jazzy, from `ros2_ws`:

```bash
sudo apt install python3-numpy python3-scipy
colcon build --packages-up-to phbot_ph
source install/setup.bash
ros2 run phbot_ph ph_simulation_node
```

Once the EKF is publishing fresh `/localization/odom` in the `odom`
frame, and the robot is stationary near a valid target, invoke Mission
Control normally or send a test goal at the current estimated position:

```bash
ros2 action send_goal --feedback /ph/sample phbot_interfaces/action/SamplePh "{sample_id: 0, x_coord: 0.0, y_coord: 0.0}"
ros2 topic echo /ph/measurement
```

The example target assumes the EKF pose is near (0, 0) metres. If odometry is
missing, stale (receipt or ROS header), in the wrong frame, outside the pH
field, or more than `max_target_offset_m` from the planned site, the goal
fails without producing a measurement. Existing Mission Control stationary
and localisation checks are still required for powered operation.

For faster development, override `sampling_dwell_s` (default 35 s):
`ros2 run phbot_ph ph_simulation_node --ros-args -p sampling_dwell_s:=0.2`.

The synthetic ground-truth field currently has **169 illustrative anchors**
at 0.5 m spacing, covering x and y from -1 to 5 m in the same local map
frame (default `odom`) used by the global planner. The configurable field
origin defaults to (-1, -1) m and its dimensions default to 6 x 6 m.
Coordinates are passed directly to Clough-Tocher cubic interpolation:
**there is no 0..10 rescaling**. The configured field must lie entirely
inside the anchor region. Nearest-neighbour is retained as a numerical
fallback, not a substitute for missing ground-truth coverage. The electrode simulation applies the temperature-dependent ideal
Nernst slope, independently sampled seeded Gaussian voltage noise (default
2 mV), and quantization with an idealized 12-bit virtual ADC at 0–5 V.
**The 5 V simulated ADC is not an electrically safe input for the ESP32.**
No actual probe response, moisture effects, actuator motion, or calibration
is validated by this model. The 35 s dwell is a scheduling demonstration.

Parameters: `field_origin_x_m`, `field_origin_y_m`, `field_width_m`,
`field_height_m`, `sampling_dwell_s`, `noise_std_mv`, `adc_bits`,
`random_seed`, `frame_id`, `odom_timeout_s` (default 0.5 s),
`max_target_offset_m` (default 0.15 m), `csv_path`.

The pH sampler snapshots the EKF pose at action execution start and uses
that fixed position after the dwell. It does not read the robot's actual,
independently measured ground-truth position; this is an explicit approximation
in the physical robot's synthetic pH simulation.

This package does **not yet** subscribe to global map metadata. If the map
origin or dimensions change, configure these field parameters accordingly.
The pH action still receives **planned sampling coordinates**, but evaluates
the ground-truth field at the **EKF estimate** captured when sampling begins.
Mission Control assigns a unique UTC-timestamped `mission_id` on each
successful mission start and forwards it to each `/ph/sample` action goal.
The ID is retained through sampling, replanning, pause and resume.
The pH node uses it to group all completed samples from a mission in one
dedicated CSV. Manual action calls may omit `mission_id` (the empty-string
default); in that case, each accepted measurement receives its own unique
standalone CSV.
Out-of-bounds and nonfinite coordinates are rejected, not clamped. One goal
may run at a time. Pending goals can be cancelled; cancelled operations do
not produce successful measurements.

The default `csv_path` is
`~/.ros/phbot/rover_ph_pose_telemetry_log.csv` and now acts as a **filename
template**, not as a single append-only output file. For example, all
samples of one mission are saved to
`~/.ros/phbot/rover_ph_pose_telemetry_log_20261110T183000_000000Z_1a2b3c4d.csv`.
The original file is never modified. All existing CSV columns are
preserved, including EKF-estimated `x_m`/`y_m`, planned
`target_x_m`/`target_y_m`, `target_error_m`, and electrode diagnostics.
An existing mission CSV with a different header is rejected rather than
corrupted. Restarting the action server during the *same mission* will
append to the same mission file when the same ID is supplied.
The `true_ph` column is for diagnostics only and **must never be read by
the pH map generator**. The public measurement topic contains only measured
pH and the estimated sampling coordinates.

Mission Control sends the `/ph/sample` action after validated physical
arrival and stationary settling; it advances only on a successful action
result. The pH node itself does not command motors. `ph_map_node.py` is
currently a placeholder.

Local core tests (outside ROS):
`python3 -m pytest ros2_ws/src/phbot_ph/test/test_ph_simulation_core.py ros2_ws/src/phbot_ph/test/test_sampling_pose_core.py ros2_ws/src/phbot_ph/test/test_mission_csv_core.py`.

Changing the `SamplePh.action` goal requires rebuilding and sourcing
`phbot_interfaces`, `phbot_ph`, and `phbot_mission` together; do not
run stale action clients or servers from earlier builds.
