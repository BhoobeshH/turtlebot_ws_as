# Robot Localization in Simulation: EKF Fusing Noisy Odometry and IMU Yaw

Assignment 1: Localization and Simulation Foundations
Platform: ROS 2 Humble, Gazebo Classic 11, TurtleBot3 Burger


---

## 1. Summary

A differential-drive TurtleBot3 Burger drives a short trajectory in the Gazebo
`turtlebot3_world` environment. Its odometry and IMU heading are corrupted with
known, motion-dependent errors, and an Extended Kalman Filter (EKF) estimates the
robot pose `[x, y, yaw]` from those corrupted measurements only. The estimate is
compared with the simulator's ground-truth pose. Ground truth is used for
evaluation only and is never an input to the estimator.

## 2. Problem Statement

**Scenario:** tracking the 2D pose (position and heading) of a differential-drive
robot in an indoor simulated environment, using noisy wheel odometry and a noisy
IMU heading.

**Why it is a problem:** wheel odometry is dead reckoning. Small errors in wheel
size or wheel spacing accumulate with distance, so the estimated pose drifts away
from the true pose without bound. The goal is to reduce that drift by fusing a
second sensor and to measure how far the result is from ground truth.

**Data used by the estimator**

| Source | Topic | Used for |
|---|---|---|
| Noisy wheel odometry | `/odom_noisy` | linear and angular velocity (motion model) |
| IMU | `/imu` | absolute heading (yaw) measurement |

**Data used only for evaluation**

| Source | Topic |
|---|---|
| Simulator ground-truth pose | `/ground_truth` |


## 3. Package layout

```
my_localization/
├── my_localization/
│   ├── ekf_node.py        # EKF: predict from odometry, update from IMU yaw
│   ├── noisy_odom.py      # adds simulated errors to odometry -> /odom_noisy
│   └── evaluate.py        # compares estimates with ground truth, prints errors, saves plot
├── launch/ekf_launch.py   # starts noisy_odom and ekf_node (EKF reads /odom_noisy)
├── models/                # copy of the Burger model.sdf used (ground-truth plugin, encoder odometry)
├── results/               # saved plots from the runs
├── package.xml
└── setup.py
```

## 4. Simulation Setup

- **Robot and world:** TurtleBot3 Burger in `turtlebot3_world` (Gazebo Classic 11,
  ROS 2 Humble). The robot spawns at about (-2.0, -0.5) m in the world.
- **Odometry source:** the diff-drive plugin is set to encoder mode
  (`<odometry_source>0</odometry_source>` in `model.sdf`). By default the plugin
  uses the simulator's world pose instead, which would make the odometry a copy of
  ground truth.
- **Ground truth:** a `libgazebo_ros_p3d.so` plugin attached to `base_link`
  publishes the true pose in the world frame on `/ground_truth` at 30 Hz.
- **Sensor rates:** odometry 30 Hz, IMU 200 Hz, LiDAR 5 Hz (the LiDAR is not used
  by this estimator).
- **Why noise is added:** Gazebo's wheels barely slip and its IMU orientation has no
  noise of its own, so the raw sensor data is almost perfect and there would be
  nothing to estimate. The assignment asks to model uncertainty in the odometry and
  in any sensor used, so errors are injected explicitly (section 5).

## 5. Uncertainty Model

All injected errors are **zero when the robot is stationary**, as for real wheel
encoders. They are applied to the measurements only. The robot's real motion in
Gazebo and the ground truth are untouched.

### 5.1 Odometry errors (`noisy_odom.py`)

The node reads the clean velocities `v` (forward) and `w` (turn rate) from `/odom`,
corrupts them, and integrates them into a drifting pose published on `/odom_noisy`:

```
v_n = v * (1 + s)        + N(0, (0.02 * |v|)^2)
w_n = w + b * v          + N(0, (0.02 * (|v| + |w|))^2)

yaw += w_n * dt
x   += v_n * dt * cos(yaw)
y   += v_n * dt * sin(yaw)
```

| Parameter | Meaning | Real-world cause | Value |
|---|---|---|---|
| `s` (`v_scale_err`) | speed scale error | wheel radius error | `<fill: 0.05>` (5 %) |
| `b` (`curv_bias`) | heading drift per metre driven | unequal wheel diameters | `<fill: 0.05>` rad/m |
| 0.02 | random noise factor | slip, quantisation | fixed |

These values are **assumptions** chosen as plausible for a small wheeled robot.
They are not measured from hardware.

### 5.2 IMU yaw noise (in `ekf_node.py`)

The simulated IMU orientation has no noise, so Gaussian noise is added to the yaw
before it is used as a measurement:

```
z = wrap(yaw_imu + offset + N(0, sigma^2)),   sigma = r_yaw = <fill: 0.05> rad (about 2.9 degrees)
```

`offset` aligns the IMU yaw frame with the odometry frame at start-up. The same
`sigma` is used as the filter's measurement noise `R`, so the filter's assumption
matches the simulated sensor.

## 6. Localization Method: Extended Kalman Filter

**State:** `x = [x, y, yaw]`, with covariance `P` (3x3), initialised from the first
odometry pose and `P = 0.1 * I`.

### 6.1 Predict step (on every `/odom_noisy` message)

The motion model integrates the measured velocities over the time step `dt`:

```
x_k   = x   + v * dt * cos(yaw)
y_k   = y   + v * dt * sin(yaw)
yaw_k = wrap(yaw + w * dt)
```

The model is nonlinear in `yaw`, so it is linearised with the Jacobian:

```
F = [ 1  0  -v*dt*sin(yaw) ]
    [ 0  1   v*dt*cos(yaw) ]
    [ 0  0   1             ]

P = F P F^T + Q,      Q = diag(q_xy, q_xy, q_yaw) * dt
```

`Q` is the process noise: how little the filter trusts the motion model. Scaling
it by `dt` makes the growth depend on elapsed time, not on the message rate.

### 6.2 Update step (on every `/imu` message)

The IMU measures only yaw, so `H = [0 0 1]`:

```
innovation   y = wrap(z - yaw)
S = H P H^T + R,       R = r_yaw^2
K = P H^T S^-1
x = x + K y
P = (I - K H) P
```

The gain `K` weighs the prediction against the IMU. A large `r_yaw` makes the
filter rely on the odometry, a small one makes it follow the IMU.

### 6.3 Output

The pose and covariance are published as `geometry_msgs/PoseWithCovarianceStamped`
on `/ekf_pose` in the `odom` frame. The filter does not publish a TF, to avoid
clashing with the simulator's own `odom` transform.

### 6.4 Filter parameters

| Parameter | Meaning | Value |
|---|---|---|
| `q_xy` | position process noise (per second) | `<fill: 0.001>` |
| `q_yaw` | heading process noise (per second) | `<fill: 0.02>` |
| `r_yaw` | IMU yaw measurement noise (std, rad) | `<fill: 0.05>` |


## 7. Evaluation Method (`evaluate.py`)

The evaluator subscribes to `/ground_truth`, `/odom_noisy` and `/ekf_pose`. When
stopped with Ctrl+C it:

1. Shifts each trajectory so it starts at (0, 0), because the estimators work in
   the `odom` frame and the ground truth is in the world frame. This assumes all
   three start at the same pose with the same heading, which holds at the spawn point.
2. Interpolates each estimate to the ground-truth timestamps.
3. Computes the position error `e = sqrt((x_est - x_gt)^2 + (y_est - y_gt)^2)` at
   each timestamp.
4. Prints RMSE, final error and maximum error for the noisy odometry and for the EKF.
5. Saves `trajectory.png`: estimated and ground-truth paths, labelled axes in
   metres, and a legend.

Only `evaluate.py` reads `/ground_truth`. This can be checked with
`grep -rn ground_truth src/my_localization --include=*.py`.

## 8. How to Run

Requirements: Ubuntu 22.04, ROS 2 Humble, Gazebo Classic, TurtleBot3 packages
(humble branch) built in `~/turtlebot_ws_as`, `python3-matplotlib`.

```bash
git clone git@github.com:BhoobeshH/turtlebot_ws_as.git
cd ~/turtlebot_ws_as
colcon build --symlink-install
source install/setup.bash
export TURTLEBOT3_MODEL=burger
```

Run each command in its own terminal (source the workspace in each):

```bash
# 1. simulation
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# 2. noisy odometry + EKF
ros2 launch my_localization ekf_launch.py

# 3. evaluator (start before driving; plots are saved in the current folder)
cd ~/turtlebot_ws_as/src/my_localization/results
ros2 run my_localization evaluate

# 4. drive the robot
ros2 run turtlebot3_teleop teleop_keyboard

# 5 view in rviz
ros2 run rviz2 rviz2
```
**Set the Fixed Frame in RViz as odom**

**Add PoseWithCovariance in topic ekf_pose**

Drive a path of several metres with a few turns, stop the robot, then press Ctrl+C in
the evaluator terminal. It prints the errors and saves `trajectory.png`.

## 9. Results



| Method | RMSE [m] | Final error [m] | Max error [m] |
|---|---|---|---|
| Noisy odometry (dead reckoning) | `0.224` | `0.374` | `0.375` |
| EKF (odometry + IMU yaw) | `0.112` | `0.171` | `0.171` |



## 10. Analysis

**Why the noisy odometry drifts.** Two errors act on it. The speed scale error `s`
stretches every distance by a constant fraction, giving a position error of about
`s * d` after driving `d` metres. The curvature bias `b` turns the heading by
`b * d` radians after `d` metres, so on a straight drive the sideways error grows as
`b * d^2 / 2`. For example, `d = 4 m`, `b = 0.05 rad/m` gives about 0.4 m. The
heading error also rotates all later motion, which is why dead reckoning gets worse
the further and the more the robot turns.

**Why the EKF is better.** The IMU gives an absolute heading, so the heading error
does not accumulate and the curvature bias is removed. The sideways drift is largely
eliminated.

**What error remains.** The IMU measures heading only, so nothing in the filter
corrects the speed scale error. The remaining position error is therefore close to
`s * d`, about 5 % of the distance driven. `<fill: compare with your result, e.g. path 4.3 m x 5 % = 0.22 m against a measured final error of 0.22 m>`

**Uncertainty growth.** The covariance of `x` and `y` grows steadily, because no
sensor measures position directly. This is correct behaviour for this design and
is visible as a growing uncertainty ellipse when `/ekf_pose` is plotted in RViz.

## 11. Limitations

- Only heading is corrected. Position error still grows with distance. Bounding it
  needs an absolute position measurement, for example scan matching against a map
  (AMCL or a particle filter) using the LiDAR.
- The simulated IMU yaw is unbiased. A real IMU also drifts, so the benefit seen
  here would be smaller on hardware.
- The noise levels are assumed, not measured. Results depend on them.
- The noise is random, so exact numbers change from run to run. Results come from a
  small number of runs on short trajectories.
- The evaluator aligns trajectories by their start point only. This is valid because
  every trajectory starts at the spawn pose with the same heading.
- Only position error is reported. Heading error could be added in the same way.

