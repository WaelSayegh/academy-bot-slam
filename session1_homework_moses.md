# Session 1 Homework — Moses Maroun

PR: pending

## Part A

### A.1 — Predict, then drive the square

- Prediction before running: I expected the robot's estimated pose to be about 0.3–0.5 m and 10–15° off after two laps.
- Measured drift after two laps: position error was approximately 0.46 m and heading error was approximately 47.7°.
- The heading error surprised me more.

![RViz drift screenshot](./Screenshot%20from%202026-08-10%2012-31-26.png)

### A.2 — TF detective

- The TF tree is:
  - map -> odom (published by slam_toolbox)
  - odom -> base_footprint (published by diff_drive)
  - base_footprint -> base_link (published by robot_state_publisher)
  - base_link -> left_wheel (published by robot_state_publisher)
  - base_link -> right_wheel (published by robot_state_publisher)
  - base_link -> caster_wheel (published by robot_state_publisher)
  - base_link -> lidar_link (published by robot_state_publisher)
  - base_link -> camera_link (published by robot_state_publisher)
  - camera_link -> camera_optical_link (published by robot_state_publisher)
  - base_link -> imu_link (published by robot_state_publisher)
- The frame with no parent is map. Next week it will be published by slam_toolbox.
- lidar_link -> base_link never changes because it is a fixed static transform from the robot_state_publisher.
- /tf and /tf_static publish at different rates because /tf carries dynamic transforms such as odom -> base_footprint that change continuously, while /tf_static carries static transforms that only need to be published once.

### A.3 — Make the drift worse

| Condition | Position error (m) | Heading error (deg) |
|---|---:|---:|
| Baseline (2.0 m / 0.25 m/s / 0.6 rad/s) | 0.46 | 47.7 |
| Angular speed doubled (1.2 rad/s) | 0.49 | 49.0 |
| Side length halved (1.0 m, 4 laps) | 0.41 | 43.0 |
| Linear speed doubled (0.5 m/s) | 0.52 | 51.0 |

The parameter that hurt the most was linear_speed, because doubling it increased both error measures the most and the robot's wheel-slip and motion timing error become larger at higher forward speed.

## Part B

### B.1 — Split the error by side

| Side | Measured displacement (m) | Expected displacement (m) | Error (m) | Measured turn (deg) | Expected turn (deg) | Error (deg) |
|---|---:|---:|---:|---:|---:|---:|
| 1 | 1.92 | 2.00 | -0.08 | 88.5 | 90.0 | -1.5 |
| 2 | 1.90 | 2.00 | -0.10 | 88.0 | 90.0 | -2.0 |
| 3 | 1.91 | 2.00 | -0.09 | 87.8 | 90.0 | -2.2 |
| 4 | 1.89 | 2.00 | -0.11 | 88.2 | 90.0 | -1.8 |

### B.2 — Written answers

- The turning error is larger than the straight-line error. In this run the heading error was about 2° per turn, while the straight-line displacement error was about 0.1 m per side, which is a smaller effect over a 2 m leg.
- A 3° heading error at the first corner gives a lateral error of $2.0 \times \sin(3°) \approx 0.10$ m after one side.
- If square_driver used /odom, it could correct drift only if it knew the difference between the commanded motion and the measured motion. In practice it would need the current pose estimate and the error between the true and expected pose, which it currently does not have.
- The robot is stationary but /odom still reports a pose because the wheel encoders and the motion model continue to estimate position from the last commanded motion. If the wheel encoders are wrong or the robot slips, the estimate can be wrong even while standing still.

### B.3 — Prediction for Session 2

I predict the error-vs-distance graph will flatten once the LiDAR-based loop closure corrects the pose estimate, because the map-based corrections will keep the drift from growing steadily.

## Part C

### Change made

I updated the square driver so it can stop after a configurable number of laps, publish a zero twist, and shut down cleanly instead of running forever.

### Verification

- Build: `colcon build --symlink-install --packages-select acadbot_control`
- Runtime verification: the driver printed:
  - `Completed a full loop (4 sides). Watch the odometry drift in RViz!`
  - `Completed a full loop (8 sides). Watch the odometry drift in RViz!`
  - `Completed 2 laps; stopping square_driver cleanly.`
- Without the zero twist, the robot would continue moving briefly after the node shut down because the last non-zero velocity command would remain active until the controller stopped receiving commands. The zero twist is therefore necessary to make the robot stop immediately.
