# AcadBot — Robotics Academy SLAM Track (Summer 2026)

The hands-on code project for the four **SLAM sessions** (Sessions 12–15) of the
Robotics track. Everything runs in **ROS 2 Jazzy + Gazebo Harmonic** inside a
single Docker image, so every student has an identical environment.

**AcadBot** is a small differential-drive robot with a 2D LiDAR, an RGBD camera
and an IMU. Across the four sessions students take it from "raw odometry that
drifts" all the way to "drives itself around a mapped building, recovering when
it gets stuck."

```
   Session 1  ──►  Session 2  ──►  Session 3  ──►  Session 4
  Intro/Drift    LiDAR SLAM      Visual SLAM     Autonomy
   (odometry      (slam_toolbox   (camera, VO,    (Nav2 + recovery
    drift demo)    builds a map)   depth)          + C++ mission node)
```

---

## 1. Build the Docker image (once)

```bash
cd code/academy-bot-slam/docker
./build_ros_image.sh          # builds  acadbot:jazzy
```

This installs ROS 2 Jazzy, Gazebo Harmonic, Nav2, slam_toolbox and the dev
tooling (see `docker/user_install.sh`).

## 2. Start a container

```bash
cd code/academy-bot-slam/docker
./run_ros_container.sh        # opens a shell, mounts ../ros2_ws at /ros2_ws
```

The `ros2_ws/` source tree is **bind-mounted**, so you edit code on your host
and build inside the container. Open extra shells into the same container by
re-running `./run_ros_container.sh`.

## 3. Build the workspace (inside the container)

```bash
cd /ros2_ws
colcon build --symlink-install
source install/setup.bash      # (the container's .bashrc does this for you)
```

---

## What to run, per session

### Session 1 — Odometry & drift
```bash
ros2 launch acadbot_gazebo simulation.launch.py        # robot in Gazebo
ros2 launch acadbot_control square_driver.launch.py    # open-loop C++ driver
rviz2 -d src/acadbot_description/rviz/slam.rviz         # Fixed Frame = odom
```
Drive the open-loop square and watch the estimated pose drift from the true pose.

### Session 2 — LiDAR SLAM with slam_toolbox
```bash
ros2 launch acadbot_bringup mapping.launch.py          # sim + slam_toolbox + RViz
ros2 run teleop_twist_keyboard teleop_twist_keyboard   # drive to build the map
```
Drive **slowly and in straight lines, and never spin in place** — a stationary
rotation gives the scan matcher nothing to match and smears the map. When it
looks good, save it in *both* formats (see `acadbot_navigation/maps/README.md`);
Session 4 loads the serialized pose graph, not the `.pgm`.

**Homework Part B** adds `acadbot_localization` — a standalone package that
loads a saved map and runs `map_server` + AMCL + a small C++ convergence
monitor, without bringing up the rest of Nav2:
```bash
ros2 launch acadbot_localization localization.launch.py \
    map:=/ros2_ws/src/acadbot_navigation/maps/<your_map>.yaml
```
Set the initial pose in RViz (**2D Pose Estimate** / **SetInitialPose**,
depending on toolbar config) before AMCL will publish anything — see
`acadbot_localization/README.md` for details on the monitor node's output.

### Session 3 — Visual SLAM
The robot already publishes `/camera/image`, `/camera/depth_image` and
`/camera/points`. This session is mostly theory + exploring the camera stream
and visual-odometry concepts on top of the same robot.

### Session 4 — Full autonomy (navigation + recovery + C++ control)
```bash
ros2 launch acadbot_bringup autonomy.launch.py         # sim + Nav2 + localization + RViz
ros2 launch acadbot_control patrol.launch.py           # C++ Nav2 action client
```
Set the initial pose in RViz first (**2D Pose Estimate**) until the laser lines
up with the map — nothing downstream works until it does.

Patrol waypoints are in the **map** frame, whose origin is wherever mapping
started. AcadBot spawns at Gazebo `(-3, -2)`, so `map = gazebo + (3, 2)`; read
real coordinates off RViz's *Publish Point* rather than guessing. See
`acadbot_control/config/patrol_waypoints.yaml`.
The C++ `patrol_commander` sends waypoint goals to Nav2; when the robot is
blocked, Nav2's **recovery behaviors** (clear costmap → spin → back up → wait)
kick in. Block its path with a chair in RViz's view to trigger them live.

---

## Packages

| Package | Role |
|---|---|
| `acadbot_description` | URDF/xacro robot model + sensors, RViz configs |
| `acadbot_gazebo`      | Gazebo Harmonic world, robot spawn, `ros_gz` bridge |
| `acadbot_control`     | **C++** nodes: `square_driver` (drift demo), `patrol_commander` (Nav2 client) |
| `acadbot_slam`        | `slam_toolbox` mapping + localization configs/launch |
| `acadbot_navigation`  | Nav2 params (incl. recovery behaviors), maps, launch |
| `acadbot_bringup`     | One-command launch files per session |
| `acadbot_localization` | Standalone `map_server` + AMCL localization launch, with a convergence-monitoring node |

See [`PROJECT.md`](PROJECT.md) for the full architecture, the TF tree, the topic
graph and the session-by-session learning outcomes.

**Handing in code?** Every homework in the track has a code deliverable, handed
in as a pull request from your own fork. The workflow — fork, branch, commit,
PR, review — is in [`CONTRIBUTING.md`](CONTRIBUTING.md). Read it once in week 1.

---

## Useful launch arguments

| Argument | Applies to | Default | Why you'd change it |
|---|---|---|---|
| `headless:=true` | `simulation`, `mapping`, `autonomy`, `localization` | `false` | Gazebo server only — no GUI, no GPU. Needed for CI and for machines with no working X. |
| `rviz:=false` | `mapping`, `autonomy`, `localization` | `true` | Skip RViz2 for the same reason. |
| `localization:=amcl` | `autonomy`, `navigation` | `slam` | Use AMCL + `map_server` on the `.yaml` grid instead of slam_toolbox on the pose graph. |
| `nav2_delay:=<sec>` | `autonomy`, `navigation` | `12.0` | How long to wait for localization before starting Nav2. Raise it if you see `Failed to change state for node: controller_server`. |
| `map:=<path>` | `localization` | `acadbot_navigation/maps/academy_map.yaml` | Point at a different saved `.yaml` map |

## Smoke-testing after a dependency bump

You do not need to drive to prove the SLAM pipeline is alive — the failure mode
worth catching is slam_toolbox not activating, which shows up standing still:

```bash
ros2 launch acadbot_bringup mapping.launch.py headless:=true rviz:=false
ros2 lifecycle get /slam_toolbox          # must be: active [3]
ros2 topic echo /map --once | head -5     # must produce a grid
ros2 run tf2_ros tf2_echo map odom        # must resolve
```

The same idea applies to the localization stack — no driving needed to confirm
the servers themselves are alive:

```bash
ros2 launch acadbot_localization localization.launch.py headless:=true rviz:=false
ros2 lifecycle get /map_server            # must be: active [3]
ros2 lifecycle get /amcl                  # must be: active [3]
```
(`map → odom` will still correctly fail to resolve here until an initial pose
is set — that's expected, not a bug.)

Map *quality* cannot be checked this way — it depends on how you drive. Build
the reference map by hand.

## Known-good state

Everything here was run end to end on a headless container: workspace builds
clean (6/6 packages); the drift demo produces 0.458 m and 47.7° over two laps;
mapping saves and serializes; all eight Nav2 servers reach `active` and a
`navigate_to_pose` goal across the divider corridor succeeds within tolerance.
Full record: `Sessions/_generator/VERIFICATION.md` one level up.
