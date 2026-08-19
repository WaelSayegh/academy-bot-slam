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
looks good, save it with `map_saver_cli` (see `acadbot_navigation/maps/README.md`)
— Sessions 3 and 4 localize with AMCL on that `.yaml`/`.pgm` pair. Serializing
the pose graph as well is still worth doing if you want to try slam_toolbox's
own localization mode (`localization:=slam`).

### Session 3 — Localization on your saved map (AMCL)
```bash
ros2 launch acadbot_localization localization.launch.py   # sim + map_server + AMCL + RViz + monitor
ros2 run teleop_twist_keyboard teleop_twist_keyboard      # drive; AMCL updates on motion
```
Loads `acadbot_navigation/maps/academy_map.yaml` by default (`map:=` to point
elsewhere). Set the initial pose in RViz (**2D Pose Estimate**); the
`localization_monitor` node logs `sigma` and a `CONVERGED`/`SEARCHING` verdict
once a second. Lost on purpose? Global relocalization:
```bash
ros2 service call /reinitialize_global_localization std_srvs/srv/Empty
```
The camera streams (`/camera/image`, `/camera/depth_image`, `/camera/points`)
are also touched briefly this session — they publish in every simulation run.

### Session 4 — Full autonomy (navigation + recovery + C++ control)
```bash
ros2 launch acadbot_bringup autonomy.launch.py localization:=amcl   # sim + Nav2 + AMCL + RViz
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

### Final project — Courier delivery pipeline

```bash
ros2 launch acadbot_bringup courier.launch.py     # sim + Nav2 + AMCL + courier_manager
ros2 run acadbot_courier_pipeline courier_cli \
  --pickup reception --dropoff lab_bench          # request + drive a delivery, streaming feedback
```

One C++ node, `courier_manager`, sits on top of the Session-4 autonomy stack:
it accepts delivery jobs over the `RequestDelivery` **service** (admission
only — instant accept/reject with a reason), then drives each accepted job as
a cancellable `ExecuteDelivery` **action**, sending Nav2 a `navigate_to_pose`
goal for the pickup leg and then the dropoff leg. A busy `courier_manager`
queues new jobs FIFO (`max_queue_size` in `courier.yaml`) instead of
rejecting them — queued jobs auto-promote as earlier ones finish. `courier_cli`
is a purpose-built operator CLI that wraps both calls into one command and
streams feedback (`leg`, `target`, `distance_left`, `attempt`) at ≥1 Hz;
Ctrl+C sends a clean Nav2 cancel instead of leaving a goal orphaned.
Locations, retry counts and per-leg timeouts are all read from
`acadbot_courier_pipeline/config/courier.yaml`, never compiled in. See
[`COURIER_MANAGER.md`](COURIER_MANAGER.md) for the design.

---

## Packages

| Package | Role |
|---|---|
| `acadbot_description` | URDF/xacro robot model + sensors, RViz configs |
| `acadbot_gazebo`      | Gazebo Harmonic world, robot spawn, `ros_gz` bridge |
| `acadbot_control`     | **C++** nodes: `square_driver` (drift demo), `patrol_commander` (Nav2 client) |
| `acadbot_slam`        | `slam_toolbox` mapping + localization configs/launch |
| `acadbot_localization` | `map_server` + AMCL on a saved map + **C++** `localization_monitor` (built as the Session-2 homework; this is the course-official version) |
| `acadbot_navigation`  | Nav2 params (incl. recovery behaviors), maps, launch |
| `acadbot_bringup`     | One-command launch files per session |
| `acadbot_courier_interfaces` | `RequestDelivery` service + `ExecuteDelivery` action definitions |
| `acadbot_courier_pipeline`   | **Final project.** `courier_manager` (**C++** lifecycle node: service admission + Nav2-driven delivery action) and `courier_cli` (operator CLI) |

See [`PROJECT.md`](PROJECT.md) for the full architecture, the TF tree, the topic
graph and the session-by-session learning outcomes.

**Handing in code?** Every homework in the track has a code deliverable, handed
in as a pull request from your own fork. The workflow — fork, branch, commit,
PR, review — is in [`CONTRIBUTING.md`](CONTRIBUTING.md). Read it once in week 1.

---

## Useful launch arguments

| Argument | Applies to | Default | Why you'd change it |
|---|---|---|---|
| `headless:=true` | `simulation`, `mapping`, `localization`, `autonomy` | `false` | Gazebo server only — no GUI, no GPU. Needed for CI and for machines with no working X. |
| `rviz:=false` | `mapping`, `localization`, `autonomy` | `true` | Skip RViz2 for the same reason. |
| `map:=<path>` | `localization` | `acadbot_navigation/maps/academy_map.yaml` | Localize on a different saved `.yaml` map. |
| `localization:=slam` | `autonomy`, `navigation` | `amcl` | Use slam_toolbox localization mode on a serialized `.posegraph` instead of AMCL on the `.yaml`/`.pgm` grid. Needs that `.posegraph` saved next to the map. |
| `nav2_delay:=<sec>` | `autonomy`, `navigation` | `12.0` | How long to wait for localization before starting Nav2. Raise it if you see `Failed to change state for node: controller_server`. |

## Smoke-testing after a dependency bump

You do not need to drive to prove the SLAM pipeline is alive — the failure mode
worth catching is slam_toolbox not activating, which shows up standing still:

```bash
ros2 launch acadbot_bringup mapping.launch.py headless:=true rviz:=false
ros2 lifecycle get /slam_toolbox          # must be: active [3]
ros2 topic echo /map --once | head -5     # must produce a grid
ros2 run tf2_ros tf2_echo map odom        # must resolve
```

Map *quality* cannot be checked this way — it depends on how you drive. Build
the reference map by hand.

## Known-good state

Everything here was run end to end on a headless container: workspace builds
clean (9/9 packages); the drift demo produces 0.458 m and 47.7° over two laps;
mapping saves and serializes; all eight Nav2 servers reach `active` and a
`navigate_to_pose` goal across the divider corridor succeeds within tolerance.
`courier_manager` reaches `active [3]` under `courier.launch.py` and all four
required scenarios (reject, accept-and-drive, mid-drive cancel, blockage with
retries) plus FIFO queueing (queue-full rejection, cancel-while-queued,
auto-promotion) were verified live; `/execute_delivery` feedback streams at
~96 Hz, well over the 1 Hz requirement.
Full record: `Sessions/_generator/VERIFICATION.md` one level up.
