
# Navigation and Courier — Detailed Reference

This document explains the courier package, navigation stack configuration,
and the key concepts used in this repository. It links to the code so you can
read the implementation after understanding the high-level ideas.

## Purpose

- Explain what the courier package does and where its configuration lives.
- Explain AMCL vs SLAM and where map formats are used in this repo.
- Summarise Nav2 components used here (global planner, local planner,
  costmaps, behavior/recovery actions) and point to their params in the
  workspace.

## Courier package (high level)

Files to inspect:

- Dispatcher and executor logic: [ros2_ws/src/acadbot_courier/src/dispatcher.cpp](ros2_ws/src/acadbot_courier/src/dispatcher.cpp)
- Execution helpers / interfaces: [ros2_ws/src/acadbot_courier/include/executor.hpp](ros2_ws/src/acadbot_courier/include/executor.hpp)
- Runtime configuration (named locations, retry limits): [ros2_ws/src/acadbot_courier/config/courier_params.yaml](ros2_ws/src/acadbot_courier/config/courier_params.yaml)
- Launch wrapper (one-command bringup): [ros2_ws/src/acadbot_courier/launch/courier.launch.py](ros2_ws/src/acadbot_courier/launch/courier.launch.py)

What it does:

- Accepts a delivery job (pickup + dropoff by name), looks up the named
  locations from `courier_params.yaml`, and runs two Nav2 `navigate_to_pose`
  action goals (pickup then dropoff).
- Implements retry logic and timeouts configured in the YAML (`retry_limit`,
  `leg_timeout_sec`). These parameters control how many times a failed leg
  is retried before the whole job is declared FAILED.
- The `dispatcher` node is the orchestrator; `executor` contains reusable
  helper functions used by the dispatcher for building goals, calling Nav2
  actions, and publishing feedback.

Notes about `courier_params.yaml`:

- `location_names`: canonical list of place names the courier understands.
- `locations.<name>.(x,y,yaw)`: map-frame coordinates in metres and radians.
  These coordinates are used to construct `geometry_msgs/PoseStamped` goals
  for the Nav2 `navigate_to_pose` action.

If you want line-by-line comments in the `dispatcher.cpp`/`executor.cpp` code,
I can add an annotated copy or inline comments — confirm and I will proceed.

## Maps, SLAM and AMCL

Where the maps live:

- Occupancy-grid (for AMCL / map_server): [ros2_ws/src/acadbot_navigation/maps/academy_map_v2.yaml](ros2_ws/src/acadbot_navigation/maps/academy_map_v2.yaml)
  and [ros2_ws/src/acadbot_navigation/maps/academy_map_v2.pgm](ros2_ws/src/acadbot_navigation/maps/academy_map_v2.pgm)
- slam_toolbox serialized map (for SLAM-mode localization): the serialized
  files in the same folder (`*.data` and `*.posegraph`).

Key difference — SLAM vs AMCL (plain language):

- SLAM (simultaneous localization and mapping) builds a map while the robot
  is moving and also estimates the robot's trajectory. A SLAM system (like
  `slam_toolbox`) records a pose graph (robot poses and constraints between
  them) and optimizes that graph to produce a consistent map. The pose graph
  contains where the robot has been and loop-closure constraints; it is a
  full representation for re-localisation and map correction.
- AMCL (Adaptive Monte Carlo Localization) does not build a map — it
  assumes a static occupancy-grid map already exists and uses a particle-
  filter to estimate the robot's pose on that map. AMCL's job is to match
  sensor scans to the map and output a best estimate (with covariance) of
  where the robot is.

In short: SLAM = build+refine the map + trajectory; AMCL = localise on a
static map (no mapping / pose graph optimization). The repository uses the
occupancy-grid `academy_map_v2.pgm`/`.yaml` for AMCL-based localization and
keeps serialized `slam_toolbox` outputs for alternate workflows.

## Nav2 components used in this repo

Primary Nav2 param file in this workspace:

- [ros2_ws/src/acadbot_navigation/config/nav2_params.yaml](ros2_ws/src/acadbot_navigation/config/nav2_params.yaml)

Important components and where they are configured:

- AMCL: particle-filter based localization. Configured under the `amcl`
  section of `nav2_params.yaml`. Key params: `min_particles`, `max_particles`,
  `laser_max_range`, and `update_min_d`/`update_min_a` (how often AMCL updates
  based on motion).
- Global planner: configured under `planner_server`. This repo uses
  `nav2_navfn_planner::NavfnPlanner` (exposed here as `GridBased`). The
  Navfn planner is a grid-based planner that can run as Dijkstra (uniform
  cost) or A*; the active mode is controlled by `use_astar` in
  `nav2_params.yaml` (`false` → Dijkstra, `true` → A*). See
  `planner_server` → `GridBased` in the params file.
- Local planner (controller): this repo uses the DWB local planner
  (`dwb_core::DWBLocalPlanner`) configured under `controller_server`. DWB
  scores sampled velocity trajectories, simulates them forward for `sim_time`,
  and picks the best trajectory according to the configured critics
  (`PathAlign`, `GoalDist`, `BaseObstacle`, etc.). See `FollowPath` in the
  params file for the critic weights and sampling settings.
- Costmaps: two costmaps are used — `global_costmap` (anchored to `map` and
  covering the whole known map) and `local_costmap` (rolling window anchored
  to `odom`, follows the robot). Each costmap is a stack of layers:
  `static_layer` (from the occupancy-grid map), `obstacle_layer` (from
  sensors, e.g. `/scan`) and `inflation_layer` (grows obstacle regions to
  account for robot size). See the `local_costmap` and `global_costmap`
  sections in `nav2_params.yaml` for exact settings.
- Recovery behaviors: defined under `behavior_server`; common ones in this
  repo are `spin` (rotate in place), `backup` (reverse), `drive_on_heading`
  and `wait`. Nav2 will try configured recovery behaviors on planner or
  controller failures before giving up — these are critical to the courier's
  retry logic.

## Where the repo wires Nav2 and the courier together

- `ros2_ws/src/acadbot_courier/launch/courier.launch.py` includes the
  higher-level bringup (`acadbot_bringup/launch/autonomy.launch.py`) with
  `localization:=amcl` and then starts the `dispatcher` node (the courier
  orchestrator). The dispatcher reads `courier_params.yaml` for named
  locations and retry policies.

## Reading the code (recommended order)

1. `courier_params.yaml` — understand what can be tuned without code changes.
2. `dispatcher.cpp` — the high-level mission logic: validate names, create
   Nav2 goals, retry, timeouts and feedback.
3. `executor.cpp` / `executor.hpp` — lower-level Nav2 client calls and
   helpers used by the dispatcher.
4. `nav2_params.yaml` — learn how AMCL, planners, costmaps and behaviors are
   configured for the robot in this project.

If you want, I can now add line-by-line comments to `dispatcher.cpp` and
`executor.cpp` explaining each statement and variable for someone reading
them for the first time. This would be a large patch — confirm and I'll
annotate those two files first, then proceed to `nav2_params.yaml` and the
rest of the courier package.

---

Generated by the assistant. If you'd like the full per-line annotations,
reply "annotate dispatcher and executor" and I'll start adding inline
comments in those files.
