# acadbot_courier

Final project — AcadBot as a delivery courier inside the mapped building.

A requester names a **pickup** and a **dropoff** by location name. The robot
accepts or rejects the job, drives the two legs over Nav2, streams progress
while it moves, and reports the truth — including which leg failed, if one
did. Built on top of the map (Session 2) and AMCL localization (Session 3)
already in this repo.

## Packages

| Package | Contents |
|---|---|
| `acadbot_courier_msgs` | Interfaces only — `RequestDelivery.srv`, `ExecuteDelivery.action`, `NamedLocation.msg`. No node code. |
| `acadbot_courier` | The `dispatcher` node, its config, and the launch file. |

## Interfaces

**`RequestDelivery`** (service) — instant accept/reject.
```
string pickup
string dropoff
---
bool accepted
string reason
string job_id
```

**`ExecuteDelivery`** (action) — the actual drive, minutes long, cancellable,
streamed.
```
# goal
string job_id
NamedLocation pickup
NamedLocation dropoff
---
# result
bool success
string result_message
string failed_leg
---
# feedback (>=1/sec)
string current_leg
NamedLocation heading_to
float64 distance_remaining
```

A service answers a question immediately; an action carries out a job that
runs for a while, reports progress, and can be told to stop. That split is
why the two are separate interfaces instead of one.

## Node architecture

`dispatcher` is the only ROS node in this package:

- hosts the `RequestDelivery` service server and the `ExecuteDelivery` action
  server
- owns a queue of jobs (a second request while one is running gets queued,
  not rejected)
- owns an `Executor` — a plain C++ class, **not** a node — which wraps a
  single `navigate_to_pose` action client to Nav2 (the same one
  `patrol_commander` uses)
- per-leg retry (`retry_limit`) and per-leg timeout (`leg_timeout_sec`),
  both YAML-configured

One node, one process, one Nav2 client — there's no separate executor node,
since a second action layer between dispatcher and Nav2 wouldn't do
anything for a single physical robot.

## Configuration — `config/courier_params.yaml`

**In YAML, changeable without touching code:**
- `location_names` + `locations.<name>.{x,y,yaw}` — named delivery points
- `retry_limit` — per-leg retry count
- `leg_timeout_sec` — per-leg timeout before a leg is treated as failed

**Left fixed on purpose:**
- one dispatcher node / one Nav2 client — no runtime concurrency toggle
- global planner choice (A*/Dijkstra via `use_astar`) — a launch-time Nav2
  param, not something the courier interface exposes per job
- DWB speed and goal tolerances — tuned once for this robot, not per delivery

## Build

```bash
cd /ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Run — one command

```bash
ros2 launch acadbot_courier courier.launch.py
```

Brings up: simulation, AMCL (auto-localized at the known spawn pose — no
manual 2D Pose Estimate needed), Nav2, and the `dispatcher` node. RViz opens
by default.

---

## Demo script

Four scenarios, matching the assignment's live-run requirements. Run these
from a second terminal once the launch above is up and settled.

### 1. Successful delivery, end to end
```bash
ros2 service call /request_delivery acadbot_courier_msgs/srv/RequestDelivery \
  "{pickup: reception, dropoff: lab_bench}"
# note the job_id in the response, then:
ros2 action send_goal /execute_delivery acadbot_courier_msgs/action/ExecuteDelivery \
  "{job_id: 'job-0'}" --feedback
```
Watch the feedback stream `current_leg` flip from `pickup` to `dropoff`, with
`distance_remaining` counting down each leg. Result comes back `success: true`.

### 2. Rejected request — unknown location
```bash
ros2 service call /request_delivery acadbot_courier_msgs/srv/RequestDelivery \
  "{pickup: reception, dropoff: kitchen}"
```
Expect an immediate `accepted: false`, `reason` naming `kitchen`, empty
`job_id`. No crash, no hang.

### 3. Cancel mid-drive
Send a delivery with a long leg (e.g. `reception` → `far_corner`), then while
it's driving, `Ctrl+C` the `action send_goal` call (or cancel from another
terminal with `ros2 action cancel`). Expect the robot to stop within about a
second, the dispatcher log to show `canceled by requester`, and the action
result to be `CANCELED` — not `SUCCEEDED`, and not retried.

### 4. Blockage → Nav2 recovery → retry → honest failure
Start a delivery, then drop an obstacle in Gazebo on the planned path.
Expect, in order:
- `behavior_server` running `spin` / `backup` / `wait` to try to clear it
- if still blocked, the leg comes back `ABORTED` from Nav2
- the dispatcher retries the same leg, up to `retry_limit`
- once retries are exhausted, the job ends with `success: false` and
  `failed_leg` naming which leg it was — never a false `SUCCEEDED`

### Verify the localization prerequisite (optional, before demo 1)
```bash
ros2 lifecycle get /map_server   # active [3]
ros2 lifecycle get /amcl         # active [3]
ros2 run tf2_ros tf2_echo map odom   # resolves immediately — spawn pose is set automatically
```

---

## What broke, and what fixed it

**Map path mismatch.** After merging the mapping branch, the launch file
still pointed at the old map filename — AMCL never localized, every run
throwing `Invalid frame ID "map"`. Fixed by correcting the path to
`academy_map_v2` and adding `set_initial_pose` at the known spawn point, which
also removed the need to click 2D Pose Estimate before every test.

**Cancel vs. timeout looked identical.** A requester cancel and an internal
leg-timeout both cancel the underlying Nav2 goal the same way, and both come
back as `CANCELED` from the action client. Left alone, a timeout would have
ended the whole job instead of being retried like any other failure. Fixed
with a `leg_timed_out` flag set only by the leg's own timer, checked before
deciding whether a `CANCELED` result means "stop for good" or "retry."

## Bonus — behavior tree

Not finished. `config/mission.xml` and one leaf-node header
(`include/drive_leg_node.hpp`) exist on the `final_behavior_tree` branch, but
the mission logic isn't wired into `BehaviorTree.CPP` — the working
hand-written dispatcher above is what's demoed. Sequenced deliberately last,
and the nine core requirements were prioritized over rebuilding a working
system on a new framework the night before presenting.

## Requirements checklist

| # | Requirement | Status |
|---|---|---|
| 1 | Accept jobs through a service | done |
| 2 | Reject bad jobs, with a reason | done |
| 3 | Run jobs through an action | done |
| 4 | Feedback >=1/sec | done |
| 5 | Drive with Nav2 (`navigate_to_pose` client) | done |
| 6 | Cancel means stop | done |
| 7 | Handle failure honestly | done |
| 8 | Everything configurable is in YAML | done |
| 9 | One command brings it up | done |
| bonus | Mission logic as a behavior tree | started, not finished |