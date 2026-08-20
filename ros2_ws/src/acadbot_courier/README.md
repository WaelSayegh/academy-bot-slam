# acadbot_courier

The **C++ courier** for the Session 4 autonomy project: a ROS 2 node that takes
a *pickup → dropoff* job, navigates there with Nav2, and reports an honest
success/failure result — including a behavior-tree bonus track where the whole
mission is expressed as a runtime-loaded BehaviorTree.CPP tree.

It depends on the message/action definitions in
[`acadbot_courier_interfaces`](../acadbot_courier_interfaces).

---

## How it works

```
  courier_client                                 courier_server
   │   /courier/accept_job  (srv)                  │
   ├──────────────────────────────────────────────►│  validate job → job_id
   │                                               │
   │   /courier/execute_delivery (action)          │
   ├──────────────────────────────────────────────►│  run mission (BT or loop)
   │                                               │     │
   │◄──────────────────────────────────────────────┤     │  navigate_to_pose (action)
   │        feedback / result                      │     ▼
   │                                               │  bt_navigator + controller
   │                                               │  (Nav2 owns recovery:
   │                                               │   clear → spin → back-up → wait)
```

1. A **client** calls `courier/accept_job` with a pickup and a dropoff location.
   The server validates the names (against `location_names`), rejects duplicates
   and busy requests, and replies with a numeric `job_id`.
2. The client sends an `ExecuteDelivery` action goal with that `job_id`.
3. The server walks the two legs — first to pickup, then to dropoff — by sending
   `navigate_to_pose` goals to Nav2. Each leg is retried up to `retry_count`
   times with a `retry_delay_sec` pause between attempts.
4. Feedback (current leg, target, distance remaining) streams back once per
   `feedback_period_sec`. The result reports `success`, the `failed_leg` (if
   any) and the total `attempts` spent.

### Two mission engines (`mission_mode`)

| Mode | Description |
|---|---|
| `"bt"` (default) | The mission is a BehaviorTree.CPP tree loaded at runtime from [`config/courier_bt.xml`](config/courier_bt.xml). The only custom leaf is `DriveToLocation`; everything else (retry budget, leg timeout, pause) is built-in BT.CPP. Edit the XML and restart — no recompile. |
| `"loop"` | A hand-written C++ for-loop in `courier_server.cpp` that replicates the same retry logic. Kept as a readable baseline/reference. |

### Behavior tree

```xml
Sequence
├── RetryUntilSuccessful {retry_count}
│   └── Fallback
│       ├── Timeout {leg_timeout_msec}
│       │   └── DriveToLocation (pose, location_name, leg="pickup")
│       └── Delay {retry_delay_msec} → AlwaysFailure
└── RetryUntilSuccessful {retry_count}      (same shape, leg="dropoff")
    └── Fallback …
```

- `RetryUntilSuccessful` is the per-leg retry budget (`retry_count`).
- The `Fallback` tries the leg; on failure it falls through to a `Delay` +
  `AlwaysFailure` so the whole subtree fails once and the retry decorator starts
  the next attempt.
- `Timeout` bounds a single attempt (`leg_timeout_msec`) so a stuck goal cannot
  hang the mission.
- **Recovery is split deliberately:** Nav2's `bt_navigator` owns navigation-level
  recovery (clear costmaps → spin → back-up → wait) through its
  `default_bt_xml` recovery tree. The mission tree has *no* recovery leaves — a
  failing leg just retries, then gives up honestly.

Blackboard entries seeded by the server (`courier_server.cpp`):
`pickup_name`, `dropoff_name`, `pickup_pose` / `dropoff_pose` (`"x,y,yaw"` in
the `frame_id` frame), `retry_count` (int), `leg_timeout_msec` and
`retry_delay_msec` (unsigned int), `attempts`, `failed_leg`.

> Note: BT.CPP ports are type-strict — `Timeout.msec` / `Delay.delay_msec` are
> `unsigned int` and `RetryUntilSuccessful.num_attempts` is `int`. The blackboard
> values must match or `createTreeFromFile()` throws at load time.

### Custom node: `DriveToLocation`

`src/courier_bt_nodes.hpp` — a `StatefulActionNode` that:

- resolves its target from the `pose` port (`"x,y,yaw"`) or, failing that, from
  a named location in the `CourierNavContext`;
- sends a Nav2 `navigate_to_pose` goal on a worker thread and streams feedback
  through the `on_feedback` callback;
- honours cancellation (its own halt, or the server's cancel request).

---

## Build

```bash
cd /ros2_ws
colcon build --symlink-install --packages-select acadbot_courier_interfaces acadbot_courier
source install/setup.bash
```

## Run

Start the whole stack (sim + Nav2 + AMCL + RViz + courier server) — this is the
same stack as Session 4 autonomy, plus the courier and an auto `/initialpose`:

```bash
ros2 launch acadbot_courier courier.launch.py            # with GUI
ros2 launch acadbot_courier courier.launch.py headless:=true   # no Gazebo GUI
ros2 launch acadbot_courier courier.launch.py rviz:=false      # skip RViz
```

Then, in a second terminal, run a job:

```bash
ros2 run acadbot_courier courier_client \
    --ros-args -p pickup_name:=reception -p dropoff_name:=lab_bench
```

### Client options

| Parameter | Default | Meaning |
|---|---|---|
| `pickup_name` | `reception` | Pickup location name |
| `dropoff_name` | `lab_bench` | Dropoff location name |
| `cancel_after_sec` | `0.0` | If > 0, send a cancel request after this many seconds (handy for testing cancellation). |

**Ctrl-C** cancels the delivery cleanly: the client sends `CancelGoal`, the
server stops the robot, and the client exits. Exit codes: `0` success, `2`
canceled, `1` anything else (rejected job, delivery failure, timeout).

---

## Configuration (`config/courier.yaml`)

| Parameter | Default | Meaning |
|---|---|---|
| `mission_mode` | `"bt"` | `"bt"` = behavior tree, `"loop"` = hand-written loop |
| `frame_id` | `"map"` | Frame for navigation goals |
| `retry_count` | `5` | Max attempts per leg |
| `retry_delay_sec` | `2.0` | Pause between attempts |
| `leg_timeout_sec` | `120.0` | Single-attempt timeout (fed to the BT `Timeout` as `leg_timeout_msec`) |
| `feedback_period_sec` | `1.0` | Feedback publish rate |
| `busy_reject` | `true` | Reject new jobs/goals while one is running |
| `location_names` | `["reception", "lab_bench"]` | Valid job locations |
| `location_poses` | `[0.6, 4.2, 0, 5.5, 0.6, 0]` | `x, y, yaw` per name |

Location poses are in the **map** frame. AcadBot spawns at Gazebo `(-3, -2)`, so
`map = gazebo + (3, 2)`; read real coordinates off RViz's *Publish Point*.

## Interfaces

`acadbot_courier_interfaces`:

- **`AcceptJob.srv`** — `pickup_name`, `dropoff_name` → `accepted`, `job_id`, `reason`
- **`ExecuteDelivery.action`**
  - Goal: `job_id`
  - Result: `success`, `failed_leg`, `attempts`
  - Feedback: `leg`, `target_location`, `distance_remaining`, `note`

## Demo ideas

- **Normal delivery** — run the client; watch feedback print once a second until
  `Result: success=true`.
- **Recovery** — block the route with a box/chair mid-drive. Nav2 should run its
  recovery cycle (clear costmap → spin → back-up → wait → replan) and find a way
  around. Confirm the recovery tree is loaded with:
  ```bash
  ros2 param get /bt_navigator default_bt_xml
  ```
  (should show `navigate_to_pose_w_replanning_and_recovery.xml`).
- **Retry** — block the path *and* the detour; the leg will exhaust
  `retry_count` attempts (visible in the server log and `attempts` in the
  result) and the delivery aborts with the failing leg named.
- **Cancellation** — `ctrl-c` the client mid-delivery, or run with
  `-p cancel_after_sec:=10`.

## Files

| File | Role |
|---|---|
| `src/courier_server.cpp` | Action server, job validation, BT/loop mission engines |
| `src/courier_bt_nodes.hpp` | `DriveToLocation` leaf + `CourierNavContext` |
| `src/courier_client.cpp` | Example client (job → delivery → report) |
| `config/courier.yaml` | Server parameters |
| `config/courier_bt.xml` | Runtime behavior-tree mission definition |
| `launch/courier.launch.py` | One-command full-stack launcher |