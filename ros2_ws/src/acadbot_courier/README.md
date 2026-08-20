# acadbot_courier

AcadBot as a courier. Someone asks for a delivery between two **named** places —
*"take this from reception to lab_bench"* — and the robot accepts the job, drives
to the pickup, then to the dropoff, says what it is doing the whole way, stops
cleanly if the requester changes their mind, and tells the truth when it fails.

The driving itself is Nav2's. This package is the mission layer on top of it.

---

## Run it

```bash
ros2 launch acadbot_courier courier.launch.py
```

That brings up the simulation, the saved map, AMCL, the Nav2 servers, RViz and
the courier. No **2D Pose Estimate** click: `initial_pose_seeder` tells AMCL
where the robot starts and checks that it worked. Nav2 reaches `active` about
20 seconds in.

Then, in another terminal:

```bash
# book a job -- returns immediately with an id, or a reason it was refused
ros2 service call /request_delivery acadbot_courier_msgs/srv/RequestDelivery \
  "{pickup: 'charging_dock', dropoff: 'storage'}"

# run it -- streams progress, and Ctrl-C cancels
ros2 action send_goal /execute_delivery \
  acadbot_courier_msgs/action/ExecuteDelivery "{job_id: 'job_0001'}" --feedback
```

Arguments: `headless:=true` (no Gazebo GUI), `rviz:=false`, `map:=<path>`,
`nav2_delay:=<s>`, `initial_x`/`initial_y`/`initial_yaw`.

---

## The two interfaces, and why they are two

**`RequestDelivery` (service)** books a job. Booking is a transaction: it either
succeeds or it does not, it takes no measurable time, and there is nothing to
report while it happens. The reply is immediate — an id, or a refusal that says
why.

**`ExecuteDelivery` (action)** runs the job. It takes minutes, it has something
to say the whole time, and it must be interruptible. Those three properties are
exactly what an action provides and a service does not.

The goal carries only the `job_id`. The pickup and dropoff **poses are resolved
once, when the job is booked, and frozen into it** — looking the names up again
at execution time would let an edit to `courier.yaml` move the target of a job
that had already been accepted.

Feedback carries the leg, the location name it is driving to, the distance
left, which attempt this is, how many recoveries Nav2 has run, and the state
(`NAVIGATING` / `RETRYING` / `CANCELING`). It is published on the courier's own
timer, not on Nav2's, so a stalled Nav2 shows up as a frozen distance rather
than as silence.

---

## Nodes

| node | job |
|---|---|
| `courier_server` | the service, the action server, and the Nav2 client |
| `initial_pose_seeder` | tells AMCL where the robot starts, so the demo needs no mouse |

---

## Configuration

Everything tunable is in `config/courier.yaml`. Nothing about the building or
the robot's limits is compiled in — adding a room is an edit, not a rebuild.

| parameter | default | meaning |
|---|---|---|
| `locations` | 4 rooms | `name: [x, y, yaw]` in the **map** frame |
| `max_retries` | 2 | per leg, so up to 3 attempts each |
| `retry_delay` | 3.0 s | between attempts on the same leg |
| `feedback_period` | 0.5 s | the action promises at least 1 Hz |
| `leg_timeout` | 120 s | before an attempt is abandoned and retried |
| `goal_frame` | `map` | frame the goals are sent in |

Coordinates are in the **map** frame, whose origin is where the robot stood when
Session 2 mapping started — its spawn point at Gazebo `(-3, -2)`. So
`map = gazebo + (3, 2)`. Read new coordinates off RViz's **Publish Point** and
keep them at least the costmap `inflation_radius` (0.45 m) clear of any wall.

Localization and the Nav2 servers both take the course's own
`acadbot_navigation/config/nav2_params.yaml`, untouched: nothing about AMCL,
the planner, the controller or the recovery behaviours changes here.

---

## What actually works

Measured, not assumed.

| route | outcome |
|---|---|
| `charging_dock → storage` | **succeeds**, ~50 s, 0 recoveries, crosses the corridor |
| `reception → lab_bench` | pickup succeeds; **dropoff fails** — see below |

`reception → lab_bench` is kept deliberately, because failing well is a
requirement. Coming from the far north-west the robot runs down the divider's
west face and clips the tip while rounding it, ending at map `(3.25, 0.90)` —
0.175 m from a wall with a 0.20 m footprint radius, so its own footprint is
inside the wall. `navfn` then cannot plan out of its own start pose. The courier
retries three times and reports:

```
success: false
message: 'the dropoff leg failed after 3 attempts: Nav2 aborted the goal'
failed_leg: dropoff
```

Slowing the controller to 0.20 m/s stops it wedging hard enough to break the
planner, but does not get it round the corner — `nav2_params.yaml` sets
`BaseObstacle.scale` to 0.02, so DWB weights obstacle proximity at almost
nothing. Tuning that is out of scope here.

---

## Verifying

```bash
# 1. it comes up unattended
ros2 lifecycle get /bt_navigator          # active [3], ~20s after launch

# 2. bad requests are refused, with a reason
ros2 service call /request_delivery acadbot_courier_msgs/srv/RequestDelivery \
  "{pickup: 'kitchen', dropoff: 'storage'}"
#   accepted=False, reason="unknown pickup 'kitchen'; known locations are: ..."

# 3. a delivery runs and reports
ros2 service call /request_delivery acadbot_courier_msgs/srv/RequestDelivery \
  "{pickup: 'charging_dock', dropoff: 'storage'}"
ros2 action send_goal /execute_delivery \
  acadbot_courier_msgs/action/ExecuteDelivery "{job_id: 'job_0001'}" --feedback

# 4. cancel stops it
#    Ctrl-C the send_goal above while it drives; expect status CANCELED,
#    success false, failed_leg empty, and the robot stationary.
```

The locations are drawn in RViz as arrows with their names, with the current
target highlighted. Shown over the global costmap, a location sitting inside the
inflation band is visible before it costs anyone a failed delivery — which is
how two of the four were found to be badly placed.

---

## Known limits

- **One job at a time.** A running job owns the robot; a second booking is
  refused with a reason. Two deliveries on one base is not a thing this
  supports.
- **Finished jobs are remembered for the life of the process**, so a replayed
  `job_id` is refused rather than delivered twice. The map grows without bound,
  which is fine at demo scale and would not be in a real building.
- **Arrival means within Nav2's `xy_goal_tolerance`**, 0.20 m — the honest
  definition of "delivered" here.

---

## Bonus: Behavior Tree Engine (`mission:=bt`)

This package provides two interchangeable implementations of the courier mission runner:
1. **FSM State Machine (Default):** `courier_server`
   ```bash
   ros2 launch acadbot_courier courier.launch.py mission:=fsm
   ```
2. **Behavior Tree (Bonus):** `courier_bt_server` using `BehaviorTree.CPP v4` and `behavior_trees/courier.xml`:
   ```bash
   ros2 launch acadbot_courier courier.launch.py mission:=bt
   ```
Both implementations expose the identical `/request_delivery` service and `/execute_delivery` action.

