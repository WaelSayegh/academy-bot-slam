# AcadBot Courier

`acadbot_courier` turns AcadBot into a two-leg indoor courier. A request service
admits or rejects named pickup/dropoff jobs immediately. A separate action runs
an accepted job, reports progress at a fixed rate, retries failed Nav2 legs, and
propagates cancellation to the underlying `navigate_to_pose` goal before ending.

The robot has no manipulator. Reaching and dwelling at a configured pose is the
only claim this package makes about pickup or dropoff.

## Layout

- `include/acadbot_courier/types.hpp` — shared leg, feedback, state, and outcome types.
- `include/acadbot_courier/location_book.hpp` — immutable named-pose lookup.
- `include/acadbot_courier/job_registry.hpp` — thread-safe job IDs and lifecycle.
- `include/acadbot_courier/nav_leg_client.hpp` — the only Nav2-facing class.
- `src/location_book.cpp` — pose lookup and yaw-to-quaternion conversion.
- `src/job_registry.cpp` — pending, active, finished, and expiry transitions.
- `src/nav_leg_client.cpp` — Nav2 send, feedback, timeout, cancel, TF distance fallback, and costmap clearing.
- `src/courier_node.cpp` — request service, delivery action, retries, dwell, and feedback timer.
- `src/delivery_client.cpp` — demo CLI that requests and executes one delivery.
- `src/initial_pose_publisher.cpp` — delayed AMCL initial-pose seed.
- `config/locations.yaml` — named locations and initial pose.
- `config/courier.yaml` — retry, timing, admission, frame, and ROS interface settings.
- `launch/courier_demo.launch.py` — simulation, AMCL, Nav2, and courier one-command launch.

The interfaces are in the separate `acadbot_courier_msgs` package so clients do
not depend on the runtime implementation.

## Configuration

### `locations.yaml`

The named poses reuse the academy map's documented patrol waypoints. The AMCL
seed is the `reception` pose and matches the simulator spawn configured by the
courier launch file.

| Key | Meaning | Initial value |
|---|---|---:|
| `location_names` | Ordered list of accepted location names | four demo names |
| `locations.<name>.x` | Map-frame x coordinate | named waypoint value |
| `locations.<name>.y` | Map-frame y coordinate | named waypoint value |
| `locations.<name>.yaw` | Map-frame heading in radians | named waypoint value |
| `initial_pose.x` | AMCL seed x coordinate | `0.6` |
| `initial_pose.y` | AMCL seed y coordinate | `4.2` |
| `initial_pose.yaw` | AMCL seed heading in radians | `0.0` |
| `initial_pose.covariance_x` | AMCL x variance | `0.25` |
| `initial_pose.covariance_y` | AMCL y variance | `0.25` |
| `initial_pose.covariance_yaw` | AMCL yaw variance | `0.068` |
| `global_frame` | Frame used by the AMCL seed | `map` |
| `delay_sec` | Delay inside the seed helper after it starts | `5.0` |

ROS parameter files select settings by node name, so the initial pose appears in
both the `courier_node` and `initial_pose_publisher` sections. Keep the duplicate
`x`, `y`, and `yaw` values equal.

### `courier.yaml`

| Key | Meaning | Default |
|---|---|---:|
| `max_attempts_per_leg` | Total attempts including the first try | `3` |
| `retry_backoff_sec` | Cancellable pause between attempts | `2.0` |
| `clear_costmap_before_retry` | Clear both costmaps before attempts after the first | `true` |
| `leg_timeout_sec` | Maximum navigation time per attempt | `120.0` |
| `feedback_period_sec` | Courier feedback timer period | `1.0` |
| `job_ttl_sec` | Lifetime of an unclaimed pending job | `60.0` |
| `nav_goal_response_timeout_sec` | Wait for Nav2 availability and goal response | `5.0` |
| `nav_cancel_confirm_timeout_sec` | Wait for Nav2 cancellation and terminal result | `5.0` |
| `costmap_service_timeout_sec` | Per-service costmap clear wait | `2.0` |
| `dwell.pickup` | Stand-in pause for human loading | `3.0` |
| `dwell.dropoff` | Stand-in pause for human unloading | `3.0` |
| `max_concurrent_jobs` | Simultaneous active jobs; must remain one | `1` |
| `queue_depth` | Waiting job queue; must remain disabled | `0` |
| `nav_action_name` | Nav2 action name | `navigate_to_pose` |
| `global_frame` | Navigation goal frame | `map` |
| `robot_base_frame` | TF frame used by distance fallback | `base_link` |
| `global_costmap_clear_service` | Nav2 global clear service | `/global_costmap/clear_entirely_global_costmap` |
| `local_costmap_clear_service` | Nav2 local clear service | `/local_costmap/clear_entirely_local_costmap` |

## Build and launch

```bash
cd /ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch acadbot_courier courier_demo.launch.py
```

The launch file uses the verified `acadbot_bringup/autonomy.launch.py`
arguments: `localization`, `nav2_delay`, `headless`, `rviz`, `x`, `y`, and
`yaw`. Courier-facing `spawn_x`, `spawn_y`, and `spawn_yaw` arguments default to
Gazebo `(-2.4, 2.2, 0.0)`, corresponding to map-frame reception
`(0.6, 4.2, 0.0)`. It defaults to AMCL and staggers startup behind the base stack.

## Demo scenarios

### Successful delivery

```bash
ros2 run acadbot_courier delivery_client reception lab_bench
```

Expect one feedback line per second, first for `pickup`, then `dropoff`, followed
by `outcome=SUCCEEDED success=true` only after both poses are reached.

### Unknown-location rejection

```bash
ros2 service call /courier_node/request_delivery \
  acadbot_courier_msgs/srv/RequestDelivery \
  "{pickup: 'kitchen', dropoff: 'lab_bench'}"
```

Expect `accepted: false`, an empty job ID, and a reason naming `kitchen` plus the
known locations.

### Cancellation while driving

```bash
ros2 run acadbot_courier delivery_client reception lab_bench
```

Press Ctrl-C while the robot is moving. The client sends an action cancellation
and waits. Expect the robot to stop and the terminal result to contain
`outcome=CANCELED success=false`.

### Blockage and honest failure

Temporarily lower `max_attempts_per_leg` or `leg_timeout_sec` if needed for a
short demo, launch the stack, start a valid delivery, and place an obstacle in
Gazebo that prevents the active leg from completing:

```bash
ros2 run acadbot_courier delivery_client reception lab_bench
```

Expect Nav2 recoveries, courier retries, and finally `NAV_ABORTED` (or `TIMEOUT`)
with `success=false` and `failed_leg` naming the blocked leg. Restore the normal
configuration after the demo.

## Raw interface checks

```bash
ros2 service call /courier_node/request_delivery \
  acadbot_courier_msgs/srv/RequestDelivery \
  "{pickup: 'reception', dropoff: 'lab_bench'}"

ros2 action send_goal /courier_node/deliver \
  acadbot_courier_msgs/action/Deliver \
  "{job_id: 'job_0001'}" --feedback

ros2 param get /courier_node locations.reception.x
ros2 param get /courier_node max_attempts_per_leg
ros2 service list | grep clear
ros2 action --help
```
