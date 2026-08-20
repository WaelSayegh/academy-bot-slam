# tools

Test harnesses. Not part of any package, not built by colcon — run them with
`python3` from the repo root inside the container.

## `fake_nav2.py`

A stand-in `navigate_to_pose` action server for testing `acadbot_courier`
without Gazebo.

The courier's own logic — booking, the leg sequence, retries, timeouts,
cancellation, what it reports when it fails — has nothing to do with driving.
Against the real stack each test costs a minute of startup and gives you
whatever the planner felt like doing that time, which makes a failure hard to
reproduce and a fix hard to confirm. This answers instantly and does exactly
what it is told, so a full two-leg delivery takes about three seconds and an
abort happens on the goal you choose.

It is not a simulator. Nothing moves; there is no map, no costmap, no TF. It
answers one question: *does the courier do the right thing when Nav2 says X.*

```bash
source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash
CFG=/ros2_ws/install/acadbot_courier/share/acadbot_courier/config/courier.yaml

python3 tools/fake_nav2.py &
ros2 run acadbot_courier courier_bt_server \
  --ros-args --params-file $CFG -p use_sim_time:=false -p retry_delay:=1.0 &

ros2 service call /request_delivery acadbot_courier_msgs/srv/RequestDelivery \
  "{pickup: charging_dock, dropoff: storage}"
ros2 action send_goal /execute_delivery \
  acadbot_courier_msgs/action/ExecuteDelivery "{job_id: job_0001}" --feedback
```

`use_sim_time:=false` because there is no `/clock` here, and `retry_delay:=1.0`
so the retry pauses do not dominate the run. Swap `courier_bt_server` for
`courier_server` to test the state machine instead — both engines run against
it unchanged, which is how their equivalence was checked.

### Behaviour is set by environment variables

| variable | effect |
|---|---|
| `FAIL_GOALS=1,3` | abort the 1st and 3rd goals, succeed on the rest. Empty (default) succeeds every time. |
| `DRIVE_TIME=1.5` | seconds of pretend driving per goal. |
| `ACCEPT_DELAY=3` | stall this long before acknowledging a goal. |

Each one exists because it isolates a specific failure:

**`FAIL_GOALS=1,3`** forces exactly one retry on each leg, which is how you see
whether the attempt counter restarts when the mission moves from the pickup to
the dropoff. It should read `pickup 1, pickup 2, dropoff 1, dropoff 2` with
`attempts_used: 4`. A counter that never resets reads `dropoff 3, dropoff 4`
instead — the mission still works, so nothing looks wrong until you read the
feedback.

**`FAIL_GOALS=1,2,3`** exhausts a leg, so the courier has to report an honest
failure: `success: false`, `failed_leg: pickup`, and the dropoff never
attempted.

**`ACCEPT_DELAY=3`** holds open the window between *goal sent* and *goal
acknowledged*. Cancel inside it and there is no goal handle to cancel yet. A
courier that does not remember the cancel stops its own mission and leaves the
robot driving — the client is told CANCELED while the robot keeps going, which
is the worst kind of bug to find on real hardware and is invisible on a stack
that acknowledges in 5 ms. Cancel with Ctrl-C on the `send_goal` above, and
expect a line saying the goal was acknowledged after the halt and cancelled
anyway.

### A note on the harness itself

The action server is reentrant and spun by a `MultiThreadedExecutor`. That is
not decoration: `execute()` blocks while it pretends to drive, and a
single-threaded executor sitting inside it cannot service the cancel request
meant to interrupt it. The goal runs to completion and the cancel is handled
afterwards — which looks exactly like a courier that ignores cancellation. The
same trap applies to calling `spin_once()` from inside the callback: it raises
`Executor is already spinning`, which the action server reports as an aborted
goal. Both were hit while writing this, and both read as bugs in the code under
test rather than in the harness.
