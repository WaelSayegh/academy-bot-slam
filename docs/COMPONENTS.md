# Every component, explained

A reference for the individual pieces the AcadBot Courier is built from. Each entry gives
**what it is**, **why it is there**, and **the code**, with a link to the file it lives in.

[`REPORT.md`](REPORT.md) covers what was built, what it measured and what broke;
[`BACKGROUND.md`](BACKGROUND.md) builds the underlying theory from first principles. This
document sits between them: one entry per moving part.

The whole design is organised around one rule from the brief: **the courier does not write a
planner or a controller.** Nav2 already plans and drives; `acadbot_navigation`'s
`nav2_params.yaml` is unmodified on this branch — [F3](#f3-nav2_paramsyaml--the-stack-we-plan-against-and-do-not-touch)
documents what it configures and who chose it. Everything here is the mission layer above it:
which goals to send, in what order, what to do when one fails, and how to tell a human.

---

## Contents

| Group | Components |
|---|---|
| [The layout](#the-layout) | the file tree · three executables and their sources · every file, one line each |
| [A. The interfaces](#a-the-interfaces) | `RequestDelivery.srv` · `ExecuteDelivery.action` · why a separate package |
| [B. Value types and the location table](#b-value-types-and-the-location-table) | `Pose2D` / `Leg` / `JobState` · `Job` and frozen poses · parameter-override discovery · the failure paths · `known_names()` |
| [C. The courier node](#c-the-courier-node) | construction · the service handler · job ids · goal acceptance · cancel and the deferred finish · the state machine · retry · the three endings · feedback on its own clock |
| [D. Seeing it](#d-seeing-it) | location markers · `courier.rviz` |
| [E. Localization seeding](#e-localization-seeding) | `initial_pose_seeder` · the advancing-stamp check · the AMCL params file that proved unnecessary |
| [F. Bringup and configuration](#f-bringup-and-configuration) | `courier.launch.py` ordering · `courier.yaml` · the inherited `nav2_params.yaml` (Dijkstra, inflation, DWB critics) |
| [G. The test harness](#g-the-test-harness) | stand-in Nav2 · cancel client · shell drivers |
| [H. The behaviour tree (bonus)](#h-the-behaviour-tree-bonus) | `courier.xml` · `GoToLocation` · `courier_bt_server` · one config block for two engines · `mission:=fsm\|bt` |

Start with [the layout](#the-layout) if you want the map before the territory.

---

# The layout

Two packages, three executables, twenty-eight tracked files. Why it is split this way — rather than
a node dropped into `acadbot_control`, or a Python file in `acadbot_bringup` — is argued in
[`REPORT.md` §2.2](REPORT.md). This section is the map.

```
ros2_ws/src/
├── acadbot_courier_msgs/            THE CONTRACT — what a client depends on
│   ├── srv/RequestDelivery.srv
│   ├── action/ExecuteDelivery.action
│   ├── CMakeLists.txt
│   └── package.xml
│
└── acadbot_courier/                 THE IMPLEMENTATION
    ├── include/acadbot_courier/     headers, one per translation unit
    ├── src/                         three executables' worth of sources
    ├── behavior_trees/courier.xml   the mission, as data (bonus)
    ├── config/                      courier.yaml
    ├── launch/courier.launch.py     the whole stack, one command
    ├── rviz/courier.rviz
    ├── CMakeLists.txt
    ├── package.xml
    └── README.md

tools/                               NOT BUILT — test harness
├── fake_nav2.py
└── README.md

docs/                                BACKGROUND · COMPONENTS · REPORT
```

**Three executables, and which sources make them.** This is the part a file listing does not
tell you:

| executable | built from | what it is |
|---|---|---|
| `courier_server` | `courier_server.cpp` · `location_table.cpp` · `location_markers.cpp` | the courier, state-machine engine |
| `courier_bt_server` | `courier_bt_server.cpp` · `bt_nodes.cpp` · `location_table.cpp` · `location_markers.cpp` | the courier, behaviour-tree engine |
| `initial_pose_seeder` | `initial_pose_seeder.cpp` | tells AMCL where the robot starts |

`location_table.cpp` and `location_markers.cpp` compile into **both** courier binaries. That is
the one piece of sharing between the two engines, and it is deliberate: they can disagree about
how to run a mission, but they cannot disagree about what a location *is*.

## Every file, one line each

**The contract** — `acadbot_courier_msgs/`

| file | ln | what it does | entry |
|---|---|---|---|
| `srv/RequestDelivery.srv` | 8 | book a delivery: two names in, an id or a refusal out | [A1](#a1-requestdeliverysrv--booking) |
| `action/ExecuteDelivery.action` | 35 | run a booked job: goal, result, and the six feedback fields | [A2](#a2-executedeliveryaction--running-the-job) |
| `CMakeLists.txt` · `package.xml` | 34 | `rosidl` generation; no dependency on the implementation | [A3](#a3-why-the-interfaces-are-their-own-package) |

**Shared vocabulary** — used by every target in the package

| file | ln | what it does | entry |
|---|---|---|---|
| `include/…/types.hpp` | 97 | `Pose2D`, `Leg`, `JobState`, `Job` — and the frozen poses | [B1](#b1-pose2d-leg-jobstate), [B2](#b2-job--and-why-the-poses-are-frozen) |
| `include/…/location_table.hpp` · `src/location_table.cpp` | 142 | discovers locations from parameter overrides; refuses a bad floor plan | [B3](#b3-locationtablefrom_parameters--discovery-from-parameter-overrides)–[B5](#b5-known_names--the-rejection-message) |
| `include/…/location_markers.hpp` · `src/location_markers.cpp` | 138 | draws the location table in RViz, current target highlighted | [D1](#d1-make_location_markers) |

**Engine 1 — the state machine**

| file | ln | what it does | entry |
|---|---|---|---|
| `include/…/courier_server.hpp` | 168 | the node: `Phase`, `CancelReason`, and every member the FSM tracks | [C](#c-the-courier-node) |
| `src/courier_server.cpp` | 607 | booking, the action server, the Nav2 client, retry, timeout, the three endings, and `main()` | [C1](#c1-construction)–[C12](#c12-to_goal_pose-and-one-naming-trap) |

**Engine 2 — the behaviour tree (bonus)**

| file | ln | what it does | entry |
|---|---|---|---|
| `behavior_trees/courier.xml` | 74 | the entire mission: two legs, retry, timeout, inter-attempt delay | [H1](#h1-courierxml--the-mission-as-data) |
| `include/…/bt_nodes.hpp` · `src/bt_nodes.cpp` | 345 | `GoToLocation`, the only custom leaf; `LegStatus`; `NavContext` | [H2](#h2-gotolocation--the-only-custom-leaf), [H3](#h3-legstatus--feedback-that-survives-the-gap) |
| `include/…/courier_bt_server.hpp` · `src/courier_bt_server.cpp` | 526 | the same plumbing as engine 1, with the leg loop replaced by a ticked tree, plus `main()` | [H4](#h4-courier_bt_server--the-plumbing-and-the-tick) |

**Making it run**

| file | ln | what it does | entry |
|---|---|---|---|
| `src/initial_pose_seeder.cpp` | 235 | seeds AMCL and *verifies* it worked, so requirement 9 needs no mouse | [E1](#e1-initial_pose_seeder), [E2](#e2-transform_advancing--the-check-that-took-four-attempts) |
| `config/courier.yaml` | 73 | locations and limits, one `/**:` block read by both engines | [F2](#f2-courieryaml), [H5](#h5-one-config-block-for-two-engines) |
| `launch/courier.launch.py` | 192 | sim → localization → seed → Nav2 → courier → RViz, in that order for reasons | [F1](#f1-courierlaunchpy), [H6](#h6-missionfsmbt) |
| `rviz/courier.rviz` | 105 | the map, the costmaps, and the location markers | [D2](#d2-courierrviz) |
| `CMakeLists.txt` · `package.xml` | 94 | three targets; `behaviortree_cpp` and `ament_index_cpp` for engine 2 | [H4](#h4-courier_bt_server--the-plumbing-and-the-tick) |
| `README.md` | 181 | how to run it, and what it does and does not do | — |

**Not built, not installed** — `tools/`

| file | ln | what it does | entry |
|---|---|---|---|
| `fake_nav2.py` | 136 | a `navigate_to_pose` that fails on command, so the courier is testable in seconds | [G1](#g1-fake_nav2py--a-stand-in-navigate_to_pose) |
| `README.md` | 81 | the three switches, and the two traps in the harness itself | [G1](#g1-fake_nav2py--a-stand-in-navigate_to_pose) |

## Two rules the layout follows

**Nothing depends on the launch package, and the launch package depends on everything.**
`acadbot_bringup` composes; it never implements. Putting the courier's interfaces there would
mean a client wanting to call `/request_delivery` had to depend on a package full of launch
files for four unrelated sessions.

**A header exists where a second translation unit needs the declaration** — not one per class
as a reflex. `location_table.hpp` exists because four other files use the table;
`courier_server.hpp` because the class is declared apart from the 600-line file that implements
it. Each executable's `main()` sits at the bottom of its own implementation file — the
convention the course's `patrol_commander.cpp` and `square_driver.cpp` already use.

---

# A. The interfaces

Two files in [`acadbot_courier_msgs`](../ros2_ws/src/acadbot_courier_msgs). They are the
contract everything else compiles against, and they are where the design is decided.

## A1. `RequestDelivery.srv` — booking

**What.** A service: name a pickup and a dropoff, get back an id or a refusal.

**Why a service.** Booking is a transaction — it either succeeds or it does not, it takes no
measurable time, and there is nothing to report while it happens. Requirement 1 says *"the
reply comes back immediately"*, which is the definition of a service and the opposite of an
action.

```
string pickup       # named location; must exist in the courier's location table
string dropoff      # named location; must exist, and must differ from pickup
---
bool   accepted
string job_id       # refer to this when sending the ExecuteDelivery goal;
                    # empty when accepted is false
string reason       # why it was rejected, in words meant for a human;
                    # empty when accepted is true
```

`reason` is a `string` rather than an error enum on purpose: requirement 2 asks for a
*human-readable* reason, and the messages this project produces name what you could have said
instead (§C2).

## A2. `ExecuteDelivery.action` — running the job

**What.** An action: run a booked job, streaming progress, cancellable.

**Why an action.** It takes minutes, it has something to say the whole time, and it must be
interruptible. Those three properties are exactly what an action provides and a service does
not.

```
string job_id
---
bool   success
string job_id
string message           # human-readable outcome, success or failure
string failed_leg        # "pickup" or "dropoff"; empty when success is true
uint8  attempts_used     # total navigate_to_pose goals sent across both legs
---
uint8 LEG_PICKUP  = 0
uint8 LEG_DROPOFF = 1

uint8   leg
string  target_location
float32 distance_remaining
uint8   attempt
uint16  nav2_recoveries
string  state            # NAVIGATING / RETRYING / CANCELING
```

Three decisions worth defending:

- **The goal carries only `job_id`.** The poses were resolved and frozen at booking (§B2).
  Re-sending them here could only introduce a disagreement between the job accepted and the
  job run.
- **`failed_leg` is a string, not the enum.** Requirement 7 says the result must *name* the
  leg that failed, and a string does that legibly in `ros2 action send_goal` output.
- **`nav2_recoveries` is surfaced.** It is Nav2's own count, and it is the most diagnostic
  number available — zero means a clean drive, fifteen means the robot is fighting something.
  It turns the blockage demonstration into a number instead of a vibe.

## A3. Why the interfaces are their own package

`rosidl_generate_interfaces` wants its own package. Mixing generation and consumers is
workable but fragile, and it would mean anyone depending on the contract drags in the node
that implements it. This is the standard split.

One build fact that catches people: **any package generating an action needs `action_msgs`**,
because the generated goal-handling services are built on its `GoalInfo`/`GoalStatus` types.

```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/RequestDelivery.srv"
  "action/ExecuteDelivery.action"
  DEPENDENCIES action_msgs
)
```

---

# B. Value types and the location table

**Why these are their own translation units at all**, rather than living inside
`courier_server.cpp`: three separate consumers include `location_table.hpp`, and
`location_table.cpp` compiles into **both** courier executables.

```
courier_server.hpp      ─┐
courier_bt_server.hpp   ─┼─►  location_table.hpp
location_markers.hpp    ─┘
```

Inlined into the state machine, the behaviour-tree engine would need its own copy — and then
the two engines could disagree about what a location *is*, which would quietly invalidate every
comparison the bonus exists to make. The markers would need a third copy.

Worth being honest about the order in which that became true: `location_markers.hpp` needed the
table **before** the behaviour tree existed, so there were already two consumers on day one. The
second engine made a reasonable decision look prescient rather than causing it.

The size argument cuts the other way and is worth stating: this is 80 lines of `.cpp` and 20 of
header, small enough to inline if there were one consumer. Two defences that would be
rationalisations are deliberately **not** offered — there are no unit tests in this project, so
no test seam depends on it, and nothing outside the package links against it, which is exactly
why there is no `courier_core` library either ([`REPORT.md` §2.2](REPORT.md)). The
multi-consumer fact is the whole argument.

What the file earns its keep with is not storage but **validation**: `from_parameters` is where
requirement 8 is enforced and where three malformed floor plans are refused at startup (B3, B4).
Parsing-and-validating configuration is a different job from running a delivery, and the file
boundary says so.

## B1. `Pose2D`, `Leg`, `JobState`

**What.** The vocabulary, in [`types.hpp`](../ros2_ws/src/acadbot_courier/include/acadbot_courier/types.hpp).

**Why separate from the node.** So the pieces that only *describe* a delivery — the location
table, the markers — need not include the one that runs it. `location_table.cpp` compiles
without `rclcpp_action`, without `nav2_msgs`, without the courier interfaces. That is a
checkable statement that config parsing knows nothing about actions.

```cpp
struct Pose2D { double x{0.0}; double y{0.0}; double yaw{0.0}; };

enum class Leg { PICKUP, DROPOFF };

/// The spelling that goes into action feedback and into the result's
/// failed_leg field, defined once so the two cannot drift apart.
inline const char * to_string(Leg leg)
{
  return leg == Leg::PICKUP ? "pickup" : "dropoff";
}
```

`Pose2D` holds a yaw rather than a quaternion because that is how a location is written in
YAML; the conversion happens once, on the way into a Nav2 goal (§C12).

## B2. `Job` — and why the poses are frozen

**What.** A booked delivery.

**Why it stores poses and not just names.** This is the design decision most likely to be
asked about.

```cpp
struct Job
{
  std::string id;
  std::string pickup_name;
  std::string dropoff_name;
  Pose2D pickup;              // resolved once, at booking, and frozen
  Pose2D dropoff;
  JobState state{JobState::BOOKED};

  const std::string & name_of(Leg leg) const
  { return leg == Leg::PICKUP ? pickup_name : dropoff_name; }

  const Pose2D & pose_of(Leg leg) const
  { return leg == Leg::PICKUP ? pickup : dropoff; }
};
```

Both the names and the poses are kept: the names are what the requester asked for and what
feedback reports; the poses are what Nav2 is sent. **Resolving once, at booking, is the
point** — looking the names up again when the action runs would let an edit to `courier.yaml`
move the target of a job that had already been accepted. The requester would be told one thing
and the robot would do another.

`name_of()` / `pose_of()` keep `leg == PICKUP ? … : …` out of the state machine, which
otherwise branches on leg in six places.

**Terminal states are remembered, not discarded.** A finished job stays in the registry as
`SUCCEEDED` / `FAILED` / `CANCELED` so a replayed goal is rejected rather than delivering
twice (§C4).

## B3. `LocationTable::from_parameters` — discovery from parameter overrides

**What.** The name → pose table, in
[`location_table.cpp`](../ros2_ws/src/acadbot_courier/src/location_table.cpp). The only class
that knows a location has coordinates at all.

**Why it reads overrides rather than a declared list.** ROS 2 parameters have no map type, so
a name → pose table has to be encoded somehow. The obvious encoding is a list of names plus a
group per name — but that redundancy is one-directional: a name with no pose throws, while a
pose whose name was left out of the list is *silently ignored*. Add a room, forget to list it,
and you get an unusable location and no complaint.

Reading the **parameter overrides** — every key the YAML actually supplied, declared or not —
means a location exists by being written down and nothing else.

```cpp
const auto & overrides =
  node.get_node_parameters_interface()->get_parameter_overrides();

for (const auto & kv : overrides) {
  const std::string & key = kv.first;
  if (key.rfind(prefix, 0) != 0) { continue; }        // prefix is "locations."
  const std::string name = key.substr(prefix.size());

  std::vector<double> xyyaw;
  try {
    xyyaw = node.declare_parameter<std::vector<double>>(key);
  } catch (const rclcpp::exceptions::InvalidParameterTypeException &) {
    throw std::runtime_error(
      "location '" + name + "' must be a list of three numbers, [x, y, yaw].");
  }
  ...
}
```

**Why not `automatically_declare_parameters_from_overrides`,** which looks like the obvious
tool: it declares *every* key at construction, so the node's own
`declare_parameter("max_retries", 2)` would then throw `ParameterAlreadyDeclaredException`.
Every parameter in the node would need a `has_parameter` guard to fix one feature. Reading the
overrides touches nothing else.

A pleasant side effect: **duplicate names stop being possible.** The override map is keyed, so
YAML collapses them before the code sees them — one fewer failure mode to write and to
explain.

## B4. Refusing to start on a bad floor plan

**What.** Two failure paths, both fatal at construction.

**Why fatal.** A configuration mistake should stop the node at startup, not surface as a
failed delivery twenty minutes into a demo.

```cpp
// Refused rather than skipped. A short list is a typo, and skipping it
// would surface an hour later as "unknown location", sending you to the
// service handler when the actual fault is a forgotten yaw.
if (xyyaw.size() != 3) {
  throw std::runtime_error(
    "location '" + name + "' needs exactly 3 numbers [x, y, yaw], got " +
    std::to_string(xyyaw.size()) + ".");
}
```

The alternative — `if (coords.size() == 3) { use it }` — silently drops the malformed entry.
The delivery then comes back *"unknown location: reception"* and you go looking at the service
handler, when the real fault is a missing third number.

These messages only work because [`main()`](../ros2_ws/src/acadbot_courier/src/courier_server.cpp)
catches them:

```cpp
try {
  rclcpp::spin(std::make_shared<acadbot_courier::CourierServer>());
} catch (const std::exception & e) {
  RCLCPP_FATAL(rclcpp::get_logger("courier_server"), "%s", e.what());
  rclcpp::shutdown();
  return 1;
}
```

Without the catch, a missing coordinate prints `terminate called after throwing` and an abort
trace, which tells the reader nothing.

## B5. `known_names()` — the rejection message

**What.** The configured names as `"charging_dock, lab_bench, reception, storage"`.

**Why.** It goes into the refusal, so a mistyped location tells the requester what they could
have said instead of merely that they were wrong:

```
unknown pickup 'kitchen'; known locations are: charging_dock, lab_bench, reception, storage
```

---

# C. The courier node

All in [`courier_server.cpp`](../ros2_ws/src/acadbot_courier/src/courier_server.cpp), declared
in [`courier_server.hpp`](../ros2_ws/src/acadbot_courier/include/acadbot_courier/courier_server.hpp).

## C1. Construction

Five parameters, the location table, the service, the action server, the Nav2 client, the
marker publisher and its timer.

```cpp
goal_frame_      = declare_parameter<std::string>("goal_frame", "map");
max_retries_     = declare_parameter<int>("max_retries", 2);
retry_delay_     = declare_parameter<double>("retry_delay", 3.0);
feedback_period_ = declare_parameter<double>("feedback_period", 0.5);
leg_timeout_     = declare_parameter<double>("leg_timeout", 120.0);

locations_ = LocationTable::from_parameters(*this);   // throws on a bad floor plan
```

There is deliberately **no `nav2_wait_timeout`**. The service checks
`action_server_is_ready()` at booking time and refuses with a reason instead of waiting — which
turns "Nav2 isn't up" into requirement 2's *"cannot be served right now"* rather than a hang.

## C2. `handle_request` — the five refusals

**What.** The service handler. Validates, mints an id, returns. No driving happens here, which
is the entire reason this is a service.

**Why five.** Requirement 2 asks for unknown names and "cannot be served right now"; the rest
fall out of what a single-robot courier can honestly promise.

```cpp
const auto reject = [&](const std::string & why) {
    response->accepted = false;
    response->job_id = "";
    response->reason = why;
    RCLCPP_WARN(get_logger(), "rejected '%s' -> '%s': %s",
      request->pickup.c_str(), request->dropoff.c_str(), why.c_str());
  };

if (!active_job_id_.empty()) {
  reject("a delivery is already running (" + active_job_id_ + ")");   return; }

const auto pickup = locations_.find(request->pickup);
if (!pickup) {
  reject("unknown pickup '" + request->pickup + "'; known locations are: " +
    locations_.known_names());                                        return; }
// ... same for dropoff ...

if (request->pickup == request->dropoff) {
  reject("pickup and dropoff are both '" + request->pickup +
    "'; there is nothing to deliver");                                return; }

if (!nav_client_->action_server_is_ready()) {
  reject("navigation is not available: Nav2's navigate_to_pose action "
    "server is not up");                                              return; }
```

Order matters: the Nav2 check is last, so a typo is reported as a typo even when the stack is
still coming up.

## C3. Job ids

```cpp
std::ostringstream id;
id << "job_" << std::setw(4) << std::setfill('0') << ++next_job_number_;
```

A counter, not a UUID — **someone has to retype it live, in the room.** `job_0007` is a
defensible choice; `f47ac10b-58cc-...` is not, at a whiteboard.

## C4. `handle_goal` — refusing a goal

Three refusals, and the middle one is why finished jobs are kept:

```cpp
if (!active_job_id_.empty())            return REJECT;  // one job owns the robot
const auto it = jobs_.find(goal->job_id);
if (it == jobs_.end())                  return REJECT;  // never booked
if (it->second.state != JobState::BOOKED) return REJECT; // already ran -> would deliver twice
if (!nav_client_->action_server_is_ready()) return REJECT;
```

## C5. `handle_cancel` — and the finish that cannot happen here

**What.** Accepts the cancel and propagates it to Nav2.

**Why it defers.** Two subtleties, both learned by hitting them.

```cpp
cancel_reason_ = CancelReason::CLIENT_REQUEST;

if (phase_ == Phase::RETRY_WAIT) {
  // Nothing is driving and no Nav2 goal is in flight, so no result is
  // coming to finish on. The goal does not enter the CANCELING state until
  // this callback returns, though, so canceled() cannot be called from
  // here -- finish on the next spin instead.
  if (retry_timer_) { retry_timer_->cancel(); }
  retry_timer_ = create_wall_timer(1ms, [this]() {
      retry_timer_->cancel();
      finish_canceled();
    });
  return rclcpp_action::CancelResponse::ACCEPT;
}

phase_ = Phase::CANCELING;
if (nav_handle_) { nav_client_->async_cancel_goal(nav_handle_); }
// If nav_handle_ is null, Nav2 has not acknowledged the goal yet.
// on_nav_goal_response cancels it the moment the handle arrives.
```

- **Cancelling during the retry delay** has no Nav2 goal to cancel, so waiting for a Nav2
  result would hang forever. Measured on the real stack: **4.5 ms** to `CANCELED`.
- **Cancelling before Nav2 acknowledges** leaves no handle to cancel. The flag is set, and
  `on_nav_goal_response` cancels the handle the moment it arrives.

## C6. `CancelReason` — disambiguating a CANCELED result

**What.** A three-valued enum recorded when a cancel is *issued*.

**Why.** A `CANCELED` result from Nav2 is ambiguous on its own. It can mean the client
cancelled the delivery, that our own leg timeout gave up on this attempt, or that Nav2
abandoned the goal unilaterally. Those want three different outcomes, and a `bool` cannot tell
them apart.

```cpp
enum class CancelReason { NONE, CLIENT_REQUEST, LEG_TIMEOUT };
```

```cpp
case rclcpp_action::ResultCode::CANCELED:
  switch (cancel_reason_) {
    case CancelReason::CLIENT_REQUEST: finish_canceled();               return;
    case CancelReason::LEG_TIMEOUT:    cancel_reason_ = CancelReason::NONE;
                                       leg_failed("attempt ran past the ... leg timeout");
                                                                        return;
    case CancelReason::NONE:           leg_failed("Nav2 cancelled the goal on its own");
                                                                        return;
  }
```

`NONE` at the moment a `CANCELED` lands means Nav2 did it by itself — **a failure, not a
success, and definitely not a silent stop.**

## C7. `send_leg_goal` — the async pattern

Everything hangs off callbacks; nothing blocks. That is what makes a single-threaded executor
sufficient and every mutex unnecessary (see [`BACKGROUND.md` §8](BACKGROUND.md)).

```cpp
rclcpp_action::Client<NavigateToPose>::SendGoalOptions opts;
opts.goal_response_callback =
  [this](NavGoalHandle::SharedPtr handle) {on_nav_goal_response(handle);};
opts.feedback_callback =
  [this](NavGoalHandle::SharedPtr h, auto feedback) {on_nav_feedback(h, feedback);};
opts.result_callback =
  [this](const NavGoalHandle::WrappedResult & result) {on_nav_result(result);};

nav_client_->async_send_goal(goal, opts);

leg_timeout_timer_ = create_wall_timer(
  std::chrono::duration<double>(leg_timeout_),
  std::bind(&CourierServer::on_leg_timeout, this));

publish_markers();     // move the highlight to the new target
```

## C8. A rejected goal is a failed leg

**What.** The one place this node deliberately differs from the course's `patrol_commander`.

```cpp
if (!handle) {
  // patrol_commander skips a rejected waypoint. A courier cannot: a leg
  // that was never driven is a leg that failed.
  leg_failed("Nav2 rejected the goal");
  return;
}
```

`patrol_commander.cpp:101` logs a warning and advances to the next waypoint. For a patrol that
is fine — the point is to keep patrolling. For a courier it is the beginning of reporting a
delivery that never happened.

## C9. `leg_failed` — retry, then stop lying

```cpp
if (attempt_ <= max_retries_) {
  phase_ = Phase::RETRY_WAIT;
  retry_timer_ = create_wall_timer(
    std::chrono::duration<double>(retry_delay_),
    std::bind(&CourierServer::on_retry_elapsed, this));
  return;
}

finish_failed(std::string("the ") + to_string(leg_) + " leg failed after " +
  std::to_string(attempt_) + " attempts: " + why);
```

Retries are counted **per leg** — a job that struggles at pickup and again at dropoff gets its
allowance at each — while the result reports `attempts_used` across both.

## C10. The three endings

Exactly one runs per accepted goal, and **success is reachable only from the dropoff leg
returning `SUCCEEDED`.**

```cpp
void CourierServer::finish_failed(const std::string & message)
{
  ...
  result->success = false;
  result->failed_leg = to_string(leg_);
  job.state = JobState::FAILED;
  // abort(), never succeed(). A delivery reported as done is a delivery that
  // happened.
  execute_handle_->abort(result);
  reset_to_idle();
}
```

`finish_canceled()` leaves `failed_leg` **empty** — a cancelled delivery is not a failed one.
Nothing went wrong; the requester changed their mind.

There is also a branch for arriving *as* the cancel lands:

```cpp
case rclcpp_action::ResultCode::SUCCEEDED:
  if (cancel_reason_ == CancelReason::CLIENT_REQUEST) {
    // Arrived just as the cancel landed. The requester asked us to stop,
    // so this is a cancelled delivery, not a successful one.
    finish_canceled();
    return;
  }
```

Observed live: a cancel issued with `distance_remaining: 0.19` — inside the 0.20 m goal
tolerance — still returned `CANCELED`.

## C11. Feedback on the courier's own clock

**What.** A timer at `feedback_period` (0.5 s) publishing cached values.

**Why not just forward Nav2's feedback.** Because then a stalled Nav2 becomes *silence*, and
silence is indistinguishable from a crashed node. Publishing on our own clock means a stalled
leg reads as a **frozen distance** — visible, diagnosable, and honest.

```cpp
feedback->leg = (leg_ == Leg::PICKUP) ? ExecuteDelivery::Feedback::LEG_PICKUP
                                      : ExecuteDelivery::Feedback::LEG_DROPOFF;
feedback->target_location    = job.name_of(leg_);
feedback->distance_remaining = last_distance_;      // cached from Nav2
feedback->attempt            = static_cast<uint8_t>(attempt_);
feedback->nav2_recoveries    = last_recoveries_;
feedback->state              = phase_name(phase_);
```

0.5 s rather than 1.0 s deliberately: the contract is "at least once a second", and publishing
at exactly the promised rate leaves no margin for a late tick.

## C12. `to_goal_pose`, and one naming trap

```cpp
out.header.frame_id = goal_frame_;
out.pose.position.x = pose.x;
out.pose.position.y = pose.y;
tf2::Quaternion q;
q.setRPY(0.0, 0.0, pose.yaw);
out.pose.orientation = tf2::toMsg(q);
```

The private helper for the phase string is called **`phase_name`, not `to_string`** — a member
of that name would hide the free `to_string(Leg)` and `to_string(JobState)` from `types.hpp`,
and every call to those inside the class would need qualifying.

---

# D. Seeing it

## D1. `make_location_markers`

**What.** A free function turning the table plus the active target into a `MarkerArray`
([`location_markers.cpp`](../ros2_ws/src/acadbot_courier/src/location_markers.cpp)).

**Why a free function.** It is a pure mapping with no state to own between calls. Taking the
active location as an argument keeps this file ignorant of legs, jobs and Nav2 entirely.

**Why arrows rather than spheres.** `yaw_goal_tolerance` is 0.25 rad, so a location's heading
is real work the robot must do. A sphere would hide the one thing about a location that is
easy to get wrong.

```cpp
arrow.ns = "courier_locations";
arrow.id = id;
arrow.type = visualization_msgs::msg::Marker::ARROW;
tf2::Quaternion q;  q.setRPY(0.0, 0.0, pose.yaw);
arrow.pose.orientation = tf2::toMsg(q);

// Zero lifetime means "until replaced". Same ns + id overwrites, so
// republishing on every change never accumulates stale arrows.
arrow.lifetime = rclcpp::Duration(0, 0);
```

Ids come from map iteration order, which is stable for a table fixed at startup — so each
location keeps its id across republishes and markers are *replaced* rather than stacked. That
is what makes republishing on every leg change free.

**This is diagnostic, not decorative.** Drawn over the global costmap, a location sitting
inside the inflation band is visible before it costs anyone a failed delivery — which is how
two of the four locations were found to be badly placed.

## D2. `courier.rviz`

`nav2.rviz` plus a `MarkerArray` display on `/courier_server/locations`, at **Transient Local**
durability to match the publisher — RViz's MarkerArray display defaults to volatile, and a
volatile subscriber gets no retained sample, so an RViz started after the node would show
nothing.

Two other differences: the local costmap starts **disabled** (a rolling 4 × 4 m window that
paints over the markers during a delivery), and **Publish Point** is enabled, which is how new
location coordinates get read off the map.

---

# E. Localization seeding

## E1. `initial_pose_seeder`

**What.** A second, small node
([`initial_pose_seeder.cpp`](../ros2_ws/src/acadbot_courier/src/initial_pose_seeder.cpp)) that
tells AMCL where the robot starts.

**Why it exists.** AMCL publishes nothing until told, and without `map → odom` the Nav2
costmaps cannot configure — the lifecycle manager times out, aborts the whole bringup, and
every goal afterwards is rejected. Requirement 9 says one command brings it up, so the
**2D Pose Estimate** click has to go.

**Why a separate executable rather than folding it into the courier.** A delivery service that
also sets up localization is harder to defend than a sixty-line node whose name says what it
does. It is also optional: drop it from the launch and click in RViz instead.

**Why it does not simply publish once.** Publishing at a fixed moment is unreliable — AMCL
looks up `base_footprint → odom` around the pose's timestamp, and early in a run its TF buffer
holds a fraction of a second. Five different fixed delays each failed differently. So the
seeder does not guess: it publishes, checks whether it worked, and repeats.

Three gates before it publishes at all:

```cpp
if (!odometry_alive())        return;   // nothing to place the pose against yet
if (scans_seen_ < min_scans_) return;   // AMCL cannot hold map->odom without a laser
publish_pose();
```

The scan gate is not obvious and cost a full debugging round: **AMCL only publishes `map→odom`
off the back of a filter update, and a filter update needs a scan.** Seed at sim time 0.4 and
it accepts the pose, emits a transform briefly, and then goes quiet.

And it leaves an already-localized robot alone:

```cpp
if (attempts_ == 0) {
  finish("map->odom is already advancing: something has set a pose "
         "already, so nothing to do.");
```

Without that guard, launching after you had set a good pose by hand would replace it with a
guess at the origin.

## E2. `transform_advancing()` — the check that took four attempts

**What.** The test for "is AMCL actually localized?"

**Why it is written this way.** Three earlier versions were wrong in the same way — each
*passed while the system was broken*.

| version | why it lied |
|---|---|
| does `map→odom` exist? | tf2 caches for ten seconds and `TimePointZero` means *latest available*, so a transform emitted once and abandoned kept answering yes |
| exists, three times in a row? | three checks a second apart all read the same dead transform |
| how old is it? | the age is measured in **simulated** time, which barely advances while Gazebo is starting — exactly the window the check exists to police |

The working version asks none of those. It asks whether the stamp is **changing**:

```cpp
bool transform_advancing()
{
  try {
    const auto tf =
      tf_buffer_->lookupTransform(map_frame_, odom_frame_, tf2::TimePointZero);
    const rclcpp::Time stamp(tf.header.stamp);
    const bool advancing = have_stamp_ && stamp > last_stamp_;
    last_stamp_ = stamp;
    have_stamp_ = true;
    return advancing;
  } catch (const tf2::TransformException &) {
    have_stamp_ = false;
    return false;
  }
}
```

A stamp that keeps moving means AMCL is still publishing; one frozen at last second's value
means it has stopped. **That holds whatever the clock is doing** — which is the property the
other three lacked.

It also gives up honestly, rather than spinning silently:

```cpp
RCLCPP_ERROR(get_logger(),
  "gave up after %.0fs: AMCL never published map->odom. Set the pose by "
  "hand with RViz's 2D Pose Estimate.", give_up_after_);
```

## E3. Localization parameters — and the file that turned out to be unnecessary

The courier localizes with `nav2_bringup`'s `localization_launch.py` (`map_server` + `amcl` +
their lifecycle manager) and hands it the course's own
[`nav2_params.yaml`](../ros2_ws/src/acadbot_navigation/config/nav2_params.yaml). No AMCL
parameter file of our own — and getting to that point took removing one.

**The file that used to be here.** `config/amcl.yaml` existed for most of this project: 75
lines holding the course's AMCL block verbatim plus `transform_tolerance: 1.0`, added during the
seeding investigation because that value is also the window AMCL allows when looking up
`base_footprint → odom` for an incoming initial pose.

It was deleted after a key-by-key diff showed the two `amcl:` blocks were identical except for
that one parameter — and then a direct query showed the parameter was already AMCL's default:

```console
$ ros2 run nav2_amcl amcl               # no params file at all
$ ros2 param get /amcl_probe transform_tolerance
Double value is: 1.0
```

Seventy-five lines that changed nothing. Worse than nothing: a frozen copy of someone else's
AMCL block, which would silently keep stale values if the course ever retuned its particle
counts or motion noise — exactly the drift argument used against duplicated location blocks in
[H5](#h5-one-config-block-for-two-engines), applied in one place and violated in the other.

Verified before removal, not after: with `nav2_params.yaml` supplying AMCL, the seeder still
localizes, `/amcl` and `/bt_navigator` both reach `active [3]`, `map → odom` appears, and
`charging_dock → storage` delivers in 56 s with `attempts_used: 2`.

**Two absences worth keeping in mind**, because both look like obvious things to add and both
cost a debugging session. Neither file has them, which is why neither is bitten:

**No `map_server` block.** `localization_launch.py` passes the map path to that node itself as
`parameters=[params_file, {'yaml_filename': map_yaml_file}]` — and a `yaml_filename` written in
the params file **wins over that override rather than losing to it**, leaving `map_server`
active but empty and AMCL stuck on `Waiting for map....` forever.

**No `set_initial_pose`.** It looks like exactly the right tool. AMCL applies it the instant it
activates — about 0.4 s after its process starts, before it has received a single `/clock`
message — so under `use_sim_time` the pose is stamped **t = 0**, a time for which no
`odom → base_footprint` has ever existed:

```
[amcl] Setting pose (0.000000): 0.000 0.000 0.000
```

Delaying the launch does not help: the race is between AMCL's own activation and its own first
clock message, so it moves along with it. [E1](#e1-initial_pose_seeder) is the answer that works.

---

# F. Bringup and configuration

## F1. `courier.launch.py`

**What.** The one command
([`courier.launch.py`](../ros2_ws/src/acadbot_courier/launch/courier.launch.py)): simulation,
`map_server` + AMCL, the seeder, the Nav2 servers, RViz, the courier.

**Why it composes the stack rather than including `autonomy.launch.py`.** That file forwards no
`params_file` — but the real obstacle is one level below it. `navigation.launch.py` accepts a
single `params_file` and hands **the same one** to both `localization_launch.py` and
`nav2_stack.launch.py`. AMCL here needs this package's file, which holds an `amcl:` block and
nothing else; the Nav2 servers need the course's untouched `nav2_params.yaml`, with its twelve
server blocks. One file cannot be both, so no pass-through argument would have helped — what is
missing is a *second* argument. See [`REPORT.md` §2.2](REPORT.md) for the cheaper alternative:
the two `amcl:` blocks differ by exactly one parameter.

**Why the ordering is what it is.** Localization starts **with** the simulation — AMCL keeps
its own TF buffer, and that buffer only starts filling when the node does, so a late start
leaves it holding a single sample. Nav2 is held back behind `nav2_delay`, because its costmaps
cannot configure without `map→odom`, and a timed-out lifecycle transition aborts the entire
bringup.

The courier itself starts immediately, not with Nav2: until `navigate_to_pose` exists the
service refuses bookings with a reason saying so, which is the behaviour requirement 2 asks
for anyway.

## F2. `courier.yaml`

Everything tunable, in one file
([`courier.yaml`](../ros2_ws/src/acadbot_courier/config/courier.yaml)).

```yaml
courier_server:
  ros__parameters:
    use_sim_time: true
    goal_frame: "map"
    max_retries: 2          # per leg, so up to 3 attempts each
    retry_delay: 3.0
    feedback_period: 0.5
    leg_timeout: 120.0

    locations:
      reception:     [0.60, 4.20,  0.00]
      lab_bench:     [4.50, 4.20,  3.14]
      storage:       [5.50, 0.60,  0.00]
      charging_dock: [0.50, 2.00, -1.57]
```

The file carries a long comment block explaining that coordinates are in the **map** frame
(`map = gazebo + (3, 2)`), that locations must be at least `inflation_radius` (0.45 m) clear
of walls, and which two coordinates were moved after measurement and why. That is the one
place in this package where heavy comments earn their keep: someone editing coordinates needs
the frame explained, and needs to know why `lab_bench` is not in the obvious corner.

## F3. `nav2_params.yaml` — the stack we plan against, and do not touch

**What it is.** `acadbot_navigation/config/nav2_params.yaml`, ~330 lines configuring twelve
Nav2 servers. **Not a component of this project** — it is course material, and it is documented
here because every measurement in [`REPORT.md`](REPORT.md) is a measurement *of this
configuration* and is meaningless without it.

**Provenance, since it decides who is answerable for the results.** `git blame` puts every line
below in the course's initial commit, `3b8d932`, authored by the instructor on 2026-06-30. The
file has been modified once since — `165c76c`, which *appended* a `docking_server:` block
because Nav2's stock bringup starts that server, it fails to configure without a `dock_plugins`
entry, and the lifecycle manager then aborts the whole bringup and leaves every server
INACTIVE. That commit predates this project, sits on `main`, and touched no planner or
controller line. On the final-project branch,
`git log main..HEAD -- ros2_ws/src/acadbot_navigation/` is **empty**.

### The global planner — Dijkstra

```yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5
      use_astar: false          # ← Dijkstra, not A*
      allow_unknown: true
```

`NavfnPlanner` propagates a potential field outward from the goal across the global costmap,
then extracts the route by gradient descent from the robot's cell. `use_astar: false` makes
that propagation uniform-cost, which is **Dijkstra**; `true` would add a heuristic and expand
fewer cells. At 160 × 120 cells the saving is milliseconds against a 20 Hz budget.

| parameter | value | what it decides |
|---|---|---|
| `use_astar` | `false` | Dijkstra rather than A\* |
| `tolerance` | 0.5 m | accept a plan ending this far from an unreachable goal |
| `allow_unknown` | `true` | never-observed cells are traversable |

**"Path cost" is not distance.** It is accumulated per-cell cost from the costmap layers, so
what the planner minimises is set by the inflation parameters below, not by geometry.

### The costmaps — what makes cost mean something

Identical in both `local_costmap` and `global_costmap`:

```yaml
robot_radius: 0.20
resolution: 0.05
inflation_layer:
  plugin: "nav2_costmap_2d::InflationLayer"
  cost_scaling_factor: 3.0
  inflation_radius: 0.45
```

The inflation layer paints an exponentially decaying cost halo out to 0.45 m around every
obstacle, so the cheapest route is a compromise between short and clear-of-walls. Two numbers
here are load-bearing elsewhere in this document: `inflation_radius: 0.45` is the clearance a
location in `courier.yaml` must have (F2, B4), and `robot_radius: 0.20` is why a robot stopped
0.175 m from a wall face has its own footprint inside that wall ([`REPORT.md` §5.3](REPORT.md)).

### The controller — DWB, which minimises nothing globally

```yaml
FollowPath:
  plugin: "dwb_core::DWBLocalPlanner"
  max_vel_x: 0.26 ; max_vel_theta: 1.0
  vx_samples: 20  ; vtheta_samples: 20
  sim_time: 1.7
```

400 sampled `(v, ω)` pairs per cycle, each rolled forward 1.7 s, each scored by a weighted sum
of critics, best one published — 20 times a second. It is sampling and scoring, not search.

| critic | scale |
|---|---|
| `PathDist`, `PathAlign`, `RotateToGoal` | 32.0 |
| `GoalDist`, `GoalAlign` | 24.0 |
| `Oscillation` | (enabled, unweighted here) |
| **`BaseObstacle`** | **0.02** |

**Path adherence is weighted 1600× obstacle proximity.** This single ratio is the direct cause
of the one route this project cannot complete, and it is inherited, not chosen.

### Why none of it was changed

The brief: *"you do not write a planner or a controller."* Beyond that, `nav2_params.yaml` is
shared with Sessions 3 and 4, which are presumably tuned around its current behaviour —
so raising `BaseObstacle.scale` to fix one courier route would silently change the demo every
other session depends on. The route is kept as the honest-failure demonstration instead.

---

# G. The test harness

Why requirements 3, 4, 6 and 7 could be exercised in seconds rather than minutes, and why
cancel could be tested against an *abort on demand* instead of hoping the real robot
misbehaved at the right moment.

Only G1 is kept in the repository, at [`tools/fake_nav2.py`](../tools/fake_nav2.py). G2 and G3
were scratchpad-only and are recorded here rather than preserved — a decision made after G1 was
lost once and had to be rewritten.

## G1. `fake_nav2.py` — a stand-in `navigate_to_pose`

A `rclpy` action server that accepts `NavigateToPose` and does exactly what it is told. Three
environment variables, each isolating one thing the real stack hides:

| variable | behaviour | what it is for |
|---|---|---|
| `FAIL_GOALS=1,3` | abort those goals, succeed on the rest | one forced retry per leg |
| `DRIVE_TIME=1.5` | seconds of pretend driving per goal | keeps a full delivery under 5 s |
| `ACCEPT_DELAY=3` | stall before acknowledging a goal | opens the pre-acknowledge cancel window |

It publishes decreasing `distance_remaining` so the courier's own feedback path is exercised
too. No Gazebo, no map, no costmap — a two-leg delivery in about three seconds.

`FAIL_GOALS=1,3` deserves a note, because it is the shape that catches a specific class of bug.
It forces each leg to fail once and succeed on the second try, which is the only way to see
whether the per-leg attempt counter *restarts* at the pickup→dropoff transition. It should read

```
pickup 1, pickup 2, dropoff 1, dropoff 2      attempts_used: 4
```

A counter that never resets reads `dropoff 3, dropoff 4` instead — the delivery still succeeds,
so nothing looks wrong until someone reads the feedback. That is exactly the bug this found in
H2.

`ACCEPT_DELAY` exists for the same reason in the opposite direction: on the real stack Nav2
acknowledges a goal in a few milliseconds, so the window in which a cancel has *nothing to
cancel yet* is far too narrow to hit deliberately. Holding it open for three seconds turns an
un-testable race into a two-line test.

**Two traps in the harness itself**, both of which read as bugs in the code under test:

- Its `execute` callback blocks while pretending to drive, so the action server must be
  reentrant and spun by a `MultiThreadedExecutor`. A single-threaded executor sitting inside
  `execute` cannot service the cancel request meant to interrupt it: the goal runs to
  completion and the cancel is handled afterwards, which looks precisely like a courier that
  ignores cancellation.
- Calling `spin_once()` from inside that callback raises `Executor is already spinning`, which
  the action server then reports as an **aborted goal** — so a harness bug arrives disguised as
  Nav2 refusing to drive.

## G2. `cancel_client.py`

`ros2 action send_goal` cannot be scripted into cancelling, so this sends the goal, streams
feedback, cancels after N seconds and prints the terminal status. It is what produced the
literal evidence for requirement 6:

```
terminal status  : CANCELED
result.success   : False
result.failed_leg: ''
```

One trap worth recording: `self.handle` collides with a read-only property on rclpy's `Node`
and raises at construction. It is `self.goal_handle`.

## G3. Shell drivers

`test_courier.sh`, `test_cancel.sh`, `test_real.sh`, `test_cancel_real.sh` — bring up a stack,
book, run, cancel, and grep the outcome. Three separate bugs in *these* hid or faked results
and each cost a full run: a `grep -o "job_[0-9]*"` that matched zero digits and so also matched
the field name `job_id`; a `timeout 4` on `tf2_echo` that killed it a moment before it
resolved; and a `head -24` that truncated exactly the line before the terminal status.

**A check that truncates is a check that can lie** — and it lies in the direction of "no
result", which reads as failure. That lesson generalises well past this project.

---

# H. The behaviour tree (bonus)

The same mission, run by a `BehaviorTree.CPP` tree instead of the hand-written state machine in
group C. It is a **second executable**, not a replacement: `courier_server.cpp` is byte-for-byte
untouched, and `mission:=fsm|bt` picks which one the launch file starts.

That decision is worth stating plainly, because the alternative was tempting. Extracting the
shared plumbing — the service handler, the job registry, the feedback timer, the markers — into
a common base class would have avoided duplicating about 200 lines. It was rejected: the state
machine version was finished, verified on the real stack, and is the demo fallback. A refactor
to accommodate a bonus feature risks the thing that already works. **The duplication is the
cheaper mistake.**

What is *not* duplicated: `types.hpp`, `location_table.cpp` and `location_markers.cpp` compile
into both executables unchanged, so the two engines cannot disagree about what a location is.

## H1. `courier.xml` — the mission as data

**What it is.** [`behavior_trees/courier.xml`](../ros2_ws/src/acadbot_courier/behavior_trees/courier.xml),
30 lines, the entire delivery.

**Why.** The interesting part is what is *absent* compared with `courier_server.hpp`:

| hand-written in group C | replaced by |
|---|---|
| `attempt_`, `max_retries_`, `retry_timer_`, `on_retry_elapsed()` | `RetryUntilSuccessful` |
| `leg_timeout_timer_`, `on_leg_timeout()`, `CancelReason::LEG_TIMEOUT` | `Timeout` |
| `leg_` and the pickup→dropoff transition | `Sequence` |
| "if the pickup failed, do not attempt the dropoff" | `Sequence`, for free |

```xml
<Sequence name="delivery">
  <RetryUntilSuccessful num_attempts="{max_attempts}" name="pickup_attempts">
    <Fallback>
      <Timeout msec="{leg_timeout_msec}">
        <GoToLocation pose="{pickup_pose}" location_name="{pickup_name}" leg="pickup"/>
      </Timeout>
      <Delay delay_msec="{retry_delay_msec}">
        <AlwaysFailure/>
      </Delay>
    </Fallback>
  </RetryUntilSuccessful>
  <!-- leg 2: identical but for the target -->
</Sequence>
```

**The one non-obvious construction** is that `Delay → AlwaysFailure` tail. The obvious way to
express "wait 3 s between attempts" is to wrap the attempt in a `Delay` — and it is wrong: it
would postpone the *first* attempt too, making every delivery three seconds slower than the
state machine and quietly destroying the comparison. Putting the delay in the `Fallback`'s
second branch means it runs only after something failed, which is what `leg_failed()` does.

It has one residual difference, measured rather than assumed: on the **final** attempt the delay
still runs before the leg is declared failed, where the state machine checks the counter first
and reports immediately. Every exhausted leg therefore costs one extra `retry_delay`. Removing
it would mean a scripted precondition on the node, which buys three seconds at the cost of the
readability that justifies the tree.

**Two legs are written out rather than factored into a `SubTree`.** Eight more lines, and the
whole mission fits on one screen — which is the property that makes a tree worth having at all.

## H2. `GoToLocation` — the only custom leaf

**What it is.** [`bt_nodes.hpp`](../ros2_ws/src/acadbot_courier/include/acadbot_courier/bt_nodes.hpp)
/ [`bt_nodes.cpp`](../ros2_ws/src/acadbot_courier/src/bt_nodes.cpp). Sends one Nav2 goal;
`SUCCESS` if the robot arrived, `FAILURE` otherwise. It knows nothing about retries, timeouts or
which leg comes next — those are the decorators wrapped around it.

**Why a `StatefulActionNode`.** This is the same constraint as C7 in a new costume. A
`SyncActionNode::tick()` that waited for Nav2 would block the executor thread — and that thread
is the one that has to deliver the result the tick is waiting for. So:

```cpp
BT::NodeStatus GoToLocation::onStart()   // send, return RUNNING
BT::NodeStatus GoToLocation::onRunning() // read a flag the callbacks set
void           GoToLocation::onHalted()  // cancel the Nav2 goal
```

`pose` is a port carrying a `Pose2D`, **not** a location name, for the reason in B2: a job's
targets are frozen at booking. Looking the name up here would let an edit to `courier.yaml` move
the destination of an already-accepted delivery.

**Three bugs a review found in the first version of this file**, all three of them things
group C already got right. They are recorded because each is a category, not a typo.

**1 — the per-leg attempt counter never reset.**

```cpp
context_.status->leg = (leg_name == "pickup") ? Leg::PICKUP : Leg::DROPOFF;
...
context_.status->attempt++;      // nothing ever set this back to 0
```

`courier_server.cpp` resets `attempt_ = 0` at the leg transition; this did not, so feedback
reported *attempt 4, 5, 6* on the dropoff. Note the second-order mistake: `leg` is overwritten
*before* the increment, so the code could not have detected the change even if it had tried. The
fix compares first:

```cpp
const Leg leg = (leg_name == "pickup") ? Leg::PICKUP : Leg::DROPOFF;
if (leg != context_.status->leg) {
  context_.status->attempt = 0;   // retries are counted per leg
}
context_.status->leg = leg;
```

**2 — a cancel arriving before Nav2 acknowledged the goal was silently dropped.** `onHalted`
only cancelled `if (nav_handle_)`. Halt inside the window between `async_send_goal` and the
goal-response callback and there is no handle yet — so nothing was cancelled, and the response,
when it arrived, was stored into an already-halted node. **The tree stops and the robot keeps
driving**, while the client is told CANCELED. Group C handles this explicitly; the fix mirrors it:

```cpp
if (halted_) {
  // The halt landed in the gap between sending the goal and Nav2
  // acknowledging it. There was nothing to cancel then; there is now.
  context_.nav_client->async_cancel_goal(handle);
  return;
}
```

**3 — the Nav2 callbacks captured `this`, and `this` is a tree leaf.** A leaf dies with its tree,
which happens when the next job calls `createTreeFromFile`. A result still in flight from a
cancelled goal would then write into freed memory. The state machine is immune because its
callbacks target the *node*, which outlives everything. The fix is a liveness token:

```cpp
std::weak_ptr<bool> alive = alive_;          // expires with this leaf
opts.result_callback = [this, alive](const NavGoalHandle::WrappedResult & r) {
    if (alive.expired()) { return; }
    on_result(r);
  };
```

The pattern worth taking away: **moving logic into a tree changes object lifetimes.** State that
used to live as long as the process now lives as long as a tree, and every callback bound to it
inherits that shorter life.

## H3. `LegStatus` — feedback that survives the gap

**What it is.** A small struct holding the leg, the target name, the distance, the attempt, the
recovery count and the state string. It lives on the *node*, and the leaf writes into it through
the `NavContext`.

**Why not in the leaf.** Because of a gap that only appears once the tree exists: between
attempts the tree is sitting inside `Delay` with **no leaf running at all** — and the action
still owes its client feedback at ≥1 Hz through exactly that window. Keeping the numbers beside
the tree means the feedback timer can publish whatever the tree happens to be doing. It is the
tree's version of C11's rule: publish on the courier's own clock, never on Nav2's.

## H4. `courier_bt_server` — the plumbing, and the tick

**What it is.** [`courier_bt_server.cpp`](../ros2_ws/src/acadbot_courier/src/courier_bt_server.cpp).
Booking, the action server, the Nav2 client, the markers and the feedback timer — group C's, with
the leg loop replaced by a tree.

**The tick.** `handle_accepted` loads the blackboard from the frozen job and builds the tree; a
10 Hz timer ticks it:

```cpp
BT::NodeStatus status = tree_.tickOnce();
if (status == BT::NodeStatus::RUNNING) { return; }
tree_running_ = false;
if (status == BT::NodeStatus::SUCCESS) { finish_succeeded(); }
else { finish_failed("the " + std::string(to_string(leg_status_->leg)) + " leg failed"); }
```

`tickOnce`, never `tickWhileRunning` — the latter loops until the tree finishes, which is the
blocking call this entire design exists to avoid.

**The blackboard, and a trap that is not guessable.** Port types are checked when the tree is
*built*, not when it compiles:

```cpp
blackboard->set("max_attempts", max_retries_ + 1);                                 // int
blackboard->set("leg_timeout_msec", static_cast<unsigned int>(leg_timeout_ * 1000.0));
blackboard->set("retry_delay_msec", static_cast<unsigned int>(retry_delay_ * 1000.0));
```

`Timeout`'s `msec` and `Delay`'s `delay_msec` are `InputPort<unsigned>`;
`RetryUntilSuccessful`'s `num_attempts` is `InputPort<int>`. Writing the first two as plain
`int` compiles perfectly and then fails at runtime:

```
Tree creation error: the port [leg_timeout_msec] was initially created with
type [int] and, later type [unsigned int] was used somewhere else.
```

A related one: **`name` cannot be used as a port.** BT.CPP owns that attribute for the node's own
name, so the port is `location_name`.

**Cancel.** `handle_cancel` halts the tree — which reaches `onHalted` and cancels the Nav2 goal —
then finishes on a 1 ms timer, for C5's reason: the goal does not enter CANCELING until the
callback returns. It differs from group C in one measurable way: the state machine waited for
Nav2's CANCELED *result* before reporting, whereas a halted tree has no result callback left to
finish on, so the BT reports CANCELED while the stop is still in flight. The Nav2 goal is
verifiably cancelled either way — see [`REPORT.md` §11.2](REPORT.md) — but the two cancel
latencies are not comparable numbers.

## H5. One config block for two engines

**What it is.** [`courier.yaml`](../ros2_ws/src/acadbot_courier/config/courier.yaml) is keyed
`/**:` rather than by node name, so `courier_server` and `courier_bt_server` read the same block.

**Why.** The first version had a block per node — and therefore two copies of the four
locations. Two copies can drift, and a location edited in one and not the other would silently
send the two engines to different coordinates, poisoning the one comparison the bonus exists to
make. One block makes that impossible by construction.

**Why not YAML anchors**, which are the obvious alternative: ROS 2's `rcl_yaml_param_parser`
does not handle aliases reliably, and the resulting failure reads as a configuration bug rather
than a parsing one. Verified after the change that both nodes still load all four locations, and
that the state machine still reports `2 retries per leg, 120s leg timeout`.

## H6. `mission:=fsm|bt`

**What it is.** One launch argument; the two nodes sit under `IfCondition` on it.

**Why exactly one runs.** Both claim `/request_delivery` and `/execute_delivery`. That is
deliberate — the demo commands are then *identical* for either engine, which is the claim being
demonstrated: same interface, different internals. Namespacing the BT node would have made the
comparison harder to show and easier to fudge.

One consequence worth knowing: the markers publisher is node-relative, so under the BT node it
would become `/courier_bt_server/locations` and the RViz config would silently show nothing. A
one-line remapping in the launch file puts it back on the FSM's topic, so `courier.rviz` works
unchanged for both.

---

## Which requirement each component serves

| Requirement | Components |
|---|---|
| 1 · service, immediate, identifier | A1, C2, C3 |
| 2 · reject with a reason | A1, B3, B5, C2, C4 |
| 3 · action for execution | A2, C4–C10 |
| 4 · feedback a human can follow | A2, C11 |
| 5 · drive with Nav2 | C7, C8, C12 |
| 6 · cancel means stop | A2, C5, C6, C10 |
| 7 · honest failure | B2, C8, C9, C10 |
| 8 · everything in YAML | B3, B4, F2, C1, H5 |
| 9 · one command | E1, E2, E3, F1, D2 |
| bonus · mission logic as a behaviour tree | H1–H6 |

Group H satisfies requirements 3–7 a second time, through a different mechanism and over the
same interfaces. Groups A, B, D, E and F serve both engines unchanged.
