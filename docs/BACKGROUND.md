# What we are actually doing

Background for the **AcadBot Courier** final project.
[`REPORT.md`](REPORT.md) is the report — *what was built, what it measured, and what broke*;
[`COMPONENTS.md`](COMPONENTS.md) is the piece-by-piece reference. This document covers *what
any of it means*.

It assumes you have written a ROS 2 publisher and subscriber before, and nothing else. Every
section points at the code that implements it.

---

## 1. The problem, stated precisely

A courier robot has to answer one question repeatedly: **"how do I get from here to there?"**
That decomposes into three questions that are genuinely different, and confusing them is the
main way beginners get stuck.

1. **Where am I?** — *localization*. The robot has a map. It does not know where on that map
   it is standing.
2. **How do I get to the goal?** — *planning*. Given where I am and where I want to be, draw
   a route that avoids walls.
3. **What do I do in the next 50 milliseconds?** — *control*. Given the route, what wheel
   speeds keep me on it, right now, without hitting the chair that just appeared.

Nav2 answers all three. **This project answers none of them.** The brief is explicit:

> Under the hood your code is a `navigate_to_pose` action client. You do not write a planner
> or a controller.

What this project builds is the layer *above* those three — the part that decides **which
goals to send, in what order, what to do when one fails, and how to tell a human what is
happening.** That layer is called *mission control*, and it is where a delivery robot stops
being a demo and starts being a product.

So the function we implement is roughly:

```
book(pickup_name, dropoff_name)  ->  job_id | refusal
run(job_id)                      ->  drive to pickup, then to dropoff
                                     ...reporting continuously
                                     ...retrying honestly
                                     ...stopping if asked
                                     -> SUCCEEDED | ABORTED | CANCELED
```

Everything below is the vocabulary needed to understand that one page.

---

## 2. Frames, transforms, and TF

A robot juggles many coordinate systems at once. The LiDAR reports distances *from the LiDAR*.
The map is drawn *from the map's corner*. The wheels measure motion *from wherever the robot
switched on*. Getting a number from one of those into another is a **transform**, and ROS 2
has a subsystem devoted to nothing else: **TF** (`tf2`).

A **frame** is a named coordinate system. A **transform** is the translation and rotation that
takes coordinates in one frame into another. TF stores a *tree* of them, timestamped, and
answers questions like *"where was the LiDAR, in map coordinates, 0.3 seconds ago?"*

AcadBot's tree, from [`PROJECT.md`](../PROJECT.md):

```
map ──(AMCL)──► odom ──(diff_drive)──► base_footprint ──► base_link ──► lidar_link
                                                                    ──► camera_link
                                                                    ──► imu_link
```

Three of these matter, and **the reason there are three is the single most important idea in
this section.**

### 2.1 `base_footprint` — the robot

Where the robot is, projected onto the floor. Everything bolted to the robot (`lidar_link`,
`camera_link`) hangs off it with a **static** transform: the LiDAR never moves relative to
the chassis, so that transform is published once and never changes.

### 2.2 `odom` — smooth, continuous, and wrong

The `odom → base_footprint` transform comes from **wheel odometry**: count wheel rotations,
integrate, and you get a position. In AcadBot it comes from the Gazebo `DiffDrive` plugin
(see [`gazebo_control.xacro`](../ros2_ws/src/acadbot_description/urdf/gazebo_control.xacro)).

Odometry has one wonderful property and one fatal flaw.

- **Wonderful:** it is smooth and continuous. It never jumps. Ask twice in a row and you get
  two nearly-identical answers. Controllers need that.
- **Fatal:** it **drifts**. Wheels slip, radii are never exactly what you typed, and every
  small error is *integrated* — added to a running total that never gets corrected. Error
  grows without bound.

Session 1 of this course measures exactly this: drive a 2 m square twice, open-loop, and see
how far the estimate has wandered. The measured result was **2.35 m of position error and
37.6° of heading error after two laps** — on a path only 16 m long.

### 2.3 `map` — correct, but jumpy

The `map → odom` transform is the **correction**. Something looks at the world, works out
where the robot *really* is, and publishes the difference between truth and what odometry
believes.

That correction jumps. When the localizer suddenly realises the robot is 30 cm further left
than it thought, `map → odom` snaps. Planners can cope with that; controllers cannot.

**This is why there are two frames instead of one.** Nav2's local costmap and controller work
in `odom` — smooth, no jumps, locally accurate. The global costmap and planner work in `map` —
globally correct, occasional jumps. Each layer gets the property it needs.

> **The single most common failure in this whole project** was `map → odom` not existing.
> Without it the global costmap cannot configure, the Nav2 lifecycle manager times out and
> aborts the entire bringup, and every goal afterwards is rejected. The symptom appears three
> layers away from the cause. See [`REPORT.md` §5.1](REPORT.md).

---

## 3. Maps: what an occupancy grid actually is

The map AcadBot navigates is an **occupancy grid** — a bitmap where each pixel is one square
of floor. `academy_map.pgm` is 160 × 120 pixels at `resolution: 0.050`, so each pixel is
5 cm × 5 cm and the whole map covers 8.0 m × 6.0 m.

Each cell holds one of three states:

| state | meaning | how it got that way |
|---|---|---|
| **free** (white) | a LiDAR beam passed *through* and terminated beyond | observed empty |
| **occupied** (black) | a beam *stopped* here | observed a surface |
| **unknown** (grey) | no beam ever reached it | never observed, or *occluded* |

The third one is worth dwelling on. Grey does not mean "empty". It means "no information". The
interior of a wall is grey — the LiDAR sees the near face from each side and nothing in
between. So is the inside of a pillar the robot drove all the way around.

The `.yaml` beside the `.pgm` carries the metadata:

```yaml
image: academy_map.pgm
resolution: 0.050
origin: [-1.054, -1.009, 0]   # where the image's bottom-left corner sits in the map frame
occupied_thresh: 0.65
free_thresh: 0.196
```

`origin` matters more than it looks. **The map frame's `(0,0)` is wherever the robot was
standing when mapping began**, not the corner of the image and not the origin of the
simulation. For AcadBot, mapping started at the spawn point at Gazebo `(-3, -2)`, so:

```
map_x = gazebo_x + 3      map_y = gazebo_y + 2
```

Every coordinate in [`courier.yaml`](../ros2_ws/src/acadbot_courier/config/courier.yaml) is in
the map frame. Mixing the two frames puts your locations outside the building, and Nav2
rejects every one of them. This is a genuine trap — the course's own
`patrol_waypoints.yaml` carries stale world-frame defaults for exactly this reason.

---

## 4. Localization: AMCL and the particle filter

**SLAM** (Simultaneous Localization and Mapping) builds a map while figuring out where you are
— that is Session 2, `slam_toolbox`. **Localization** is the easier half: the map already
exists, and you only need to find yourself in it. That is what a courier does, and the tool is
**AMCL** — Adaptive Monte Carlo Localization.

### 4.1 How it works, in four steps

AMCL maintains a cloud of **particles**. Each particle is a guess: *"maybe the robot is here,
facing this way."* Typically 500–2000 of them ([`nav2_params.yaml`](../ros2_ws/src/acadbot_navigation/config/nav2_params.yaml)
sets `min_particles: 500`, `max_particles: 2000`).

1. **Predict.** The robot moves. Every particle is moved by the same odometry delta, plus
   noise — because odometry is not trusted. The noise magnitudes are `alpha1`…`alpha4`, all
   `0.2` here. The cloud *spreads*.
2. **Weight.** A laser scan arrives. For each particle, ask: *"if the robot really were here,
   what would the LiDAR see?"* Compare against the map. Particles whose predicted view matches
   the real scan get high weight; particles that predict a wall where the scan shows open
   floor get almost none.
3. **Resample.** Draw a new particle set, choosing in proportion to weight. Good guesses
   breed; bad guesses die. The cloud *tightens* around the truth.
4. **Publish.** The weighted mean of the cloud is the pose estimate. AMCL publishes the
   `map → odom` correction that makes it consistent.

The "adaptive" part is that the particle count shrinks when the filter is confident and grows
when it is lost.

### 4.2 Why it needs to be told where to start

AMCL is handed a map it did not build and asked where the robot is *inside* it — a question
with exactly one right answer. It **deliberately does not guess**: `nav2_params.yaml` sets no
initial pose, so AMCL publishes nothing at all until somebody tells it.

That default is reasonable arithmetic. 2000 particles spread over 48 m² of floor **and** 360°
of heading is very thin coverage. Seeding the cloud around a known starting point puts every
particle somewhere useful.

In Sessions 3 and 4 a human supplies it by clicking **2D Pose Estimate** in RViz. Requirement 9
of this project says *one command brings it up*, which means doing that without the mouse —
and that turned out to be the single hardest part of the project. See
[`initial_pose_seeder.cpp`](../ros2_ws/src/acadbot_courier/src/initial_pose_seeder.cpp) and
[`REPORT.md` §5.1](REPORT.md).

### 4.3 Measuring convergence

The uncertainty lives in the covariance matrix of `/amcl_pose`. The position standard
deviation is

```cpp
const double sigma = std::sqrt(std::max({c[0], c[7], 0.0}));   // var(x), var(y)
```

from the course's [`localization_monitor.cpp`](../ros2_ws/src/acadbot_localization/src/localization_monitor.cpp).
Below ~0.25 m the filter is considered converged. Watching sigma fall from 0.43 m to 0.22 m as
the robot drives is the cloud tightening, made numeric.

---

## 5. Costmaps: how a map becomes a thing you can plan on

You cannot plan directly on an occupancy grid, because a robot is not a point. AcadBot is
`robot_radius: 0.20`, so **40 cm across**, and that breaks a grid planner in two different ways.

**The gap is too narrow.** Every cell of a 30 cm gap reads *free*, so a planner treating the
robot as a point routes straight through it — and a 40 cm robot cannot follow that path. Nothing
in the grid records the difference.

**Or the gap fits, but the path does not.** This is the subtler one and the reason inflation
exists rather than a simple width test. An 80 cm corridor holds a 40 cm robot comfortably, 20 cm
either side — but only near the middle. A planner minimising cost over free cells has no reason
to prefer the middle: a route hugging the wall is the same cost as one down the centre, and
often shorter. Every cell it chose was free, and the robot still scrapes.

A **costmap** solves this by giving every cell a cost from 0 to 254, built in **layers**:

| layer | contributes |
|---|---|
| **static** | the saved map — walls that are always there |
| **obstacle** | live LiDAR returns — the chair someone just put down |
| **inflation** | a computed halo of decreasing cost around every obstacle |

**Inflation is the layer that matters most for this project.** Around each occupied cell it
paints a gradient out to `inflation_radius` (0.45 m in `nav2_params.yaml`), highest against
the obstacle and decaying outward. Planners are then free to treat the robot as a *point* and
simply avoid high cost — the inflation has already accounted for its size.

Two consequences that cost this project real time:

- **A goal inside the inflation band is expensive or unreachable.** `lab_bench` was originally
  placed at map `(6.20, 4.20)`, where the gap between a cylinder and the east wall is 1.125 m
  wide — but 0.45 m of inflation from each side leaves only ~0.22 m clear, against a robot
  needing 0.40 m. Every delivery there aborted.
- **A robot that gets *into* the band cannot plan its way out.** The planner fails on its own
  start pose. Observed exactly: `GridBased plugin failed to plan from (3.25, 0.90)` — that
  robot was 0.175 m from a wall with a 0.20 m footprint radius, i.e. inside it.

Nav2 keeps two costmaps: a **global** one (whole map, `map` frame, for the planner) and a
**local** one (a 4 × 4 m rolling window, `odom` frame, for the controller). Recall §2.3 —
each gets the frame whose properties it needs.

---

## 6. Planning versus control

These are two different jobs done by two different servers, and blaming the wrong one wastes
hours.

**The planner** (`planner_server`) draws a route from A to B across the global costmap. It runs
occasionally, thinks globally, and knows nothing about wheels. Here it is configured as:

```yaml
GridBased:
  plugin: "nav2_navfn_planner::NavfnPlanner"
  use_astar: false        # ← Dijkstra
  tolerance: 0.5
  allow_unknown: true
```

**So the algorithm minimising path cost in this project is Dijkstra.** `NavFn` propagates a
potential field outward from the goal across every reachable cell of the global costmap, then
extracts the route by walking downhill through that field from the robot's position.
`use_astar: false` makes the propagation uniform-cost — full Dijkstra. Setting it `true` adds a
straight-line heuristic so fewer cells are expanded.

At this scale the choice barely matters: the map is 160 × 120 cells, so a complete expansion is
under 20,000 cells and finishes far inside the 20 Hz `expected_planner_frequency`. A\* would
save milliseconds. On a building-sized map the difference becomes real.

**What "cost" means here is the part worth understanding**, because it is not distance. Each
cell's cost comes from the costmap layers of §5 — and the inflation layer paints an
exponentially decaying halo (`cost_scaling_factor: 3.0`) out to `inflation_radius: 0.45` around
every obstacle. So the cheapest path is a *compromise* between short and far-from-walls, and
tuning inflation changes what "shortest" means. The other two settings matter as much:
`allow_unknown: true` permits planning through never-observed cells, and `tolerance: 0.5`
accepts a plan finishing within half a metre when the exact goal pose is unreachable.

**The controller** (`controller_server`, running **DWB**) turns that route into velocity
commands at 20 Hz. DWB stands for Dynamic Window Approach, and it is **not a search at all** —
which is the distinction most worth taking from this section. Each cycle it:

```yaml
FollowPath:
  plugin: "dwb_core::DWBLocalPlanner"
  max_vel_x: 0.26 ; max_vel_theta: 1.0
  vx_samples: 20  ; vtheta_samples: 20
  sim_time: 1.7
```

samples 20 × 20 = **400** achievable `(v, ω)` pairs, forward-simulates each for 1.7 s, scores
the 400 resulting trajectories, and publishes the winner. No graph, no path being minimised —
a one-shot choice among 400 candidate futures, repeated 20 times a second.

The scoring is a weighted sum of **critics**, and their weights are the controller's
personality:

| critic | scale | what it rewards |
|---|---|---|
| `PathDist`, `PathAlign` | 32.0 | staying on and aligned with the planner's route |
| `RotateToGoal` | 32.0 | turning to the goal heading on arrival |
| `GoalDist`, `GoalAlign` | 24.0 | making progress toward the goal pose |
| `Oscillation` | — | not dithering back and forth |
| **`BaseObstacle`** | **0.02** | **not being near obstacles** |

Read those numbers: **path-following is weighted 1600× more than obstacle proximity.** DWB as
configured will hug a wall to stay on its line. That is the direct cause of the one route this
project could not make work — see [`REPORT.md` §5.3](REPORT.md).

**None of these values are this project's.** The planner choice, the critic weights and the
inflation radius all arrived in the course's initial commit; `git blame` attributes every line
to the instructor, and `acadbot_navigation` is unmodified on the final-project branch. That is
the brief's rule — *you do not write a planner or a controller* — visible in the history rather
than merely asserted ([`REPORT.md` §2.5](REPORT.md)).

### 6.1 Dijkstra, and what it is actually minimising

Worth doing properly, because "the planner uses Dijkstra" is a sentence people repeat without
being able to say what it means.

**First, the grid is a graph.** Every costmap cell is a node. Each cell is connected to its
neighbours, and the *weight* of stepping into a neighbour is that neighbour's cost — high next
to a wall, low in open space. So "find the cheapest path" is a shortest-path problem on a graph
of ~19,000 nodes, and the thing being minimised is **the sum of the costs of the cells you step
through**, not the number of steps.

That is the sentence to keep: **cheapest ≠ shortest.** A route that detours two metres into open
floor can easily cost less than one that squeezes past a doorframe, because the doorframe cells
carry inflation cost and the open floor does not.

**Dijkstra's algorithm** finds the cheapest route from one source to every reachable node:

1. Give the source cost 0 and every other node cost ∞.
2. Take the unvisited node with the **smallest cost so far** and mark it finalised.
3. For each of its neighbours, check whether reaching it *through this node* is cheaper than
   the best route found so far. If it is, write down the cheaper figure.
4. Repeat from 2 until every reachable node is finalised.

The whole thing is one idea: **always expand the cheapest frontier node next.** When a node
finally comes off that frontier, no cheaper route to it can exist — anything still unexplored
already costs more. That is why the answer is *optimal* and not merely good, and it is also why
the algorithm needs costs to be **non-negative**: a negative-cost edge could make a route you
already finalised retrospectively cheaper. Costmap cells are 0–254, so that never arises.

Picture it as flooding. Water released at the source spreads outward, moving slowly through
expensive cells and quickly through cheap ones; the moment it reaches the goal, the route it
took is the cheapest one. NavFn literally builds that flood — a **potential field**, computed
from the goal outward — and then finds the path by starting at the robot and walking downhill.

**A\* is Dijkstra plus a hint.** Instead of expanding by cost-so-far `g(n)`, it expands by
`g(n) + h(n)`, where `h(n)` estimates the remaining cost to the goal — usually straight-line
distance. The effect is that the flood stops spreading evenly in all directions and pushes
towards the goal instead. As long as `h` never *overestimates* the true remaining cost — the
"admissible" condition, which straight-line distance satisfies because no route can beat a
straight line — A\* returns the same optimal answer as Dijkstra while touching far fewer nodes.

**Which is why `use_astar: false` is defensible here.** A\*'s advantage is fewer expansions, and
this map is 160 × 120. Dijkstra floods every reachable cell and still finishes inside the 20 Hz
planner budget. On a warehouse-sized map the difference stops being academic.

One refinement worth knowing so the output does not surprise you: NavFn does not do textbook
Dijkstra on eight-connected cells, which would produce paths made only of 45° segments. It
computes an **interpolated** potential — solving for a smooth surface rather than discrete
per-cell values — so the gradient descent that extracts the path can run at any angle. That is
why Nav2's global plan looks like a smooth curve rather than a staircase.

### 6.2 Recovery behaviours, and the two levels of retry

When planning or control fails, Nav2 does not give up — it tries a cheap "get unstuck" action and
retries. Most navigation failures are stale information rather than genuine impossibility: a
phantom obstacle from a bad LiDAR return, someone who will have walked past in three seconds, a
robot nudged a few centimetres too close to a wall.

**There are two independent retry layers in this project, and confusing them is the main way to
misread the feedback.**

```
┌──────────────────────────────────────────────────────────────────────────────┐
│  LEVEL 1 — MISSION recovery.  Ours: courier.xml / courier_server.cpp         │
│  a leg fails  →  wait retry_delay (3 s)  →  retry the leg  (3 attempts)      │
│                 →  still failing?  ABORT, naming failed_leg                  │
└───────────────────────────────────┬──────────────────────────────────────────┘
                                    │  one navigate_to_pose goal per attempt
┌───────────────────────────────────▼──────────────────────────────────────────┐
│  LEVEL 2 — MOTION recovery.  Nav2's: bt_navigator's own behaviour tree       │
│  replan at 1 Hz; on failure a RoundRobin advances ONE action per failure:    │
│                                                                              │
│     1. clear local + global costmap    drop stale marks (a service, not a    │
│                                        behavior_server plugin)               │
│     2. Spin        spin_dist 1.57 rad  = 90°, NOT a full turn — re-observe   │
│     3. Wait        5.0 s               let a moving obstacle remove itself   │
│     4. BackUp      0.30 m @ 0.15 m/s   reverse out of a wedge                │
│                                                                              │
│  all inside  RecoveryNode number_of_retries="6"  — six rounds, ONE goal      │
│  →  exhausted?  the goal comes back ABORTED, and Level 1 hears about it      │
└──────────────────────────────────────────────────────────────────────────────┘
```

Read the nesting carefully: Nav2 burns up to six recovery rounds **inside a single goal**, and
only when those are exhausted does it abort — at which point *one* of our three attempts has
failed. So a job that ends in failure may have seen twenty-odd recovery actions.

**Who owns what.** `behavior_server` *hosts* the behaviours, each as its own action server
(`/spin`, `/backup`, `/wait`, …). It does not decide anything. The **navigator's behaviour tree**
chooses which fire and in what order — which is why the escalation is configuration rather than
compiled-in code, and why swapping `default_nav_to_pose_bt_xml` changes the whole strategy.

Two consequences worth knowing. `behavior_plugins` in `nav2_params.yaml` lists **five** plugins,
but the default tree references only three of them plus the costmap clear — `drive_on_heading`
and `assisted_teleop` are loaded and unreachable. And rung 1 is nearly useless against a *real*
obstacle: clearing the costmap removes the mark, the LiDAR re-observes it immediately, and the
round is spent. Clearing is for phantoms.

**The number this project surfaces.** Nav2's feedback carries `number_of_recoveries` and the
courier republishes it. It counts recovery *invocations* — including costmap clears, and across
all three `RecoveryNode`s in that tree — so it is not a count of spins. It remains the single
most diagnostic number available: **zero means a clean drive; fifteen means the robot is fighting
the building.** Parameters and provenance are in [`REPORT.md` §2.5](REPORT.md).

---

## 7. Topics, services, actions — the heart of this project

ROS 2 offers three communication patterns. Requirement 3 of the brief is essentially a test of
whether you know which is which.

### 7.1 Topics — a broadcast

Anonymous, one-way, many-to-many. A publisher shouts; whoever subscribed hears it. No reply,
no delivery guarantee, no idea who is listening.

Good for continuous streams: `/scan`, `/odom`, `/tf`, `/cmd_vel`. Bad for anything where you
need to know it worked.

### 7.2 Services — a function call

Request in, response out, one client, one server, blocking until answered. Like calling a
function across the network.

Good for **transactions that are effectively instant**: *"is this location valid?"*,
*"reserve a job id"*. Bad for anything slow, because the client sits there waiting, learning
nothing.

### 7.3 Actions — a long job you can watch and stop

An action is the answer to *"do this thing, it will take a while, tell me how it is going, and
let me stop it."* Three message parts:

| part | carries |
|---|---|
| **Goal** | what to do — sent once |
| **Feedback** | progress — streamed continuously while it runs |
| **Result** | the outcome — sent once, at the end |

Plus a lifecycle: a goal can be **accepted or rejected**, and once running it can be
**cancelled**. It ends in exactly one terminal state — `SUCCEEDED`, `ABORTED`, or `CANCELED`.

**This is the whole design of the courier.** Booking a delivery is a transaction — instant,
either accepted or refused — so it is a *service*. Executing a delivery takes minutes, has
things to say throughout, and must be interruptible — so it is an *action*. Requirement 1 says
"the reply comes back immediately"; requirement 3 says "it runs for minutes, it streams
feedback, it can be cancelled". Those are the definitions above, restated.

```
# RequestDelivery.srv                # ExecuteDelivery.action
string pickup                        string job_id
string dropoff                       ---
---                                  bool   success
bool   accepted                      string failed_leg
string job_id                        ---
string reason                        uint8   leg
                                     float32 distance_remaining
```

### 7.4 Terminal states are a promise

`SUCCEEDED` means the thing happened. Requirement 7 puts it bluntly:

> A robot that reports success for a delivery it did not make is the one unforgivable bug in
> this project.

That is why the courier calls `abort()` and never `succeed()` on failure, and why `success`
can only be set from the dropoff leg returning `SUCCEEDED`.

---

## 8. Nested actions, and the deadlock that eats beginners

The courier is an action **server** (it serves `ExecuteDelivery`) and simultaneously an action
**client** (it calls Nav2's `navigate_to_pose`). That nesting has a well-known failure mode.

An **executor** is the loop that runs a node's callbacks. A *single-threaded* executor runs
exactly one at a time. So if a callback blocks — waiting for a future, sleeping, spinning —
**nothing else in that node can run**, including the callback that would deliver the very
result it is waiting for. The node deadlocks, permanently, and the stack trace tells you
nothing useful.

Two ways out:

1. **A multi-threaded executor with reentrant callback groups.** Works, but now callbacks
   genuinely interleave and every shared member needs a mutex.
2. **Never block.** Structure the node so every step is a callback or a timer, and no function
   ever waits. Then a single-threaded executor is sufficient — and because callbacks are
   serialized by construction, **no mutex is needed anywhere**.

This project takes the second route, which is also what the course's
[`patrol_commander.cpp`](../ros2_ws/src/acadbot_control/src/patrol_commander.cpp) does. The
brief tells you to read that file first for exactly this reason.

The shape looks like this — send, then hang callbacks off the result:

```cpp
rclcpp_action::Client<NavigateToPose>::SendGoalOptions opts;
opts.goal_response_callback = [this](NavGoalHandle::SharedPtr h) {on_nav_goal_response(h);};
opts.feedback_callback      = [this](auto, auto fb)             {on_nav_feedback(fb);};
opts.result_callback        = [this](const auto & r)            {on_nav_result(r);};
nav_client_->async_send_goal(goal, opts);      // returns immediately
```

There is one subtlety worth knowing cold: **you cannot finish a goal from inside
`handle_cancel`.** The goal does not enter the `CANCELING` state until that callback has
returned `ACCEPT`, so calling `canceled()` from within it throws. The courier defers the
finish by one spin.

---

## 9. State machines and behaviour trees

The courier's mission logic — *go to pickup, then dropoff; retry a failed leg; give up after
N tries; stop if cancelled* — can be expressed two ways.

### 9.1 A finite state machine

States and transitions, written by hand. The courier's is two axes rather than one enum:

```
leg   ∈ { PICKUP, DROPOFF }
phase ∈ { NAVIGATING, RETRY_WAIT, CANCELING }
```

Two small enums compose to six states but read better and map one-to-one onto what the
feedback publishes. Explicit, debuggable, and every transition is visible in the source.

### 9.2 A behaviour tree

The same logic as a *tree* of composable nodes. Both versions are built here — the state
machine in `courier_server` and the tree in `courier_bt_server` — so this is a description of
working code, not of an alternative.

**Ticking.** Nothing in a tree runs continuously. Something outside it calls `tickOnce()`, and
that tick travels from the root down to whichever leaf is currently active. Every node returns
one of three things:

| status | meaning |
|---|---|
| `SUCCESS` | this node finished, and it worked |
| `FAILURE` | this node finished, and it did not work |
| `RUNNING` | not finished — ask me again next tick |

`RUNNING` is the whole idea. A leaf that drives a robot cannot answer in one tick, so it
returns `RUNNING` and *is asked again*. The tick must never block: if a leaf sat waiting for
Nav2 to arrive, the tick would never return, the executor would never run another callback,
and the answer the leaf is waiting for could never be delivered — the deadlock from §8, in a
new costume. In BehaviorTree.CPP the shape that expresses this correctly is a
`StatefulActionNode`, with three entry points: `onStart` (fire the request), `onRunning`
(called each tick — has the answer arrived?), and `onHalted` (someone stopped me; clean up).

**The node types.** There are only four kinds, and that is the point:

- **Action** — a leaf that does something. `GoToLocation` here.
- **Condition** — a leaf that only answers a question, never acts.
- **Control** — has many children and decides between them. **Sequence** runs them in order
  and stops at the first `FAILURE` ("and then"); **Fallback** tries them in order until one
  succeeds ("or else").
- **Decorator** — wraps exactly *one* child and changes its meaning.
  `RetryUntilSuccessful` re-runs a failing child N times; `Timeout` gives it a deadline;
  `Delay` waits before running it; `Inverter` swaps success and failure.

**Halting.** When a `Timeout` expires, it *halts* its child: the child's `onHalted` is called
and it must abandon whatever it started — here, cancel the Nav2 goal. Halting is how a tree
interrupts work that is still in progress, and it is the mechanism a cancelled mission uses
too, from the root down.

**The blackboard.** Nodes never call each other, so values move through a shared key–value
store. In the XML, `location_name="{pickup_name}"` means *read the entry `pickup_name`*, while
`leg="pickup"` is a literal. Each such attribute is a **port**, and ports are typed — a
mismatch is caught when the tree is built. That check is stricter than it first looks:
`Timeout`'s `msec` port is `unsigned`, so writing that entry as a plain `int` fails at *tree
creation time* with a message about the entry having two types. It compiles perfectly.

**Where structure lives.** Not in the source:

```xml
<RetryUntilSuccessful num_attempts="{max_attempts}">
  <Timeout msec="{leg_timeout_msec}">
    <GoToLocation pose="{pickup_pose}" location_name="{pickup_name}" leg="pickup"/>
  </Timeout>
</RetryUntilSuccessful>
```

That fragment is the courier's per-leg retry and timeout — the two fiddliest parts of the
hand-written state machine — as two decorators nobody had to write. The full tree is
[`courier.xml`](../ros2_ws/src/acadbot_courier/behavior_trees/courier.xml), and changing how
many times a leg is retried is an edit to it, not a rebuild.

Nav2's own `bt_navigator` works exactly this way, which is why the recovery escalation in §6.2
is configurable rather than compiled in. BehaviorTree.CPP 4.9.0 ships in the course image as a
Nav2 dependency.

### 9.3 What the tree actually buys, and what it costs

The honest comparison, since both are built and both were measured
([`REPORT.md` §11](REPORT.md)):

**It buys deletion.** Four pieces of hand-written state disappear: the retry counter, the
retry timer, the leg-timeout timer, and the leg variable with its pickup→dropoff transition.
They become `RetryUntilSuccessful`, `Delay`, `Timeout` and `Sequence`. Less code cannot be
wrong, and these particular decorators are used by every Nav2 installation in the world, so
they are far better tested than anything written for one course project.

**It costs directness.** In the state machine, every transition is a line you can read and a
breakpoint you can set. In the tree, the control flow is in a data file and the order things
happen in is a property of tick traversal. When the courier misbehaves, the state machine tells
you where in the source to look; the tree tells you to reason about a graph.

**And it does not touch the hard part.** The genuinely difficult things in this project were
the async discipline (§8), seeding AMCL (§4.2), and a controller that cuts corners (§6). A
behaviour tree helps with none of them. It restructures the easiest 15% of the work. That is a
real benefit and worth having — but a tree is not a substitute for understanding what the
robot is doing.

The measured difference in behaviour is small and explainable: about **0.4 s slower per
two-leg delivery**, because a tree only changes state when it is ticked, and this one is ticked
at 10 Hz. Four transitions per delivery, up to 100 ms of latency each. Ticking faster costs
CPU; that is the trade, and it is a *tuning* decision rather than a design flaw.

---

## 10. Simulated time, and why it ruins everything

Gazebo publishes `/clock`, and every node with `use_sim_time: true` uses that instead of the
wall clock. This is correct and necessary — a simulation that runs at 0.7× real time must have
its timestamps agree — but it introduces failure modes that do not exist on a real robot.

- **Sim time starts at zero and stays there** for the first seconds while Gazebo initialises.
  A node that reads `now()` during startup gets `0`.
- **Sim time advances at whatever rate the simulation manages.** Measured here: **RTF 0.708**
  steady-state, but effectively ~0.1 during the first few seconds.
- **A timestamp of zero means "the latest available"** to tf2 — which is sometimes exactly
  what you want, and sometimes hides a bug.

Nearly every hard problem in this project traced back to one of those three. Four separate
"is localization up?" checks were written before one was correct, and the first three each
*passed while the system was broken*. [`REPORT.md` §5.1](REPORT.md) has the full autopsy; it
is the most useful thing in this repository to read twice.

---

## 11. Vocabulary, mapped to this repository

| term | one line | where it lives here |
|---|---|---|
| frame | a named coordinate system | `map`, `odom`, `base_footprint` |
| TF | the timestamped tree of transforms between frames | `/tf`, `/tf_static` |
| odometry | pose from integrating wheel motion; smooth, drifts | `/odom`, DiffDrive plugin |
| occupancy grid | the map as free / occupied / unknown cells | `academy_map.pgm` |
| AMCL | particle-filter localization on a known map | `amcl:` block in [`nav2_params.yaml`](../ros2_ws/src/acadbot_navigation/config/nav2_params.yaml) |
| particle | one hypothesis about the robot's pose | 500–2000 of them |
| costmap | the map plus obstacles plus inflation, as cost | `global_costmap`, `local_costmap` |
| inflation radius | halo of cost around obstacles; 0.45 m here | `nav2_params.yaml` |
| planner | draws the global route — `NavfnPlanner`, **Dijkstra** (`use_astar: false`) | `planner_server` |
| path cost | accumulated costmap cell cost, not distance — inflation shapes it | `inflation_radius: 0.45` |
| controller | scores 400 sampled trajectories, picks one (DWB) | `controller_server` |
| critic | one term in DWB's weighted trajectory score | `BaseObstacle.scale: 0.02` |
| recovery | clear / spin / back up / wait when stuck | `behavior_server` |
| lifecycle node | node with configure → activate states | `map_server`, `amcl`, all Nav2 servers |
| topic / service / action | broadcast / function call / long cancellable job | §7 |
| goal, feedback, result | the three parts of an action | [`ExecuteDelivery.action`](../ros2_ws/src/acadbot_courier_msgs/action/ExecuteDelivery.action) |
| executor | the loop that runs callbacks | `rclcpp::spin` in [`courier_server.cpp`](../ros2_ws/src/acadbot_courier/src/courier_server.cpp) |
| behaviour tree | mission logic as a ticked tree of nodes | §9.2, [`courier.xml`](../ros2_ws/src/acadbot_courier/behavior_trees/courier.xml) |
| tick | one traversal of the tree, root to active leaf | 10 Hz timer in `courier_bt_server` |
| `RUNNING` | "not finished, ask me again" — the status that makes trees work | `GoToLocation::onRunning` |
| decorator | node wrapping one child to change its meaning | `RetryUntilSuccessful`, `Timeout`, `Delay` |
| halt | how a tree interrupts a child that is still working | `GoToLocation::onHalted` cancels the Nav2 goal |
| blackboard / port | the typed key–value store nodes pass values through | `{pickup_pose}`, `{max_attempts}` |

---

## 12. Questions you may be asked

**Why is booking a service but delivery an action?**
Booking is a transaction: instant, atomic, either accepted or refused, nothing to report while
it happens. Delivery takes minutes, has continuous progress worth reporting, and must be
interruptible. Those are the definitions of the two primitives (§7).

**Why does the action goal carry only a `job_id` and not the coordinates?**
Because the poses were resolved and frozen when the job was booked. Re-resolving them at
execution time would let an edit to `courier.yaml` move the target of a job that had already
been accepted — the requester is told one thing and the robot does another.

**Why are there two frames, `map` and `odom`?**
`odom` is smooth and drifts; `map` is correct and jumps. Controllers need smoothness, planners
need global correctness (§2.3).

**Why does AMCL need to be told where the robot starts?**
It is given a map it did not build and asked a question with one right answer. Guessing means
spreading 2000 particles over 48 m² and 360° — too thin to converge reliably (§4.2).

**Why doesn't your node need a mutex?**
Because nothing in it blocks, so a single-threaded executor serializes every callback by
construction. That is a deliberate design constraint, not luck (§8). It holds in the behaviour
tree version too: ticks arrive on a timer, on the same thread as the Nav2 callbacks, so the
flag a callback sets and the tick that reads it cannot race.

**You built both a state machine and a behaviour tree. Which is better?**
Neither, for this mission. The tree deletes four pieces of hand-written state — the retry
counter, the retry timer, the leg timeout and the leg sequencing — and replaces them with
decorators used by every Nav2 installation in existence. It gives that up in exchange for
control flow that lives in a data file rather than in readable, breakpointable source. At two
legs and one retry rule the state machine is easier to follow; at a dozen behaviours with
priorities and preconditions the tree wins outright. What matters is that the tree changed
*neither* interface and none of the hard parts of the project (§9.3).

**Why can't a behaviour tree node just wait for Nav2 to finish?**
Because the tick would never return. The thread that runs the tick is the same thread that
delivers Nav2's reply, so a leaf that waits inside `tick()` deadlocks the whole node — the same
trap as §8, reached from a different direction. Leaves return `RUNNING` and get asked again.

**Why does one of your routes fail?**
DWB weights path-following 1600× more than obstacle proximity (`PathAlign.scale: 32.0` versus
`BaseObstacle.scale: 0.02`), so on a tight corner it hugs the wall until its footprint is
inside it. Tuning the controller is explicitly out of scope for this project, so that route is
kept as the honest-failure demonstration instead (§6, [`REPORT.md` §5.3](REPORT.md)).
