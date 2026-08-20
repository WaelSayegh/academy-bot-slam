# AcadBot Courier: a delivery robot that tells the truth

Final project for the Robotics Academy SLAM track. A service books a delivery between two
named places; an action runs it, streaming progress, retrying honestly, and stopping cleanly
when asked. The driving is Nav2's — this is the mission layer above it.

[`BACKGROUND.md`](BACKGROUND.md) builds the theory from first principles.
[`COMPONENTS.md`](COMPONENTS.md) is the piece-by-piece reference. This document is what was
built, what it measured, and what broke.

---

## Table of contents

| | |
|---|---|
| [0. The result in one page](#0-the-result-in-one-page) | headline numbers |
| [1. The task](#1-the-task) | the nine requirements, and what is out of scope |
| [2. The design](#2-the-design) | service vs action, codebase structure and why, the state machine, the inherited Nav2 stack and where our code meets it |
| [3. The procedure, chronologically](#3-the-procedure-chronologically) | eight phases, in the order they happened |
| [4. Results](#4-results) | every measurement |
| [5. Problems faced](#5-problems-faced) | six autopsies, including the one that cost a day |
| [6. Ablations and negative results](#6-ablations-and-negative-results) | four things that did not work |
| [7. Analysis](#7-analysis) | reading the results |
| [8. Engineering](#8-engineering) | build, tests, commits |
| [9. What I would do next](#9-what-i-would-do-next) | |
| [10. Presentation appendix](#10-presentation-appendix) | slides, demo script, questions |
| [11. Bonus: the same mission as a behaviour tree](#11-bonus-the-same-mission-as-a-behaviour-tree) | what it deleted, what it cost, three bugs it introduced |

---

## 0. The result in one page

All nine requirements implemented and verified **on the real stack** — Gazebo, `map_server`,
AMCL, the full Nav2 server set, and the courier.

| # | Requirement | Evidence |
|---|---|---|
| 1 | Service, immediate reply, identifier | `accepted=True, job_id='job_0001'` returned without delay |
| 2 | Reject bad jobs with a reason | 5 distinct refusals, each naming what could have been said instead |
| 3 | Action for execution | both legs driven under one `ExecuteDelivery` goal |
| 4 | Feedback a human can follow | leg, target, distance, attempt, recoveries, state at 2 Hz |
| 5 | Drive with Nav2 | `navigate_to_pose` client; no planner or controller written |
| 6 | Cancel means stop | `CANCELED` in **17 ms** driving, **4.5 ms** mid-retry; robot stopped |
| 7 | Honest failure | retry *saved* one delivery; another reported `failed_leg: dropoff` |
| 8 | Everything in YAML | 4 locations + 5 limits; no coordinate in any `.cpp` |
| 9 | One command | Nav2 `active` **20 s** after launch, no RViz click |

The headline delivery:

```
success: true
message: delivered from 'charging_dock' to 'storage'
failed_leg: ''
attempts_used: 3
Goal finished with status: SUCCEEDED       # ~50 s, 0 recoveries
```

**Plus the bonus:** the same mission implemented a second time as a **BehaviorTree.CPP** tree
(§11). `mission:=bt` runs it over the identical service and action; the state machine is
byte-for-byte untouched. Both engines produce identical results on identical inputs, and the
tree costs about **0.4 s** more per two-leg delivery — which turns out to be the tick rate, not
the tree.

**Two numbers worth the whole report.** The one route that does not complete —
`reception → lab_bench` — fails because DWB weights path-following **1600×** more than
obstacle proximity (`PathAlign.scale: 32.0` vs `BaseObstacle.scale: 0.02`) and drives its own
footprint into a wall. And the hardest single problem, seeding AMCL without a mouse, took
**four** attempts at the verification alone — the first three each *passed while the system
was broken*.

Scale: 2 packages, 3 nodes, 2 interchangeable mission engines, ~2,260 lines of C++ and 74 of
tree XML, 40 commits.

---

## 1. The task

### 1.1 What a courier is

Someone asks for a delivery — *"take this from reception to lab_bench"*. The robot accepts the
job, drives to the pickup, then to the dropoff, reports what it is doing while it moves, and
tells the truth when something goes wrong. If the requester changes their mind halfway, they
cancel and the robot stops cleanly.

### 1.2 The nine requirements

Paraphrased from the brief: accept jobs through a **service** with an immediate reply and an
identifier; **reject** bad jobs with a human-readable reason; run jobs through an **action**;
publish **feedback** at least once a second naming the leg, the destination and the distance
left; drive with **Nav2**; make **cancel** actually cancel; **retry** a configurable number of
times then fail *naming the leg*; put everything configurable in **YAML**; and bring the whole
thing up with **one command**.

The one line that shapes every design decision:

> A robot that reports success for a delivery it did not make is the one unforgivable bug in
> this project.

### 1.3 What is explicitly out of scope

> Under the hood your code is a `navigate_to_pose` action client. **You do not write a planner
> or a controller.**

`acadbot_navigation/config/nav2_params.yaml` is untouched on this branch —
`git log main..HEAD -- ros2_ws/src/acadbot_navigation/` returns nothing. This matters twice in
what follows: it is why the courier is a thin mission layer, and it is why the one failing route
was *documented* rather than fixed by retuning DWB (§5.3). What that file actually configures,
and who chose it, is §2.5.

---

## 2. The design

### 2.1 Service versus action — the central decision

The brief tests one idea above all: do you know which ROS 2 primitive fits which job?

**Booking is a transaction.** It either succeeds or it does not, it takes no measurable time,
and there is nothing to report while it happens. That is a **service**.

**Executing is a journey.** Minutes long, continuously interesting, and it must be
interruptible. That is an **action** — goal, feedback, result, plus accept/reject and cancel.

Splitting them this way also answers "what does the identifier do": the service mints a
`job_id`, and the action goal carries nothing else.

```
string job_id        # ExecuteDelivery.action, Goal — that is the whole goal
```

**The poses are resolved once, at booking, and frozen into the job.** Re-resolving them when
the action runs would let an edit to `courier.yaml` move the target of a job already accepted
— the requester is told one thing and the robot does another.

### 2.2 Codebase structure, and why it is this and not something smaller

Two new packages, three executables, twenty-eight tracked files. The existing tree already had six
packages and a launch-only `acadbot_bringup`, so "add a file to something that exists" was a live
option and was rejected for reasons worth writing down.

```
ros2_ws/src/
├── acadbot_courier_msgs/               THE CONTRACT — what a client depends on
│   ├── srv/RequestDelivery.srv         book a delivery: two names in, an id or a refusal out
│   ├── action/ExecuteDelivery.action   run a booked job: goal, result, six feedback fields
│   ├── CMakeLists.txt                  rosidl generation, nothing else
│   └── package.xml                     no dependency on the implementation — that is the point
│
└── acadbot_courier/                    THE IMPLEMENTATION
    │
    ├── include/acadbot_courier/        a header exists where a 2nd .cpp needs the declaration
    │   ├── types.hpp                   Pose2D, Leg, JobState, Job — the frozen poses live here
    │   ├── location_table.hpp          name → pose lookup, and the rejection message
    │   ├── location_markers.hpp        the RViz arrows
    │   ├── courier_server.hpp          engine 1: Phase, CancelReason, every FSM member
    │   ├── bt_nodes.hpp                engine 2: GoToLocation, LegStatus, NavContext
    │   └── courier_bt_server.hpp       engine 2: the node that owns and ticks the tree
    │
    ├── src/                            three executables' worth of sources
    │   │                               — shared by both engines —
    │   ├── location_table.cpp          discovers locations from parameter overrides; refuses a
    │   │                               bad floor plan at startup rather than at first delivery
    │   ├── location_markers.cpp        draws the table, current target highlighted
    │   │                               — engine 1: the state machine —
    │   ├── courier_server.cpp          booking, action server, Nav2 client, retry, timeout,
    │   │                               the three endings, and main()                (607 lines)
    │   │                               — engine 2: the behaviour tree (bonus) —
    │   ├── bt_nodes.cpp                GoToLocation: onStart sends, onRunning polls,
    │   │                               onHalted cancels. Nothing waits.
    │   ├── courier_bt_server.cpp       the same plumbing, leg loop replaced by a 10 Hz tick,
    │   │                               and its own main()
    │   │                               — making the demo need no mouse —
    │   └── initial_pose_seeder.cpp     seeds AMCL and *verifies* it took, by requiring the
    │                                   map→odom stamp to advance
    │
    ├── behavior_trees/courier.xml      the whole mission as data: two legs, retry, timeout,
    │                                   inter-attempt delay. 30 lines, no SubTree.
    ├── config/courier.yaml             locations and limits, one /**: block read by both
    │                                   engines so they cannot drift apart
    ├── launch/courier.launch.py        sim → localization → seed → Nav2 → courier → RViz,
    │                                   in that order for reasons (§5.1)
    ├── rviz/courier.rviz               map, costmaps, and the location markers
    ├── CMakeLists.txt                  three targets; location_* compiles into both couriers
    ├── package.xml
    └── README.md                       how to run it, and what it does *not* do

tools/                                  NOT BUILT — no package.xml, colcon never sees it
├── fake_nav2.py                        a navigate_to_pose that aborts on command, so the
│                                       courier is testable in seconds without Gazebo
└── README.md                           the three switches, and two traps in the harness itself

docs/                                   BACKGROUND · COMPONENTS · REPORT
```

Two things are absent by decision rather than oversight: there is no `config/amcl.yaml` — it
existed for most of the project and turned out to set a parameter to its own default (§6.3) —
and no intermediate library, for the reason given below. [`COMPONENTS.md` § The layout](COMPONENTS.md)
maps the same files to the entries that explain each one.

**Why the interfaces are a package of their own.** So a client can depend on the *contract*
without dragging in the node that implements it. This is not hypothetical tidiness: the moment
anything else in the building wants to book a delivery, it needs `RequestDelivery.srv` and
nothing else. Generating the interfaces inside `acadbot_courier` would make every such client
depend on Nav2, `behaviortree_cpp` and an RViz config.

**Why not a file in `acadbot_bringup`.** Because the dependency arrow points the wrong way.
`acadbot_bringup` is launch-only — its entire `CMakeLists.txt` is `install(DIRECTORY launch)`,
and it exists to *compose* the other packages. Anything implemented there would make every
session's bringup depend on the final project, and a client calling `/request_delivery` would
have to depend on a package full of launch files for four unrelated sessions.

**Why not a node in `acadbot_control`.** This is the closest existing home — it is where
`patrol_commander` lives, and the courier is the same category of thing. The reason it is
separate is that the final project is a self-contained deliverable: it can be added, reviewed
or removed without touching a package the course's own sessions depend on. That is a preference
rather than a rule, and someone could reasonably have decided the other way.

**Why C++ rather than a Python node.** The strongest structural decision here, and the evidence
for it arrived from an unexpected direction. The courier is an **action server that holds an
action client**. In `rclcpp` that shape falls out of the API: `handle_accepted` returns
immediately, everything after it is a callback or a timer, one executor serialises the lot, and
no state needs a mutex (§7). `rclpy`'s action server is built the other way round —
`execute_callback` is a blocking function you are expected to loop inside, which is exactly the
shape that deadlocks a node that is both server and client.

The proof is `tools/fake_nav2.py`, a *far* simpler node than the courier, which hit all three
failure modes while being written:

| attempt | what happened |
|---|---|
| `spin_once()` inside `execute` | `Executor is already spinning` → reported as an **aborted goal** |
| blocking `sleep` inside `execute` | cancels never serviced; the executor was stuck inside the callback |
| working version | `ReentrantCallbackGroup` + `MultiThreadedExecutor` — i.e. threads, therefore locks |

A Python courier would meet all three with a nested action client and a dozen members of shared
state instead of a counter. Two secondary reasons: BehaviorTree.CPP is C++ only, so the bonus
would have needed a different library; and every node in this repository is C++ already.

**Why `initial_pose_seeder` is its own executable** rather than folded into the courier: a
delivery service that also quietly sets up localization is harder to defend than a small node
whose name says what it does — and it can be dropped from the launch by anyone who would rather
click **2D Pose Estimate**.

**Why the courier binaries have no intermediate library.** `courier_server` builds from four
sources directly rather than a `courier_core` library plus a thin `main`. Nothing outside the
package links against it, no tests depend on it, and every other package in the tree builds
plain executables. The sharing that does exist — `location_table.cpp` and `location_markers.cpp`
compiling into both engines — needs no library to happen.

**Why the location table is its own translation unit.** Three separate consumers include
`location_table.hpp` — `courier_server.hpp`, `courier_bt_server.hpp` and
`location_markers.hpp` — and `location_table.cpp` compiles into **both** courier executables.
Inlined into the state machine, the behaviour-tree engine would need its own copy, and the two
engines could then disagree about what a location *is*, which would quietly invalidate every
comparison in §11.

The order matters for honesty: `location_markers.hpp` needed the table *before* the behaviour
tree existed, so there were already two consumers on day one. The second engine made a
reasonable decision look prescient rather than causing it. And two defences are deliberately not
offered — there are no unit tests, so no test seam depends on the boundary, and nothing outside
the package links against it. At 80 lines of `.cpp` this would belong inside
`courier_server.cpp` if there were one consumer. There are three.

What the file earns its keep with is not storage but **validation**. `from_parameters()` is
where requirement 8 is enforced — reading the parameter *overrides* rather than a declared list,
so a location exists purely by being written in YAML and there is no second list of names to
keep in step — and where three malformed floor plans are refused at **startup** rather than at
first delivery: a value that is not a list of numbers, a list of the wrong length, and no
locations at all. Skipping a short list instead of throwing would surface an hour later as
*"unknown location"*, sending the reader to the service handler when the fault is a forgotten
yaw. Parsing-and-validating configuration is a different job from running a delivery, and the
file boundary says so.

**Why there are headers at all, and only these.** A header exists where a *second* translation
unit needs the declaration — not one per class as a reflex. `courier_server.hpp` exists because
`courier_server.cpp` and `courier_bt_server.cpp` both construct their class from a `main()` at
the bottom of the same file — the convention the course's own `patrol_commander.cpp` and
`square_driver.cpp` use, and which `initial_pose_seeder.cpp` here already followed.
`location_table.hpp` exists because four other files use the table. The alternative that was
considered and rejected — one big `.cpp` per node with no headers — reads fine at 200 lines and
badly at 583.

**Where the behaviour tree lives, and why it is one XML file.** The tree structure is entirely in
`behavior_trees/courier.xml`: one file, no `<SubTree>`, no `<include>`. The C++ around it is not
a split of the tree but a different kind of thing, and the boundary is fixed rather than chosen:

| in XML | in C++ |
|---|---|
| which nodes, in what order, wrapped in which decorators | what a custom node *does* |
| `num_attempts`, `msec`, `delay_msec` — all tunable without a rebuild | the `rclcpp_action` client, the three callbacks, cancel-on-halt |
| — | owning the ROS node, the executor, and calling `tickOnce()` |

BT.CPP's XML **composes**; it cannot *define* a node. Everything in `courier.xml` except
`GoToLocation` — `Sequence`, `RetryUntilSuccessful`, `Fallback`, `Timeout`, `Delay`,
`AlwaysFailure` — ships compiled inside the library. `GoToLocation` sends a Nav2 goal, so it has
to be C++. And XML does not run itself: something must own the executor and tick it, which is
`courier_bt_server.cpp`. The XML is the score; the C++ is the orchestra and the conductor.

Splitting the *tree* across several XMLs is supported and was rejected. The two legs are written
out longhand rather than factored into a `DriveToLeg` subtree: it costs eight duplicated lines
and buys the whole mission fitting on one screen, which is the main thing a tree has over the
state machine. Subtree port remapping also adds ceremony — explicit remaps or `_autoremap` —
around exactly the part a reader came to see. Nav2 ships a directory of tree XMLs because it has
many distinct behaviours selected at runtime; one mission with two legs is not that case.

**Why `tools/` is not a package.** `fake_nav2.py` is a test harness, not a deliverable. Making it
an `ament_python` package would put it in the install space, give it a `package.xml` implying it
is part of the robot, and add a build step to something run with `python3` directly. It sits at
the repo root, outside `ros2_ws/src/`, so colcon never sees it.

**One place this is arguably wrong** — `courier.launch.py` lives in `acadbot_courier`, where
`acadbot_bringup` is home to every other one-command launch and requirement 9 is precisely a
one-command bringup. The reason is worth stating exactly, because the obvious version of it is
wrong.

`autonomy.launch.py` forwards no `params_file`, and one level down `navigation.launch.py`
accepts a single one that it hands to **both** `localization_launch.py` (AMCL) and
`nav2_stack.launch.py` (the twelve Nav2 servers). For most of this project that looked like a
hard blocker: AMCL was given `acadbot_courier/config/amcl.yaml`, which holds an `amcl:` block and
nothing else, so the same file could not also configure Nav2.

**That reasoning was wrong, and the investigation that killed it is the more useful result.**
A key-by-key diff showed the courier's `amcl:` block and `nav2_params.yaml`'s were identical but
for `transform_tolerance: 1.0`. Then a direct query showed that value is AMCL's own default:

```console
$ ros2 run nav2_amcl amcl               # no params file at all
$ ros2 param get /amcl_probe transform_tolerance
Double value is: 1.0
```

So `amcl.yaml` was 75 lines that changed no behaviour. It has been **deleted**, and
`courier.launch.py` now passes `nav2_params.yaml` to localization and to Nav2 alike — one file,
both roles, which is exactly what `navigation.launch.py` already assumes. Verified rather than
assumed: `/amcl` and `/bt_navigator` both reach `active [3]`, the seeder still localizes,
`map → odom` appears, and `charging_dock → storage` delivers in **56 s** with
`attempts_used: 2` (§4.3).

This also removes the argument that made `courier.launch.py` compose the stack instead of
including `navigation.launch.py` — the params-file conflict it was working around does not
exist. The ordering, which was the last open question, **checks out**:
`navigation.launch.py` starts localization immediately and holds Nav2 behind
`TimerAction(nav2_delay)`, which is structurally identical to what this launch file does by
hand, and it exposes `params_file`, `map`, `localization` and `nav2_delay` as arguments. The
seeder runs concurrently with localization rather than between it and Nav2, so it stays a
sibling action either way.

**So there is no longer a technical reason for this file to live here.** What is left is a
preference: `acadbot_courier` ships runnable on its own, and a deliverable that can be added or
removed without leaving a dangling launch file in a course package is worth something (§9).

Two lessons, and the second is the one worth keeping. First: **a duplicated config file that
sets a parameter to its own default is worse than no file**, because it is a frozen copy — if
the course retunes AMCL's particle counts or motion noise, the courier silently keeps the old
values. That is the same drift argument used to justify a single `/**:` block in `courier.yaml`
(§11.1, H5), applied in one place and violated in another for months. Second: the blocker was
never measured. One `ros2 param get` against a bare node would have dissolved it at any point,
and instead it shaped a launch file, a config file and three paragraphs of this report.

**Where the launch file lives is a separate question, and still open.** `acadbot_bringup`
already `exec_depend`s on all five implementation packages and its two launch files already
reach into four of them through `get_package_share_directory`, so a `courier.launch.py` there
would be the established pattern rather than a departure. The argument for its current home is
narrower: the package ships runnable on its own, and a deliverable that can be added or removed
without leaving a dangling launch file behind in a course package is worth something. A
judgement call, with low stakes either way.

### 2.3 The state machine

Two axes, not one enum, mirroring what the feedback publishes:

```
leg   ∈ { PICKUP, DROPOFF }
phase ∈ { NAVIGATING, RETRY_WAIT, CANCELING }
```

Driven entirely by callbacks and timers. **Nothing blocks** — no `spin_until_future_complete`,
no worker thread, no waiting on a future. That is what makes a single-threaded executor
sufficient for a node that is simultaneously an action server and an action client, and it is
why none of the state needs a mutex.

The one genuinely subtle piece is `CancelReason`. A `CANCELED` result from Nav2 is ambiguous:
the client cancelled, our leg timeout gave up, or Nav2 abandoned the goal by itself. Three
outcomes, so the *reason* is recorded when the cancel is issued rather than guessed at when
the result lands.

This is one of the two mission engines. The same logic exists a second time as a
BehaviorTree.CPP tree in `courier_bt_server`, selected with `mission:=bt`, over the identical
service and action — §11. The state machine described here is unchanged by its existence, which
is deliberate: it is the verified implementation and the demo fallback.

### 2.4 Configured versus deliberately fixed

| configured | fixed in code |
|---|---|
| the four locations (`name: [x, y, yaw]`) | one job at a time |
| `max_retries`, per leg | the two legs, in that order |
| `retry_delay`, `feedback_period`, `leg_timeout` | terminal-state semantics |
| `goal_frame` | jobs remembered for the process lifetime |
| AMCL's initial pose (`initial_x/y/yaw`) | Nav2's planner and controller |

"One job at a time" is fixed on purpose: a running job owns the robot, and a queue would be a
different product with different failure modes.

### 2.5 The navigation configuration — inherited, not chosen

Every number in §4 is a measurement *of a particular Nav2 configuration*, so the configuration
belongs in the report. None of it is this project's work, and the history says so rather than
the author.

**Who set it.** `git blame` attributes every line below to `3b8d932`, the course's initial
commit, authored by the instructor on 2026-06-30:

```
^3b8d932 (Wael Al Sayegh 2026-06-30)   plugin: "nav2_navfn_planner::NavfnPlanner"
^3b8d932 (Wael Al Sayegh 2026-06-30)   use_astar: false
^3b8d932 (Wael Al Sayegh 2026-06-30)   BaseObstacle.scale: 0.02
^3b8d932 (Wael Al Sayegh 2026-06-30)   PathAlign.scale: 32.0
```

The file has changed once since, in `165c76c` on `main`, which *appended* a `docking_server:`
block — Nav2's stock bringup starts that server, it cannot configure without a `dock_plugins`
entry, and the lifecycle manager then aborts the entire bringup and leaves every server
INACTIVE. That predates this project and touched no planner or controller line.

**The algorithm minimising path cost is Dijkstra.**

```yaml
GridBased:
  plugin: "nav2_navfn_planner::NavfnPlanner"
  tolerance: 0.5
  use_astar: false          # ← Dijkstra
  allow_unknown: true
```

`NavfnPlanner` propagates a potential field outward from the goal across the global costmap and
extracts the route by gradient descent from the robot's cell. `use_astar: false` makes the
propagation uniform-cost. At 160 × 120 cells a complete expansion is under 20,000 cells and
finishes well inside the 20 Hz `expected_planner_frequency`, so A\* would buy milliseconds.

**And "cost" is not distance.** It is accumulated per-cell costmap cost, which is shaped by:

```yaml
robot_radius: 0.20 ; resolution: 0.05
inflation_layer:  cost_scaling_factor: 3.0 ; inflation_radius: 0.45
```

Two of those numbers are load-bearing in the results. `inflation_radius: 0.45` is the clearance
a location in `courier.yaml` needs, and is how two of the four locations were found to be badly
placed (§4.2). `robot_radius: 0.20` is why a robot stopped 0.175 m from a wall face has its own
footprint inside the wall, and why the planner then cannot plan out of its own start pose
(§5.3).

**The controller is not a search.** DWB samples `vx_samples: 20` × `vtheta_samples: 20` = 400
velocity pairs per cycle, rolls each forward `sim_time: 1.7` s, scores them with a weighted sum
of critics and publishes the winner, 20 times a second:

| critic | scale |
|---|---|
| `PathDist`, `PathAlign`, `RotateToGoal` | 32.0 |
| `GoalDist`, `GoalAlign` | 24.0 |
| **`BaseObstacle`** | **0.02** |

`max_vel_x: 0.26`, `max_vel_theta: 1.0`. That 1600:1 ratio between path adherence and obstacle
proximity is the single most consequential number in this report — it is the direct cause of
the one route that does not complete (§5.3) and the reason lowering the speed helped but did
not fix it (§6.1).

**The recovery behaviours: five are loaded, three ever run.** When planning or control fails,
Nav2 escalates through recovery actions before giving up. `behavior_server` loads five plugins:

```yaml
behavior_server:
  ros__parameters:
    behavior_plugins: ["spin", "backup", "drive_on_heading", "wait", "assisted_teleop"]
    spin:             {plugin: "nav2_behaviors::Spin"}
    backup:           {plugin: "nav2_behaviors::BackUp"}
    drive_on_heading: {plugin: "nav2_behaviors::DriveOnHeading"}
    wait:             {plugin: "nav2_behaviors::Wait"}
    assisted_teleop:  {plugin: "nav2_behaviors::AssistedTeleop"}
```

But **that list only says what is available, not what is used.** Which recoveries actually fire
is decided by the navigator's behaviour tree, and `bt_navigator` sets no
`default_nav_to_pose_bt_xml` — so Nav2 falls back to its own built-in tree,
`navigate_to_pose_w_replanning_and_recovery.xml`, which ships inside `nav2_bt_navigator` and is
not in this repository at all. Its `RecoveryActions` node is a `RoundRobin`, cycling one action
per failure, inside a `RecoveryNode number_of_retries="6"`:

| order | action | parameters | plugin? |
|---|---|---|---|
| 1 | clear local **and** global costmap | — | no — a service call to the costmaps |
| 2 | `Spin` | `spin_dist: 1.57` | `nav2_behaviors::Spin` |
| 3 | `Wait` | `wait_duration: 5.0` | `nav2_behaviors::Wait` |
| 4 | `BackUp` | `backup_dist: 0.30`, `backup_speed: 0.15` | `nav2_behaviors::BackUp` |

There are two further single-retry recoveries inside the pipeline: a failed `ComputePathToPose`
clears the global costmap and retries once, and a failed `FollowPath` clears the local costmap
and retries once.

So **`drive_on_heading` and `assisted_teleop` are configured and never invoked** — the default
tree does not reference them. They cost nothing but they are dead configuration, and anyone
reading `nav2_params.yaml` alone would reasonably conclude the robot might drive on a heading
to get unstuck. It will not.

**Who implemented them.** Upstream Nav2, in `nav2_behaviors` — the plugin sources sit at
`/opt/ros/jazzy/include/nav2_behaviors/plugins/{spin,back_up,drive_on_heading,wait,assisted_teleop}.hpp`
in the course image, and the behaviour-tree action nodes that call them are in
`nav2_behavior_tree`. The instructor *selected and configured* them in `3b8d932`; `git blame`
puts every line of the block above in that commit. **This project wrote none of them and
invokes none of them directly.**

**Where we use them: nowhere as a caller — only as an observer.** The courier never calls a
recovery. It reads Nav2's running count out of `navigate_to_pose` feedback and republishes it,
because it is the single most diagnostic number available to a human watching a delivery:

| file | line | what it does |
|---|---|---|
| `acadbot_courier_msgs/action/ExecuteDelivery.action` | 34 | `uint16 nav2_recoveries` — the field |
| `src/courier_server.cpp` | 312 | FSM reads `feedback->number_of_recoveries` |
| `src/courier_server.cpp` | 530 | FSM republishes it on the courier's own timer |
| `src/courier_server.cpp` | 364 | a Nav2 ABORT is reported as *"its recoveries were exhausted"* |
| `src/bt_nodes.cpp` | 192 | BT leaf reads the same field into `LegStatus` |
| `src/courier_bt_server.cpp` | 377 | BT engine republishes it |

That number is what makes §4.3 readable: `charging_dock → storage` completes with **0
recoveries**, `charging_dock` at its original coordinate cost **4** every visit, and the failing
`reception → lab_bench` dropoff burns **15** before Nav2 gives up. Zero means a clean drive;
fifteen means the robot is fighting the building.

**Why none of it was changed**, beyond the brief's rule: `nav2_params.yaml` is shared with
Sessions 3 and 4, which are presumably tuned around its current behaviour. Raising
`BaseObstacle.scale` to rescue one courier route would silently alter the demo every other
session depends on — the same shape of argument as the `transform_tolerance` question in §2.2.
[`COMPONENTS.md` F3](COMPONENTS.md) has the full parameter listing.

### 2.6 What we inherit, and where our code touches it

Session 4's lecture walks the Nav2 stack server by server. Almost none of it is ours to build —
but every piece of it is something this project *consumes* at a specific line. This section pairs
each inherited concept with the code that meets it, because "we use Nav2" is not an answer and
`courier_server.cpp:41` is.

**The server team — one goal's journey.** Nav2 is not one node: BT navigator, planner server,
controller server, behavior server, smoothers, lifecycle manager. This project attaches at
exactly one point, an action client, and never talks to any other server directly.

`src/courier_server.cpp:41`
```cpp
nav_client_ =
  rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
```

**Lifecycle nodes — why the stack starts in order.** Every Nav2 server is a managed node, and
the lifecycle manager aborts the *whole* bringup if one fails its transition. Costmaps refuse to
configure without `map → odom`, so Nav2 must not start before localization has produced one.

`launch/courier.launch.py:124-133`
```python
nav2 = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(pkg_nav, 'launch', 'nav2_stack.launch.py')),
    launch_arguments={
        'use_sim_time': 'true',
        'params_file': nav2_params,
    }.items(),
)
delayed_nav2 = TimerAction(
    period=LaunchConfiguration('nav2_delay'), actions=[nav2])
```

The default is 15 s (`launch/courier.launch.py:186`), raised from the course's 12 because the
seeder must win first. The ordering is not cosmetic — §5.1 is the autopsy of getting it wrong.

**Costmaps — the inflation dial.** `inflation_radius: 0.45` against `robot_radius: 0.20` decides
which floor is usable. We never edit it; we *obey* it, and the rule is written where someone
adding a room will read it.

`config/courier.yaml:12-15`
```yaml
# Read your own coordinates off RViz's Publish Point rather than guessing, and
# keep a location at least the costmap inflation_radius (0.45 m) clear of any
# wall or obstacle -- closer than that and Nav2 spends its time fighting the
# goal instead of driving to it.
```

That rule is also enforced visually rather than only in prose: the markers draw every location
over the global costmap, so one sitting inside the inflation band is visible before it costs a
delivery (D1). Two of four were found that way (§4.2).

**The global planner — NavFn, Dijkstra.** Our contribution to planning is one `PoseStamped` in
the right frame with the yaw the YAML asked for. Everything after that is NavFn's.

`src/courier_server.cpp:549-562`
```cpp
geometry_msgs::msg::PoseStamped CourierServer::to_goal_pose(
  const Pose2D & pose) const
{
  geometry_msgs::msg::PoseStamped out;
  out.header.frame_id = goal_frame_;
  out.header.stamp = now();
  out.pose.position.x = pose.x;
  out.pose.position.y = pose.y;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, pose.yaw);
  out.pose.orientation = tf2::toMsg(q);
  return out;
}
```

When the planner cannot plan, we learn about it only as an `ABORTED` result — `src/courier_server.cpp:364`.

**The local controller — DWB.** We never see a velocity command. What reaches us is the
by-product: `distance_remaining`, which we republish on our own clock so a stalled controller
reads as a frozen number rather than as silence (C11).

`src/courier_server.cpp:307-313`
```cpp
void CourierServer::on_nav_feedback(
  NavGoalHandle::SharedPtr,
  const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
  last_distance_ = feedback->distance_remaining;
  last_recoveries_ = static_cast<uint16_t>(feedback->number_of_recoveries);
}
```

DWB also silently defines what this project means by "delivered": `xy_goal_tolerance: 0.20`. Our
success message claims nothing stronger.

**`nav2_params.yaml` — one file runs the whole stack.** Our involvement is a path, passed
through untouched.

`launch/courier.launch.py:68`
```python
nav2_params = os.path.join(pkg_nav, 'config', 'nav2_params.yaml')
```

**The recovery ladder.** Nav2 escalates — re-plan, clear costmap, spin, back up, wait, abort —
inside a single goal. This project invokes none of it and observes all of it, in one field:

| file | line | role |
|---|---|---|
| `acadbot_courier_msgs/action/ExecuteDelivery.action` | 34 | `uint16 nav2_recoveries` |
| `src/courier_server.cpp` | 312 | read from Nav2 feedback |
| `src/courier_server.cpp` | 530 | republished on our timer |
| `src/courier_server.cpp` | 364 | the ABORT, named honestly |
| `src/bt_nodes.cpp` | 192 | the same read, BT engine |
| `src/courier_bt_server.cpp` | 377 | the same republish |

`src/courier_server.cpp:364`
```cpp
case rclcpp_action::ResultCode::ABORTED:
  leg_failed("Nav2 aborted the goal (its recoveries were exhausted)");
```

That is the whole seam between Nav2's retry layer and ours: Nav2 exhausts six rounds inside one
goal, then aborts, and only then does a courier *attempt* count as failed.

**`behavior_server` configuration.** Nothing in this project references it —
`git grep -l behavior_server -- ros2_ws/src/acadbot_courier` returns nothing. That is the
intended answer rather than an omission: which recoveries exist, how fast a spin may be, how far
a back-up goes, all belong to a file this project does not own. The only place the word appears
in our tree is prose explaining that we leave it alone.

**Debugging the full stack.** The lecture's checklist — no `map → odom`, TF extrapolation,
goal rejected because a server never activated — is not theory here; five of its six symptoms
were hit and are written up in §5. One of them is answered by code rather than by a procedure:
the seeder does not trust that localization came up, it *checks*, by requiring the
`map → odom` timestamp to advance.

`src/initial_pose_seeder.cpp:99-113`
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

Existence was not enough, and neither was age — a stale cached transform answers the first, and
sim time frozen at zero defeats the second. Only *advancing* distinguishes a live localizer from
a dead one (§5.1, E2).

---

---

## 3. The procedure, chronologically

### Phase 0 — Reading the ground

Read the whole repository first. The brief says to read `patrol_commander.cpp` before writing
anything, and it is right for a reason it does not state: that file is a working example of
the exact composition needed (a C++ Nav2 action client, fully async) **and** it gets three
things wrong for a courier — it discards the goal handle so cancel is impossible, it treats
`SUCCEEDED`, `ABORTED`, `CANCELED` and *rejected* identically, and it has no retry.

Also found: the map-frame offset (`map = gazebo + (3, 2)`) and the fact that
`patrol_waypoints.yaml`'s compiled defaults are stale world-frame values that land outside the
building — a live demonstration of why requirement 8 exists.

### Phase 1 — The interfaces

`acadbot_courier_msgs` first, because it is the only piece with zero dependency on anything
unverified and everything else compiles against it. Built and confirmed with
`ros2 interface show`.

### Phase 2 — Location validation, and a detour

Drove all four candidate locations as raw `navigate_to_pose` goals to check they were
reachable. Results in §4.2. `lab_bench` at `(6.20, 4.20)` aborted from both directions.

**This phase was sequenced wrong**, and it is worth recording. The early code does not depend
on real coordinates — placeholders would have carried for days — and a "read off four
coordinates" task turned into an extended Nav2 investigation. What it *did* produce was
requirement-9 knowledge that would otherwise have been discovered later: the seed must precede
Nav2, and `distance_remaining` is populated.

### Phase 3 — Value types and the location table

`types.hpp`, `location_table.*`, `location_markers.*`, one file at a time, syntax-checked
individually with `g++ -fsyntax-only` against the ROS include tree so each could be verified
before the package was buildable.

The location encoding changed twice under review: three scalar parameters per location became
one `[x, y, yaw]` array (fewer lines, reads like a table), and the `locations.names` list was
then removed entirely in favour of reading parameter overrides — because a list and a set of
poses can disagree in one direction silently.

### Phase 4 — The courier node

`courier_server.hpp` first, agreeing the state machine shape before any implementation:
the two axes, the event list, the transition table, the two cancel races. Then
`courier_server.cpp`, with `main()` then in its own file. First successful build of the package.

### Phase 5 — Testing against a stand-in

Rather than bring up Gazebo for every iteration, wrote a `rclpy` `navigate_to_pose` server that
succeeds, aborts, hangs or rejects on command. That turned a two-minute test cycle into five
seconds and made the failure paths *reproducible on demand* instead of hoping the real robot
misbehaved conveniently.

All of requirements 1–7 verified here before the real stack was involved, including both
cancel paths.

### Phase 6 — The launch, and the seeding problem

`courier.launch.py`, `courier.rviz`. Then requirement 9's real difficulty: AMCL
publishes nothing until told where the robot is, and "one command" cannot mean "one command
and then go and click something".

This took eight attempts across two mechanisms and is the subject of §5.1.

### Phase 7 — Real-stack verification

Every requirement re-verified against Gazebo + AMCL + Nav2. Requirement 7 was proven by an
accident rather than a contrived test: Nav2 genuinely exhausted its recoveries on the
`lab_bench` dropoff, and the courier reported the failure naming the leg.

### Phase 8 — Route selection

With `reception → lab_bench` failing reproducibly, tested `charging_dock → storage` on the
**stock** stack — no tuning, nothing modified. It completes. That became the delivery to
demonstrate, and the failing route was kept deliberately as the honest-failure demonstration.

### Phase 9 — The behaviour tree, and a review that paid for itself

Only after all nine requirements were verified. The tree was written, it compiled, and it ran a
delivery — at which point a deliberate review found **three** defects in it, each one something
the state machine already handled correctly (§11.4). One of them would have left the robot
driving after the client was told the delivery was cancelled.

The lesson is about sequencing rather than about trees: a second implementation of working
behaviour is *the* situation where "it ran successfully" is worthless as evidence, because the
happy path is the easy part and it is already known to be reachable. What the review actually
did was compare the new code against the old one line by line, asking of every piece of state in
`courier_server.hpp` where it went. Two of the three bugs fell straight out of that question.

Phase 9 also produced the only tooling kept in the repository,
[`tools/fake_nav2.py`](../tools/fake_nav2.py) — the scratchpad version had been lost once
already, and it was the thing that made all three fixes verifiable in seconds.

---

## 4. Results

### 4.1 The nine requirements, on the real stack

```
# req 1, 2
accepted=True,  job_id='job_0001', reason=''
accepted=False, job_id='',  reason="unknown pickup 'kitchen'; known locations are:
                                    charging_dock, lab_bench, reception, storage"
accepted=False, job_id='',  reason="pickup and dropoff are both 'reception';
                                    there is nothing to deliver"

# req 3, 4, 5, 7 — the successful delivery
job_0001: pickup leg, attempt 1 of 3 -> 'charging_dock' (0.50, 2.00)
job_0001: pickup leg, attempt 1 failed: Nav2 aborted the goal
job_0001: retrying the pickup leg in 3.0s (attempt 2 of 3)
job_0001: reached pickup 'charging_dock'
job_0001: dropoff leg, attempt 1 of 3 -> 'storage' (5.50, 0.60)
job_0001: delivered from 'charging_dock' to 'storage'

# req 6
terminal status  : CANCELED
result.success   : False
result.failed_leg: ''

# req 9
bt_navigator: active [3]        # t = 20 s, no RViz click
```

### 4.2 Location reachability

Measured by sending each candidate as a raw `navigate_to_pose` goal.

| location | map (x, y, yaw) | result | recoveries |
|---|---|---|---|
| `reception` | 0.60, 4.20, 0.00 | SUCCEEDED | 0 |
| `storage` | 5.50, 0.60, 0.00 | SUCCEEDED | 0 |
| `charging_dock` *(original)* | 0.50, 0.50, −1.57 | SUCCEEDED | **4** |
| `lab_bench` *(original)* | 6.20, 4.20, 3.14 | **ABORTED** | 16 / 0 / 5 |

Two were moved as a result:

- **`lab_bench` 6.20 → 4.50.** Reaching the north-east corner means threading the gap between
  `obstacle_2` (a 0.3 m cylinder at map `(5.5, 3.5)`) and the east wall at `x ≈ 6.925`. That
  gap is **1.125 m** of physical space, of which **~0.22 m** survives 0.45 m of inflation from
  each side — against a robot needing 0.40 m. It aborted from the west (16 recoveries, wedged
  at `(2.88, 1.08)`) and from the east (0 recoveries, wedged at `(6.01, 1.71)`), and once
  wedged, the next goal failed too.
- **`charging_dock` (0.50, 0.50) → (0.50, 2.00).** `obstacle_1` is a 0.5 m box at map
  `(1.0, 0.5)`, edge at `x ≈ 0.75`. The goal sat 0.25 m from it, inside the inflation band —
  four recoveries on every visit.

### 4.3 Delivery routes

| route | outcome | time | recoveries | attempts |
|---|---|---|---|---|
| `charging_dock → storage` | **SUCCEEDED** | ~50 s | 0 | 3 |
| `reception → lab_bench` | pickup ok, **dropoff ABORTED** | — | 15 | 4 |

The successful one is richer than a clean run: Nav2 aborted the first pickup attempt, the
retry recovered it, and the delivery completed. **Requirement 7's retry demonstrated in the
success path**, not only the failure path.

### 4.4 Cancel

| cancel landed | courier → terminal | chain |
|---|---|---|
| robot driving at 0.19 m/s | **17 ms** | `controller_server: Cancellation was successful. Stopping the robot.` |
| during the 3 s retry delay | **4.5 ms** | no Nav2 goal in flight; deferred finish |
| at `distance_remaining: 0.19` | `CANCELED` | inside the 0.20 m goal tolerance and still not reported as success |

`/cmd_vel` measured at 0.19 m/s immediately before the cancel, and silent afterwards.

### 4.5 Bringup and environment

| quantity | value |
|---|---|
| Nav2 `active` after launch | 20 s |
| seeder attempts needed | 2 |
| simulation real-time factor | 0.708 |
| `/scan` rate | 7.7 Hz |
| map | 160 × 120 cells @ 0.05 m = 8.0 × 6.0 m |
| map origin | `[-1.054, -1.009]` |

---

## 5. Problems faced

### 5.1 Seeding AMCL without a mouse — eight attempts

The single hardest problem, and the most instructive.

**The requirement.** AMCL is handed a map it did not build and refuses to guess where the
robot is, so it publishes nothing until told. Without `map → odom` the Nav2 costmaps cannot
configure, the lifecycle manager times out and aborts the *entire* bringup, and every goal
afterwards is rejected. The symptom appears three layers from the cause.

**Attempt 1 — `set_initial_pose`.** The obvious tool. The parameter loaded correctly and AMCL
genuinely applied it:

```
[amcl] Setting pose (0.000000): 0.000 0.000 0.000
```

**t = 0.** AMCL applies it the instant it activates — ~0.4 s after its process starts, before
it has received a single `/clock` message — so under `use_sim_time` the pose is stamped at a
time for which no `odom→base_footprint` has ever existed. Accepted, and no transform ever
validates.

**Attempt 2 — delay localization by 8 s**, so the clock is running. Still `(0.000000)`. The
race is between AMCL's own activation and its own first clock message, so it moves along with
the delay. *This attempt also made things worse* (attempt 6).

**Attempt 3 — `transform_tolerance: 1.0`.** No effect. tf2 never permits **future**
extrapolation regardless of tolerance; the parameter only admits stale data.

**Attempt 4 — publish `/initialpose` three times.** All three lost a millisecond-scale race:

```
Requested time  9.867 but the latest data is at time  9.860    (7 ms into the future)
Requested time 10.571 but the earliest data is at time 10.900  (buffer starts later)
```

**Attempt 5 — publish ten times, later.** Still lost.

**Attempt 6 — start localization with the sim again.** The delay from attempt 2 was actively
harmful: AMCL keeps its own TF buffer, and that buffer only starts filling when the node does,
so a late start leaves it holding a single sample.

**Attempt 7 — a seeder node that verifies.** Publish, check, repeat until it worked. Right
idea; wrong check, three times over:

| check | why it lied |
|---|---|
| does `map→odom` exist? | tf2 caches ten seconds and `TimePointZero` means *latest available* — a transform emitted once and abandoned kept answering yes |
| exists three times running? | three checks a second apart all read the same dead transform |
| how old is it? | age is in **simulated** time, which barely advances while Gazebo starts — exactly the window the check polices |

**Attempt 8 — ask whether the stamp is _changing_.**

```cpp
const bool advancing = have_stamp_ && stamp > last_stamp_;
```

A stamp that keeps moving means AMCL is still publishing. That holds whatever the clock is
doing. **Nav2 active at 20 s, two published attempts, no click.**

Two things also learned along the way. A side experiment stamping the pose three ways — zero,
`now()`, `now() − 0.5 s` — showed **all three working** once TF was mature, so the timestamp
was never the discriminating variable, contrary to what four attempts assumed. And the seeder
needed a second gate nobody would guess: **AMCL only publishes `map→odom` off the back of a
filter update, and a filter update needs a scan.** Seeding before the laser streams gets the
pose accepted, a transform emitted briefly, then silence.

**The lesson, which is the most transferable thing in this project:** the fix was right on
attempt 7; the *verification* was wrong three times. Each check tested something adjacent to
what mattered and reported success on a broken system.

### 5.2 `map_server` and parameter precedence

AMCL sat on `Waiting for map....` forever. `map_server` was `active` but empty:

```
map_server: yaml-filename parameter is empty, set map through 'load_map'-service
$ ros2 param get /map_server yaml_filename
String value is:
```

Cause: I had written `yaml_filename: ""` into `amcl.yaml` intending it to be explicit and
self-documenting. `nav2_bringup`'s `localization_launch.py` passes the map as
`parameters=[params_file, {'yaml_filename': map_yaml_file}]` — and **the params file wins over
the dict override**, the opposite of the precedence I assumed. Declaring it did not document
the map source; it silently blanked it.

Fix: no `map_server` block at all — which is exactly why `nav2_params.yaml` has none either.

### 5.3 The corner-cutting controller

`reception → lab_bench` reaches the pickup, then fails the dropoff:

```
robot at (3.25, 0.90)
GridBased plugin failed to plan from (3.25, 0.90) to (4.50, 4.20)
distance_remaining: min 0.0, last 0.0        # never had a valid plan
15 recoveries
```

The divider's west face is at map `x = 3.425`. The robot sits at `x = 3.25` — **0.175 m away,
with a 0.20 m footprint radius.** Its footprint is inside the wall. That is why the planner
cannot plan and `backup failed`: it is in collision, not merely near an obstacle.

The corridor itself is fine — 1.925 m of gap, 1.025 m still clear after inflation, and
`spawn → storage` crosses it with **zero** recoveries. The problem is the *approach*: from the
far north-west the robot runs down the divider's west face and clips the tip while rounding
it. `nav2_params.yaml` sets `BaseObstacle.scale: 0.02` against `PathAlign.scale: 32.0`, so DWB
weights staying on the line 1600× more than keeping clear of walls.

Fixing that means retuning the controller, which the brief puts out of scope. So the route is
kept as the honest-failure demonstration, which is what it is good at.

### 5.4 Three bugs in my own test scripts

Each hid or faked a result and each cost a full run:

- `grep -o "job_[0-9]*"` matches **zero** digits, so it also matched the field name `job_id`
  and passed `"job_\njob_0002"` as an id. Two tests silently never ran, and a third "passed"
  for the wrong reason.
- `timeout 4 ros2 run tf2_ros tf2_echo map odom` — `tf2_echo` prints "Waiting for transform"
  a couple of times before resolving and needs ~5 s. The timeout killed it at the boundary and
  **reported failure on a working stack**, which sent me back into §5.1 for another round.
- `head -24` truncated the cancel client at exactly the line before the terminal status — the
  one line that test existed to produce.

**A check that truncates is a check that can lie**, and it lies toward "no result", which
reads as failure.

The eventual fix for the second was to stop probing with `tf2_echo` at all and wait on
`bt_navigator` reaching `active` — which *cannot* happen without `map→odom`, so it proves the
same thing without needing a clock of its own to be right about.

### 5.5 Two rclcpp API surprises

`declare_parameter<T>(name)` with no default throws
**`UninitializedStaticallyTypedParameterException`** — not the name anyone guesses. Checked in
the Jazzy headers rather than assumed.

`automatically_declare_parameters_from_overrides` looks like the right way to read a location
table, but it declares *every* key at construction, so the node's own
`declare_parameter("max_retries", 2)` then throws `ParameterAlreadyDeclaredException`. Every
parameter would need a `has_parameter` guard to fix one feature. Reading
`get_parameter_overrides()` directly touches nothing else.

### 5.6 A name collision worth knowing

A private static `to_string(Phase)` would **hide** the free `to_string(Leg)` and
`to_string(JobState)` from `types.hpp` — member lookup wins — so every call inside the class
would need qualifying. Renamed to `phase_name`.

---

## 6. Ablations and negative results

### 6.1 Reducing controller speed — partial, and revealing

Hypothesis (proposed after watching a run): 0.26 m/s and 1.0 rad/s are too fast, and the robot
overshoots into walls.

| speed | result |
|---|---|
| 0.26 / 1.0 (stock) | wedges at `(3.24, 1.22)`, planner fails |
| 0.20 / 0.70 | **planner failures gone**; leg times out at 120 s, 7 recoveries |
| 0.20 / 0.70, 240 s timeout | still stuck at `(3.25, 0.90)`, 15 recoveries |

**Partly right.** Slowing down genuinely stopped the robot wedging hard enough to break the
planner — the `Failed to create plan` errors vanished at 0.20 m/s. It did not get the robot
round the corner, because the cause is critic weighting rather than speed. Kept out of the
final configuration: it costs a third more delivery time for a fix that does not fix.

An earlier speed test was **invalid** and worth flagging as a methodology note: all three
`/initialpose` messages were rejected on TF races, so localization never established and every
goal failed for want of a map, not for want of speed. Changing two things at once (speed *and*
the launch composition) hid it.

### 6.2 `set_initial_pose` — the right-looking wrong tool

Covered in §5.1. Loads correctly, is applied correctly, and cannot work under `use_sim_time`
because activation precedes the first `/clock` message. No delay fixes it.

### 6.3 `transform_tolerance` — a misreading of tf2

Set to 1.0 expecting it to absorb a 7 ms overshoot. It does not: tf2 refuses **future**
extrapolation regardless of tolerance, which only permits *stale* data. It was kept in
`amcl.yaml` anyway, on the reasoning that it is correct to set explicitly.

**It was not even a change.** Months later, a direct query showed 1.0 is AMCL's own default:

```console
$ ros2 run nav2_amcl amcl               # no params file at all
$ ros2 param get /amcl_probe transform_tolerance
Double value is: 1.0
```

So this ablation is stronger than it first read. The parameter did not fail to fix the problem —
it was never a modification at all, and the entire `amcl.yaml` that existed to carry it was
75 lines of no-op (§2.2). The failed hypothesis cost one debugging session; *not measuring the
supposed fix afterwards* cost a config file, a launch-file design decision, and three paragraphs
of this report that argued from it.

### 6.4 Delaying localization — actively harmful

Introduced to give the clock time to start; removed once measured. AMCL's TF buffer begins
filling when the node does, so starting it late leaves it with a single sample and makes the
seeding race *worse*. A fix for a wrong diagnosis that broke a working configuration.

---

## 7. Analysis

**Why the successful route works and the other does not.** Both cross the same corridor.
`charging_dock → storage` approaches it from mid-height and passes through the clear band;
`reception → lab_bench` descends the divider's west face and has to round a corner at the tip.
With obstacle proximity weighted at 0.02 against path-following at 32.0, DWB will hug the wall
to stay on its line — fine in open space, fatal at a 0.19 m clearance.

**Why the failure is a good result, not a hidden one.** Requirement 7 asks the courier to
retry a configurable number of times and then fail *naming the leg*. That is exactly what
happens, against a real Nav2 failure rather than a stubbed one:

```
attempt 1 failed → attempt 2 failed → attempt 3 failed
success: false, failed_leg: dropoff, attempts_used: 4
```

A courier that silently skipped the leg — which is what `patrol_commander` does with a
rejected goal — would have reported the delivery done.

**Why the retry matters in the success path too.** `charging_dock → storage` needed
`attempts_used: 3`: Nav2 aborted the first pickup attempt, the retry recovered it. The same
mechanism saves a delivery here and refuses to lie about one there.

**Why no mutex is needed.** The node is an action server and an action client at once — the
classic deadlock setup. Because nothing blocks, a single-threaded executor serializes every
callback by construction. That is a design constraint held deliberately, and it is why the
state machine can be read linearly.

**Why cancel is fast.** 17 ms driving, 4.5 ms mid-retry. The retry case is the interesting
one: no Nav2 goal is in flight, so no result is coming, and without the deferred finish the
action would hang forever waiting for one.

---

## 8. Engineering

**Build.** Two ament_cmake packages, C++17, `-Wall -Wextra -Wpedantic`, no warnings.
Interfaces generate cleanly and are confirmed with `ros2 interface show`.

**Incremental verification.** Each source file was syntax-checked on its own with
`g++ -fsyntax-only` against the ROS include tree before the package was buildable, so errors
surfaced one file at a time rather than in a heap at first link.

**The stand-in Nav2** is the piece of tooling that paid for itself most. A `rclpy`
`navigate_to_pose` server that aborts, stalls or succeeds on command turned a two-minute test
cycle into three seconds and made the failure paths reproducible on demand. Requirements 3, 4,
6 and 7 were all developed against it before the real stack was involved, and every one of the
behaviour tree's three defects was confirmed fixed against it (§11.4).

It is the only tooling kept in the repository, at [`tools/fake_nav2.py`](../tools/fake_nav2.py),
and that decision was made the hard way — the scratchpad version was lost once and rewritten.
Its two later switches exist because the real stack *cannot* produce the conditions they create:
`FAIL_GOALS` makes a specific attempt fail on demand, and `ACCEPT_DELAY` holds open a cancel
window that is a few milliseconds wide on a real Nav2.

**Commits.** 40, each one change with the reasoning in the body — including the ones that
record a wrong turn, because `d065d7e`'s explanation of why `set_initial_pose` cannot work is
worth more than the two lines it deleted.

---

## 9. What I would do next

- ~~**The behaviour-tree bonus.**~~ **Done** — §11. It landed as predicted:
  `RetryUntilSuccessful` and `Timeout` replaced the per-leg retry and the leg timeout, and
  keeping the state machine alongside under `mission:=fsm|bt` made the comparison the
  deliverable. What was *not* predicted is that reimplementing verified behaviour would
  introduce three defects, one of them serious (§11.4).
- **Share the plumbing between the two engines** — but only now that both are trusted. About
  500 lines are duplicated, which was the right call while the tree was unproven and is the
  wrong one to leave standing. A common `JobDesk` holding the service, the job registry and the
  feedback timer would leave each engine as nothing but its mission logic, which is what the
  comparison is actually about.
- **Raise the tick rate, or measure what it should be.** 10 Hz costs 0.4 s per delivery (§11.3)
  and was chosen by intuition. The right number is a measurement, not a guess.
- **Move `courier.launch.py` into `acadbot_bringup`**, where every other one-command launch
  lives (§2.2). Both blockers are now gone: `amcl.yaml` was deleted so one `params_file` serves
  localization and Nav2 alike, and the ordering was checked — `navigation.launch.py` starts
  localization immediately and delays Nav2 behind `nav2_delay`, the same shape this project
  composes by hand. The move is `sim` + `IncludeLaunchDescription(navigation.launch.py)` +
  seeder + courier + RViz, plus an `exec_depend` on `acadbot_courier`. It is not done because
  the package currently ships runnable on its own, which is a preference rather than a
  constraint — and because it touches the one-command bringup requirement 9 rests on, so it
  needs a full re-verification run.
- **A queue.** "One job at a time" is a real limitation, honestly stated. A queue means
  deciding what cancel means for a job that has not started.
- **Bounded job history.** `jobs_` grows for the life of the process so replayed ids can be
  refused. Fine at demo scale, not in a building.
- **The corridor.** Out of scope here, but the fix is known: raise `BaseObstacle.scale`, or add
  an intermediate waypoint at the corridor mouth so the robot approaches the tip head-on.

---

## 10. Presentation appendix

### 10.1 Slide sequence

1. **The product.** *"Take this from reception to lab_bench."* One sentence, one photo of the map.
2. **Two primitives, two jobs.** Booking is a transaction → service. Delivery is a journey →
   action. Show the two `.srv`/`.action` files side by side.
3. **Why the goal carries only a job id.** Poses frozen at booking; a config edit cannot move
   an accepted job's target.
4. **The state machine.** Two axes; the transition diagram; `CancelReason` and why a bool
   cannot express it.
5. **Honesty.** `abort()` never `succeed()`; a rejected Nav2 goal is a failed leg, not a
   skipped waypoint — with the `patrol_commander` contrast.
6. **What is configurable.** `courier.yaml` on one slide.
7. **What broke: seeding AMCL.** Eight attempts, three wrong verifications. The best slide in
   the deck.
8. **What broke: the corner.** `(3.25, 0.90)`, 0.175 m from a wall, 0.20 m radius.
   `BaseObstacle.scale: 0.02` vs `PathAlign.scale: 32.0`.
9. **Results.** The nine-requirement table from §0.
10. **Bonus: the same mission as a behaviour tree.** `courier.xml` beside the four members and
    three callbacks it replaces; the identical demo commands; +0.4 s, which is the tick rate.
    And the three bugs the review found — one of which left the robot driving after the client
    was told CANCELED (§11.4).
11. **What I would change.** The queue; the sequencing mistake in Phase 2; sharing the plumbing
    between the two engines once both are trusted.

### 10.2 Live demo, in order

```bash
ros2 launch acadbot_courier courier.launch.py         # ~20 s to Nav2 active

# 1. a rejection
ros2 service call /request_delivery acadbot_courier_msgs/srv/RequestDelivery \
  "{pickup: 'kitchen', dropoff: 'storage'}"

# 2. a delivery that succeeds  (~50 s)
ros2 service call /request_delivery acadbot_courier_msgs/srv/RequestDelivery \
  "{pickup: 'charging_dock', dropoff: 'storage'}"
ros2 action send_goal /execute_delivery \
  acadbot_courier_msgs/action/ExecuteDelivery "{job_id: 'job_0002'}" --feedback

# 3. a cancel — Ctrl-C the send_goal mid-drive; expect CANCELED and a stopped robot

# 4. an honest failure
#    reception -> lab_bench, or drop an obstacle in the corridor during (2)

# 5. the bonus: the same demo, run by the behaviour tree
#    relaunch with mission:=bt, then repeat (2) verbatim -- the commands do not change
ros2 launch acadbot_courier courier.launch.py mission:=bt
```

Keep the feedback stream visible; keep RViz showing the location markers with the current
target highlighted.

Step 5 is the point of the bonus and it needs saying out loud, because *nothing visibly
happens*: the service, the action, the feedback fields and the RViz view are identical, and the
only difference on screen is the node name in the log prefix. Show
[`courier.xml`](../ros2_ws/src/acadbot_courier/behavior_trees/courier.xml) beside it — 30 lines
that replace the retry counter, the retry timer, the leg timeout and the leg sequencing — and
then the four decorators are the whole argument (§11.1).

### 10.3 Questions to expect

**Why a service and an action, rather than two services or two actions?**
§2.1. Booking has no duration and nothing to report; delivery has both, plus it must be
cancellable.

**What happens if I send the same job id twice?**
Rejected — `it is already SUCCEEDED`. Finished jobs are kept precisely so that check can be
made; deleting them on completion would let a replayed goal deliver twice.

**What if Nav2 is not running when I book?**
The service refuses with *"navigation is not available"*. Checked, not waited on, so the reply
stays immediate — that is requirement 2's "cannot be served right now".

**Why does your node not need a mutex?**
Nothing blocks, so a single-threaded executor serializes every callback. Deliberate: the
alternative is a multi-threaded executor and a mutex on a dozen members.

**One of your routes fails. Why did you not fix it?**
Because fixing it means retuning DWB's critics, and the brief says the courier does not write a
planner or a controller. The robot drives its own footprint into a wall at `(3.25, 0.90)`
because obstacle proximity is weighted at 0.02 against path-following at 32.0. The route is
kept as the honest-failure demonstration — and the courier's behaviour there is exactly right.

**What would you do differently?**
Sequence Phase 2 later — the early code never needed real coordinates, and validating them
first turned into a Nav2 investigation. And write the verification before the fix: the AMCL
seeder was correct on the first try, but three successive checks reported success on a broken
system.

**You built the mission twice. Which one would you ship?**
The state machine, for this mission — see §11.5. The tree wins as soon as the mission grows
priorities and preconditions, and it is the right answer at Nav2's scale. At two legs it is a
demonstration of a technique rather than a better courier, and it says so.

---

## 11. Bonus: the same mission as a behaviour tree

### 11.1 What was built, and what was deliberately not

A second executable, `courier_bt_server`, running the delivery from
[`courier.xml`](../ros2_ws/src/acadbot_courier/behavior_trees/courier.xml) instead of the
hand-written leg loop. `mission:=fsm|bt` selects one at launch; both claim the same service and
action names, so **the client commands are identical** for either.

Three constraints were set before writing anything:

1. **`courier_server.cpp` is not touched.** It is verified and it is the demo fallback. A
   comparison in which one side was disturbed to make room for the other proves nothing.
2. **Duplicate the plumbing; do not refactor to share it.** The service handler, job registry,
   feedback timer and marker publisher were copied — about 500 lines. Extracting a common base
   class would have been better code and a worse decision: it puts working, verified behaviour
   at risk to accommodate a bonus.
3. **`nav2_params.yaml` stays untouched**, as everywhere else in this project.

So the tree did **not** make the project smaller. Total lines went *up*:

| | state machine | behaviour tree |
|---|---|---|
| mission engine | 751 lines C++ | 872 lines C++ + 74 XML |
| of which plumbing duplicated from the other | — | ~500 |

What shrank is the mission logic specifically. Four pieces of hand-written state have no
counterpart in the tree at all:

| gone from the tree | replaced by |
|---|---|
| `attempt_`, `max_retries_`, `retry_timer_`, `on_retry_elapsed()` | `RetryUntilSuccessful` |
| `leg_timeout_timer_`, `on_leg_timeout()`, `CancelReason::LEG_TIMEOUT` | `Timeout` |
| `leg_`, and the pickup→dropoff transition inside `on_nav_result` | `Sequence` |
| "pickup failed, so do not attempt the dropoff" | `Sequence`, for free |

Concretely: three dedicated callbacks totalling **63 lines** (`on_leg_timeout`,
`on_retry_elapsed`, `leg_failed`), four members and two `create_wall_timer` calls, replaced by
four XML decorators that nobody wrote and that every Nav2 installation in the world exercises
daily. Retry count and leg timeout become an edit to a data file rather than a rebuild.

### 11.2 Equivalence and cancellation, measured

Both engines were run against [`tools/fake_nav2.py`](../tools/fake_nav2.py) with
`FAIL_GOALS=1,3` — each leg aborts once, then succeeds — which is the shape that exposes
per-leg bookkeeping.

```
state machine                          behaviour tree
pickup  leg, attempt 1 of 3            leg: pickup,  attempt 1
pickup  leg, attempt 2 of 3            leg: pickup,  attempt 2
dropoff leg, attempt 1 of 3            leg: dropoff, attempt 1
dropoff leg, attempt 2 of 3            leg: dropoff, attempt 2
attempts_used: 4                       attempts_used: 4
status: SUCCEEDED                      status: SUCCEEDED
```

Identical, including the distinction between per-leg `attempt` and cross-leg `attempts_used`.
`FAIL_GOALS=1,2,3` exhausts a leg and both report `success: false`, `failed_leg: pickup`, with
the dropoff never attempted.

**Cancellation** was verified in both of its interesting cases. Cancelling mid-drive:

```
342.1938  BT cancel requested
342.1938  GoToLocation: halted/cancelled
342.1953  cancelled during the pickup leg     ← reported to the client, 1.5 ms
342.2502  fake_nav2: goal 2: canceled         ← the drive actually stopped
```

And cancelling in the window before Nav2 has acknowledged the goal — forced open with
`ACCEPT_DELAY=3`, because on the real stack it is a few milliseconds wide:

```
399.882  GoToLocation: halted/cancelled                      ← no goal handle yet
401.990  goal acknowledged after the halt -- cancelling it
401.999  fake_nav2: goal 1: canceled                         ← drive stopped
```

**One divergence that must not be read as a speed-up.** The state machine's measured 17 ms
cancel (§4.4) waited for Nav2's CANCELED *result* before reporting. A halted tree has no result
callback left to finish on, so the BT reports CANCELED while the stop is still in flight — hence
1.5 ms. The goal is verifiably cancelled either way, as the traces above show, but **the two
numbers measure different things** and comparing them would be dishonest.

### 11.3 Timing: what a tick rate costs

Identical two-leg deliveries, no failures, 1.0 s of driving per leg, wall-clock from goal sent
to result received. The first run of the session was discarded as a cold start (4434 ms).

| engine | runs | mean | spread |
|---|---|---|---|
| `courier_server` (FSM) | 3 | **2915 ms** | 2851–2973 |
| `courier_bt_server` (BT) | 4 | **3360 ms** | 3153–3662 |

**+445 ms**, and the explanation is arithmetic rather than architectural. A tree only changes
state when it is ticked, and this one ticks at 10 Hz. A delivery has roughly four transitions
(start pickup, pickup done, start dropoff, dropoff done), each of which can wait up to one full
tick period: 4 × 100 ms = up to 400 ms. Measured 445 ms.

So this is a **tuning** number, not a design cost. Ticking at 50 Hz would recover most of it and
spend CPU instead. Worth knowing, and worth not over-reading: 0.4 s on a 50 s delivery is 0.9%.

One further cost is structural rather than measured. The `Delay → AlwaysFailure` construction
that gives the tree its inter-attempt pause also runs after the *final* attempt, so an exhausted
leg waits one extra `retry_delay` before being declared failed. The state machine checks its
counter first and reports immediately. Removing it means a scripted precondition inside the tree,
which buys three seconds at the price of the readability that justifies the tree in the first
place — so it was left in, and written down.

### 11.4 Three bugs the second implementation introduced

All three found by review rather than by running it — the delivery worked before any of them was
fixed. All three were things the state machine already got right, which is the useful part: they
are not tree bugs, they are *"reimplemented working behaviour and lost a detail"* bugs.

**1 — the per-leg attempt counter never reset.** `courier_server.cpp` does `attempt_ = 0` at the
leg transition; the tree version incremented forever, so feedback reported *attempt 4, 5, 6* on
the dropoff. There was a second-order mistake underneath it: the leg was overwritten *before* the
increment, so the code could not have detected the transition even if it had tried.

Severity: cosmetic — the delivery completes correctly. Which is exactly why it survived a working
end-to-end run, and why `FAIL_GOALS=1,3` exists.

**2 — a cancel arriving before Nav2 acknowledged the goal was silently dropped.** `onHalted`
cancelled only `if (nav_handle_)`, and in the window between `async_send_goal` and the
goal-response callback there is no handle. Nothing was cancelled; the response then arrived and
was stored into an already-halted node.

Severity: **serious.** The tree stops, the client is told CANCELED, and the robot keeps driving to
the pickup. The state machine handles this explicitly, with a comment saying so — this is a
regression introduced by reimplementation, not an oversight in a new design. It is also
essentially untestable on the real stack, where the window is a few milliseconds wide, which is
why `ACCEPT_DELAY` was added to the harness. The trace in §11.2 is that test.

**3 — Nav2 callbacks bound to an object with the wrong lifetime.** The callbacks captured `this`,
and `this` is a *tree leaf*. A leaf dies with its tree, which happens when the next job calls
`createTreeFromFile`. A result still in flight from a cancelled goal would then write into freed
memory. Fixed with a `weak_ptr` liveness token; verified by inspection only, since triggering it
needs a new job booked inside the few milliseconds while a cancelled goal's result is in flight.

This one generalises furthest: **moving logic into a tree changes object lifetimes.** State that
used to live as long as the process now lives as long as a tree, and every callback bound to it
silently inherits that shorter life. Nothing in the compiler or the tree library warns about it.

### 11.5 What the tree is actually worth

**For this mission, the state machine is the better implementation**, and the tree is a
demonstration of a technique. Two legs, one retry rule and one timeout do not need a tree; every
transition in the state machine is a readable line with a place to put a breakpoint, and the
tree's control flow is a property of tick traversal over a data file.

**The tree is unambiguously right one scale up.** Add "if the battery is low, dock first",
"if the corridor is blocked, wait and re-plan", "on three failures, ask a human" — and the state
machine becomes a combinatorial mess of flags while the tree grows a subtree and a priority. That
is exactly why Nav2's own navigator is a behaviour tree: recovery escalation is configuration
there, and it is `#ifdef`-free.

**The honest summary is that it restructured the easiest part of the project.** The three genuinely
hard problems here were the async discipline (§7), seeding AMCL under simulated time (§5.1) and a
controller that cuts corners (§5.3). A behaviour tree helps with none of them — and the async
discipline it *does* touch, it makes no easier: `StatefulActionNode` exists precisely because a
tick must not block, which is the same rule, restated.

What it did prove, and what makes it worth the bonus: **the mission logic and the interface are
genuinely separable.** Two engines, one service, one action, one config block, identical observed
behaviour on identical inputs. That the swap is invisible to the client is the strongest evidence
in this report that the layering was right.
