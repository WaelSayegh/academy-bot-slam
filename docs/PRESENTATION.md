# AcadBot Courier — presentation draft

Slide-by-slide outline for the final-project presentation. **Source for a `python-pptx`
generator, not a rendered deck.**

## How to read this

Each slide gives:

| field | meaning |
|---|---|
| `TYPE` | `COVER` / `SECTION` (black) · `CONTENT` (white) · `VIDEO` (white, one big frame) |
| `KICKER` | the small-caps label above the title — same vocabulary as the instructor's Session 4 deck (`NAV2`, `COSTMAPS`, `PLAN`, `CONTROL`, `RECOVERY`, `ORCHESTRATION`, `COMMAND`, `DEBUG`) |
| `TITLE` | the headline |
| `LEAD` | one sentence under the title |
| `BLOCKS` | the 3–6 rounded boxes the course decks use, or a table |
| `VISUAL` | diagram, screenshot or video to place |
| `SAY` | speaker notes — roughly what comes out of your mouth, timed |
| `TAG` | `KEEP` (10-min spine) · `ADD` (instructor asked for it) · `CUT-FIRST` (drop when short) |

### The 10-minute budget, honestly

`KEEP` is **18** slides: 1–4, 6–8, 10, 15, 16, 18–23, 27, 28. Do the arithmetic before you trust it:

| | |
|---|---|
| four videos | ~2:00 of screen time |
| two section dividers + cover + closing | ~0:30 |
| eleven talking slides | **~7:30 → 41 s each** |

That works, with no slack. The `SAY` notes are written to that length — several are ~45 s and
will need trimming, not reading. If you overrun in rehearsal, drop **slide 7 (Dijkstra)** and
**slide 9 (the critics)** first and fold their one-line takeaways into slides 6 and 8: *"the planner is Dijkstra
over that cost, and the controller scores 400 candidate trajectories against it."* That buys 80 s
and loses least.

Every `ADD` slide you keep on top costs a further ~40 s — at 10 minutes you can afford roughly
one, not six. They are drafted because you asked for them; they are not free.

Footer on every content slide, matching the course deck: `inmind.ai academy · SLAM Track`,
with `NN / total` top-right.

---

# PART 1 — THE ASK

## Slide 1 · COVER · KEEP

```
TYPE    COVER (black)
TITLE   AcadBot Courier
LEAD    A delivery robot that tells the truth
SUB     Final project · Robotics Academy SLAM Track · Elias Antoun
VISUAL  RViz screenshot: the map, the four location arrows, one highlighted
```

**SAY** (15 s) — "Someone asks for a delivery between two named places. The robot accepts the
job, drives to the pickup, then the dropoff, reports what it's doing the whole way, stops when
told, and — the part I care about most — tells the truth when it fails."

---

## Slide 2 · CONTENT · KEEP

```
KICKER  THE TASK
TITLE   Nine requirements, and one sentence that shaped everything
LEAD    Paraphrased from the brief.
BLOCKS
  1  Service      accept a job, reply immediately, hand back an id
  2  Reject       bad requests refused with a human-readable reason
  3  Action       execution runs as an action, not a service
  4  Feedback     leg, destination, distance — at least 1 Hz
  5  Nav2         drive with it; do not write a planner or a controller
  6  Cancel       cancel actually stops the robot
  7  Retry        N times, then fail *naming the leg*
  8  YAML         everything configurable, nothing compiled in
  9  One command  the whole stack, no mouse
QUOTE (large, centred, accent colour)
  "A robot that reports success for a delivery it did not make is the one
   unforgivable bug in this project."
```

**SAY** (45 s) — Read the nine fast, then slow down on the quote. "Every design decision in
this project traces back to that line. It's why failure is a first-class outcome and not an
error path."

---

## Slide 3 · CONTENT · KEEP

```
KICKER  DESIGN
TITLE   The central decision: a service AND an action
LEAD    The brief tests one idea above all — do you know which ROS 2 primitive fits which job?
BLOCKS
  Booking is a transaction    Either accepted or refused. No measurable duration.
                              Nothing to report while it happens.  →  SERVICE
  Executing is a journey      Minutes long. Continuously interesting.
                              Must be interruptible.               →  ACTION
  The goal carries an id      string job_id — that is the whole goal
  Poses are frozen at booking Re-resolving at execution would let a YAML edit move
                              the target of a job already promised
VISUAL  Two-column diagram:
        RequestDelivery.srv  |  ExecuteDelivery.action
        pickup, dropoff      |  job_id
        ---                  |  ---
        accepted, job_id,    |  success, job_id, message,
        reason               |  failed_leg, attempts_used
                             |  ---
                             |  leg, target_location, distance_remaining,
                             |  attempt, nav2_recoveries, state
```

**SAY** (60 s) — "Booking either works or it doesn't, instantly — that's a service. Driving
takes minutes, has something to say the whole time, and has to be cancellable. Those three
properties are the definition of an action. The goal carries only the job id, because the
coordinates were resolved and frozen when the job was booked — otherwise editing the YAML
mid-delivery would move a destination I'd already promised."

---

## Slide 4 · CONTENT · KEEP

```
KICKER  COMMAND
TITLE   Where my code attaches
LEAD    Nav2 is a team of servers. My project touches it at exactly one point.
VISUAL  Horizontal chain, in the instructor's slide-5 style:
        [ client ] → [ courier ] → [ bt_navigator ] → [ planner ] → [ controller ]
                                                                  → [ collision monitor ] → [ wheels ]
        with the courier box highlighted and labelled "MINE"
        annotate the courier→bt_navigator arrow: navigate_to_pose
BLOCKS
  One action client   rclcpp_action::create_client<NavigateToPose>   (courier_server.cpp:41)
  Everything below    inherited, unmodified — nav2_params.yaml untouched on this branch
  Everything above    the mission: which goals, in what order, what on failure
```

**SAY** (45 s) — "This is the whole footprint. One action client. Everything below that arrow is
Nav2's and I changed none of it — `git log main..HEAD` on the navigation package returns
nothing. Everything above it is mine: which goals to send, in what order, and what to do when
one fails."

---

# PART 2 — THE STACK I INHERITED

*(instructor asked to see these — keep as many as time allows)*

## Slide 5 · SECTION · ADD

```
TYPE    SECTION (black)
TITLE   What Nav2 already does
LEAD    Four slides, so the demo makes sense.
```

---

## Slide 6 · CONTENT · KEEP

```
KICKER  COSTMAPS
TITLE   Costmaps: why you cannot plan on the map
LEAD    A robot is not a point. AcadBot is robot_radius 0.20 — 40 cm across.
BLOCKS
  Global costmap    whole map, map frame, for the planner
  Local costmap     rolling 4 × 4 m window, odom frame, for the controller
  Static layer      the Session-2 map
  Obstacle layer    live LiDAR; cleared by raytracing
  Inflation layer   a cost halo to inflation_radius 0.45, cost_scaling_factor 3.0
  The point         inflation lets the planner treat the robot as a point again
VISUAL  Two panels of the same corridor:
        (a) 30 cm gap — every cell free, a 40 cm robot cannot enter
        (b) 80 cm corridor — it fits, but nothing biases the path to the centre
```

**SAY** (45 s) — "Two different failures. A 30 cm gap reads free in every cell, and a point
planner routes straight through it. And even when the gap *does* fit — 80 cm for a 40 cm robot —
nothing makes the planner prefer the middle; hugging the wall costs the same and is shorter.
Inflation fixes both by paying for proximity, so the planner can go back to treating the robot
as a dot."

**NOTE** — `inflation_radius: 0.45` is also the clearance rule for my own locations. That's how
two of my four were caught as badly placed. Mention if asked.

---

## Slide 7 · CONTENT · KEEP

```
KICKER  PLAN
TITLE   The global planner: Dijkstra
LEAD    planner_server answers one question — is there a collision-free path, and what is it?
BLOCKS
  NavfnPlanner        propagates a potential field from the goal, then walks downhill
  use_astar: false    → Dijkstra. Uniform-cost expansion, optimal.
  A* if you want it   same optimal path, fewer cells expanded — matters when maps get big
  tolerance: 0.5      accept a pose half a metre out rather than failing
  allow_unknown: true may plan through never-observed cells
  Cost ≠ distance     the sum of costmap cell costs, so inflation defines "shortest"
VISUAL  side-by-side flood diagrams — Dijkstra expanding evenly vs A* leaning toward the goal
        (instructor's slide 9 has this; mirror it)
```

**SAY** (40 s) — "Dijkstra, because `use_astar` is false. It floods outward from the goal and
extracts the path by gradient descent. On a 160 × 120 map a full flood is under 20,000 cells and
finishes inside the 20 Hz budget, so A* would buy milliseconds. The thing worth remembering is
that cost isn't distance — it's accumulated costmap cost, so the inflation radius is what
decides which route is 'cheapest'."

---

## Slide 8 · CONTENT · KEEP

```
KICKER  CONTROL
TITLE   The controller: DWB, and the number behind my one failure
LEAD    Not a search. 400 candidate futures, scored, 20 times a second.
BLOCKS
  Sample     vx_samples 20 × vtheta_samples 20 inside 0.26 m/s, 1.0 rad/s
  Simulate   roll each forward sim_time 1.7 s against the local costmap
  Score      weighted critics; publish the winner on /cmd_vel
  Arrived    xy_goal_tolerance 0.20 m — my definition of "delivered"
TABLE  critic weights
  PathDist / PathAlign / RotateToGoal   32.0
  GoalDist / GoalAlign                  24.0
  BaseObstacle                           0.02   ← 1600× less than path adherence
VISUAL  fan of candidate trajectories from the robot: red rejected, grey scored, green winner
```

**SAY** (45 s) — "DWB samples 400 velocity pairs every cycle, simulates each 1.7 seconds ahead,
scores them and sends the best. And look at the weights: path adherence is weighted **1600
times** obstacle proximity. That single ratio is why one of my routes doesn't complete, and I'll
come back to it. It's inherited configuration — `git blame` puts every one of those lines in the
course's first commit."

---

## Slide 9 · CONTENT · ADD

```
KICKER  CONTROL
TITLE   The critics — seven numbers that are the robot's driving style
LEAD    Every one of the 400 candidate trajectories gets a score. The critics are the terms
        in that score; the weights are the personality.

FORMULA (centred, large)
        score(trajectory)  =  Σ  scale_i × critic_i(trajectory)
        lowest total wins

TABLE   the seven, as configured — all of them inherited, none of them mine
  critic         scale   what it measures
  PathDist        32.0   how far the trajectory ends from the global path
  PathAlign       32.0   how well its heading lines up with the path
  RotateToGoal    32.0   turning to the final yaw once inside position tolerance
  GoalDist        24.0   how far it ends from the goal point
  GoalAlign       24.0   how well its heading lines up with the goal
  Oscillation        —   no scale set; penalises flip-flopping direction
  BaseObstacle    0.02   the costmap cost of the cells it drives through

CALLOUT (accent box, large)
        32.0  ÷  0.02  =  1600 ×
        Staying on the line matters 1600 times more than not being near a wall.
```

**SAY** (45 s) — "DWB doesn't have rules, it has weights. Each candidate trajectory is scored by
seven critics and the cheapest one gets published. Look at the bottom two rows: following the
planner's line is weighted 32, and how close you are to an obstacle is weighted 0.02. That's a
factor of 1600. This robot will scrape a wall to stay on its line, and it does — that ratio is
the direct cause of the one route I can't complete."

**NOTE — the honest framing.** These are the *inherited* values. `git blame` puts every one of
them in the course's first commit, `3b8d932`. I did not tune them, and the brief says I don't
write a controller. If asked *"so fix it"*: raising `BaseObstacle.scale` makes it hug less and
wander more, and `nav2_params.yaml` is shared with Sessions 3 and 4 — a fix for my one route
changes their demos too.

**BACKUP — the numbers that interact with the critics**, if a question goes there:

| | |
|---|---|
| `vx_samples` × `vtheta_samples` | 20 × 20 = **400** candidates per cycle |
| `sim_time` | **1.7 s** of forward simulation each |
| `controller_frequency` | **20 Hz** — so 8,000 trajectories scored per second |
| `max_vel_x` / `max_speed_xy` | **0.15 m/s** (locally lowered from the course's 0.26) |
| `max_vel_theta` | 1.0 rad/s |
| `xy_goal_tolerance` / `yaw_goal_tolerance` | 0.20 m / 0.25 rad — my definition of "delivered" |
| `progress_checker` | must move 0.5 m every 10 s, or `FollowPath` fails |

**One thing worth saying out loud if you show this slide:** a critic is *not* a constraint. A
trajectory that clips a wall is not rejected — it is scored, badly, and then compared. With
`BaseObstacle` at 0.02 the penalty for scraping is smaller than the reward for staying on the
line, so it wins. The only hard stop in the whole chain is `collision_monitor`, which sits below
the controller and owns `/cmd_vel`.

---

## Slide 10 · CONTENT · KEEP

```
KICKER  RECOVERY
TITLE   Recoveries: Nav2 gets unstuck before I ever hear about it
LEAD    Escalation is tree shape, not an algorithm. And it is not what the plugin list says.
BLOCKS
  Loaded (behavior_server)   spin · backup · drive_on_heading · wait · assisted_teleop
  Actually invoked           the navigator's default BT decides, and it uses a RoundRobin:
                             1 clear both costmaps → 2 Spin 1.57 → 3 Wait 5 s → 4 BackUp 0.30 m
  Never invoked              drive_on_heading, assisted_teleop — loaded, unreachable
  Wrapped in                 RecoveryNode number_of_retries="6", inside ONE goal
  I invoke none of them      I observe the count: nav2_recoveries in my feedback
VISUAL  the two retry layers, nested:
        my leg retry (3 attempts, 3 s apart)
          └── one navigate_to_pose goal
                └── Nav2: 6 recovery rounds → then ABORTED
```

**SAY** (50 s) — "This is the slide I'd want a question on. `behavior_plugins` lists five
behaviours, but that list only says what's *available* — the navigator's behaviour tree decides
which fire, and its default RoundRobin uses three of them plus a costmap clear. Two are loaded
and never reachable. And note the nesting: Nav2 exhausts six recovery rounds inside a single
goal, *then* aborts, and only then does one of my three attempts count as failed. Two retry
layers, easy to confuse."

**NOTE** — the escalation order is read from Nav2's default BT XML, not observed in my logs. Say
"according to the default tree" if pressed.

---

# PART 3 — WHAT I BUILT

## Slide 11 · SECTION · ADD

```
TYPE    SECTION (black)
TITLE   The mission layer
LEAD    Two engines, one interface.
```

---

## Slide 12 · CONTENT · ADD

```
KICKER  STRUCTURE
TITLE   Two packages, three executables, 28 files
LEAD    The contract is separate from the implementation so a client can depend on one without the other.
VISUAL  the annotated tree from REPORT.md §2.2 — trimmed to two levels for legibility
BLOCKS
  acadbot_courier_msgs    RequestDelivery.srv · ExecuteDelivery.action — the contract
  acadbot_courier         two mission engines, the seeder, config, launch, rviz
  tools/                  fake_nav2.py — not built, not installed
  Shared by both engines  location_table.cpp · location_markers.cpp compile into both,
                          so they cannot disagree about what a location IS
```

**SAY** (35 s) — "Interfaces are their own package so anything that wants to book a delivery
depends on the contract, not on Nav2 and BehaviorTree and an RViz config. And the two mission
engines share exactly two files — the location table and the markers — so they can disagree
about how to run a mission but not about where the rooms are."

---

## Slide 13 · CONTENT · ADD

```
KICKER  CONFIG
TITLE   Requirement 8: no coordinate in any .cpp
LEAD    A location exists by being written down. There is no second list of names.
CODE (left)
  /**:
    ros__parameters:
      max_retries: 2
      retry_delay: 3.0
      leg_timeout: 120.0
      locations:
        reception:     [0.60, 4.20,  0.00]
        lab_bench:     [4.50, 4.20,  3.14]
        storage:       [5.50, 0.60,  0.00]
        charging_dock: [0.50, 2.00, -1.57]
CODE (right)
  const auto & overrides = node.get_node_parameters_interface()
                               ->get_parameter_overrides();
  for (const auto & kv : overrides) {
    if (key.rfind("locations.", 0) != 0) { continue; }
    // the name is discovered, never declared
  }
BLOCKS
  Discovery       reads parameter OVERRIDES, so names need not be known at compile time
  Refuses early   not-a-list, wrong length, or no locations → dies at startup, not at delivery
  One /**: block  both engines read it, so they cannot drift to different coordinates
```

**SAY** (40 s) — "ROS 2 parameters have no map type, and you can't ask for a parameter whose name
you don't know — which is exactly what requirement 8 demands. So I read the overrides and
discover the names. Adding a room is an edit, not a rebuild. And a typo dies at startup naming
the fault, rather than showing up an hour later as 'unknown location'."

---

## Slide 14 · CONTENT · ADD

```
KICKER  DEBUG
TITLE   Requirement 9: one command, no mouse
LEAD    The hardest requirement in the project, and it is one line of the brief.
BLOCKS
  The problem      AMCL must be told where it starts. Normally a human clicks 2D Pose Estimate.
  set_initial_pose looks right, cannot work: AMCL applies it ~0.4 s after start, before its
                   first /clock, so under sim time the pose is stamped t=0
  The answer       a seeder node: publish /initialpose, CHECK it took, repeat
  The check        require the map→odom timestamp to ADVANCE
                   — existence passes on a stale cache; age is defeated by frozen sim time
  Measured         Nav2 reaches active ~20 s after launch, unattended
CODE
  const bool advancing = have_stamp_ && stamp > last_stamp_;
```

**SAY** (40 s) — "This took eight attempts. The interesting part isn't the fix, it's that I got
the fix right early and the *verification* wrong three times — every wrong version reported
success on a broken system. Checking that the transform exists passes on a stale cache. Checking
its age fails because sim time is frozen at zero. Only checking that it's *advancing* tells you
the localizer is alive."

---

## Slide 15 · CONTENT · KEEP

```
KICKER  ORCHESTRATION
TITLE   Engine 1 — the state machine
LEAD    Two axes, not one enum, mirroring exactly what the feedback publishes.
CODE
  leg   ∈ { PICKUP, DROPOFF }
  phase ∈ { NAVIGATING, RETRY_WAIT, CANCELING }
BLOCKS
  Nothing blocks        no spin_until_future_complete, no worker thread, no waiting on a future
  So no mutex           one executor serialises every callback by construction
  Why that matters      this node is an action SERVER holding an action CLIENT —
                        block it and the result that would unblock it can never arrive
  CancelReason          a CANCELED result from Nav2 is ambiguous: client cancel, my leg
                        timeout, or Nav2 giving up. Recorded when issued, not guessed later.
VISUAL  docs/figures/courier_fsm.png — the per-leg phase loop (NAVIGATING /
        RETRY_WAIT / CANCELING boxed as "one leg, runs for PICKUP then DROPOFF"),
        the attempts-left decision, and the three terminal states. Regenerate:
        dot -Tpng -Gdpi=200 docs/figures/courier_fsm.dot -o docs/figures/courier_fsm.png
```

**SAY** (50 s) — "The one thing I'd point at is why nothing blocks. This node is an action server
that holds an action client — it's waiting on Nav2 while something else waits on it. If I block
the thread to wait for a result, the callback that delivers that result can never run. Same
thread. So every step is a callback or a timer, and because a single-threaded executor then
serialises everything, not one member needs a mutex. That's a design constraint, not luck."

---

## Slide 16 · CONTENT · KEEP

```
KICKER  ORCHESTRATION
TITLE   Engine 2 — the same mission as a behaviour tree
LEAD    mission:=bt. Same service, same action, same commands. 30 lines of XML.
CODE
  <Sequence name="delivery">
    <RetryUntilSuccessful num_attempts="{max_attempts}">
      <Fallback>
        <Timeout msec="{leg_timeout_msec}">
          <GoToLocation pose="{pickup_pose}" location_name="{pickup_name}" leg="pickup"/>
        </Timeout>
        <Delay delay_msec="{retry_delay_msec}"><AlwaysFailure/></Delay>
      </Fallback>
    </RetryUntilSuccessful>
    <!-- leg 2: identical but for the target -->
  </Sequence>
TABLE  what the tree DELETED
  attempt_, max_retries_, retry_timer_, on_retry_elapsed()   →  RetryUntilSuccessful
  leg_timeout_timer_, on_leg_timeout(), CancelReason::LEG_TIMEOUT → Timeout
  leg_, and the pickup→dropoff transition                    →  Sequence
  "pickup failed, so skip the dropoff"                       →  Sequence, for free
BLOCKS
  Only custom node   GoToLocation — a StatefulActionNode, because a tick must not block
  Ticked at 10 Hz    tickOnce from a timer. Never tickWhileRunning.
```

**SAY** (55 s) — "Same mission, second implementation, and the state machine is untouched — a
launch argument picks one. Four pieces of hand-written state vanish and become decorators that
every Nav2 installation in the world exercises daily. What doesn't change is the async rule:
`StatefulActionNode` exists precisely because a tick can't block, which is the same constraint
as the last slide wearing a different hat."

---

## Slide 17 · CONTENT · ADD

```
KICKER  ORCHESTRATION
TITLE   FSM vs BT — measured, not argued
LEAD    Both engines, identical inputs, against a stand-in Nav2 that aborts on command.
TABLE
  identical outputs      pickup 1, pickup 2, dropoff 1, dropoff 2 · attempts_used 4 · SUCCEEDED
  mean wall clock        FSM 2915 ms (n=3)   BT 3360 ms (n=4)
  difference             +445 ms  ≈ 4 state transitions × a 100 ms tick period
  code size              FSM 751 lines   BT 872 lines + 74 XML  (≈500 duplicated by choice)
BLOCKS
  The tree is not smaller   duplicating the plumbing was deliberate: the FSM was verified
                            and is the fallback. A refactor risks what already works.
  What shrank               the mission logic: 3 callbacks, 4 members, 2 timers → 4 decorators
  The cost is the tick      not the tree. 0.4 s on a 50 s delivery is 0.9%.
  Verdict                   at two legs, the FSM is easier to follow. At a dozen behaviours
                            with priorities, the tree wins outright — which is why Nav2 uses one.
```

**SAY** (45 s) — "I measured rather than argued. Same inputs, identical outputs, and the tree is
0.4 seconds slower per delivery — which turns out to be arithmetic, not architecture: four state
transitions against a 10 Hz tick. Honest summary: at two legs the state machine is the better
implementation and the tree is a demonstration of a technique. One scale up, the tree wins."

---

# PART 4 — IT WORKS (VIDEOS)

> **Recording notes.** All four pre-recorded. Suggested 25–40 s each, trimmed hard; put the
> terminal and RViz side by side so the feedback stream and the robot are visible together.
> Caption each video with what to watch for — nobody spots it unprompted.
>
> **Videos 1–3 are scenarios already verified.** Video 4 has **never been run** — see its slide.

## Slide 18 · SECTION · KEEP

```
TYPE    SECTION (black)
TITLE   Four things it does
LEAD    One succeeds, one is refused, one is cancelled, one goes wrong.
```

---

## Slide 19 · VIDEO · KEEP

```
KICKER  DEMO 1 / 4
TITLE   A delivery, end to end
CAPTION Watch: distance_remaining counting down, leg flipping pickup → dropoff, 0 recoveries.
RECORD
  ros2 launch acadbot_courier courier.launch.py
  ros2 service call /request_delivery acadbot_courier_msgs/srv/RequestDelivery \
    "{pickup: charging_dock, dropoff: storage}"
  ros2 action send_goal /execute_delivery \
    acadbot_courier_msgs/action/ExecuteDelivery "{job_id: job_0001}" --feedback
RESULT overlay (freeze the last frame)
  success: true
  message: delivered from 'charging_dock' to 'storage'
  failed_leg: ''
  attempts_used: 2                 # ~56 s, 0 recoveries
```

**SAY** (35 s) — "Booked, then run. The feedback names the leg, the destination and the distance
left, on my own timer rather than Nav2's — so if Nav2 stalls you see a frozen number instead of
silence. This route crosses the corridor and completes with zero recoveries, which is why it's
the one I show."

---

## Slide 20 · VIDEO · KEEP

```
KICKER  DEMO 2 / 4
TITLE   A request that is refused
CAPTION Watch: it refuses instantly, says WHY, and lists what I could have asked for.
RECORD
  ros2 service call /request_delivery acadbot_courier_msgs/srv/RequestDelivery \
    "{pickup: 'kitchen', dropoff: 'storage'}"
RESULT overlay
  accepted: false
  reason: "unknown pickup 'kitchen'; known locations are:
           charging_dock, lab_bench, reception, storage"
BLOCKS  the five refusals
  unknown pickup · unknown dropoff · pickup == dropoff
  a job already running · Nav2 not available
```

**SAY** (30 s) — "Five distinct refusals, each naming what could have been said instead. That
last one matters: if Nav2 isn't up I refuse immediately rather than accepting and hanging —
checked, not waited on, so the reply stays instant."

---

## Slide 21 · VIDEO · KEEP

```
KICKER  DEMO 3 / 4
TITLE   Cancel means stop
CAPTION Watch: Ctrl-C mid-drive → status CANCELED, and the robot STOPS.
RECORD
  # start the delivery from demo 1, then Ctrl-C the send_goal while it is driving
RESULT overlay
  Goal finished with status: CANCELED
  success: false
  failed_leg: ''                   # nothing failed — it was cancelled
BLOCKS
  Measured              CANCELED reported in 17 ms while driving, 4.5 ms mid-retry
  failed_leg is empty   a cancel is not a failure and must not be reported as one
  The subtle case       a cancel can arrive before Nav2 has acknowledged the goal — there is
                        nothing to cancel yet. Remembered, and cancelled when the handle lands.
                        Without that the mission stops and the robot keeps driving.
```

**SAY** (40 s) — "Cancel is a request, not a guarantee — you wait for the callback. Two things
worth noticing: `failed_leg` is empty, because a cancel isn't a failure and reporting it as one
would be a lie. And there's a window of a few milliseconds where the cancel arrives before Nav2
has even acknowledged the goal, so there's nothing to cancel — if you don't remember it, your
mission stops and the robot drives on."

---

## Slide 22 · VIDEO · KEEP

```
KICKER  DEMO 4 / 4
TITLE   Something gets in the way
CAPTION Watch: nav2_recoveries climbing, then the delivery continuing anyway.
RECORD
  # during a running delivery, insert a box in the Gazebo GUI in front of the robot
  # keep the feedback terminal visible so nav2_recoveries increments on screen
RESULT overlay — one of:
  recovered:  nav2_recoveries: N, then  success: true
  or honest:  success: false, failed_leg: 'dropoff',
              message: 'the dropoff leg failed after 3 attempts: Nav2 aborted the goal'
BLOCKS
  Nav2's job first   clear costmap → spin → wait → back up, up to 6 rounds, inside ONE goal
  Then mine          Nav2 ABORTS → that attempt failed → retry, up to 3 per leg
  Either way         the outcome is reported truthfully and names the leg
```

**SAY** (40 s) — "This is the requirement-7 slide. Nav2 tries to get unstuck on its own — you'll
see the recovery counter climb in my feedback. If it wins, the delivery continues. If it
exhausts its recoveries it aborts, my attempt counts as failed, and I retry. And if all three
attempts fail, I say so and name the leg. The only unacceptable outcome is claiming success."

> ⚠️ **THIS SCENARIO HAS NOT BEEN RUN.** It's the lecture's Exercise 4.2 and it's on my not-done
> list. Both overlay outcomes above are predictions from the code, not observations. **Test it
> before filming** — and if the courier does something unexpected, that's better to find now than
> on stage. Ask me to run it.

---

# PART 5 — RESULTS AND HONESTY

## Slide 23 · CONTENT · KEEP

```
KICKER  RESULTS
TITLE   Nine for nine, on the real stack
LEAD    Gazebo, map_server, AMCL, the full Nav2 server set, and the courier.
TABLE
  1  service, immediate, id      accepted=True, job_id='job_0001', no delay
  2  reject with a reason        5 distinct refusals
  3  action for execution        both legs under one goal
  4  feedback a human follows    6 fields at 2 Hz, on my own clock
  5  drive with Nav2            navigate_to_pose client; no planner, no controller
  6  cancel means stop          CANCELED in 17 ms; robot stationary
  7  honest failure             a retry SAVED one delivery; another named failed_leg
  8  everything in YAML         4 locations + 5 limits; no coordinate in any .cpp
  9  one command                Nav2 active ~20 s after launch, no click
BLOCKS
  Plus the bonus   the same mission as a behaviour tree, over an identical interface
```

**SAY** (35 s) — "All nine on the real stack, not against a mock. My favourite line is 7: on one
run a retry *saved* a delivery that would otherwise have failed — which is the whole point of
retrying rather than reporting a failure the first time Nav2 gives up."

---

## Slide 24 · CONTENT · CUT-FIRST

```
KICKER  DEBUG
TITLE   The route that does not work, and why I left it
LEAD    reception → lab_bench: the pickup succeeds, the dropoff does not.
BLOCKS
  What happens   coming from the north-west the robot runs down the divider's west face and
                 clips the tip, ending at map (3.25, 0.90)
  The geometry   0.175 m from a wall face, with a 0.20 m footprint radius —
                 its own footprint is inside the wall
  The symptom    "GridBased plugin failed to plan" — the planner cannot escape its own
                 start pose. 15 recoveries, then ABORTED.
  The cause      BaseObstacle.scale 0.02 vs PathAlign.scale 32.0
  Why not fixed  retuning DWB means editing a file Sessions 3 and 4 depend on, and the brief
                 says I do not write a controller. Slowing to 0.20 m/s stopped it wedging
                 hard enough to break the planner, but did not get it round the corner.
  What I kept    the courier's behaviour here is CORRECT: three attempts, then
                 failed_leg: dropoff. It refuses to claim a delivery it did not make.
```

**SAY** (40 s) — "I want to show the thing that doesn't work, because how it fails is the
requirement. The robot ends up with its own footprint inside a wall, so the planner can't plan
out of its start pose. The cause is inherited configuration weighting path-following 1600× over
obstacle proximity. I could fix it by retuning the controller — and that's explicitly out of
scope, and would change the demo for every other session. So I kept it as the honest-failure
case."

---

## Slide 25 · CONTENT · CUT-FIRST

```
KICKER  DEBUG
TITLE   Two bugs worth more than the features
LEAD    Both found by review, not by running it. Both were in the SECOND implementation.
BLOCKS
  Cancel dropped   onHalted only cancelled if the goal handle existed. Land a cancel in the
                   gap between send and acknowledge and nothing was cancelled: the tree
                   stopped, the client was told CANCELED, and the robot kept driving.
  Lifetime         Nav2 callbacks captured `this` — and `this` was a tree LEAF, which dies
                   when the next job builds a new tree. A result in flight would write into
                   freed memory.
  The lesson       moving logic into a tree changes object lifetimes. Nothing warns you.
  How they surfaced  by asking, of every piece of state in the state machine,
                     "where did this go in the tree?" Two of three fell straight out.
  How they were proved  ACCEPT_DELAY=3 in my stand-in Nav2 holds open a window that is a
                        few milliseconds wide on the real stack
```

**SAY** (40 s) — "The delivery worked before either of these was fixed, which is the point. A
second implementation of working behaviour is exactly where 'it ran successfully' is worthless
evidence — the happy path is already known to be reachable. What found them was reading the two
side by side and asking where each piece of state went."

---

## Slide 26 · CONTENT · CUT-FIRST

```
KICKER  ENGINEERING
TITLE   The tooling that paid for itself
LEAD    tools/fake_nav2.py — a navigate_to_pose that does exactly what it is told.
BLOCKS
  Why           the courier's logic has nothing to do with driving. Against the real stack a
                test costs a minute of startup and whatever the planner felt like that run.
  What          a stand-in action server. A two-leg delivery in ~3 s, reproducibly.
  FAIL_GOALS    abort the goals you name → forces one retry per leg
  ACCEPT_DELAY  stall before acknowledging → opens the cancel race deliberately
  Both engines  run against it unchanged. That is how their equivalence was checked.
  The trap      its own execute() blocks, so it needs a reentrant callback group and a
                multithreaded executor — otherwise cancels are never serviced and it looks
                exactly like a courier that ignores cancellation
```

**SAY** (30 s) — "Three of the switches exist because the real stack *can't* produce those
conditions on demand. And the harness bit me twice in ways that looked like bugs in the code
under test — worth writing down."

---

## Slide 27 · CONTENT · KEEP

```
KICKER  CLOSING
TITLE   What I would do next
BLOCKS
  A queue              one job at a time is a real limit, honestly stated. A queue means
                       deciding what cancel means for a job that has not started.
  Share the plumbing   ~500 lines are duplicated between the engines. Right while the tree
                       was unproven; wrong to leave standing now that it is not.
  Measure the tick     10 Hz costs 0.4 s per delivery and was chosen by intuition.
  Bounded job history  jobs are remembered forever so replayed ids are refused. Fine at demo
                       scale, not in a building.
  The corridor         out of scope here, but the fix is known: raise BaseObstacle.scale, or
                       add a waypoint at the corridor mouth so the approach is head-on.
```

**SAY** (30 s) — Pick two, don't read all five.

---

## Slide 28 · SECTION · KEEP

```
TYPE    SECTION (black)
TITLE   Thank you
LEAD    Questions.
SUB     github.com/<repo> · branch final_project/Elias_Antoun
```

---

# APPENDIX — questions to expect

*Not slides. Prep only.*

| question | the answer |
|---|---|
| Why a service **and** an action? | Booking has no duration and nothing to report; delivery has both and must be cancellable. §2.1 |
| Why does the goal carry only an id? | The poses were frozen at booking. Re-resolving would let a YAML edit move a promised target. |
| Why no mutex? | Nothing blocks, so one executor serialises every callback. Deliberate, and it holds in both engines. |
| Same job id twice? | Rejected — *"it is already SUCCEEDED"*. Finished jobs are kept so that check can be made. |
| Nav2 not running when I book? | Refused with a reason. Checked, not waited on, so the reply stays immediate. |
| Which planner? Which algorithm? | `NavfnPlanner`, `use_astar: false` → **Dijkstra**. Potential field from the goal, gradient descent out. |
| Did you tune Nav2? | No. `git log main..HEAD` on `acadbot_navigation` is empty. `git blame` puts the critics and the planner in the instructor's first commit. |
| Why does a route fail? | DWB weights path adherence 1600× obstacle proximity; the footprint ends inside a wall. Out of scope to fix. |
| Which recovery behaviours run? | Per the navigator's default tree: costmap clear → Spin 1.57 → Wait 5 s → BackUp 0.30. `drive_on_heading` and `assisted_teleop` are loaded and never invoked. |
| FSM or BT — which is better? | For two legs, the FSM. The tree wins as soon as there are priorities and preconditions. Neither touched the hard parts. |
| Why duplicate 500 lines? | The FSM was verified and is the fallback. Refactoring it to accommodate a bonus risks what works. |
| What was hardest? | Seeding AMCL without a mouse. Eight attempts — and the fix was right on attempt seven; the *verification* was wrong three times. |

---

# BUILD NOTES for the generator

- Course style: black `COVER`/`SECTION`, white `CONTENT`, blue→purple gradient accents,
  kicker in small caps top-left, `NN / total` top-right, footer `inmind.ai academy · SLAM Track`.
- Slide count as drafted: **28**. `KEEP` is **18** — see the budget note at the top; that is a
  full 10 minutes with no slack, and slides 7 and 9 are the first to fold if you overrun.
- `ADD` slides: 5, 9, 11, 12, 13, 14, 17 — the Nav2 background and config material the instructor
  asked to see. Each costs ~40 s.
- `CUT-FIRST`: 24, 25, 26.
- Diagrams needed: the frames chain (slide 4), the two-corridor costmap panel (6), the
  Dijkstra-vs-A* flood (7), the DWB trajectory fan (8), the nested retry layers (9), the FSM
  state diagram (14). Mirror the instructor's Session 4 visual language where it exists.
- Videos: four files, 25–40 s each, terminal + RViz side by side, captions burned in or on the
  slide.
