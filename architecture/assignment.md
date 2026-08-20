# Final Project — AcadBot Courier
### (assignment brief — backbone document, do not edit)

This file is a verbatim copy of the assignment as given. Every design decision
in `architecture.md` must be traceable back to a requirement in this file.
If a decision doesn't map to something here, it's scope creep — flag it.

**Individual work. Budget about 2-3 days of work.**
Robot: **AcadBot** · everything runs in the `acadbot:jazzy` container ·
ROS 2 Jazzy + Nav2.

---

## The idea

AcadBot becomes a **courier** inside the building it mapped in Session 2.

Someone asks for a delivery — *"take this from `reception` to `lab_bench`"*.
The robot accepts the job, drives to the pickup, then drives to the dropoff,
reports what it is doing while it moves, and tells the truth when something
goes wrong. If the requester changes their mind halfway, they can cancel and
the robot stops cleanly.

That is the whole product. You build the software that makes it real, on top of
the Nav2 stack you already have.

---

## What it must do

Nine requirements. Everything here is observable from a terminal — that is how
it will be assessed.

**1 · Accept jobs through a service.**
A request names a **pickup** and a **dropoff** by name (not by coordinates).
The reply comes back immediately and says whether the job was accepted, and if
it was, gives it an identifier the requester can refer to later.

**2 · Reject bad jobs, with a reason.**
An unknown location name, or a request that cannot be served right now, comes
back as a rejection carrying a human-readable reason. It must not crash, hang,
or silently accept.

**3 · Run jobs through an action.**
Executing a delivery is an **action**, not a service. It runs for minutes, it
streams feedback while it runs, and it can be cancelled.

**4 · Feedback that a human can follow.**
While a job runs, the action publishes progress: which leg it is on
(pickup or dropoff), which location it is heading to, and how far it still has
to go. At least once a second.

**5 · Drive with Nav2.**
Under the hood your code is a `navigate_to_pose` action client. You do not
write a planner or a controller.

**6 · Cancel means stop.**
Cancelling the delivery action must stop the robot promptly, cancel the Nav2
goal underneath it, and end the action as `CANCELED`. Not `SUCCEEDED`, not a
node that keeps driving.

**7 · Handle failure honestly.**
If Nav2 exhausts its recoveries and aborts a leg, the job must retry a
configurable number of times, then finish as a failure that names which leg
failed. A robot that reports success for a delivery it did not make is the one
unforgivable bug in this project.

**8 · Everything configurable is in YAML.**
Named locations and their poses, the retry count, timeouts, and any other limit
you invent are loaded from configuration at launch. No coordinates, no tuning
constants compiled into your source.

**9 · One command brings it up.**
A single launch brings up everything needed to demonstrate the system —
simulation, localization on your map, Nav2, and your own nodes.

**Language: C++** for the nodes that do the work.

---

## What you hand in

Three things, in this order.

### 1. A presentation, with slides

A short deck (PowerPoint or PDF) covering:

- the idea and how you shaped it,
- your interface design: what the service takes and returns, what the action's
  goal, feedback and result carry, and why you split them that way,
- your node and package layout, and why,
- what you configured and what you deliberately left fixed,
- what broke while you built it, and what you did about it.

### 2. A live run — in the room

You run it in front of us. Plan for these, in any order that suits your demo:

- a delivery that succeeds end to end,
- a rejected request (unknown location),
- a cancel while the robot is driving,
- a delivery that hits a blockage — drop an obstacle in Gazebo and let Nav2's
  recoveries do their work.

Have a terminal visible with your feedback stream. RViz on screen helps.

## Before you start

You need the map from Session 2 in place, and the localization stack from
Session 3 working:

```bash
git pull # main branch
git checkout -b "final_project/your_name"
cd /ros2_ws && colcon build --symlink-install && source install/setup.bash
ls src/acadbot_navigation/maps/          # academy_map.yaml + academy_map.pgm

# the stack your project sits on top of
ros2 launch acadbot_bringup autonomy.launch.py localization:=amcl
```

If your own map is poor, ask for the reference map — a bad map will cost you
hours and it is not what you are being graded on.

Read `patrol_commander.cpp` before you write anything. It already shows you how
to talk to Nav2 from C++; your job is to build something considerably more
capable around that idea.

---

## How it is judged

| | What we look for |
|---|---|
| **It runs** | The live demo works, on your machine, without a rescue |
| **Interfaces** | Service vs action used for the right jobs; sensible fields; feedback that means something |
| **Honesty** | Failures reported as failures; cancel actually cancels |
| **Configuration** | Locations and limits in YAML; nothing important hardcoded |
| **Structure** | The way you split packages, nodes and files makes sense and you can defend it |
| **The talk** | You can explain your own design choices, including the ones you would now change |

The design is yours. How many nodes, how many packages, how you name things,
how you lay out your files — all of that is part of what is being assessed.

---

## Bonus track — do the mission logic as a behavior tree

Take this only if the nine requirements above are already working.

Nav2's `bt_navigator` runs a **behavior tree** — a tree of
sequences and fallbacks that decides what happens when. Your courier's own
logic is the same shape: *go to pickup, then go to dropoff; if a leg fails,
retry; if retries run out, give up cleanly.*

Rebuild your mission logic as a behavior tree instead of hand-written control
flow, using **BehaviorTree.CPP** (already a Nav2 dependency, so it is in the
image). Load the tree from an XML file that you can edit without recompiling,
and write your own leaf nodes for the courier-specific steps.
