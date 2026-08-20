# AcadBot Courier — Final Project

A requester books a delivery between two named locations through a ROS 2
service; the robot drives both legs with Nav2 through an action, streaming
progress, retrying honestly on failure, and stopping cleanly on request.
Built on the Session 2–4 map, AMCL, and Nav2 stack from the rest of this
repo. Two packages: `acadbot_courier_interfaces` (the `.srv`/`.action`
definitions) and `acadbot_courier` (the node, this package).

---

## 1. Launch everything (one command)

```bash
ros2 launch acadbot_courier courier.launch.py
```

Brings up simulation + AMCL + Nav2 + RViz + the courier node together.
**Set the initial pose in RViz (2D Pose Estimate) before booking anything**
— nothing downstream works until AMCL has a valid pose, and a booking made
before that will be rejected as "Nav2 is not ready yet."

| Argument | Default | Why you'd change it |
|---|---|---|
| `headless:=true` | `false` | Gazebo server only — no GUI, no GPU. |
| `rviz:=false` | `true` | Skip RViz2 on a machine with no display. |
| `nav2_delay:=<sec>` | `12.0` | How long to wait for localization before starting Nav2. |
| `courier_params_file:=<path>` | `config/courier.yaml` | Point at a different courier parameters file. |

## 2. Book and run a delivery

```bash
ros2 service call /request_delivery acadbot_courier_interfaces/srv/RequestDelivery "{pickup: 'reception', dropoff: 'lab_bench'}"
```
Take the `job_id` from the response, then:
```bash
ros2 action send_goal /deliver acadbot_courier_interfaces/action/Deliver "{job_id: 'D-000N'}" --feedback
```
Job ids increment (`D-0001`, `D-0002`, ...) and are one-shot — a finished
job's id can't be reused, book a fresh one each time.

## 3. The four demo scenarios

**Success** — book and send as above; watch feedback stream both legs and
end in `success: true`, `failed_leg: ''`.

**Rejection** — book an unknown location:
```bash
ros2 service call /request_delivery acadbot_courier_interfaces/srv/RequestDelivery "{pickup: 'reception', dropoff: 'nowhere'}"
```
Response names the bad location and lists every valid one.

**Cancel** — send a goal, then Ctrl-C it mid-drive:
```bash
ros2 action send_goal /deliver acadbot_courier_interfaces/action/Deliver "{job_id: 'D-000N'}"
```
Omit `--feedback` when you plan to Ctrl-C — with feedback streaming, a
Ctrl-C landing mid-write can crash the CLI itself before the cancel is even
sent (a Python stdout-buffering bug, not ours); without it, there's nothing
for the interrupt to collide with. Confirm the cancel actually reached
Nav2 in the stack's own terminal, not just the CLI: look for
`[bt_navigator] Goal canceled`.

**Blockage** — spawn a wall that seals the only passage between the two
halves of the room, forcing real Nav2 retries and, eventually, an honest
failure:
```bash
ros2 run ros_gz_sim create -file install/acadbot_gazebo/share/acadbot_gazebo/worlds/blockage_box.sdf -name blockage_box -x 0.5 -y -2.0 -z 0.5
```
Book and send a delivery crossing from a west location (`reception`,
`lab_bench`) to an east one (`charging_dock`, `store_room`, or vice versa).
Watch `attempt` climb in the feedback and, once `max_attempts` is
exhausted, the result comes back `success: false` with a real Nav2
`error_msg` where one was available. Remove the wall afterward — every
other test drives through that same passage:
```bash
gz service -s /world/academy_world/remove --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --timeout 1000 --req 'name: "blockage_box" type: MODEL'
```

## 4. Named locations

Surveyed in RViz with Publish Point, sanity-checked reachable with a manual
2D Goal Pose. Coordinates are in the map frame; see `config/courier.yaml`.

| Location | x | y |
|---|---|---|
| `reception` | 1.020 | -0.297 |
| `lab_bench` | 1.932 | 1.377 |
| `charging_dock` | 5.020 | 2.498 |
| `store_room` | 6.053 | 1.439 |

`reception` and `lab_bench` sit west of the interior divider; `charging_dock`
and `store_room` sit east of it — a delivery between one of each has to
cross the corridor the blockage demo seals off.

## 5. Configuration — `config/courier.yaml`

| Parameter | Meaning |
|---|---|
| `location_names`, `locations.<name>.{x,y,yaw}` | The location book. Node refuses to start if `location_names` is empty. |
| `booking_timeout_sec` | A `BOOKED` job never picked up for execution expires after this long. |
| `max_attempts` | Retries per leg before the job fails. |
| `leg_timeout_sec` | Give up on a single leg's Nav2 goal after this long. |
