#!/usr/bin/env python3
"""A stand-in navigate_to_pose server, for testing the courier without Gazebo.

The courier's own logic -- booking, the leg sequence, retries, timeouts,
cancellation, what it reports when it fails -- has nothing to do with driving.
Testing it against the real stack costs a minute of startup per run and gives
you whatever the planner felt like doing that time. This answers instantly and
does exactly what you tell it to, so a full delivery takes seconds and a
failure is reproducible.

It is not a simulator. Nothing moves; there is no map, no costmap and no TF.
It exists to answer "does the courier do the right thing when Nav2 says X".

    ros2 run acadbot_courier courier_bt_server \\
      --ros-args --params-file <install>/share/acadbot_courier/config/courier.yaml \\
      -p use_sim_time:=false &
    python3 tools/fake_nav2.py &

    ros2 service call /request_delivery acadbot_courier_msgs/srv/RequestDelivery \\
      "{pickup: charging_dock, dropoff: storage}"
    ros2 action send_goal /execute_delivery \\
      acadbot_courier_msgs/action/ExecuteDelivery "{job_id: job_0001}" --feedback

Both mission engines work with it: swap courier_bt_server for courier_server.
Pass -p retry_delay:=1.0 to stop the retry pauses dominating the run.

Behaviour is set by environment variables, so a test is one line:

    FAIL_GOALS=1,3      abort the 1st and 3rd goals, succeed on the rest.
                        "1,3" forces exactly one retry on each leg, which is
                        how you see whether the attempt counter restarts when
                        the mission moves from the pickup to the dropoff.
                        Empty (the default) succeeds every time.
    DRIVE_TIME=1.5      seconds of pretend driving per goal.
    ACCEPT_DELAY=3      seconds to stall before acknowledging a goal.
                        Holds open the window between "goal sent" and "goal
                        acknowledged", which is where a cancel has nothing to
                        cancel yet. A courier that ignores that window stops
                        its mission and leaves the robot driving.
"""
import os
import time

import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


def _goal_numbers(spec):
    return {int(n) for n in spec.replace(' ', '').split(',') if n}


class FakeNav2(Node):
    def __init__(self):
        super().__init__('fake_nav2')

        self.fail_goals = _goal_numbers(os.environ.get('FAIL_GOALS', ''))
        self.drive_time = float(os.environ.get('DRIVE_TIME', '1.5'))
        self.accept_delay = float(os.environ.get('ACCEPT_DELAY', '0'))
        self.goals_seen = 0

        # Reentrant, and spun by a MultiThreadedExecutor in main(): execute()
        # blocks while it pretends to drive, and a single-threaded executor
        # sitting inside it cannot service the cancel request that is meant to
        # interrupt it -- the goal would run to completion and the cancel would
        # be handled afterwards, which looks exactly like a courier that
        # ignores cancellation.
        self._server = ActionServer(
            self, NavigateToPose, 'navigate_to_pose',
            execute_callback=self.execute,
            goal_callback=self.on_goal,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup())

        self.get_logger().info(
            f'fake navigate_to_pose up: drive_time={self.drive_time}s, '
            f'accept_delay={self.accept_delay}s, '
            f'aborting goals {sorted(self.fail_goals) or "none"}')

    def on_goal(self, _):
        if self.accept_delay > 0:
            self.get_logger().info(
                f'stalling {self.accept_delay}s before acknowledging')
            time.sleep(self.accept_delay)
        return GoalResponse.ACCEPT

    def execute(self, goal_handle):
        self.goals_seen += 1
        n = self.goals_seen
        will_abort = n in self.fail_goals
        self.get_logger().info(
            f'goal {n}: will {"ABORT" if will_abort else "SUCCEED"}')

        # Feedback shaped like Nav2's, so the courier's own feedback -- which
        # republishes distance_remaining on its own timer -- has something
        # real to carry.
        steps = max(1, int(self.drive_time / 0.25))
        for i in range(steps):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info(f'goal {n}: canceled')
                return NavigateToPose.Result()

            feedback = NavigateToPose.Feedback()
            feedback.distance_remaining = float(steps - i) * 0.5
            feedback.number_of_recoveries = 0
            goal_handle.publish_feedback(feedback)

            # sleep, not spin_once: this callback already runs inside the
            # executor, and re-entering it raises "Executor is already
            # spinning", which the action server then reports as an aborted
            # goal -- a harness bug that reads as a courier bug.
            time.sleep(0.25)

        if will_abort:
            goal_handle.abort()
        else:
            goal_handle.succeed()
        return NavigateToPose.Result()


def main():
    rclpy.init()
    node = FakeNav2()
    try:
        rclpy.spin(node, executor=MultiThreadedExecutor())
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
