#!/usr/bin/env python3
# Copyright 2026 Hiba Tarabay
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""
Operator CLI for the AcadBot courier pipeline.

Requests a delivery over /request_delivery and, if accepted, drives
/execute_delivery to completion while streaming feedback -- a purpose-built
replacement for `ros2 service call` + `ros2 action send_goal --feedback`
that does not crash on Ctrl+C mid-feedback-print (see DEMO.md).
"""

import argparse
import signal
import sys

from acadbot_courier_interfaces.action import ExecuteDelivery
from acadbot_courier_interfaces.srv import RequestDelivery
from action_msgs.msg import GoalStatus
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


def parse_args():
    parser = argparse.ArgumentParser(description='AcadBot courier operator CLI')
    parser.add_argument('--pickup', required=True, help='pickup location name')
    parser.add_argument('--dropoff', required=True, help='dropoff location name')
    return parser.parse_args()


def format_feedback(fb):
    if fb.leg == 'queued':
        return f'[queued] position {fb.queue_position} in line'
    return (
        f"[{fb.leg}] heading to '{fb.target}': "
        f'{fb.distance_left:.2f} m left (attempt {fb.attempt})'
    )


class CourierCli(Node):
    def __init__(self, pickup, dropoff):
        super().__init__('courier_cli')
        self.pickup = pickup
        self.dropoff = dropoff
        self.goal_handle = None
        # Set by the SIGINT handler; drained on the executor thread inside
        # the spin loop below, never touched from inside the OS signal
        # handler itself -- see install_sigint_handler() for why.
        self.cancel_requested = False

    def request_delivery(self):
        client = self.create_client(RequestDelivery, 'request_delivery')
        self.get_logger().info('Waiting for /request_delivery service...')
        client.wait_for_service()

        req = RequestDelivery.Request()
        req.pickup = self.pickup
        req.dropoff = self.dropoff
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def run_delivery(self, job_id):
        action_client = ActionClient(self, ExecuteDelivery, 'execute_delivery')
        self.get_logger().info('Waiting for /execute_delivery action server...')
        action_client.wait_for_server()

        goal = ExecuteDelivery.Goal()
        goal.job_id = job_id
        goal.pickup = self.pickup
        goal.dropoff = self.dropoff

        send_goal_future = action_client.send_goal_async(
            goal, feedback_callback=self.on_feedback
        )
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            print('Goal was rejected by the action server.')
            return None

        result_future = self.goal_handle.get_result_async()

        self.install_sigint_handler()
        while rclpy.ok() and not result_future.done():
            if self.cancel_requested:
                self.cancel_requested = False
                print('\nCtrl+C received -- sending cancel request...')
                self.goal_handle.cancel_goal_async()
            rclpy.spin_once(self, timeout_sec=0.1)
        self.restore_default_sigint_handler()

        return result_future.result()

    def on_feedback(self, feedback_msg):
        print(format_feedback(feedback_msg.feedback))

    def install_sigint_handler(self):
        # The default KeyboardInterrupt-on-SIGINT behaviour can land in the
        # middle of a print() call (see feedback_callback above), and
        # Python's own reentrancy guard on stdout then raises
        # "reentrant call inside <_io.BufferedWriter name='stdout'>" from
        # *inside* the handler -- before the handler ever gets to send a
        # cancel. That's the exact bug DEMO.md documents for
        # `ros2 action send_goal --feedback`. To avoid it, the handler here
        # does no I/O and touches no rclpy API itself; it only flips a flag
        # that the main spin loop above checks and acts on between spins,
        # i.e. never while a print is in flight.
        self._previous_sigint_handler = signal.signal(signal.SIGINT, self._on_sigint)

    def restore_default_sigint_handler(self):
        signal.signal(signal.SIGINT, self._previous_sigint_handler)

    def _on_sigint(self, signum, frame):
        self.cancel_requested = True


def main():
    args = parse_args()
    rclpy.init()
    node = CourierCli(args.pickup, args.dropoff)

    exit_code = 1
    try:
        response = node.request_delivery()

        if not response.accepted:
            print(f'Request rejected: {response.reason}')
            exit_code = 1
        else:
            print(f'Request accepted: job_id={response.job_id}')
            result = node.run_delivery(response.job_id)

            if result is None:
                exit_code = 1
            else:
                status = result.status
                res = result.result
                canceled = status == GoalStatus.STATUS_CANCELED
                print(
                    f'Result: success={res.success} '
                    f"failed_leg='{res.failed_leg}' message='{res.message}'"
                    + (' (goal was canceled)' if canceled else '')
                )
                exit_code = 0 if (res.success and not canceled) else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == '__main__':
    main()
