"""
rclpy action client, simplified.

Roberto Masocco <r.masocco@dotxautomation.com>

September 11, 2022
"""

# Copyright 2024 dotX Automation s.r.l.
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

import time
from typing import TypeVar

import rclpy
from rclpy.task import Future
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.node import Node

from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal

ActionType = TypeVar('ActionType')
GoalType = TypeVar('GoalType')
ResultType = TypeVar('ResultType')
FeedbackCallbackType = TypeVar('FeedbackCallbackType')


class Client():
    """
    Wraps an action client providing a simpler interface.
    Supports a/synchronous operation, and the back-end is managed internally by
    spinning the node when necessary.
    """

    def __init__(
            self,
            node: Node,
            type: ActionType,
            action_name: str,
            feedback_callback: FeedbackCallbackType = None,
            wait: bool = True,
            spin_period=0.1) -> None:
        """
        Creates a new Client.
        Can wait for the server to become active.

        :param node: Reference to the ROS 2 node to use.
        :param type: Action interface type.
        :param action_name: Name of the action to look for.
        :param feedback_callback: Feedback routine.
        :param wait: Indicates whether to wait for the server immediately.
        :param spin_period: Period of the internal spinning loop (seconds).
        """
        # Initialize internal attributes
        self._node = node
        self._feedback_callback = feedback_callback
        self._spin_period = spin_period

        # Create the ROS 2 action client
        self._client = ActionClient(self._node, type, action_name)

        # Wait for the server to come up
        while wait and not self._client.wait_for_server(timeout_sec=1.0):
            if not rclpy.ok():
                raise RuntimeError(
                    "Interrupted while waiting for action {}".format(self._client._action_name))
            self._node.get_logger().warn(
                "Action {} not available...".format(self._client._action_name))

        self._node.get_logger().info(
            "Initialized client for action {}".format(self._client._action_name))

    def send_goal(self, goal_msg: GoalType) -> Future:
        """
        Sends a new goal to the server.

        :param goal_msg Goal to send.
        :returns: GoalHandle future.
        """
        return self._client.send_goal_async(goal_msg, self._feedback_callback)

    def send_goal_sync(self, goal_msg: GoalType, timeout_sec: float = None) -> ClientGoalHandle:
        """
        Sends a new goal and waits for the response of the server.
        Wraps back-end operations while waiting for the Future to complete.
        A None or <= 0.0 timeout means no timeout.

        :param goal_msg: Goal to send.
        :param timeout_sec: Maximum time to wait for the goal to be accepted (seconds).
        :returns: Goal handle.
        """
        goal_future = self.send_goal(goal_msg)
        if timeout_sec is None or timeout_sec <= 0.0:
            rclpy.spin_until_future_complete(self._node, goal_future)
            return goal_future.result()
        else:
            if self._wait_spinning(goal_future, timeout_sec):
                return goal_future.result()
            else:
                return None

    def cancel(self, goal_handle: ClientGoalHandle) -> Future:
        """
        Cancels the specified goal.

        :param goal_handle Goal handle to use.
        :returns: Cancel future.
        """
        return goal_handle.cancel_goal_async()

    def cancel_sync(self, goal_handle: ClientGoalHandle, timeout_sec: float = None) -> CancelGoal.Response:
        """
        Cancels the specified goal.
        Wraps back-end operations while waiting for the Future to complete.
        A None or <= 0.0 timeout means no timeout.

        :param goal_handle: Goal handle to use.
        :param timeout_sec: Maximum time to wait for the cancellation result (seconds).
        :returns: Cancellation result, or None if the server didn't respond.
        """
        cancel_future = self.cancel(goal_handle)
        if timeout_sec is None or timeout_sec <= 0.0:
            rclpy.spin_until_future_complete(self._node, cancel_future)
            return cancel_future.result()
        else:
            if self._wait_spinning(cancel_future, timeout_sec):
                return cancel_future.result()
            else:
                return None

    def get_result(self, goal_handle: ClientGoalHandle) -> Future:
        """
        Asks the server for the result of the specified goal.

        :param goal_handle Goal handle to use.
        :returns: Result future.
        """
        return goal_handle.get_result_async()

    def get_result_sync(
            self,
            goal_handle: ClientGoalHandle,
            timeout_sec: float = None) -> ResultType:
        """
        Asks the server for the result of the specified goal.
        Wraps back-end operations while waiting for the Future to complete.
        A None or <= 0.0 timeout means no timeout.

        :param goal_handle: Goal handle to use.
        :param timeout_sec: Maximum time to wait for the result (seconds).
        :returns: Result, or None if the server didn't respond.
        """
        result_future = self.get_result(goal_handle)
        if timeout_sec is None or timeout_sec <= 0.0:
            rclpy.spin_until_future_complete(self._node, result_future)
            return result_future.result()
        else:
            if self._wait_spinning(result_future, timeout_sec):
                return result_future.result()
            else:
                return None

    def cancel_and_get_result_sync(
            self,
            goal_handle: ClientGoalHandle,
            timeout_sec: float = None) -> ResultType:
        """
        Cancels the specified goal and waits for the result.

        :param goal_handle: Goal handle to use.
        :param timeout_sec: Maximum time to wait for the result (seconds).
        :returns: Result, or None if the server didn't respond, errored out, or the cancellation request got rejected.
        """
        # First, cancel the goal
        cancel_result = self.cancel_sync(goal_handle)
        if cancel_result == None or \
            (cancel_result.return_code != CancelGoal.Response.ERROR_NONE and
             cancel_result.return_code != CancelGoal.Response.ERROR_GOAL_TERMINATED):
            return None

        # Then, get the result
        return self.get_result_sync(goal_handle, timeout_sec)

    def call(
            self,
            goal_msg: GoalType,
            cancel_on_timeout: bool = False,
            send_goal_timeout: float = None,
            get_result_timeout: float = None,
            cancel_timeout_sec: float = None) -> tuple:
        """
        Calls the action, returns only when it has been completed or canceled.
        Can cancel the action automatically if the timeout expires while waiting
        for the action result.
        All wait operations timeouts can be set.
        A None or <= 0.0 timeout means no timeout.

        :param goal_msg: Goal message to send.
        :param cancel_on_timeout: Indicates whether to cancel the goal if get_result_timeout expires.
        :param send_goal_timeout: Maximum time to wait for the goal to be accepted (seconds).
        :param get_result_timeout: Maximum time to wait for the result (seconds).
        :param cancel_timeout_sec: Maximum time to wait for cancellation result (seconds), set to 0 to not wait.
        :returns: Tuple with accepted flag, status code, and result.
        """
        # Send the new goal using the internal wrapper
        goal_handle = self.send_goal_sync(goal_msg, send_goal_timeout)
        if goal_handle is None:
            self._node.get_logger().error(
                "{}: server did not respond to goal request".format(self._client._action_name))
            return (False, GoalStatus.STATUS_UNKNOWN, None)
        if not goal_handle.accepted:
            # Yeah, it'll look the same to the upper level
            self._node.get_logger().error("{}: goal REJECTED".format(self._client._action_name))
            return (False, GoalStatus.STATUS_UNKNOWN, None)
        self._node.get_logger().info("{}: goal ACCEPTED".format(self._client._action_name))

        # Get the result, using the internal wrapper to deal with the back-end
        # The following can happen now:
        # - The goal is completed before the timeout expires, and we get the result
        # - The timeout expires, but we should not cancel the goal, simply return
        # - The timeout expires, and we should cancel the goal without waiting for the result
        # - The timeout expires, and we should cancel the goal and wait for the result
        # In the last case, the following can happen:
        # - The goal cancellation response arrives before the timeout expires
        # - The timeout expires before a cancellation response is received
        goal_result = self.get_result_sync(goal_handle, get_result_timeout)
        if goal_result is not None:
            # The goal was completed before the timeout expired
            self._node.get_logger().info("{}: goal COMPLETED".format(self._client._action_name))
            return (True, goal_result.status, goal_result.result)
        else:
            if not cancel_on_timeout:
                # The timeout expired, but we should not cancel the goal
                self._node.get_logger().error("{}: goal timed out".format(self._client._action_name))
                return (True, GoalStatus.STATUS_UNKNOWN, None)
            else:
                self._node.get_logger().warn(
                    "{}: goal timed out, CANCELING...".format(self._client._action_name))
                if cancel_timeout_sec is not None and cancel_timeout_sec == 0.0:
                    # We should cancel the goal without waiting for the result
                    self.cancel(goal_handle)
                    return (True, GoalStatus.STATUS_CANCELING, None)
                else:
                    # We should cancel the goal and wait for the result
                    cancel_result = self.cancel_sync(
                        goal_handle,
                        cancel_timeout_sec)
                    if cancel_result is not None:
                        # Goal cancellation response arrived before the timeout expired
                        self._node.get_logger().info(
                            "{}: goal cancellation response received".format(self._client._action_name))
                        if cancel_result.return_code == CancelGoal.Response.ERROR_NONE:
                            return (True, GoalStatus.STATUS_CANCELED, None)
                        else:
                            return (True, GoalStatus.STATUS_UNKNOWN, None)
                    else:
                        # Timeout expired before a cancellation response was received
                        self._node.get_logger.warn(
                            "{}: goal cancellation timed out".format(self._client._action_name))
                        return (True, GoalStatus.STATUS_CANCELING, None)

    def _wait_spinning(self, future: Future, timeout_sec: float) -> bool:
        """
        Wraps the busy-wait for a Future object, with a timeout.
        Inspired by the spin_until_* family.

        :param future: Future object to wait for.
        :param timeout_sec: Maximum time to wait for the Future (seconds).
        :returns: True if the Future got a value, False if the timeout expired.
        :throws: RuntimeError if the back-end is interrupted while waiting.
        """
        # We'll use the global executor and context for this, nothing fancy
        executor = rclpy.get_global_executor()
        executor.add_node(self._node)

        # Wait for the future, blockingly polling the back-end for work to do
        timeout_elapsed = False
        start = time.monotonic()
        end = start + timeout_sec
        try:
            while True:
                # Check that the back-end is still up
                if not executor._context.ok() or executor._is_shutdown:
                    raise RuntimeError("Interrupted while waiting for Future")

                # Block waiting for work to do/executing queued jobs
                executor.spin_once(self._spin_period)

                # Poll the Future
                if future.done():
                    break

                # Check the time
                now = time.monotonic()
                if now >= end:
                    timeout_elapsed = True
                    break
        finally:
            executor.remove_node(self._node)
        return not timeout_elapsed
