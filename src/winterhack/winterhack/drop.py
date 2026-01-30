#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Drop action server - open gripper using servo_controller.
"""

import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse

from servo_controller_msgs.msg import ServoPosition, ServosPosition
from winterhack_interfaces.action import Drop


class _AutoGoalHandle:
    def __init__(self, node: Node):
        self._node = node

    def publish_feedback(self, _feedback):
        pass

    def abort(self):
        self._node.get_logger().warn("Auto-start drop aborted")

    def succeed(self):
        self._node.get_logger().info("Auto-start drop completed")


class DropServer(Node):
    """Action server for dropping objects (open gripper only)."""

    def __init__(self):
        super().__init__("drop_server")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("auto_start", False),
                ("servo.topic", "/servo_controller"),
                ("gripper.open_pulse", 200),
                ("gripper.duration", 0.6),
                ("gripper.settle_s", 0.5),
            ],
        )

        self.servo_topic = str(self.get_parameter("servo.topic").value)
        self.gripper_open_pulse = int(self.get_parameter("gripper.open_pulse").value)
        self.gripper_duration = float(self.get_parameter("gripper.duration").value)
        self.gripper_settle_s = float(self.get_parameter("gripper.settle_s").value)

        self.servo_pub = self.create_publisher(ServosPosition, self.servo_topic, 10)

        self._auto_running = False
        self._auto_start_timer = None

        self.action_server = ActionServer(
            self,
            Drop,
            "/drop",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.get_logger().info("Drop action server ready")

        self.auto_start = bool(self.get_parameter("auto_start").value)
        if self.auto_start:
            self._auto_start_timer = self.create_timer(0.1, self._auto_start_once)

    def goal_callback(self, goal_request):
        if self._auto_running:
            self.get_logger().warn("Rejecting external goal while auto_start is running")
            return GoalResponse.REJECT
        self.get_logger().info("Received drop goal")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Drop goal cancelled")
        return CancelResponse.ACCEPT

    def _auto_start_once(self):
        if self._auto_start_timer is not None:
            self._auto_start_timer.cancel()
            self.destroy_timer(self._auto_start_timer)
            self._auto_start_timer = None
        if self._auto_running:
            return
        self._auto_running = True
        thread = threading.Thread(target=self._run_auto_sequence, daemon=True)
        thread.start()

    def _run_auto_sequence(self):
        goal_handle = _AutoGoalHandle(self)
        try:
            self.execute_callback(goal_handle)
        finally:
            self._auto_running = False

    def _send_gripper(self, pulse: int, duration: float) -> None:
        msg = ServosPosition()
        msg.duration = float(duration)
        msg.position_unit = "pulse"
        servo_msg = ServoPosition()
        servo_msg.id = 10
        servo_msg.position = float(max(0, min(1000, int(pulse))))
        msg.position.append(servo_msg)
        self.servo_pub.publish(msg)

    def execute_callback(self, goal_handle):
        feedback = Drop.Feedback()
        result = Drop.Result()

        feedback.stage = "Opening gripper"
        feedback.progress = 0.5
        goal_handle.publish_feedback(feedback)

        self._send_gripper(self.gripper_open_pulse, self.gripper_duration)
        time.sleep(self.gripper_duration + self.gripper_settle_s)

        feedback.stage = "completed"
        feedback.progress = 1.0
        goal_handle.publish_feedback(feedback)

        result.success = True
        result.message = "Drop complete"
        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    server = DropServer()
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
