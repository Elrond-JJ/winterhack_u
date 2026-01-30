#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# This code tests the joint execution via the direct hardware controllers launched
# by servo_controller.launch.py. The servo_controller node loads
# config/servo_controller.yaml, instantiates arm_controller and
# gripper_controller, and starts FollowJointTrajectory action servers at:
#   /arm_controller/follow_joint_trajectory
#   /gripper_controller/follow_joint_trajectory
#
# Launch the controller with real-robot:
#   ros2 launch servo_controller servo_controller.launch.py

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from time import sleep

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def main() -> None:
    rclpy.init()
    node = Node("arm_test_demo")

###########################################################################
# Configure arm controller and execute target joints sequence.
###########################################################################

    joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5"]
    targets = [
        [0.0, 0.83776, -2.03156, -1.19380, 0.0],
        [0.6981317, 0.83776, -2.03156, -1.19380, 0.0],
        [0.0, 0.83776, -2.03156, -1.19380, 0.0],
    ]

    arm_controller_client = ActionClient(
        node,
        FollowJointTrajectory,
        "/arm_controller/follow_joint_trajectory",
    )

    node.get_logger().info("Waiting for arm controller action server...")
    if not arm_controller_client.wait_for_server(timeout_sec=5.0):
        node.get_logger().error("Arm controller action server not available")
        node.destroy_node()
        rclpy.shutdown()
        return

    trajectory = JointTrajectory(joint_names=joint_names)
    point = JointTrajectoryPoint()
    point.time_from_start.sec = 2
    point.time_from_start.nanosec = 50_000_000
    trajectory.points = [point]
    controller_goal = FollowJointTrajectory.Goal(trajectory=trajectory)

    node.get_logger().info("arm controller is running")
    
    for target in targets:
        point.positions = target

        send_goal_future = arm_controller_client.send_goal_async(controller_goal)
        rclpy.spin_until_future_complete(node, send_goal_future, timeout_sec=5.0)
        controller_goal_handle = send_goal_future.result()
        if not controller_goal_handle.accepted:
            break

        result_future = controller_goal_handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future, timeout_sec=20.0)
        status = result_future.result().status
        if status != 4:
            break
        sleep(1.0)

###########################################################################
# Configure gripper controller and execute close/open sequence.
###########################################################################

    gripper_controller_client = ActionClient(
        node,
        FollowJointTrajectory,
        "/gripper_controller/follow_joint_trajectory",
    )
    node.get_logger().info("Waiting for gripper controller action server...")
    if not gripper_controller_client.wait_for_server(timeout_sec=20.0):
        node.get_logger().error("Gripper controller action server not available")
        node.destroy_node()
        rclpy.shutdown()
        return
    gripper_trajectory = JointTrajectory(joint_names=["gripper_base_joint"])
    gripper_point = JointTrajectoryPoint(positions=[0.0])
    gripper_point.time_from_start.sec = 2
    gripper_point.time_from_start.nanosec = 50_000_000
    gripper_trajectory.points = [gripper_point]
    gripper_goal = FollowJointTrajectory.Goal(trajectory=gripper_trajectory)

    node.get_logger().info("gripper controller is running")

    for gripper_position in (-1.638, 0.0):
        gripper_point.positions[0] = gripper_position

        send_goal_future = gripper_controller_client.send_goal_async(gripper_goal)
        rclpy.spin_until_future_complete(node, send_goal_future, timeout_sec=20.0)
        gripper_goal_handle = send_goal_future.result()
        if not gripper_goal_handle.accepted:
            break
        result_future = gripper_goal_handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future, timeout_sec=20.0)
        result = result_future.result()
        if result.status != 4:
            break
        sleep(1.0)

###########################################################################
# End of demo.
###########################################################################

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()