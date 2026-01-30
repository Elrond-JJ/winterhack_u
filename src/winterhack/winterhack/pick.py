#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Pick action server - orchestrates pick using kinematics + servo_controller.
"""

import json
import time
import threading
from collections import deque
from typing import Deque, List, Optional

import rclpy
from geometry_msgs.msg import PoseStamped, PointStamped
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.wait_for_message import wait_for_message
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from kinematics_msgs.srv import SetRobotPose
from servo_controller_msgs.msg import ServoPosition, ServosPosition
from winterhack_interfaces.action import Pick

import tf2_geometry_msgs  # noqa: F401  Needed for geometry_msgs TF conversions


class _AutoGoalHandle:
    def __init__(self, node: Node):
        self._node = node

    def publish_feedback(self, _feedback):
        pass

    def abort(self):
        self._node.get_logger().warn("Auto-start pick aborted")

    def succeed(self):
        self._node.get_logger().info("Auto-start pick completed")


class PickServer(Node):
    """Action server for object picking via kinematics + servo_controller."""

    def __init__(self):
        super().__init__("pick_server")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("auto_start", False),
                ("ik.service_name", "/kinematics/set_pose_target"),
                ("ik.timeout_s", 3.0),
                ("ik.pitch_near", 80.0),
                ("ik.pitch_far", 30.0),
                ("ik.pitch_z_threshold", 0.2),
                ("ik.pitch_range", [-180.0, 180.0]),
                ("ik.resolution", 1.0),
                ("servo.topic", "/servo_controller"),
                ("servo.arm_ids", [1, 2, 3, 4, 5]),
                ("servo.move_duration", 1.5),
                ("servo.move_settle_s", 0.2),
                ("servo.home_pulses", [500, 720, 100, 120, 500]),
                ("servo.home_duration", 1.0),
                ("gripper.open_pulse", 200),
                ("gripper.close_pulse", 600),
                ("gripper.duration", 0.6),
                ("gripper.settle_s", 0.5),
                ("grasp_offsets.descend", [-0.01, 0.0, -0.0125]),
                ("lift_z", 0.03),
                ("median_window", 3),
                ("color_priority", ["GREEN", "RED", "BLUE", "YELLOW"]),
                ("z_min", 0.035),
                ("dz_grasp", 0.0),
                ("dx_grasp", 0.0),
                ("dy_grasp", 0.0),
                # Enhanced depth compensation (inspired by track_and_grab.py line 331-332)
                ("depth_compensation_radius_m", 0.015),  # Object radius compensation
                ("depth_compensation_error_m", 0.04),    # Error compensation (increased from 0.015)
                ("depth_use_enhanced_compensation", True),  # Use track_and_grab style compensation
                ("detection_wait_timeout", 5.0),
                ("detection_topic", "/color_detection/detection_info"),
                ("camera_frame", "depth_camera_link"),
                ("camera_is_optical", True),
                ("base_frame", "base_footprint"),
            ],
        )

        self._load_parameters()

        # TF setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._tf_ready = False
        self._tf_wait_start = time.monotonic()
        self._tf_wait_timer = self.create_timer(0.5, self._check_tf_ready)

        self._recent_targets: Deque[PoseStamped] = deque(
            maxlen=int(self.get_parameter("median_window").get_parameter_value().integer_value or 3)
        )

        self._auto_running = False
        self._auto_start_timer = None

        self._ik_client = self.create_client(SetRobotPose, self.ik_service_name)
        self._ik_ready = False
        self._ik_wait_timer = self.create_timer(1.0, self._check_ik_ready)
        self._check_ik_ready()

        self.servo_pub = self.create_publisher(ServosPosition, self.servo_topic, 10)

        self.action_server = ActionServer(
            self,
            Pick,
            "/pick",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.get_logger().info("Pick action server ready")
        self._log_configuration()

        self.auto_start = bool(self.get_parameter("auto_start").value)
        if self.auto_start:
            self._auto_start_timer = self.create_timer(0.1, self._auto_start_once)

    def _load_parameters(self) -> None:
        self.ik_service_name = str(self.get_parameter("ik.service_name").value)
        self.ik_timeout_s = float(self.get_parameter("ik.timeout_s").value)
        self.ik_pitch_near = float(self.get_parameter("ik.pitch_near").value)
        self.ik_pitch_far = float(self.get_parameter("ik.pitch_far").value)
        self.ik_pitch_z_threshold = float(self.get_parameter("ik.pitch_z_threshold").value)
        self.ik_pitch_range = self._parse_float_list(
            self.get_parameter("ik.pitch_range").value, [-180.0, 180.0], 2
        )
        self.ik_resolution = float(self.get_parameter("ik.resolution").value)

        self.servo_topic = str(self.get_parameter("servo.topic").value)
        self.arm_servo_ids = self._parse_int_list(
            self.get_parameter("servo.arm_ids").value, [1, 2, 3, 4, 5], 5
        )
        self.move_duration = float(self.get_parameter("servo.move_duration").value)
        self.move_settle_s = float(self.get_parameter("servo.move_settle_s").value)
        self.home_pulses = self._parse_int_list(
            self.get_parameter("servo.home_pulses").value, [500, 720, 100, 120, 500], 5
        )
        self.home_duration = float(self.get_parameter("servo.home_duration").value)

        self.gripper_open_pulse = int(self.get_parameter("gripper.open_pulse").value)
        self.gripper_close_pulse = int(self.get_parameter("gripper.close_pulse").value)
        self.gripper_duration = float(self.get_parameter("gripper.duration").value)
        self.gripper_settle_s = float(self.get_parameter("gripper.settle_s").value)

        self.descend_offset = self.get_parameter("grasp_offsets.descend").value
        self.lift_z = float(self.get_parameter("lift_z").value)

        self.detection_wait_timeout = float(self.get_parameter("detection_wait_timeout").value)
        self.detection_topic = str(self.get_parameter("detection_topic").value)
        self.camera_frame = str(self.get_parameter("camera_frame").value)
        self.camera_is_optical = bool(self.get_parameter("camera_is_optical").value)
        self.base_frame = str(self.get_parameter("base_frame").value)

    def _parse_int_list(self, value, fallback, length: int) -> List[int]:
        if isinstance(value, (list, tuple)) and len(value) >= length:
            return [int(v) for v in value[:length]]
        return list(fallback)

    def _parse_float_list(self, value, fallback, length: int) -> List[float]:
        if isinstance(value, (list, tuple)) and len(value) >= length:
            return [float(v) for v in value[:length]]
        return list(fallback)

    def _log_configuration(self) -> None:
        self.get_logger().info("=== Pick Configuration ===")
        self.get_logger().info(f"IK service: {self.ik_service_name}")
        self.get_logger().info(f"Arm servo ids: {self.arm_servo_ids}")
        self.get_logger().info(f"Home pulses: {self.home_pulses}")
        self.get_logger().info(
            f"Gripper pulses: open={self.gripper_open_pulse}, close={self.gripper_close_pulse}"
        )
        self.get_logger().info("==========================")

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

    def _check_ik_ready(self):
        if self._ik_ready:
            return
        self._ik_ready = self._ik_client.wait_for_service(timeout_sec=0.1)
        if self._ik_ready and self._ik_wait_timer is not None:
            self.get_logger().info("Kinematics service ready")
            self._ik_wait_timer.cancel()
            self.destroy_timer(self._ik_wait_timer)
            self._ik_wait_timer = None

    def goal_callback(self, goal_request):
        if self._auto_running:
            self.get_logger().warn("Rejecting external goal while auto_start is running")
            return GoalResponse.REJECT
        if not self._ik_ready:
            self.get_logger().warn("Rejecting pick goal: kinematics service unavailable")
            return GoalResponse.REJECT
        if not self._tf_ready:
            self.get_logger().warn("Rejecting pick goal: TF not ready")
            return GoalResponse.REJECT
        self.get_logger().info("Received grasp goal")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Grasp goal cancelled")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        feedback = Pick.Feedback()
        result = Pick.Result()

        try:
            feedback.stage = "Waiting for detection"
            feedback.progress = 0.05
            goal_handle.publish_feedback(feedback)

            detection_msg = self._wait_for_detection_info(self.detection_wait_timeout)
            if detection_msg is None:
                result.success = False
                result.message = "No detection message received"
                goal_handle.abort()
                return result

            target_pose = self._pose_from_detection(detection_msg)
            if target_pose is None:
                result.success = False
                result.message = "No valid detection target available"
                goal_handle.abort()
                return result

            if getattr(goal_handle, "is_cancel_requested", False) and goal_handle.is_cancel_requested:
                result.success = False
                result.message = "Pick canceled"
                goal_handle.canceled()
                return result

            feedback.stage = "Opening gripper"
            feedback.progress = 0.15
            goal_handle.publish_feedback(feedback)
            self._send_gripper(self.gripper_open_pulse, self.gripper_duration)
            time.sleep(self.gripper_settle_s)

            feedback.stage = "Move to home"
            feedback.progress = 0.25
            goal_handle.publish_feedback(feedback)
            self._move_home()

            if getattr(goal_handle, "is_cancel_requested", False) and goal_handle.is_cancel_requested:
                result.success = False
                result.message = "Pick canceled"
                goal_handle.canceled()
                return result

            feedback.stage = "Compute IK"
            feedback.progress = 0.35
            goal_handle.publish_feedback(feedback)

            grasp_pose = self._compute_grasp_pose(target_pose)
            pulses = self._compute_ik_pulses(grasp_pose)
            if not pulses:
                result.success = False
                result.message = "IK failed for grasp pose"
                goal_handle.abort()
                return result

            feedback.stage = "Move to grasp"
            feedback.progress = 0.45
            goal_handle.publish_feedback(feedback)
            self._send_arm_pulses(pulses, self.move_duration)
            time.sleep(self.move_duration + self.move_settle_s)

            feedback.stage = "Closing gripper"
            feedback.progress = 0.6
            goal_handle.publish_feedback(feedback)
            self._send_gripper(self.gripper_close_pulse, self.gripper_duration)
            time.sleep(self.gripper_settle_s)

            feedback.stage = "Lift"
            feedback.progress = 0.7
            goal_handle.publish_feedback(feedback)
            lift_pose = PoseStamped()
            lift_pose.header = grasp_pose.header
            lift_pose.pose.position.x = grasp_pose.pose.position.x
            lift_pose.pose.position.y = grasp_pose.pose.position.y
            lift_pose.pose.position.z = grasp_pose.pose.position.z + self.lift_z
            lift_pose.pose.orientation = grasp_pose.pose.orientation
            lift_pulses = self._compute_ik_pulses(lift_pose)
            if lift_pulses:
                self._send_arm_pulses(lift_pulses, self.move_duration)
                time.sleep(self.move_duration + self.move_settle_s)
            else:
                self.get_logger().warn("IK failed for lift pose; skipping lift")

            feedback.stage = "Return home"
            feedback.progress = 0.85
            goal_handle.publish_feedback(feedback)
            self._move_home()

            feedback.stage = "completed"
            feedback.progress = 1.0
            goal_handle.publish_feedback(feedback)

            result.success = True
            result.message = "Successfully grasped object"
            goal_handle.succeed()
            return result

        except Exception as e:
            self.get_logger().error(f"Grasp execution exception: {str(e)}")
            result.success = False
            result.message = f"Exception: {str(e)}"
            goal_handle.abort()
            return result

    def _move_home(self) -> None:
        if not self.home_pulses or len(self.home_pulses) < len(self.arm_servo_ids):
            return
        self._send_arm_pulses(self.home_pulses, self.home_duration)
        time.sleep(self.home_duration + self.move_settle_s)

    def _send_arm_pulses(self, pulses: List[int], duration: float) -> None:
        pairs = []
        for servo_id, pulse in zip(self.arm_servo_ids, pulses):
            pairs.append((servo_id, self._clamp_pulse(pulse)))
        self._publish_servo_positions(pairs, duration)

    def _send_gripper(self, pulse: int, duration: float) -> None:
        self._publish_servo_positions([(10, self._clamp_pulse(pulse))], duration)

    def _publish_servo_positions(self, positions: List[tuple], duration: float) -> None:
        msg = ServosPosition()
        msg.duration = float(duration)
        msg.position_unit = "pulse"
        for servo_id, pulse in positions:
            servo_msg = ServoPosition()
            servo_msg.id = int(servo_id)
            servo_msg.position = float(pulse)
            msg.position.append(servo_msg)
        self.servo_pub.publish(msg)

    def _clamp_pulse(self, value: int) -> int:
        return max(0, min(1000, int(value)))

    def _compute_ik_pulses(self, pose: PoseStamped) -> Optional[List[int]]:
        if not self._ik_ready:
            return None
        pitch = self._select_pitch(pose.pose.position.z)
        request = SetRobotPose.Request()
        request.position = [
            float(pose.pose.position.x),
            float(pose.pose.position.y),
            float(pose.pose.position.z),
        ]
        request.pitch = float(pitch)
        request.pitch_range = [float(v) for v in self.ik_pitch_range]
        request.resolution = float(self.ik_resolution)

        future = self._ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.ik_timeout_s)
        if not future.done():
            self.get_logger().warn("IK service call timeout")
            return None
        response = future.result()
        if response is None or not response.success or not response.pulse:
            self.get_logger().warn("IK service returned no pulses")
            return None
        pulses = [self._clamp_pulse(p) for p in response.pulse[: len(self.arm_servo_ids)]]
        return pulses

    def _select_pitch(self, z_value: float) -> float:
        if z_value < self.ik_pitch_z_threshold:
            return self.ik_pitch_near
        return self.ik_pitch_far

    def _wait_for_detection_info(self, timeout_sec: float) -> Optional[String]:
        try:
            ok, msg = wait_for_message(
                String,
                self,
                self.detection_topic,
                time_to_wait=timeout_sec,
            )
        except Exception as e:
            self.get_logger().warn(f"Failed waiting for detection_info: {e}")
            return None
        if not ok:
            return None
        return msg

    def _pose_from_detection(self, msg: String) -> Optional[PoseStamped]:
        if not self._tf_ready:
            return None

        try:
            data = json.loads(msg.data)
            detections = data.get("detections", [])
        except Exception as e:
            self.get_logger().warn(f"Failed to parse detection_info: {e}")
            return None

        if not detections:
            return None

        color_priority: List[str] = [
            c.upper()
            for c in self.get_parameter("color_priority").get_parameter_value().string_array_value
        ]

        chosen = None
        for color in color_priority:
            for det in detections:
                if str(det.get("color_name", "")).upper() == color:
                    chosen = det
                    break
            if chosen:
                break

        if not chosen:
            return None

        z_opt = chosen.get("z_3d")
        x_opt = chosen.get("x_3d")
        y_opt = chosen.get("y_3d")

        if z_opt is None or x_opt is None or y_opt is None:
            u = chosen.get("center_x")
            v = chosen.get("center_y")
            z_opt = chosen.get("depth_value")
            cx = chosen.get("frame_center_x")
            cy = chosen.get("frame_center_y")
            fx = 554.25
            fy = 554.25
            if u is None or v is None or z_opt is None or cx is None or cy is None:
                self.get_logger().warn("Detection missing 3D coordinates and depth; skipping")
                return None
            x_opt = (float(u) - float(cx)) * float(z_opt) / fx
            y_opt = (float(v) - float(cy)) * float(z_opt) / fy

        if self.camera_is_optical:
            x_link = float(x_opt)
            y_link = float(y_opt)
            z_link = float(z_opt)
        else:
            x_link = float(z_opt)
            y_link = float(-x_opt)
            z_link = float(-y_opt)
        
        # Apply enhanced depth compensation (inspired by track_and_grab.py line 331-332)
        if bool(self.get_parameter("depth_use_enhanced_compensation").value):
            radius_comp = float(self.get_parameter("depth_compensation_radius_m").value)
            error_comp = float(self.get_parameter("depth_compensation_error_m").value)
            total_compensation = radius_comp + error_comp
            
            # Apply compensation to z (depth) coordinate
            z_link += total_compensation
            
            self.get_logger().info(
                f"Enhanced depth compensation: {total_compensation:.3f}m "
                f"(radius={radius_comp:.3f}m + error={error_comp:.3f}m)",
                throttle_duration_sec=2.0
            )

        pt = PointStamped()
        pt.header.frame_id = self.camera_frame
        pt.header.stamp = rclpy.time.Time().to_msg()
        pt.point.x = x_link
        pt.point.y = y_link
        pt.point.z = z_link

        try:
            if not self.tf_buffer.can_transform(
                self.base_frame,
                self.camera_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5),
            ):
                self.get_logger().warn(
                    f"TF not ready for {self.camera_frame} -> {self.base_frame}",
                    throttle_duration_sec=2.0,
                )
                return None
            transformed = self.tf_buffer.transform(
                pt, self.base_frame, timeout=rclpy.duration.Duration(seconds=1.0)
            )
        except Exception as e:
            self.get_logger().warn(f"TF transform failed: {e}")
            return None

        pose = PoseStamped()
        pose.header = transformed.header
        z_min = float(self.get_parameter("z_min").value)
        dz_grasp = float(self.get_parameter("dz_grasp").value)
        dx_grasp = float(self.get_parameter("dx_grasp").value)
        dy_grasp = float(self.get_parameter("dy_grasp").value)

        pose.pose.position.x = transformed.point.x + dx_grasp
        pose.pose.position.y = transformed.point.y + dy_grasp
        pose.pose.position.z = max(transformed.point.z + dz_grasp, z_min)
        pose.pose.orientation.w = 1.0

        self._recent_targets.append(pose)
        xs = sorted(p.pose.position.x for p in self._recent_targets)
        ys = sorted(p.pose.position.y for p in self._recent_targets)
        zs = sorted(p.pose.position.z for p in self._recent_targets)
        mid = len(self._recent_targets) // 2
        pose_smoothed = PoseStamped()
        pose_smoothed.header = pose.header
        pose_smoothed.pose.position.x = xs[mid]
        pose_smoothed.pose.position.y = ys[mid]
        pose_smoothed.pose.position.z = zs[mid]
        pose_smoothed.pose.orientation.w = 1.0
        return pose_smoothed

    def _compute_grasp_pose(self, pregrasp_pose: PoseStamped) -> PoseStamped:
        grasp = PoseStamped()
        grasp.header = pregrasp_pose.header
        grasp.pose.position.x = pregrasp_pose.pose.position.x + self.descend_offset[0]
        grasp.pose.position.y = pregrasp_pose.pose.position.y + self.descend_offset[1]
        grasp.pose.position.z = pregrasp_pose.pose.position.z + self.descend_offset[2]
        grasp.pose.orientation = pregrasp_pose.pose.orientation
        return grasp

    def _check_tf_ready(self):
        if self._tf_ready:
            return
        if self.tf_buffer.can_transform(
            self.base_frame,
            self.camera_frame,
            rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=0.1),
        ):
            self._tf_ready = True
            self.get_logger().info(
                f"TF ready for {self.camera_frame} -> {self.base_frame}"
            )
            if self._tf_wait_timer is not None:
                self._tf_wait_timer.cancel()
                self.destroy_timer(self._tf_wait_timer)
                self._tf_wait_timer = None
        else:
            if (time.monotonic() - self._tf_wait_start) >= 2.0:
                self.get_logger().warn(
                    f"Waiting for TF {self.camera_frame} -> {self.base_frame}",
                    throttle_duration_sec=5.0,
                )


def main(args=None):
    rclpy.init(args=args)
    server = PickServer()
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
