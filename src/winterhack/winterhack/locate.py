#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Locate action server - hybrid alignment approach:
1. Coarse base alignment using wheel movement
2. Fine servo-based visual tracking (inspired by LanderPi track_and_grab.py)
Enhanced with depth-aware control for improved positioning accuracy.
"""

import json
import time
import threading
import numpy as np
from dataclasses import dataclass
from typing import Any, Optional

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import String
from servo_controller_msgs.msg import ServoPosition, ServosPosition
from winterhack_interfaces.action import Locate

from .servo_tracker import ServoTracker


@dataclass
class PIDState:
    """State for a simple PID controller."""
    integral: float = 0.0
    prev_error: float = 0.0
    prev_time: float | None = None


class LocateNode(Node):
    """Hybrid alignment: base movement for coarse positioning + servo tracking for fine alignment."""

    def __init__(self):
        """Initialize parameters, subscriptions, publishers, and control state."""
        super().__init__("locate")
        self._cb_group = ReentrantCallbackGroup()

        # Topics
        self.declare_parameter("detections_topic", "/color_detection/detection_info")
        self.declare_parameter("cmd_vel_topic", "controller/cmd_vel")
        self.declare_parameter("servo_topic", "/servo_controller")

        # Detection selection
        self.declare_parameter("color_priority", ["GREEN"])

        # Alignment mode: "base_only", "servo_only", or "hybrid"
        self.declare_parameter("alignment_mode", "hybrid")
        
        # Servo tracking parameters (inspired by track_and_grab.py)
        self.declare_parameter("use_servo_tracking", True)
        self.declare_parameter("servo_yaw_limits", [200, 800])  # servo1 pulse limits
        self.declare_parameter("servo_pitch_limits", [100, 720])  # servo4 pulse limits
        self.declare_parameter("servo_initial_yaw", 500)
        self.declare_parameter("servo_initial_pitch", 150)
        self.declare_parameter("servo_pid_p", 20.5)
        self.declare_parameter("servo_pid_i", 1.0)
        self.declare_parameter("servo_pid_d", 1.2)
        self.declare_parameter("servo_update_interval", 0.02)  # 50Hz (same as track_and_grab)
        self.declare_parameter("servo_stability_time_s", 2.0)  # Same as track_and_grab
        self.declare_parameter("servo_stability_tolerance_px", 3)  # Same as track_and_grab
        
        # Threshold to switch from base to servo control
        self.declare_parameter("base_to_servo_threshold_px", 50)  # When pixel error < 50px, use servo

        # Gates
        self.declare_parameter("detection_timeout_s", 0.5)

        # Control
        self.declare_parameter("k_yaw_px", 0.002)  # rad/s per pixel horizontal error (P fallback)
        self.declare_parameter("k_fwd_z", 0.8)     # m/s per pixel vertical error (P fallback)
        # Desired pixel location for the detected object (-1 uses frame center).
        self.declare_parameter("target_u_px", 320.0)
        self.declare_parameter("target_v_px", 200.0)
        
        # Depth-based control (inspired by track_and_grab.py)
        self.declare_parameter("use_depth_control", True)  # Use depth instead of pixel y for forward control
        self.declare_parameter("target_distance_m", 0.30)  # Target distance in meters (similar to track_and_grab approach distance)
        self.declare_parameter("distance_tolerance_m", 0.02)  # Distance tolerance (2cm)
        self.declare_parameter("depth_offset_m", 0.055)  # Distance compensation: 0.015 (object radius) + 0.04 (error compensation)
        self.declare_parameter("min_valid_depth_m", 0.1)  # Minimum valid depth
        self.declare_parameter("max_valid_depth_m", 3.0)  # Maximum valid depth
        
        # Stability parameters (inspired by track_and_grab.py's 2-second stability check)
        self.declare_parameter("stability_time_s", 2.0)  # Time to stay stable before declaring success
        self.declare_parameter("stability_tolerance_px", 3)  # Pixel tolerance for stability check (same as track_and_grab)
        self.declare_parameter("stability_depth_tolerance_m", 0.01)  # Depth stability tolerance (1cm)
        
        self.declare_parameter("use_pid", True)
        self.declare_parameter("yaw_kp", 0.1)
        self.declare_parameter("yaw_ki", 0.02)
        self.declare_parameter("yaw_kd", 0.01)
        self.declare_parameter("fwd_kp", 0.5)  # Increased for distance-based control
        self.declare_parameter("fwd_ki", 0.02)
        self.declare_parameter("fwd_kd", 0.01)
        self.declare_parameter("yaw_i_max", 0.02)
        self.declare_parameter("fwd_i_max", 0.02)
        self.declare_parameter("pid_dt_min", 0.02)
        self.declare_parameter("yaw_max", 0.2)
        self.declare_parameter("v_max", 0.2)
        self.declare_parameter("deadband_px", 6)
        self.declare_parameter("log_control", True)
        self.declare_parameter("log_every_n", 1)
        self.declare_parameter("alignment_timeout_s", 10.0)

        self.latest_detection: dict[str, Any] | None = None
        self.last_detection_time = None
        self.yaw_pid = PIDState()
        self.fwd_pid = PIDState()
        self.fwd_within_deadband_count = 0
        self.log_counter = 0
        self.in_deadband = False
        self._last_success = False
        self._last_message = ""
        self._timed_out = False
        self._active = False
        self.sub_det = None
        
        # Stability tracking (inspired by track_and_grab.py)
        self.last_stable_position = None  # (u, v, depth, timestamp)
        self.stability_start_time = None
        self.stable_readings_buffer = []  # Rolling buffer for stability checking
        
        # Servo tracker (NEW: inspired by track_and_grab.py)
        self.servo_tracker: Optional[ServoTracker] = None
        self.use_servo_tracking = bool(self.get_parameter("use_servo_tracking").value)
        self.alignment_mode = str(self.get_parameter("alignment_mode").value)
        self.base_to_servo_threshold = int(self.get_parameter("base_to_servo_threshold_px").value)
        self.servo_last_update_time = 0.0
        
        if self.use_servo_tracking:
            yaw_limits = self._parse_int_list(self.get_parameter("servo_yaw_limits").value, [200, 800])
            pitch_limits = self._parse_int_list(self.get_parameter("servo_pitch_limits").value, [100, 720])
            pid_params = {
                "P": float(self.get_parameter("servo_pid_p").value),
                "I": float(self.get_parameter("servo_pid_i").value),
                "D": float(self.get_parameter("servo_pid_d").value),
            }
            self.servo_tracker = ServoTracker(
                yaw_limits=tuple(yaw_limits),
                pitch_limits=tuple(pitch_limits),
                initial_yaw=int(self.get_parameter("servo_initial_yaw").value),
                initial_pitch=int(self.get_parameter("servo_initial_pitch").value),
                pid_params=pid_params,
            )
            self.servo_tracker.stability_duration_s = float(self.get_parameter("servo_stability_time_s").value)
            self.servo_tracker.stability_threshold_px = int(self.get_parameter("servo_stability_tolerance_px").value)

        self.pub_cmd = self.create_publisher(Twist, self.get_parameter("cmd_vel_topic").value, 10)
        self.pub_servo = self.create_publisher(ServosPosition, self.get_parameter("servo_topic").value, 10)

        self.timer = self.create_timer(0.05, self._loop, callback_group=self._cb_group)  # 20 Hz
    
    def _parse_int_list(self, value, fallback):
        """Parse parameter as integer list."""
        if isinstance(value, (list, tuple)) and len(value) >= len(fallback):
            return [int(v) for v in value[:len(fallback)]]
        return list(fallback)

    def reset_result(self) -> None:
        """Reset the last alignment result."""
        self._last_success = False
        self._last_message = ""
        self._timed_out = False

    def get_result(self) -> tuple[bool, str]:
        """Return the last alignment result (success, message)."""
        return self._last_success, self._last_message

    def start_alignment(self, *, reset_detection: bool = False) -> None:
        """Enable alignment control and clear prior status."""
        self.reset_result()
        self.in_deadband = False
        self.fwd_within_deadband_count = 0
        self._active = True
        
        # Reset stability tracking
        self.stability_start_time = None
        self.last_stable_position = None
        self.stable_readings_buffer = []
        
        # Reset servo tracker
        if self.servo_tracker is not None:
            self.servo_tracker.reset()
            # Initialize servos to starting position
            self._publish_servo_positions([(1, self.servo_tracker.yaw), (4, self.servo_tracker.pitch)], duration=1.0)
            time.sleep(1.0)
        
        if self.sub_det is None:
            self.sub_det = self.create_subscription(
                String,
                self.get_parameter("detections_topic").value,
                self._on_detection,
                10,
                callback_group=self._cb_group,
            )
        if reset_detection:
            self.latest_detection = None
            self.last_detection_time = None

    def stop_alignment(self) -> None:
        """Disable alignment control and stop the base."""
        self._active = False
        if self.sub_det is not None:
            self.destroy_subscription(self.sub_det)
            self.sub_det = None
        self.latest_detection = None
        self.last_detection_time = None
        self._publish_stop()

    def _publish_stop(self) -> None:
        """Publish zero velocity command."""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.pub_cmd.publish(cmd)

    def _on_shutdown(self) -> None:
        """Ensure base is stopped on ROS shutdown."""
        self._publish_stop()

    # --- Callbacks ---
    def _on_detection(self, msg: String):
        """Parse detection JSON and cache the chosen detection."""
        try:
            payload = json.loads(msg.data)
        except Exception:
            return

        detections = payload.get("detections", [])
        if not detections:
            return

        chosen = self._select_detection(detections)
        self.latest_detection = chosen
        self.last_detection_time = self.get_clock().now()

    def _loop(self):
        """Run the hybrid alignment loop: base movement + servo tracking."""
        if not self._active:
            return
        if self.latest_detection is None or self.last_detection_time is None:
            return

        timeout_s = float(self.get_parameter("detection_timeout_s").value)
        if timeout_s > 0.0:
            age_ns = (self.get_clock().now() - self.last_detection_time).nanoseconds
            if age_ns > int(timeout_s * 1e9):
                self._publish_stop()
                self._last_success = False
                self._last_message = "Detection timeout"
                self._timed_out = True
                self.get_logger().info("Detection timeout; stopping base")
                return

        det = self.latest_detection
        try:
            u = float(det.get("center_x"))
            v = float(det.get("center_y"))
            frame_cx = det.get("frame_center_x")
            frame_cy = det.get("frame_center_y")
            if frame_cx is None or frame_cy is None:
                return
            frame_width = int(frame_cx) * 2  # Assuming center is at width/2
            frame_height = int(frame_cy) * 2
        except (TypeError, ValueError):
            return

        # Extract depth information if available
        depth_value = det.get("depth_value")  # In meters
        use_depth = bool(self.get_parameter("use_depth_control").value)
        
        # Validate depth (similar to track_and_grab.py depth validation)
        valid_depth = False
        compensated_depth = None
        if depth_value is not None and not np.isnan(depth_value):
            min_depth = float(self.get_parameter("min_valid_depth_m").value)
            max_depth = float(self.get_parameter("max_valid_depth_m").value)
            if min_depth <= depth_value <= max_depth:
                valid_depth = True
                # Apply depth offset compensation (inspired by track_and_grab.py line 331-332)
                depth_offset = float(self.get_parameter("depth_offset_m").value)
                compensated_depth = depth_value + depth_offset

        # Calculate pixel errors for decision making
        target_u_px = float(self.get_parameter("target_u_px").value)
        desired_u_px = float(frame_cx) if target_u_px < 0.0 else target_u_px
        x_error_px = desired_u_px - u
        
        target_v_px = float(self.get_parameter("target_v_px").value)
        desired_v_px = float(frame_cy) if target_v_px < 0.0 else target_v_px
        y_error_px = desired_v_px - v
        
        pixel_error = np.sqrt(x_error_px**2 + y_error_px**2)
        
        # === HYBRID ALIGNMENT STRATEGY ===
        # Decision: Use base movement or servo tracking based on error magnitude
        use_base_control = True
        use_servo_control = False
        
        if self.alignment_mode == "hybrid":
            # Hybrid: Use base for large errors, servo for fine adjustments
            if pixel_error > self.base_to_servo_threshold:
                use_base_control = True
                use_servo_control = False
            else:
                use_base_control = False
                use_servo_control = True and self.use_servo_tracking
        elif self.alignment_mode == "servo_only":
            use_base_control = False
            use_servo_control = self.use_servo_tracking
        else:  # "base_only"
            use_base_control = True
            use_servo_control = False
        
        # === SERVO TRACKING (Fine adjustment, inspired by track_and_grab.py) ===
        servo_stable = False
        if use_servo_control and self.servo_tracker is not None:
            # Update servo positions (similar to track_and_grab.py line 303)
            current_time = time.time()
            servo_update_interval = float(self.get_parameter("servo_update_interval").value)
            
            if current_time - self.servo_last_update_time >= servo_update_interval:
                servo_commands, servo_stable = self.servo_tracker.update(
                    center_x=u,
                    center_y=v,
                    frame_width=frame_width,
                    frame_height=frame_height,
                    deadband_tolerance=0.02,
                )
                
                if servo_commands is not None:
                    pitch, yaw = servo_commands
                    self._publish_servo_positions([(1, yaw), (4, pitch)], duration=servo_update_interval)
                
                self.servo_last_update_time = current_time
            
            # Stop base movement when using servo control
            if use_base_control:
                self._publish_stop()
            
            # Check if servo tracking achieved stable alignment
            if servo_stable:
                if not self.in_deadband:
                    self._publish_stop()
                    self.in_deadband = True
                    self._last_success = True
                    if use_depth and valid_depth:
                        self._last_message = f"Servo alignment complete (stable, depth={compensated_depth:.3f}m)"
                    else:
                        self._last_message = "Servo alignment complete (stable)"
                    self.get_logger().info(self._last_message)
                return
        
        # === BASE CONTROL (Coarse adjustment) ===
        if use_base_control:
            # YAW CONTROL
            dead_px = int(self.get_parameter("deadband_px").value)
            within_yaw_deadband = abs(x_error_px) <= dead_px

            if within_yaw_deadband:
                wz = 0.0
                self.yaw_pid.integral = 0.0
                self.yaw_pid.prev_error = 0.0
            elif bool(self.get_parameter("use_pid").value):
                wz = self._pid_step(
                    self.yaw_pid,
                    x_error_px,
                    float(self.get_parameter("yaw_kp").value),
                    float(self.get_parameter("yaw_ki").value),
                    float(self.get_parameter("yaw_kd").value),
                    float(self.get_parameter("yaw_i_max").value),
                )
                yaw_max = float(self.get_parameter("yaw_max").value)
                wz = max(-yaw_max, min(yaw_max, wz))
            else:
                wz = self._pixel_yaw(x_error_px)

            # FORWARD CONTROL (depth-based or pixel-based)
            vx = 0.0
            within_fwd_deadband = False
            distance_error = None

            if use_depth and valid_depth and compensated_depth is not None:
                # Use depth-based control
                target_distance = float(self.get_parameter("target_distance_m").value)
                distance_tolerance = float(self.get_parameter("distance_tolerance_m").value)
                distance_error = compensated_depth - target_distance
                
                within_fwd_deadband = abs(distance_error) <= distance_tolerance
                
                if within_fwd_deadband:
                    vx = 0.0
                    self.fwd_pid.integral = 0.0
                    self.fwd_pid.prev_error = 0.0
                elif bool(self.get_parameter("use_pid").value):
                    vx = self._pid_step(
                        self.fwd_pid,
                        -distance_error,
                        float(self.get_parameter("fwd_kp").value),
                        float(self.get_parameter("fwd_ki").value),
                        float(self.get_parameter("fwd_kd").value),
                        float(self.get_parameter("fwd_i_max").value),
                    )
                else:
                    k_fwd = 0.5
                    vx = -k_fwd * distance_error
                    
                v_max = float(self.get_parameter("v_max").value)
                vx = max(-v_max, min(v_max, vx))
                
            else:
                # Fallback to pixel-based control
                within_fwd_deadband = abs(y_error_px) <= dead_px
                
                if within_fwd_deadband:
                    vx = 0.0
                    self.fwd_pid.integral = 0.0
                    self.fwd_pid.prev_error = 0.0
                else:
                    if bool(self.get_parameter("use_pid").value):
                        vx = self._pid_step(
                            self.fwd_pid,
                            y_error_px,
                            float(self.get_parameter("fwd_kp").value),
                            float(self.get_parameter("fwd_ki").value),
                            float(self.get_parameter("fwd_kd").value),
                            float(self.get_parameter("fwd_i_max").value),
                        )
                    else:
                        k_fwd_px = float(self.get_parameter("k_fwd_z").value)
                        vx = k_fwd_px * y_error_px
                    v_max = float(self.get_parameter("v_max").value)
                    vx = max(-v_max, min(v_max, vx))

            # Base stability check
            current_position = (u, v, compensated_depth if valid_depth else None)
            is_stable = False
            
            if within_yaw_deadband and within_fwd_deadband:
                stability_tol_px = int(self.get_parameter("stability_tolerance_px").value)
                depth_tol = float(self.get_parameter("stability_depth_tolerance_m").value)
                
                if self.last_stable_position is not None:
                    last_u, last_v, last_depth = self.last_stable_position
                    
                    pos_stable = (abs(u - last_u) < stability_tol_px and 
                                 abs(v - last_v) < stability_tol_px)
                    
                    if use_depth and valid_depth and last_depth is not None:
                        depth_stable = abs(compensated_depth - last_depth) < depth_tol
                        pos_stable = pos_stable and depth_stable
                    
                    if pos_stable:
                        if self.stability_start_time is None:
                            self.stability_start_time = time.monotonic()
                        else:
                            stability_time_s = float(self.get_parameter("stability_time_s").value)
                            elapsed = time.monotonic() - self.stability_start_time
                            if elapsed >= stability_time_s:
                                is_stable = True
                    else:
                        self.stability_start_time = None
                else:
                    self.stability_start_time = time.monotonic()
                
                self.last_stable_position = current_position
            else:
                self.stability_start_time = None
                self.last_stable_position = None

            # Decision: Stop or Continue
            if is_stable:
                if not self.in_deadband:
                    self._publish_stop()
                    self.in_deadband = True
                    self._last_success = True
                    if use_depth and valid_depth:
                        self._last_message = f"Base alignment complete (stable, depth={compensated_depth:.3f}m)"
                    else:
                        self._last_message = "Base alignment complete (stable, pixel-based)"
                    self.get_logger().info(self._last_message)
                return
            self.in_deadband = False

            # Logging
            if bool(self.get_parameter("log_control").value):
                self.log_counter += 1
                log_every_n = int(self.get_parameter("log_every_n").value)
                if log_every_n <= 1 or self.log_counter % log_every_n == 0:
                    mode_str = "SERVO" if use_servo_control else "BASE"
                    if use_depth and valid_depth:
                        self.get_logger().info(
                            f"[{mode_str}] x_err={x_error_px:.1f}px wz={wz:.3f} | "
                            f"depth={compensated_depth:.3f}m dist_err={distance_error:.3f}m vx={vx:.3f}"
                        )
                    else:
                        self.get_logger().info(
                            f"[{mode_str}] x_err={x_error_px:.1f}px wz={wz:.3f} vx={vx:.3f} (no depth)"
                        )

            # Publish base command
            cmd = Twist()
            cmd.linear.x = float(vx)
            cmd.angular.z = float(wz)
            self.pub_cmd.publish(cmd)

    # --- Detection selection ---
    def _select_detection(self, detections: list[dict[str, Any]]) -> dict[str, Any]:
        """Pick a detection based on color priority or largest area."""
        color_priority = [
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
            chosen = max(detections, key=lambda d: float(d.get("area", 0.0)))
        return chosen
    
    # --- Servo control (NEW: inspired by track_and_grab.py) ---
    def _publish_servo_positions(self, positions: list[tuple[int, int]], duration: float) -> None:
        """
        Publish servo positions.
        
        Args:
            positions: List of (servo_id, pulse) tuples
            duration: Movement duration in seconds
        """
        msg = ServosPosition()
        msg.duration = float(duration)
        msg.position_unit = "pulse"
        for servo_id, pulse in positions:
            servo_msg = ServoPosition()
            servo_msg.id = int(servo_id)
            servo_msg.position = float(self._clamp_pulse(pulse))
            msg.position.append(servo_msg)
        self.pub_servo.publish(msg)
    
    def _clamp_pulse(self, value: int) -> int:
        """Clamp servo pulse to valid range [0, 1000]."""
        return max(0, min(1000, int(value)))

    # --- Camera intrinsics ---
    def _pixel_yaw(self, x_error_px: float) -> float:
        """Compute yaw command from pixel error using P fallback."""
        dead_px = int(self.get_parameter("deadband_px").value)
        if abs(x_error_px) <= dead_px:
            return 0.0
        k_yaw_px = float(self.get_parameter("k_yaw_px").value)
        yaw_max = float(self.get_parameter("yaw_max").value)
        wz = k_yaw_px * x_error_px
        return max(-yaw_max, min(yaw_max, wz))

    def _pid_step(self, state: PIDState, error: float, kp: float, ki: float, kd: float, i_max: float) -> float:
        """Advance PID state and return control output."""
        now = self.get_clock().now()
        if state.prev_time is None:
            state.prev_time = now
            state.prev_error = error
            return kp * error

        dt = (now - state.prev_time).nanoseconds * 1e-9
        dt_min = float(self.get_parameter("pid_dt_min").value)
        if dt < dt_min:
            dt = dt_min

        state.integral += error * dt
        if i_max > 0.0:
            state.integral = max(-i_max, min(i_max, state.integral))

        derivative = (error - state.prev_error) / dt
        state.prev_error = error
        state.prev_time = now

        return kp * error + ki * state.integral + kd * derivative


class _AutoGoalHandle:
    def __init__(self, node: LocateNode):
        self._node = node

    def publish_feedback(self, _feedback):
        pass

    def abort(self):
        self._node.get_logger().warn("Auto-start locate aborted")

    def succeed(self):
        self._node.get_logger().info("Auto-start locate completed")


class LocateServer(LocateNode):
    """Action server for alignment using detection pixel errors."""

    def __init__(self):
        super().__init__()

        self.declare_parameter("auto_start", False)

        self._auto_running = False
        self._auto_start_timer = None
        self._cancel_requested = False

        self.action_server = ActionServer(
            self,
            Locate,
            "/locate",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._cb_group,
        )

        self.get_logger().info("Locate action server ready")

        if bool(self.get_parameter("auto_start").value):
            self._auto_start_timer = self.create_timer(0.1, self._auto_start_once)

    def goal_callback(self, _goal_request):
        if self._auto_running:
            self.get_logger().warn("Rejecting external goal while auto_start is running")
            return GoalResponse.REJECT
        self.get_logger().info("Received locate goal")
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle):
        self.get_logger().info("Locate goal cancelled")
        self._cancel_requested = True
        self.stop_alignment()
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

    def execute_callback(self, goal_handle):
        feedback = Locate.Feedback()
        result = Locate.Result()

        self._cancel_requested = False
        self.start_alignment(reset_detection=False)

        feedback.stage = "Aligning"
        feedback.progress = 0.0
        goal_handle.publish_feedback(feedback)

        timeout_s = float(self.get_parameter("alignment_timeout_s").value)
        start_time = time.monotonic()

        while rclpy.ok():
            if getattr(goal_handle, "is_cancel_requested", False) and goal_handle.is_cancel_requested:
                self.stop_alignment()
                result.success = False
                result.message = "Locate canceled"
                goal_handle.canceled()
                return result

            if self.in_deadband:
                self.stop_alignment()
                feedback.stage = "completed"
                feedback.progress = 1.0
                goal_handle.publish_feedback(feedback)
                result.success = True
                result.message = "Alignment complete"
                goal_handle.succeed()
                return result

            if self._timed_out:
                self.stop_alignment()
                result.success = False
                result.message = self._last_message or "Detection timeout"
                goal_handle.abort()
                return result

            if timeout_s > 0.0 and (time.monotonic() - start_time) > timeout_s:
                self.stop_alignment()
                result.success = False
                result.message = "Alignment timeout"
                goal_handle.abort()
                return result

            time.sleep(0.05)

        self.stop_alignment()
        result.success = False
        result.message = "ROS shutdown"
        goal_handle.abort()
        return result


def main(args=None):
    rclpy.init(args=args)
    server = LocateServer()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(server)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        server.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()