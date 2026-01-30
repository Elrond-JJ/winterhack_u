#!/usr/bin/env python3
"""
Mission runner node coordinating Nav2 patrol and pick/drop actions.
Supports Mission 1 (Coordinates) and Mission 2 (Ordered Exploration).
"""

import json
import math
import time
from enum import Enum
from typing import Callable, Iterable, Optional, List, Set, Tuple, Dict

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import FollowWaypoints, NavigateToPose
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from tf2_ros import Buffer, TransformListener, TransformException

from winterhack_interfaces.action import Drop, Locate, Pick

try:
    from tf_transformations import quaternion_from_euler  # type: ignore[import-not-found]
except Exception:
    quaternion_from_euler: Optional[Callable[[float, float, float], Tuple[float, float, float, float]]] = None

# Default patrol targets for Mission 0/2 (corners of the map)
DEFAULT_GLOBAL_TARGETS = [(-1.5, -1.5), (1.5, -1.5), (1.5, 1.5), (-1.5, 1.5)]
DEFAULT_HOME = (0.0, 0.0)


class MissionPhase(Enum):
    INIT = "INIT"
    PATROL_SEARCH = "PATROL_SEARCH"
    STOP_SEARCH = "STOP_SEARCH"
    LOCATE = "LOCATE"
    PICK = "PICK"
    RETRIEVE_HOME = "RETRIEVE_HOME"
    DROP = "DROP"
    IDLE = "IDLE"


def yaw_to_quat(yaw: float) -> Quaternion:
    """Convert yaw (rad) to quaternion."""
    if quaternion_from_euler is not None:
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)
    else:
        half = yaw * 0.5
        qx = 0.0
        qy = 0.0
        qz = math.sin(half)
        qw = math.cos(half)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)


def make_pose_stamped(
    x: float, y: float, yaw: float, frame_id: str, stamp=None
) -> PoseStamped:
    """Create a PoseStamped in the given frame."""
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = stamp if stamp is not None else rclpy.time.Time().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0
    pose.pose.orientation = yaw_to_quat(yaw)
    return pose


class MissionRunner(Node):
    """Phase-driven mission controller for patrol and pick/drop."""

    def __init__(self) -> None:
        super().__init__("mission_runner")

        self._cb_group = ReentrantCallbackGroup()
        self._timer_cb_group = MutuallyExclusiveCallbackGroup()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.declare_parameter("home_xy", [0.0, 0.0])
        self.declare_parameter("target_pick_count", 2)
        self.declare_parameter("nav_follow_waypoints_name", "/follow_waypoints")
        self.declare_parameter("nav_to_pose_name", "/navigate_to_pose")
        self.declare_parameter("locate_action_name", "/locate")
        self.declare_parameter("pick_action_name", "/pick")
        self.declare_parameter("drop_action_name", "/drop")
        self.declare_parameter("enable_locate", True)
        self.declare_parameter("enable_pick", True)
        self.declare_parameter("enable_drop", True)
        self.declare_parameter("enable_go_home", True)
        self.declare_parameter("detection_topic", "/color_detection/detection_info")
        self.declare_parameter("map_frame", "map")

        self.declare_parameter("mission_id", 0)
        descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY)
        self.declare_parameter("block_coordinates", [0.0], descriptor)

        self._global_targets = list(DEFAULT_GLOBAL_TARGETS)
        self._home_xy = self._parse_xy(self.get_parameter("home_xy").value, DEFAULT_HOME)
        self._target_pick_count = int(self.get_parameter("target_pick_count").value)
        self._nav_follow_waypoints_name = str(self.get_parameter("nav_follow_waypoints_name").value)
        self._nav_to_pose_name = str(self.get_parameter("nav_to_pose_name").value)
        self._locate_action_name = str(self.get_parameter("locate_action_name").value)
        self._pick_action_name = str(self.get_parameter("pick_action_name").value)
        self._drop_action_name = str(self.get_parameter("drop_action_name").value)
        self._enable_locate = bool(self.get_parameter("enable_locate").value)
        self._enable_pick = bool(self.get_parameter("enable_pick").value)
        self._enable_drop = bool(self.get_parameter("enable_drop").value)
        self._enable_go_home = bool(self.get_parameter("enable_go_home").value)
        self._detection_topic = str(self.get_parameter("detection_topic").value)
        self._map_frame = str(self.get_parameter("map_frame").value)

        self.mission_id = int(self.get_parameter("mission_id").value)
        self._current_patrol_idx = 0 
        self._relative_patrol_idx = 0
        
        self._block_targets: List[Tuple[float, float]] = []
        self.near_target = False

        if self.mission_id == 1:
            raw_coords = self.get_parameter("block_coordinates").value
            if isinstance(raw_coords, list):
                for i in range(0, len(raw_coords), 2):
                    if i + 1 < len(raw_coords):
                        try:
                            pt = (float(raw_coords[i]), float(raw_coords[i+1]))
                            self._block_targets.append(pt)
                        except (ValueError, TypeError):
                            pass
            if self._block_targets:
                self._target_pick_count = len(self._block_targets)
                self.get_logger().info(f"Mission 1: Initialized with {len(self._block_targets)} targets.")

        self.mission_2_targets = ["RED", "BLUE", "YELLOW"]
        self.mission_2_index = 0
        self.found_locations: Dict[str, Tuple[float, float]] = {}

        if self.mission_id == 2:
            self.get_logger().info("Mission 2: Sequential extraction mode (RED->BLUE->YELLOW).")
            self._target_pick_count = 3

        self.detected_colour: Optional[str] = None
        self.detected_colours: Set[str] = set()
        self.picked_total = 0
        self._state = MissionPhase.INIT

        self._follow_client = ActionClient(self, FollowWaypoints, self._nav_follow_waypoints_name, callback_group=self._cb_group)
        self._nav_client = ActionClient(self, NavigateToPose, self._nav_to_pose_name, callback_group=self._cb_group)
        self._pick_client = ActionClient(self, Pick, self._pick_action_name, callback_group=self._cb_group)
        self._locate_client = ActionClient(self, Locate, self._locate_action_name, callback_group=self._cb_group)
        self._drop_client = ActionClient(self, Drop, self._drop_action_name, callback_group=self._cb_group)

        self._follow_goal_future = None
        self._follow_goal_handle = None
        self._follow_result_future = None
        self._follow_cancel_future = None
        self._follow_cancel_wait_until = None

        self._nav_goal_future = None
        self._nav_goal_handle = None
        self._nav_result_future = None

        self._pick_goal_future = None
        self._pick_goal_handle = None
        self._pick_result_future = None

        self._locate_goal_future = None
        self._locate_goal_handle = None
        self._locate_result_future = None

        self._drop_goal_future = None
        self._drop_goal_handle = None
        self._drop_result_future = None

        self._detection_sub = self.create_subscription(String, self._detection_topic, self._on_detection, 10, callback_group=self._cb_group)
        self._stop_srv = self.create_service(Trigger, "/mission/stop", self._on_stop, callback_group=self._cb_group)
        self._timer = self.create_timer(0.1, self._tick, callback_group=self._timer_cb_group)
        
        rclpy.get_default_context().on_shutdown(self._on_shutdown)

    def _get_robot_pose(self) -> Optional[Tuple[float, float]]:
        try:
            t = self.tf_buffer.lookup_transform(self._map_frame, "base_link", rclpy.time.Time())
            return (t.transform.translation.x, t.transform.translation.y)
        except TransformException:
            return None

    def _is_near_home(self, threshold=0.5) -> bool:
        pose = self._get_robot_pose()
        if not pose:
            return False 
        dx = pose[0] - self._home_xy[0]
        dy = pose[1] - self._home_xy[1]
        dist = math.sqrt(dx*dx + dy*dy)
        return dist < threshold

    def _set_state(self, new_state: MissionPhase) -> None:
        if self._state == new_state:
            return
        self.get_logger().info(f"State Transition: {self._state.value} -> {new_state.value}")
        self._state = new_state

    def _parse_xy(self, value: Iterable, fallback: Tuple[float, float]) -> Tuple[float, float]:
        if isinstance(value, (list, tuple)) and len(value) >= 2:
            try:
                return float(value[0]), float(value[1])
            except (TypeError, ValueError):
                return fallback
        return fallback

    def _on_detection(self, msg: String) -> None:
        if self._state != MissionPhase.PATROL_SEARCH:
            return
        if self.mission_id == 1 and not self.near_target:
            return
        if self._is_near_home(threshold=0.5):
            return

        detected_color = self._extract_detected_colour(msg)
        if detected_color is None:
            return
        
        if self.mission_id == 2:
            if detected_color in self.mission_2_targets:
                if detected_color not in self.found_locations:
                    pose = self._get_robot_pose()
                    if pose:
                        self.found_locations[detected_color] = pose
                        self.get_logger().info(f"Memory: {detected_color} recorded at ({pose[0]:.2f}, {pose[1]:.2f})")

            required_color = self.mission_2_targets[self.mission_2_index]
            if detected_color != required_color:
                return 

        if detected_color in self.detected_colours:
            return
        self.detected_colour = detected_color
        self.detected_colours.add(detected_color)
        
        self.get_logger().info(f"Target Acquired: {self.detected_colour}. Stopping patrol.")
        self._set_state(MissionPhase.STOP_SEARCH)

    def _extract_detected_colour(self, msg: String) -> Optional[str]:
        data = (msg.data or "").strip()
        if not data: return None
        try:
            payload = json.loads(data)
        except json.JSONDecodeError: return None
        def normalize(v): return str(v).strip().upper() if v else None
        if isinstance(payload, dict):
            detections = payload.get("detections")
            if isinstance(detections, list):
                for det in detections:
                    if isinstance(det, dict):
                        c = normalize(det.get("color_name"))
                        if c: return c
            summary = payload.get("summary")
            if isinstance(summary, dict):
                for k in summary.keys():
                    c = normalize(k)
                    if c: return c
        return None

    def _tick(self) -> None:
        if self._state == MissionPhase.IDLE: return
        if self._state == MissionPhase.INIT:
            self.picked_total = 0
            self.detected_colour = None
            self.detected_colours.clear()
            self.near_target = False
            self.found_locations.clear()
            self.mission_2_index = 0
            self._current_patrol_idx = 0
            self._set_state(MissionPhase.PATROL_SEARCH)
            return
        
        if self._state == MissionPhase.PATROL_SEARCH: self._tick_patrol()
        elif self._state == MissionPhase.STOP_SEARCH: self._tick_stop_search()
        elif self._state == MissionPhase.LOCATE: self._tick_locate()
        elif self._state == MissionPhase.PICK: self._tick_pick()
        elif self._state == MissionPhase.RETRIEVE_HOME: self._tick_retrieve_home()
        elif self._state == MissionPhase.DROP: self._tick_drop()

    def _tick_patrol(self) -> None:
        if self.mission_id == 1:
            if self.picked_total >= len(self._block_targets):
                self._complete_mission()
                return
            target_xy = self._block_targets[self.picked_total]
            self._update_nav_goal()
            if self._nav_goal_handle is None and self._nav_goal_future is None:
                self.near_target = False
                self.get_logger().info(f"Mission 1: Approaching Block {self.picked_total + 1} at {target_xy}")
                self._send_nav_goal_xy(target_xy[0], target_xy[1], feedback_callback=self._on_nav_feedback)
            elif self._nav_result_future is not None and self._nav_result_future.done():
                success = (self._nav_result_future.result().status == GoalStatus.STATUS_SUCCEEDED)
                self._clear_nav_goal()
                if success:
                    self._set_state(MissionPhase.LOCATE)
            return

        if self.mission_id == 2:
            if self.mission_2_index >= len(self.mission_2_targets):
                 self._complete_mission()
                 return
            
            target_color = self.mission_2_targets[self.mission_2_index]
            if target_color in self.found_locations:
                known_pose = self.found_locations[target_color]
                self._update_nav_goal()
                if self._nav_goal_handle is None and self._nav_goal_future is None:
                    self.get_logger().info(f"Mission 2: Navigating to known {target_color} at {known_pose}.")
                    self._clear_follow_goal()
                    self._send_nav_goal_xy(known_pose[0], known_pose[1])
                elif self._nav_result_future is not None and self._nav_result_future.done():
                    success = (self._nav_result_future.result().status == GoalStatus.STATUS_SUCCEEDED)
                    self._clear_nav_goal()
                    if success:
                        self._set_state(MissionPhase.LOCATE)
                    else:
                        del self.found_locations[target_color]
                return
        
        self._update_follow_goal()
        if self._follow_goal_handle is None and self._follow_goal_future is None:
            # Slice list to resume from breakpoint
            remaining_targets = self._global_targets[self._current_patrol_idx:]
            if not remaining_targets:
                self._current_patrol_idx = 0
                remaining_targets = self._global_targets
            
            self.get_logger().info(f"Resuming patrol loop from index {self._current_patrol_idx}...")
            self._relative_patrol_idx = 0
            self._send_follow_goal(remaining_targets)
        
        elif self._follow_result_future is not None and self._follow_result_future.done():
            self._clear_follow_goal()
            self._current_patrol_idx = 0
            self.get_logger().info("Patrol cycle completed. Resetting to index 0.")

    def _tick_stop_search(self) -> None:
        self._update_follow_goal()
        self._update_nav_goal()
        
        if self._follow_goal_handle:
            if not self._follow_cancel_future:
                # Store the absolute breakpoint before canceling
                self._current_patrol_idx += self._relative_patrol_idx
                self.get_logger().info(f"Saving patrol breakpoint at index {self._current_patrol_idx}")
                self._follow_cancel_future = self._follow_goal_handle.cancel_goal_async()
            elif self._follow_cancel_future.done():
                self._clear_follow_goal()
        elif self._nav_goal_handle:
             if not self._follow_cancel_future: 
                 self._follow_cancel_future = self._nav_goal_handle.cancel_goal_async()
             elif self._follow_cancel_future.done():
                 self._clear_nav_goal()

        if self._follow_goal_handle is None and self._nav_goal_handle is None:
            self._follow_cancel_future = None
            self._set_state(MissionPhase.LOCATE)

    def _tick_locate(self) -> None:
        if not self._enable_locate:
            self._set_state(MissionPhase.PICK)
            return
        self._update_locate_goal()
        if self._locate_goal_handle is None and self._locate_goal_future is None:
            self._send_locate_goal()
        if self._locate_result_future is not None and self._locate_result_future.done():
            success = (self._locate_result_future.result().status == GoalStatus.STATUS_SUCCEEDED)
            self._clear_locate_goal()
            if success:
                self._set_state(MissionPhase.PICK)
            else:
                self._resume_search_after_fail()

    def _tick_pick(self) -> None:
        if not self._enable_pick:
            self.picked_total += 1
            self._set_state(MissionPhase.RETRIEVE_HOME)
            return
        self._update_pick_goal()
        if self._pick_goal_handle is None and self._pick_goal_future is None:
            self._send_pick_goal()
        if self._pick_result_future is not None and self._pick_result_future.done():
            success = (self._pick_result_future.result().status == GoalStatus.STATUS_SUCCEEDED) and self._pick_result_future.result().result.success
            self._clear_pick_goal()
            if success:
                self.picked_total += 1
                self._set_state(MissionPhase.RETRIEVE_HOME)
            else:
                self._resume_search_after_fail()

    def _resume_search_after_fail(self):
        self.get_logger().warn("Action failed. Resuming patrol.")
        if self.detected_colour:
            self.detected_colours.discard(self.detected_colour)
            self.detected_colour = None
        self._set_state(MissionPhase.PATROL_SEARCH)

    def _tick_retrieve_home(self) -> None:
        if not self._enable_go_home:
            self._set_state(MissionPhase.DROP)
            return
        self._update_nav_goal()
        if self._nav_goal_handle is None and self._nav_goal_future is None:
            self.get_logger().info(f"Navigating to Home: {self._home_xy}")
            self._send_nav_goal_xy(self._home_xy[0], self._home_xy[1])
        elif self._nav_result_future is not None and self._nav_result_future.done():
            success = (self._nav_result_future.result().status == GoalStatus.STATUS_SUCCEEDED)
            self._clear_nav_goal()
            if success:
                self._set_state(MissionPhase.DROP)
            else:
                self._set_state(MissionPhase.PATROL_SEARCH) 

    def _tick_drop(self) -> None:
        if not self._enable_drop:
            self._on_drop_success()
            return
        self._update_drop_goal()
        if self._drop_goal_handle is None and self._drop_goal_future is None:
            self._send_drop_goal()
        elif self._drop_result_future is not None and self._drop_result_future.done():
            success = (self._drop_result_future.result().status == GoalStatus.STATUS_SUCCEEDED)
            self._clear_drop_goal()
            self._on_drop_success()

    def _on_drop_success(self):
        self.get_logger().info(f"Drop finished. Total items: {self.picked_total}")
        self.detected_colour = None
        self.detected_colours.clear() 
        
        if self.mission_id == 2:
            self.mission_2_index += 1
            if self.mission_2_index >= len(self.mission_2_targets):
                self._complete_mission()
            else:
                self.get_logger().info(f"Targeting: {self.mission_2_targets[self.mission_2_index]}. Resuming search.")
                self._set_state(MissionPhase.PATROL_SEARCH)
            return

        if self.picked_total >= self._target_pick_count:
            self._complete_mission()
        else:
            self._set_state(MissionPhase.PATROL_SEARCH)

    def _complete_mission(self) -> None:
        self.detected_colour = None
        self.detected_colours.clear()
        self._set_state(MissionPhase.IDLE)
        self.get_logger().info("Mission Accomplished.")

    def _send_follow_goal(self, poses_list=None) -> None:
        if not self._follow_client.wait_for_server(timeout_sec=0.0): return
        targets = poses_list if poses_list is not None else self._global_targets
        if not targets: return
        stamp = self.get_clock().now().to_msg()
        poses = [make_pose_stamped(x, y, 0.0, self._map_frame, stamp) for x, y in targets]
        goal = FollowWaypoints.Goal()
        goal.poses = poses
        self._follow_goal_future = self._follow_client.send_goal_async(goal, feedback_callback=self._on_follow_feedback)

    def _on_follow_feedback(self, feedback_msg):
        # Continuous tracking of relative progress within the current slice
        self._relative_patrol_idx = feedback_msg.feedback.current_waypoint

    def _send_nav_goal_xy(self, x: float, y: float, feedback_callback=None) -> None:
        if not self._nav_client.wait_for_server(timeout_sec=0.0): return
        self.get_logger().info(f"Sending Nav goal -> ({x:.2f}, {y:.2f})")
        goal = NavigateToPose.Goal()
        goal.pose = make_pose_stamped(x, y, 0.0, self._map_frame, self.get_clock().now().to_msg())
        self._nav_goal_future = self._nav_client.send_goal_async(goal, feedback_callback=feedback_callback)

    def _on_nav_feedback(self, feedback_msg) -> None:
        if feedback_msg.feedback.distance_remaining < 0.25:
            self.near_target = True

    # --- Standard Action Senders ---
    def _send_pick_goal(self):
        if not self._pick_client.wait_for_server(0): return
        self._pick_goal_future = self._pick_client.send_goal_async(Pick.Goal(), feedback_callback=self._log_feedback)
    def _send_locate_goal(self):
        if not self._locate_client.wait_for_server(0): return
        self._locate_goal_future = self._locate_client.send_goal_async(Locate.Goal(), feedback_callback=self._log_feedback)
    def _send_drop_goal(self):
        if not self._drop_client.wait_for_server(0): return
        self._drop_goal_future = self._drop_client.send_goal_async(Drop.Goal(), feedback_callback=self._log_feedback)
    def _log_feedback(self, msg):
        s = (msg.feedback.stage or "").strip()
        if s: self.get_logger().info(f"Feedback: {s}")

    # --- Goal Updates & Clearing ---
    def _update_follow_goal(self):
        if self._follow_goal_future and self._follow_goal_future.done():
            self._follow_goal_handle = self._follow_goal_future.result()
            self._follow_goal_future = None
            if self._follow_goal_handle and self._follow_goal_handle.accepted:
                self._follow_result_future = self._follow_goal_handle.get_result_async()
            else: 
                self.get_logger().warn("Patrol REJECTED.")
                self._follow_goal_handle = None

    def _update_nav_goal(self):
        if self._nav_goal_future and self._nav_goal_future.done():
            self._nav_goal_handle = self._nav_goal_future.result()
            self._nav_goal_future = None
            if self._nav_goal_handle and self._nav_goal_handle.accepted:
                self._nav_result_future = self._nav_goal_handle.get_result_async()
            else: 
                self.get_logger().warn("Nav REJECTED.")
                self._nav_goal_handle = None 
            
    def _update_pick_goal(self):
        if self._pick_goal_future and self._pick_goal_future.done():
            self._pick_goal_handle = self._pick_goal_future.result()
            self._pick_goal_future = None
            if self._pick_goal_handle and self._pick_goal_handle.accepted:
                self._pick_result_future = self._pick_goal_handle.get_result_async()
    def _update_locate_goal(self):
        if self._locate_goal_future and self._locate_goal_future.done():
            self._locate_goal_handle = self._locate_goal_future.result()
            self._locate_goal_future = None
            if self._locate_goal_handle and self._locate_goal_handle.accepted:
                self._locate_result_future = self._locate_goal_handle.get_result_async()
    def _update_drop_goal(self):
        if self._drop_goal_future and self._drop_goal_future.done():
            self._drop_goal_handle = self._drop_goal_future.result()
            self._drop_goal_future = None
            if self._drop_goal_handle and self._drop_goal_handle.accepted:
                self._drop_result_future = self._drop_goal_handle.get_result_async()

    def _clear_follow_goal(self):
        self._follow_goal_future = None
        self._follow_goal_handle = None
        self._follow_result_future = None
        self._follow_cancel_future = None
    def _clear_nav_goal(self):
        self._nav_goal_future = None
        self._nav_goal_handle = None
        self._nav_result_future = None
    def _clear_pick_goal(self):
        self._pick_goal_future = None
        self._pick_goal_handle = None
        self._pick_result_future = None
    def _clear_locate_goal(self):
        self._locate_goal_future = None
        self._locate_goal_handle = None
        self._locate_result_future = None
    def _clear_drop_goal(self):
        self._drop_goal_future = None
        self._drop_goal_handle = None
        self._drop_result_future = None

    def _on_stop(self, request, response):
        self._on_shutdown()
        self.detected_colour = None
        self.detected_colours.clear()
        self._set_state(MissionPhase.IDLE)
        response.success = True
        return response

    def _on_shutdown(self):
        for h in [self._follow_goal_handle, self._nav_goal_handle, self._pick_goal_handle, self._locate_goal_handle, self._drop_goal_handle]:
            if h: h.cancel_goal_async()

def main():
    rclpy.init()
    node = MissionRunner()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try: executor.spin()
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()