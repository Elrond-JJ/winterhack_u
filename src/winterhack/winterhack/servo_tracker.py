#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Servo-based visual tracking for fine-grained alignment.
Inspired by LanderPi track_and_grab.py's PID-based servo control.
"""

import time
from typing import Optional, Tuple


class PID:
    """Simple PID controller for servo tracking."""
    
    def __init__(self, P=20.5, I=1.0, D=1.2):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        
        self.sample_time = 0.00
        self.current_time = time.time()
        self.last_time = self.current_time
        
        self.clear()
    
    def clear(self):
        """Reset PID state."""
        self.SetPoint = 0.0
        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0
        self.int_error = 0.0
        self.windup_guard = 20.0
        self.output = 0.0
    
    def update(self, feedback_value):
        """Calculate PID output for given feedback."""
        error = self.SetPoint - feedback_value
        
        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error
        
        if delta_time >= self.sample_time:
            self.PTerm = self.Kp * error
            self.ITerm += error * delta_time
            
            # Anti-windup
            if self.ITerm < -self.windup_guard:
                self.ITerm = -self.windup_guard
            elif self.ITerm > self.windup_guard:
                self.ITerm = self.windup_guard
            
            self.DTerm = 0.0
            if delta_time > 0:
                self.DTerm = delta_error / delta_time
            
            self.last_time = self.current_time
            self.last_error = error
            
            self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)


class ServoTracker:
    """
    Fine-grained visual tracking using arm servos.
    Based on LanderPi's track_and_grab.py approach.
    """
    
    def __init__(
        self,
        yaw_limits: Tuple[int, int] = (200, 800),  # servo1 limits (pulse)
        pitch_limits: Tuple[int, int] = (100, 720),  # servo4 limits (pulse)
        initial_yaw: int = 500,
        initial_pitch: int = 150,
        pid_params: Optional[dict] = None,
    ):
        """
        Initialize servo tracker.
        
        Args:
            yaw_limits: (min, max) pulse limits for yaw servo
            pitch_limits: (min, max) pulse limits for pitch servo
            initial_yaw: Starting yaw position (pulse)
            initial_pitch: Starting pitch position (pulse)
            pid_params: Optional dict with P, I, D keys
        """
        self.yaw_limits = yaw_limits
        self.pitch_limits = pitch_limits
        self.yaw = initial_yaw
        self.pitch = initial_pitch
        
        # PID controllers (inspired by track_and_grab.py line 41-42)
        params = pid_params or {"P": 20.5, "I": 1.0, "D": 1.2}
        self.pid_yaw = PID(params["P"], params["I"], params["D"])
        self.pid_pitch = PID(params["P"], params["I"], params["D"])
        
        # Stability tracking (inspired by track_and_grab.py line 309-310)
        self.last_servo_position = (self.pitch, self.yaw)
        self.stability_start_time: Optional[float] = None
        self.stability_threshold_px = 3  # Same as track_and_grab.py
        self.stability_duration_s = 2.0  # Same as track_and_grab.py
    
    def reset(self):
        """Reset tracker to initial position and clear PID state."""
        self.yaw = 500
        self.pitch = 150
        self.pid_yaw.clear()
        self.pid_pitch.clear()
        self.stability_start_time = None
        self.last_servo_position = (self.pitch, self.yaw)
    
    def update(
        self,
        center_x: float,
        center_y: float,
        frame_width: int,
        frame_height: int,
        deadband_tolerance: float = 0.02,
    ) -> Tuple[Optional[Tuple[int, int]], bool]:
        """
        Update servo positions based on detected object center.
        
        Args:
            center_x: Object center x in pixels
            center_y: Object center y in pixels
            frame_width: Image width
            frame_height: Image height
            deadband_tolerance: Normalized tolerance for deadband (0.02 = 2%)
        
        Returns:
            ((pitch, yaw) pulses or None, is_stable)
            - Servo positions to publish (None if no update needed)
            - True if position is stable for required duration
        """
        # Normalize coordinates (inspired by track_and_grab.py line 80-86)
        center_x_norm = center_x / frame_width
        center_y_norm = center_y / frame_height
        
        # Update yaw (horizontal centering)
        yaw_updated = False
        if abs(center_x_norm - 0.5) > deadband_tolerance:
            self.pid_yaw.SetPoint = 0.5  # Target is image center
            self.pid_yaw.update(center_x_norm)
            new_yaw = self.yaw + self.pid_yaw.output
            self.yaw = max(self.yaw_limits[0], min(self.yaw_limits[1], int(new_yaw)))
            yaw_updated = True
        else:
            self.pid_yaw.clear()
        
        # Update pitch (vertical centering)
        pitch_updated = False
        if abs(center_y_norm - 0.5) > deadband_tolerance:
            self.pid_pitch.SetPoint = 0.5
            self.pid_pitch.update(center_y_norm)
            new_pitch = self.pitch + self.pid_pitch.output
            self.pitch = max(self.pitch_limits[0], min(self.pitch_limits[1], int(new_pitch)))
            pitch_updated = True
        else:
            self.pid_pitch.clear()
        
        # Check stability (inspired by track_and_grab.py line 309-310)
        is_stable = False
        current_position = (self.pitch, self.yaw)
        
        if abs(current_position[0] - self.last_servo_position[0]) < self.stability_threshold_px and \
           abs(current_position[1] - self.last_servo_position[1]) < self.stability_threshold_px:
            # Position is stable
            if self.stability_start_time is None:
                self.stability_start_time = time.time()
            else:
                elapsed = time.time() - self.stability_start_time
                if elapsed >= self.stability_duration_s:
                    is_stable = True
        else:
            # Position changed, reset stability timer
            self.stability_start_time = None
        
        self.last_servo_position = current_position
        
        # Return servo commands if updated
        if yaw_updated or pitch_updated:
            return (int(self.pitch), int(self.yaw)), is_stable
        else:
            return None, is_stable
    
    def get_current_position(self) -> Tuple[int, int]:
        """Get current servo positions as (pitch, yaw) in pulses."""
        return (int(self.pitch), int(self.yaw))
    
    def is_centered(self, center_x: float, center_y: float, frame_width: int, frame_height: int, tolerance: float = 0.02) -> bool:
        """Check if object is centered within tolerance."""
        center_x_norm = center_x / frame_width
        center_y_norm = center_y / frame_height
        return abs(center_x_norm - 0.5) <= tolerance and abs(center_y_norm - 0.5) <= tolerance
