"""
Joystick Controller Module
--------------------------
Handles Pygame joystick input and converts it into a delta pose array for robot control.

Axis Mapping (Standard Gamepad):
--------------------------------
- Left Stick X (Axis 0)      : dy (Translation Left/Right)
- Left Stick Y (Axis 1)      : dx (Translation Forward/Back) [Inverted]
- Right Stick X (Axis 2)     : dyaw (Rotation Z)
- Right Stick Y (Axis 3)     : dpitch (Rotation Y) [Inverted]
- Right Throttle (Axis 4)    : +dz (Translation Up)
- Left Throttle (Axis 5)     : -dz (Translation Down)

Button Mapping:
---------------
- Button 6 (LB)              : Close Gripper (-1)
- Button 7 (RB)              : Open Gripper (1)

Logic Notes:
------------
- Throttles are normalized from [-1, 1] to [0, 1] range to act as linear actuators.
- Deadzones are applied to stick axes to prevent drift.
- Output is a numpy array: [dx, dy, dz, droll, dpitch, dyaw].
"""

import pygame
import numpy as np

class RobotJoystick:
    def __init__(self, joystick_id=0, max_lin_vel=0.2, max_ang_vel=0.5, deadzone=0.1):
        self.max_lin_vel = max_lin_vel
        self.max_ang_vel = max_ang_vel
        self.deadzone = deadzone
        
        # Axis Configuration
        self.AXIS_LX = 0
        self.AXIS_LY = 1
        self.AXIS_RX = 2
        self.AXIS_RY = 3
        self.AXIS_RT = 4
        self.AXIS_LT = 5
        
        # Button Configuration
        self.BTN_CLOSE = 6
        self.BTN_OPEN = 7

        # Pygame Initialization
        pygame.init()
        pygame.joystick.init()
        
        if pygame.joystick.get_count() <= joystick_id:
            raise ConnectionError(f"No joystick found at ID {joystick_id}")
            
        self.joy = pygame.joystick.Joystick(joystick_id)
        self.joy.init()

    def _apply_deadzone(self, value):
        if abs(value) < self.deadzone:
            return 0.0
        return value

    def _normalize_trigger(self, raw_val):
        # Convert range [-1, 1] (1=rest) to [0, 1] (0=rest)
        return (1.0 - raw_val) / 2.0

    def get_command(self, dt):
        pygame.event.pump()

        # Translation (World Frame)
        # Note: Joystick Y-axes are inverted (-1 is forward/up)
        dx = self._apply_deadzone(-self.joy.get_axis(self.AXIS_LY)) * self.max_lin_vel * dt
        dy = self._apply_deadzone(self.joy.get_axis(self.AXIS_LX)) * self.max_lin_vel * dt

        # Z-Axis (Trigger Logic)
        up_val = self._normalize_trigger(self.joy.get_axis(self.AXIS_RT))
        down_val = self._normalize_trigger(self.joy.get_axis(self.AXIS_LT))
        dz = (up_val - down_val) * self.max_lin_vel * dt

        # Rotation
        dpitch = self._apply_deadzone(-self.joy.get_axis(self.AXIS_RY)) * self.max_ang_vel * dt
        dyaw = self._apply_deadzone(self.joy.get_axis(self.AXIS_RX)) * self.max_ang_vel * dt
        droll = 0.0

        # Gripper Logic
        gripper_cmd = 0
        if self.joy.get_button(self.BTN_OPEN):
            gripper_cmd = 1
        elif self.joy.get_button(self.BTN_CLOSE):
            gripper_cmd = -1

        delta_pose = np.array([dx, dy, dz, droll, dpitch, dyaw])
        
        return [*delta_pose, gripper_cmd]
    
    def close(self):
        pygame.quit()