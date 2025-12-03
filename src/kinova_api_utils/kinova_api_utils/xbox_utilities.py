"""
Joystick Controller Module
--------------------------
Handles Pygame joystick input and converts it into a delta pose array for robot control.

Axis Mapping:
-------------
- Left Stick X (Axis 0)      : dy (Translation Left/Right) [Inverted]
- Left Stick Y (Axis 1)      : dx (Translation Forward/Back) [Inverted]
- Right Stick X (Axis 2)     : dyaw (Rotation Z)
- Right Stick Y (Axis 3)     : dpitch (Rotation Y)
- Right Throttle (Axis 4)    : +dz (Translation Up)
- Left Throttle (Axis 5)     : -dz (Translation Down)

Button Mapping (State Machine):
-------------------------------
- Button 6 (Click)           : Set Mode to AUTO-CLOSE (-1)
- Button 7 (Click)           : Set Mode to AUTO-OPEN (+1)

Logic Notes:
------------
- Gripper output is a POSITION (0.0 to 1.0).
- Buttons act as triggers to switch the automatic movement direction.
- Returns a flat list of 7 values: [dx, dy, dz, droll, dpitch, dyaw, gripper_pos]
"""

import pygame
import numpy as np

class RobotJoystick:
    def __init__(self, joystick_id=0, max_lin_vel=3, max_ang_vel=3, grip_speed=0.1, 
                 deadzone=0.1, smooth_factor=0.1, initial_gripper_pos=1.0):
        """
        Args:
            grip_speed (float): Speed of auto-opening/closing (0 to 1 scale per second).
                                e.g., 0.5 means it takes 2 seconds to fully open/close.
        """
        self.max_lin_vel = max_lin_vel
        self.max_ang_vel = max_ang_vel
        self.grip_speed = grip_speed
        self.deadzone = deadzone
        self.alpha = smooth_factor
        
        # Axis Indices
        self.AXIS_LX = 0
        self.AXIS_LY = 1
        self.AXIS_RX = 2
        self.AXIS_RY = 3
        self.AXIS_RT = 4
        self.AXIS_LT = 5
        
        # Button Indices
        self.BTN_CLOSE = 6 
        self.BTN_OPEN = 7

        # Internal State (Smoothing)
        self.prev_axes = np.zeros(6)
        
        # Internal State (Gripper)
        self.gripper_pos = max(0.0, min(1.0, initial_gripper_pos))
        
        # Gripper Auto-Mode State:
        #  0 : Idle (Not moving)
        # -1 : Closing continuously
        #  1 : Opening continuously
        self.gripper_auto_mode = 0 

        # Button Edge Detection Memory
        self.prev_btn_close = False
        self.prev_btn_open = False

        # Pygame Init
        pygame.init()
        pygame.joystick.init()
        
        if pygame.joystick.get_count() <= joystick_id:
            raise ConnectionError(f"No joystick found at ID {joystick_id}")
            
        self.joy = pygame.joystick.Joystick(joystick_id)
        self.joy.init()
        
        print(f"Joystick Initialized: {self.joy.get_name()}")
        print(f"Initial Gripper Pos: {self.gripper_pos}")

    def _apply_deadzone(self, value):
        if abs(value) < self.deadzone:
            return 0.0
        return value

    def _normalize_trigger(self, raw_val):
        return (1.0 - raw_val) / 2.0

    def _smooth_input(self, current_axes):
        smoothed = (self.alpha * current_axes) + ((1.0 - self.alpha) * self.prev_axes)
        self.prev_axes = smoothed
        return smoothed

    def get_command(self, dt):
        """
        Returns: [dx, dy, dz, droll, dpitch, dyaw, gripper_position]
        """
        pygame.event.pump()

        # 1. Read Raw Axes
        raw_axes = np.array([
            self.joy.get_axis(self.AXIS_LX),
            self.joy.get_axis(self.AXIS_LY),
            self.joy.get_axis(self.AXIS_RX),
            self.joy.get_axis(self.AXIS_RY),
            self.joy.get_axis(self.AXIS_RT),
            self.joy.get_axis(self.AXIS_LT)
        ])

        # 2. Smooth inputs
        s_axes = self._smooth_input(raw_axes)

        # 3. Map to Robot Frame
        dx = self._apply_deadzone(-s_axes[1]) * self.max_lin_vel * dt
        dy = self._apply_deadzone(-s_axes[0]) * self.max_lin_vel * dt

        up_val = self._normalize_trigger(s_axes[4])
        down_val = self._normalize_trigger(s_axes[5])
        dz = (up_val - down_val) * self.max_lin_vel * dt

        dpitch = self._apply_deadzone(s_axes[3]) * self.max_ang_vel * dt
        dyaw = self._apply_deadzone(s_axes[2]) * self.max_ang_vel * dt
        droll = 0.0

        # --- 4. Gripper State Machine Logic ---
        
        # A. Get current raw button states (True/False)
        curr_btn_close = self.joy.get_button(self.BTN_CLOSE)
        curr_btn_open = self.joy.get_button(self.BTN_OPEN)

        # B. Detect "Rising Edge" (Moment of click)
        # It is True only if pressed NOW but NOT pressed BEFORE
        clicked_close = curr_btn_close and not self.prev_btn_close
        clicked_open = curr_btn_open and not self.prev_btn_open

        # C. Update State (Mode) based on clicks
        if clicked_close:
            self.gripper_auto_mode = -1 
            

        if clicked_open:
            self.gripper_auto_mode = 1  
            
            
        # D. Execute Movement based on current Mode
        # Position = Old_Pos + (Direction * Speed * Time)
        change = self.gripper_auto_mode * self.grip_speed * dt
        self.gripper_pos += change

        # E. Clamp (Hard Limits)
        # Even if mode is still -1, position stops at 0.0
        self.gripper_pos = max(0.0, min(1.0, self.gripper_pos))

        # F. Update History for next loop
        self.prev_btn_close = curr_btn_close
        self.prev_btn_open = curr_btn_open

        delta_pose = np.array([dx, dy, dz, droll, dpitch, dyaw])
        # print(f'Gripper Pos: {self.gripper_pos:.3f}')
        
        return [*delta_pose, self.gripper_pos]
    
    def close(self):
        pygame.quit()