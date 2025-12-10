
import pygame

pygame.init()
pygame.joystick.init()

# Check if controller is connected
if pygame.joystick.get_count() == 0:
    print("No gamepad connected.")
    exit()

# initialize the first joystick
joystick = pygame.joystick.Joystick(0)
joystick.init()

print(f"Gamepad connected: {joystick.get_name()}")
print(f"Number of axes: {joystick.get_numaxes()}")
print(f"Number of buttons: {joystick.get_numbuttons()}")
print("-"*30)
print("Waiting for input, Press any button or move sticks")

try:
    while True:
        for event in pygame.event.get():

            # Button pressed
            if event.type == pygame.JOYBUTTONDOWN:
                print(f"Button {event.button} pressed")
            
            # Analog stick or triggers
            elif event.type == pygame.JOYAXISMOTION:
                if abs(event.value) > 0.1:  # Deadzone threshold
                    print(f"Axis {event.axis} moved to {event.value:.2f}")
            
            # D-Pad
            elif event.type == pygame.JOYHATMOTION:
                print(f"D-Pad moved to {event.value}")

except KeyboardInterrupt:
    print("\nExiting gamepad test.")

'''

import pygame
import numpy as np

# Configuration Constants
DEADZONE = 0.1
MAX_LIN_VEL = 0.5  # Meters per second
MAX_ANG_VEL = 1.0  # Radians per second

def apply_deadzone(value, threshold=DEADZONE):
    if abs(value) < threshold:
        return 0.0
    return value

def normalize_trigger(raw_value):
    """
    Converts Pygame trigger scale (1.0=rest, -1.0=pressed)
    to a 0.0 (rest) to 1.0 (pressed) scale.
    """
    # 1.0 minus value makes rest=0 and pressed=2
    # Divide by 2 to get range 0.0 to 1.0
    return (1.0 - raw_value) / 2.0

def get_joystick_command(joystick, dt):
    """
    Reads joystick and returns delta_pose (x, y, z, roll, pitch, yaw)
    and gripper_command.
    """
    pygame.event.pump() # Process event queue to get fresh data
    
    # --- 1. Translation (X, Y) ---
    # Axis 0: Left Stick Left/Right -> dy (Side)
    # Axis 1: Left Stick Up/Down    -> dx (Forward) - INVERTED
    raw_dy = apply_deadzone(joystick.get_axis(0))
    raw_dx = apply_deadzone(-joystick.get_axis(1)) # Note the negative sign
    
    dx = raw_dx * MAX_LIN_VEL * dt
    dy = raw_dy * MAX_LIN_VEL * dt

    # --- 2. Translation (Z) - Using Throttles ---
    # Axis 5: Left Throttle (Down)
    # Axis 4: Right Throttle (Up)
    val_left_trigger = normalize_trigger(joystick.get_axis(5))
    val_right_trigger = normalize_trigger(joystick.get_axis(4))
    
    # Right trigger moves UP (+Z), Left moves DOWN (-Z)
    dz = (val_right_trigger - val_left_trigger) * MAX_LIN_VEL * dt

    # --- 3. Rotation (Pitch, Yaw) ---
    # Axis 2: Right Stick Left/Right -> dyaw (Rotate Z)
    # Axis 3: Right Stick Up/Down    -> dpitch (Rotate Y) - INVERTED
    raw_dyaw = apply_deadzone(joystick.get_axis(2))
    raw_dpitch = apply_deadzone(-joystick.get_axis(3)) # Note the negative sign

    droll = 0.0 # Mapping Roll to buttons is safer, or leave as 0
    dpitch = raw_dpitch * MAX_ANG_VEL * dt
    dyaw = raw_dyaw * MAX_ANG_VEL * dt

    # --- 4. Gripper ---
    # Button 6: Close (-1), Button 7: Open (+1)
    gripper_cmd = 0
    if joystick.get_button(7):
        gripper_cmd = 1
    elif joystick.get_button(6):
        gripper_cmd = -1

    # Return Delta Pose Array [x, y, z, roll, pitch, yaw] and Gripper
    delta_pose = np.array([dx, dy, dz, droll, dpitch, dyaw])
    return delta_pose, gripper_cmd

# --- Example Usage Loop ---
if __name__ == "__main__":
    pygame.init()
    pygame.joystick.init()
    
    if pygame.joystick.get_count() > 0:
        js = pygame.joystick.Joystick(0)
        js.init()
        print(f"Controller: {js.get_name()}")
        
        try:
            clock = pygame.time.Clock()
            while True:
                dt = clock.tick(30) / 1000.0 # Delta time in seconds
                
                d_pose, grip = get_joystick_command(js, dt)
                
                # Only print if there is movement
                if np.linalg.norm(d_pose) > 0 or grip != 0:
                    print(f"Delta: {np.round(d_pose, 3)} | Grip: {grip}")
                    
        except KeyboardInterrupt:
            print("Exiting...")

'''