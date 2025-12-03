"""
ROS2 Joystick Node
------------------
A ROS2 node that interfaces with the RobotJoystick class to publish delta pose
commands and gripper states at a fixed frequency.

Topics Published:
-----------------
None

Parameters:
-----------
- loop_rate (double): Frequency of the control loop in Hz. Default: 50.0
- linear_vel (double): Maximum linear velocity (m/s). Default: 0.2
- angular_vel (double): Maximum angular velocity (rad/s). Default: 0.5
- joystick_id (int): System ID of the joystick. Default: 0

Dependencies:
-------------
- rclpy
- std_msgs
- joystick_controller (local file)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int8
from kinova_api_utils import xbox_utilities, utilities
import sys

class JoystickNode(Node):
    def __init__(self):
        super().__init__('joystick_teleop_node')

        # --- Parameters ---
        self.declare_parameter('loop_rate', 70.0)
        self.declare_parameter('linear_vel', 0.2)
        self.declare_parameter('angular_vel', 0.5)
        self.declare_parameter('joystick_id', 0)

        self.loop_rate = self.get_parameter('loop_rate').value
        lin_vel = self.get_parameter('linear_vel').value
        ang_vel = self.get_parameter('angular_vel').value
        joy_id = self.get_parameter('joystick_id').value

        self.get_logger().info(f"Joystick Node Started with Parameters: loop_rate={self.loop_rate}, linear_vel={lin_vel}, angular_vel={ang_vel}, joystick_id={joy_id}")

        # --- Robot Initialization ---
        try:
            self.get_logger().info("Connecting to robot...")
            self.robot = utilities.ExecuteRobotAction()
            self.robot.connect_to_robot()

            if self.robot.isConnected:
                self.get_logger().info("Robot connected successfully.")

        except Exception as e:
            self.get_logger().error(f"Failed to connect to robot: {e}")

        # --- Joystick Initialization ---
        try:
            self.controller = xbox_utilities.RobotJoystick(
                joystick_id=joy_id,
                max_lin_vel=lin_vel,
                max_ang_vel=ang_vel
            )
            self.get_logger().info(f"Joystick Initialized at {self.loop_rate}Hz")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize joystick: {e}")
            sys.exit(1)

        # --- Timer ---
        # We use a fixed timer period (dt) for consistent velocity integration
        self.dt = 1.0 / self.loop_rate
        self.timer = self.create_timer(self.dt, self.timer_callback)

    def timer_callback(self):
        try:
            # Calculate delta pose based on the fixed time step (dt)
            action = self.controller.get_command(self.dt)

            # Move robot
            self.robot.act_twist(action, self.dt)
            

        except Exception as e:
            self.get_logger().error(f"Error in control loop: {e}")

    def destroy_node(self):
        self.controller.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    node = JoystickNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()