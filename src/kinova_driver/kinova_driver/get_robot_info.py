#!/usr/bin/env python3

# Import ROS2 essentials
import rclpy
from rclpy.node import Node

from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from kinova_interfaces.msg import JointState7D

from tf2_ros import TransformBroadcaster, TransformStamped


# Import Kinova API essentials
from kinova_api_utils import utilities

import sys
import os
import numpy as np


from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient


class GetRobotInfoNode(Node):
    def __init__(self):
        super().__init__("get_robot_info")

        self.is_connected = False
        self.connection = None
        self.base = None
        self.base_cyclic = None
        self.router = None

        self.joint_angles_api = None    
        self.gripper_value_api = None                                                                           # API Object containing joint_angles

        self.connect_srv = self.create_service(Trigger, "connect_to_robot", self.connectCallback)               # Service to connect to robot on request
        
        self.joint_state_pub = self.create_publisher(JointState, "joint_states", 10)                            # To Publish JointState
        self.joint_state_pub_timer = self.create_timer(0.1, self.joint_stateCallback)

        self.get_logger().info("WAITING for connection request ......")


    def connectCallback(self, request, response):

        # Check if robot is already connected
        if self.is_connected:
            response.success = True
            response.message = "Connection Already Established"
        
        # Try to connect
        try:
            self.get_logger().info("Connecting to robot...")

            # Use API Utilities : It sets IP address, username, password and ports : using default args
            connection_args = utilities.parseConnectionArguments()
            
            self.connection = utilities.DeviceConnection.createTcpConnection(connection_args) # Create TCP Connection
            self.router = self.connection.__enter__()     
            self.base = BaseClient(self.router)                                         # Shareable robot authenticated instance
            self.base_cyclic = BaseCyclicClient(self.router)

            self.feedback = None
            
            self.is_connected = True
            
            response.success = True
            response.message = "Successfully connected to the robot"
            self.get_logger().info(response.message)               

        except Exception as e:
            self.is_connected = False

            response.success = False
            response.message = f"Failed to connect : {e}"
            self.get_logger().info(response.message)
            
        
        return response
    
    def joint_stateCallback(self):

        if not self.is_connected:
            return
        
        try:

            self.feedback = self.base_cyclic.RefreshFeedback()
            actuators = self.feedback.actuators

            joint_msg = JointState()

            joint_msg.header.stamp = self.get_clock().now().to_msg()


            j1 = np.deg2rad(actuators[0].position)
            j2 = np.deg2rad(actuators[1].position)
            j3 = np.deg2rad(actuators[2].position)
            j4 = np.deg2rad(actuators[3].position)
            j5 = np.deg2rad(actuators[4].position)
            j6 = np.deg2rad(actuators[5].position)
            j7 = np.deg2rad(actuators[6].position)

            g0 = self.feedback.interconnect.gripper_feedback.motor[0].position              

            joint_msg.name = ['gen3_joint_1', 'gen3_joint_2', 'gen3_joint_3', 'gen3_joint_4', 'gen3_joint_5', 'gen3_joint_6', 'gen3_joint_7', 'gen3_robotiq_85_left_knuckle_joint']
            joint_msg.position = [j1, j2, j3, j4, j5, j6, j7, g0-1]

            self.joint_state_pub.publish(joint_msg)
            # self.get_logger().info(f"Published Joints : {joint_msg}")

        except Exception as e:
            self.get_logger().info("Failed to publish joint angles !")
            self.get_logger().info(f"Error in def joint_stateCallback :{e}")

    
    def disconnect_from_robot(self):

        if self.is_connected and self.connection:
            self.connection.__exit__(None, None, None)
            self.is_connected = False
            self.get_logger().info("Successfully disconnected from robot.")
        

def main(args = None):
    rclpy.init(args=args)
    node = GetRobotInfoNode()
    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down.")

    finally:
        node.disconnect_from_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()