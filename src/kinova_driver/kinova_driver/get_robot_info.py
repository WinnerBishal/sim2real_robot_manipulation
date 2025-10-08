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

from kortex_api.RouterClient import RouterClientSendOptions

from kortex_api.autogen.client_stubs.DeviceManagerClientRpc import DeviceManagerClient
from kortex_api.autogen.client_stubs.DeviceConfigClientRpc import DeviceConfigClient
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient

from kortex_api.autogen.messages import Common_pb2, DeviceManager_pb2, DeviceConfig_pb2, Session_pb2, Base_pb2, ProductConfiguration_pb2

from google.protobuf import json_format



class GetRobotInfoNode(Node):
    def __init__(self):
        super().__init__("get_robot_info")

        self.is_connected = False
        self.connection = None
        self.base = None
        self.router = None

        self.joint_angles_api = None                                                                               # API Object containing joint_angles

        self.connect_srv = self.create_service(Trigger, "connect_to_robot", self.connectCallback)               # Service to connect to robot on request
        
        self.joint_state_pub = self.create_publisher(JointState, "joint_states", 10)                            # To Publish JointState
        self.joint_state_pub_timer = self.create_timer(0.1, self.joint_stateCallback)

        # self.transform_broadcaster = TransformBroadcaster(self, 10)                                              # To broadcast transform data
        # self.transform_broadcaster_timer = self.create_timer(0.1, self.transformBroadcaster)


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

            self.joint_angles_api = self.base.GetMeasuredJointAngles()
            
            joint_msg = JointState()

            joint_msg.header.stamp = self.get_clock().now().to_msg()


            j1 = np.deg2rad(self.joint_angles_api.joint_angles[0].value)
            j2 = np.deg2rad(self.joint_angles_api.joint_angles[1].value)
            j3 = np.deg2rad(self.joint_angles_api.joint_angles[2].value)
            j4 = np.deg2rad(self.joint_angles_api.joint_angles[3].value)
            j5 = np.deg2rad(self.joint_angles_api.joint_angles[4].value)
            j6 = np.deg2rad(self.joint_angles_api.joint_angles[5].value)
            j7 = np.deg2rad(self.joint_angles_api.joint_angles[6].value)

            joint_msg.name = ['gen3_joint_1', 'gen3_joint_2', 'gen3_joint_3', 'gen3_joint_4', 'gen3_joint_5', 'gen3_joint_6', 'gen3_joint_7']
            joint_msg.position = [j1, j2, j3, j4, j5, j6, j7]

            self.joint_state_pub.publish(joint_msg)
            # self.get_logger().info(f"Published Joints : {joint_msg}")

        except Exception as e:
            self.get_logger().info("Failed to publish joint angles !")
            self.get_logger().info(f"Error in def joint_stateCallback :{e}")
    
    # def transformBroadcaster(self):

    #     try:
    #         ee_pose = self.base.ComputeForwardKinematics(self.joint_angles_api)
            
    #     except Exception as e:
    #         self.get_logger().info(f"Error in def transformBroadcaster(self) : {e}")
        


    
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