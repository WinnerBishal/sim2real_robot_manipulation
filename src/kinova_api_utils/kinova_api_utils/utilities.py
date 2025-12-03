#!/usr/bin/env python3

import argparse
import threading
import time

from kortex_api.TCPTransport import TCPTransport
from kortex_api.UDPTransport import UDPTransport
from kortex_api.RouterClient import RouterClient, RouterClientSendOptions
from kortex_api.SessionManager import SessionManager
from kortex_api.autogen.messages import Session_pb2

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient

from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2

import numpy as np

def parseConnectionArguments(parser = argparse.ArgumentParser()):
    parser.add_argument("--ip", type=str, help="IP address of destination", default="192.168.1.10")
    parser.add_argument("-u", "--username", type=str, help="username to login", default="admin")
    parser.add_argument("-p", "--password", type=str, help="password to login", default="admin")
    return parser.parse_args()

class DeviceConnection:
    
    TCP_PORT = 10000
    UDP_PORT = 10001

    @staticmethod
    def createTcpConnection(args): 
        """
        returns RouterClient required to create services and send requests to device or sub-devices,
        """

        return DeviceConnection(args.ip, port=DeviceConnection.TCP_PORT, credentials=(args.username, args.password))

    @staticmethod
    def createUdpConnection(args): 
        """        
        returns RouterClient that allows to create services and send requests to a device or its sub-devices @ 1khz.
        """

        return DeviceConnection(args.ip, port=DeviceConnection.UDP_PORT, credentials=(args.username, args.password))

    def __init__(self, ipAddress, port=TCP_PORT, credentials = ("","")):

        self.ipAddress = ipAddress
        self.port = port
        self.credentials = credentials

        self.sessionManager = None

        # Setup API
        self.transport = TCPTransport() if port == DeviceConnection.TCP_PORT else UDPTransport()
        self.router = RouterClient(self.transport, RouterClient.basicErrorCallback)

    # Called when entering 'with' statement
    def __enter__(self):
        
        self.transport.connect(self.ipAddress, self.port)

        if (self.credentials[0] != ""):
            session_info = Session_pb2.CreateSessionInfo()
            session_info.username = self.credentials[0]
            session_info.password = self.credentials[1]
            session_info.session_inactivity_timeout = 10000   # (milliseconds)
            session_info.connection_inactivity_timeout = 2000 # (milliseconds)

            self.sessionManager = SessionManager(self.router)
            print("Logging as", self.credentials[0], "on device", self.ipAddress)
            self.sessionManager.CreateSession(session_info)

        return self.router

    # Called when exiting 'with' statement
    def __exit__(self, exc_type, exc_value, traceback):
    
        if self.sessionManager != None:

            router_options = RouterClientSendOptions()
            router_options.timeout_ms = 1000 
            
            self.sessionManager.CloseSession(router_options)

        self.transport.disconnect()

class ExecuteRobotAction:

    def __init__(self):
        
        self.currentPose = np.zeros(6)
        self.currentJointAngles = np.zeros(6)
        self.currentGripperPosition = 0.0

        self.action = None
        self.goalPose = np.zeros(6)

        self.connection = None
        self.router = None
        self.base = None
        self.baseCyclic = None

        self.jointData = None

        self.isConnected = False
        
        # Cache for gripper to avoid spamming the bus during continuous control
        self.last_gripper_val = -1.0
    
    def connect_to_robot(self):

        try:
            connectionArgs = parseConnectionArguments()

            self.connection = DeviceConnection.createTcpConnection(connectionArgs)
            self.router = self.connection.__enter__()
            self.base = BaseClient(self.router)
            self.baseCyclic = BaseCyclicClient(self.router)

            self.isConnected = True

            print("\n Connected to Robot Successfully \n")
            
            # CRITICAL: Set Servoing Mode to SINGLE_LEVEL_SERVOING
            # This is required for the robot to accept real-time Twist commands
            base_servo_mode = Base_pb2.ServoingModeInformation()
            base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
            self.base.SetServoingMode(base_servo_mode)

            jointData = self.baseCyclic.RefreshFeedback().actuators
            self.currentJointAngles = [np.deg2rad(jointData[i].position) for i in range(len(jointData))]
            self.currentGripperPosition = self.baseCyclic.RefreshFeedback().interconnect.gripper_feedback.motor[0].position

        except Exception as e:
            self.isConnected = False
            print(f"ERROR in ExecuteRobotAction.connect_to_robot(): {e}")
    
    def get_current_state(self):

        self.jointData = self.baseCyclic.RefreshFeedback().actuators
        self.currentJointAngles = [np.deg2rad(self.jointData[i].position) for i in range(len(self.jointData))]
        self.currentGripperPosition = self.baseCyclic.RefreshFeedback().interconnect.gripper_feedback.motor[0].position

        return self.currentJointAngles + [self.currentGripperPosition]
    
    def disconnect_from_robot(self):
        
        try:
            disconnect = self.connection.__exit__()
            print(f"Disconnected from robot. {disconnect}")
        
        except Exception as e:
            print(f"Error disconnecting: {e}")
    
    def check_for_end_or_abort(self, e):
    
        def check(notification, e = e):
            print("EVENT : " + \
                Base_pb2.ActionEvent.Name(notification.action_event))
            if notification.action_event == Base_pb2.ACTION_END \
            or notification.action_event == Base_pb2.ACTION_ABORT:
                e.set()
        return check


    def move_robot(self, action):

        robotAction = Base_pb2.Action()
        robotAction.name = "Cartesian Action Movement"
        robotAction.application_data = ""

        poseData = self.base.GetMeasuredCartesianPose()

        self.goalPose[0] = poseData.x + action[0]
        self.goalPose[1] = poseData.y + action[1]
        self.goalPose[2] = poseData.z + action[2]
        self.goalPose[3] = poseData.theta_x + np.rad2deg(action[3])
        self.goalPose[4] = poseData.theta_y + np.rad2deg(action[4])
        self.goalPose[5] = poseData.theta_z + np.rad2deg(action[5])

        commandPose = robotAction.reach_pose.target_pose

        commandPose.x = self.goalPose[0]
        commandPose.y = self.goalPose[1]
        commandPose.z = self.goalPose[2]
        commandPose.theta_x = self.goalPose[3]
        commandPose.theta_y = self.goalPose[4]
        commandPose.theta_z = self.goalPose[5]
        
        # Just a way to check if the action is successfully executed by the robot.

        e = threading.Event()
        notif_handle = self.base.OnNotificationActionTopic(
            self.check_for_end_or_abort(e),
            Base_pb2.NotificationOptions()
        )

        # Execute the commands here
        print("Executing Action")
        self.base.ExecuteAction(robotAction)

        finished = e.wait(20)
        self.base.Unsubscribe(notif_handle)

        if finished:
            print("Current robot action complete.")
        
        else:
            print("Timeout on action notification wait")
        
    
    def move_gripper(self, action):

        currentPosition = self.baseCyclic.RefreshFeedback().interconnect.gripper_feedback.motor[0].position
        # print(f"Current Position: {currentPosition}") # Commented out for speed
        targetPosition = 1 - action[6]

        gripperCommand = Base_pb2.GripperCommand()
        finger = gripperCommand.gripper.finger.add()

        # Run gripper in position mode

        gripperCommand.mode = Base_pb2.GRIPPER_POSITION
        finger.finger_identifier = 1
        finger.value = targetPosition

        self.base.SendGripperCommand(gripperCommand)

        # print("Gripper Commanded") # Commented out for speed
    
    def act(self, action):
        """
        Original slow action method.
        """
        if not self.isConnected:
            print("Robot is not connected, exiting !")
            return False
        
        self.move_robot(action)
        self.move_gripper(action)

        # FOR LOGGING PURPOSES
        poseData = self.base.GetMeasuredCartesianPose()
        jointData = self.baseCyclic.RefreshFeedback().actuators

        self.currentJointAngles = [np.deg2rad(jointData[i].position) for i in range(len(jointData))]

        self.currentPose[0] = poseData.x
        self.currentPose[1] = poseData.y
        self.currentPose[2] = poseData.z
        self.currentPose[3] = poseData.theta_x
        self.currentPose[4] = poseData.theta_y
        self.currentPose[5] = poseData.theta_z

        self.currentGripperPosition = self.baseCyclic.RefreshFeedback().interconnect.gripper_feedback.motor[0].position

    def act_twist(self, action, dt=0.05):
        """
        High-Frequency Velocity Control Method.
        action: [dx, dy, dz, droll, dpitch, dyaw, gripper_pos]
        dt: Time step of the loop (seconds)
        """
        if not self.isConnected:
            print("Robot is not connected, exiting !")
            return False

        # --- 1. Velocity Command (Twist) ---
        # Initialize the Twist Command object
        command = Base_pb2.TwistCommand()
        
        # NOTE: We use BASE frame for teleop so "Forward" on joystick = "Forward" in room.
        # If you prefer "Forward" = "Gripper pointing direction", change to CARTESIAN_REFERENCE_FRAME_TOOL
        command.reference_frame = Base_pb2.CARTESIAN_REFERENCE_FRAME_BASE
        # command.duration = 0  # 0 means continue indefinitely (streaming mode)

        twist = command.twist
        
        # Linear (m/s) -> Recover velocity from displacement (dx/dt)
        twist.linear_x = action[0] / dt
        twist.linear_y = action[1] / dt
        twist.linear_z = action[2] / dt

        # Angular (deg/s) -> Kortex expects degrees for Twist
        twist.angular_x = np.rad2deg(action[3] / dt)
        twist.angular_y = np.rad2deg(action[4] / dt)
        twist.angular_z = np.rad2deg(action[5] / dt)

        # Send command
        self.base.SendTwistCommand(command)

        # --- 2. Gripper Command (Optimized) ---
        # Only send command if position changes significantly to save bandwidth
        current_gripper_cmd = action[6]
        
        if abs(current_gripper_cmd - self.last_gripper_val) > 0.01:
            self.move_gripper(action)
            self.last_gripper_val = current_gripper_cmd
        
        return True

def check_robot_connection(args):

    try:
        with DeviceConnection.createTcpConnection(args) as router:

            pass

        return True
    
    except:

        return False
    

if __name__ == "__main__":

    robot = ExecuteRobotAction()
    robot.connect_to_robot()
    # robot.act([0, 0, 0.1, 0, 0, 0, 0.5]) # Old way
    # robot.act_twist([0, 0, 0.1, 0, 0, 0, 0.5], dt=0.05) # New way