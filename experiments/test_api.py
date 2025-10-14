#! /usr/bin/env python3

###
# KINOVA (R) KORTEX (TM)
#
# Copyright (c) 2021 Kinova inc. All rights reserved.
#
# This software may be modified and distributed
# under the terms of the BSD 3-Clause license.
#
# Refer to the LICENSE file for details.
#
###

# import sys
# import os
# import time
# import threading

# from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
# from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient

# from kortex_api.autogen.client_stubs.GripperCyclicClientRpc import GripperCyclicClient
# from kortex_api.autogen.messages import GripperCyclic_pb2

# from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2, Common_pb2

# # Maximum allowed waiting time during actions (in seconds)
# TIMEOUT_DURATION = 100

# def main():
    
#     # Import the utilities helper module
#     sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
#     from src.kinova_api_utils.kinova_api_utils import utilities

#     # Parse arguments
#     args = utilities.parseConnectionArguments()
    
#     # Create connection to the device and get the router
#     with utilities.DeviceConnection.createTcpConnection(args) as router:

#         # Create required services
#         base = BaseClient(router)
#         base_cyclic = BaseCyclicClient(router)
        
#         feedback = base_cyclic.RefreshFeedback()
#         gripper_feedback = feedback.interconnect.gripper_feedback
#         gripper_motor_val = gripper_feedback.motor[0].position
#         # Example core
#         success = True

#         print("TEST STARTS ...")
        
#         print(gripper_motor_val)

#         return 0 if success else 1

# if __name__ == "__main__":
#     exit(main())

#! /usr/bin/env python3

###
# KINOVA (R) KORTEX (TM)
#
# Copyright (c) 2019 Kinova inc. All rights reserved.
#
# This software may be modified and distributed under the
# terms of the BSD 3-Clause license.
#
# Refer to the LICENSE file for details.
#
###

import sys
import os
import time

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.messages import Base_pb2

class GripperCommandExample:
    def __init__(self, router, proportional_gain = 2.0):

        self.proportional_gain = proportional_gain
        self.router = router

        # Create base client using TCP router
        self.base = BaseClient(self.router)

    def ExampleSendGripperCommands(self):

        # Create the GripperCommand we will send
        gripper_command = Base_pb2.GripperCommand()
        finger = gripper_command.gripper.finger.add()

        # Close the gripper with position increments
        print("Performing gripper test in position...")
        gripper_command.mode = Base_pb2.GRIPPER_POSITION
        position = 0.00
        finger.finger_identifier = 1
        while position < 1.0:
            finger.value = position
            print("Going to position {:0.2f}...".format(finger.value))
            self.base.SendGripperCommand(gripper_command)
            position += 0.1
            time.sleep(1)

        # Set speed to open gripper
        print ("Opening gripper using speed command...")
        gripper_command.mode = Base_pb2.GRIPPER_SPEED
        finger.value = 0.1
        self.base.SendGripperCommand(gripper_command)
        gripper_request = Base_pb2.GripperRequest()

        # Wait for reported position to be opened
        gripper_request.mode = Base_pb2.GRIPPER_POSITION
        while True:
            gripper_measure = self.base.GetMeasuredGripperMovement(gripper_request)
            if len (gripper_measure.finger):
                print("Current position is : {0}".format(gripper_measure.finger[0].value))
                if gripper_measure.finger[0].value < 0.01:
                    break
            else: # Else, no finger present in answer, end loop
                break

       

def main():
    # Import the utilities helper module
    import argparse
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    from src.kinova_api_utils.kinova_api_utils import utilities

    # Parse arguments
    parser = argparse.ArgumentParser()
    args = utilities.parseConnectionArguments(parser)

    # Create connection to the device and get the router
    with utilities.DeviceConnection.createTcpConnection(args) as router:

        example = GripperCommandExample(router)
        example.ExampleSendGripperCommands()

if __name__ == "__main__":
    main()