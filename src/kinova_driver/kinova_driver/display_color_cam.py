#!/usr/bin/env python3

import rclpy 
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
import matplotlib.pyplot as plt

from sensor_msgs.msg import Image, CameraInfo

class SubscribeCameraNode(Node):
    def __init__(self):
        super().__init__("display_color_cam")

        self.isAvailable = False
    
        self.color_cam_raw_sub = self.create_subscription(Image, "/camera/color/image_raw", self.displayCamCallback, 1)
        # self.color_cam_info_sub = self.create_subscription(CameraInfo, "/camera/color/camera_info", self.logCamInfoCallback, 10)

        self.cv_bridge = CvBridge()

    def displayCamCallback(self, msg):

        raw_img = self.cv_bridge.imgmsg_to_cv2(msg)
        rgb_img = cv2.cvtColor(raw_img, cv2.COLOR_BGR2RGB)

        if raw_img is not None:
            cv2.imshow("Live Kinova Hand Camera", rgb_img)
            cv2.waitKey(1)
        
        else:
            self.get_logger().info("Image could not be loaded.")
    
    # def logCamInfoCallback(self, msg):

    #     try:
    #         # pass
    #         self.get_logger().info(f"Received Image of shape: ({msg.width}, {msg.height})")
        
    #     except Exception as e:
    #         self.get_logger().info(f"Error : {e}")

def main(args = None):
    rclpy.init(args=args)

    node = SubscribeCameraNode()

    try:
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()
    
    except Exception as e:
        node.get_logger().info(f"ERROR : {e}")

if __name__ == "__main__":
    main()

