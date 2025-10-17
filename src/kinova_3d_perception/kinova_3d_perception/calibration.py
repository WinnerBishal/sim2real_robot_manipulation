import rclpy
from rclpy.node import Node

import torch
import transformers
from PIL import Image, ImageDraw

from sensor_msgs.msg import Image as ImageMsg
from sensor_msgs.msg import CameraInfo

class DepthProcessorNode(Node):
    def __init__(self):
        super().__init__("calibration")

        self.collecting_images = True


def main(args = None):
    rclpy.init(args=args)
    node = DepthProcessorNode()

    try:
        rclpy.spin(node)
        
    except Exception as e:
        print(f"{e}")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
