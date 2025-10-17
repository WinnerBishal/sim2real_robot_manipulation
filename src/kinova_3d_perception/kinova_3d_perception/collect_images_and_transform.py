import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
import os
import threading
import cv2
from cv_bridge import CvBridge
import yaml

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

IMAGE_TOPIC = '/camera/color/image_raw'
SAVE_FOLDER = os.path.join(os.path.expanduser('~'), 'ros2_images_and_transforms_checkerboard')

TO_FRAME = 'gen3_base_link'
FROM_FRAME = 'gen3_end_effector_link'

class SyncSaverServiceNode(Node):
    """
    A simple ROS2 node that continuously subscribes to an image topic.
    It runs a Service Server. When the service is called, it saves the most
    recently received image and looks up its corresponding transform from the
    TF2 buffer, then saves both to a folder as .png and .yaml respectively..
    """
    def __init__(self):
        super().__init__('collect_images_and_transform')
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.last_image_msg = None
        self.file_counter = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        if not os.path.exists(SAVE_FOLDER):
            os.makedirs(SAVE_FOLDER)
            self.get_logger().info(f"Created save directory: {SAVE_FOLDER}")

        self.image_subscription = self.create_subscription(
            Image, IMAGE_TOPIC, self.image_callback, 10)
            
        self.srv = self.create_service(Trigger, 'save_image_and_transform', self.trigger_save_callback)
        
        self.get_logger().info("Saver service node is ready.")
        self.get_logger().info(f"Call the '{self.get_name()}/save_image_and_transform' service with Trigger srv to save data.")

    def image_callback(self, msg):
        """Thread-safe method to store the latest image message."""
        with self.lock:
            self.last_image_msg = msg
        
    def trigger_save_callback(self, request, response):
        """
        This function is called when the 'save_image_and_transform' service is called.
        It saves the most recent image and its corresponding transform.
        """
        self.get_logger().info('Trigger received, attempting to save data...')

        with self.lock:
            local_image_msg = self.last_image_msg
        
        if local_image_msg is None:
            self.get_logger().error("Cannot save, no image has been received yet.")
            response.success = False
            response.message = "Image data not yet available."
            return response

        try:
            transform_stamped = self.tf_buffer.lookup_transform(
                TO_FRAME,
                FROM_FRAME,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1) 
            )
        except TransformException as ex:
            self.get_logger().error(f'Could not transform {FROM_FRAME} to {TO_FRAME}: {ex}')
            response.success = False
            response.message = f"TF lookup failed: {ex}"
            return response
        
        # --- Save the Image ---
        try:
            cv_image = self.bridge.imgmsg_to_cv2(local_image_msg, desired_encoding="bgr8")
            image_filename = os.path.join(SAVE_FOLDER, f"{self.file_counter}.png")
            cv2.imwrite(image_filename, cv_image)
        except Exception as e:
            self.get_logger().error(f"Failed to save image: {e}")
            response.success = False
            response.message = f"Failed to save image: {e}"
            return response

        # --- Save the Transform Data ---
        try:
            tf_filename = os.path.join(SAVE_FOLDER, f"{self.file_counter}.yaml")
            t = transform_stamped.transform
            transform_dict = {
                'header': {
                    'stamp': f"{transform_stamped.header.stamp.sec}.{transform_stamped.header.stamp.nanosec:09d}",
                    'frame_id': transform_stamped.header.frame_id
                },
                'child_frame_id': transform_stamped.child_frame_id,
                'transform': {
                    'translation': {'x': t.translation.x, 'y': t.translation.y, 'z': t.translation.z},
                    'rotation': {'x': t.rotation.x, 'y': t.rotation.y, 'z': t.rotation.z, 'w': t.rotation.w}
                }
            }
            with open(tf_filename, 'w') as f:
                yaml.dump(transform_dict, f, default_flow_style=False)
                
        except Exception as e:
            self.get_logger().error(f"Failed to save transform data: {e}")
            response.success = False
            response.message = f"Failed to save transform data: {e}"
            return response
            
        self.get_logger().info(f"Successfully saved data for frame {self.file_counter} to {SAVE_FOLDER}")
        response.success = True
        response.message = f"Successfully saved frame {self.file_counter}."
        
        self.file_counter += 1
        return response

def main(args=None):
    rclpy.init(args=args)
    node = SyncSaverServiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()