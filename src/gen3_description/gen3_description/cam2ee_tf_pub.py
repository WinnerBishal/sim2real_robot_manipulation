import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class StaticTransformNode(Node):
    def __init__(self):
        super().__init__("cam2ee_tf_pub")

        self.cam2ee_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(0.1, self.broadcastCallback)

        self.get_logger().info("Started cam2ee broadcaster")

    def broadcastCallback(self):

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()

        t.header.frame_id = "gen3_end_effector_link"
        t.child_frame_id = "camera_link"

#         [-0.55223736  0.45715441 -0.14907519  0.68104356]
# [[ 0.05592244]
#  [-0.00453678]
#  [ 0.07754135]]
# [ 0.235462   -0.22987059 -0.20560648  0.92165239]
# [[ 0.02059796]
#  [-0.14139705]
#  [ 0.40595394]]

# [ 0.07326283 -0.0576429  -0.08719902  0.99181963]
# [[0.40953281]
#  [0.04550084]
#  [0.06003826]]
        t.transform.translation.x = 0.40953281
        t.transform.translation.y = 0.04550084
        t.transform.translation.z = 0.06003826

        t.transform.rotation.x = 0.07326283
        t.transform.rotation.y = -0.0576429
        t.transform.rotation.z = -0.08719902
        t.transform.rotation.w = 0.99181963

        # t.transform.translation.x = 0.05592244
        # t.transform.translation.y = -0.00453678
        # t.transform.translation.z = 0.07754135

        # t.transform.rotation.x = -0.55223736
        # t.transform.rotation.y = 0.45715441
        # t.transform.rotation.z = -0.14907519
        # t.transform.rotation.w = 0.68104356

        # t.transform.translation.x = 0.02059796
        # t.transform.translation.y = -0.14139705
        # t.transform.translation.z = 0.40595394

        # t.transform.rotation.x = 0.235462
        # t.transform.rotation.y = -0.22987059
        # t.transform.rotation.z = -0.20560648
        # t.transform.rotation.w = 0.92165239

        self.cam2ee_broadcaster.sendTransform(t)

def main(args=None):

    rclpy.init(args=args)
    node = StaticTransformNode()
    try:
        rclpy.spin(node)
    
    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()