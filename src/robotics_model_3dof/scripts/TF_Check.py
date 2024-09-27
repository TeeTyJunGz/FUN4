#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

class TF2Echo(Node):
    def __init__(self, source_frame, target_frame):
        super().__init__('tf2_echo_node')
        self.source_frame = source_frame
        self.target_frame = target_frame

        self.eff_pub = self.create_publisher(PoseStamped, "tf2_end_effector", 10)
        
        self.declare_parameter('frequency', 100.0)
        
        self.frequency = self.get_parameter('frequency').get_parameter_value().double_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_timer(1/self.frequency, self.timer_callback)

    def timer_callback(self):
        try:
            # Wait for the transform with a timeout (2 seconds)
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(self.target_frame, self.source_frame, now, rclpy.duration.Duration(seconds=2))

            self.end_effector_pub(trans)
        except (LookupException, ConnectivityException, ExtrapolationException) as ex:
            self.get_logger().warn(f"Could not transform {self.source_frame} to {self.target_frame}: {ex}")
        except Exception as ex:
            self.get_logger().error(f"Unexpected error: {ex}")

    def end_effector_pub(self, trans: TransformStamped):
        # self.get_logger().info(f"Translation: {trans.transform.translation}")
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = trans.transform.translation.x
        msg.pose.position.y = trans.transform.translation.y
        msg.pose.position.z = trans.transform.translation.z
        self.eff_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    node = TF2Echo("link_0", "end_effector")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    import sys
    main(sys.argv)
