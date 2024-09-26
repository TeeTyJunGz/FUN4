#!/usr/bin/python3
import rclpy
import random
import numpy as np
import roboticstoolbox as rtb

from math import pi
from rclpy.node import Node
from spatialmath import SE3
from std_srvs.srv import SetBool
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped, Vector3
from rcl_interfaces.msg import SetParametersResult
from robotic_interfaces.srv import Keyboard

class Keyboard(Node):
    def __init__(self):
        super().__init__('robot_keyboard')

        self.keyboard_srv = self.create_client(Keyboard, "robots_keyboard")

        self.declare_parameter('frequency', 100.0)

        self.frequency = self.get_parameter('frequency').get_parameter_value().double_value

        self.create_timer(1/self.frequency, self.timer_callback)
        self.add_on_set_parameters_callback(self.set_param_callback)
    
    def set_param_callback(self, params):
        for param in params:
            if param.name == 'frequency':
                self.get_logger().info(f'Updated frequency: {param.value}')
                self.frequency = param.value
            # elif param.name == 'L_Base_F1':
            #     self.get_logger().info(f'Updated max angle: {param.value}')
            #     self.Z_offset = param.value
            # elif param.name == 'L_F2_F3':
            #     self.get_logger().info(f'Updated max speed: {param.value}')
            #     self.L1 = param.value
            # elif param.name == 'L_F3_Fe':
            #     self.get_logger().info(f'Updated max speed: {param.value}')
            #     self.L2 = param.value
            else:
                self.get_logger().warn(f'Unknown parameter: {param.name}')
                # Return failure result for unknown parameters
                return SetParametersResult(successful=False, reason=f'Unknown parameter: {param.name}')
        # If all parameters are known, return success
        return SetParametersResult(successful=True)
    
    def keyboard_call(self):
        srv = Keyboard.Request()
        srv.mode = "IPK"
        
        self.keyboard_srv.call_async(srv)
    
    def timer_callback(self):
            pass

            
def main(args=None):
    rclpy.init(args=args)
    node = Keyboard()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
