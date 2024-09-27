#!/usr/bin/python3
import numpy as np
import rclpy

from math import pi
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rcl_interfaces.msg import SetParametersResult

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.create_subscription(JointState, "q_velocities", self.q_velocities_callback, 10)
        self.declare_parameter('frequency', 100.0)

        self.frequency = self.get_parameter('frequency').get_parameter_value().double_value
        
        self.q_velocities = [0.0, 0.0, 0.0]
        
        self.joint_state_msg = JointState()
        self.joint_state_msg.name = ['joint_1', 'joint_2', 'joint_3']
        self.joint_state_msg.position = [0.0, 0.0, 0.0]
        self.joint_state_msg.velocity = [0.0, 0.0, 0.0]

        self.position_limits = [2 * pi, 2 * pi, 2 * pi]
        # self.joint_limits_lower = [-pi/2, -pi/2, -pi/2]
        # self.joint_limits_upper = [pi/2, pi/2, pi/2]

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
        
    def q_velocities_callback(self, msg: JointState):
        self.q_velocities = msg.velocity
    
    def timer_callback(self):
            
        self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()

        self.joint_state_msg.velocity = [self.q_velocities[0], self.q_velocities[1], self.q_velocities[2]]
        
        for i in range(len(self.joint_state_msg.name)):
                
            self.joint_state_msg.position[i] += self.joint_state_msg.velocity[i] * 1/self.frequency
            # self.joint_state_msg.position[i] = np.clip(self.joint_state_msg.position[i], self.joint_limits_lower[i], self.joint_limits_upper[i])
            self.joint_state_msg.position[i] %= self.position_limits[i]
            
        self.joint_pub.publish(self.joint_state_msg)

            
def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
