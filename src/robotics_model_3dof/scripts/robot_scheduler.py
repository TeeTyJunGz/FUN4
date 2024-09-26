#!/usr/bin/python3
import rclpy

from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from robotic_interfaces.srv import Keyboard

class RobotSCHController(Node):
    def __init__(self):
        super().__init__('robot_scheduler')

        self.create_service(Keyboard, "robots_keyboard", self.keyboard_callback)

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
    
    def keyboard_callback(self, request: Keyboard, response: Keyboard):
        srv = request
        if srv.request :
            self.get_logger().info("IPK")

        # elif srv.mode == "Teleoperation":
        #     self.get_logger().info("Teleoperation")
        # elif srv.mode == "Auto":
        #     self.get_logger().info("Auto")
        
        return response
    
    def timer_callback(self):
            pass

            
def main(args=None):
    rclpy.init(args=args)
    node = RobotSCHController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
