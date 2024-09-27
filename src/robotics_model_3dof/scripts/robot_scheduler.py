#!/usr/bin/python3
import rclpy

from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from geometry_msgs.msg import PoseStamped
from robotic_interfaces.srv import Keyboard
from rcl_interfaces.msg import SetParametersResult

class RobotSCHController(Node):
    def __init__(self):
        super().__init__('robot_scheduler')

        self.create_service(Keyboard, "robots_keyboard", self.keyboard_callback)
        self.call_run_auto = self.create_client(SetBool, "auto")
        # self.call_random = self.create_client(SetBool, "rand_target")
        
        self.create_subscription(Bool, 'kinematics_Ready_State', self.kinematics_state_callback, 10)
        self.target_pub = self.create_publisher(PoseStamped, 'target', 10)

        self.declare_parameter('frequency', 100.0)
        self.declare_parameter('L_Base_F1', 0.2)
        self.declare_parameter('L_F2_F3', 0.25)
        self.declare_parameter('L_F3_Fe', 0.28)

        self.frequency = self.get_parameter('frequency').get_parameter_value().double_value
        self.Z_offset = self.get_parameter('L_Base_F1').get_parameter_value().double_value
        self.L1 = self.get_parameter('L_F2_F3').get_parameter_value().double_value
        self.L2 = self.get_parameter('L_F3_Fe').get_parameter_value().double_value

        self.mode = "Initial"
        self.kinematics_state = True
        self.IPK = False
        self.IPK_target = [0.0, 0.0, 0.0]

        self.create_timer(1/self.frequency, self.timer_callback)
        self.add_on_set_parameters_callback(self.set_param_callback)
    
    def set_param_callback(self, params):
        for param in params:
            if param.name == 'frequency':
                self.get_logger().info(f'Updated frequency: {param.value}')
                self.frequency = param.value
            else:
                self.get_logger().warn(f'Unknown parameter: {param.name}')
                # Return failure result for unknown parameters
                return SetParametersResult(successful=False, reason=f'Unknown parameter: {param.name}')
        # If all parameters are known, return success
        return SetParametersResult(successful=True)
    
    def keyboard_callback(self, request: Keyboard, response: Keyboard):
        srv = request
        if srv.mode == "IPK" :
            self.get_logger().info("Inverse Pose Kinematics Mode")
            
            self.IPK = True
            self.mode = srv.mode
            
            r_max = self.L1 + self.L2
            r_min = abs(self.L1 - self.L2)
            
            distance_squared = srv.x**2 + srv.y**2 + (srv.z - self.Z_offset)**2
            
            self.get_logger().info(f"Target X: {srv.x}")
            self.get_logger().info(f"Target Y: {srv.y}")
            self.get_logger().info(f"Target Z: {srv.z}")
            
            response.message = "Change mode to Inverse Pose Kinematics (IPK) successfully"

            if r_min**2 < distance_squared < r_max**2:
                self.get_logger().info(f"Target is in robot workspace!")
                response.workspace = "Target is in robot workspace!"
                
                self.IPK_target = [srv.x, srv.y, srv.z]
                response.success = True
                
            else:
                self.get_logger().warn(f"Target is not in robot workspace!")
                response.workspace = "Target is not in robot workspace!"

                response.success = False
            
        elif srv.mode == "Teleop":
            self.get_logger().info("Tele-operation Mode")
            
            self.mode = srv.mode
            
            response.message = "Change mode to Tele-operation (Teleop) successfully"
            response.success = True
            
        elif srv.mode == "Auto":
            self.get_logger().info("Autonomous Mode")
            
            self.mode = srv.mode
                        
            response.message = "Change mode to Autonomous (Auto) successfully"            
            response.success = True
        else:
            response.message = "Unknown Mode, try use IPK, Teleop, Auto"            
            response.success = False
            
        return response
    
    def kinematics_state_callback(self, msg: Bool):
        if msg.data:
            self.kinematics_state = True
        # else:
        #     self.kinematics_state = False
        
    def call_auto_function(self, boolean):
        srv = SetBool.Request()
        srv.data = boolean
        
        self.call_run_auto.call_async(srv)
        # rclpy.spin_until_future_complete(self, future)
        
    def pub_target(self, arr):
        msg = PoseStamped()
        msg.pose.position.x = arr[0]
        msg.pose.position.y = arr[1]
        msg.pose.position.z = arr[2]
        self.target_pub.publish(msg)
        
    def timer_callback(self):
        if self.mode == "IPK" and self.kinematics_state and self.IPK:
            
            self.pub_target(self.IPK_target)
            self.kinematics_state = False
            self.IPK = False
                        
        elif self.mode == "Auto" and self.kinematics_state:
            
            self.call_auto_function(True)
            self.kinematics_state = False
            
def main(args=None):
    rclpy.init(args=args)
    node = RobotSCHController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
