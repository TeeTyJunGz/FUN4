#!/usr/bin/python3
import rclpy

from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool
from geometry_msgs.msg import PoseStamped
from robotic_interfaces.srv import Keyboard, StateScheduler
from rcl_interfaces.msg import SetParametersResult

class RobotSCHController(Node):
    def __init__(self):
        super().__init__('robot_scheduler')

        self.create_service(Keyboard, "robots_keyboard", self.keyboard_callback)
        self.call_run_auto = self.create_client(StateScheduler, "state_sch")
        # self.call_kinematics_state = self.create_client(SetBool, "kinematics_Ready_State_Service")
        
        # self.call_random = self.create_client(SetBool, "rand_target")
        self.create_subscription(String, 'joy_mode', self.joy_mode_callback, 10)
        self.create_subscription(Bool, 'kinematics_Ready_State', self.kinematics_state_callback, 10)
        self.target_pub = self.create_publisher(PoseStamped, 'IPK_target', 10)

        self.declare_parameter('frequency', 100.0)
        self.declare_parameter('L_Base_F1', 0.2)
        self.declare_parameter('L_F2_F3', 0.25)
        self.declare_parameter('L_F3_Fe', 0.28)

        self.frequency = self.get_parameter('frequency').get_parameter_value().double_value
        self.Z_offset = self.get_parameter('L_Base_F1').get_parameter_value().double_value
        self.L1 = self.get_parameter('L_F2_F3').get_parameter_value().double_value
        self.L2 = self.get_parameter('L_F3_Fe').get_parameter_value().double_value

        self.joy = "Based"
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
    
    def joy_mode_callback(self, msg: String):
        self.joy = msg.data
        
    def kinematics_state_callback(self, msg: Bool):
        if msg.data:
            self.kinematics_state = True
        
    def call_state_function(self, state):
        srv = StateScheduler.Request()
        
        if state == "Teleop":
            srv.state = state + " " + self.joy
            
        elif state == "Auto":
            srv.state = state
                        
        future = self.call_run_auto.call_async(srv)
        # future.add_done_callback(self.handle_auto_target_response)
        # rclpy.spin_until_future_complete(self, future)
        
    # def handle_auto_target_response(self, future):
    #     try:
    #         response = future.result()
    #         if response:
    #             self.kinematics_state = response.success
    #             self.get_logger().info(f"Kine: {self.kinematics_state}")

    #         else:
    #             self.get_logger().error("Received an empty response from the service.")
    #     except Exception as e:
    #         self.get_logger().error(f"Service call failed with error: {str(e)}")
            
    def pub_target(self, arr):
        msg = PoseStamped()
        msg.pose.position.x = arr[0]
        msg.pose.position.y = arr[1]
        msg.pose.position.z = arr[2]
        self.target_pub.publish(msg)
        
    def timer_callback(self):
        if self.mode == "Initial":
            pass
        
        elif self.mode == "IPK" and self.kinematics_state and self.IPK:
            
            self.pub_target(self.IPK_target)
            self.kinematics_state = False
            self.IPK = False
                        
        elif self.mode == "Teleop":
            self.call_state_function("Teleop")
                        
        elif self.mode == "Auto" and self.kinematics_state:
            
            self.call_state_function("Auto")
            
            # srv = SetBool.Request()
            # srv.data = True
            # future = self.call_kinematics_state.call_async(srv)
            
            # future.add_done_callback(self.handle_random_target_response)
            
            self.kinematics_state = False
            

            
def main(args=None):
    rclpy.init(args=args)
    node = RobotSCHController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
