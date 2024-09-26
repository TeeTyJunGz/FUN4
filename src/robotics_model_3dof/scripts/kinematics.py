#!/usr/bin/python3
import rclpy
import numpy as np
import roboticstoolbox as rtb

from math import pi
from rclpy.node import Node
from spatialmath import SE3
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped
from rcl_interfaces.msg import SetParametersResult

L1 = 0.200
L2 = 0.120
L3 = 0.100
L4 = 0.250
L5 = 0.280

robot = rtb.DHRobot(
    [
        rtb.RevoluteMDH(alpha = 0.0     ,a = 0.0      ,d = L1     ,offset = 0.0),
        rtb.RevoluteMDH(alpha = -pi/2   ,a = 0.0      ,d = -L2    ,offset = -pi/2),
        rtb.RevoluteMDH(alpha = 0.0     ,a = L4       ,d = L3     ,offset = 0.0),
    ],tool = SE3([
    [0, 0, 1, L5],
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 0, 1]]),
    name = "3DOF_Robot"
)


class Kinematics(Node):
    def __init__(self):
        super().__init__('kinematics_calculator')
        
        self.create_subscription(PoseStamped, 'target', self.target_callback, 10)

        self.q_pub = self.create_publisher(JointState, "/q_velocities", 10)
        
        self.declare_parameter('frequency', 100.0)
        self.declare_parameter('Kp', 10.0)

        self.frequency = self.get_parameter('frequency').get_parameter_value().double_value
        self.Kp = self.get_parameter('Kp').get_parameter_value().double_value
        
        self.target = np.array([0.0, 0.0, 0.0])
        self.target_rc = False
        self.q = [0.0, 0.0, 0.0]
        
        self.q_velocities = JointState()
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.target_frame = "end_effector"
        self.source_frame = "link_0"

        self.create_timer(1/self.frequency, self.timer_callback)
        self.add_on_set_parameters_callback(self.set_param_callback)
    
    def set_param_callback(self, params):
        for param in params:
            if param.name == 'frequency':
                self.get_logger().info(f'Updated frequency: {param.value}')
                self.frequency = param.value
            elif param.name == 'Kp':
                self.get_logger().info(f'Updated Kp: {param.value}')
                self.Kp = param.value
            else:
                self.get_logger().warn(f'Unknown parameter: {param.name}')
                # Return failure result for unknown parameters
                return SetParametersResult(successful=False, reason=f'Unknown parameter: {param.name}')
        # If all parameters are known, return success
        return SetParametersResult(successful=True)
    
    def target_callback(self, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z

        self.target = np.array([x, y, z])
        self.target_rc = True
        
    def timer_callback(self):
        try:
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.source_frame,
                rclpy.time.Time()
            )
            translation = transform.transform.translation
            self.current = np.array([translation.x, translation.y, translation.z])
            # self.get_logger().info(f"Position: x={self.translation.x}, y={self.translation.y}, z={self.translation.z}")

        except:
            pass
        
        if self.target_rc:
            current_pose = robot.fkine(self.q).t[:3]
                    
            error = self.target - current_pose
            v_end_effector = self.Kp * error
            
            J_full = robot.jacob0(self.q)
            J_translational = J_full[:3, :3]  # 3x3 matrix

            q_dot = np.linalg.pinv(J_translational).dot(v_end_effector)

            # print(f"Current: {current_pose}")
            # print(f"targets: {self.target}")
            # print(f"Errored: {error}")

            # print("Joint velocities: ", q_dot)
            
            self.q_velocities.velocity = [q_dot[0], q_dot[1], q_dot[2]]
            
            self.q = self.q + q_dot * 1/self.frequency
            
            if np.linalg.norm(error) < 1e-3:
                self.q_velocities.velocity = [0.0, 0.0, 0.0]
                self.target_rc = False
                print("Target pose reached!")
                
            self.q_pub.publish(self.q_velocities)
            
def main(args=None):
    rclpy.init(args=args)
    node = Kinematics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
