#!/usr/bin/python3
import rclpy
import sys, select, termios, tty

from rclpy.node import Node
from geometry_msgs.msg import Twist
from robotic_interfaces.srv import Keyboard

move_bindings = {
    'w': (1, 0, 0),
    's': (-1, 0, 0),
    'a': (0, 1, 0),
    'd': (0, -1, 0),
    'i': (0, 0, 1),
    ',': (0, 0, -1),
}

speed_bindings = {
    'u': 1.1,  # Increase speed
    'm': 0.9,  # Decrease speed
}

class TeleopNode(Node):
    def __init__(self): 
        super().__init__('teleop_twist_keyboard')
        self.keyboard_call = self.create_client(Keyboard, "robotKeyboard")
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.settings = termios.tcgetattr(sys.stdin)
        self.speed = 0.1
        self.key_timeout = 0.1
        self.state = "Auto"
        self.control_j = "Based"
        self.inv_state = "Teleop Based"
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.workspace = False
        
        srv = Keyboard.Request()
        srv.mode = "Teleop Based"
        self.keyboard_call.call_async(srv)
        
    def run(self):
        twist = Twist()
        try:
            while True:
                msg = f"""
                
            Control your robot!
            ---------------------------
            Moving around:
                 w        |         -x
            a    s    d   |   +y    +x    -y
                
            i/, : increase/decrease z axis velocity

            Current Speed: {self.speed}
                u/m : increase/decrease speed

            Mode: {self.inv_state}
                c : Swap mode (Teleoperation (Defualt) | Auto)
                b : Teleoperation Based Controlled (Defualt)
                e : Teleoperation End Effector Controlled

            Press I (UPPERCASE) to input Inverse Pose Kinematics Position
            Inverse Pose Kinematics Position x: {round(self.x, 2)} y: {round(self.y, 2)} z: {round(self.z, 2)}

            CTRL-C to quit
                """
                
                print(msg)
                key = self.get_key()
                if key in move_bindings.keys():
                    vx, vy, vz = move_bindings[key]
                    twist.linear.x = vx * self.speed
                    twist.linear.y = vy * self.speed
                    twist.linear.z = vz * self.speed
                    self.publisher.publish(twist)

                elif key in speed_bindings.keys():
                    self.speed *= speed_bindings[key]

                elif key == '':
                    twist.linear.x = 0.0
                    twist.linear.y = 0.0
                    twist.linear.z = 0.0
                    self.publisher.publish(twist)
                    
                elif key == 'e':
                    if self.state != "Teleop" + " " + self.control_j:
                        self.control_j = "End Effector"
                        
                        srv = Keyboard.Request()
                        srv.mode = "Teleop End Effector"
                        self.keyboard_call.call_async(srv)
                        
                        
                        
                        self.inv_state = "Teleop" + " " + self.control_j
                        # print(f"Change Teleop: {srv.mode}")

                elif key == 'b':
                    if self.state != "Teleop" + " " + self.control_j:
                        srv.mode = "Teleop Based"
                        
                        srv = Keyboard.Request()
                        self.control_j = "Based"
                        self.keyboard_call.call_async(srv)
                        
                        self.inv_state = "Teleop" + " " + self.control_j
                        # print(f"Change Teleop: {srv.mode}")
                
                elif key == 'c':
                    if self.state == "Auto":
                        srv = Keyboard.Request()
                        srv.mode = "Auto"
                        future = self.keyboard_call.call_async(srv)
                        # future.add_done_callback(self.handle_random_target_response)
                        
                        self.inv_state = "Autonomous"
                        self.state = "Teleop" + " " + self.control_j
                        # print(f"Current Mode: {srv.mode}")
                        
                    else:
                        srv = Keyboard.Request()
                        srv.mode = self.state
                        future = self.keyboard_call.call_async(srv)
                        # future.add_done_callback(self.handle_random_target_response)
                        
                        self.inv_state = self.state
                        self.state = "Auto"
                        # print(f"Current Mode: {srv.mode}")
                elif key == 'I':
                    self.x = float(input("Please enter the value for x: "))
                    self.y = float(input("Please enter the value for y: "))
                    self.z = float(input("Please enter the value for z: "))
                    self.inv_state = "Inverse Pose Kinematics (IPK) |Press c to change|"
                    self.state = "Teleop " + self.control_j
                
                    srv = Keyboard.Request()
                    srv.mode = "IPK"
                    srv.x = self.x
                    srv.y = self.y
                    srv.z = self.z
                    future = self.keyboard_call.call_async(srv)
                    # future.add_done_callback(self.handle_random_target_response)
                    

                else:
                    if key == '\x03':
                        break

        except Exception as e:
            print(e)

        finally:
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            self.publisher.publish(twist)
            self.cleanup()
            
    def handle_random_target_response(self, future):
        try:
            response = future.result()
            if response:
                self.workspace = response.success
            else:
                print("Received an empty response from the service.")
        except Exception as e:
            print(f"Service call failed with error: {str(e)}")
    
    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], self.key_timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def cleanup(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        print("\nShutting down gracefully...")

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()

    try:
        node.run()
    except KeyboardInterrupt:
        node.cleanup()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
