# teleop_twist_keyboard_custom.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty

msg = """
Control your robot!
---------------------------
Moving around:
        w
   a    s    d
        x

i/, : increase/decrease z axis velocity

u/m : increase/decrease speed

CTRL-C to quit
"""

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
        super().__init__('teleop_keyboard')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.settings = termios.tcgetattr(sys.stdin)
        self.speed = 0.5  # Initial speed

    def run(self):
        twist = Twist()

        try:
            print(msg)
            while True:
                key = self.get_key()
                if key in move_bindings.keys():
                    vx, vy, vz = move_bindings[key]
                    twist.linear.x = vx * self.speed
                    twist.linear.y = vy * self.speed
                    twist.linear.z = vz * self.speed
                    self.publisher.publish(twist)

                elif key in speed_bindings.keys():
                    self.speed *= speed_bindings[key]
                    print(f"Current speed: {self.speed}")

                else:
                    twist.linear.x = 0.0
                    twist.linear.y = 0.0
                    twist.linear.z = 0.0
                    self.publisher.publish(twist)

        except Exception as e:
            print(e)

        finally:
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            self.publisher.publish(twist)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
