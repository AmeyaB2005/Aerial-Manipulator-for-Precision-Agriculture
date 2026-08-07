import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty

# Help message
msg = """
Control Your Drone (EFT E610P)!
---------------------------
Moving around:
        w
   a    s    d

q/e : Turn Left (Yaw CCW) / Turn Right (Yaw CW)
r/f : Move Up (Climb) / Move Down (Descend)

t : Autonomous Takeoff
l : Autonomous Landing

space or x : Force Stop (Zero velocity)

CTRL-C to quit
"""

# Key mappings: [linear_x, linear_y, linear_z, angular_z]
moveBindings = {
    'w': (1.0, 0.0, 0.0, 0.0),
    's': (-1.0, 0.0, 0.0, 0.0),
    'a': (0.0, 1.0, 0.0, 0.0),
    'd': (0.0, -1.0, 0.0, 0.0),
    'r': (0.0, 0.0, 1.0, 0.0),
    'f': (0.0, 0.0, -1.0, 0.0),
    'q': (0.0, 0.0, 0.0, 1.0),
    'e': (0.0, 0.0, 0.0, -1.0),
    'x': (0.0, 0.0, 0.0, 0.0),
    ' ': (0.0, 0.0, 0.0, 0.0),
}

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.settings = termios.tcgetattr(sys.stdin)
        self.speed = 1.0 # m/s
        self.turn = 1.0  # rad/s

        self.timer = self.create_timer(0.1, self.run_loop)
        print(msg)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run_loop(self):
        key = self.getKey()
        if key:
            twist = Twist()
            if key in moveBindings.keys():
                x, y, z, th = moveBindings[key]
                twist.linear.x = x * self.speed
                twist.linear.y = y * self.speed
                twist.linear.z = z * self.speed
                twist.angular.z = th * self.turn
                self.pub.publish(twist)
                self.get_logger().info(f"Published keypress '{key}': vx={twist.linear.x}, vy={twist.linear.y}, vz={twist.linear.z}, yaw={twist.angular.z}")
            elif key == 't':
                twist.angular.x = 1.0
                self.pub.publish(twist)
                self.get_logger().info("Published Takeoff trigger ('t')")
            elif key == 'l':
                twist.angular.x = -1.0
                self.pub.publish(twist)
                self.get_logger().info("Published Landing trigger ('l')")
            elif key == '\x03': # CTRL-C
                self.get_logger().info("Exiting keyboard control...")
                self.destroy_node()
                sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)
        rclpy.shutdown()

if __name__ == '__main__':
    main()
