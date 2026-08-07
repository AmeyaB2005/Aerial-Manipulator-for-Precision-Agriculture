import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoystickTeleop(Node):
    def __init__(self):
        super().__init__('joystick_teleop')

        # Subscribe to standard ROS 2 joy topic
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        # Publisher for cmd_vel
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Scaling parameters for velocity commands
        self.declare_parameter('scale_linear_x', 2.0)
        self.declare_parameter('scale_linear_y', 2.0)
        self.declare_parameter('scale_linear_z', 2.0)
        self.declare_parameter('scale_angular_z', 1.5)

        self.scale_x = self.get_parameter('scale_linear_x').get_parameter_value().double_value
        self.scale_y = self.get_parameter('scale_linear_y').get_parameter_value().double_value
        self.scale_z = self.get_parameter('scale_linear_z').get_parameter_value().double_value
        self.scale_az = self.get_parameter('scale_angular_z').get_parameter_value().double_value

        self.get_logger().info("Joystick Teleoperation Node Initialized! Listening to /joy...")

    def joy_callback(self, msg):
        twist = Twist()

        # Typical Xbox/DualShock mappings:
        # msg.axes[1] -> Left Stick Vertical (Forward/Backward)
        # msg.axes[0] -> Left Stick Horizontal (Left/Right)
        # msg.axes[4] -> Right Stick Vertical (Climb/Descend)
        # msg.axes[3] -> Right Stick Horizontal (Yaw CCW/CW)
        
        # Ensure we don't index out of bounds if controller has fewer axes
        if len(msg.axes) >= 2:
            twist.linear.x = msg.axes[1] * self.scale_x
            twist.linear.y = msg.axes[0] * self.scale_y
        if len(msg.axes) >= 5:
            twist.linear.z = msg.axes[4] * self.scale_z
        if len(msg.axes) >= 4:
            twist.angular.z = msg.axes[3] * self.scale_az

        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = JoystickTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
