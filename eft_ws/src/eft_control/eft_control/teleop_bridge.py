import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand

class TeleopBridge(Node):
    def __init__(self):
        super().__init__('teleop_bridge')

        # Configure QoS profile to match PX4's expectations
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers to PX4 uORB topics
        self.offboard_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Subscriber to cmd_vel (from teleop_twist_keyboard)
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Control state (NED coordinate frame: X-Forward, Y-Right/East, Z-Down/Descend)
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.yaw_rate = 0.0

        # Heartbeat timer (Offboard mode requires continuous publishing at > 2Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.offboard_started = False
        self.arm_count = 0

    def cmd_vel_callback(self, msg):
        # Map teleop keys to velocities:
        # Twist linear.x -> forward (pitch), linear.y -> left (roll), linear.z -> up (throttle)
        # PX4 uses NED coordinates: x -> Forward, y -> East/Right, z -> Down/Descend
        self.vx = msg.linear.x
        self.vy = -msg.linear.y  # Invert Y to map left (ROS) to East/Right (PX4)
        self.vz = -msg.linear.z  # Invert Z to map up (ROS) to Down (PX4)
        self.yaw_rate = -msg.angular.z # Invert Yaw to match turning directions

    def timer_callback(self):
        # 1. Publish Offboard Control Mode
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        offboard_msg.position = False
        offboard_msg.velocity = True
        offboard_msg.acceleration = False
        offboard_msg.attitude = False
        offboard_msg.body_rate = False
        self.offboard_mode_pub.publish(offboard_msg)

        # 2. Publish Trajectory Setpoint (Velocity Setpoint)
        setpoint_msg = TrajectorySetpoint()
        setpoint_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        setpoint_msg.position = [float('nan'), float('nan'), float('nan')]
        setpoint_msg.velocity = [self.vx, self.vy, self.vz]
        setpoint_msg.acceleration = [float('nan'), float('nan'), float('nan')]
        setpoint_msg.jerk = [float('nan'), float('nan'), float('nan')]
        setpoint_msg.yaw = float('nan')
        setpoint_msg.yawspeed = self.yaw_rate
        self.trajectory_pub.publish(setpoint_msg)

        # 3. Arm and engage Offboard mode once heartbeat is established
        if not self.offboard_started:
            self.arm_count += 1
            if self.arm_count == 10:
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0) # Set Offboard Mode
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0) # Arm
                self.get_logger().info("Dispatched Offboard Mode and Arming commands!")
                self.offboard_started = True

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
