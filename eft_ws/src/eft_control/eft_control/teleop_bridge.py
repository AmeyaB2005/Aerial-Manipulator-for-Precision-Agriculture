"""
teleop_bridge: /cmd_vel (ROS FLU) -> PX4 offboard setpoints (NED) over uXRCE-DDS

/cmd_vel conventions:
    linear.x  forward +, linear.y left +, linear.z up + (climb rate, m/s)
    angular.z CCW +
    angular.x special commands:
         2.0  joystick ARM     -> OFFBOARD + ARM, climb to takeoff_height
        -2.0  joystick DISARM  -> DISARM
         1.0  keyboard takeoff (kept for keyboard_teleop)
        -1.0  keyboard land    (kept for keyboard_teleop)
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand

DT = 0.1            # heartbeat period (10 Hz)
CMD_TIMEOUT = 0.3   # s without /cmd_vel -> zero velocities


class TeleopBridge(Node):
    def __init__(self):
        super().__init__('teleop_bridge')

        self.declare_parameter('auto_arm', False)
        self.declare_parameter('takeoff_height', 1.5)
        self.declare_parameter('max_height', 30.0)
        self.auto_arm = bool(self.get_parameter('auto_arm').value)
        self.takeoff_height = float(self.get_parameter('takeoff_height').value)
        self.max_height = float(self.get_parameter('max_height').value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.vehicle_cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos)

        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0          # ROS up-positive climb rate
        self.yaw_rate = 0.0
        self.target_z = 0.0    # NED: negative = up
        self.last_cmd_time = self.get_clock().now()

        self.tick = 0
        self.auto_armed = False
        self.timer = self.create_timer(DT, self.timer_callback)

        self.get_logger().info(
            f"Teleop bridge up (auto_arm={self.auto_arm}). Streaming offboard heartbeat at 10 Hz.")

    # ------------------------------------------------------------------
    def cmd_vel_callback(self, msg):
        self.last_cmd_time = self.get_clock().now()

        cmd = msg.angular.x
        if math.isclose(cmd, 2.0) or math.isclose(cmd, 1.0):
            self.arm_offboard()
            if self.target_z >= 0.0 or math.isclose(cmd, 2.0):
                self.target_z = min(self.target_z, -self.takeoff_height)
            else:
                self.target_z -= 0.5  # keyboard: incremental takeoff
            self.get_logger().info(f"ARM/takeoff: target Z = {self.target_z:.2f} m")
            return
        if math.isclose(cmd, -2.0):
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
            self.target_z = 0.0
            self.vx = self.vy = self.vz = self.yaw_rate = 0.0
            self.get_logger().info("DISARM sent")
            return
        if math.isclose(cmd, -1.0):
            self.target_z = 0.0
            self.get_logger().info("Land: target Z = 0.0 m")
            return

        self.vx = msg.linear.x
        self.vy = -msg.linear.y         # ROS left+ -> NED east+
        self.vz = msg.linear.z
        self.yaw_rate = -msg.angular.z  # ROS CCW+ -> NED CW+

    # ------------------------------------------------------------------
    def timer_callback(self):
        now = self.get_clock().now()
        stamp = int(now.nanoseconds / 1000)

        if (now - self.last_cmd_time).nanoseconds / 1e9 > CMD_TIMEOUT:
            self.vx = self.vy = self.vz = self.yaw_rate = 0.0

        # Integrate climb rate into the altitude setpoint
        self.target_z -= self.vz * DT
        self.target_z = max(-self.max_height, min(0.0, self.target_z))

        mode = OffboardControlMode()
        mode.timestamp = stamp
        mode.position = True
        mode.velocity = True
        mode.acceleration = False
        mode.attitude = False
        mode.body_rate = False
        self.offboard_mode_pub.publish(mode)

        sp = TrajectorySetpoint()
        sp.timestamp = stamp
        nan = float('nan')
        sp.position = [nan, nan, float(self.target_z)]
        sp.velocity = [float(self.vx), float(self.vy), nan]
        sp.acceleration = [nan, nan, nan]
        sp.jerk = [nan, nan, nan]
        sp.yaw = nan
        sp.yawspeed = float(self.yaw_rate)
        self.trajectory_pub.publish(sp)

        if self.auto_arm and not self.auto_armed:
            self.tick += 1
            if self.tick >= 10:
                self.arm_offboard()
                self.auto_armed = True
                self.get_logger().info("auto_arm: OFFBOARD + ARM sent")

    # ------------------------------------------------------------------
    def arm_offboard(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.param1 = float(param1)
        msg.param2 = float(param2)
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
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
