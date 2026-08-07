import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleOdometry, VehicleStatus
import math

class WaypointController(Node):
    def __init__(self):
        super().__init__('waypoint_controller')

        # QoS Profiles matching PX4 requirements
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        qos_profile_sub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Publishers
        self.offboard_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Subscriber to monitor position
        self.odom_sub = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_callback, qos_profile_sub)

        # Subscriber to vehicle status
        self.status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v4', self.status_callback, qos_profile)

        # Drone State Variables (NED frame)
        self.current_pos = [0.0, 0.0, 0.0]
        self.vehicle_status = VehicleStatus()
        self.current_state = "INITIALIZING"
        self.state_counter = 0
        self.setpoint_pos = None
        self.max_vel = 0.5 # m/s (slow and smooth flight speed)

        # Waypoint configurations (NED: X->North, Y->East, Z->Down)
        self.takeoff_alt = -3.0 # 3 meters altitude (NED Z is negative)
        self.waypoint = [5.0, 5.0, -3.0] # 5 meters North, 5 meters East, 3 meters up

        # Setpoint timer (Offboard mode requires continuous publishing at > 2Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("Waypoint Flight Controller node initialized. Starting heartbeat...")

    def odom_callback(self, msg):
        self.current_pos = [msg.position[0], msg.position[1], msg.position[2]]

    def status_callback(self, msg):
        self.vehicle_status = msg

    def get_interpolated_setpoint(self, target, max_vel):
        dt = 0.1 # 10Hz timer
        step = max_vel * dt
        
        # Initialize setpoint to current position if it is None
        if self.setpoint_pos is None:
            if self.current_pos != [0.0, 0.0, 0.0]:
                self.setpoint_pos = list(self.current_pos)
            else:
                self.setpoint_pos = [0.0, 0.0, 0.0]

        dx = target[0] - self.setpoint_pos[0]
        dy = target[1] - self.setpoint_pos[1]
        dz = target[2] - self.setpoint_pos[2]
        dist = math.sqrt(dx**2 + dy**2 + dz**2)
        
        if dist > step:
            self.setpoint_pos[0] += (dx / dist) * step
            self.setpoint_pos[1] += (dy / dist) * step
            self.setpoint_pos[2] += (dz / dist) * step
        else:
            self.setpoint_pos = list(target)
            
        return self.setpoint_pos

    def timer_callback(self):
        # 1. Publish offboard control mode heartbeat
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = self.get_gps_time()
        offboard_msg.position = True
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        offboard_msg.attitude = False
        offboard_msg.body_rate = False
        self.offboard_mode_pub.publish(offboard_msg)

        # State Machine Logic
        self.state_counter += 1

        # Periodically log current coordinates and state (every 10 cycles / 1 second)
        if self.state_counter % 10 == 0:
            self.get_logger().info(
                f"[State: {self.current_state}] Pos: "
                f"[{self.current_pos[0]:.2f}, {self.current_pos[1]:.2f}, {self.current_pos[2]:.2f}] | "
                f"Armed: {self.vehicle_status.arming_state} (Target: {VehicleStatus.ARMING_STATE_ARMED}) | "
                f"Offboard: {self.vehicle_status.nav_state} (Target: {VehicleStatus.NAVIGATION_STATE_OFFBOARD})"
            )

        if self.current_state == "INITIALIZING":
            # Stream setpoints for ~2 seconds (20 cycles) before attempting to arm/switch mode
            self.publish_trajectory_setpoint(0.0, 0.0, 0.0)
            if self.state_counter >= 20:
                # Continuously request Offboard mode and Arming command every 1 second (10 cycles) until confirmed
                if self.state_counter % 10 == 0:
                    self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                    self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
                    self.get_logger().info("Sending Offboard Mode and Arming commands...")

                # Transition only when PX4 has successfully armed and entered offboard mode
                if (self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED and 
                        self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD):
                    self.get_logger().info("PX4 successfully armed and set to offboard mode!")
                    self.current_state = "TAKEOFF"
                    self.state_counter = 0
                    self.get_logger().info(f"Takeoff initiated! Rising to {abs(self.takeoff_alt)}m...")

        elif self.current_state == "TAKEOFF":
            # Command takeoff altitude smoothly
            target = [0.0, 0.0, self.takeoff_alt]
            sp = self.get_interpolated_setpoint(target, self.max_vel)
            self.publish_trajectory_setpoint(sp[0], sp[1], sp[2])

            # Check if takeoff altitude reached (within 0.25m)
            alt_error = abs(self.current_pos[2] - self.takeoff_alt)
            if alt_error < 0.25:
                self.current_state = "WAYPOINT"
                self.state_counter = 0
                self.get_logger().info(f"Takeoff altitude reached! Flying to waypoint: {self.waypoint}...")

        elif self.current_state == "WAYPOINT":
            # Command waypoint position smoothly
            sp = self.get_interpolated_setpoint(self.waypoint, self.max_vel)
            self.publish_trajectory_setpoint(sp[0], sp[1], sp[2])

            # Check distance to waypoint
            dist = math.sqrt(
                (self.current_pos[0] - self.waypoint[0])**2 +
                (self.current_pos[1] - self.waypoint[1])**2 +
                (self.current_pos[2] - self.waypoint[2])**2
            )
            if dist < 0.35:
                self.current_state = "LAND"
                self.state_counter = 0
                self.get_logger().info("Waypoint reached! Triggering landing sequence...")

        elif self.current_state == "LAND":
            # Send auto-land command
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            # Monitor landing progress (disarm when height is close to ground level)
            if abs(self.current_pos[2]) < 0.15:
                self.current_state = "LANDED"
                self.get_logger().info("Landed successfully! Disarming...")
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)

        elif self.current_state == "LANDED":
            # Stop sending active setpoints
            pass

    def publish_trajectory_setpoint(self, x, y, z, yaw=0.0):
        msg = TrajectorySetpoint()
        msg.timestamp = self.get_gps_time()
        msg.position = [float(x), float(y), float(z)]
        msg.velocity = [float('nan'), float('nan'), float('nan')]
        msg.acceleration = [float('nan'), float('nan'), float('nan')]
        msg.jerk = [float('nan'), float('nan'), float('nan')]
        msg.yaw = float(yaw)
        self.trajectory_pub.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.get_gps_time()
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_cmd_pub.publish(msg)

    def get_gps_time(self):
        return int(self.get_clock().now().nanoseconds / 1000)

def main(args=None):
    rclpy.init(args=args)
    node = WaypointController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
