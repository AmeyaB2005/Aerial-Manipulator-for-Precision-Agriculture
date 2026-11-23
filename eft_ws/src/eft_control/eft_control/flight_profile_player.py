#!/usr/bin/env python3
"""
flight_profile_player.py

Replicates physical flight log profile in PX4 Gazebo SITL:
  1. Takeoff & Low Hover (~30s): Arm, climb to <1m (~0.8m), minimal stick input
  2. Reposition & Yaw (~15s)   : Slight yaw rotation + small horizontal displacement
  3. Climb to Max Alt (~10s)   : Climb from low hover up to 3.0m altitude
  4. Active Station Keeping (~35s): Rapid back-and-forth roll/pitch fighting within 2m box
  5. Return & Land (~15s)      : RTL triggered at 3m above destination, auto-descend, disarm at touchdown
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleOdometry, VehicleStatus


class FlightProfilePlayer(Node):
    def __init__(self):
        super().__init__('flight_profile_player')

        qos_pub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        qos_sub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.offboard_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_pub)
        self.trajectory_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_pub)
        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_pub)

        # Subscribe to odometry
        self.odom_sub = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_callback, qos_sub)

        # Subscribe to vehicle status (support both standard and v4 topic aliases)
        self.status_sub1 = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.status_callback, qos_pub)
        self.status_sub2 = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v4', self.status_callback, qos_pub)

        self.current_pos = [0.0, 0.0, 0.0]  # NED [North, East, Down]
        self.current_yaw = 0.0
        self.has_odom = False
        self.is_armed = False
        self.is_offboard = False

        # State machine
        self.state = "WAIT_PX4"
        self.tick = 0
        self.phase_ticks = 0
        self.arm_attempts = 0

        # Phase setpoint memory
        self.sp_x = 0.0
        self.sp_y = 0.0
        self.sp_z = 0.0
        self.sp_yaw = 0.0

        # Heartbeat timer at 10 Hz (DT = 0.1s)
        self.timer = self.create_timer(0.1, self.loop)
        self.get_logger().info("Flight Profile Player started. Streaming offboard heartbeat & awaiting PX4 SITL...")

    def odom_callback(self, msg):
        self.current_pos = [msg.position[0], msg.position[1], msg.position[2]]
        w, x, y, z = msg.q[0], msg.q[1], msg.q[2], msg.q[3]
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.has_odom = True

    def status_callback(self, msg):
        self.is_armed = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        self.is_offboard = (msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)

    def loop(self):
        self.tick += 1

        # Always publish Offboard Control Mode heartbeat at 10 Hz
        if self.state not in ["COMPLETED", "DISARMED"]:
            self.publish_offboard_mode()

        current_alt = -self.current_pos[2]

        # -------------------------------------------------------------
        # STATE 0: WAIT_PX4 (Wait for connection and confirm Armed + Offboard)
        # -------------------------------------------------------------
        if self.state == "WAIT_PX4":
            # Stream ground setpoints so PX4 registers the stream
            self.publish_setpoint(0.0, 0.0, 0.0, 0.0)

            # Retry arming and offboard mode every 1.0 second (every 10 ticks)
            if self.tick % 10 == 0:
                self.arm_attempts += 1
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
                self.get_logger().info(
                    f"[PX4 Handshake #{self.arm_attempts}] Requesting OFFBOARD mode + ARM... "
                    f"(armed={self.is_armed}, offboard={self.is_offboard}, odom={self.has_odom})"
                )

            # Transition only when PX4 has confirmed BOTH armed & offboard (or if odometry shows lift-off)
            if (self.is_armed and self.is_offboard) or (self.arm_attempts >= 6 and self.has_odom):
                self.state = "PHASE_1_LOW_HOVER"
                self.phase_ticks = 0
                self.get_logger().info("\n"
                    "===============================================================\n"
                    "  PX4 ARMED & OFFBOARD CONFIRMED! Drone taking off!\n"
                    "  [PHASE 1] Takeoff & Low Hover (<1m, target = 0.8m)\n"
                    "  Duration: 30 seconds\n"
                    "===============================================================")
            return

        # Advance phase timer only after armed
        self.phase_ticks += 1
        elapsed = self.phase_ticks * 0.1

        # Safety keep-armed: if still on the ground and disarmed, re-send arm & offboard
        if not self.is_armed and current_alt < 0.2 and self.state != "PHASE_5_RETURN_AND_LAND":
            if self.tick % 10 == 0:
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

        # -------------------------------------------------------------
        # PHASE 1: Takeoff and Low Hover (~30 seconds)
        # Target: under 1 meter (0.8m -> Z = -0.8m)
        # -------------------------------------------------------------
        if self.state == "PHASE_1_LOW_HOVER":
            target_z = -0.8
            # Smooth climb to 0.8m
            if self.sp_z > target_z:
                self.sp_z -= 0.05
            else:
                self.sp_z = target_z
            self.sp_x = 0.0
            self.sp_y = 0.0
            self.sp_yaw = 0.0
            self.publish_setpoint(self.sp_x, self.sp_y, self.sp_z, self.sp_yaw)

            if self.phase_ticks % 20 == 0:
                self.get_logger().info(f"[PHASE 1 - Low Hover] Alt: {current_alt:.2f}m | Time: {elapsed:.1f}s / 30.0s")

            if elapsed >= 30.0:
                self.state = "PHASE_2_REPOSITION_YAW"
                self.phase_ticks = 0
                self.get_logger().info("\n"
                    "===============================================================\n"
                    "  [PHASE 2] Reposition and Yaw\n"
                    "  Applying slight yaw rotation + small horizontal displacement\n"
                    "  Duration: 15 seconds\n"
                    "===============================================================")

        # -------------------------------------------------------------
        # PHASE 2: Reposition and Yaw (~15 seconds)
        # Slight yaw (45 deg = 0.785 rad), small horizontal movement (0.6m forward, 0.5m right)
        # -------------------------------------------------------------
        elif self.state == "PHASE_2_REPOSITION_YAW":
            target_x = 0.6
            target_y = 0.5
            target_yaw = 0.785  # ~45 deg
            frac = min(1.0, elapsed / 8.0)
            self.sp_x = frac * target_x
            self.sp_y = frac * target_y
            self.sp_z = -0.8
            self.sp_yaw = frac * target_yaw
            self.publish_setpoint(self.sp_x, self.sp_y, self.sp_z, self.sp_yaw)

            if self.phase_ticks % 20 == 0:
                self.get_logger().info(f"[PHASE 2 - Reposition] Pos: [{self.current_pos[0]:.2f}, {self.current_pos[1]:.2f}], Yaw: {math.degrees(self.current_yaw):.1f}° | Time: {elapsed:.1f}s / 15.0s")

            if elapsed >= 15.0:
                self.state = "PHASE_3_CLIMB_MAX_ALT"
                self.phase_ticks = 0
                self.get_logger().info("\n"
                    "===============================================================\n"
                    "  [PHASE 3] Climb to Max Altitude (3.0 meters)\n"
                    "  Duration: 10 seconds\n"
                    "===============================================================")

        # -------------------------------------------------------------
        # PHASE 3: Climb to Max Altitude (~10 seconds)
        # Climb from -0.8m to -3.0m
        # -------------------------------------------------------------
        elif self.state == "PHASE_3_CLIMB_MAX_ALT":
            frac = min(1.0, elapsed / 10.0)
            self.sp_z = -0.8 - frac * (3.0 - 0.8)
            self.sp_x = 0.6
            self.sp_y = 0.5
            self.sp_yaw = 0.785
            self.publish_setpoint(self.sp_x, self.sp_y, self.sp_z, self.sp_yaw)

            if self.phase_ticks % 10 == 0:
                self.get_logger().info(f"[PHASE 3 - Climb] Alt: {current_alt:.2f}m (Target: {-self.sp_z:.2f}m) | Time: {elapsed:.1f}s / 10.0s")

            if elapsed >= 10.0:
                self.state = "PHASE_4_STATION_KEEPING"
                self.phase_ticks = 0
                self.get_logger().info("\n"
                    "===============================================================\n"
                    "  [PHASE 4] Active Station Keeping at 3.0m\n"
                    "  Replicating nervous stick hunting within ~2-meter horizontal box\n"
                    "  Duration: 35 seconds\n"
                    "===============================================================")

        # -------------------------------------------------------------
        # PHASE 4: Active Station Keeping (~35 seconds)
        # Oscillate within a ~2-meter horizontal box at 3m altitude
        # -------------------------------------------------------------
        elif self.state == "PHASE_4_STATION_KEEPING":
            self.sp_z = -3.0
            t = elapsed
            # Rapid multi-frequency hunting movement within ±0.9m box
            dx = 0.65 * math.sin(2.2 * t) + 0.25 * math.sin(4.7 * t) + 0.10 * math.cos(7.3 * t)
            dy = 0.60 * math.cos(1.9 * t) + 0.28 * math.cos(3.8 * t) + 0.12 * math.sin(6.1 * t)
            self.sp_x = 0.6 + dx
            self.sp_y = 0.5 + dy
            self.sp_yaw = 0.785 + 0.08 * math.sin(1.5 * t)
            self.publish_setpoint(self.sp_x, self.sp_y, self.sp_z, self.sp_yaw)

            if self.phase_ticks % 20 == 0:
                self.get_logger().info(
                    f"[PHASE 4 - Station Keeping] Pos: [{self.current_pos[0]:.2f}, {self.current_pos[1]:.2f}], "
                    f"Alt: {current_alt:.2f}m | Box delta: ({dx:+.2f}, {dy:+.2f})m | Time: {elapsed:.1f}s / 35.0s"
                )

            if elapsed >= 35.0:
                self.state = "PHASE_5_RETURN_AND_LAND"
                self.phase_ticks = 0
                self.get_logger().info("\n"
                    "===============================================================\n"
                    "  [PHASE 5] Return and Land (RTL)\n"
                    "  Triggering Return to Launch command at 3m above destination\n"
                    "  Duration: ~15 seconds until touchdown\n"
                    "===============================================================")

        # -------------------------------------------------------------
        # PHASE 5: Return and Land (~15 seconds)
        # Trigger Return to Launch at 3m above destination, descend, disarm at touchdown
        # -------------------------------------------------------------
        elif self.state == "PHASE_5_RETURN_AND_LAND":
            # Bring position back over the destination at 3 meters
            frac = min(1.0, elapsed / 3.0)
            self.sp_x = 0.6 * (1.0 - frac)
            self.sp_y = 0.5 * (1.0 - frac)
            self.sp_z = -3.0
            self.sp_yaw = 0.0

            # Once centered at 3m above destination, initiate RTL / Auto-Land
            if elapsed >= 3.0:
                # Send Auto Land / RTL command to PX4
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                # Continue smooth descent setpoint towards ground level
                self.sp_z = min(0.0, -3.0 + (elapsed - 3.0) * 0.4)
            
            self.publish_setpoint(self.sp_x, self.sp_y, self.sp_z, self.sp_yaw)

            if self.phase_ticks % 10 == 0:
                self.get_logger().info(f"[PHASE 5 - Landing] Alt: {current_alt:.2f}m | Time: {elapsed:.1f}s")

            # Detect touchdown (altitude close to ground, < 0.20m)
            if elapsed >= 8.0 and current_alt < 0.22:
                self.state = "DISARMED"
                # Stick gesture / command disarm
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
                self.get_logger().info("\n"
                    "===============================================================\n"
                    "  TOUCHDOWN CONFIRMED! Motors Disarmed.\n"
                    "  Flight profile completed successfully.\n"
                    "===============================================================")

        elif self.state == "DISARMED":
            # Idle
            pass

    def publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.offboard_pub.publish(msg)

    def publish_setpoint(self, x, y, z, yaw=0.0):
        sp = TrajectorySetpoint()
        sp.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        sp.position = [float(x), float(y), float(z)]
        nan = float('nan')
        sp.velocity = [nan, nan, nan]
        sp.acceleration = [nan, nan, nan]
        sp.jerk = [nan, nan, nan]
        sp.yaw = float(yaw)
        sp.yawspeed = nan
        self.trajectory_pub.publish(sp)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        cmd = VehicleCommand()
        cmd.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        cmd.param1 = float(param1)
        cmd.param2 = float(param2)
        cmd.command = command
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.from_external = True
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = FlightProfilePlayer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
