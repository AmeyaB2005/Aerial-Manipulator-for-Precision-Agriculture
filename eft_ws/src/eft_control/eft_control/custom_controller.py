import rclpy
from rclpy.node import Node
import numpy as np
import math

# PX4 ROS 2 Message Interfaces
from px4_msgs.msg import OffboardControlMode, VehicleCommand, ActuatorMotors, VehicleOdometry, VehicleAttitude
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Point

def euler_to_quat(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return np.array([w, x, y, z])

def quaternion_multiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2
    return np.array([w, x, y, z])

class CustomController(Node):
    def __init__(self):
        super().__init__('custom_controller')

        # =====================================================================
        # 1. PHYSICAL CONSTANTS & DRONE PARAMETERS (EFT E610P)
        # =====================================================================
        self.m = 11.77         # Mass (kg) - base + tank + 6 rotors
        self.g = 9.81          # Gravity (m/s^2)
        self.kf = 8.0e-05      # Motor thrust constant
        self.km = 0.028        # Motor moment scale (from momentConstant in SDF)
        self.max_rad_s = 1000.0 # Max motor speed in rad/s
        
        # Inertia tensor (including rotors calculated via parallel axis theorem)
        self.I = np.array([1.27, 1.27, 2.51])

        # =====================================================================
        # 2. TUNING GAINS (CASCADED PID ARCHITECTURE)
        # =====================================================================
        # Outer Loop: Position PID Gains (Error -> Desired Acceleration)
        self.Kp_pos = np.array([1.0, 1.0, 1.5])   # [X, Y, Z]
        self.Kd_pos = np.array([0.6, 0.6, 0.8])
        self.Ki_pos = np.array([0.01, 0.01, 0.02]) # Small integral gain to reduce steady-state offset

        # Middle Loop: Attitude P Gains (Angle Error -> Desired Angular Rates)
        self.Kp_att = np.array([5.0, 5.0, 3.0])   # [Roll, Pitch, Yaw]

        # Inner Loop: Angular Rate P Gains (Rate Error -> Target Torques)
        self.Kp_rate = np.array([5.0, 5.0, 3.0]) # [p, q, r]

        # =====================================================================
        # 3. STATE VARIABLES & ERROR INTEGRATORS
        # =====================================================================
        self.timestamp = 0
        self.current_pos = np.array([0.0, 0.0, 0.0])
        self.current_vel = np.array([0.0, 0.0, 0.0])
        self.current_quat = np.array([1.0, 0.0, 0.0, 0.0]) # [w, x, y, z]
        self.current_rates = np.array([0.0, 0.0, 0.0])     # [p, q, r] (rad/s)

        self.target_pos = np.array([0.0, 0.0, -3.0])       # Target: Hover at 3m (NED Z is negative)
        self.integral_pos_error = np.array([0.0, 0.0, 0.0])
        self.loop_counter = 0

        # =====================================================================
        # 4. CONTROL ALLOCATION (EXACT 4x6 MIXER MATRIX FOR EFT E610P)
        # =====================================================================
        # Exact motor positions (PX, PY) and direction constants from airframe setup
        px = np.array([0.0, 0.0, 0.6092, -0.6092, 0.6092, -0.6092])
        py = np.array([-0.7035, 0.7035, 0.35175, -0.35175, -0.35175, 0.35175])
        km_dir = np.array([-1.0, 1.0, -1.0, 1.0, 1.0, -1.0])

        # Construct M matrix
        self.M = np.zeros((4, 6))
        self.M[0, :] = self.kf                 # Thrust row
        self.M[1, :] = -py * self.kf           # Roll torque row
        self.M[2, :] = px * self.kf            # Pitch torque row
        self.M[3, :] = km_dir * self.km * self.kf # Yaw torque row
        
        # Moore-Penrose Pseudo-Inverse (6x4)
        self.M_pinv = np.linalg.pinv(self.M)

        # =====================================================================
        # 5. PUBLISHERS & SUBSCRIBERS
        # =====================================================================
        # Configure Best Effort QoS Profile for telemetry topics
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.pub_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.pub_vehicle_command = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.pub_motors = self.create_publisher(ActuatorMotors, '/fmu/in/actuator_motors', 10)

        # We subscribe to VehicleOdometry for position, velocity, and angular rates (clean & filtered)
        self.sub_odom = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_callback, qos_profile)
        # We subscribe to VehicleAttitude for the Hamiltonian quaternion q(w, x, y, z)
        self.sub_att = self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self.att_callback, qos_profile)
        # Subscribe to dynamically set target position
        self.sub_target = self.create_subscription(Point, '/eft/target_position', self.target_callback, 10)

        # 100 Hz Control Loop Timer
        self.dt = 0.01
        self.timer = self.create_timer(self.dt, self.control_loop)
        self.setpoint_counter = 0

    # --- Callbacks ---
    def odom_callback(self, msg):
        self.timestamp = msg.timestamp
        self.current_pos = np.array(msg.position)
        self.current_vel = np.array(msg.velocity)
        self.current_rates = np.array(msg.angular_velocity)

    def att_callback(self, msg):
        self.current_quat = np.array(msg.q) # [w, x, y, z]

    def target_callback(self, msg):
        self.target_pos = np.array([msg.x, msg.y, msg.z])
        self.get_logger().info(f"Updated target position to: {self.target_pos.round(2)}")

    # --- Helper: Quaternion to Euler (Roll, Pitch, Yaw) ---
    def quat_to_euler(self, q):
        w, x, y, z = q[0], q[1], q[2], q[3]
        roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return roll, pitch, yaw

    def arm_and_set_offboard(self):
        # Engage Offboard Mode
        cmd_mode = VehicleCommand()
        cmd_mode.timestamp = self.timestamp
        cmd_mode.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        cmd_mode.param1 = 1.0
        cmd_mode.param2 = 6.0
        cmd_mode.target_system, cmd_mode.target_component = 1, 1
        cmd_mode.source_system, cmd_mode.source_component = 1, 1
        cmd_mode.from_external = True
        self.pub_vehicle_command.publish(cmd_mode)

        # Arm Drone
        cmd_arm = VehicleCommand()
        cmd_arm.timestamp = self.timestamp
        cmd_arm.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        cmd_arm.param1 = 1.0
        cmd_arm.target_system, cmd_arm.target_component = 1, 1
        cmd_arm.source_system, cmd_arm.source_component = 1, 1
        cmd_arm.from_external = True
        self.pub_vehicle_command.publish(cmd_arm)
        self.get_logger().info('Offboard Mode Set & Drone Armed!')

    # =====================================================================
    # 6. MAIN 100Hz CONTROL LOOP
    # =====================================================================
    def control_loop(self):
        # Return early if no telemetry message has been received yet (timestamp is 0)
        if self.timestamp == 0:
            return

        # Publish Offboard Heartbeat with direct actuator authority enabled
        mode_msg = OffboardControlMode()
        mode_msg.timestamp = self.timestamp
        mode_msg.position = False
        mode_msg.velocity = False
        mode_msg.acceleration = False
        mode_msg.attitude = False
        mode_msg.body_rate = False
        mode_msg.direct_actuator = True
        self.pub_offboard_mode.publish(mode_msg)

        if self.setpoint_counter == 50:
            self.arm_and_set_offboard()
        if self.setpoint_counter < 100:
            self.setpoint_counter += 1
            return

        # -----------------------------------------------------------------
        # A. OUTER LOOP: Position Control
        # -----------------------------------------------------------------
        pos_error = self.target_pos - self.current_pos
        self.integral_pos_error += pos_error * self.dt
        self.integral_pos_error = np.clip(self.integral_pos_error, -2.0, 2.0)

        # Calculate target acceleration (Inertial Frame)
        a_des = (self.Kp_pos * pos_error) + (self.Kd_pos * (-self.current_vel)) + (self.Ki_pos * self.integral_pos_error)

        # Total Desired Thrust Force vector F_des = m * (g_vec - a_des)
        g_vec = np.array([0.0, 0.0, self.g])
        F_des = self.m * (g_vec - a_des)
        T = np.linalg.norm(F_des)
        T = max(0.0, T) # Ensure non-negative thrust

        # Extract current yaw to rotate target accelerations into the body yaw-aligned frame
        roll_curr, pitch_curr, yaw_curr = self.quat_to_euler(self.current_quat)
        
        # Rotate from Inertial (World) to body-heading frame
        a_des_x_body = a_des[0] * math.cos(yaw_curr) + a_des[1] * math.sin(yaw_curr)
        a_des_y_body = -a_des[0] * math.sin(yaw_curr) + a_des[1] * math.cos(yaw_curr)

        # Calculate Roll and Pitch angles
        pitch_des = -a_des_x_body / self.g
        roll_des = a_des_y_body / self.g
        
        # Clamp roll and pitch setpoints to prevent extreme tilt angles (+/- 30 deg)
        max_tilt = 0.52
        pitch_des = max(-max_tilt, min(max_tilt, pitch_des))
        roll_des = max(-max_tilt, min(max_tilt, roll_des))
        yaw_des = 0.0

        # -----------------------------------------------------------------
        # B. MIDDLE LOOP: Quaternion Attitude Control (Shortest Path)
        # -----------------------------------------------------------------
        q_des = euler_to_quat(roll_des, pitch_des, yaw_des)
        q_curr = self.current_quat
        q_curr_inv = np.array([q_curr[0], -q_curr[1], -q_curr[2], -q_curr[3]])
        
        q_err = quaternion_multiply(q_curr_inv, q_des)
        if q_err[0] < 0:
            q_err = -q_err
            
        # The vector part of the error quaternion represents the rotation vector error in body frame
        att_error = 2.0 * q_err[1:4]
        
        rates_des = self.Kp_att * att_error
        
        # Limit desired body rates to prevent extreme commands
        rates_des = np.clip(rates_des, -3.0, 3.0)

        # -----------------------------------------------------------------
        # C. INNER LOOP: Angular Rate Control -> Desired Torques (N-m)
        # -----------------------------------------------------------------
        rate_error = rates_des - self.current_rates
        torques = self.I * (self.Kp_rate * rate_error)
        
        # Ground takeoff safety guard: if the drone is on the ground (altitude > -0.3m in NED),
        # disable control torques to avoid ground contact physics explosions.
        if self.current_pos[2] > -0.3:
            tau_phi, tau_theta, tau_psi = 0.0, 0.0, 0.0
        else:
            tau_phi, tau_theta, tau_psi = torques[0], torques[1], torques[2]

        # -----------------------------------------------------------------
        # D. MIXER MATRIX: Virtual Commands -> Motor Speeds (omega^2)
        # -----------------------------------------------------------------
        control_vector = np.array([T, tau_phi, tau_theta, tau_psi])
        omega_squared = np.dot(self.M_pinv, control_vector)

        # Convert squared angular velocities to normalized motor commands [0.0, 1.0]
        motor_cmds = []
        for w2 in omega_squared:
            w = math.sqrt(max(0.0, w2))
            norm_cmd = min(1.0, max(0.0, w / self.max_rad_s))
            motor_cmds.append(norm_cmd)

        # -----------------------------------------------------------------
        # E. PUBLISH TO MOTOR ACTUATORS
        # -----------------------------------------------------------------
        actuator_msg = ActuatorMotors()
        actuator_msg.timestamp = self.timestamp
        actuator_msg.control = motor_cmds + [0.0] * 6 # Fill remaining array slots with 0 to match float32[12] size
        self.pub_motors.publish(actuator_msg)

        # -----------------------------------------------------------------
        # F. DIAGNOSTIC PRINTING (THROTTLED AT 1Hz / 100 LOOPS)
        # -----------------------------------------------------------------
        self.loop_counter += 1
        if self.loop_counter % 100 == 0:
            self.get_logger().info(
                f"\n--- Loop {self.loop_counter} ---\n"
                f"Pos Curr: {self.current_pos.round(2)} | Target: {self.target_pos.round(2)}\n"
                f"Att Curr: R={round(roll_curr, 2)} P={round(pitch_curr, 2)} Y={round(yaw_curr, 2)}\n"
                f"Att Des : R={round(roll_des, 2)} P={round(pitch_des, 2)} Y={round(yaw_des, 2)}\n"
                f"Rates Des: {rates_des.round(2)} | Curr: {self.current_rates.round(2)}\n"
                f"Torques: {torques.round(3)} | Thrust: {round(T, 1)} N\n"
                f"Motor Cmds: {np.array(motor_cmds).round(3)}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = CustomController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
