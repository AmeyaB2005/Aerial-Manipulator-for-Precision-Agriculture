import rclpy
from rclpy.node import Node
import numpy as np
import math

# PX4 ROS 2 Message Interfaces
from px4_msgs.msg import OffboardControlMode, VehicleCommand, ActuatorMotors, VehicleOdometry, VehicleAttitude, SensorCombined
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class CustomController(Node):
    def __init__(self):
        super().__init__('custom_controller')

        # =====================================================================
        # 1. PHYSICAL CONSTANTS & DRONE PARAMETERS (EFT E610P)
        # =====================================================================
        self.m = 11.77         # Mass (kg) - Link sum (base + tank + 6 rotors)
        self.g = 9.81          # Gravity (m/s^2)
        self.kf = 8.0e-05      # Motor thrust constant (matching simulation)
        self.km = 0.05         # Motor moment scale
        self.max_rad_s = 1000.0 # Max motor speed in rad/s (matching simulation maxRotVelocity)

        # =====================================================================
        # 2. TUNING GAINS (CASCADED PID ARCHITECTURE)
        # =====================================================================
        # Outer Loop: Position PID Gains (Error -> Desired Acceleration/Thrust)
        self.Kp_pos = np.array([1.5, 1.5, 2.0])   # [X, Y, Z]
        self.Kd_pos = np.array([0.8, 0.8, 1.2])
        self.Ki_pos = np.array([0.01, 0.01, 0.05])

        # Middle Loop: Attitude PID Gains (Angle Error -> Desired Angular Rates)
        self.Kp_att = np.array([6.0, 6.0, 3.0])   # [Roll, Pitch, Yaw]

        # Inner Loop: Angular Rate PID Gains (Rate Error -> Target Torques)
        self.Kp_rate = np.array([0.15, 0.15, 0.10]) # [p, q, r]
        self.Kd_rate = np.array([0.01, 0.01, 0.005])

        # =====================================================================
        # 3. STATE VARIABLES & ERROR INTEGRATORS
        # =====================================================================
        self.current_pos = np.array([0.0, 0.0, 0.0])
        self.current_vel = np.array([0.0, 0.0, 0.0])
        self.current_quat = np.array([1.0, 0.0, 0.0, 0.0]) # [q0, q1, q2, q3]
        self.current_rates = np.array([0.0, 0.0, 0.0])     # [p, q, r] (rad/s)

        self.target_pos = np.array([0.0, 0.0, -3.0])       # Target: Hover at 3m (NED Z is negative)
        self.integral_pos_error = np.array([0.0, 0.0, 0.0])

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
        self.M[3, :] = km_dir * self.km         # Yaw torque row
        
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

        self.sub_odom = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_callback, qos_profile)
        self.sub_att = self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self.att_callback, qos_profile)
        self.sub_sensor = self.create_subscription(SensorCombined, '/fmu/out/sensor_combined', self.sensor_callback, qos_profile)

        # 100 Hz Control Loop Timer
        self.dt = 0.01
        self.timer = self.create_timer(self.dt, self.control_loop)
        self.setpoint_counter = 0

    # --- Callbacks ---
    def odom_callback(self, msg):
        self.current_pos = np.array(msg.position)
        self.current_vel = np.array(msg.velocity)

    def att_callback(self, msg):
        self.current_quat = np.array(msg.q) # [w, x, y, z]

    def sensor_callback(self, msg):
        self.current_rates = np.array(msg.gyro_rad) # [p, q, r]

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
        cmd_mode.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        cmd_mode.param1 = 1.0
        cmd_mode.param2 = 6.0
        cmd_mode.target_system, cmd_mode.target_component = 1, 1
        cmd_mode.source_system, cmd_mode.source_component = 1, 1
        cmd_mode.from_external = True
        self.pub_vehicle_command.publish(cmd_mode)

        # Arm Drone
        cmd_arm = VehicleCommand()
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
        # Publish Offboard Heartbeat with direct actuator authority enabled
        mode_msg = OffboardControlMode()
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
        # A. OUTER LOOP: Position Control (Newton-Euler Kinematics/Dynamics)
        # -----------------------------------------------------------------
        pos_error = self.target_pos - self.current_pos
        self.integral_pos_error += pos_error * self.dt
        self.integral_pos_error = np.clip(self.integral_pos_error, -2.0, 2.0) # Prevent integral windup

        # Calculate target acceleration (Inertial Frame)
        a_des = (self.Kp_pos * pos_error) + (self.Kd_pos * (-self.current_vel)) + (self.Ki_pos * self.integral_pos_error)

        # Total Thrust Force (N) along Body Z
        T = self.m * (self.g - a_des[2])
        T = max(0.0, T) # Ensure non-negative thrust

        # Extract current yaw to rotate target accelerations into the body yaw-aligned frame
        roll_curr, pitch_curr, yaw_curr = self.quat_to_euler(self.current_quat)
        
        # Rotate from Inertial (World) to body-heading frame
        a_des_x_body = a_des[0] * math.cos(yaw_curr) + a_des[1] * math.sin(yaw_curr)
        a_des_y_body = -a_des[0] * math.sin(yaw_curr) + a_des[1] * math.cos(yaw_curr)

        # Calculate Roll and Pitch angles with correct sign relationships
        pitch_des = -a_des_x_body / self.g
        roll_des = a_des_y_body / self.g
        
        # Clamp roll and pitch setpoints to prevent extreme tilt angles (+/- 30 deg)
        max_tilt = 0.52
        pitch_des = max(-max_tilt, min(max_tilt, pitch_des))
        roll_des = max(-max_tilt, min(max_tilt, roll_des))
        
        yaw_des = 0.0

        # -----------------------------------------------------------------
        # B. MIDDLE LOOP: Attitude Control
        # -----------------------------------------------------------------
        # Shortest-path yaw error wrapping
        yaw_error = yaw_des - yaw_curr
        yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))
        
        att_error = np.array([roll_des - roll_curr, pitch_des - pitch_curr, yaw_error])
        rates_des = self.Kp_att * att_error
        
        # Limit desired body rates to prevent extreme commands
        rates_des = np.clip(rates_des, -2.0, 2.0)

        # -----------------------------------------------------------------
        # C. INNER LOOP: Angular Rate Control -> Desired Torques
        # -----------------------------------------------------------------
        rate_error = rates_des - self.current_rates
        torques = (self.Kp_rate * rate_error) + (self.Kd_rate * (-self.current_rates))
        
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
        actuator_msg.control = motor_cmds + [0.0] * 6 # Fill remaining array slots with 0 to match float32[12] size
        self.pub_motors.publish(actuator_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CustomController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
