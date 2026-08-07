import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleOdometry
from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import Path
from tf2_ros import TransformBroadcaster
import math

def quaternion_to_euler(w, x, y, z):
    # roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    # yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

def euler_to_quaternion(roll, pitch, yaw):
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
    return w, x, y, z

class TrajectoryVisualizer(Node):
    def __init__(self):
        super().__init__('trajectory_visualizer')

        # QoS configuration to match PX4 telemetry best effort publishers
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Subscriber to PX4 vehicle odometry
        self.odom_sub = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odom_callback,
            qos_profile
        )

        # Path publisher (in ENU 'map' frame)
        self.path_pub = self.create_publisher(Path, '/drone/path', 10)
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'map'

        # TF transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info("Trajectory Visualizer & TF Broadcaster Node Initialized!")

    def odom_callback(self, msg):
        # 1. Coordinate Frame translation (NED -> ENU)
        # Position: X_enu = Y_ned, Y_enu = X_ned, Z_enu = -Z_ned
        x_enu = msg.position[1]
        y_enu = msg.position[0]
        z_enu = -msg.position[2]

        # Orientation: Convert PX4 NED quaternion to ENU
        # q = [w, x, y, z]
        roll_ned, pitch_ned, yaw_ned = quaternion_to_euler(
            msg.q[0], msg.q[1], msg.q[2], msg.q[3]
        )

        # Convert angles to ENU
        roll_enu = roll_ned
        pitch_enu = -pitch_ned
        yaw_enu = -yaw_ned + (math.pi / 2.0)

        w_enu, x_enu_rot, y_enu_rot, z_enu_rot = euler_to_quaternion(
            roll_enu, pitch_enu, yaw_enu
        )

        # 2. Broadcast Dynamic Transform (map -> base_footprint)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_footprint'

        t.transform.translation.x = float(x_enu)
        t.transform.translation.y = float(y_enu)
        t.transform.translation.z = float(z_enu)

        t.transform.rotation.w = float(w_enu)
        t.transform.rotation.x = float(x_enu_rot)
        t.transform.rotation.y = float(y_enu_rot)
        t.transform.rotation.z = float(z_enu_rot)

        self.tf_broadcaster.sendTransform(t)

        # 3. Publish Path Message
        pose = PoseStamped()
        pose.header.stamp = t.header.stamp
        pose.header.frame_id = 'map'
        pose.pose.position.x = float(x_enu)
        pose.pose.position.y = float(y_enu)
        pose.pose.position.z = float(z_enu)
        pose.pose.orientation.w = float(w_enu)
        pose.pose.orientation.x = float(x_enu_rot)
        pose.pose.orientation.y = float(y_enu_rot)
        pose.pose.orientation.z = float(z_enu_rot)

        self.path_msg.poses.append(pose)
        self.path_msg.header.stamp = t.header.stamp
        self.path_pub.publish(self.path_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
