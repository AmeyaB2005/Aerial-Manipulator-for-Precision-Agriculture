import mujoco
import mujoco.viewer
import time
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class MuJoCoControllerNode(Node):
    def __init__(self):
        super().__init__('mujoco_controller_node')
        self.subscription = self.create_subscription(
            Bool,
            '/drone/at_tree',
            self.listener_callback,
            10
        )
        self.bending_target = False

    def listener_callback(self, msg):
        self.bending_target = msg.data
        self.get_logger().info(f"Received target state: {self.bending_target}")

# Initialize ROS 2
rclpy.init()
ros_node = MuJoCoControllerNode()

model = mujoco.MjModel.from_xml_path('tdcr_arm.xml')
data = mujoco.MjData(model)

with mujoco.viewer.launch_passive(model, data) as viewer:
    start_time = time.time()
    
    while viewer.is_running():
        step_start = time.time()
        
        # Spin ROS 2 node briefly to check for messages
        rclpy.spin_once(ros_node, timeout_sec=0.0)
        
        if ros_node.bending_target:
            # Bend the arm towards the East/North-East side
            tension_NE = 40.0
            tension_SE = 40.0
            tension_SW = 0.0
            tension_NW = 0.0
        else:
            # Go back to a straight state (zero tension)
            tension_NE = 0.0
            tension_SE = 0.0
            tension_SW = 0.0
            tension_NW = 0.0
            
        # Apply tension to the actuators
        data.actuator('pull_NE').ctrl[0] = tension_NE
        data.actuator('pull_SE').ctrl[0] = tension_SE
        data.actuator('pull_SW').ctrl[0] = tension_SW
        data.actuator('pull_NW').ctrl[0] = tension_NW

        mujoco.mj_step(model, data)
        viewer.sync()

        # Enforce real-time execution
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)

# Shutdown ROS 2 when viewer closes
ros_node.destroy_node()
rclpy.shutdown()
