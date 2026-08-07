import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('eft_control')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'eft_e610p.rviz')
    urdf_file = os.path.join(pkg_share, 'urdf', 'eft_e610p.urdf')

    # Read URDF contents
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    
    # Extend the current environment variables
    px4_env = os.environ.copy()
    px4_env['GALLIUM_DRIVER'] = 'llvmpipe'
    px4_env['PX4_GZ_NO_FOLLOW'] = '1'

    # 1. Micro XRCE-DDS Agent
    dds_agent = ExecuteProcess(
        cmd=['/home/ameya/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent', 'udp4', '-p', '8888'],
        output='screen'
    )

    # 2. PX4 SITL & Gazebo
    px4_gazebo = ExecuteProcess(
        cmd=['make', 'px4_sitl', 'gz_eft_e610p'],
        cwd='/home/ameya/PX4-Autopilot',
        env=px4_env,
        output='screen'
    )

    # 3. Robot State Publisher Node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True
        }]
    )

    # 4. Joint State Publisher Node
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True
        }]
    )

    # 5. Trajectory Visualizer & TF Broadcaster
    trajectory_visualizer = Node(
        package='eft_control',
        executable='trajectory_visualizer',
        name='trajectory_visualizer',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 6. Joy Node (ROS 2 driver for joystick)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 7. Joystick Teleop Node (Joy -> cmd_vel)
    joystick_teleop = Node(
        package='eft_control',
        executable='joystick_teleop',
        name='joystick_teleop',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 8. Teleop Bridge (cmd_vel -> PX4 offboard)
    teleop_bridge = Node(
        package='eft_control',
        executable='teleop_bridge',
        name='teleop_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 9. RViz2 Node
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}]
    )

    # 10. ROS-Gazebo Clock Bridge Node
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']
    )

    # Info log instruction printout
    instruction_log = LogInfo(
        msg="\n"
            "===============================================================\n"
            "SIMULATION READY FOR JOYSTICK TELEOP!\n"
            "Ensure your gamepad/joystick is connected to /dev/input/js0\n"
            "===============================================================\n"
    )

    return LaunchDescription([
        dds_agent,
        px4_gazebo,
        robot_state_publisher,
        joint_state_publisher,
        trajectory_visualizer,
        joy_node,
        joystick_teleop,
        teleop_bridge,
        rviz2,
        instruction_log,
        clock_bridge
    ])
