"""
qgc_fsi6x_teleop.launch.py

PX4 SITL (Gazebo Harmonic) + QGroundControl + FlySky FS-i6X stick teleop.

Same stack as qgc_teleop.launch.py, plus:
  9.  joy_node         -> reads the FS-i6X over USB, publishes /joy
  10. joystick_teleop  -> /joy -> /cmd_vel, handles ARM / DISARM stick gestures
  11. teleop_bridge    -> /cmd_vel -> PX4 offboard (/fmu/in/*)
  12. LogInfo banner   -> arm / disarm instructions

Usage:
  ros2 launch eft_control qgc_fsi6x_teleop.launch.py
  ros2 launch eft_control qgc_fsi6x_teleop.launch.py enable_joy:=false
  ros2 launch eft_control qgc_fsi6x_teleop.launch.py invert_yaw:=true invert_roll:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument  # noqa: F401  (external path arguments)
from launch.substitutions import LaunchConfiguration
from eft_control.external_paths import external_path_args


def generate_launch_description():
    pkg_share = get_package_share_directory('eft_control')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'eft_e610p.rviz')
    urdf_file = os.path.join(pkg_share, 'urdf', 'eft_e610p.urdf')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    px4_env = os.environ.copy()
    px4_env['GALLIUM_DRIVER'] = 'llvmpipe'
    px4_env['PX4_GZ_NO_FOLLOW'] = '1'

    # ------------------------------------------------------------------
    # Launch arguments
    # ------------------------------------------------------------------
    qgc_path = LaunchConfiguration('qgc_path')
    enable_joy = LaunchConfiguration('enable_joy')
    joy_id = LaunchConfiguration('joy_id')
    joy_name = LaunchConfiguration('joy_name')
    axis_roll = LaunchConfiguration('axis_roll')
    axis_pitch = LaunchConfiguration('axis_pitch')
    axis_throttle = LaunchConfiguration('axis_throttle')
    axis_yaw = LaunchConfiguration('axis_yaw')
    invert_roll = LaunchConfiguration('invert_roll')
    invert_pitch = LaunchConfiguration('invert_pitch')
    invert_throttle = LaunchConfiguration('invert_throttle')
    invert_yaw = LaunchConfiguration('invert_yaw')

    declared_args = [
        DeclareLaunchArgument(
            'qgc_path',
            default_value=os.path.expanduser('~/Downloads/QGroundControl-x86_64.AppImage'),
            description='Absolute path to QGroundControl AppImage'),
        DeclareLaunchArgument(
            'enable_joy', default_value='true',
            description='Start joy_node + joystick_teleop + teleop_bridge'),
        # NOTE: the ROS 2 `joy` package uses SDL2. Its `device_name` param is the
        # joystick *name* (e.g. "FlySky FS-i6X ..."), NOT a /dev/input/js* path.
        # Leave joy_name empty to pick the controller by index (joy_id, 0 = js0).
        DeclareLaunchArgument(
            'joy_id', default_value='0',
            description='SDL joystick index (0 is normally /dev/input/js0)'),
        DeclareLaunchArgument(
            'joy_name', default_value='',
            description='SDL joystick name; if set, overrides joy_id'),
        DeclareLaunchArgument('axis_roll', default_value='0', description='Right stick horizontal (CH1)'),
        DeclareLaunchArgument('axis_pitch', default_value='1', description='Right stick vertical (CH2)'),
        DeclareLaunchArgument('axis_throttle', default_value='2', description='Left stick vertical (CH3)'),
        DeclareLaunchArgument('axis_yaw', default_value='3', description='Left stick horizontal (CH4)'),
        # Flip an axis if its sign is wrong. Expected after inversion:
        # roll right = +, pitch forward = +, throttle up = +, yaw right = +
        DeclareLaunchArgument('invert_roll', default_value='false'),
        DeclareLaunchArgument('invert_pitch', default_value='false'),
        DeclareLaunchArgument('invert_throttle', default_value='false'),
        DeclareLaunchArgument('invert_yaw', default_value='false'),
    ]

    # ------------------------------------------------------------------
    # 1-8: same as qgc_teleop.launch.py
    # ------------------------------------------------------------------
    # 1. Micro XRCE-DDS Agent
    dds_agent = ExecuteProcess(
        cmd=[LaunchConfiguration('xrce_agent'), 'udp4', '-p', '8888'],
        output='screen'
    )

    # 2. PX4 SITL & Gazebo
    px4_gazebo = ExecuteProcess(
        cmd=['make', 'px4_sitl', 'gz_eft_e610p'],
        cwd=LaunchConfiguration('px4_dir'),
        env=px4_env,
        output='screen'
    )

    # 3. Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    # 4. Joint State Publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    # 5. Trajectory Visualizer & TF Broadcaster
    trajectory_visualizer = Node(
        package='eft_control',
        executable='trajectory_visualizer',
        name='trajectory_visualizer',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 6. RViz2
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}]
    )

    # 7. QGroundControl
    qgc = ExecuteProcess(
        cmd=[qgc_path],
        output='screen'
    )

    # 8. ROS <-> Gazebo clock bridge
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']
    )

    # ------------------------------------------------------------------
    # 9-11: FS-i6X teleop chain (only if enable_joy:=true)
    # ------------------------------------------------------------------
    # 9. joy_node: FS-i6X -> /joy at 20 Hz
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        condition=IfCondition(enable_joy),
        parameters=[{
            'device_id': ParameterValue(joy_id, value_type=int),
            'device_name': ParameterValue(joy_name, value_type=str),
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,
        }]
    )

    # 10. joystick_teleop: /joy -> /cmd_vel + arm/disarm gestures
    #     Wall-clock on purpose: the 0.5 s hold timer must work before /clock exists.
    joystick_teleop = Node(
        package='eft_control',
        executable='joystick_teleop',
        name='joystick_teleop',
        output='screen',
        condition=IfCondition(enable_joy),
        parameters=[{
            'axis_roll': ParameterValue(axis_roll, value_type=int),
            'axis_pitch': ParameterValue(axis_pitch, value_type=int),
            'axis_throttle': ParameterValue(axis_throttle, value_type=int),
            'axis_yaw': ParameterValue(axis_yaw, value_type=int),
            'invert_roll': ParameterValue(invert_roll, value_type=bool),
            'invert_pitch': ParameterValue(invert_pitch, value_type=bool),
            'invert_throttle': ParameterValue(invert_throttle, value_type=bool),
            'invert_yaw': ParameterValue(invert_yaw, value_type=bool),
            'scale_linear_x': 2.0,
            'scale_linear_y': 2.0,
            'scale_linear_z': 1.5,
            'scale_angular_z': 1.5,
            'stick_deadzone': 0.08,
            'stick_threshold': 0.75,
            'center_threshold': 0.3,
            'arm_hold_time': 0.5,
        }]
    )

    # 11. teleop_bridge: /cmd_vel -> PX4 offboard
    #     Wall-clock so the 10 Hz heartbeat starts immediately (PX4 needs it
    #     streaming before it will accept the switch to offboard).
    teleop_bridge = Node(
        package='eft_control',
        executable='teleop_bridge',
        name='teleop_bridge',
        output='screen',
        condition=IfCondition(enable_joy),
        parameters=[{
            'auto_arm': False,
            'takeoff_height': 1.5,
            'max_height': 30.0,
        }]
    )

    # 12. Banner
    banner = LogInfo(
        condition=IfCondition(enable_joy),
        msg="\n"
            "===============================================================\n"
            "  EFT E610P  |  FS-i6X TELEOP (Mode 2)\n"
            "---------------------------------------------------------------\n"
            "  ARM    : LEFT stick BOTTOM-LEFT  , hold 0.5 s (right stick centred)\n"
            "           -> OFFBOARD + ARM, climbs to 1.5 m\n"
            "  THEN   : bring THROTTLE to MID before it will respond\n"
            "  FLY    : right stick = move fwd/back/left/right\n"
            "           left stick  = throttle up/down (climb rate), yaw\n"
            "  DISARM : land first, then LEFT stick BOTTOM-RIGHT, hold 0.5 s\n"
            "===============================================================\n"
    )

    return LaunchDescription(external_path_args() + declared_args + [
        dds_agent,
        px4_gazebo,
        robot_state_publisher,
        joint_state_publisher,
        trajectory_visualizer,
        rviz2,
        qgc,
        clock_bridge,
        joy_node,
        joystick_teleop,
        teleop_bridge,
        banner,
    ])
