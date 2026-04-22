
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    pkg_name = 'line_tracking_robot'
    pkg_share = get_package_share_directory(pkg_name)

    world_file = os.path.join(pkg_share, 'worlds', 'line_track.world')
    urdf_file = os.path.join(pkg_share, 'models', 'line_robot', 'robot.urdf.xacro')
    rviz_config = os.path.join(pkg_share, 'config', 'robot.rviz')

    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='true')
    robot_x_arg = DeclareLaunchArgument('robot_x', default_value='-3.0')
    robot_y_arg = DeclareLaunchArgument('robot_y', default_value='-3.0')
    robot_yaw_arg = DeclareLaunchArgument('robot_yaw', default_value='0.0')

    use_rviz = LaunchConfiguration('use_rviz')
    robot_x = LaunchConfiguration('robot_x')
    robot_y = LaunchConfiguration('robot_y')
    robot_yaw = LaunchConfiguration('robot_yaw')

    robot_description = Command(['xacro ', urdf_file])

    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen'
    )

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }]
    )

    spawn_robot = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                name='spawn_robot',
                output='screen',
                arguments=[
                    '-name', 'line_tracking_robot',
                    '-topic', 'robot_description',
                    '-x', robot_x,
                    '-y', robot_y,
                    '-z', '0.05',
                    '-Y', robot_yaw,
                ]
            )
        ]
    )

    bridge = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='gz_ros_bridge',
                output='screen',
                parameters=[{'use_sim_time': True}],
                arguments=[
                    '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                    '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                    '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                    '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                    '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                    '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
                    '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                    '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                ],
            )
        ]
    )

    camera_processor = TimerAction(
        period=8.0,
        actions=[
            Node(
                package=pkg_name,
                executable='camera_processor_node.py',
                name='camera_processor',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'line_threshold': 90,
                    'roi_top_ratio': 0.58,
                    'roi_bottom_ratio': 0.98,
                    'debug_view': True,
                }]
            )
        ]
    )

    obstacle_avoidance = TimerAction(
        period=8.0,
        actions=[
            Node(
                package=pkg_name,
                executable='obstacle_avoidance_node.py',
                name='obstacle_avoidance',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'stop_distance': 0.65,
                    'warn_distance': 0.95,
                    'front_angle_deg': 35.0,
                    'side_angle_deg': 80.0,
                    'avoid_linear_speed': 0.18,
                    'avoid_angular_speed': 1.20,
                    'bypass_turn_gain': 0.30,
                    'return_linear_speed': 0.14,
                    'return_angular_speed': 0.95,
                    'line_reacquire_hold': 0.40,
                    'min_bypass_time': 1.10,
                    'max_avoid_time': 8.00,
                }]
            )
        ]
    )

    line_follower = TimerAction(
        period=8.0,
        actions=[
            Node(
                package=pkg_name,
                executable='line_follower_node.py',
                name='line_follower',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'kp': 1.20,
                    'ki': 0.002,
                    'kd': 0.20,
                    'base_speed': 0.28,
                    'max_angular': 2.20,
                    'search_angular': 0.60,
                    'search_linear': 0.03,
                    'min_speed_factor': 0.42,
                }]
            )
        ]
    )

    robot_controller = TimerAction(
        period=9.0,
        actions=[
            Node(
                package=pkg_name,
                executable='robot_controller_node.py',
                name='robot_controller',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'emergency_stop_dist': 0.10,
                }]
            )
        ]
    )

    rviz = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config],
                condition=IfCondition(use_rviz),
                parameters=[{'use_sim_time': True}]
            )
        ]
    )

    return LaunchDescription([
        use_rviz_arg,
        robot_x_arg,
        robot_y_arg,
        robot_yaw_arg,
        LogInfo(msg='>>> Launching Line Tracking Robot – Jazzy + Gazebo Harmonic'),
        gz_sim,
        robot_state_pub,
        spawn_robot,
        bridge,
        camera_processor,
        obstacle_avoidance,
        line_follower,
        robot_controller,
        rviz,
    ])
