import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('line_follower_bot')
    urdf_path = os.path.join(pkg_share, 'urdf', 'line_follower.urdf')
    bridge_config = os.path.join(pkg_share, 'config', 'bridge.yaml')

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # 1. Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    # 2. Spawn Robot in Gazebo Sim (gz-sim)
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'ir_line_follower',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.05'
        ],
        output='screen'
    )

    # 3. ROS <-> Gazebo Bridge
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config}],
        output='screen'
    )

    # 4. 3-IR PID Line Follower Node
    pid_controller = Node(
        package='line_follower_bot',
        executable='main_follower',
        name="main_follower",
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        robot_state_publisher,
        spawn_entity,
        ros_gz_bridge,
        pid_controller
    ])