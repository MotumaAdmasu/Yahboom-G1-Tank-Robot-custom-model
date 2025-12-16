from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # Get package path
    pkg_path = get_package_share_directory('yahboom_g1_description')
    urdf_file = os.path.join(pkg_path, 'urdf', 'yahboom_g1.urdf')

    # Start Gazebo
    gazebo_cmd = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(urdf_file, 'r').read()}]
    )

    # Spawn robot in Gazebo
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'yahboom_g1', '-file', urdf_file],
        output='screen'
    )

    return LaunchDescription([
        gazebo_cmd,
        robot_state_publisher_node,
        spawn_entity_node,
    ])
