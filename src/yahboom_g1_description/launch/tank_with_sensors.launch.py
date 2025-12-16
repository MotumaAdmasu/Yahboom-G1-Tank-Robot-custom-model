from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_path = get_package_share_directory('yahboom_g1_description')
    urdf_file = os.path.join(pkg_path, 'urdf', 'yahboom_g1_with_sensors.urdf')

    # Read URDF
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # Simple Tank Movement Controller
    movement_controller = Node(
        package='yahboom_g1_description',
        executable='simple_tank_movement.py',
        output='screen',
        name='movement_controller'
    )

    # RViz with clean config
    rviz_config = os.path.join(pkg_path, 'rviz', 'clean_tank_movement.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    return LaunchDescription([
        robot_state_publisher,
        movement_controller,
        rviz_node,
    ])
