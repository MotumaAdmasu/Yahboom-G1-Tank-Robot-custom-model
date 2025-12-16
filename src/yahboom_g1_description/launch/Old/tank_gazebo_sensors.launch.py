from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
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

    # Start Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('gazebo_ros'),
            '/launch/gazebo.launch.py'
        ]),
        launch_arguments={'world': 'empty'}.items()
    )

    # Spawn tank in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'yahboom_tank_sensors', '-topic', 'robot_description', '-x', '0', '-y', '0', '-z', '0.1'],
        output='screen'
    )

    # Simple Tank Movement Controller
    movement_controller = Node(
        package='yahboom_g1_description',
        executable='simple_tank_movement.py',
        output='screen',
        name='movement_controller'
    )

    return LaunchDescription([
        robot_state_publisher,
        gazebo_launch,
        spawn_entity,
        movement_controller,
    ])
