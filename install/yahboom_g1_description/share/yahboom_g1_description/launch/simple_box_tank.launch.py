from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_path = get_package_share_directory('yahboom_g1_description')
    urdf_file = os.path.join(pkg_path, 'urdf', 'yahboom_g1_simple_box.urdf')

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

    # Start Gazebo (simpler approach)
    gazebo = ExecuteProcess(
        cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', 'empty.world'],
        output='screen'
    )

    # Gazebo GUI
    gazebo_gui = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )

    # Spawn robot
    spawn_entity = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
            '-entity', 'simple_tank',
            '-file', urdf_file,
            '-x', '0', '-y', '0', '-z', '0.3'
        ],
        output='screen'
    )

    # Delay spawn
    delayed_spawn = TimerAction(
        period=10.0,
        actions=[spawn_entity]
    )

    return LaunchDescription([
        robot_state_publisher,
        gazebo,
        gazebo_gui,
        delayed_spawn,
    ])
