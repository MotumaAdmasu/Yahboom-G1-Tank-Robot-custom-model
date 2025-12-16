from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # Get package path
    pkg_path = get_package_share_directory('yahboom_g1_description')
    urdf_file = os.path.join(pkg_path, 'urdf', 'yahboom_g1_simple.urdf')

    # Start Gazebo
    gazebo_cmd = ExecuteProcess(
        cmd=['gzserver', '--verbose', 'empty.world'],
        output='screen'
    )

    # Start Gazebo client
    gazebo_gui = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )

    # Spawn robot in Gazebo
    spawn_entity_cmd = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
            '-entity', 'yahboom_g1',
            '-file', urdf_file,
            '-x', '0', '-y', '0', '-z', '0.1'
        ],
        output='screen'
    )

    # Delay the spawn to ensure Gazebo is ready
    delayed_spawn = TimerAction(
        period=8.0,
        actions=[spawn_entity_cmd]
    )

    return LaunchDescription([
        gazebo_cmd,
        gazebo_gui,
        delayed_spawn,
    ])
