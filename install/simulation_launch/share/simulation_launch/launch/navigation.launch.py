import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # --- Include MITI Gazebo simulation ---
    gazebo_launch = os.path.join(
        get_package_share_directory('roverrobotics_gazebo'),
        'launch', 'miti_gazebo.launch.py'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
        launch_arguments={'world': 'empty_world.sdf'}.items()
    )

    # --- Path smoothing node ---
    path_smoother = Node(
        package='path_smoothing',
        executable='smoothing_node',
        name='path_smoother',
        output='screen'
    )

    # --- Trajectory generation node ---
    trajectory_gen = Node(
        package='trajectory_generation',
        executable='trajectory_node',
        name='trajectory_generator',
        output='screen'
    )

    # --- PID trajectory tracking node ---
    tracking_ctrl = Node(
        package='trajectory_tracking',
        executable='tracking_node',
        name='trajectory_controller',
        output='screen'
    )

    # --- Assemble everything ---
    return LaunchDescription([
        gazebo,
        path_smoother,
        trajectory_gen,
        tracking_ctrl,
    ])

