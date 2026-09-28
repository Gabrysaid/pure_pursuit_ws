"""Step 1: start the Prius simulation and the path recorder.
Drive with teleop in another terminal (it needs its own keyboard):

  ros2 run teleop_twist_keyboard teleop_twist_keyboard

Ctrl+C here writes ~/pp_data/waypoints.csv
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('pure_pursuit_controller')
    params_file = os.path.join(pkg_share, 'config', 'pure_pursuit.yaml')

    args = [
        DeclareLaunchArgument('launch_sim', default_value='true'),
        DeclareLaunchArgument('output_file',
                              default_value=os.path.expanduser('~/pp_data/waypoints.csv')),
        DeclareLaunchArgument('min_distance', default_value='0.5'),
    ]

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('prius_bringup'), 'launch', 'gz_sim.launch.py')),
        condition=IfCondition(LaunchConfiguration('launch_sim')))

    recorder = Node(
        package='pure_pursuit_controller',
        executable='path_recorder',
        name='path_recorder',
        output='screen',
        parameters=[params_file, {
            'use_sim_time': True,
            'output_file': LaunchConfiguration('output_file'),
            'min_distance': ParameterValue(LaunchConfiguration('min_distance'), value_type=float),
        }])

    return LaunchDescription(args + [sim, recorder])
