"""Starts the Prius simulation, the Pure Pursuit controller and (optionally)
a path recorder that logs the executed trajectory.

ros2 launch pure_pursuit_controller pure_pursuit.launch.py
ros2 launch pure_pursuit_controller pure_pursuit.launch.py lookahead_gain:=0.3 run_tag:=k02
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('pure_pursuit_controller')
    params_file = os.path.join(pkg_share, 'config', 'pure_pursuit.yaml')

    launch_sim = LaunchConfiguration('launch_sim')
    record = LaunchConfiguration('record_trajectory')
    waypoints = LaunchConfiguration('waypoints_file')
    run_tag = LaunchConfiguration('run_tag')
    data_dir = LaunchConfiguration('data_dir')

    args = [
        DeclareLaunchArgument('launch_sim', default_value='true',
                              description='Start Gazebo with the Prius world'),
        DeclareLaunchArgument('record_trajectory', default_value='true',
                              description='Log the executed path to CSV'),
        DeclareLaunchArgument('data_dir', default_value=os.path.expanduser('~/pp_data')),
        DeclareLaunchArgument('waypoints_file',
                              default_value=os.path.expanduser('~/pp_data/waypoints.csv')),
        DeclareLaunchArgument('run_tag', default_value='',
                              description='Suffix for the output CSV, e.g. k04'),
        DeclareLaunchArgument('target_speed', default_value='4.0'),
        DeclareLaunchArgument('lookahead_min', default_value='3.0'),
        DeclareLaunchArgument('lookahead_gain', default_value='0.4'),
        DeclareLaunchArgument('start_delay', default_value='6.0',
                              description='Seconds to wait for Gazebo before starting nodes'),
    ]

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('prius_bringup'), 'launch', 'gz_sim.launch.py')),
        condition=IfCondition(launch_sim))

    controller = Node(
        package='pure_pursuit_controller',
        executable='pure_pursuit_node',
        name='pure_pursuit_node',
        output='screen',
        parameters=[params_file, {
            'use_sim_time': True,
            'waypoints_file': waypoints,
            'target_speed': ParameterValue(
                LaunchConfiguration('target_speed'), value_type=float),
            'lookahead_min': ParameterValue(
                LaunchConfiguration('lookahead_min'), value_type=float),
            'lookahead_gain': ParameterValue(
                LaunchConfiguration('lookahead_gain'), value_type=float),
        }])

    output_csv = PythonExpression([
        "'", data_dir, "/actual_trajectory' + ('_' + '", run_tag,
        "' if '", run_tag, "' else '') + '.csv'"])

    recorder = Node(
        package='pure_pursuit_controller',
        executable='path_recorder',
        name='path_recorder',
        output='screen',
        condition=IfCondition(record),
        parameters=[params_file, {
            'use_sim_time': True,
            'output_file': output_csv,
            'save_time': True,
        }])

    # recorder first, controller right after, both once Gazebo is up
    nodes = TimerAction(period=LaunchConfiguration('start_delay'),
                        actions=[recorder, controller])

    return LaunchDescription(args + [sim, nodes])
