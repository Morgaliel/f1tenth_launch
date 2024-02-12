# Copyright 2024 The Autoware Contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    pkg_prefix = FindPackageShare('f1tenth_launch')

    f1tenth_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution(
                [pkg_prefix, 'launch', 'f1tenth.launch.py']
            )
        ),
        launch_arguments={
            'vehicle_model': LaunchConfiguration('vehicle_model'),
            'sensor_model': LaunchConfiguration('sensor_model'),
            'map_path': LaunchConfiguration('map_path'),
            'mapping': LaunchConfiguration('mapping'),
            'launch_vehicle_interface': 'false',
            'launch_sensing_driver': 'false',
            'launch_system_monitor': 'false',
            'launch_dummy_diag_publisher': 'false',
            'use_trajectory_loader': LaunchConfiguration('use_trajectory_loader'),
            'use_sim_time': 'true'
        }.items()
    )

    return [
        f1tenth_launch
    ]


def generate_launch_description():
    declared_arguments = []

    def add_launch_arg(name: str, default_value: str = None):
        declared_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value)
        )

    add_launch_arg('vehicle_model', 'f1tenth_vehicle')
    add_launch_arg('sensor_model', 'f1tenth_sensor_kit')
    add_launch_arg('map_path', PathJoinSubstitution([EnvironmentVariable('HOME'), 'autoware_map/race_track_01']))
    add_launch_arg('mapping', 'false')
    add_launch_arg('use_trajectory_loader', 'true')

    return LaunchDescription([
        *declared_arguments,
        OpaqueFunction(function=launch_setup)
    ])
