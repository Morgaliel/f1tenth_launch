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
from launch.conditions import IfCondition
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    vehicle_model = LaunchConfiguration('vehicle_model').perform(context)
    sensor_model = LaunchConfiguration('sensor_model').perform(context)
    if (LaunchConfigurationEquals('config_dir', '').evaluate(context)):
        config_dir = PathJoinSubstitution([FindPackageShare(sensor_model + '_description'), 'config']).perform(context)
    else:
        config_dir = LaunchConfiguration('config_dir').perform(context)

    vehicle_launch_pkg = LaunchConfiguration('vehicle_model').perform(context) + '_launch'
    vehicle_model_file = PathJoinSubstitution([FindPackageShare('tier4_vehicle_launch'), 'urdf', 'vehicle.xacro']).perform(context)

    vehicle_description_node = Node(
        name='robot_state_publisher',
        namespace='',
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {
                'robot_description': ParameterValue(Command(
                    [f'xacro {vehicle_model_file} vehicle_model:={vehicle_model} sensor_model:={sensor_model} config_dir:={config_dir}']
                ), value_type=str)
            }
        ]
    )

    vehicle_interface_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution(
                [FindPackageShare(vehicle_launch_pkg), 'launch', 'vehicle_interface.launch.py']
            ),
        ),
        launch_arguments={
            'vehicle_param_file': PathJoinSubstitution([FindPackageShare(LaunchConfiguration('vehicle_model').perform(context) + '_description'), 'config/vehicle_info.param.yaml'])
        }.items(),
        condition=IfCondition(LaunchConfiguration('launch_vehicle_interface'))
    )

    return [
        vehicle_description_node,
        vehicle_interface_launch
    ]


def generate_launch_description():
    declared_arguments = []

    def add_launch_arg(name: str, default_value: str = None):
        declared_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value)
        )

    add_launch_arg('vehicle_model')
    add_launch_arg('sensor_model')
    add_launch_arg('launch_vehicle_interface')
    add_launch_arg('config_dir', '')

    return LaunchDescription([
        *declared_arguments,
        OpaqueFunction(function=launch_setup)
    ])
