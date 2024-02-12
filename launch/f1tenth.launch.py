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
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    pkg_prefix = FindPackageShare('f1tenth_launch')
    rviz_config = PathJoinSubstitution([pkg_prefix, 'rviz', LaunchConfiguration('rviz_config')])
    vehicle_param_file = PathJoinSubstitution([FindPackageShare(LaunchConfiguration(
        'vehicle_model').perform(context) + '_description'), 'config/vehicle_info.param.yaml'])
    if not LaunchConfiguration('map_path').perform(context) and UnlessCondition(LaunchConfiguration('mapping')).evaluate(context):
        raise RuntimeError(
            'map_path is not set. If you consider mapping mode, please use arg mapping:=True')

    # Global parameters
    global_parameter_loader_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution(
                [FindPackageShare('global_parameter_loader'), 'launch', 'global_params.launch.py']
            )
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'vehicle_model': LaunchConfiguration('vehicle_model'),
        }.items()
    )

    # Vehicle
    vehicle_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution(
                [pkg_prefix, 'launch/components', 'f1tenth_vehicle.launch.py']
            )
        ),
        launch_arguments={
            'vehicle_model': LaunchConfiguration('vehicle_model'),
            'sensor_model': LaunchConfiguration('sensor_model'),
            'launch_vehicle_interface': LaunchConfiguration('launch_vehicle_interface'),
            'config_dir': PathJoinSubstitution([FindPackageShare(LaunchConfiguration('sensor_model').perform(context) + '_description'), 'config'])
        }.items(),
        condition=IfCondition(LaunchConfiguration('launch_vehicle'))
    )

    # System
    system_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution(
                [pkg_prefix, 'launch/components', 'f1tenth_system.launch.py']
            )
        ),
        launch_arguments={
            'sensor_model': LaunchConfiguration('sensor_model'),
            'system_run_mode': LaunchConfiguration('system_run_mode'),
            'launch_system_monitor': LaunchConfiguration('launch_system_monitor'),
            'launch_dummy_diag_publisher': LaunchConfiguration('launch_dummy_diag_publisher')
        }.items(),
        condition=IfCondition(LaunchConfiguration('launch_system'))
    )

    # Sensing
    sensing_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution(
                [pkg_prefix, 'launch/components', 'f1tenth_sensing.launch.py']
            )
        ),
        launch_arguments={
            'sensor_model': LaunchConfiguration('sensor_model'),
            'vehicle_param_file': vehicle_param_file,
            'launch_sensing_driver': LaunchConfiguration('launch_sensing_driver')
        }.items(),
        condition=IfCondition(LaunchConfiguration('launch_sensing'))
    )

    # Localization
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution(
                [pkg_prefix, 'launch/components', 'f1tenth_localization.launch.py']
            )
        ),
        launch_arguments={
            'map_path': LaunchConfiguration('map_path'),
            'mapping': LaunchConfiguration('mapping')
        }.items(),
        condition=IfCondition(LaunchConfiguration('launch_localization'))
    )

    # TODO(amadeuszsz): integrate perception pipeline
    # Perception

    # Planning
    planning_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution(
                [pkg_prefix,
                 'launch/components', 'f1tenth_planning.launch.py'])),
        launch_arguments={
            'map_path': LaunchConfiguration('map_path'),
            'use_trajectory_loader': LaunchConfiguration('use_trajectory_loader')
        }.items(),
        condition=IfCondition(LaunchConfiguration('launch_planning'))
    )

    # Control
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file_path=PathJoinSubstitution(
            [pkg_prefix, 'launch/components', 'f1tenth_control.launch.py'])),
        launch_arguments={
            'lateral_controller_mode': 'pure_pursuit',
            'longitudinal_controller_mode': 'pid',
            'vehicle_param_file': vehicle_param_file,
            'nearest_search_param_path': PathJoinSubstitution([pkg_prefix, 'config/control/common/nearest_search.param.yaml']),
            'vehicle_cmd_gate_param_path': PathJoinSubstitution([pkg_prefix, 'config/control/vehicle_cmd_gate/vehicle_cmd_gate.param.yaml']),
            'control_validator_param_path': PathJoinSubstitution([pkg_prefix, 'config/control/control_validator/control_validator.param.yaml']),
            'operation_mode_transition_manager_param_path': PathJoinSubstitution([pkg_prefix, 'config/control/operation_mode_transition_manager/operation_mode_transition_manager.param.yaml']),
            'external_cmd_selector_param_path': PathJoinSubstitution([pkg_prefix, 'config/control/external_cmd_selector/external_cmd_selector.param.yaml']),
            'trajectory_follower_node_param_path': PathJoinSubstitution([pkg_prefix, 'config/control/trajectory_follower/trajectory_follower_node.param.yaml']),
            'latlon_controller_param_path_dir': PathJoinSubstitution([pkg_prefix, 'config/control/trajectory_follower']),
            'shift_decider_param_path': PathJoinSubstitution([pkg_prefix, 'config/control/shift_decider/shift_decider.param.yaml']),
            'csv_path_map': PathJoinSubstitution([FindPackageShare(LaunchConfiguration('vehicle_model').perform(context) + '_launch'), 'data'])
        }.items(),
        condition=IfCondition(LaunchConfiguration('launch_control'))
    )

    # API
    autoware_api_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution(
                [pkg_prefix, 'launch/components', 'f1tenth_autoware_api.launch.py']
            )
        ),
        condition=IfCondition(LaunchConfiguration('launch_vehicle'))
    )

    # Tools
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d', str(rviz_config.perform(context)), '-s',
            str(PathJoinSubstitution(
                [pkg_prefix, 'rviz/image/autoware_f1tenth.png']).perform(context)),
            '--ros-args', '--log-level', 'error', '--enable-stdout-logs'
        ],
        respawn=LaunchConfiguration('rviz_respawn'),
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return [
        global_parameter_loader_launch,
        vehicle_launch,
        system_launch,
        sensing_launch,
        planning_launch,
        localization_launch,
        control_launch,
        autoware_api_launch,
        rviz2
    ]


def generate_launch_description():
    declared_arguments = []

    def add_launch_arg(name: str, default_value: str = None):
        declared_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value)
        )

    # Essential parameters
    add_launch_arg('vehicle_model')
    add_launch_arg('sensor_model')
    add_launch_arg('map_path', '')

    # Optional parameters
    # Modules to be launched
    add_launch_arg('launch_vehicle', 'true')
    add_launch_arg('launch_system', 'true')
    add_launch_arg('launch_sensing', 'true')
    add_launch_arg('launch_sensing_driver', 'true')
    add_launch_arg('launch_localization', 'true')
    add_launch_arg('launch_perception', 'false')
    add_launch_arg('launch_planning', 'true')
    add_launch_arg('launch_control', 'true')
    # Global parameters
    add_launch_arg('use_sim_time', 'false')
    add_launch_arg('use_intra_process', 'false')
    add_launch_arg('use_multithread', 'false')
    # Vehicle
    add_launch_arg('launch_vehicle_interface', 'true')
    # System
    add_launch_arg('system_run_mode', 'online')
    add_launch_arg('launch_system_monitor', 'true')
    add_launch_arg('launch_dummy_diag_publisher', 'false')
    # Tools
    add_launch_arg('rviz', 'true')
    add_launch_arg('rviz_config', 'f1tenth.rviz')
    add_launch_arg('rviz_respawn', 'true')
    # Localization
    add_launch_arg('mapping', 'false')
    # Planning
    add_launch_arg('use_trajectory_loader', 'true')

    return LaunchDescription([
        *declared_arguments,
        OpaqueFunction(function=launch_setup)
    ])
