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
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    cfg_pkg_prefix = FindPackageShare('f1tenth_launch')

    system_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution([
                FindPackageShare('tier4_system_launch'), 'launch', 'system.launch.xml'
            ]),
        ),
        launch_arguments={
            'run_mode': LaunchConfiguration('system_run_mode'),
            'launch_system_monitor': LaunchConfiguration('launch_system_monitor'),
            'launch_dummy_diag_publisher': LaunchConfiguration('launch_dummy_diag_publisher'),
            'sensor_model': LaunchConfiguration('sensor_model'),
            'component_state_monitor_topic_path': PathJoinSubstitution([cfg_pkg_prefix, 'config/system/component_state_monitor/topics.yaml']),
            'emergency_handler_param_path': PathJoinSubstitution([cfg_pkg_prefix, 'config/system/emergency_handler/emergency_handler.param.yaml']),
            'duplicated_node_checker_param_path': PathJoinSubstitution([cfg_pkg_prefix, 'config/system/duplicated_node_checker/duplicated_node_checker.param.yaml']),
            'mrm_comfortable_stop_operator_param_path': PathJoinSubstitution([cfg_pkg_prefix, 'config/system/mrm_comfortable_stop_operator/mrm_comfortable_stop_operator.param.yaml']),
            'mrm_emergency_stop_operator_param_path': PathJoinSubstitution([cfg_pkg_prefix, 'config/system/mrm_emergency_stop_operator/mrm_emergency_stop_operator.param.yaml']),
            'system_error_monitor_param_path': PathJoinSubstitution([cfg_pkg_prefix, 'config/system/system_error_monitor/system_error_monitor.param.yaml']),
            'system_error_monitor_planning_simulator_param_path': PathJoinSubstitution([cfg_pkg_prefix, 'config/system/system_error_monitor/system_error_monitor.planning_simulation.param.yaml']),
            'diagnostic_aggregator_vehicle_param_path': PathJoinSubstitution([cfg_pkg_prefix, 'config/system/system_error_monitor/diagnostic_aggregator/vehicle.param.yaml']),
            'diagnostic_aggregator_system_param_path': PathJoinSubstitution([cfg_pkg_prefix, 'config/system/system_error_monitor/diagnostic_aggregator/system.param.yaml']),
            'dummy_diag_publisher_param_path': PathJoinSubstitution([cfg_pkg_prefix, 'config/system/dummy_diag_publisher/dummy_diag_publisher.param.yaml']),
            'system_monitor_cpu_monitor_param_path': PathJoinSubstitution([cfg_pkg_prefix, 'config/system/system_monitor/cpu_monitor.param.yaml']),
            'system_monitor_gpu_monitor_param_path': PathJoinSubstitution([cfg_pkg_prefix, 'config/system/system_monitor/gpu_monitor.param.yaml']),
            'system_monitor_hdd_monitor_param_path': PathJoinSubstitution([cfg_pkg_prefix, 'config/system/system_monitor/hdd_monitor.param.yaml']),
            'system_monitor_mem_monitor_param_path': PathJoinSubstitution([cfg_pkg_prefix, 'config/system/system_monitor/mem_monitor.param.yaml']),
            'system_monitor_net_monitor_param_path': PathJoinSubstitution([cfg_pkg_prefix, 'config/system/system_monitor/net_monitor.param.yaml']),
            'system_monitor_ntp_monitor_param_path': PathJoinSubstitution([cfg_pkg_prefix, 'config/system/system_monitor/ntp_monitor.param.yaml']),
            'system_monitor_process_monitor_param_path': PathJoinSubstitution([cfg_pkg_prefix, 'config/system/system_monitor/process_monitor.param.yaml']),
            'system_monitor_voltage_monitor_param_path': PathJoinSubstitution([cfg_pkg_prefix, 'config/system/system_monitor/voltage_monitor.param.yaml'])
        }.items()
    )

    return [
        system_launch
    ]


def generate_launch_description():
    declared_arguments = []

    def add_launch_arg(name: str, default_value: str = None):
        declared_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value)
        )

    add_launch_arg('sensor_model')
    add_launch_arg('system_run_mode', 'online')
    add_launch_arg('launch_system_monitor', 'true')
    add_launch_arg('launch_dummy_diag_publisher', 'false')

    return LaunchDescription([
        *declared_arguments,
        OpaqueFunction(function=launch_setup)
    ])
