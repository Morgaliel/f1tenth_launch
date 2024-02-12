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
from launch.actions import ExecuteProcess
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.substitutions import FindExecutable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace


def launch_setup(context, *args, **kwargs):
    pkg_prefix = FindPackageShare('f1tenth_launch')
    localization_param_dir = PathJoinSubstitution(
        [pkg_prefix, 'config/localization'])

    ekf_param_file = PathJoinSubstitution(
        [localization_param_dir, 'ekf_localizer.param.yaml'])
    
    pose_twist_fusion_param_file = PathJoinSubstitution(
        [localization_param_dir, 'pose_twist_fusion.param.yaml'])

    localization_error_monitor_param_file = PathJoinSubstitution(
        [localization_param_dir, 'localization_error_monitor.param.yaml'])
    
    twist2accel_param_file = PathJoinSubstitution(
        [localization_param_dir, 'twist2accel.param.yaml'])
    
    configuration_basename = 'localization.lua'
    load_state_filename = PathJoinSubstitution([LaunchConfiguration('map_path'), 'map.pbstream']).perform(context)
    if IfCondition(LaunchConfiguration('mapping')).evaluate(context):
        configuration_basename = 'mapping.lua'
        load_state_filename = ''

    gyro_odometer_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution(
                [FindPackageShare('gyro_odometer'), 'launch', 'gyro_odometer.launch.xml']
            )
        ),
        launch_arguments={
            'input_vehicle_twist_with_covariance_topic': '/sensing/vehicle_velocity_converter/twist_with_covariance',
            'input_imu_topic': '/sensing/imu_corrector/imu',
            'output_twist_with_covariance_topic': 'twist_estimator/twist_with_covariance',
            'output_twist_with_covariance_raw_topic': 'twist_estimator/twist_with_covariance_raw',
            'output_twist_topic': 'twist_estimator/twist',
            'output_twist_raw_topic': 'twist_estimator/twist_raw'
        }.items()
    )

    ekf_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution([
                FindPackageShare('ekf_localizer'), 'launch', 'ekf_localizer.launch.xml'
            ]),
        ),
        launch_arguments={
            'input_initial_pose_name': '/initialpose',
            'input_pose_with_cov_name': 'pose_estimator/pose_with_covariance',
            'input_twist_with_cov_name': 'twist_estimator/twist_with_covariance',
            'output_odom_name': 'pose_twist_fusion_filter/kinematic_state',
            'output_pose_name': 'pose_twist_fusion_filter/pose',
            'output_pose_with_covariance_name': 'pose_twist_fusion_filter/pose_with_covariance',
            'output_biased_pose_name': 'pose_twist_fusion_filter/biased_pose',
            'output_biased_pose_with_covariance_name': 'pose_twist_fusion_filter/biased_pose_with_covariance',
            'output_twist_name': 'pose_twist_fusion_filter/twist',
            'output_twist_with_covariance_name': 'pose_twist_fusion_filter/twist_with_covariance',
            'param_file': ekf_param_file
        }.items()
    )

    localization_error_monitor_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution([
                FindPackageShare('localization_error_monitor'), 'launch', 'localization_error_monitor.launch.xml'
            ]),
        ),
        launch_arguments={
            'input/odom': '/localization/kinematic_state',
            'param_file': localization_error_monitor_param_file
        }.items()
    )

    stop_filter_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution(
                [FindPackageShare('stop_filter'), 'launch', 'stop_filter.launch.xml']
            )
        ),
        launch_arguments={
            'use_twist_with_covariance': 'true',
            'input_odom_name': 'pose_twist_fusion_filter/kinematic_state',
            'input_twist_with_covariance_name': 'pose_twist_fusion_filter/twist_with_covariance',
            'output_odom_name': 'stop_filter/kinematic_state'
        }.items()
    )

    twist2accel_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution([
                FindPackageShare('twist2accel'), 'launch', 'twist2accel.launch.xml'
            ]),
        ),
        launch_arguments={
            'in_odom': 'kinematic_state',
            'in_twist': 'twist_estimator/twist_with_covariance',
            'out_accel': 'acceleration',
            'param_file': twist2accel_param_file
        }.items()
    )

    cartographer_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_node',
        arguments = [
            '-configuration_directory', localization_param_dir,
            '-configuration_basename', configuration_basename,
            '-load_state_filename', load_state_filename,
            '--ros-args', '--log-level', 'error', '--enable-stdout-logs'
        ],
        remappings = [
            ('imu', '/sensing/imu_corrector/imu'),
            ('odom', '/localization/stop_filter/kinematic_state'),
            ('scan', '/sensing/lidar/scan'),
            ('tracked_pose', '/localization/cartographer/pose')
        ],
    )

    cartographer_occupancy_grid_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_occupancy_grid_node',
        parameters = [
            {
                'resolution': 0.05
            }
        ],
        remappings = [
            ('map', '/map'),
        ],
    )

    pose_twist_fusion_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution(
                [FindPackageShare('cartographer_utils'), 'launch', 'pose_twist_fusion.launch.py']
            )
        ),
        launch_arguments={
            'pose_twist_fusion_param_file': pose_twist_fusion_param_file,
            'input_pose': '/localization/cartographer/pose',
            'input_twist': '/localization/twist_estimator/twist',
            'input_pose_with_covariance': '/localization/pose_estimator/pose_with_covariance',
            'input_twist_with_covariance': '/localization/twist_estimator/twist_with_covariance',
            'output_odometry': '/localization/kinematic_state'
        }.items()
    )

    pose_initializer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution(
                [FindPackageShare('cartographer_utils'), 'launch', 'pose_initializer.launch.py']
            )
        ),
        launch_arguments={
            'configuration_directory': localization_param_dir,
            'configuration_basename': configuration_basename,
            'input_get_trajectory_states_service_name': '/localization/get_trajectory_states',
            'input_finish_trajectory_service_name': '/localization/finish_trajectory',
            'input_start_trajectory_service_name': '/localization/start_trajectory'
        }.items()
    )

    localization_trigger = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' service call /localization/trigger_node std_srvs/srv/SetBool "{data: true}"'
        ]],
        shell=True
    )

    init_localization = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' topic pub /api/localization/initialization_state autoware_adapi_v1_msgs/msg/LocalizationInitializationState "{stamp: now, state: 3}"'
        ]],
        shell=True
    )

    delayed_services_trigger = TimerAction(
        period=5.0,
        actions=[localization_trigger, init_localization]
    )

    group = GroupAction(
        [
            PushRosNamespace('localization'),
            gyro_odometer_launch,
            ekf_launch,
            localization_error_monitor_launch,
            stop_filter_launch,
            twist2accel_launch,
            cartographer_node,
            cartographer_occupancy_grid_node,
            pose_twist_fusion_launch,
            pose_initializer_launch
        ]
    )

    return [
        group,
        delayed_services_trigger
    ]


def generate_launch_description():
    declared_arguments = []

    def add_launch_arg(name: str, default_value: str = None):
        declared_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value)
        )

    add_launch_arg('map_path')
    add_launch_arg('mapping')

    return LaunchDescription([
        *declared_arguments,
        OpaqueFunction(function=launch_setup)
    ])
