#  Copyright (c) 2026 Franka Robotics GmbH
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

import base64
from math import pi
import os
from typing import Any

from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import (
    OnProcessExit,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml

from launch import LaunchDescription, LaunchDescriptionEntity

LOWER_TORQUE_THRESHOLDS_ACCELERATION = [20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0]
LOWER_TORQUE_THRESHOLD_NOMINAL = [10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0]
LOWER_FORCE_THRESHOLDS_ACCELERATION = [20.0, 20.0, 20.0, 20.0, 20.0, 20.0]
LOWER_FORCE_THRESHOLDS_NOMINAL = [10.0, 10.0, 10.0, 10.0, 10.0, 10.0]

UPPER_TORQUE_THRESHOLDS_ACCELERATION = [40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0]
UPPER_TORQUE_THRESHOLDS_NOMINAL = [40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0]
UPPER_FORCE_THRESHOLDS_ACCELERATION = [40.0, 40.0, 40.0, 40.0, 40.0, 40.0]
UPPER_FORCE_THRESHOLDS_NOMINAL = [40.0, 40.0, 40.0, 40.0, 40.0, 40.0]

CONTROLLER_NAMES_KEY = 'controller_names'
TARGET_TOPIC_NAME_KEY = 'target_topic_name'
K_ALPHA_KEY = 'k_alpha'
K_GAINS_KEY = 'k_gains'
D_GAINS_KEY = 'd_gains'
MAX_TORQUE_ACCELERATION_KEY = 'upper_torque_thresholds_acceleration'
MAX_TORQUE_NOMINAL_KEY = 'upper_torque_thresholds_nominal'
MAX_FORCE_ACCELERATION_KEY = 'upper_force_thresholds_acceleration'
MAX_FORCE_NOMINAL_KEY = 'upper_force_thresholds_nominal'
LOAD_GRIPPER_KEY = 'load_gripper'
MOCK_HARDWARE_KEY = 'mock_hardware'
NAMESPACE_KEY = 'namespace'
ROBOT_IP_KEY = 'robot_ip'
ROBOT_TYPE_KEY = 'robot_type'
RESET_POSITION_KEY = 'reset_position'
SYNC_AFTER_ACTIVATION_KEY = 'sync_after_activation'
ACTIVATE_CONTROLLER_KEY = 'activate_controller'

default_parameters = {
    K_ALPHA_KEY: 0.99,
    K_GAINS_KEY: [600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0],
    D_GAINS_KEY: [30.0, 30.0, 30.0, 30.0, 10.0, 10.0, 5.0],
    MAX_TORQUE_ACCELERATION_KEY: UPPER_TORQUE_THRESHOLDS_ACCELERATION,
    MAX_TORQUE_NOMINAL_KEY: UPPER_TORQUE_THRESHOLDS_NOMINAL,
    MAX_FORCE_ACCELERATION_KEY: UPPER_FORCE_THRESHOLDS_ACCELERATION,
    MAX_FORCE_NOMINAL_KEY: UPPER_FORCE_THRESHOLDS_NOMINAL,
    RESET_POSITION_KEY: [0.0, -0.25 * pi, 0.0, -0.75 * pi, 0.0, 0.5 * pi, 0.25 * pi],
    LOAD_GRIPPER_KEY: False,
    MOCK_HARDWARE_KEY: False,
    ROBOT_TYPE_KEY: 'fr3',
    ROBOT_IP_KEY: 'you_need_to_configure_the_robot_ip',
    SYNC_AFTER_ACTIVATION_KEY: True,
    ACTIVATE_CONTROLLER_KEY: False,
}


def load_yaml(file_path: str) -> dict[str, Any]:
    """Load a YAML file and return its content as a dictionary."""
    if not os.path.exists(file_path):
        raise FileNotFoundError(f'File not found: {file_path}')
    with open(file_path) as file:
        return yaml.safe_load(file)


def get_known_controller_names(context) -> list[str]:
    """Load the known controller names from the follower_controllers.yaml file."""
    complete_controller_list = load_yaml(
        PathJoinSubstitution(
            [
                FindPackageShare('franka_follower_controllers'),
                'config',
                'follower_controllers.yaml',
            ]
        ).perform(context)
    )['/**'].keys()

    return [
        item
        for item in complete_controller_list
        if item
        not in ['controller_manager', 'franka_robot_state_broadcaster', 'joint_state_broadcaster']
    ]


def cvt_to_string(input_list: list) -> str:
    """Convert a list of numbers to a comma-separated string."""
    return ', '.join(map(str, map(float, input_list)))


def cvt_to_float_list(input_list: list) -> list[float]:
    """Convert a list of strings or numbers to a list of floats."""
    return list(map(float, input_list))


def create_ros2_control_parameter_file(config) -> str:
    """Create a YAML configuration file for the follower controllers."""
    random_string: str = base64.urlsafe_b64encode(os.urandom(6)).decode().lower()
    target_file_name = f'/tmp/launch_params_{random_string}'

    k_gains: list[float] = cvt_to_float_list(config[K_GAINS_KEY])
    d_gains: list[float] = cvt_to_float_list(config[D_GAINS_KEY])
    reset_position: list[float] = cvt_to_float_list(config[RESET_POSITION_KEY])

    robot_type = config.get(ROBOT_TYPE_KEY, 'fr3') or 'fr3'

    config_data = {
        '/**': {
            'joint_follower_controller': {
                'ros__parameters': {
                    'robot_type': robot_type,
                    'target_joint_states_topic_name': config[TARGET_TOPIC_NAME_KEY],
                    'k_gains': k_gains,
                    'd_gains': d_gains,
                    'k_alpha': config[K_ALPHA_KEY],
                    'sync_after_activation': config[SYNC_AFTER_ACTIVATION_KEY],
                }
            },
            'gravity_compensation_controller': {
                'ros__parameters': {
                    'robot_type': robot_type,
                }
            },
            'move_to_position_controller': {
                'ros__parameters': {
                    'start_joint_configuration': reset_position,
                }
            },
        }
    }

    with open(target_file_name, 'w') as param_file:
        param_file.write(yaml.dump(config_data, default_flow_style=False))

    return target_file_name


def add_ros2_control_launch_config(
    ros2_control_config: dict[str, Any],
) -> list[LaunchDescriptionEntity]:
    """Create the shared launch configuration for a single robot."""
    launch_config = []

    config_file = create_ros2_control_parameter_file(ros2_control_config)

    namespace = ros2_control_config.get(NAMESPACE_KEY, '')

    launch_config.append(
        ExecuteProcess(
            cmd=[
                [
                    FindExecutable(name='ros2'),
                    ' service call ',
                    f'{"" if not namespace else "/" + namespace}',
                    '/service_server/set_full_collision_behavior ',
                    'franka_msgs/srv/SetFullCollisionBehavior ',
                    '"{ ',
                    'lower_torque_thresholds_acceleration: ',
                    f'[{cvt_to_string(LOWER_TORQUE_THRESHOLDS_ACCELERATION)}], ',
                    'upper_torque_thresholds_acceleration: '
                    f'[{cvt_to_string(ros2_control_config[MAX_TORQUE_ACCELERATION_KEY])}], ',
                    'lower_torque_thresholds_nominal: ',
                    f'[{cvt_to_string(LOWER_TORQUE_THRESHOLD_NOMINAL)}], ',
                    'upper_torque_thresholds_nominal: ',
                    f'[{cvt_to_string(ros2_control_config[MAX_TORQUE_NOMINAL_KEY])}], ',
                    'lower_force_thresholds_acceleration: ',
                    f'[{cvt_to_string(LOWER_FORCE_THRESHOLDS_ACCELERATION)}], ',
                    'upper_force_thresholds_acceleration: ',
                    f'[{cvt_to_string(ros2_control_config[MAX_FORCE_ACCELERATION_KEY])}], ',
                    'lower_force_thresholds_nominal: ',
                    f'[{cvt_to_string(LOWER_FORCE_THRESHOLDS_NOMINAL)}], ',
                    'upper_force_thresholds_nominal: ',
                    f'[{cvt_to_string(ros2_control_config[MAX_FORCE_NOMINAL_KEY])}] ',
                    '}"',
                ]
            ],
            shell=True,
            name='set_robot_collision_behavior',
            output='both',
        )
    )

    launch_config.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare('franka_bringup'), 'launch', 'franka.launch.py']
                )
            ),
            launch_arguments={
                'robot_type': ros2_control_config[ROBOT_TYPE_KEY],
                'arm_prefix': namespace,
                'namespace': namespace,
                'robot_ip': ros2_control_config[ROBOT_IP_KEY],
                'load_gripper': str(ros2_control_config[LOAD_GRIPPER_KEY]),
                'use_fake_hardware': str(ros2_control_config[MOCK_HARDWARE_KEY]),
                'fake_sensor_commands': str(ros2_control_config[MOCK_HARDWARE_KEY]),
                'joint_state_rate': str(30),
                'controllers_yaml': PathJoinSubstitution(
                    [
                        FindPackageShare('franka_follower_controllers'),
                        'config',
                        'follower_controllers.yaml',
                    ]
                ),
            }.items(),
        )
    )

    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=namespace,
        arguments=[
            *ros2_control_config[CONTROLLER_NAMES_KEY],
            '--controller-manager-timeout',
            '30',
            '--inactive',
            '--param-file',
            config_file,
        ],
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare('franka_follower_controllers'),
                    'config',
                    'follower_controllers.yaml',
                ]
            )
        ],
        output='screen',
    )
    launch_config.append(controller_spawner)

    if ros2_control_config[ACTIVATE_CONTROLLER_KEY]:
        controller_to_start = ros2_control_config[CONTROLLER_NAMES_KEY][0]

        launch_config.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=controller_spawner,
                    on_exit=[
                        LogInfo(
                            msg=(
                                f'Controller spawner is done, starting controller: {controller_to_start}'
                            )
                        ),
                        ExecuteProcess(
                            cmd=[
                                [
                                    FindExecutable(name='ros2'),
                                    ' service call ',
                                    f'{"" if not namespace else "/" + namespace}',
                                    '/controller_manager/switch_controller ',
                                    'controller_manager_msgs/srv/SwitchController ',
                                    '"{ ',
                                    'activate_controllers: ',
                                    f'[{controller_to_start}], ',
                                    'deactivate_controllers: [], ',
                                    'strictness: 1, ',
                                    'activate_asap: false, ',
                                    "timeout: {'sec': 1, 'nanosec': 0} }\"",
                                ]
                            ],
                            shell=True,
                            name='start_controller_service_call',
                            output='both',
                        ),
                    ],
                )
            ),
        )

    return launch_config


def add_controller(context) -> list[LaunchDescriptionEntity]:
    """Create the launch configuration for a follower controller."""
    raw_config = load_yaml(LaunchConfiguration('config_file').perform(context))
    known_controller_names = get_known_controller_names(context)
    print(f'Known Controllers: {known_controller_names}')

    # Detect multi-robot format: top-level keys whose values are dicts containing robot_ip.
    # e.g. config_franka_robot.yml with LEFT: {...} and RIGHT: {...} sections.
    robot_sections = [
        v for v in raw_config.values()
        if isinstance(v, dict) and ROBOT_IP_KEY in v
    ]

    if robot_sections:
        # Multi-robot config: launch each section independently
        launch_entities = []
        for section_config in robot_sections:
            controller_config = default_parameters | section_config
            controller_names = controller_config.get(CONTROLLER_NAMES_KEY)
            if not controller_names:
                raise ValueError(
                    'Each controller configuration needs to specify a list of controller names!'
                )
            if not set(controller_names).issubset(known_controller_names):
                raise ValueError(
                    f"Unknown controllers in configuration file (property '{CONTROLLER_NAMES_KEY}')!"
                    f' Known controllers are: {known_controller_names}'
                )
            launch_entities.extend(add_ros2_control_launch_config(controller_config))
        return launch_entities
    else:
        # Single-robot flat config (backwards compatible)
        controller_config = default_parameters | raw_config
        controller_names = controller_config.get(CONTROLLER_NAMES_KEY)
        if not controller_names:
            raise ValueError(
                'Each controller configuration needs to specify a list of controller names!'
            )
        if not set(controller_names).issubset(known_controller_names):
            raise ValueError(
                f"Unknown controllers in configuration file (property '{CONTROLLER_NAMES_KEY}')!"
                f' Known controllers are: {known_controller_names}'
            )
        return add_ros2_control_launch_config(controller_config)


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'config_file',
                description='Path to the robot configuration file to load',
            ),
            OpaqueFunction(function=add_controller),
        ]
    )
