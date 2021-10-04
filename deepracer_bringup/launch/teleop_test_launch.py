#################################################################################
#   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          #
#                                                                               #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#   You may not use this file except in compliance with the License.            #
#   You may obtain a copy of the License at                                     #
#                                                                               #
#       http://www.apache.org/licenses/LICENSE-2.0                              #
#                                                                               #
#   Unless required by applicable law or agreed to in writing, software         #
#   distributed under the License is distributed on an "AS IS" BASIS,           #
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.    #
#   See the License for the specific language governing permissions and         #
#   limitations under the License.                                              #
#################################################################################

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, RegisterEventHandler)
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    empty_world_name = 'empty_world'

    deepracer_description_dir = get_package_share_directory('deepracer_description')
    default_model_path = os.path.join(deepracer_description_dir, 'models/xacro/deepracer/deepracer.xacro')
    robot_xml = xacro.process_file(default_model_path)
    robot_description = {"robot_description": robot_xml.toxml()}

    # robot state publisher node
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )
    # spawn entity
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'deepracer', '-x', '0', '-y', '0', '-z', '0.03'],
                        output='screen')
    # controllers
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'joint_state_controller'],
        output='screen'
    )
    load_left_rear_wheel_velocity_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'left_rear_wheel_velocity_controller'],
        output='screen'
    )
    load_right_rear_wheel_velocity_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'right_rear_wheel_velocity_controller'],
        output='screen'
    )
    load_left_front_wheel_velocity_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'left_front_wheel_velocity_controller'],
        output='screen'
    )
    load_right_front_wheel_velocity_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'right_front_wheel_velocity_controller'],
        output='screen'
    )
    load_left_steering_hinge_position_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'left_steering_hinge_position_controller'],
        output='screen'
    )
    load_right_steering_hinge_position_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'right_steering_hinge_position_controller'],
        output='screen'
    )

    # ros gazebo launcher
    gazebo_dir = get_package_share_directory('gazebo_ros')
    gazebo_server_launcher = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
                launch_file_path=gazebo_dir + '/launch/gzserver.launch.py'),
        launch_arguments={'pause': 'false',
                          'record': 'false',
                          'verbose': 'false',
                          'physics': 'ode',
                          'world': empty_world_name}.items())

    gazebo_client_launcher = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            launch_file_path=gazebo_dir + '/launch/gzclient.launch.py'),
        launch_arguments={'verbose': 'false'}.items())

    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true'
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_left_rear_wheel_velocity_controller,
                         load_right_rear_wheel_velocity_controller,
                         load_left_front_wheel_velocity_controller,
                         load_right_front_wheel_velocity_controller,
                         load_left_steering_hinge_position_controller,
                         load_right_steering_hinge_position_controller],
            )
        ),
        gazebo_server_launcher,
        node_robot_state_publisher,
        spawn_entity,
        gazebo_client_launcher
        ])
