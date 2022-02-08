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
from launch.substitutions import Command
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    deepracer_description_dir = get_package_share_directory('deepracer_description')
    deepracer_bringup_dir = get_package_share_directory('deepracer_bringup')
    default_model_path = os.path.join(deepracer_description_dir, 'models/xacro/deepracer/deepracer.xacro')

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', default_model_path])}]
    )

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

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'deepracer', '-x', '0', '-y', '0', '-z', '0.03'],
                        output='screen')

    return LaunchDescription([RegisterEventHandler(
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
        robot_state_publisher_node,
        spawn_entity,
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0.136966', '0', '0.143272', '0', '0.2618', '0', 'base_link', 'camera_link'],
            parameters=[
                deepracer_bringup_dir + '/config/static_tf.yaml'
            ]
            ),
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0.02913', '0', '0.884699', '0', '0', '3.1416', 'base_link', 'laser'],
            parameters=[
                deepracer_bringup_dir + '/config/static_tf_sim.yaml'
            ]
            )
    ])
