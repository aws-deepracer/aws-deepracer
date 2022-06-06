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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    deepracer_bringup_dir = get_package_share_directory('deepracer_bringup')
    return LaunchDescription([
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
            arguments=['0.02913', '0', '0.184699', '3.1416', '0', '0', 'base_link', 'laser_frame'],
            parameters=[
                deepracer_bringup_dir + '/config/static_tf.yaml']),


        launch_ros.actions.Node(
            package='camera_pkg',
            namespace='camera_pkg',
            executable='camera_node',
            name='camera_node',
            parameters=[
                {'resize_images': False}
            ]
            ),

        launch_ros.actions.Node(
            package='rplidar_ros2',
            namespace='',
            executable='rplidar_scan_publisher',
            name='rplidar_scan_publisher',
            parameters=[{
                    'serial_port': '/dev/ttyUSB0',
                    'serial_baudrate': 115200,
                    'frame_id': 'laser_frame',
                    'inverted': False,
                    'angle_compensate': True,
                }]
                ),

        launch_ros.actions.Node(
            package='servo_pkg',
            namespace='servo_pkg',
            executable='servo_node',
            name='servo_node',
            remappings=[('/ctrl_pkg/servo_msg', '/cmdvel_to_servo_pkg/servo_msg')]
            ),

        launch_ros.actions.Node(
            package='cmdvel_to_servo_pkg',
            namespace='cmdvel_to_servo_pkg',
            executable='cmdvel_to_servo_node',
            name='cmdvel_to_servo_node'
            ),

        launch_ros.actions.Node(
            package='enable_deepracer_nav_pkg',
            namespace='enable_deepracer_nav_pkg',
            executable='enable_deepracer_nav_node',
            name='enable_deepracer_nav_node'
            ),

        launch_ros.actions.Node(
                package='rf2o_laser_odometry',
                executable='rf2o_laser_odometry_node',
                name='rf2o_laser_odometry',
                output='screen',
                parameters=[{
                    'laser_scan_topic': '/scan',
                    'odom_topic': '/odom',
                    'publish_tf': True,
                    'base_frame_id': 'base_link',
                    'odom_frame_id': 'odom',
                    'init_pose_from_topic': '',
                    'freq': 20.0}],
            )

    ])
