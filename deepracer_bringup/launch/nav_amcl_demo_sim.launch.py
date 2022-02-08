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
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

# demo of deepracer simulation in aws bookstore world with navigation and amcl
def generate_launch_description():
    deepracer_bringup_dir = get_package_share_directory('deepracer_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    bookstore_path = get_package_share_directory('aws_robomaker_bookstore_world')
    bookstore_world = os.path.join(bookstore_path, 'worlds', 'bookstore.world')
    bookstore_map = os.path.join(bookstore_path, 'maps', 'turtlebot3_waffle_pi', 'map.yaml')
    nav_params = os.path.join(deepracer_bringup_dir, 'config', 'nav2_params_nav_amcl_sim_demo.yaml')

    world_cfg = LaunchConfiguration('world')
    map_cfg = LaunchConfiguration('map')
    params_cfg = LaunchConfiguration('params')

    declare_world_arg = DeclareLaunchArgument('world', default_value=bookstore_world, description='SDF world file')
    declare_map_arg = DeclareLaunchArgument('map', default_value=bookstore_map, description='map file')
    declare_params_arg = DeclareLaunchArgument('params', default_value=nav_params, description='params file')

    include_files = GroupAction([
        # start deepracer simulation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([deepracer_bringup_dir, '/launch/deepracer_sim.launch.py']),
            launch_arguments = {'world': world_cfg}.items()
         ),
        # start navigation planner and controller
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([deepracer_bringup_dir, '/launch/deepracer_navigation_sim.launch.py']),
            launch_arguments = {'params': params_cfg}.items()
        ),
        # start localization (amcl) and map_server
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_bringup_dir, '/launch/localization_launch.py']),
            launch_arguments={'map': map_cfg,
                              'params_file': params_cfg}.items()),
    ])

    ld = LaunchDescription()
    ld.add_action(declare_world_arg)
    ld.add_action(declare_map_arg)
    ld.add_action(declare_params_arg)
    ld.add_action(include_files)

    return ld
