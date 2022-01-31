# AWS DeepRacer Bringup

## Overview

The AWS DeepRacer repository contains the configuration and launch files to enable ROS Navigation Stack on AWS DeepRacer and control the vehicle using teleop-twist-keyboard, along with the core components to integrate AWS DeepRacer with ROS Navigation stack. For detailed information, see [Introduction to the ROS Navigation stack using AWS DeepRacer Evo](https://github.com/aws-deepracer/aws-deepracer/blob/main/introduction-to-the-ros-navigation-stack-using-aws-deepracer-evo.md).

The **deepracer_bringup** package consists of the launch files required to launch ROS Navigation simulation/device flow along with the recommended config files required for the launch. The config files can be modified as per requirement.

## License

The source code is released under [Apache 2.0](https://aws.amazon.com/apache-2-0/).

## Config Files

1. `agent_control.yaml`: Controller manager parameters for agent in simulation environment.

1. `nav2_params_nav_amcl_sim_demo.yaml`: Sample configuration parameters to launch Nav2 stack in simulation environment.

1. `nav2_params_nav_amcl_dr_demo.yaml`: Sample configuration parameters to launch Nav2 stack on device.

1. `nav2_slam_params.yaml`: Sample configuration parameters to launch Nav2 stack on device with SLAM toolbox.

1. `slam_toolbox.yaml`: Sample configuration parameters to launch Nav2 SLAM toolbox on Deepracer.

1. `nav2_default_view.rviz` : Pre-defined view to visualize robot in RViz.

1. `static_tf.yaml`: Static transforms for device flow.

1. `static_tf_sim.yaml`: Static transforms for simulation flow.

## Launch Files

1. `nav_amcl_demo_sim.launch.py`: Demo application for Nav2 in simulation environment.
    * deepracer_sim.launch.py
    * deepracer_navigation_sim.launch.py
    * localization_launch.py

1. `deepracer_sim.launch.py`: Gazebo server and spawn AWS DeepRacer in simulation environment.
    * gzserver and client
    * deepracer_spawn.launch.py

1. `deepracer_navigation_sim.launch.py`: Nav2 related packages in simulation environment.
    * controller_server
    * planner_server
    * recoveries_server
    * bt_navigator
    * waypoint_follower
    * lifecycle_manager

1. `deepracer_spawn.launch.py`: Gazebo simulation related packages to spawn DeepRacer in simulation environment.
    * robot state publisher
    * joint state / wheel velocity / steering hinge controllers
    * static transforms

1. `teleop_test_launch.py`: Teleop launch.
    * joint state / wheel velocity / steering hinge controllers
    * gzserver and client

1. `deepracer.launch.py`: AWS DeepRacer robot related packages.
    * static transforms
    * camera node
    * rplidar node
    * servo node
    * cmdvel to servo node
    * enable deepracer nav node
    * rf2o laser odometry node

1. `deepracer_navigation_dr.launch.py`: Nav2 related packages on AWS DeepRacer to be run along with SLAM toolbox.
    * controller_server
    * planner_server
    * recoveries_server
    * bt_navigator
    * waypoint_follower
    * lifecycle_manager


1. `slam_toolbox.launch.py`: Nav2 SLAM related packages to be launched on AWS DeepRacer device.
    * sync_slam_toolbox_node

## Usage

1. [Simulation Flow](https://github.com/aws-deepracer/aws-deepracer/blob/main/introduction-to-the-ros-navigation-stack-using-aws-deepracer-evo.md#part-11--clone-and-build-the-demo-application-in-an-aws-robomaker-development-environment)

1. [Device Flow](https://github.com/aws-deepracer/aws-deepracer/blob/main/introduction-to-the-ros-navigation-stack-using-aws-deepracer-evo.md#part-21--clone-and-build-the-robot-packages-on-the-aws-deepracer-device)


## Resources

* [Getting started with AWS DeepRacer OpenSource](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md)
* [Introduction to the ROS Navigation stack using AWS DeepRacer Evo](https://github.com/aws-deepracer/aws-deepracer/blob/main/introduction-to-the-ros-navigation-stack-using-aws-deepracer-evo.md)
