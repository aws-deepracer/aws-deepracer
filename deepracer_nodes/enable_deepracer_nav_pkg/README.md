# AWS DeepRacer Enable DeepRacer Nav Package

## Overview

The Enable DeepRacer Nav ROS package creates the enable_deepracer_nav_node which enables the camera and servo GPIO on the AWS DeepRacer. For detailed information, see [Introduction to the ROS Navigation stack using AWS DeepRacer Evo](https://github.com/aws-deepracer/aws-deepracer/blob/main/introduction-to-the-ros-navigation-stack-using-aws-deepracer-evo.md).

## License

The source code is released under [Apache 2.0](https://aws.amazon.com/apache-2-0/).

## Installation

### Prerequisites

The `enable_deepracer_nav_pkg` specifically depends on the following ROS2 packages as build and run dependencies:

1. `deepracer_interfaces_pkg`: This package contains the custom message and service type definitions used across the AWS DeepRacer core application, modified to support the ROS Nav2 integration packages.

### Downloading and building

Open up a terminal on the AWS DeepRacer device and run the following commands as the root user.

1. Switch to the root user before you source the ROS2 installation:

        sudo su

1. Source the ROS2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash 

1. Create a workspace directory for the package:

        mkdir -p ~/deepracer_nav2_ws
        cd ~/deepracer_nav2_ws/

1. Clone the entire AWS DeepRacer project on the DeepRacer device.

        git clone https://github.com/aws-deepracer/aws-deepracer.git
        cd ~/deepracer_nav2_ws/aws-deepracer/

1. Fetch the unreleased dependencies:

        cd ~/deepracer_nav2_ws/aws-deepracer/deepracer_nodes
        rosws update

1. Resolve the dependencies:

        cd ~/deepracer_nav2_ws/aws-deepracer/deepracer_nodes && rosdep install -i --from-path . --rosdistro foxy -y

1. Build the `enable_deepracer_nav_pkg` and `deepracer_interfaces_pkg`:

        cd ~/deepracer_nav2_ws/aws-deepracer/ && colcon build --packages-select deepracer_interfaces_pkg enable_deepracer_nav_pkg


## Using the `enable_deepracer_nav_mode`

Although the **enable_deepracer_nav_node** is built to work with the ROS Nav2 integration packages, you can run it independently for development, testing, and debugging purposes.

### Running the node

To launch the built `enable_deepracer_nav_node` as the root user on the AWS DeepRacer device, open another terminal on the AWS DeepRacer device and run the following commands as the root user.

1. Switch to the root user before you source the ROS2 installation:

        sudo su

1. DeepRacer ROS Nav2 workspace:

        cd ~/deepracer_nav2_ws/aws-deepracer/

1. Source the ROS2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash 

1. Source the setup script for the installed packages:

        source ~/deepracer_nav2_ws/aws-deepracer/install/setup.bash

1. Launch the `enable_deepracer_nav_node` using the launch script:

        ros2 launch enable_deepracer_nav_pkg enable_deepracer_nav_pkg_launch.py

## Launch files

The `enable_deepracer_nav_pkg_launch.py` launch file included in this package gives an example of how to launch the `enable_deepracer_nav_node`.

        from launch import LaunchDescription
        from launch_ros.actions import Node

        def generate_launch_description():
            return LaunchDescription([
                Node(
                    package='enable_deepracer_nav_pkg',
                    namespace='enable_deepracer_nav_pkg',
                    executable='enable_deepracer_nav_node',
                    name='enable_deepracer_nav_node'
                )
        ])


## Node details

### `enable_deepracer_nav`

#### Service Clients

| Service name | Service type | Description |
| ---------- | ------------ | ----------- |
|`/camera_pkg/media_state`|`VideoStateSrv`|Service that is called to activate the camera node and start publishing the images to the video_mjpeg and display_mjpeg topics.|
|`/servo_pkg/servo_gpio`|`ServoGPIOSrv`|A service that is called enable or disable the servo GPIO pin.|


## Resources

* [Getting started with AWS DeepRacer OpenSource](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md)
* [Introduction to the ROS Navigation stack using AWS DeepRacer Evo](https://github.com/aws-deepracer/aws-deepracer/blob/main/introduction-to-the-ros-navigation-stack-using-aws-deepracer-evo.md)