# AWS DeepRacer Cmdvel to Servo Package

## Overview

The cmdvel to servo ROS package creates the cmdvel_to_servo_node which decides the action messages (servo control messages specifically angle and throttle) to be sent out after converting the `cmd_vel`. For detailed information, see [Introduction to the ROS Navigation stack using AWS DeepRacer Evo](https://github.com/aws-deepracer/aws-deepracer/blob/main/introduction-to-the-ros-navigation-stack-using-aws-deepracer-evo.md).

## License

The source code is released under [Apache 2.0](https://aws.amazon.com/apache-2-0/).

## Installation

### Prerequisites

The `cmdvel_to_servo_pkg` specifically depends on the following ROS2 packages as build and run dependencies:

1. `deepracer_interfaces_pkg`: This package contains the custom message and service type definitions used across the AWS DeepRacer core application, modified to support the ROS Nav2 integration packages.

### Downloading and building

Open up a terminal on the AWS DeepRacer device and run the following commands as the root user.

1. Switch to the root user before you source the ROS2 installation:

        sudo su

1. Source the ROS2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash 

1. Create a workspace directory for the package:

        mkdir -p ~/deepracer_nav2_ws
        cd ~/deepracer_nav2_ws

1. Clone the entire AWS DeepRacer package on the DeepRacer device.

        git clone https://github.com/aws-deepracer/aws-deepracer.git
        cd ~/deepracer_nav2_ws/aws-deepracer

1. Fetch the unreleased dependencies:

        cd ~/deepracer_nav2_ws/aws-deepracer/deepracer_nodes
        rosws update

1. Resolve the dependencies:

        cd ~/deepracer_nav2_ws/aws-deepracer/deepracer_nodes && rosdep install -i --from-path . --rosdistro foxy -y

1. Build the `cmdvel_to_servo_pkg` and `deepracer_interfaces_pkg`:

        cd ~/deepracer_nav2_ws/aws-deepracer/ && colcon build --packages-select deepracer_interfaces_pkg cmdvel_to_servo_pkg


## Using the `cmdvel_to_servo_node`

Although the **cmdvel_to_servo_node** is built to work with the ROS Nav2 integration packages, you can run it independently for development, testing, and debugging purposes.

### Running the node

To launch the built `cmdvel_to_servo_node` as the root user on the AWS DeepRacer device, open another terminal on the AWS DeepRacer device and run the following commands as the root user.

1. Switch to the root user before you source the ROS2 installation:

        sudo su

1. DeepRacer ROS Nav2 workspace:

        cd ~/deepracer_nav2_ws/aws-deepracer/

1. Source the ROS2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash 

1. Source the setup script for the installed packages:

        source ~/deepracer_nav2_ws/aws-deepracer/install/setup.bash

1. Launch the `cmdvel_to_servo_node` using the launch script:

        ros2 launch cmdvel_to_servo_pkg cmdvel_to_servo_pkg_launch.py


### Changing the `MAX_SPEED` scale of the AWS DeepRacer

You can modify the `MAX_SPEED` scale of the AWS DeepRacer using a ROS2 service call in case the car isn’t moving as expected. This can occur because of the vehicle battery percentage, the surface on which the car is operating, or for other reasons.

1. Switch to the root user before you source the ROS2 installation:

        sudo su

1. Navigate to the ROS Nav2 workspace:

        cd ~/deepracer_nav2_ws/aws-deepracer/

1. Source the ROS2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash

1. Source the setup script for the installed packages:

        source ~/deepracer_nav2_ws/aws-deepracer/install/setup.bash

1. Change the `MAX SPEED` to xx% of the `MAX` Scale:

        ros2 service call /cmdvel_to_servo_pkg/set_max_speed deepracer_interfaces_pkg/srv/SetMaxSpeedSrv "{max_speed_pct: 0.xx}"

    Example: Change the `MAX SPEED` to 75% of the `MAX` Scale:

        ros2 service call /cmdvel_to_servo_pkg/set_max_speed deepracer_interfaces_pkg/srv/SetMaxSpeedSrv "{max_speed_pct: 0.75}"


## Launch files

The `cmdvel_to_servo_pkg_launch.py` launch file included in this package gives an example of how to launch the `cmdvel_to_servo_node`.

        from launch import LaunchDescription
        from launch_ros.actions import Node

        def generate_launch_description():
            return LaunchDescription([
                Node(
                    package='cmdvel_to_servo_pkg',
                    namespace='cmdvel_to_servo_pkg',
                    executable='cmdvel_to_servo_node',
                    name='cmdvel_to_servo_node'
                )
        ])


## Node details

### `cmdvel_to_servo`

#### Subscribed topics

| Topic name | Message type | Description |
|----------- | ------------ | ----------- |
|`/cmd_vel`|`geometry_msgs.msg.Twist`|Message which expresses velocity in free space broken into its linear and angular parts.|

#### Published topics

| Topic name | Message type | Description |
| ---------- | ------------ | ----------- |
|`servo_msg`|`ServoCtrlMsg`|This message sends motor throttle and servo steering angle ratios with respect to the device calibration. It can also be used to send raw PWM values for angle and throttle.|

#### Services

| Service name | Service type | Description |
| ---------- | ------------ | ----------- |
|`set_max_speed`|`SetMaxSpeedSrv`|Sets the max speed percentage scale.|

## Resources

* [Getting started with AWS DeepRacer OpenSource](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md)
* [Introduction to the ROS Navigation stack using AWS DeepRacer Evo](https://github.com/aws-deepracer/aws-deepracer/blob/main/introduction-to-the-ros-navigation-stack-using-aws-deepracer-evo.md)