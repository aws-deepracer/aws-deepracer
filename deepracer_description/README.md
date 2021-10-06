# AWS DeepRacer Description Package

## Overview

The AWS DeepRacer repository contains the configuration and launch files to enable ROS Navigation Stack on AWS DeepRacer and control the vehicle using teleop-twist-keyboard, along with the core components to integrate AWS DeepRacer with ROS Navigation stack. For detailed information, see [Introduction to the ROS Navigation stack using AWS DeepRacer Evo](https://github.com/aws-deepracer/aws-deepracer/blob/main/introduction-to-the-ros-navigation-stack-using-aws-deepracer-evo.md).

The [AWS DeepRacer Evo](https://aws.amazon.com/deepracer/) robot is an 1/18th scale 4WD with monster truck chassis and ackermann drive type platform with independent servo and motors to control the wheels. The AWS DeepRacer hardware consists of two independent forward facing 4 MP RGB cameras, a 360 degree planar LiDAR (restricted to view only 300 degrees with reverse orientation), an integrated accelerometer and gyroscope providing the IMU data. The **deepracer_description** package contains the URDF model description of the car and arguments necessary to configure the sensors in Gazebo environment. 

## License

The source code is released under [Apache 2.0](https://aws.amazon.com/apache-2-0/).

## URDF Files

The URDF (**Universal Robot Description Format**) is a specification which defines various XML tags and parameters that can be used to describe a robot’s physical description in ROS. The XML is parsed to identify the joints, wheels, and other components of the robots and their corresponding position, mass, inertia, visual and collision information, etc.

The main components of the URDF files are organized as below:

* `deepracer_description/meshes` - Defines the meshes and the STL files for the AWS DeepRacer hardware.
* `deepracer_description/xacro`
    * `deepracer_description/xacro/deepracer` - This is the main file which hosts all the arguments and configurations. This acts as an entry point to define various sensor configuration for DeepRacer.
    * `deepracer_description/xacro/deepracer_control` - Defines the gazebo ros control and deepracer_drive plugin parameters.
    * `deepracer_description/xacro/sensor` - Defines various gazebo sensor plugins that are supported by DeepRacer device.
    * `deepracer_description/xacro/urdf` - Defines the different combinations of sensor placements for AWS DeepRacer and AWS DeepRacer Evo.



## Configuring the sensors

The **deepracer_description** package exposes different arguments to configure the sensors on the gazebo model and other parameters. The different types of sensor configurations supported are:

1. front_facing_camera_and_lidar
2. stereo_cameras_and_lidar
3. stereo_cameras
4. front_facing_camera

The sensor_type argument in the deepracer.xacro file can be modified to load the required sensor configuration.

        <!-- Default: Use front facing camera and LiDAR sensor -->
        <xacro:arg name="sensor_type" default="front_facing_camera_and_lidar" />

## Resources

* [Getting started with AWS DeepRacer OpenSource](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md)
* [Introduction to the ROS Navigation stack using AWS DeepRacer Evo](https://github.com/aws-deepracer/aws-deepracer/blob/main/introduction-to-the-ros-navigation-stack-using-aws-deepracer-evo.md)
