# AWS DeepRacer

<p align="center">
<img src="/media/deepracer_circle_sticker.png" width="250" height="250" >
</p>

## Overview

The AWS DeepRacer Evo vehicle is a 1/18th scale Wi-Fi enabled 4-wheel ackermann steering platform that features two RGB cameras and a LiDAR sensor. This repository contains the configuration and launch files to enable ROS Navigation Stack on AWS DeepRacer and control the vehicle using teleop-twist-keyboard, along with the core components to integrate AWS DeepRacer with ROS Navigation stack. For detailed information, see [Introduction to the ROS Navigation stack using AWS DeepRacer Evo](https://github.com/aws-deepracer/aws-deepracer/blob/main/introduction-to-the-ros-navigation-stack-using-aws-deepracer-evo.md).

## Contents

The AWS DeepRacer repository consists of the following packages:

1. **deepracer_bringup**: The deepracer_bringup package hosts the launch files and configuration parameter files to launch teleop and navigation packages and nodes.

1. **deepracer_description**: The deepracer_description package hosts the URDF files for the AWS DeepRacer device in Gazebo simulation. This provides the arguments to configure the sensors.

1. **deepracer_gazebo**: The deepracer_gazebo package hosts the deepracer_drive plugin that is required to move the car in simulation.

1. **deepracer_nodes**: The deepracer_nodes packages hosts the set of nodes that are responsible to launch the AWS DeepRacer robot packages required for ROS Nav2 stack compatibility.

## ROS Navigation Stack on AWS DeepRacer

### Navigation2Goal

<p align="center">
<img src="/media/dr-sim-rviz-navigation2goal-1.gif" height="450" >
</p>

### Mapping using SLAM Toolbox

<p align="center">
<img src="/media/dr-slam.gif" height="250" align="left">
<img src="/media/slam-hallway-map.png" height="250" >
</p>


## Resources
* [Frequently Asked Questions & Known Issues](https://github.com/aws-deepracer/aws-deepracer/blob/main/frequently_asked_questions.md)
* [Getting started with AWS DeepRacer OpenSource](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md)
* [Introduction to the ROS Navigation stack using AWS DeepRacer Evo](https://github.com/aws-deepracer/aws-deepracer/blob/main/introduction-to-the-ros-navigation-stack-using-aws-deepracer-evo.md)
