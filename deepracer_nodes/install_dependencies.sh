#!/usr/bin/env bash
echo "Cloning the rf2o_laser_odometry package"
cd ~/deepracer_nav2_ws/aws-deepracer/deepracer_nodes
git clone https://github.com/MAPIRlab/rf2o_laser_odometry.git
cd rf2o_laser_odometry
git fetch origin pull/29/head:foxy-devel
git checkout foxy-devel
echo ""
echo "Cloning the rplidar_ros package"
cd ~/deepracer_nav2_ws/aws-deepracer/deepracer_nodes
git clone https://github.com/Slamtec/rplidar_ros.git -b ros2