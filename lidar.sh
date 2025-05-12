#!/bin/bash

# 启动 roscore
gnome-terminal --tab --command="bash -c 'roscore'" --title="roscore"

# 等待 roscore 启动
sleep 3

# 启动第一个ROS节点: hk_node
gnome-terminal --tab --working-directory="/home/climber/RM2024Climber" --command="bash -c 'roslaunch hk_camera_pkg hk_node.launch'" --title="hk_node"

sleep 1

# 启动第二个ROS节点: infer_node
gnome-terminal --tab --working-directory="/home/climber/RM2024Climber" --command="bash -c 'roslaunch tensorrt_pkg infer_node.launch'" --title="infer_node"

sleep 1

# 启动第三个ROS节点: livox_lidar
gnome-terminal --tab --working-directory="/home/climber/RM2024Climber" --command="bash -c 'roslaunch livox_ros_driver livox_lidar.launch'" --title="livox_lidar"

sleep 1

# 启动第四个ROS节点: pub_depth_node
gnome-terminal --tab --working-directory="/home/climber/RM2024Climber" --command="bash -c 'roslaunch pub_depth_pkg pub_depth_node.launch'" --title="pub_depth_node"

sleep 1

# 启动第五个ROS节点: world_node
gnome-terminal --tab --working-directory="/home/climber/RM2024Climber" --command="bash -c 'roslaunch world_points_pkg world_node.launch'" --title="world_node"

# 等待所有节点完成
wait
