#!/bin/bash

source install/setup.bash
ros2 launch aruco_pose_estimation pose_estimation_client.launch.xml \
    update_xacro:=true \
    xacro_path:=/home/ros/urdf/coby_env.xacro \
    parent_frame_id:=ur10e_base_link \
    child_frame_id:=aruco_marker \
    estimate_pose_srv:=/estimate_pos

