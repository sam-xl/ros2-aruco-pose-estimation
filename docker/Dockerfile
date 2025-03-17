FROM registry.tudelft.nl/samxl/humble_dev_base:0.0.2 as pose_estimation_builder

RUN sudo apt-get update && \
    sudo apt-get install -y --no-install-recommends \
    ros-humble-tf-transformations

RUN pip3 install --upgrade opencv-python==4.10.0.82

COPY aruco_pose_estimation /home/ros/target_ws/src/aruco_pose_estimation
COPY aruco_interfaces /home/ros/target_ws/src/aruco_interfaces

WORKDIR /home/ros/target_ws
RUN sudo chown ros:ros .
RUN source /ros_entrypoint.sh && \
    colcon build 
RUN sudo rm -rf build log src