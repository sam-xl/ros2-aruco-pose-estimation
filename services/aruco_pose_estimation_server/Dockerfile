FROM ros:jazzy

SHELL ["bash", "-c"]

ARG DEBIAN_FRONTEND noninteractive

ARG WORKSPACE_DIR=/workspace
RUN --mount=type=bind,target=${WORKSPACE_DIR}/src/aruco_pose_estimation \
    apt-get update && apt-get install --no-install-recommends -y \
        python3-pip \
    && pip3 install --upgrade --break-system-packages \
        opencv-python==4.10.0.82 \
        open3d \
    && rosdep install \
        --from-paths ${WORKSPACE_DIR}/src \
        --skip-keys "opencv open3d" \
        -iy \
    && cd ${WORKSPACE_DIR} \
    && source /opt/ros/${ROS_DISTRO}/setup.bash \
    && colcon build --packages-up-to aruco_pose_estimation \
    && rm -rf build log /var/lib/apt/lists/*
