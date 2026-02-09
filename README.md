## Aruco pose estimation server

## Overview
This repository hosts a server that uses a 2D vision camera and aruco markers to estimate the pose of an object.

This package hase been developed and tested under ros2 jazzy and UBuntu 24.04.

## Installation

### Dependencies
This package works in combination with a 2D camera which can publish images on the ROS network. This package was tested for the Realsense D435 camera, compatible with ROS2 `realsense-ros` driver,
available at [ros2_intel_realsense](https://github.com/IntelRealSense/realsense-ros). The code should work equally well on other and
different cameras, provided a proper calibration of the camera parameters.

### Building
To build from source, clone the lastest version from this repository into your workspace:

```bash
cd ros_ws/src
git clone https://github.com/sam-xl/ros2-aruco-pose-estimation.git
```

Install the dependencies of the cloned packages using `rosdep`:

```bash
rosdep install --from-paths . -iy
```

Finally, build all packages in the workspace:

```bash
colcon build
```

## Usage

```bash
ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=True publish_tf:=False
```

Launch the cell description and bringup the controllers. This is mainly needed for the TF tree.

For example:
```bash
ros2 launch nxtgen_accuracy_measurement_cell_description bringup.xml launch_rviz:=True 
```

Run the pose estimation server.

```bash
ros2 launch aruco_pose_estimation server.launch
```

This launch file bringups the pose estimation server and publishes a static transform between the tool end effector and the camera frame. This transform is obtained from a calibration package such as easy_handeye2.

- `image_topic`: The image topic which can see the aruco marker. 
- `camera_frame`: The child frame for the calibrated transform.
- `tool_frame`: The parent frame for the calibrated transform.
- `marker_size`: The edge size in meters of the marker.

See the server.launch file to see other default values. For example we use the `DICT_ARUCO_ORIGINAL` for aruco detection.

Run the example client file (or run call the server via cli)
```bash
ros2 launch aruco_pose_estimation client.launch
```


## Launch files
* **`server.launch`**: Runs a pose estimation server and creates a static transform between the tool frame and camera frame (created using hand-eye calibration)

* **`client.launch`**: Runs an example client and outputs the transform

## Nodes

### `aruco_node`
This node exposes a pose estimation server at `/estimat_pose`

#### Parameters 

* `marker_size` - size of the markers in meters
* `aruco_dictionary_id` - dictionary type that was used to generate markers (example `DICT_ARUCO_ORIGINAL`)
* `use_depth_input` - use depth image for pose estimation (default `false`)
* `image_topic` - RGB image topic to subscribe to, provided by the camera ROS2 driver
* `depth_image_topic` - Depth image topic to subscribe to, provided by the camera ROS2 driver
* `camera_info_topic` - Camera info topic to subscribe to, providing intrinsic and distortion parameters
* `camera_frame` - Camera optical frame to use (default to the frame id provided by the camera info message.)
* `detecter_markers_topic` - Topic to publish the detected markers as ArucoMarkers message
* `markers_visualization_topic` - Topic to publish the detected markers as PoseArray message
* `output_image_topic` - Topic to publish the output image with detected markers drawn on it, for visualization purposes

#### Published Topics
* `/topic_name` ([message/type](link/to/msg/file))\
TODO: Add description ...

#### Subscribed Topics
* `/camera/image_raw`: RGB image input (`sensor_msgs.msg.Image`)
* `/camera/depth/image_rect_raw`: Depth image input (`sensor_msgs.msg.Image`)
* `/camera/camera_info`: Camera intrinsic, projection, distortion parameters (`sensor_msgs.msg.CameraInfo`)

#### Services 

* `/estimate_pose` ([aruco_interfaces/srv/EstimatePose](aruco_interfaces/srv/EstimatePose.srv))\

_Request_:

publish_tf(`bool`):  Publish the transform between base and marker.
base_frame_id(`string`): The resulting transform is with respect to this frame id.
marker_frame_id(`string`): The frame name in the TF tree of the aruco marker.

_Response_

success (`bool`): If the pose estimation was successful
transform (`geometry_msgs/TransformStamped`): The resulting transform between base and marker


## Detailed Documentation
Other than the brief introduction in this page, you can also check the detailed documentation [here](./docs/).

## Bugs & Feature Requests
Please report bugs and request features using the [Issue Tracker](<PACKAGE_ISSUE_URL>).
