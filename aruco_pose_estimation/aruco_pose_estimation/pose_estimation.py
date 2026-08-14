#!/usr/bin/env python3

# Code taken and readapted from:
# https://github.com/GSNCodes/ArUCo-Markers-Pose-Estimation-Generation-Python/tree/main

# Python imports
import numpy as np
import cv2
import tf_transformations

# ROS2 imports
from rclpy.impl import rcutils_logger

# ROS2 message imports
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from aruco_interfaces.msg import ArucoMarkers

# utils import python code
from aruco_pose_estimation.utils import aruco_display


def pose_estimation(rgb_frame: np.array, depth_frame: np.array, aruco_dict, aruco_params, marker_size: float,
                    matrix_coefficients: np.array, distortion_coefficients: np.array,
                    pose_array: PoseArray, markers: ArucoMarkers) -> list[np.array, PoseArray, ArucoMarkers]:
    '''
    rgb_frame - Frame from the RGB camera stream
    depth_frame - Depth frame from the depth camera stream
    matrix_coefficients - Intrinsic matrix of the calibrated camera
    distortion_coefficients - Distortion coefficients associated with your camera
    pose_array - PoseArray message to be published
    markers - ArucoMarkers message to be published

    return:-
    frame - The frame with the axis drawn on it
    pose_array - PoseArray with computed poses of the markers
    markers - ArucoMarkers message containing markers id number and pose
    '''

    # old code version
    # parameters = cv2.aruco.DetectorParameters_create()
    # corners, marker_ids, _ = cv2.aruco.detectMarkers(frame, aruco_dict_type, parameters=parameters)
    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
    corners, marker_ids, rejected = detector.detectMarkers(image=rgb_frame)

    # new code version
    # corners, marker_ids, _ = cv2.aruco.detectMarkers(image=rgb_frame, dictionary=aruco_dict, parameters=aruco_params)

    frame_processed = rgb_frame
    logger = rcutils_logger.RcutilsLogger(name="aruco_node")

    # If markers are detected
    if len(corners) > 0:

        logger.debug("Detected {} markers.".format(len(corners)))

        for i, marker_id in enumerate(marker_ids):
            # Estimate pose of each marker and return the values rvec and tvec

            # using deprecated function
            # rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners=corners[i],
            #                                                               markerLength=marker_size,
            #                                                               cameraMatrix=matrix_coefficients,
            #                                                               distCoeffs=distortion_coefficients)
            # tvec = tvec[0]

            # alternative code version using solvePnP
            tvec, rvec, quat = my_estimatePoseSingleMarkers(corners=corners[i], marker_size=marker_size,
                                                                    camera_matrix=matrix_coefficients,
                                                                    distortion=distortion_coefficients)

            # show the detected markers bounding boxes
            frame_processed = aruco_display(corners=corners, ids=marker_ids,
                                            image=frame_processed)

            # draw frame axes
            frame_processed = cv2.drawFrameAxes(image=frame_processed, cameraMatrix=matrix_coefficients,
                                                distCoeffs=distortion_coefficients, rvec=rvec, tvec=tvec,
                                                length=0.05, thickness=3)

            if (depth_frame is not None):
                # get the centroid of the pointcloud
                try:
                    centroid = depth_to_pointcloud_centroid(
                        depth_image=depth_frame,
                        intrinsic_matrix=matrix_coefficients,
                        corners=corners[i],
                    )
                    # log comparison between depthcloud centroid and tvec estimated positions
                    logger.info(f"depthcloud centroid = {centroid}")
                    logger.info(f"tvec = {tvec[0]} {tvec[1]} {tvec[2]}")

                except Exception:
                    depth_frame = None


            # compute pose from the rvec and tvec arrays
            if (depth_frame is not None):
                # use computed centroid from depthcloud as estimated pose
                pose = Pose()
                pose.position.x = float(centroid[0])
                pose.position.y = float(centroid[1])
                pose.position.z = float(centroid[2])
            else:
                # use tvec from aruco estimator as estimated pose
                pose = Pose()
                pose.position.x = float(tvec[0])
                pose.position.y = float(tvec[1])
                pose.position.z = float(tvec[2])

            pose.orientation.x = quat[0]
            pose.orientation.y = quat[1]
            pose.orientation.z = quat[2]
            pose.orientation.w = quat[3]

            # add the pose and marker id to the pose_array and markers messages
            pose_array.poses.append(pose)
            markers.poses.append(pose)
            markers.marker_ids.append(marker_id[0])
    else:
        logger.warn("Detected no markers.")

    return frame_processed, pose_array, markers


def my_estimatePoseSingleMarkers(corners, marker_size, camera_matrix, distortion) -> tuple[np.array, np.array, np.array]:
    '''
    This will estimate the rvec and tvec for each of the marker corners detected by:
       corners, ids, rejectedImgPoints = detector.detectMarkers(image)

    corners - is an array of detected corners for each detected marker in the image
    marker_size - is the size of the detected markers in meters
    mtx - is the camera intrinsic matrix
    distortion - is the camera distortion matrix
    RETURN list of rvecs, tvecs, and trash (so that it corresponds to the old estimatePoseSingleMarkers())
    '''
    marker_points = np.array([[-marker_size / 2.0, marker_size / 2.0, 0],
                              [marker_size / 2.0, marker_size / 2.0, 0],
                              [marker_size / 2.0, -marker_size / 2.0, 0],
                              [-marker_size / 2.0, -marker_size / 2.0, 0]], dtype=np.float32)

    # solvePnP returns the rotation and translation vectors
    retval, rvec, tvec = cv2.solvePnP(objectPoints=marker_points, imagePoints=corners,
                                        cameraMatrix=camera_matrix, distCoeffs=distortion, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    rvec = rvec.reshape(3, 1)
    tvec = tvec.reshape(3, 1)
       
    rot, jacobian = cv2.Rodrigues(rvec)
    rot_matrix = np.eye(4, dtype=np.float32)
    rot_matrix[0:3, 0:3] = rot

    # convert rotation matrix to quaternion
    quaternion = tf_transformations.quaternion_from_matrix(rot_matrix)
    norm_quat = np.linalg.norm(quaternion)
    quaternion = quaternion / norm_quat

    return tvec, rvec, quaternion


def depth_to_pointcloud_centroid(
    depth_image: np.ndarray, intrinsic_matrix: np.ndarray, corners: np.ndarray
) -> np.ndarray:
    """
    Takes a depth image (in METERS, float) and the corners of a quadrilateral,
    and returns the centroid (x, y, z) of the corresponding pointcloud, in meters.

    Args:
        depth_image: 2D array of depth values in meters. Invalid pixels should be
                      0 or NaN.
        intrinsic_matrix: 3x3 camera intrinsic matrix (float).
        corners: shape (4, 2) array of (x, y) pixel coordinates.

    Returns:
        np.ndarray of shape (3,), dtype float64: (x, y, z) centroid in meters.
    """
    corners = np.asarray(corners)
    if corners.ndim == 3:
        corners = corners.reshape(-1, 2)

    height, width = depth_image.shape
    corners_indices = np.round(corners).astype(
        np.int32
    )  # pixel indices stay int — that's correct

    if (
        np.any(corners_indices[:, 0] < 0)
        or np.any(corners_indices[:, 0] >= width)
        or np.any(corners_indices[:, 1] < 0)
        or np.any(corners_indices[:, 1] >= height)
    ):
        raise ValueError("One or more corners are outside the image bounds.")

    mask = np.zeros((height, width), dtype=np.uint8)
    cv2.fillPoly(mask, [corners_indices], color=1)

    ys, xs = np.nonzero(mask)  # pixel indices, int — correct
    depths = depth_image[ys, xs].astype(np.float64)  # depth values, float

    valid = np.isfinite(depths) & (depths > 0)
    if not np.any(valid):
        raise ValueError("No valid depth points found inside the given polygon.")

    xs_f = xs[valid].astype(np.float64)
    ys_f = ys[valid].astype(np.float64)
    z = depths[valid]  # already float64

    intrinsic_matrix = np.asarray(
        intrinsic_matrix, dtype=np.float64
    )  # guard against int intrinsics
    fx, fy = intrinsic_matrix[0, 0], intrinsic_matrix[1, 1]
    cx, cy = intrinsic_matrix[0, 2], intrinsic_matrix[1, 2]

    x = (xs_f - cx) * z / fx
    y = (ys_f - cy) * z / fy

    pointcloud = np.stack([x, y, z], axis=1).astype(
        np.float64
    )  # explicit, no accidental dtype
    centroid = np.mean(pointcloud, axis=0)

    return centroid


def is_pixel_in_polygon(pixel: tuple, corners: np.array) -> bool:
    """
    This function takes a pixel and a list of corners as input, and returns whether the pixel is inside the polygon
    defined by the corners. This function uses the ray casting algorithm to determine if the pixel is inside the polygon.
    This algorithm works by casting a ray from the pixel in the positive x-direction, and counting the number of times
    the ray intersects with the edges of the polygon. If the number of intersections is odd, the pixel is inside the
    polygon, otherwise it is outside. This algorithm works for both convex and concave polygons.

    Args:
        pixel: A tuple (x, y) representing the pixel coordinates.
        corners: A list of 4 tuples in a numpy array, each representing the (x, y) coordinates of a corner.

    Returns:
        A boolean indicating whether the pixel is inside the polygon.
    """

    # Initialize counter for number of intersections
    num_intersections = 0

    # Iterate over each edge of the polygon
    for i in range(len(corners)):
        x1, y1 = corners[i]
        x2, y2 = corners[(i + 1) % len(corners)]

        # Check if the pixel is on the same y-level as the edge
        if (y1 <= pixel[1] < y2) or (y2 <= pixel[1] < y1):
            # Calculate the x-coordinate of the intersection point
            x_intersection = (x2 - x1) * (pixel[1] - y1) / (y2 - y1) + x1

            # Check if the intersection point is to the right of the pixel
            if x_intersection > pixel[0]:
                num_intersections += 1

    # Return whether the number of intersections is odd
    return num_intersections % 2 == 1
