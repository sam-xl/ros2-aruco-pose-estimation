#!/usr/bin/python3

"""pose_estimation_client.py_

A ros node that calls the pose estimation service and updates the pose in the mentioned xacro
"""
import rclpy
from rclpy.node import Node
from aruco_interfaces.srv import EstimatePose
import xml.etree.ElementTree as ET
from scipy.spatial.transform import Rotation
import os

class PoseEstimationClient(Node):
    def __init__(self, name):
        super().__init__(node_name=name)
        self.logger = self.get_logger()

        # ROS parameters
        self.declare_parameter('parent_frame_id', 'camera_link')
        self.declare_parameter('child_frame_id', 'aruco_marker')
        self.declare_parameter('publish_tf', False)
         
        self._client = self.make_srv_client('estimate_pose_srv', EstimatePose, required=True)
        self.run()
        self.logger.info("Done.")

    def run(self):
        parent_frame_id = self.get_parameter("parent_frame_id").value
        child_frame_id = self.get_parameter("child_frame_id").value
        publish_tf = self.get_parameter("publish_tf").value

        response = self.estimate_pose(parent_frame_id, child_frame_id, publish_tf)
        if not response.success:
            self.logger.error("Pose estimation failed.")
        else:
            self.logger.info("Pose estimation successful.")
            xyz, rpy = self.transform_to_pose(response.transform.transform)
            self.logger.info(f"Translation [xyz, metres]: {xyz}")
            self.logger.info(f"Rotation [rpy, radians]: {rpy}")
            return

    def make_srv_client(self, srv_name, srv_type, required=True):
        """Help in creating and handling service clients."""
        self.logger.info("Waiting for service..")
        client = self.create_client(srv_type, srv_name)
        if not client.wait_for_service(timeout_sec=10.0):
            remapped_srv_name = self.resolve_service_name(srv_name)
            msg = f"Timed out waiting for server: '{remapped_srv_name}'"
            if required:
                self.logger.fatal(msg)
                raise RuntimeError(msg)
            self.logger.info(msg)
        return client
    
    def estimate_pose(self, parent_frame_id, child_frame_id, publish_tf):
        request = EstimatePose.Request()
        request.publish_tf = publish_tf

        request.parent_frame_id = parent_frame_id
        request.child_frame_id = child_frame_id
        
        future = self._client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        return response

    @staticmethod
    def transform_to_pose(transform):
        xyz = transform.translation
        quat = transform.rotation
        rpy = Rotation.from_quat([quat.x, quat.y, quat.z, quat.w]).as_euler('xyz')
        return xyz, rpy


def main(args=None):
    """Launch the ROS node.

    Args:
        args (list[str], optional): Args to pass to init for rclpy. Defaults to None.
    """
    rclpy.init(args=args)
    node = PoseEstimationClient("pose_estimation_client")
    rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()