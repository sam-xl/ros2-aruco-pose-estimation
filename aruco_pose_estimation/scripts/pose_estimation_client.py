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
        self.declare_parameter('update_xacro', True)
        self.declare_parameter('xacro_path', "")
         
        self._client = self.make_srv_client('estimate_pose_srv', EstimatePose, required=True)
        self.run()
        self.logger.info("Done.")
        self.destroy_node()
        exit(0)

    def run(self):
        parent_frame_id = self.get_parameter("parent_frame_id").value
        child_frame_id = self.get_parameter("child_frame_id").value
        publish_tf = self.get_parameter("publish_tf").value
        update_xacro = self.get_parameter("update_xacro").value

        response = self.estimate_pose(parent_frame_id, child_frame_id, publish_tf)
        if not response.success:
            self.logger.error("Pose estimation failed. Skipping xacro update")
            update_xacro = False    
        else:
            self.logger.info("Pose estimation successful.")
            self.logger.info(f"Translation: {response.transform.transform.translation}")
            self.logger.info(f"Received pose: rotation: {response.transform.transform.rotation}")

        if update_xacro:
            xacro_path = self.get_parameter("xacro_path").get_parameter_value().string_value
            self.logger.info(f"Updating xacro path: {xacro_path}")
            try:
                if not os.path.exists(xacro_path):
                    raise FileNotFoundError
                self.update_xacro(response.transform, xacro_path)
            except Exception as e:
                self.logger.error(f"Xacro update failed. Error: {e}")


    def make_srv_client(self, srv_name, srv_type, required=True):
        """Help in creating and handling service clients."""
        self.logger.info("Waiting for service..")
        client = self.create_client(srv_type, srv_name)
        if not client.wait_for_service(timeout_sec=5.0):
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
    
    def update_xacro(self, transform, xacro_path):
        tree, cell_description = self.get_cell_description(xacro_path)
        
        self.update_cell_description(cell_description, transform)
        tree.write(xacro_path)

    @staticmethod
    def update_cell_description(cell_description, transform):
        cell_description.attrib["parent"] = transform.header.frame_id
        cell_description.attrib["child"] = transform.child_frame_id

        origin = cell_description.find('origin')
        position = transform.transform.translation
        orientation = transform.transform.rotation
        
        origin.attrib['xyz'] = f"{position.x:0.3f} {position.y:0.3f} {position.z:0.3f}"

        rpy = Rotation.from_quat([orientation.x, orientation.y, orientation.z, orientation.z]).as_euler('xyz')

        origin.attrib['rpy'] = f"{rpy[0]:0.3f} {rpy[1]:0.3f} {rpy[2]:0.3f}"

    @staticmethod
    def get_cell_description(xacro_path) -> ET.Element:
        """Get the cell description macro element from the xacro

        Args:
            xacro_path (str): path to the xacro file

        Returns:
            ET.ElementTree: The original element tree. Need it for writing out
            ET.Element: The element to be modified using pose estimation
        """
        # need to do this otherwise the default ns0 namespace is used
        ET.register_namespace('xacro', "http://wiki.ros.org/xacro")
        tree = ET.parse(xacro_path)
        robot = tree.getroot()
        cell_description_macro = robot.find('{http://wiki.ros.org/xacro}cell_description_macro')

        return tree, cell_description_macro
    

def main(args=None):
    """Launch the ROS node.

    Args:
        args (list[str], optional): Args to pass to init for rclpy. Defaults to None.
    """
    rclpy.init(args=args)
    node = PoseEstimationClient("pose_estimation_client")
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()