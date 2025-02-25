#!/usr/bin/python3

from limo_controller.dummy_module import dummy_function, dummy_var
import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_share_directory
from std_srvs.srv import Empty
import os

class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher')
        self.get_logger().info('path_publisher has been start')
        # Declare parameters
        self.declare_parameter('file', 'path.yaml')
        self.header_gt = None
        self.pose_gt = None
        # Store path data
        self.path_msg = Path()
        self.path_msg.header.frame_id = "world"  # Change if using a different frame

        pkg_name = 'limo_controller'
        path_pkg_share_path = get_package_share_directory(pkg_name)
        ws_path, _ = path_pkg_share_path.split('install')
        file = self.get_parameter('file').value
        self.path_path = os.path.join(ws_path, 'src', pkg_name, 'config', file)
        # Pub
        self.publisher = self.create_publisher(Path, 'path', 10)
        self.path_publisher = self.create_publisher(Path, '/limo/path', 10)
        # Sub
        self.create_subscription(Odometry, '/odometry/ground_truth', self.OdoGroundTruth, 10)
        # Create Service for clearing the path
        self.clear_path_service = self.create_service(Empty, 'clear_path', self.clear_path_callback)

        self.timer = self.create_timer(0.01, self.timer)

        self.publish_path()
    
    def timer(self):
        self.update_robot_path()

    def clear_path_callback(self, request, response):
        """ Callback to clear the robot path when the service is called """
        self.get_logger().info("Clearing the robot path...")
        self.path_msg.poses = []  # Reset path
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.path_publisher.publish(self.path_msg)  # Publish the cleared path
        return response

    def update_robot_path(self):
        if self.header_gt is None or self.pose_gt is None:
            self.get_logger().warn("Odometry data not received yet, skipping path update.")
            return
        # Convert Odometry pose to PoseStamped
        pose_stamped = PoseStamped()
        pose_stamped.header = self.header_gt
        pose_stamped.pose = self.pose_gt
        self.path_msg.poses.append(pose_stamped)
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.path_publisher.publish(self.path_msg)

    def OdoGroundTruth(self, msg:Odometry):
        self.header_gt = msg.header
        self.pose_gt = msg.pose.pose

    def publish_path(self):

        # Load the YAML file
        with open(self.path_path, 'r') as file:
            path_data = yaml.safe_load(file)

        # Create Path message
        path_msg = Path()
        path_msg.header.frame_id = "world"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for point in path_data:
            pose = PoseStamped()
            pose.header.frame_id = "world"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = point['x']
            pose.pose.position.y = point['y']
            pose.pose.position.z = 0.0
            pose.pose.orientation.z = point['yaw']  # Assuming yaw directly, may need conversion

            path_msg.poses.append(pose)

        # Publish Path
        self.publisher.publish(path_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()