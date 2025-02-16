#!/usr/bin/python3

from limo_controller.dummy_module import dummy_function, dummy_var
import rclpy
import yaml
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_share_directory
import os

class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher')
        self.get_logger().info('path_publisher has been start')
        # Declare parameters
        self.declare_parameter('file', 'path.yaml')

        pkg_name = 'limo_controller'
        path_pkg_share_path = get_package_share_directory(pkg_name)
        ws_path, _ = path_pkg_share_path.split('install')
        file = self.get_parameter('file').value
        self.path_path = os.path.join(ws_path, 'src/Mobile-Robot-LAB-1', pkg_name, 'config', file)
        
        self.publisher = self.create_publisher(Path, 'path', 10)
        self.timer = self.create_timer(1.0, self.publish_path)

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