#!/usr/bin/env python3
"""
ROS 2 node that:
- Subscribes to four odometry topics:
    • Ground truth (default: /odometry/ground_truth)
    • Yaw-rate odometry (default: /odometry/yaw_rate)
    • Single-track odometry (default: /odometry/single_track)
    • Double-track odometry (default: /odometry/double_track)
- Updates a single 2D plot (x vs y) in real time.
- Saves collected odometry data to CSV.
- Computes the covariance matrix after stopping data collection.
- Exports covariance matrices to a YAML file.
"""

import os
import csv
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
import matplotlib.pyplot as plt
import tf_transformations
import pandas as pd
import numpy as np
import yaml

# Import covariance function from COV_compute.py
from COV_compute import compute_covariance

def find_workspace_by_name(start_path, workspace_name="MOBILE_ROBOT_WS"):
    """
    Traverse upward from start_path until a directory named workspace_name is found.
    """
    current_dir = os.path.abspath(start_path)
    while True:
        if os.path.basename(current_dir) == workspace_name:
            return current_dir
        parent_dir = os.path.dirname(current_dir)
        if parent_dir == current_dir:  # reached filesystem root
            return None
        current_dir = parent_dir

class PlotMultiOdomNode(Node):
    def __init__(self):
        super().__init__('plot_multi_odom_node')
        self.get_logger().info("PlotMultiOdomNode has started.")
        self.get_logger().info('Type the following command to stop data collection:\n'
                       'ros2 topic pub --once /stop_collection std_msgs/Empty "{}"')

        # Declare and get parameters
        self.declare_parameter('csv_filename', 'multi_odom_data.csv')
        self.csv_filename = self.get_parameter('csv_filename').value

        # Determine workspace path
        script_dir = os.path.dirname(os.path.abspath(__file__))
        workspace_dir = find_workspace_by_name(script_dir, "MOBILE_ROBOT_WS")
        if workspace_dir is None:
            self.get_logger().error("Could not find workspace directory 'MOBILE_ROBOT_WS'. Using script directory as fallback.")
            workspace_dir = script_dir

        # Data directory for CSV storage
        data_dir = os.path.join(workspace_dir, "src", "Mobile-Robot-LAB-1", "limo_localization", "data")
        os.makedirs(data_dir, exist_ok=True)
        self.csv_file_path = os.path.join(data_dir, self.csv_filename)

        # Config directory for YAML output
        config_dir = os.path.join(workspace_dir, "src", "Mobile-Robot-LAB-1", "limo_localization", "config")
        os.makedirs(config_dir, exist_ok=True)
        self.yaml_output_path = os.path.join(config_dir, "covariances.yaml")

        self.get_logger().info(f"CSV will be saved to: {self.csv_file_path}")
        self.get_logger().info(f"YAML will be saved to: {self.yaml_output_path}")

        # Initialize data storage
        self.latest_data = {"ground_truth": None, "yaw_rate": None, "single_track": None, "double_track": None}
        self.data_rows = []
        self.collecting_data = True

        # Subscribers for odometry topics
        self.gt_sub = self.create_subscription(Odometry, "/odometry/ground_truth", lambda msg: self.odom_callback(msg, "ground_truth"), 10)
        self.yaw_rate_sub = self.create_subscription(Odometry, "/odometry/yaw_rate", lambda msg: self.odom_callback(msg, "yaw_rate"), 10)
        self.single_track_sub = self.create_subscription(Odometry, "/odometry/single_track", lambda msg: self.odom_callback(msg, "single_track"), 10)
        self.double_track_sub = self.create_subscription(Odometry, "/odometry/double_track", lambda msg: self.odom_callback(msg, "double_track"), 10)

        # Subscriber for stopping data collection
        self.stop_sub = self.create_subscription(Empty, "/stop_collection", self.stop_collection_callback, 10)

    def odom_callback(self, msg: Odometry, topic_key: str):
        if not self.collecting_data:
            return

        # Extract data from odometry message
        t = self.get_clock().now().seconds_nanoseconds()
        curr_time = t[0] + t[1] * 1e-9
        px, py, pz = msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z
        quat = msg.pose.pose.orientation
        roll, pitch, yaw = tf_transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        vx, vy, vz = msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z
        wx, wy, wz = msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z

        self.latest_data[topic_key] = (curr_time, px, py, pz, roll, pitch, yaw, vx, vy, vz, wx, wy, wz)
        self.try_combine_data()

    def try_combine_data(self):
        """
        Combine the latest data from all topics if available.
        """
        if all(self.latest_data[tk] is not None for tk in self.latest_data):
            combined_time = max(data[0] for data in self.latest_data.values())
            row = [combined_time] + [self.latest_data[key][1:] for key in ["ground_truth", "yaw_rate", "single_track", "double_track"]]
            flat_row = [val for sublist in row for val in (sublist if isinstance(sublist, tuple) else [sublist])]
            self.data_rows.append(flat_row)

    def stop_collection_callback(self, msg: Empty):
        if self.collecting_data:
            self.get_logger().info("Stopping data collection.")
            self.collecting_data = False
            self.save_data_to_csv()
            self.compute_and_export_covariance()

    def save_data_to_csv(self):
        """
        Saves collected odometry data to CSV.
        """
        header = ["time"] + [f"{topic}_{comp}" for topic in ["ground_truth", "yaw_rate", "single_track", "double_track"]
                             for comp in ["px", "py", "pz", "roll", "pitch", "yaw", "vx", "vy", "vz", "wx", "wy", "wz"]]

        try:
            with open(self.csv_file_path, mode='w', newline='') as csv_file:
                writer = csv.writer(csv_file)
                writer.writerow(header)
                writer.writerows(self.data_rows)
            self.get_logger().info(f"Saved data to CSV: {self.csv_file_path}")
        except IOError as e:
            self.get_logger().error(f"Failed to write CSV: {e}")

    def compute_and_export_covariance(self):
        """
        Computes the covariance matrix for each odometry type and exports it to YAML.
        """
        if not os.path.exists(self.csv_file_path):
            self.get_logger().error("CSV file not found, cannot compute covariance.")
            return

        df = pd.read_csv(self.csv_file_path)
        odom_types = ["yaw_rate", "single_track", "double_track"]
        cov_dict = {}

        for odom in odom_types:
            self.get_logger().info(f"Computing covariance for '{odom}'...")
            mean_err, cov = compute_covariance(df, odom)
            if cov is not None:
                cov_dict[odom] = cov.tolist()

        with open(self.yaml_output_path, "w") as f:
            yaml.dump({"covariances": cov_dict}, f)
        self.get_logger().info(f"Covariance matrices exported to YAML: {self.yaml_output_path}")

def main(args=None):
    rclpy.init(args=args)
    node = PlotMultiOdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
