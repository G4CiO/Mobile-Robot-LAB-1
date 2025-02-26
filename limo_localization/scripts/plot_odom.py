#!/usr/bin/env python3
"""
ROS 2 node that:
- Subscribes to four odometry topics:
    • Ground truth (default: /odometry/ground_truth)
    • Yaw-rate odometry (default: /odometry/yaw_rate)
    • Single-track odometry (default: /odometry/single_track)
    • Double-track odometry (default: /odometry/double_track)
- For each topic, extracts 12 states:
    1) Position (x, y, z)
    2) Orientation (roll, pitch, yaw)
    3) Linear velocity (vx, vy, vz)
    4) Angular velocity (wx, wy, wz)
- Updates a single 2D plot (x vs y) in real time with different colors:
    • Ground Truth (green)
    • Yaw-rate (red)
    • Single-track (blue)
    • Double-track (magenta)
- Combines the latest messages from each topic into one CSV row.
- Stops collecting data on receiving an Empty msg on /stop_collection.
- Saves all columns (time + 4*12 states = 1 + 48 = 49 columns) to a CSV file.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
import matplotlib.pyplot as plt
import csv
import os
import tf_transformations

class PlotMultiOdomNode(Node):
    def __init__(self):
        super().__init__('plot_multi_odom_node')
        self.get_logger().info("PlotMultiOdomNode has started.")

        # Declare parameters
        self.declare_parameter('ground_truth_topic', '/odometry/ground_truth')
        self.declare_parameter('yaw_rate_topic', '/odometry/yaw_rate')
        self.declare_parameter('single_track_topic', '/odometry/single_track')
        self.declare_parameter('double_track_topic', '/odometry/double_track')
        self.declare_parameter('update_rate', 5.0)
        self.declare_parameter('csv_filename', 'multi_odom_data.csv')

        # Get parameter values
        self.gt_topic = self.get_parameter('ground_truth_topic').value
        self.yaw_rate_topic = self.get_parameter('yaw_rate_topic').value
        self.single_track_topic = self.get_parameter('single_track_topic').value
        self.double_track_topic = self.get_parameter('double_track_topic').value
        self.update_rate = self.get_parameter('update_rate').value
        self.csv_filename = self.get_parameter('csv_filename').value

        # Buffers for real-time plotting (x-y)
        self.gt_x_vals, self.gt_y_vals = [], []
        self.yaw_rate_x_vals, self.yaw_rate_y_vals = [], []
        self.single_track_x_vals, self.single_track_y_vals = [], []
        self.double_track_x_vals, self.double_track_y_vals = [], []

        # For CSV: store latest 12-state data from each topic.
        # Each entry is a tuple:
        # (timestamp, x, y, z, roll, pitch, yaw, vx, vy, vz, wx, wy, wz)
        self.latest_data = {
            "ground_truth": None,
            "yaw_rate": None,
            "single_track": None,
            "double_track": None
        }

        # List of combined rows for CSV.
        # Each row: [combined_time,
        #            gt_12_states, yaw_rate_12_states, single_track_12_states, double_track_12_states]
        self.data_rows = []

        # Subscribers for each topic
        self.gt_sub = self.create_subscription(
            Odometry, self.gt_topic, lambda msg: self.odom_callback(msg, "ground_truth"), 10)
        self.yaw_rate_sub = self.create_subscription(
            Odometry, self.yaw_rate_topic, lambda msg: self.odom_callback(msg, "yaw_rate"), 10)
        self.single_track_sub = self.create_subscription(
            Odometry, self.single_track_topic, lambda msg: self.odom_callback(msg, "single_track"), 10)
        self.double_track_sub = self.create_subscription(
            Odometry, self.double_track_topic, lambda msg: self.odom_callback(msg, "double_track"), 10)

        # Subscriber to stop collection
        self.stop_sub = self.create_subscription(
            Empty, '/stop_collection', self.stop_collection_callback, 10)

        # Setup real-time plotting with 4 lines on one plot
        plt.ion()
        fig_width, fig_height = 4, 3.5  # Base size (adjustable)
        self.fig, self.ax = plt.subplots(figsize=(fig_width, fig_height))
        
        base_fontsize = min(fig_width, fig_height) * 1.5  # Adjust scale factor as needed
        self.ax.set_xlabel('X [m]', fontsize=base_fontsize * 0.8)
        self.ax.set_ylabel('Y [m]', fontsize=base_fontsize * 0.8)
        self.ax.set_title('2D Trajectories: Ground Truth vs. Odometry Estimates', fontsize=base_fontsize)
        
        self.gt_line, = self.ax.plot([], [], 'g-', label='Ground Truth')
        self.yaw_rate_line, = self.ax.plot([], [], 'r-', label='Yaw Rate')
        self.single_track_line, = self.ax.plot([], [], 'b-', label='Single Track')
        self.double_track_line, = self.ax.plot([], [], 'm-', label='Double Track')
        
        legend_fontsize = base_fontsize * 0.7  # Scale legend font
        self.ax.legend(loc='upper left', fontsize=legend_fontsize, framealpha=0.8)
    
        # Create timer for plotting
        timer_period = 1.0 / float(self.update_rate)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Flag to indicate whether we are still collecting data
        self.collecting_data = True

    def odom_callback(self, msg: Odometry, topic_key: str):
        if not self.collecting_data:
            return

        # Get current time
        t = self.get_clock().now().seconds_nanoseconds()
        curr_time = t[0] + t[1] * 1e-9

        # Extract position
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        pz = msg.pose.pose.position.z

        # Extract orientation (roll, pitch, yaw)
        quat = msg.pose.pose.orientation
        roll, pitch, yaw = tf_transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])

        # Extract linear velocity
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vz = msg.twist.twist.linear.z

        # Extract angular velocity
        wx = msg.twist.twist.angular.x
        wy = msg.twist.twist.angular.y
        wz = msg.twist.twist.angular.z

        # Save the latest data for this topic
        self.latest_data[topic_key] = (
            curr_time, px, py, pz, roll, pitch, yaw, vx, vy, vz, wx, wy, wz
        )

        # Update plotting buffers based on topic
        if topic_key == "ground_truth":
            self.gt_x_vals.append(px)
            self.gt_y_vals.append(py)
        elif topic_key == "yaw_rate":
            self.yaw_rate_x_vals.append(px)
            self.yaw_rate_y_vals.append(py)
        elif topic_key == "single_track":
            self.single_track_x_vals.append(px)
            self.single_track_y_vals.append(py)
        elif topic_key == "double_track":
            self.double_track_x_vals.append(px)
            self.double_track_y_vals.append(py)

        # Attempt to combine data from all topics into one row for CSV storage.
        self.try_combine_data()

    def try_combine_data(self):
        """
        Combine the latest data from all topics if available.
        The combined row uses the maximum timestamp from the latest messages.
        """
        if all(self.latest_data[tk] is not None for tk in self.latest_data):
            combined_time = max(data[0] for data in self.latest_data.values())
            # Build row: combined_time, then for each topic, the 12 states (skip the timestamp)
            row = [combined_time]
            for key in ["ground_truth", "yaw_rate", "single_track", "double_track"]:
                row.extend(self.latest_data[key][1:])
            self.data_rows.append(row)

    def stop_collection_callback(self, msg: Empty):
        if self.collecting_data:
            self.get_logger().info("Received stop_collection. Stopping data collection.")
            self.collecting_data = False
            # Unsubscribe to stop further data collection
            self.destroy_subscription(self.gt_sub)
            self.destroy_subscription(self.yaw_rate_sub)
            self.destroy_subscription(self.single_track_sub)
            self.destroy_subscription(self.double_track_sub)
            # Save CSV file
            self.save_data_to_csv()

    def save_data_to_csv(self):
        """
        Saves the collected data to CSV. The CSV contains:
        time, then ground_truth's 12 states, yaw_rate's 12 states,
        single_track's 12 states, and double_track's 12 states.
        Total columns: 1 + 12*4 = 49.
        """
        header = ["time"]
        for topic in ["ground_truth", "yaw_rate", "single_track", "double_track"]:
            header.extend([
                f"{topic}_px", f"{topic}_py", f"{topic}_pz",
                f"{topic}_roll", f"{topic}_pitch", f"{topic}_yaw",
                f"{topic}_vx", f"{topic}_vy", f"{topic}_vz",
                f"{topic}_wx", f"{topic}_wy", f"{topic}_wz"
            ])

        file_path = os.path.abspath(self.csv_filename)
        try:
            with open(file_path, mode='w', newline='') as csv_file:
                writer = csv.writer(csv_file)
                writer.writerow(header)
                for row in self.data_rows:
                    writer.writerow(row)
            self.get_logger().info(f"Saved data to CSV: {file_path}")
        except IOError as e:
            self.get_logger().error(f"Failed to write CSV: {e}")

    def timer_callback(self):
        """
        Update the 2D plot in real time.
        """
        # Update each line with the latest data
        if self.gt_x_vals and self.gt_y_vals:
            self.gt_line.set_xdata(self.gt_x_vals)
            self.gt_line.set_ydata(self.gt_y_vals)
        if self.yaw_rate_x_vals and self.yaw_rate_y_vals:
            self.yaw_rate_line.set_xdata(self.yaw_rate_x_vals)
            self.yaw_rate_line.set_ydata(self.yaw_rate_y_vals)
        if self.single_track_x_vals and self.single_track_y_vals:
            self.single_track_line.set_xdata(self.single_track_x_vals)
            self.single_track_line.set_ydata(self.single_track_y_vals)
        if self.double_track_x_vals and self.double_track_y_vals:
            self.double_track_line.set_xdata(self.double_track_x_vals)
            self.double_track_line.set_ydata(self.double_track_y_vals)

        # Set plot limits based on all data points
        all_x = self.gt_x_vals + self.yaw_rate_x_vals + self.single_track_x_vals + self.double_track_x_vals
        all_y = self.gt_y_vals + self.yaw_rate_y_vals + self.single_track_y_vals + self.double_track_y_vals
        if all_x and all_y:
            min_x, max_x = min(all_x), max(all_x)
            min_y, max_y = min(all_y), max(all_y)
            self.ax.set_xlim(min_x - 0.5, max_x + 0.5)
            self.ax.set_ylim(min_y - 0.5, max_y + 0.5)

        plt.draw()
        plt.pause(0.001)

def main(args=None):
    rclpy.init(args=args)
    node = PlotMultiOdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
