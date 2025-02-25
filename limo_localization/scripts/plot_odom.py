#!/usr/bin/env python3

"""
This ROS 2 node subscribes to two topics:
1) The ground truth odometry (default: /odometry/ground_truth)
2) One of the three odometry outputs from your robot (default: /odometry/yaw_rate)

It plots both sets of (x, y) positions in real time using matplotlib.

**Data collection stops** when an Empty message is published to /stop_collection,
but the plot remains open for further observation until you terminate the node (Ctrl+C).

Parameters:
- odom_topic (string): odometry topic to compare against ground truth
  (default: '/odometry/yaw_rate')
- ground_truth_topic (string): ground truth odometry topic (default: '/odometry/ground_truth')
- update_rate (float): how many times per second to update the real-time plot

Usage:
  ros2 run my_package plot_odom_node_stop --ros-args \
    -p odom_topic:=/odometry/double_track \
    -p update_rate:=10.0
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
import matplotlib.pyplot as plt

class PlotOdomNodeStop(Node):
    def __init__(self):
        super().__init__('plot_odom_node_stop')
        self.get_logger().info("plot_odom_node_stop has started.")

        # Declare parameters
        self.declare_parameter('odom_topic', '/odometry/yaw_rate')
        self.declare_parameter('ground_truth_topic', '/odometry/ground_truth')
        self.declare_parameter('update_rate', 5.0)

        # Get parameter values
        self.odom_topic = self.get_parameter('odom_topic').value
        self.gt_topic = self.get_parameter('ground_truth_topic').value
        self.update_rate = self.get_parameter('update_rate').value

        # Data buffers
        self.ground_truth_x = []
        self.ground_truth_y = []
        self.odom_x = []
        self.odom_y = []

        # Flag to indicate whether we are still collecting data
        self.collecting_data = True

        # Subscribers
        self.gt_sub = self.create_subscription(
            Odometry,
            self.gt_topic,
            self.ground_truth_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10
        )

        # Subscriber to stop data collection
        self.stop_sub = self.create_subscription(
            Empty,
            '/stop_collection',
            self.stop_collection_callback,
            10
        )

        # Setup real-time plotting
        plt.ion()  # interactive mode on
        self.fig, self.ax = plt.subplots()
        (self.gt_line,)   = self.ax.plot([], [], 'g-', label='Ground Truth')
        (self.odom_line,) = self.ax.plot([], [], 'r-', label='Selected Odom')

        self.ax.set_xlabel('X [m]')
        self.ax.set_ylabel('Y [m]')
        self.ax.legend()
        self.ax.set_title('Odom vs. Ground Truth (Live Plot)')

        # Create timer to update the plot at self.update_rate Hz
        timer_period = 1.0 / float(self.update_rate)
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def ground_truth_callback(self, msg: Odometry):
        # Only collect data if we're still active
        if self.collecting_data:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            self.ground_truth_x.append(x)
            self.ground_truth_y.append(y)

    def odom_callback(self, msg: Odometry):
        # Only collect data if we're still active
        if self.collecting_data:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            self.odom_x.append(x)
            self.odom_y.append(y)

    def stop_collection_callback(self, msg: Empty):
        """
        Triggered when we receive an empty message on /stop_collection.
        This stops further data collection, but keeps the plot running.
        """
        if self.collecting_data:
            self.get_logger().info("Received stop_collection. Stopping data collection.")
            self.collecting_data = False

    def timer_callback(self):
        """
        Real-time update of the matplotlib plot.
        Called at self.update_rate Hz.
        """
        # Update the plot if we have enough data
        if len(self.ground_truth_x) > 1 and len(self.odom_x) > 1:
            self.gt_line.set_xdata(self.ground_truth_x)
            self.gt_line.set_ydata(self.ground_truth_y)
            self.odom_line.set_xdata(self.odom_x)
            self.odom_line.set_ydata(self.odom_y)

            # Dynamically adjust axes
            all_x = self.ground_truth_x + self.odom_x
            all_y = self.ground_truth_y + self.odom_y
            min_x, max_x = min(all_x), max(all_x)
            min_y, max_y = min(all_y), max(all_y)
            self.ax.set_xlim(min_x - 0.5, max_x + 0.5)
            self.ax.set_ylim(min_y - 0.5, max_y + 0.5)

            plt.draw()
            plt.pause(0.001)

def main(args=None):
    rclpy.init(args=args)
    node = PlotOdomNodeStop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
