#!/usr/bin/python3

from limo_controller.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import os

class OdometryPlotter(Node):
    def __init__(self):
        super().__init__('odometry_plotter')
        self.declare_parameter('update_rate', 0.01)
        self.update_rate = self.get_parameter('update_rate').value

        # Subscriptions
        self.sub_ground_truth = self.create_subscription(Odometry, '/odometry/ground_truth', self.callback_ground_truth, 10)
        self.sub_yaw_rate = self.create_subscription(Odometry, '/odometry/yaw_rate', self.callback_yaw_rate, 10)
        self.sub_single_track = self.create_subscription(Odometry, '/odometry/single_track', self.callback_single_track, 10)
        self.sub_double_track = self.create_subscription(Odometry, '/odometry/double_track', self.callback_double_track, 10)

        # Data storage
        self.data = {
            'ground_truth': {'x': [], 'y': []},
            'yaw_rate': {'x': [], 'y': []},
            'single_track': {'x': [], 'y': []},
            'double_track': {'x': [], 'y': []}
        }

        # Timer
        self.timer = self.create_timer(self.update_rate, self.update_plot)

        # Matplotlib setup
        self.fig, self.axes = plt.subplots(1, 3, figsize=(12, 4))
        self.fig.suptitle('Odometry Comparison with Ground Truth by No Slip condition constraints')

    def callback_ground_truth(self, msg:Odometry):
        self.data['ground_truth']['x'].append(msg.pose.pose.position.x)
        self.data['ground_truth']['y'].append(msg.pose.pose.position.y)

    def callback_yaw_rate(self, msg:Odometry):
        self.data['yaw_rate']['x'].append(msg.pose.pose.position.x)
        self.data['yaw_rate']['y'].append(msg.pose.pose.position.y)

    def callback_single_track(self, msg:Odometry):
        self.data['single_track']['x'].append(msg.pose.pose.position.x)
        self.data['single_track']['y'].append(msg.pose.pose.position.y)

    def callback_double_track(self, msg:Odometry):
        self.data['double_track']['x'].append(msg.pose.pose.position.x)
        self.data['double_track']['y'].append(msg.pose.pose.position.y)

    def update_plot(self):
        self.axes[0].clear()
        self.axes[1].clear()
        self.axes[2].clear()

        # Ground truth vs Yaw rate
        self.axes[0].plot(self.data['ground_truth']['x'], self.data['ground_truth']['y'], label='Ground Truth', linestyle='-', color='blue')
        self.axes[0].plot(self.data['yaw_rate']['x'], self.data['yaw_rate']['y'], label='Yaw Rate', linestyle=':', color='red')
        self.axes[0].set_title('Ground Truth vs Yaw Rate')
        self.axes[0].set_xlabel('X Coordinate')
        self.axes[0].set_ylabel('Y Coordinate')
        self.axes[0].legend()

        # Ground truth vs Single track
        self.axes[1].plot(self.data['ground_truth']['x'], self.data['ground_truth']['y'], label='Ground Truth', linestyle='-', color='blue')
        self.axes[1].plot(self.data['single_track']['x'], self.data['single_track']['y'], label='Single Track', linestyle=':', color='green')
        self.axes[1].set_title('Ground Truth vs Single Track')
        self.axes[1].set_xlabel('X Coordinate')
        self.axes[1].set_ylabel('Y Coordinate')
        self.axes[1].legend()

        # Ground truth vs Double track
        self.axes[2].plot(self.data['ground_truth']['x'], self.data['ground_truth']['y'], label='Ground Truth', linestyle='-', color='blue')
        self.axes[2].plot(self.data['double_track']['x'], self.data['double_track']['y'], label='Double Track', linestyle=':', color='purple')
        self.axes[2].set_title('Ground Truth vs Double Track')
        self.axes[2].set_xlabel('X Coordinate')
        self.axes[2].set_ylabel('Y Coordinate')
        self.axes[2].legend()

        plt.pause(0.001)

    def save_plots(self):
        output_dir = 'plots'
        os.makedirs(output_dir, exist_ok=True)
        self.fig.savefig(os.path.join(output_dir, 'odometry_comparison_new_single_track.png'))
        self.get_logger().info('Plots saved as odometry_comparison.png')

    def destroy_node(self):
        # self.save_plots()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OdometryPlotter()
    try:
        plt.ion()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        plt.ioff()
        plt.show()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

