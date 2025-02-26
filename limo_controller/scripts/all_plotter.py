#!/usr/bin/python3

from limo_controller.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import os
from tf_transformations import euler_from_quaternion

class AllPlotter(Node):
    def __init__(self):
        super().__init__('all_plotter')
        self.declare_parameter('update_rate', 0.01)
        self.update_rate = self.get_parameter('update_rate').value

        # Subscriptions
        self.sub_ground_truth = self.create_subscription(Odometry, '/odometry/ground_truth', self.callback_ground_truth, 10)
        self.sub_yaw_rate = self.create_subscription(Odometry, '/odometry/yaw_rate', self.callback_yaw_rate, 10)
        self.sub_single_track = self.create_subscription(Odometry, '/odometry/single_track', self.callback_single_track, 10)
        self.sub_double_track = self.create_subscription(Odometry, '/odometry/double_track', self.callback_double_track, 10)

        # Data storage
        self.data = {
            'ground_truth': {'x': [], 'y': [], 'yaw': [], 'lin_x': [], 'ang_z': []},
            'yaw_rate': {'x': [], 'y': [], 'yaw': [], 'lin_x': [], 'ang_z': []},
            'single_track': {'x': [], 'y': [], 'yaw': [], 'lin_x': [], 'ang_z': []},
            'double_track': {'x': [], 'y': [], 'yaw': [], 'lin_x': [], 'ang_z': []}
        }

        # Timer
        self.timer = self.create_timer(self.update_rate, self.update_plot)

        # Matplotlib setup
        self.fig, self.axes = plt.subplots(4, 3, figsize=(12, 16))
        self.fig.suptitle('Odometry Comparison with Ground Truth')

    def process_odom(self, msg, key):
        self.data[key]['x'].append(msg.pose.pose.position.x)
        self.data[key]['y'].append(msg.pose.pose.position.y)
        quat = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        self.data[key]['yaw'].append(yaw)
        self.data[key]['lin_x'].append(msg.twist.twist.linear.x)
        self.data[key]['ang_z'].append(msg.twist.twist.angular.z)

    def callback_ground_truth(self, msg):
        self.process_odom(msg, 'ground_truth')

    def callback_yaw_rate(self, msg):
        self.process_odom(msg, 'yaw_rate')

    def callback_single_track(self, msg):
        self.process_odom(msg, 'single_track')

    def callback_double_track(self, msg):
        self.process_odom(msg, 'double_track')

    def update_plot(self):
        for ax in self.axes.flatten():
            ax.clear()

        labels = ['Yaw Rate', 'Single Track', 'Double Track']
        keys = ['yaw_rate', 'single_track', 'double_track']
        colors = ['red', 'green', 'purple']

        for i, (label, key, color) in enumerate(zip(labels, keys, colors)):
            # XY Position
            self.axes[0][i].plot(self.data['ground_truth']['x'], self.data['ground_truth']['y'], label='Ground Truth', linestyle='-', color='blue')
            self.axes[0][i].plot(self.data[key]['x'], self.data[key]['y'], label=label, linestyle=':', color=color)
            self.axes[0][i].set_title(f'Ground Truth vs {label} (XY)')
            self.axes[0][i].set_xlabel('X Coordinate')
            self.axes[0][i].set_ylabel('Y Coordinate')
            self.axes[0][i].legend()

            # Orientation (Yaw)
            self.axes[1][i].plot(self.data['ground_truth']['yaw'], label='Ground Truth', linestyle='-', color='blue')
            self.axes[1][i].plot(self.data[key]['yaw'], label=label, linestyle=':', color=color)
            self.axes[1][i].set_title(f'Ground Truth vs {label} (Yaw)')
            self.axes[1][i].set_xlabel('Time Step')
            self.axes[1][i].set_ylabel('Yaw (rad)')
            self.axes[1][i].legend()

            # Linear Velocity X
            self.axes[2][i].plot(self.data['ground_truth']['lin_x'], label='Ground Truth', linestyle='-', color='blue')
            self.axes[2][i].plot(self.data[key]['lin_x'], label=label, linestyle=':', color=color)
            self.axes[2][i].set_title(f'Ground Truth vs {label} (Linear Velocity X)')
            self.axes[2][i].set_xlabel('Time Step')
            self.axes[2][i].set_ylabel('Linear Velocity X (m/s)')
            self.axes[2][i].legend()

            # Angular Velocity Z
            self.axes[3][i].plot(self.data['ground_truth']['ang_z'], label='Ground Truth', linestyle='-', color='blue')
            self.axes[3][i].plot(self.data[key]['ang_z'], label=label, linestyle=':', color=color)
            self.axes[3][i].set_title(f'Ground Truth vs {label} (Angular Velocity Z)')
            self.axes[3][i].set_xlabel('Time Step')
            self.axes[3][i].set_ylabel('Angular Velocity Z (rad/s)')
            self.axes[3][i].legend()

        plt.pause(0.001)

    def save_plots(self):
        output_dir = 'plots'
        os.makedirs(output_dir, exist_ok=True)
        self.fig.savefig(os.path.join(output_dir, 'odometry_comparison.png'))
        self.get_logger().info('Plots saved as odometry_comparison.png')

    def destroy_node(self):
        self.save_plots()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AllPlotter()
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


