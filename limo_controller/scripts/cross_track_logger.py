#!/usr/bin/python3

from limo_controller.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
import yaml
import os
import matplotlib.pyplot as plt
import numpy as np
from ament_index_python.packages import get_package_share_directory

class CrossTrackLogger(Node):
    def __init__(self):
        super().__init__('cross_track_logger')

        pkg_name = 'limo_controller'
        path_pkg_share_path = get_package_share_directory(pkg_name)
        ws_path, _ = path_pkg_share_path.split('install')
        file = 'pid.yaml'
        self.yaml_file = os.path.join(ws_path, 'src/Mobile-Robot-LAB-1', pkg_name, 'config', file)
        file_path = 'path.yaml'
        self.path_path = os.path.join(ws_path, 'src/Mobile-Robot-LAB-1', pkg_name, 'config', file_path)
        # Load path from YAML file
        self.path = self.load_path()
        self.data = {}
        
        if os.path.exists(self.yaml_file):
            with open(self.yaml_file, 'r') as f:
                self.data = yaml.safe_load(f) or {}
        self.t_x = (np.linspace(-10.0, 5.0, 1000))
        k = 2
        x_shift = -7
        y_shift = -8
        self.t_y = 1 / (1 + np.exp(-k * (self.t_x - x_shift))) + y_shift

        self.plot_all()

    def load_path(self):
        with open(self.path_path, 'r') as file:
            return yaml.safe_load(file)

    def plot_all(self):
        fig, axes = plt.subplots(1, 1, figsize=(9, 5))
        
        # # Plot CTE
        # for k, values in self.data.items():
        #     time_steps = [i * 0.2 for i in range(len(values['cte']))]  # Create time axis with 0.2s intervals
        #     axes[0].plot(time_steps, values['cte'], label=k)
        # axes[0].set_xlabel('Time step (second)')
        # axes[0].set_ylabel('Heading Error (rad)')
        # axes[0].legend()
        # axes[0].set_title('Heading Error over Time')
        # axes[0].grid()

        # Plot Loaded Path (self.path from the loaded YAML)
        # if self.path:
        x_vals_loaded = [p for p in self.t_x]
        y_vals_loaded = [p for p in self.t_y]
        axes.plot(x_vals_loaded, y_vals_loaded, label="Path", linestyle='--', color='black')

        # Plot Path
        for k, values in self.data.items():
            x_vals = [p['x'] for p in values['path']]
            y_vals = [p['y'] for p in values['path']]
            axes.plot(x_vals, y_vals, label=k)
        axes.set_xlabel('X position (m)')
        axes.set_ylabel('Y position (m)')
        axes.legend()
        axes.set_title('Ki Tuning')
        axes.grid()

        # Save and show the plot
        plt.tight_layout()
        # plt.savefig('cross_track_combined.png')
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = CrossTrackLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
