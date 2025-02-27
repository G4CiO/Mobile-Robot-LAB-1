#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from tf_transformations import euler_from_quaternion
import numpy as np
import matplotlib.pyplot as plt

def normalize_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

class EKFDiagnosticsPlotNode(Node):
    def __init__(self):
        super().__init__('ekf_diagnostics_plot_node')
        self.get_logger().info("EKF Diagnostics Plot Node started.")
        
        self.collecting_data = True
        self.ekf_x, self.ekf_y = [], []
        self.gt_x, self.gt_y = [], []
        self.pos_errors = []
        self.latest_gt = None
        
        self.create_subscription(Odometry, '/ekf_odom', self.ekf_odom_callback, 10)
        self.create_subscription(Odometry, '/odometry/ground_truth', self.gt_callback, 10)
        self.create_subscription(Empty, '/stop_collection', self.stop_callback, 10)
    
    def ekf_odom_callback(self, msg: Odometry):
        if not self.collecting_data:
            return
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        self.ekf_x.append(x)
        self.ekf_y.append(y)
        
        if self.latest_gt is not None:
            gt_x, gt_y = self.latest_gt
            err = np.sqrt((x - gt_x)**2 + (y - gt_y)**2)
            self.pos_errors.append(err)
    
    def gt_callback(self, msg: Odometry):
        if not self.collecting_data:
            return
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        self.gt_x.append(x)
        self.gt_y.append(y)
        self.latest_gt = (x, y)
    
    def stop_callback(self, msg: Empty):
        if self.collecting_data:
            self.get_logger().info("Stopping data collection and generating final plot.")
            self.collecting_data = False
            self.plot_final_graph()
    
    def plot_final_graph(self):
        if not self.gt_x or not self.ekf_x:
            self.get_logger().warn("No data to plot.")
            return
        
        estimates_x = np.array(self.ekf_x)
        estimates_y = np.array(self.ekf_y)
        true_x = np.array(self.gt_x)
        true_y = np.array(self.gt_y)
        
        if len(self.pos_errors) > 0:
            mean_error = np.mean(self.pos_errors)
            std_error = np.std(self.pos_errors)
        else:
            mean_error, std_error = 0.0, 0.0
        
        confidence_interval = 1.96 * std_error
        upper_bound = estimates_y + confidence_interval
        lower_bound = estimates_y - confidence_interval
        
        plt.figure(figsize=(8, 5))
        plt.plot(true_x, true_y, 'g-', label='True values')
        plt.plot(estimates_x, estimates_y, 'b-', label='EKF Estimates')
        plt.fill_between(estimates_x, lower_bound, upper_bound, color='orange', alpha=0.3, label='95% confidence interval')
        
        plt.xlabel('Measurement number')
        plt.ylabel('Position (m)')
        plt.title('EKF Position Estimation with Confidence Interval')
        plt.legend()
        plt.grid()
        plt.show()
        
        self.get_logger().info(f"Final RMSE Position: {mean_error:.4f} m")

def main(args=None):
    rclpy.init(args=args)
    node = EKFDiagnosticsPlotNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
