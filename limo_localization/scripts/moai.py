#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math
import matplotlib.pyplot as plt
from tf_transformations import euler_from_quaternion

class OdomPlotter(Node):
    def __init__(self):
        super().__init__('odom_plotter')
        # Declare separate parameters for ground truth and EKF line widths
        self.declare_parameter('line_width_gt', 2.0)  # Default ground truth line width
        self.declare_parameter('line_width_ekf', 6.0)   # Default EKF line width
        
        self.line_width_gt = self.get_parameter('line_width_gt').value
        self.line_width_ekf = self.get_parameter('line_width_ekf').value

        # Subscribers for ground truth and EKF odometry
        self.create_subscription(Odometry, '/odometry/ground_truth', self.gt_callback, 10)
        self.create_subscription(Odometry, '/ekf_odom', self.ekf_callback, 10)
        
        # Data storage lists for ground truth and EKF data
        self.gt_data = []   # Each entry: dict with t, x, y, yaw, yaw_rate
        self.ekf_data = []
        self.start_time = self.get_clock().now().nanoseconds * 1e-9

        # Enable interactive plotting
        plt.ion()
        self.figure, _ = plt.subplots(3, 3, figsize=(18, 15))
        # Create a timer to update the plot every 0.5 seconds
        self.plot_timer = self.create_timer(0.5, self.update_plot)
    
    def get_relative_time(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        return now - self.start_time

    def extract_data(self, msg):
        # Extract pose and twist information
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        # Convert quaternion to yaw (assuming planar motion)
        orientation = msg.pose.pose.orientation
        yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])[2]
        yaw_rate = msg.twist.twist.angular.z
        t = self.get_relative_time()
        return t, x, y, yaw, yaw_rate

    def gt_callback(self, msg):
        t, x, y, yaw, yaw_rate = self.extract_data(msg)
        self.gt_data.append({'t': t, 'x': x, 'y': y, 'yaw': yaw, 'yaw_rate': yaw_rate})

    def ekf_callback(self, msg):
        t, x, y, yaw, yaw_rate = self.extract_data(msg)
        self.ekf_data.append({'t': t, 'x': x, 'y': y, 'yaw': yaw, 'yaw_rate': yaw_rate})

    def compute_errors(self):
        """
        Computes the absolute error differences for position, yaw, and yaw rate between EKF and ground truth.
        For position, the error is the Euclidean distance between (x,y) points.
        For yaw, the error is the absolute value of the normalized difference.
        For yaw rate, the error is the absolute difference.
        Then, the standard deviation (std) of each error array is calculated.
        """
        min_len = min(len(self.gt_data), len(self.ekf_data))
        if min_len == 0:
            return None, None, None, None, None, None

        pos_errors = []
        yaw_errors = []
        yaw_rate_errors = []
        for i in range(min_len):
            gt = self.gt_data[i]
            ekf = self.ekf_data[i]
            pos_err = math.sqrt((ekf['x'] - gt['x'])**2 + (ekf['y'] - gt['y'])**2)
            yaw_diff = ekf['yaw'] - gt['yaw']
            yaw_err = abs(math.atan2(math.sin(yaw_diff), math.cos(yaw_diff)))
            yaw_rate_err = abs(ekf['yaw_rate'] - gt['yaw_rate'])
            pos_errors.append(pos_err)
            yaw_errors.append(yaw_err)
            yaw_rate_errors.append(yaw_rate_err)

        def std_dev(errors):
            if len(errors) == 0:
                return 0
            mean_err = sum(errors) / len(errors)
            variance = sum((e - mean_err) ** 2 for e in errors) / len(errors)
            return math.sqrt(variance)

        pos_std = std_dev(pos_errors)
        yaw_std = std_dev(yaw_errors)
        yaw_rate_std = std_dev(yaw_rate_errors)
        return pos_errors, yaw_errors, yaw_rate_errors, pos_std, yaw_std, yaw_rate_std

    def update_plot(self):
        # Avoid updating if there is no data yet
        if not self.gt_data or not self.ekf_data:
            return

        # Prepare data from ground truth and EKF
        t_gt = [d['t'] for d in self.gt_data]
        x_gt = [d['x'] for d in self.gt_data]
        y_gt = [d['y'] for d in self.gt_data]
        yaw_gt = [d['yaw'] for d in self.gt_data]
        yaw_rate_gt = [d['yaw_rate'] for d in self.gt_data]

        t_ekf = [d['t'] for d in self.ekf_data]
        x_ekf = [d['x'] for d in self.ekf_data]
        y_ekf = [d['y'] for d in self.ekf_data]
        yaw_ekf = [d['yaw'] for d in self.ekf_data]
        yaw_rate_ekf = [d['yaw_rate'] for d in self.ekf_data]

        pos_errors, yaw_errors, yaw_rate_errors, pos_std, yaw_std, yaw_rate_std = self.compute_errors()

        # Clear and re-create subplots on the same figure
        self.figure.clf()
        axs = self.figure.subplots(3, 3)

        # Top row: Main comparison plots
        title_traj = 'Trajectory'
        if pos_std is not None:
            title_traj += f' (Pos Std: ±{pos_std:.2f} m)'
        axs[0, 0].plot(x_gt, y_gt, 'k-', linewidth=self.line_width_gt, label='Ground Truth')
        axs[0, 0].plot(x_ekf, y_ekf, color='blue', linewidth=self.line_width_ekf, alpha=0.4, label='EKF')
        axs[0, 0].set_xlabel('X (m)')
        axs[0, 0].set_ylabel('Y (m)')
        axs[0, 0].set_title(title_traj)
        axs[0, 0].legend()
        axs[0, 0].grid(True)

        title_yaw = 'Orientation'
        if yaw_std is not None:
            title_yaw += f' (Yaw Std: ±{yaw_std:.2f} rad)'
        axs[0, 1].plot(t_gt, yaw_gt, 'k-', linewidth=self.line_width_gt, label='Ground Truth')
        axs[0, 1].plot(t_ekf, yaw_ekf, color='green', linewidth=self.line_width_ekf, alpha=0.4, label='EKF')
        axs[0, 1].set_xlabel('Time (s)')
        axs[0, 1].set_ylabel('Yaw (rad)')
        axs[0, 1].set_title(title_yaw)
        axs[0, 1].legend()
        axs[0, 1].grid(True)

        title_yaw_rate = 'Yaw Rate'
        if yaw_rate_std is not None:
            title_yaw_rate += f' (Yaw Rate Std: ±{yaw_rate_std:.2f} rad/s)'
        axs[0, 2].plot(t_gt, yaw_rate_gt, 'k-', linewidth=self.line_width_gt, label='Ground Truth')
        axs[0, 2].plot(t_ekf, yaw_rate_ekf, color='purple', linewidth=self.line_width_ekf, alpha=0.4, label='EKF')
        axs[0, 2].set_xlabel('Time (s)')
        axs[0, 2].set_ylabel('Yaw Rate (rad/s)')
        axs[0, 2].set_title(title_yaw_rate)
        axs[0, 2].legend()
        axs[0, 2].grid(True)

        # Middle row: Error distributions (histograms)
        axs[1, 0].hist(pos_errors, bins=20, color='blue', alpha=0.7)
        axs[1, 0].set_title('Position Error Distribution')
        axs[1, 0].set_xlabel('Error (m)')
        axs[1, 0].set_ylabel('Frequency')
        axs[1, 0].grid(True)

        axs[1, 1].hist(yaw_errors, bins=20, color='green', alpha=0.7)
        axs[1, 1].set_title('Yaw Error Distribution')
        axs[1, 1].set_xlabel('Error (rad)')
        axs[1, 1].set_ylabel('Frequency')
        axs[1, 1].grid(True)

        axs[1, 2].hist(yaw_rate_errors, bins=20, color='purple', alpha=0.7)
        axs[1, 2].set_title('Yaw Rate Error Distribution')
        axs[1, 2].set_xlabel('Error (rad/s)')
        axs[1, 2].set_ylabel('Frequency')
        axs[1, 2].grid(True)

        # Third row: Error over time graphs
        axs[2, 0].plot(t_gt[:len(pos_errors)], pos_errors, color='blue', label='Pos Error')
        axs[2, 0].set_title('Position Error Over Time')
        axs[2, 0].set_xlabel('Time (s)')
        axs[2, 0].set_ylabel('Error (m)')
        axs[2, 0].legend()
        axs[2, 0].grid(True)

        axs[2, 1].plot(t_gt[:len(yaw_errors)], yaw_errors, color='green', label='Yaw Error')
        axs[2, 1].set_title('Yaw Error Over Time')
        axs[2, 1].set_xlabel('Time (s)')
        axs[2, 1].set_ylabel('Error (rad)')
        axs[2, 1].legend()
        axs[2, 1].grid(True)

        axs[2, 2].plot(t_gt[:len(yaw_rate_errors)], yaw_rate_errors, color='purple', label='Yaw Rate Error')
        axs[2, 2].set_title('Yaw Rate Error Over Time')
        axs[2, 2].set_xlabel('Time (s)')
        axs[2, 2].set_ylabel('Error (rad/s)')
        axs[2, 2].legend()
        axs[2, 2].grid(True)

        self.figure.tight_layout()
        self.figure.canvas.draw()
        plt.pause(0.001)

def main(args=None):
    rclpy.init(args=args)
    node = OdomPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down, stopping realtime plotting...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
