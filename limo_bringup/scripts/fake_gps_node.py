#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import numpy as np
from tf_transformations import euler_from_quaternion, quaternion_from_euler

# Import the custom service (make sure to adjust the package name accordingly)
from limo_gps_interfaces.srv import SetNoiseParams


class FakeGPSNode(Node):
    def __init__(self):
        super().__init__('fake_gps_node')
        self.get_logger().info("Fake GPS node (Odometry-based) has started.")

        # Parameters for noise
        self.declare_parameter('position_noise_std', 0.5)  # meters
        self.declare_parameter('orientation_noise_std', np.deg2rad(0.01))  # radians
        self.position_noise_std = self.get_parameter('position_noise_std').value
        self.orientation_noise_std = self.get_parameter('orientation_noise_std').value

        # Subscriber to ground truth odometry
        self.sub = self.create_subscription(
            Odometry, '/odometry/ground_truth', self.odom_callback, 10
        )

        # Publisher for fake GPS data as Odometry
        self.pub = self.create_publisher(Odometry, '/gps/odom', 10)

        # Create service for reconfiguring noise parameters
        self.srv = self.create_service(SetNoiseParams, 'set_noise_params', self.set_noise_params_callback)

    def odom_callback(self, msg):
        # Extract ground truth position and orientation
        x_gt = msg.pose.pose.position.x
        y_gt = msg.pose.pose.position.y
        quat_gt = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]
        # Convert quaternion to Euler angles (roll, pitch, yaw)
        (roll, pitch, yaw) = euler_from_quaternion(quat_gt)

        # Add Gaussian noise using the current parameters
        x_noisy = x_gt + np.random.normal(0.0, self.position_noise_std)
        y_noisy = y_gt + np.random.normal(0.0, self.position_noise_std)
        yaw_noisy = yaw + np.random.normal(0.0, self.orientation_noise_std)

        # Convert noisy yaw back to quaternion
        quat_noisy = quaternion_from_euler(0.0, 0.0, yaw_noisy)

        # Create noisy Odometry message
        gps_msg = Odometry()
        gps_msg.header = msg.header
        gps_msg.header.frame_id = "odom"  # Same frame as ground truth
        gps_msg.child_frame_id = "gps_frame"

        # Position
        gps_msg.pose.pose.position.x = x_noisy
        gps_msg.pose.pose.position.y = y_noisy
        gps_msg.pose.pose.position.z = 0.0  # Assuming 2D

        # Orientation
        gps_msg.pose.pose.orientation = Quaternion(
            x=quat_noisy[0],
            y=quat_noisy[1],
            z=quat_noisy[2],
            w=quat_noisy[3]
        )

        # Covariance (position and orientation noise)
        gps_msg.pose.covariance = [
            self.position_noise_std**2, 0.0, 0.0, 0.0, 0.0, 0.0,  # x
            0.0, self.position_noise_std**2, 0.0, 0.0, 0.0, 0.0,  # y
            0.0, 0.0, 100.0, 0.0, 0.0, 0.0,                       # z (large, unused)
            0.0, 0.0, 0.0, 100.0, 0.0, 0.0,                      # roll (large, unused)
            0.0, 0.0, 0.0, 0.0, 100.0, 0.0,                      # pitch (large, unused)
            0.0, 0.0, 0.0, 0.0, 0.0, self.orientation_noise_std**2  # yaw
        ]

        # Publish noisy GPS data
        self.pub.publish(gps_msg)

    def set_noise_params_callback(self, request, response):
        new_position_noise_std = request.position_noise_std
        new_orientation_noise_std = request.orientation_noise_std

        # Update the parameters dynamically
        self.set_parameters([
            rclpy.parameter.Parameter('position_noise_std', rclpy.Parameter.Type.DOUBLE, new_position_noise_std),
            rclpy.parameter.Parameter('orientation_noise_std', rclpy.Parameter.Type.DOUBLE, new_orientation_noise_std)
        ])

        # Also update the node's internal variables
        self.position_noise_std = new_position_noise_std
        self.orientation_noise_std = new_orientation_noise_std

        response.success = True
        response.message = (
            f"Updated noise parameters: position_noise_std = {new_position_noise_std}, "
            f"orientation_noise_std = {new_orientation_noise_std}"
        )
        self.get_logger().info(response.message)
        return response

def main(args=None):
    rclpy.init(args=args)
    node = FakeGPSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
