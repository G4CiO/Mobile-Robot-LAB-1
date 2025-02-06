#!/usr/bin/python3

from limo_bringup.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np



class AckermannSteeringNode(Node):
    def __init__(self):
        super().__init__('ackermann_steering_node')
        # Sub
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        # Pub
        self.publisher = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.wheel_velo_pub = self.create_publisher(Float64MultiArray, '/velocity_controllers/commands', 10)

        # Vehicle Parameters
        self.declare_parameter('wheelbase', 0.2)   # meters
        self.declare_parameter('track_width', 0.14) # meters
        self.declare_parameter('steering_ratio', 1.0)

    def cmd_vel_callback(self, msg:Twist):
        wheelbase = self.get_parameter('wheelbase').value
        track_width = self.get_parameter('track_width').value
        steering_ratio = self.get_parameter('steering_ratio').value

        # v = msg.linear.x
        v = 15.0
        omega = msg.angular.z

        if omega == 0:
            delta_L = delta_R = 0.0  # Moving straight
        else:
            delta_ack = math.atan(wheelbase * omega / v) if v != 0 else 0
            delta_ack /= steering_ratio

            delta_L = math.atan((wheelbase * math.tan(delta_ack)) / (wheelbase - 0.5 * track_width * math.tan(delta_ack)))
            delta_R = math.atan((wheelbase * math.tan(delta_ack)) / (wheelbase + 0.5 * track_width * math.tan(delta_ack)))
        
        print(f'delta_L: {delta_L}\ndelta_R: {delta_R}')

        # Create JointTrajectory message
        trajectory = JointTrajectory()
        # trajectory.header.stamp = self.get_clock().now().to_msg()
        # trajectory.header.stamp.sec = 0
        # trajectory.header.stamp.sec = 0
        trajectory.header.frame_id = ''
        trajectory.joint_names = ['left_steering_hinge_wheel', 'right_steering_hinge_wheel']

        # Create JointTrajectoryPoint for the steering angles
        point = JointTrajectoryPoint()
        point.positions = [delta_L, delta_R]
        point.velocities = [0.0, 0.0]  # Assuming zero velocities for now
        point.accelerations = [0.0, 0.0]  # Assuming zero accelerations for now
        point.time_from_start.sec = 1  # Adjust as needed
        point.time_from_start.nanosec = 0

        # Create VelocityControllers for the wheel velocity
        wheel_velocity = Float64MultiArray()
        wheel_velocity.data = [v, v]

        # Publish the wheel velocity
        self.wheel_velo_pub.publish(wheel_velocity)

        # Add the point to the trajectory
        trajectory.points = [point]

        # Publish the trajectory
        self.publisher.publish(trajectory)

def main(args=None):
    rclpy.init(args=args)
    node = AckermannSteeringNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
