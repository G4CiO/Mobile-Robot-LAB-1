#!/usr/bin/python3

from limo_bringup.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray
import math


class BicycleModelNode(Node):
    def __init__(self):
        super().__init__('bicycle_model_node')
        self.get_logger().info("bicycle_model_node has been start.")
        # Sub
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        # Pub
        self.wheel_angle_pub = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.wheel_velo_pub = self.create_publisher(Float64MultiArray, '/velocity_controllers/commands', 10)

        # Vehicle Parameters
        self.delta_steer = 0.0
        self.declare_parameter('wheelbase', 1.0)  # meters
        self.declare_parameter('wheelradius', 0.045)   # meters

    def cmd_vel_callback(self, msg:Twist):
        wheelbase = self.get_parameter('wheelbase').value
        wheelradius = self.get_parameter('wheelradius').value

        v = msg.linear.x
        angular_velo_wheel = v / wheelradius
        omega = msg.angular.z
        if omega == 0:
            self.delta_steer = 0.0
        else:
            # delta = math.atan(wheelbase * omega / v) if v != 0 else 0
            epsilon = 1e-6  # Small threshold to avoid division by zero
            self.delta_steer = math.atan(wheelbase * omega / (v if abs(v) > epsilon else epsilon))

        # Create JointTrajectory message
        trajectory = JointTrajectory()
        trajectory.header.frame_id = ''
        trajectory.joint_names = ['left_steering_hinge_wheel', 'right_steering_hinge_wheel', 'steering']

        # Create JointTrajectoryPoint for the steering angles
        point = JointTrajectoryPoint()
        point.positions = [float(self.delta_steer), float(self.delta_steer), float(self.delta_steer)]
        point.velocities = [0.0, 0.0, 0.0]  # Assuming zero velocities for now
        point.accelerations = [0.0, 0.0, 0.0]  # Assuming zero accelerations for now
        point.time_from_start.sec = 0  # Adjust as needed
        second = 0.5
        sec_to_nanosec = second * pow(10,9)
        point.time_from_start.nanosec = int(sec_to_nanosec)

        # Create VelocityControllers for the wheel velocity
        wheel_velocity = Float64MultiArray()
        wheel_velocity.data = [angular_velo_wheel, angular_velo_wheel]

        # Publish the wheel velocity
        self.wheel_velo_pub.publish(wheel_velocity)

        # Add the point to the trajectory
        trajectory.points = [point]

        # Publish the trajectory
        self.wheel_angle_pub.publish(trajectory)

def main(args=None):
    rclpy.init(args=args)
    node = BicycleModelNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
