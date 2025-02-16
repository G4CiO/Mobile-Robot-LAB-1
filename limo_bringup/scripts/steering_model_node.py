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

class SteeringModelNode(Node):
    def __init__(self):
        super().__init__('steering_model_node')
        self.get_logger().info("steering_model_node has been start.")
        # Sub
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        # Pub
        self.wheel_angle_pub = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.wheel_velo_pub = self.create_publisher(Float64MultiArray, '/velocity_controllers/commands', 10)
        self.wheel_pose_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)

        # Vehicle Parameters
        self.delta_steer = 0.0
        self.v = 0.0
        self.omega = 0.0
        self.declare_parameter('wheelbase', 0.2)   # meters
        self.declare_parameter('wheelradius', 0.045)   # meters
        self.declare_parameter('track_width', 0.14) # meters
        self.declare_parameter('steering_ratio', 1.0)
        self.declare_parameter('steering_mode', 'ackermann')  # Options: "ackermann" or "bicycle"

    def cmd_vel_callback(self, msg:Twist):
        self.v = msg.linear.x
        self.omega = msg.angular.z
        wheelbase = self.get_parameter('wheelbase').value
        track_width = self.get_parameter('track_width').value
        steering_ratio = self.get_parameter('steering_ratio').value
        wheelradius = self.get_parameter('wheelradius').value
        steering_mode = self.get_parameter('steering_mode').value  # Read mode parameter

        angular_velo_wheel = self.v / wheelradius

        if self.omega == 0:
            delta_L = delta_R = 0.0  # Moving straight
            self.delta_steer = 0.0
        else:
            self.delta_steer = math.atan(wheelbase * self.omega / self.v) if self.v != 0 else 0
            delta_ack = self.delta_steer / steering_ratio

            delta_L = math.atan((wheelbase * math.tan(delta_ack)) / (wheelbase - 0.5 * track_width * math.tan(delta_ack)))
            delta_R = math.atan((wheelbase * math.tan(delta_ack)) / (wheelbase + 0.5 * track_width * math.tan(delta_ack)))
        print('-------------------------------------------------------')
        print(f'steer angle : {self.delta_steer}')
        print(f'delta_L: {delta_L}, delta_R: {delta_R}')

        front_wheel_position = Float64MultiArray()
        if steering_mode == 'ackermann':
            front_wheel_position.data = [delta_L, delta_R, float(self.delta_steer)]
        elif steering_mode == 'bicycle':
            front_wheel_position.data = [float(self.delta_steer), float(self.delta_steer), float(self.delta_steer)]
        else:
            self.get_logger().warn("Invalid steering_mode parameter. Using Ackermann as default.")
            front_wheel_position.data = [delta_L, delta_R, float(self.delta_steer)] # Default to Ackermann

        # Create VelocityControllers for the wheel velocity
        wheel_velocity = Float64MultiArray()
        wheel_velocity.data = [angular_velo_wheel, angular_velo_wheel]

        # Publish the wheel velocity
        self.wheel_velo_pub.publish(wheel_velocity)

        # Publisg the front wheel position
        self.wheel_pose_pub.publish(front_wheel_position)

def main(args=None):
    rclpy.init(args=args)
    node = SteeringModelNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
