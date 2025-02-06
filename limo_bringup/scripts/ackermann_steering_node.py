#!/usr/bin/python3

from limo_bringup.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import math
from sensor_msgs.msg import JointState
import numpy as np



class AckermannSteeringNode(Node):
    def __init__(self):
        super().__init__('ackermann_steering_node')
        # Sub
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        # Pub
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.publisher = self.create_publisher(Float64MultiArray, '/velocity_controllers/commands', 10)

        # Vehicle Parameters
        self.declare_parameter('wheelbase', 0.2)   # meters
        self.declare_parameter('track_width', 0.14) # meters
        self.declare_parameter('steering_ratio', 1.0)

    def pub_joint_states(self, q):
        joint_msg = JointState()
        joint_msg.name = ['rear_left_wheel', 'rear_right_wheel']
        joint_msg.header.stamp = self.get_clock().now().to_msg()

        # Ensure q contains valid float values
        if np.all(np.isfinite(q)) and np.all(np.abs(q) < 1e6):  # Adding a reasonable range check
            joint_msg.position = q.tolist()  # Convert NumPy array to list of floats
            self.joint_state_pub.publish(joint_msg)


    def cmd_vel_callback(self, msg:Twist):
        wheelbase = self.get_parameter('wheelbase').value
        track_width = self.get_parameter('track_width').value
        steering_ratio = self.get_parameter('steering_ratio').value

        v = msg.linear.x
        omega = msg.angular.z

        if omega == 0:
            delta_L = delta_R = 0.0  # Moving straight
        else:
            delta_ack = math.atan(wheelbase * omega / v) if v != 0 else 0
            delta_ack /= steering_ratio

            delta_L = math.atan((wheelbase * math.tan(delta_ack)) / (wheelbase - 0.5 * track_width * math.tan(delta_ack)))
            delta_R = math.atan((wheelbase * math.tan(delta_ack)) / (wheelbase + 0.5 * track_width * math.tan(delta_ack)))

        # Publish steering angles
        angles = Float64MultiArray()
        angles.data = [delta_L, delta_R]
        self.publisher.publish(angles)

def main(args=None):
    rclpy.init(args=args)
    node = AckermannSteeringNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
