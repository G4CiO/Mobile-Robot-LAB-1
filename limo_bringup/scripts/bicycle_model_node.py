#!/usr/bin/python3

from limo_bringup.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import math


class BicycleModelNode(Node):
    def __init__(self):
        super().__init__('bicycle_model_node')
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.publisher = self.create_publisher(Float64MultiArray, '/velocity_controllers/commands', 10)

        # Vehicle Parameters
        self.declare_parameter('wheelbase', 1.0)  # meters

    def cmd_vel_callback(self, msg:Twist):
        wheelbase = self.get_parameter('wheelbase').value

        v = msg.linear.x
        omega = msg.angular.z

        delta = math.atan(wheelbase * omega / v) if v != 0 else 0

        # Publish single steering angle
        angle = Float64MultiArray()
        angle.data = [delta]  # Single steering angle
        self.publisher.publish(angle)

def main(args=None):
    rclpy.init(args=args)
    node = BicycleModelNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
