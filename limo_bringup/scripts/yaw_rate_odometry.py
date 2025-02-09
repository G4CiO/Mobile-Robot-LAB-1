#!/usr/bin/python3

from limo_bringup.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped, Vector3
from sensor_msgs.msg import Imu
import math
import tf2_ros
import tf_transformations

class YawRateOdometryNode(Node):
    def __init__(self):
        super().__init__('yaw_rate_odometry_node')
        self.get_logger().info("yaw_rate_odometry_node has been start.")

        # State variables
        self.x_prev = 0.0
        self.y_prev = 0.0
        self.theta_prev = 0.0
        self.linear_velocity = 0.0
        self.yaw_rate = 0.0

        # ROS 2 subscriptions
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Imu, '/imu_plugin/out', self.imu_callback, 10)

        # ROS 2 publishers
        self.odom_publisher = self.create_publisher(Odometry, '/odometry/yaw_rate', 10)
        
        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Timer to update odometry
        self.dt = 1/100
        self.create_timer(self.dt, self.update_odometry)

    def cmd_vel_callback(self, msg:Twist):
        """ Callback to get forward velocity from /cmd_vel topic """
        # self.linear_velocity = msg.linear.x
        self.linear_velocity = 0.5

    def imu_callback(self, msg:Imu):
        """ Callback to get yaw rate from /imu_plugin/out topic """
        self.yaw_rate = msg.angular_velocity.z  # Rotational velocity around Z-axis

    def update_odometry(self):
        # Publish Odom (odom -> base_footprint)
        self.publish_odom()
        # Publish TF transformation (odom -> base_footprint)
        self.publish_tf()

    def publish_odom(self):

        # Compute new pose        
        beta = 0.0  # Assuming no lateral slip
        x_curr = self.x_prev + self.linear_velocity * self.dt * math.cos(self.theta_prev + beta + ((self.yaw_rate * self.dt) / 2))
        y_curr = self.y_prev + self.linear_velocity * self.dt * math.sin(self.theta_prev + beta + ((self.yaw_rate * self.dt) / 2))
        theta_curr = self.theta_prev + self.yaw_rate * self.dt
        self.quaternion = tf_transformations.quaternion_from_euler(0.0, 0.0, self.theta_prev)
        # self.v_curr = (self.v_rl + self.v_rr)/2

        # Update state
        self.x_prev = x_curr
        self.y_prev = y_curr
        self.theta_prev = theta_curr

        # Publish odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_footprint"

        # Position
        odom_msg.pose.pose.position.x = self.x_prev
        odom_msg.pose.pose.position.y = self.y_prev
        odom_msg.pose.pose.position.z = 0.0

        # Orientation (Quaternion from Yaw)
        odom_msg.pose.pose.orientation.x = self.quaternion[0]
        odom_msg.pose.pose.orientation.y = self.quaternion[1]
        odom_msg.pose.pose.orientation.z = self.quaternion[2]
        odom_msg.pose.pose.orientation.w = self.quaternion[3]

        # Twist
        odom_msg.twist.twist.linear = Vector3(x=self.linear_velocity , y=0.0, z=0.0)
        odom_msg.twist.twist.angular = Vector3(x=0.0, y=0.0, z=self.yaw_rate)

        # Publish odometry
        self.odom_publisher.publish(odom_msg)

    def publish_tf(self):
        """ Publishes the transformation from 'odom' to 'base_footprint' """
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"

        # Position
        t.transform.translation.x = self.x_prev
        t.transform.translation.y = self.y_prev
        t.transform.translation.z = 0.0

        # Orientation (Quaternion from Yaw)
        t.transform.rotation.x = self.quaternion[0]
        t.transform.rotation.y = self.quaternion[1]
        t.transform.rotation.z = self.quaternion[2]
        t.transform.rotation.w = self.quaternion[3]

        # Publish TF transform
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = YawRateOdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
