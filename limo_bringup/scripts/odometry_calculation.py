#!/usr/bin/python3

from limo_bringup.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped, Vector3
from sensor_msgs.msg import Imu, JointState
import math
import tf2_ros
import tf_transformations

class OdometryCalcurationNode(Node):
    def __init__(self):
        super().__init__('odometry_calculation_node')
        self.get_logger().info("odometry_calculation_node has been start.")

        # Vehicle Parameters
        self.declare_parameter('wheelbase', 0.2)   # meters
        self.declare_parameter('wheelradius', 0.045)   # meters
        self.declare_parameter('track_width', 0.14) # meters
        self.declare_parameter('steering_ratio', 1.0)
        self.wheelbase = self.get_parameter('wheelbase').value
        self.track_width = self.get_parameter('track_width').value
        self.wheelradius = self.get_parameter('wheelradius').value

        # State variables
        self.v_rl = 0.0
        self.v_rr = 0.0

        self.x_curr = 0.0 # m
        self.y_curr = 0.0 # m
        self.theta_curr = 0.0 # rad
        self.v_curr = 0.0 # m/s
        self.w_curr = 0.0 # rad/s
        self.x_prev = 0.0 # m
        self.y_prev = 0.0 # m
        self.theta_prev = 0.0 # rad
        self.v_prev = 0.0 # m/s
        self.w_prev = 0.0 # rad/s
        self.linear_velocity = 0.0 # m/s
        self.yaw_rate = 0.0 # rad/s

        self.x_curr_2Track = 0.0 # m
        self.y_curr_2Track = 0.0 # m
        self.theta_curr_2Track = 0.0 # rad
        self.v_curr_2Track = 0,0 # m/s
        self.w_curr_2Track = 0,0 # rad/s
        self.x_prev_2Track = 0.0 # m
        self.y_prev_2Track = 0.0 # m
        self.theta_prev_2Track = 0.0 # rad
        self.v_prev_2Track = 0.0 # m/s
        self.w_prev_2Track = 0.0 # rad/s

        self.x_prev_1Track = 0.0
        self.x_curr_1Track = 0.0
        self.y_prev_1Track = 0.0
        self.y_curr_1Track = 0.0
        self.v_prev_1Track = 0.0
        self.v_curr_1Track = 0.0
        self.w_prev_1Track = 0.0
        self.w_curr_1Track = 0.0
        self.theta_prev_1Track = 0.0
        self.theta_curr_1Track = 0.0

        self.BETA = 0.0  # Assuming no lateral slip
        # self.L_REAR_STEER_ANGLE = 0.0 # Left rear wheel steering angle (rad)
        # self.R_REAR_STEER_ANGLE = 0.0 # Right rear wheel steering angle (rad)
        # self.L_RX = -self.wheelbase/2  # Left rear wheel x-offset
        # self.R_RX = -self.wheelbase/2  # Right rear wheel x-offset 
        # self.L_RY = self.track_width/2  # Left rear wheel y-offset 
        # self.R_RY = -self.track_width/2   # Right rear wheel y-offset 

        self.delta = 0.0
        self.v = 0.0

        # ROS 2 subscriptions
        self.create_subscription(Imu, '/imu_plugin/out', self.imu_callback, 10)
        self.create_subscription(JointState, '/joint_states', self.jointstates_callback, 10)

        # ROS 2 publishers
        self.yaw_rate_publisher = self.create_publisher(Odometry, '/odometry/yaw_rate', 10)
        self.single_track_publisher = self.create_publisher(Odometry, '/odometry/single_track', 10)
        self.double_track_publisher = self.create_publisher(Odometry, '/odometry/double_track', 10)
        
        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Timer to update odometry
        self.dt = 1/100
        self.create_timer(self.dt, self.update_odometry)

    def imu_callback(self, msg:Imu):
        """ Callback to get yaw rate from /imu_plugin/out topic """
        self.yaw_rate = msg.angular_velocity.z  # Rotational velocity around Z-axis

    def jointstates_callback(self, msg:JointState):
        """ Callback to get JointState from /jointstates topic """
        self.steering_angle = msg.position[4]
        self.v_rl = msg.velocity[0] * self.wheelradius
        self.v_rr = msg.velocity[1] * self.wheelradius

    def update_odometry(self):
        # Publish Odom (odom -> base_footprint)
        self.OdoYawRate()
        self.Odo1Track()
        self.Odo2Track()
        # Publish TF transformation (odom -> base_footprint)
        self.publish_tf()

    def OdoYawRate(self):

        # Compute new pose        
        self.x_curr = self.x_prev + self.v_prev * self.dt * math.cos(self.theta_prev + self.BETA + ((self.w_prev * self.dt) / 2))
        self.y_curr = self.y_prev + self.v_prev * self.dt * math.sin(self.theta_prev + self.BETA + ((self.w_prev * self.dt) / 2))
        self.theta_curr = self.theta_prev + self.w_prev * self.dt
        self.quaternion = tf_transformations.quaternion_from_euler(0.0, 0.0, self.theta_curr)
        self.v_curr = (self.v_rl + self.v_rr)/2
        self.w_curr = self.yaw_rate

        # Publish odometry message
        self.publish_odom("odom", "base_footprint", self.x_curr, self.y_curr, self.quaternion, self.v_curr, self.w_curr, self.yaw_rate_publisher)

        # Update state
        self.x_prev = self.x_curr
        self.y_prev = self.y_curr
        self.v_prev = self.v_curr
        self.w_prev = self.w_curr
        self.theta_prev = self.theta_curr

    def Odo1Track(self):
        
        self.x_curr_1Track = self.x_prev_1Track + self.v_prev_1Track * self.dt * math.cos(self.theta_prev_1Track + self.BETA + ((self.w_prev_1Track * self.dt) / 2))
        self.y_curr_1Track = self.y_prev_1Track + self.v_prev_1Track * self.dt * math.sin(self.theta_prev_1Track + self.BETA + ((self.w_prev_1Track * self.dt) / 2))
        self.theta_curr_1Track = self.theta_prev_1Track + self.w_prev_1Track * self.dt
        self.quaternion_1Track = tf_transformations.quaternion_from_euler(0.0, 0.0, self.theta_curr_1Track)
        self.v_curr_1Track = (self.v_rl + self.v_rr) / 2
        self.w_curr_1Track = (self.v_prev_1Track / self.wheelbase) * math.tan(self.steering_angle)

        self.publish_odom("odom", "base_footprint", self.x_curr_1Track, self.y_curr_1Track, self.quaternion_1Track, self.v_curr_1Track, self.w_curr_1Track, self.single_track_publisher)

        # Update state
        self.x_prev_1Track = self.x_curr_1Track
        self.y_prev_1Track = self.y_curr_1Track
        self.v_prev_1Track = self.v_curr_1Track
        self.w_prev_1Track = self.w_curr_1Track
        self.theta_prev_1Track = self.theta_curr_1Track

    def Odo2Track(self):

        self.x_curr_2Track = self.x_prev_2Track + self.v_prev_2Track * self.dt * math.cos(self.theta_prev_2Track + self.BETA + ((self.w_prev_2Track * self.dt) / 2))
        self.y_curr_2Track = self.y_prev_2Track + self.v_prev_2Track * self.dt * math.sin(self.theta_prev_2Track + self.BETA + ((self.w_prev_2Track * self.dt) / 2))
        self.theta_curr_2Track = self.theta_prev_2Track + self.w_prev_2Track * self.dt
        self.quaternion_2Track = tf_transformations.quaternion_from_euler(0.0, 0.0, self.theta_curr_2Track)

        # A1 = self.L_RX * self.v_rr * math.sin(self.L_REAR_STEER_ANGLE)
        # A2 = self.L_RY * self.v_rr * math.cos(self.L_REAR_STEER_ANGLE)
        # A3 = self.R_RX * self.v_rl * math.sin(self.R_REAR_STEER_ANGLE)
        # A4 = self.R_RY * self.v_rl * math.cos(self.R_REAR_STEER_ANGLE)

        # B1 = self.L_RX * math.sin(self.L_REAR_STEER_ANGLE) * math.cos(self.R_REAR_STEER_ANGLE - self.BETA)
        # B2 = self.L_RY * math.cos(self.L_REAR_STEER_ANGLE) * math.cos(self.R_REAR_STEER_ANGLE - self.BETA)
        # B3 = self.R_RX * math.sin(self.R_REAR_STEER_ANGLE) * math.cos(self.L_REAR_STEER_ANGLE - self.BETA)
        # B4 = self.R_RY * math.cos(self.R_REAR_STEER_ANGLE) * math.cos(self.L_REAR_STEER_ANGLE - self.BETA)

        # C1 = self.v_rl * math.cos(self.R_REAR_STEER_ANGLE - self.BETA)
        # C2 = self.v_rr * math.cos(self.L_REAR_STEER_ANGLE - self.BETA)

        # self.v_curr_2Track = (A1 - A2 - A3 + A4) / (B1 - B2 - B3 + B4)
        # self.w_curr_2Track = (C1 - C2) / (B1 - B2 - B3 + B4)
        self.v_curr_2Track = (self.v_rl + self.v_rr) / 2
        # self.w_curr_2Track = (self.v_rr - self.v_rl) / (self.R_RY - self.L_RY)
        self.w_curr_2Track = (self.v_rr - self.v_rl) / self.track_width

        # Publish odometry message
        self.publish_odom("odom", "base_footprint", self.x_curr_2Track, self.y_curr_2Track, self.quaternion_2Track, self.v_curr_2Track, self.w_curr_2Track, self.double_track_publisher)

        # Update state
        self.x_prev_2Track = self.x_curr_2Track
        self.y_prev_2Track = self.y_curr_2Track
        self.v_prev_2Track = self.v_curr_2Track
        self.w_prev_2Track = self.w_curr_2Track
        self.theta_prev_2Track = self.theta_curr_2Track 

    def publish_odom(self, frame_id, child_frame_id, pose_x, pose_y, quaternion_list, v_curr, w_curr, publisher):
        # Publish odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = frame_id
        odom_msg.child_frame_id = child_frame_id

        # Position
        odom_msg.pose.pose.position.x = pose_x
        odom_msg.pose.pose.position.y = pose_y
        odom_msg.pose.pose.position.z = 0.0

        # Orientation (Quaternion from Yaw)
        odom_msg.pose.pose.orientation.x = quaternion_list[0]
        odom_msg.pose.pose.orientation.y = quaternion_list[1]
        odom_msg.pose.pose.orientation.z = quaternion_list[2]
        odom_msg.pose.pose.orientation.w = quaternion_list[3]

        # Twist
        odom_msg.twist.twist.linear = Vector3(x=v_curr , y=0.0, z=0.0)
        odom_msg.twist.twist.angular = Vector3(x=0.0, y=0.0, z=w_curr)

        # Publish odometry
        publisher.publish(odom_msg)    


    def publish_tf(self):
        """ Publishes the transformation from 'odom' to 'base_footprint' """
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"

        # Position
        t.transform.translation.x = self.x_curr
        t.transform.translation.y = self.y_curr
        t.transform.translation.z = 0.0

        # Orientation
        t.transform.rotation.x = self.quaternion[0]
        t.transform.rotation.y = self.quaternion[1]
        t.transform.rotation.z = self.quaternion[2]
        t.transform.rotation.w = self.quaternion[3]

        # Publish TF transform
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryCalcurationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
