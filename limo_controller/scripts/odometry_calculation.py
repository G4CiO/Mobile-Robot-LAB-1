#!/usr/bin/python3

from limo_bringup.dummy_module import dummy_function, dummy_var
import rclpy
import math
import tf2_ros
import tf_transformations
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Path
from geometry_msgs.msg import TransformStamped, Vector3
from std_srvs.srv import Empty
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

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
        self.steering_ratio = self.get_parameter('steering_ratio').value

        # State variables
        self.v_rl = 0.0
        self.v_rr = 0.0
        self.v_avr = 0.0
        self.delta_L = 0.0
        self.delta_R = 0.0
        self.BETA = 0.0  # Assuming no lateral slip
        # Offset odometry
        self.off_x = 9.073496746393584
        self.off_yaw = 1.5700039414375448

        self.x_curr = self.off_x # m
        self.y_curr = 0.0 # m
        self.v_curr = 0.0 # m/s
        self.w_curr = 0.0 # rad/s
        self.yaw_rate = 0.0 # rad/s
        self.theta_curr = self.off_yaw# rad

        self.x_curr_1Track = self.off_x # m
        self.y_curr_1Track = 0.0 # m
        self.v_curr_1Track = 0.0 # m/s
        self.w_curr_1Track = 0.0 # rad/s
        self.theta_curr_1Track = self.off_yaw # rad

        self.x_curr_2Track = self.off_x # m
        self.y_curr_2Track = 0.0 # m
        self.v_curr_2Track = 0.0 # m/s
        self.w_curr_2Track = 0.0 # rad/s
        self.theta_curr_2Track = self.off_yaw # rad

        self.x_gt = 0.0
        self.y_gt = 0.0
        self.quat_gt = []

        # ROS 2 subscriptions
        self.create_subscription(Imu, '/limo/imu', self.imu_callback, 10)
        self.create_subscription(JointState, '/joint_states', self.jointstates_callback, 10)
        self.create_subscription(Odometry, '/odometry/ground_truth', self.OdoGroundTruth, 10)

        # ROS 2 publishers
        self.yaw_rate_publisher = self.create_publisher(Odometry, '/odometry/yaw_rate', 10)
        self.single_track_publisher = self.create_publisher(Odometry, '/odometry/single_track', 10)
        self.double_track_publisher = self.create_publisher(Odometry, '/odometry/double_track', 10)
        
        # TF broadcaster
        # self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        # Timer to update odometry
        self.dt = 0.01
        self.create_timer(self.dt, self.update_odometry)

    def imu_callback(self, msg:Imu):
        """ Callback to get yaw rate """
        self.yaw_rate = msg.angular_velocity.z  # Rotational velocity around Z-axis

    def jointstates_callback(self, msg:JointState):
        """ Callback to get JointState from /joint_states topic """
        for i in range(len(msg.name)):
            if msg.name[i] == "rear_left_wheel":
                index_rl = i
            elif msg.name[i] == "rear_right_wheel":
                index_rr = i
            elif msg.name[i] == "left_steering_hinge_wheel":
                index_dl = i
            elif msg.name[i] == "right_steering_hinge_wheel":
                index_dr = i

        if index_rl is not None and index_rr is not None and index_dr is not None and index_dl is not None:
            self.v_rl = msg.velocity[index_rl]*self.wheelradius
            self.v_rr = msg.velocity[index_rr]*self.wheelradius
            self.v_avr = (self.v_rl + self.v_rr) / 2
            self.delta_L = msg.position[index_dl]
            self.delta_R = msg.position[index_dr]

    def delta_steering(self, delta_L, delta_R, wheelbase, track_width, steering_ratio):
        delta_ack_L = math.atan((wheelbase * math.tan(delta_L)) / (wheelbase - 0.5 * track_width * math.tan(delta_L)))
        delta_ack_R = math.atan((wheelbase * math.tan(delta_R)) / (wheelbase + 0.5 * track_width * math.tan(delta_R)))
        delta_ack = (delta_ack_L + delta_ack_R) / 2
        delta_steer = delta_ack * steering_ratio
        return delta_steer

    def update_odometry(self):
        # Publish Odom (odom -> base_footprint)
        self.OdoYawRate()
        self.Odo1Track()
        self.Odo2Track()
        
        # Publish TF transformation (odom -> base_footprint)
        self.publish_tf(self.x_gt, self.y_gt, self.quat_gt)

    def OdoGroundTruth(self, msg:Odometry):
        self.header_gt = msg.header
        self.pose_gt = msg.pose.pose
        self.x_gt = msg.pose.pose.position.x
        self.y_gt = msg.pose.pose.position.y
        quat_x = msg.pose.pose.orientation.x
        quat_y = msg.pose.pose.orientation.y
        quat_z = msg.pose.pose.orientation.z
        quat_w = msg.pose.pose.orientation.w
        self.quat_gt = [quat_x, quat_y, quat_z, quat_w]

    def OdoYawRate(self):
        # Compute new pose        
        self.x_curr = self.x_curr + (self.v_curr * self.dt * math.cos(self.theta_curr + self.BETA + ((self.w_curr * self.dt) / 2)))
        self.y_curr = self.y_curr + (self.v_curr * self.dt * math.sin(self.theta_curr + self.BETA + ((self.w_curr * self.dt) / 2)))
        self.theta_curr = self.theta_curr + (self.w_curr * self.dt)
        self.quaternion = tf_transformations.quaternion_from_euler(0.0, 0.0, self.theta_curr)
        self.v_curr = self.v_avr
        self.w_curr = self.yaw_rate

        # Publish odometry message
        self.publish_odom("odom", "base_footprint", self.x_curr, self.y_curr, self.quaternion, self.v_curr, self.w_curr, self.yaw_rate_publisher)

    def Odo1Track(self):
        # Compute new pose  
        delta_steer = self.delta_steering(self.delta_L, self.delta_R, self.wheelbase, self.track_width, self.steering_ratio)
        self.x_curr_1Track = self.x_curr_1Track + (self.v_curr_1Track * self.dt * math.cos(self.theta_curr_1Track + self.BETA + ((self.w_curr_1Track * self.dt) / 2)))
        self.y_curr_1Track = self.y_curr_1Track + (self.v_curr_1Track * self.dt * math.sin(self.theta_curr_1Track + self.BETA + ((self.w_curr_1Track * self.dt) / 2)))
        self.theta_curr_1Track = self.theta_curr_1Track + (self.w_curr_1Track * self.dt)
        self.quaternion_1Track = tf_transformations.quaternion_from_euler(0.0, 0.0, self.theta_curr_1Track)
        self.v_curr_1Track = self.v_avr
        self.w_curr_1Track = (self.v_curr_1Track / self.wheelbase) * math.tan(delta_steer)

        # Publish odometry message
        self.publish_odom("odom", "base_footprint", self.x_curr_1Track, self.y_curr_1Track, self.quaternion_1Track, self.v_curr_1Track, self.w_curr_1Track, self.single_track_publisher)

    def Odo2Track(self):
        # Compute new pose  
        self.x_curr_2Track = self.x_curr_2Track + (self.v_curr_2Track * self.dt * math.cos(self.theta_curr_2Track + self.BETA + ((self.w_curr_2Track * self.dt) / 2)))
        self.y_curr_2Track = self.y_curr_2Track + (self.v_curr_2Track * self.dt * math.sin(self.theta_curr_2Track + self.BETA + ((self.w_curr_2Track * self.dt) / 2)))
        self.theta_curr_2Track = self.theta_curr_2Track + (self.w_curr_2Track * self.dt)
        self.quaternion_2Track = tf_transformations.quaternion_from_euler(0.0, 0.0, self.theta_curr_2Track)
        self.v_curr_2Track = self.v_avr
        self.w_curr_2Track = (self.v_rr - self.v_rl) / self.track_width

        # Publish odometry message
        self.publish_odom("odom", "base_footprint", self.x_curr_2Track, self.y_curr_2Track, self.quaternion_2Track, self.v_curr_2Track, self.w_curr_2Track, self.double_track_publisher)

    def publish_odom(self, frame_id, child_frame_id, pose_x, pose_y, quaternion_list, v_curr, w_curr, publisher):
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
        odom_msg.twist.twist.linear = Vector3(x=v_curr, y=0.0, z=0.0)
        odom_msg.twist.twist.angular = Vector3(x=0.0, y=0.0, z=w_curr)

        # Publish odometry
        publisher.publish(odom_msg)    


    def publish_tf(self, pose_x, pose_y, quaternion_list):
        """ Publishes the transformation from 'odom' to 'base_footprint' """

        if len(quaternion_list) != 4:
            return

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"

        # Position
        t.transform.translation.x = pose_x
        t.transform.translation.y = pose_y
        t.transform.translation.z = 0.0

        # Orientation
        t.transform.rotation.x = quaternion_list[0]
        t.transform.rotation.y = quaternion_list[1]
        t.transform.rotation.z = quaternion_list[2]
        t.transform.rotation.w = quaternion_list[3]

        # Publish TF transform
        # self.tf_broadcaster.sendTransform(t)
        self.tf_static_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryCalcurationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
