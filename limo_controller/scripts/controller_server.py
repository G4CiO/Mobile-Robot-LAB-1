#!/usr/bin/python3

from limo_controller.dummy_module import dummy_function, dummy_var
import rclpy
import os
import yaml
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from ament_index_python import get_package_share_directory

class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.int_term = 0
        self.derivative_term = 0
        self.last_error = None
    
    def get_control(self, error, dt):
        self.int_term += error*self.Ki*dt
        if self.last_error is not None:
            self.derivative_term = (error-self.last_error)/dt*self.Kd
        self.last_error = error
        return self.Kp * error + self.int_term + self.derivative_term

class ControllerServer(Node):
    def __init__(self):
        super().__init__('controller_server_node')
        self.get_logger().info('controller_server_node has been start')

        # Declare parameters
        self.declare_parameter('file', 'path.yaml')

        pkg_name = 'limo_controller'
        path_pkg_share_path = get_package_share_directory(pkg_name)
        ws_path, _ = path_pkg_share_path.split('install')
        file = self.get_parameter('file').value
        self.path_path = os.path.join(ws_path, 'src/Mobile-Robot-LAB-1', pkg_name, 'config', file)

        # PID controllers for linear and angular velocities
        self.linear_pid = PIDController(Kp=2.0, Ki=0.0, Kd=0.0)
        self.angular_pid = PIDController(Kp=2.0, Ki=0.0, Kd=0.0)
        self.thresold_distance_error = 0.30

        # Pure Puresuit controllers
        self.lookahead_distance = 0.8
        self.declare_parameter('wheelbase', 0.2)   # meters

        # Load path from YAML file
        self.path = self.load_path()
        self.current_target_idx = 0
        
        # Subscribers and Publishers
        self.odom_sub = self.create_subscription(Odometry, '/odometry/ground_truth', self.odom_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Timer for control loop
        self.dt = 0.01
        self.timer = self.create_timer(self.dt, self.timer_callback)  # 100 Hz
        self.last_time = self.get_clock().now()
        
    def load_path(self):
        with open(self.path_path, 'r') as file:
            return yaml.safe_load(file)
    
    def odom_callback(self, msg:Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_yaw = self.get_yaw_from_quaternion(msg.pose.pose.orientation)
    
    def get_yaw_from_quaternion(self, quat):
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def timer_callback(self):
        # self.pid_control()
        self.pure_pursuit_control()

    def pid_control(self):
        if self.current_target_idx >= len(self.path):
            self.get_logger().info("Path tracking completed!")
            return
        
        target = self.path[self.current_target_idx]
        target_x, target_y, target_yaw = target['x'], target['y'], target['yaw']
        
        # Compute errors
        dx = target_x - self.robot_x
        dy = target_y - self.robot_y
        distance_error = math.hypot(dx, dy)
        yaw_error = target_yaw - self.robot_yaw

        # Get control inputs from PID controllers
        linear_speed = self.linear_pid.get_control(distance_error, self.dt)
        angular_speed = self.angular_pid.get_control(yaw_error, self.dt)
        
        # Publish velocity command
        cmd = Twist()
        cmd.linear.x = linear_speed
        cmd.angular.z = angular_speed
        self.cmd_vel_pub.publish(cmd)
        
        # Check if the target is reached
        if distance_error < self.thresold_distance_error:
            self.current_target_idx += 1

    def pure_pursuit_control(self):
        wheelbase = self.get_parameter('wheelbase').value

        if self.current_target_idx >= len(self.path):
            self.current_target_idx = 0  # Reset index to loop the path
            # self.get_logger().info("Path tracking completed!")
            # return
        # Implement Here
        target = self.path[self.current_target_idx]
        target_x, target_y, target_yaw = target['x'], target['y'], target['yaw']

        dx = target_x - self.robot_x
        dy = target_y - self.robot_y
        distance_error = math.hypot(dx, dy)

        # If distance < lookahead_distance, it moves to the next waypoint.
        if distance_error < self.lookahead_distance:
            self.current_target_idx += 1

        # Heading Angle Calculation
        alpha = target_yaw - self.robot_yaw
        # target_angle = math.atan2(dy, dx)
        # alpha = target_angle - self.robot_yaw

        # Steering Angle Calculation (β)
        beta = math.atan(2 * wheelbase * math.sin(alpha) / self.lookahead_distance)

        linear_velocity = self.linear_pid.get_control(distance_error, self.dt)
        # Angular Velocity Calculation (ω)
        angular_velocity = (linear_velocity * math.tan(beta)) / wheelbase

        # Debug
        # print('////////////////////////////////////////////////////////////////////////')
        # print(f'target_x = {target_x}: {self.robot_x}')
        # print(f'target_y = {target_y}: {self.robot_y}')
        # print(f'distance: {distance_error} < lookahead_distance: {self.lookahead_distance}',f'index: {self.current_target_idx}')
        # print(f'Steering Angle: {beta}')
        # print(f'target_yaw = {target_yaw}: {self.robot_yaw}')
        # print(f'yaw_error = {alpha}')

        # Publish cmd_vel
        msg = Twist()
        msg.linear.x = linear_velocity
        msg.angular.z = angular_velocity
        self.cmd_vel_pub.publish(msg)




def main(args=None):
    rclpy.init(args=args)
    node = ControllerServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()