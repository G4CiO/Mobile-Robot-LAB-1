#!/usr/bin/python3

from limo_controller.dummy_module import dummy_function, dummy_var
import rclpy
import os
import yaml
import math
import numpy as np
import cvxpy as cp
import signal
import sys
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from ament_index_python import get_package_share_directory
from tf_transformations import euler_from_quaternion
from angle import angle_mod


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
        self.declare_parameter('control_mode', 'pure_pursuit')  # Default mode: 'pure_pursuit'
        self.declare_parameter('path_file', 'path.yaml')
        self.declare_parameter('prediction_horizon', 10)
        self.declare_parameter('wheelbase', 0.2) 

        # Initialization
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.state = np.zeros(3)  # [x, y, yaw]

        pkg_name = 'limo_controller'
        path_pkg_share_path = get_package_share_directory(pkg_name)
        ws_path, _ = path_pkg_share_path.split('install')
        file = self.get_parameter('file').value
        self.path_path = os.path.join(ws_path, 'src/Mobile-Robot-LAB-1', pkg_name, 'config', file)

        # PID controllers for linear and angular velocities
        self.linear_pid = PIDController(Kp=0.5, Ki=0.0, Kd=0.0)
        self.angular_pid = PIDController(Kp=2.0, Ki=0.0, Kd=0.0)
        self.thresold_distance_error = 1.0

        # Pure Puresuit controllers
        self.state = 0
        self.linear_speed_pure = PIDController(Kp=2.0, Ki=0.0, Kd=0.0)
        self.lookahead_distance = 0.8

        # Stanley controllers
        self.k = 1.0
        self.ks = 2.5
        self.target_speed = 1.0
        self.linear_speed_stan = PIDController(Kp=70, Ki=0.0, Kd=0.0)

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
        orientation = msg.pose.pose.orientation
        self.robot_yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])[2]        
        self.v = msg.twist.twist.linear.x
    
    def get_yaw_from_quaternion(self, quat):
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def timer_callback(self):
        control_mode = self.get_parameter('control_mode').value
        if control_mode == 'pid':
            self.pid_control()
        elif control_mode == 'pure_pursuit':
            self.pure_pursuit_control()
        elif control_mode == 'stanley':
            self.stanley_control()
        else:
            self.get_logger().warn(f"Unknown mode '{control_mode}', defaulting to Pure Pursuit control")
            self.pure_pursuit_control()

    def pid_control(self):
        wheelbase = self.get_parameter('wheelbase').value
        if self.current_target_idx >= len(self.path):
            self.current_target_idx = 0  # Reset index to loop the path

        # Search nearest point index
        # if self.state == 0:
        #     dx = [self.robot_x - target_x['x'] for target_x in self.path]
        #     dy = [self.robot_y - target_y['y'] for target_y in self.path]
        #     d = np.hypot(dx, dy)
        #     self.current_target_idx = np.argmin(d) + 50
        #     self.state = 1
        
        target = self.path[self.current_target_idx]
        target_x, target_y, target_yaw = target['x'], target['y'], target['yaw']
        
        # Compute errors
        dx = target_x - self.robot_x
        dy = target_y - self.robot_y
        distance_error = math.hypot(dx, dy)
        yaw_error = target_yaw - self.robot_yaw

        # Check if the target is reached
        if distance_error < self.thresold_distance_error:
            self.current_target_idx += 1

        # Get control inputs from PID controllers
        linear_velocity = self.linear_pid.get_control(distance_error, self.dt)
        angular_velocity = self.angular_pid.get_control(yaw_error, self.dt)

        beta = math.atan(angular_velocity * wheelbase / linear_velocity)
        beta = max(-0.523598767, min(beta, 0.523598767))
        angular_velocity = (linear_velocity * math.tan(beta)) / wheelbase
        # Publish velocity command
        cmd = Twist()
        cmd.linear.x = linear_velocity
        cmd.angular.z = angular_velocity
        self.cmd_vel_pub.publish(cmd)

        # Debug
        # print('////////////////////////////////////////////////////////////////////////')
        # print(f'target_x = {target_x}: {self.robot_x}')
        # print(f'target_y = {target_y}: {self.robot_y}')
        # print(f'distance: {distance_error} < thresold_distance_error: {self.thresold_distance_error}',f'index: {self.current_target_idx}')

    def pure_pursuit_control(self):
        wheelbase = self.get_parameter('wheelbase').value

        if self.current_target_idx >= len(self.path):
            self.current_target_idx = 0  # Reset index to loop the path

        # Search nearest point index
        if self.state == 0:
            dx = [self.robot_x - target_x['x'] for target_x in self.path]
            dy = [self.robot_y - target_y['y'] for target_y in self.path]
            d = np.hypot(dx, dy)
            self.current_target_idx = np.argmin(d)
            self.state = 1

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

        # Steering Angle Calculation (β)
        beta = math.atan2(2 * wheelbase * math.sin(alpha) / self.lookahead_distance, 1.0)
        beta = max(-0.523598767, min(beta, 0.523598767))

        linear_velocity = self.linear_speed_pure.get_control(distance_error, self.dt)
        # Angular Velocity Calculation (ω)
        angular_velocity = (linear_velocity * math.tan(beta)) / wheelbase

        # Publish cmd_vel
        msg = Twist()
        msg.linear.x = linear_velocity
        msg.angular.z = angular_velocity
        self.cmd_vel_pub.publish(msg)

    def stanley_control(self):
        wheelbase = self.get_parameter('wheelbase').value
        # Calc front axle position
        fx = self.robot_x + (wheelbase/2 * np.cos(self.robot_yaw))
        fy = self.robot_y + (wheelbase/2 * np.sin(self.robot_yaw))

        # Search nearest point index
        dx = [fx - target_x['x'] for target_x in self.path]
        dy = [fy - target_y['y'] for target_y in self.path]
        d = np.hypot(dx, dy)
        self.current_target_idx = np.argmin(d)
        target = self.path[self.current_target_idx]

        # Project RMS error onto front axle vector
        front_axle_vec = [-np.cos(self.robot_yaw + np.pi / 2),-np.sin(self.robot_yaw + np.pi / 2)]
        e_fa = np.dot([dx[self.current_target_idx], dy[self.current_target_idx]], front_axle_vec)
        
        # Compute heading error
        # theta_e = self.normalize_angle(target['yaw'] - self.robot_yaw)
        # theta_e = self.normalize(target['yaw'] - self.robot_yaw)
        theta_e = target['yaw'] - self.robot_yaw
        # Normalize an angle to [-pi, pi]
        theta_e = np.arctan2(np.sin(theta_e),np.cos(theta_e))

        # Stanley control formula
        if self.v != 0.0:
            if abs(theta_e) > 0.0:
                delta = theta_e + np.arctan2(self.k * e_fa, self.ks + self.v)
                delta = max(-0.523598767, min(delta, 0.523598767))
            else:
                delta = 0.0
        else:
            delta = 0.0

        linear_velocity = self.linear_speed_stan.get_control(self.target_speed - self.v, self.dt) * self.dt
        # Angular Velocity Calculation (ω)
        angular_velocity = (linear_velocity * math.tan(delta)) / wheelbase


        # Debug
        # tx = target['x']
        # ty = target['y']
        # tya = target['yaw']
        # print('////////////////////////////////////////////////////////////////////////')
        # print(f'theta_e = {theta_e}')
        # print(f'Steering Angle: {delta}')
        # print(f'target_x = {tx}: {self.robot_x}')
        # print(f'target_y = {ty}: {self.robot_y}')
        # print(f'distance: {e_fa} ',f'index: {target_idx}')
        # print(f'target_yaw = {tya}: {self.robot_yaw}')
        # print(f'yaw_error = {theta_e}')
        # print(f'linear_velocity = {linear_velocity}, angular_velocity = {angular_velocity}')

        # Publish cmd_vel
        msg = Twist()
        msg.linear.x = linear_velocity
        msg.angular.z = angular_velocity
        self.cmd_vel_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ControllerServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Publish zero cmd_vel before shutting down
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        node.cmd_vel_pub.publish(cmd)
        node.get_logger().info("Published zero cmd_vel before shutdown.")
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()