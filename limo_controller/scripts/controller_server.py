#!/usr/bin/python3

from limo_controller.dummy_module import dummy_function, dummy_var
import rclpy
import os
import yaml
import math
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from ament_index_python import get_package_share_directory
from tf_transformations import euler_from_quaternion

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
        self.declare_parameter('wheelbase', 0.2) 
        
        # Declare the odometry source parameter, default to "ground_truth" 
        self.declare_parameter('odom_source', 'ground_truth')
        
        # Get the parameter value
        odom_source = self.get_parameter('odom_source').value

        # Create a subscription based on the chosen odometry source
        if odom_source == 'ground_truth':
            self.odom_sub = self.create_subscription(Odometry, '/odometry/ground_truth', self.odom_callback, 10)
        elif odom_source == 'ekf_odom':
            self.odom_sub = self.create_subscription(Odometry, '/ekf_odom', self.odom_callback, 10)
        else:
            self.get_logger().warn(f"Unknown odom_source '{odom_source}', defaulting to /odometry/ground_truth")
            self.odom_sub = self.create_subscription(Odometry, '/odometry/ground_truth', self.odom_callback, 10)

        # Initialization
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.state = 0

        pkg_name = 'limo_controller'
        path_pkg_share_path = get_package_share_directory(pkg_name)
        ws_path, _ = path_pkg_share_path.split('install')
        file = self.get_parameter('file').value
        self.path_path = os.path.join(ws_path, 'src/Mobile-Robot-LAB-1', pkg_name, 'config', file)

        # PID controllers for linear and angular velocities
        self.linear_pid = PIDController(Kp=2.0, Ki=0.0, Kd=0.0)
        self.angular_pid = PIDController(Kp=10.0, Ki=0.0, Kd=0.0)
        self.thresold_distance_error = 0.5

        # Pure Puresuit controllers
        self.linear_velo_pure = 3.0
        self.K_dd = 1.0
        self.min_ld = 0.5
        self.max_ld = 4.5
        self.lookahead_distance = 0.5

        # Stanley controllers
        self.linear_velo_stan = 3.0
        self.k = 1.0
        self.ks = 2.5 # Softening constant (ks) = If increase ks, It will decrease swing in steering wheel when at low speed

        # Load path from YAML file
        self.path = self.load_path()
        self.current_target_idx = 0
         
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

    def pub_cmd(self, vx, wz):
        msg = Twist()
        msg.linear.x = vx
        msg.angular.z = wz
        self.cmd_vel_pub.publish(msg)

    def normalize_angle(self, angle):
        # Normalize an angle to [-pi, pi]
        normalize_angle = math.atan2(math.sin(angle), math.cos(angle))
        return normalize_angle

    def serch_nearest_point_index(self):
        # Search nearest point index
        if self.state == 0:
            dx = [self.robot_x - target_x['x'] for target_x in self.path]
            dy = [self.robot_y - target_y['y'] for target_y in self.path]
            d = np.hypot(dx, dy)
            self.current_target_idx = np.argmin(d)
            self.state = 1

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
            # self.current_target_idx = 0  # Reset index to loop the path
            self.pub_cmd(0.0, 0.0)
            return # Stop

        # Search nearest point index
        self.serch_nearest_point_index()

        target = self.path[self.current_target_idx]
        target_x, target_y = target['x'], target['y']
        
        # Compute errors
        dx = target_x - self.robot_x
        dy = target_y - self.robot_y
        distance_error = math.hypot(dx, dy)
        
        target_yaw = math.atan2(dy,dx)
        yaw_error = target_yaw - self.robot_yaw
        # Normalize an angle to [-pi, pi]
        yaw_error = self.normalize_angle(yaw_error)

        # Check if the target is reached
        if distance_error < self.thresold_distance_error:
            self.current_target_idx += 1

        # Get control inputs from PID controllers
        linear_velocity = self.linear_pid.get_control(distance_error, self.dt)
        angular_velocity = self.angular_pid.get_control(yaw_error, self.dt)

        beta = math.atan(angular_velocity * wheelbase / linear_velocity)
        beta = max(-0.6, min(beta, 0.6))
        angular_velocity = (linear_velocity * math.tan(beta)) / wheelbase

        # Publish velocity command
        self.pub_cmd(linear_velocity, angular_velocity)

    def pure_pursuit_control(self):
        wheelbase = self.get_parameter('wheelbase').value
        if self.current_target_idx >= len(self.path):
            # self.current_target_idx = 0  # Reset index to loop the path
            self.pub_cmd(0.0, 0.0)
            return # Stop
        
        # Search nearest point index
        self.serch_nearest_point_index()

        # Implement Here
        target = self.path[self.current_target_idx]
        target_x, target_y = target['x'], target['y']

        dx = target_x - self.robot_x
        dy = target_y - self.robot_y
        distance_error = math.hypot(dx, dy)

        # If distance < lookahead_distance, it moves to the next waypoint.
        if distance_error < self.lookahead_distance:
            self.current_target_idx += 1

        # Heading Angle Calculation
        target_yaw = math.atan2(dy,dx)
        alpha = target_yaw - self.robot_yaw
        # Normalize an angle to [-pi, pi]
        alpha = self.normalize_angle(alpha)

        # Steering Angle Calculation (β)
        self.lookahead_distance = np.clip(self.K_dd * self.linear_speed_pure, self.min_ld, self.max_ld)
        beta = math.atan2(2 * wheelbase * math.sin(alpha) / self.lookahead_distance, 1.0)
        beta = max(-0.6, min(beta, 0.6))

        # Angular Velocity Calculation (ω)
        angular_velocity = (self.linear_velo_pure * math.tan(beta)) / wheelbase

        # Publish cmd_vel
        self.pub_cmd(self.linear_velo_pure, angular_velocity)

    def stanley_control(self):
        wheelbase = self.get_parameter('wheelbase').value
        if self.current_target_idx >= len(self.path) - 1:
            # self.current_target_idx = 0  # Reset index to loop the path
            self.pub_cmd(0.0, 0.0)
            return # Stop

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
        theta_e = target['yaw'] - self.robot_yaw
        # Normalize an angle to [-pi, pi]
        theta_e = self.normalize_angle(theta_e)

        # Stanley control formula
        if self.v != 0.0:
            delta = theta_e + np.arctan2(self.k * e_fa, self.ks + self.v)
            delta = max(-0.6, min(delta, 0.6))
        else:
            delta = 0.0

        # Angular Velocity Calculation (ω)
        angular_velocity = (self.linear_velo_stan * math.tan(delta)) / wheelbase

        # Publish cmd_vel
        self.pub_cmd(self.linear_velo_stan, angular_velocity)

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