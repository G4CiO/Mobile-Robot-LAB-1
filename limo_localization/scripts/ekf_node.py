#!/usr/bin/env python3

import yaml
import os
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry      # เปลี่ยนจากการใช้ PoseStamped เป็น Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped  # เก็บไว้สำหรับการแปลงข้อมูล (ถ้าต้องการ)
from tf_transformations import quaternion_from_euler, euler_from_quaternion

import numpy as np
import math

def find_workspace_by_name(start_path, workspace_name="MOBILE_ROBOT_WS"):
    current_dir = os.path.abspath(start_path)
    while True:
        if os.path.basename(current_dir) == workspace_name:
            return current_dir
        parent_dir = os.path.dirname(current_dir)
        if parent_dir == current_dir:
            return None
        current_dir = parent_dir

def load_covariances():
    """
    Loads the covariance matrices from the YAML file.
    Returns a dictionary of covariance matrices.
    """
    script_dir = os.path.dirname(os.path.abspath(__file__))
    workspace_dir = find_workspace_by_name(script_dir, "MOBILE_ROBOT_WS")
    if workspace_dir is None:
        print("Could not find workspace directory 'MOBILE_ROBOT_WS'. Using script directory as fallback.")
        workspace_dir = script_dir
    # Build the data directory path relative to the workspace.
    data_dir = os.path.join(workspace_dir, "src", "Mobile-Robot-LAB-1", "limo_localization", "config")
    data_dir = os.path.abspath(data_dir)
    yaml_file = os.path.join(data_dir, "covariances.yaml")
    
    if not os.path.exists(yaml_file):
        print(f"Error: YAML file not found at {yaml_file}")
        return None

    with open(yaml_file, "r") as f:
        data = yaml.safe_load(f)
    return data.get("covariances", None)

# Load covariance matrices from YAML using the workspace finder
covs = load_covariances()
if covs is not None:
    # For the EKF odometry update, we use different covariances for each sensor.
    R_yaw_rate    = np.array(covs.get("yaw_rate"))
    R_single_track = np.array(covs.get("single_track"))
    R_double_track = np.array(covs.get("double_track"))
    print("Loaded odometry covariances from YAML.")
else:
    # Fallback default: use same default for all three (adjust as needed)
    R_yaw_rate = np.diag([
        0.01, 0.01, 0.01,  # Position noise
        np.deg2rad(1.0), np.deg2rad(1.0), np.deg2rad(1.0),  # Orientation noise (radians)
        0.01, 0.01, 0.01,  # Linear velocity noise
        np.deg2rad(1.0), np.deg2rad(1.0), np.deg2rad(1.0)   # Angular velocity noise (rad/s)
    ]) ** 2
    # For simplicity, we use the same default for single and double track.
    R_single_track = R_yaw_rate.copy()
    R_double_track = R_yaw_rate.copy()
    print("!YAML not loaded; using default odometry covariances.")

# Process noise covariance Q (15x15)
Q = np.diag([
    0.0001, 0.0001, 0.0001,  # position noise
    0.0001, 0.0001, 0.0001,  # orientation noise (rad)
    0.0001, 0.0001, 0.0001,  # linear velocity noise
    0.0001, 0.0001, 0.0001,  # angular velocity noise (rad/s)
    0.0001, 0.0001, 0.0001   # linear acceleration noise
]) ** 2

# Measurement noise covariance for GPS (3x3): [p (3)]
R_GPS = np.diag([1.0, 1.0, 1.0]) ** 2
# Measurement noise covariance for IMU (9x9): [orientation (3), angular velocity (3), linear acceleration (3)]
R_imu = np.diag([
    np.deg2rad(1.0), np.deg2rad(1.0), np.deg2rad(1.0),
    np.deg2rad(0.5), np.deg2rad(0.5), np.deg2rad(0.5),
    0.2, 0.2, 0.2
]) ** 2

# Helper functions: Rotation Matrix, Jacobian, and their derivatives

def R_from_euler(roll, pitch, yaw):
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    R = np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp,   cp*sr,            cp*cr]
    ])
    return R

def J_from_euler(roll, pitch, yaw):
    roll = float(roll)
    pitch = float(pitch)
    cos_pitch = math.cos(pitch)
    if abs(cos_pitch) < 1e-4:
        cos_pitch = 1e-4
    tan_pitch = math.tan(pitch)
    J = np.array([
        [1, math.sin(roll)*tan_pitch, math.cos(roll)*tan_pitch],
        [0, math.cos(roll),          -math.sin(roll)],
        [0, math.sin(roll)/cos_pitch,  math.cos(roll)/cos_pitch]
    ])
    return J

def dR_droll(roll, pitch, yaw):
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    Rz = np.array([[cy, -sy, 0],
                   [sy,  cy, 0],
                   [0,   0,  1]])
    Ry = np.array([[cp, 0, sp],
                   [0,  1, 0],
                   [-sp,0, cp]])
    dRx = np.array([[0, 0, 0],
                    [0, -sr, -cr],
                    [0, cr, -sr]])
    return Rz @ Ry @ dRx

def dR_dpitch(roll, pitch, yaw):
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    Rz = np.array([[cy, -sy, 0],
                   [sy,  cy, 0],
                   [0,   0,  1]])
    dRy = np.array([[-sp, 0, cp],
                    [0, 0, 0],
                    [-cp,0, -sp]])
    Rx = np.array([[1, 0, 0],
                   [0, cr, -sr],
                   [0, sr, cr]])
    return Rz @ dRy @ Rx

def dR_dyaw(roll, pitch, yaw):
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    dRz = np.array([[-sy, -cy, 0],
                    [cy, -sy, 0],
                    [0, 0, 0]])
    Ry = np.array([[cp, 0, sp],
                   [0, 1, 0],
                   [-sp,0, cp]])
    Rx = np.array([[1, 0, 0],
                   [0, cr, -sr],
                   [0, sr, cr]])
    return dRz @ Ry @ Rx

def dJ_droll(roll, pitch, yaw):
    cos_roll = math.cos(roll)
    sin_roll = math.sin(roll)
    tan_pitch = math.tan(pitch)
    dJ = np.zeros((3,3))
    dJ[0,1] = cos_roll * tan_pitch
    dJ[0,2] = -sin_roll * tan_pitch
    dJ[1,1] = -sin_roll
    dJ[1,2] = -cos_roll
    dJ[2,1] = cos_roll / math.cos(pitch)
    dJ[2,2] = -sin_roll / math.cos(pitch)
    return dJ

def dJ_dpitch(roll, pitch, yaw):
    sin_roll = math.sin(roll)
    cos_roll = math.cos(roll)
    cos_pitch = math.cos(pitch)
    sec_pitch2 = 1.0/(cos_pitch**2)
    dJ = np.zeros((3,3))
    dJ[0,1] = sin_roll * sec_pitch2
    dJ[0,2] = cos_roll * sec_pitch2
    dJ[2,1] = sin_roll * math.sin(pitch) / (cos_pitch**2)
    dJ[2,2] = cos_roll * math.sin(pitch) / (cos_pitch**2)
    return dJ

def dJ_dyaw(roll, pitch, yaw):
    return np.zeros((3,3))

def dynamic_model(x, dt, R_from_euler, J_from_euler, u_alpha):
    x_new = np.zeros((15,1))
    roll = x[3,0]
    pitch = x[4,0]
    yaw = x[5,0]
    R_mat = R_from_euler(roll, pitch, yaw)
    b = x[6:9] * dt + 0.5 * x[12:15] * (dt**2)
    x_new[0:3] = x[0:3] + R_mat @ b
    J_mat = J_from_euler(roll, pitch, yaw)
    x_new[3:6] = x[3:6] + J_mat @ x[9:12] * dt
    x_new[6:9] = x[6:9] + x[12:15] * dt
    x_new[9:12] = x[9:12] + u_alpha * dt
    x_new[12:15] = x[12:15]
    return x_new

def jacobian_F(x, dt, R_from_euler, J_from_euler,
               dR_droll, dR_dpitch, dR_dyaw,
               dJ_droll, dJ_dpitch, dJ_dyaw):
    F = np.eye(15)
    I3 = np.eye(3)
    roll = x[3,0]
    pitch = x[4,0]
    yaw = x[5,0]
    R_mat = R_from_euler(roll, pitch, yaw)
    dR_dr = np.zeros((3,3,3))
    dR_dr[:,:,0] = dR_droll(roll, pitch, yaw)
    dR_dr[:,:,1] = dR_dpitch(roll, pitch, yaw)
    dR_dr[:,:,2] = dR_dyaw(roll, pitch, yaw)
    b = x[6:9] * dt + 0.5 * x[12:15] * (dt**2)
    L = np.zeros((3,3))
    for i in range(3):
        L[:, i] = (dR_dr[:,:,i] @ b).flatten()
    F[0:3, 3:6] = L
    F[0:3, 6:9] = R_mat * dt
    F[0:3, 12:15] = R_mat * (0.5 * dt**2)
    J_mat = J_from_euler(roll, pitch, yaw)
    dJ_droll_mat = dJ_droll(roll, pitch, yaw)
    dJ_dpitch_mat = dJ_dpitch(roll, pitch, yaw)
    dJ_dyaw_mat = dJ_dyaw(roll, pitch, yaw)
    omega = x[9:12]
    M = dJ_droll_mat * omega[0,0] + dJ_dpitch_mat * omega[1,0] + dJ_dyaw_mat * omega[2,0]
    F[3:6, 3:6] = np.eye(3) + M * dt
    F[3:6, 9:12] = J_mat * dt
    F[6:9, 6:9] = I3
    F[6:9, 12:15] = I3 * dt
    F[9:12, 9:12] = I3
    F[12:15, 12:15] = I3
    return F

def ekf_predict(xEst, PEst, dt, Q, R_from_euler, J_from_euler,
                dR_droll, dR_dpitch, dR_dyaw, dJ_droll, dJ_dpitch, dJ_dyaw, u_alpha):
    F = jacobian_F(xEst, dt, R_from_euler, J_from_euler,
                   dR_droll, dR_dpitch, dR_dyaw, dJ_droll, dJ_dpitch, dJ_dyaw)
    xPred = dynamic_model(xEst, dt, R_from_euler, J_from_euler, u_alpha)
    PPred = F @ PEst @ F.T + Q
    return xPred, PPred

def ekf_update_odom(xEst, PEst, z, R_odom):
    H = np.zeros((12, 15))
    H[0:3, 0:3] = np.eye(3)
    H[3:6, 3:6] = np.eye(3)
    H[6:9, 6:9] = np.eye(3)
    H[9:12, 9:12] = np.eye(3)

    zPred = H @ xEst
    y = z - zPred

    # Normalize the orientation error
    for i in range(3, 6):
        y[i, 0] = normalize_angle(y[i, 0])
        
    S = H @ PEst @ H.T + R_odom
    K = PEst @ H.T @ np.linalg.inv(S)
    xEst_new = xEst + K @ y
    PEst_new = (np.eye(15) - K @ H) @ PEst
    return xEst_new, PEst_new

def normalize_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

def ekf_update_imu(xEst, PEst, z, R_imu):
    H = np.zeros((9, 15))
    H[0:3, 3:6] = np.eye(3)
    H[3:6, 9:12] = np.eye(3)
    H[6:9, 12:15] = np.eye(3)
    zPred = H @ xEst
    y = z - zPred
    
    for i in range(3):
        y[i, 0] = normalize_angle(y[i, 0])
    
    S = H @ PEst @ H.T + R_imu
    K = PEst @ H.T @ np.linalg.inv(S)
    xEst_new = xEst + K @ y
    PEst_new = (np.eye(15) - K @ H) @ PEst
    
    for i in range(3):
        xEst_new[3 + i, 0] = normalize_angle(xEst_new[3 + i, 0])
    
    return xEst_new, PEst_new

def ekf_update_gps(xEst, PEst, z, R_GPS):
    # fuse only (x,y,z) position
    H = np.zeros((3, 15))
    H[0:3, 0:3] = np.eye(3)
    zPred = H @ xEst
    y = z - zPred
    S = H @ PEst @ H.T + R_GPS
    K = PEst @ H.T @ np.linalg.inv(S)
    xEst_new = xEst + K @ y
    PEst_new = (np.eye(15) - K @ H) @ PEst
    return xEst_new, PEst_new

class EKFFullNode(Node):
    def __init__(self):
        super().__init__('ekf_full_node')
        self.dt = 0.1
        self.last_time = self.get_clock().now()
        self.xEst = np.zeros((15, 1))
        self.PEst = np.eye(15)
        self.u_alpha = np.zeros((3,1))
        
        # Measurement containers and flags for three odometry topics
        self.z_yaw_rate = None
        self.new_yaw_rate = False
        self.z_single_track = None
        self.new_single_track = False
        self.z_double_track = None
        self.new_double_track = False
        
        self.z_imu = None
        self.new_imu = False
        self.z_gps = None
        self.new_gps = False
        
        # Subscribers for three odometry topics
        self.yaw_rate_sub = self.create_subscription(
            Odometry, '/odometry/yaw_rate', self.yaw_rate_callback, 10)
        self.single_track_sub = self.create_subscription(
            Odometry, '/odometry/single_track', self.single_track_callback, 10)
        self.double_track_sub = self.create_subscription(
            Odometry, '/odometry/double_track', self.double_track_callback, 10)
            
        self.imu_sub = self.create_subscription(
            Imu, '/imu', self.imu_callback, 10)
        self.gps_sub = self.create_subscription(
            Odometry, '/gps/odom', self.gps_callback, 10)

        # Publish using Odometry message
        self.ekf_pub = self.create_publisher(Odometry, '/ekf_odom', 10)
        self.timer = self.create_timer(self.dt, self.timer_callback)
        
    def yaw_rate_callback(self, msg: Odometry):
        # Extract odometry similar to your original odom_callback
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        pz = msg.pose.pose.position.z

        q = msg.pose.pose.orientation
        quat = [q.x, q.y, q.z, q.w]
        (roll, pitch, yaw) = euler_from_quaternion(quat)

        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vz = msg.twist.twist.linear.z

        wx = msg.twist.twist.angular.x
        wy = msg.twist.twist.angular.y
        wz = msg.twist.twist.angular.z

        self.z_yaw_rate = np.array([[px], [py], [pz],
                                    [roll], [pitch], [yaw],
                                    [vx], [vy], [vz],
                                    [wx], [wy], [wz]])
        self.new_yaw_rate = True

    def single_track_callback(self, msg: Odometry):
        # Process single track odometry similar to yaw_rate
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        pz = msg.pose.pose.position.z

        q = msg.pose.pose.orientation
        quat = [q.x, q.y, q.z, q.w]
        (roll, pitch, yaw) = euler_from_quaternion(quat)

        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vz = msg.twist.twist.linear.z

        wx = msg.twist.twist.angular.x
        wy = msg.twist.twist.angular.y
        wz = msg.twist.twist.angular.z

        self.z_single_track = np.array([[px], [py], [pz],
                                        [roll], [pitch], [yaw],
                                        [vx], [vy], [vz],
                                        [wx], [wy], [wz]])
        self.new_single_track = True

    def double_track_callback(self, msg: Odometry):
        # Process double track odometry similar to yaw_rate
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        pz = msg.pose.pose.position.z

        q = msg.pose.pose.orientation
        quat = [q.x, q.y, q.z, q.w]
        (roll, pitch, yaw) = euler_from_quaternion(quat)

        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vz = msg.twist.twist.linear.z

        wx = msg.twist.twist.angular.x
        wy = msg.twist.twist.angular.y
        wz = msg.twist.twist.angular.z

        self.z_double_track = np.array([[px], [py], [pz],
                                        [roll], [pitch], [yaw],
                                        [vx], [vy], [vz],
                                        [wx], [wy], [wz]])
        self.new_double_track = True
        
    def imu_callback(self, msg: Imu):
        q = msg.orientation
        quat = [q.x, q.y, q.z, q.w]
        (roll, pitch, yaw) = euler_from_quaternion(quat)
        omega_x = msg.angular_velocity.x
        omega_y = msg.angular_velocity.y
        omega_z = msg.angular_velocity.z
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        self.z_imu = np.array([
            [roll], [pitch], [yaw],
            [omega_x], [omega_y], [omega_z],
            [ax], [ay], [az]
        ])
        self.new_imu = True
        
    def gps_callback(self, msg: Odometry):
        # update only position
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        pz = msg.pose.pose.position.z
        self.z_gps = np.array([[px], [py], [pz]])
        self.new_gps = True
        
    def timer_callback(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            dt = self.dt
        self.last_time = current_time
        
        self.xEst, self.PEst = ekf_predict(
            self.xEst, self.PEst, dt, Q, R_from_euler, J_from_euler,
            dR_droll, dR_dpitch, dR_dyaw, dJ_droll, dJ_dpitch, dJ_dyaw, self.u_alpha)
        
        # Update with each odometry measurement if available.
        if self.new_yaw_rate and self.z_yaw_rate is not None:
            self.xEst, self.PEst = ekf_update_odom(self.xEst, self.PEst, self.z_yaw_rate, R_yaw_rate)
            self.new_yaw_rate = False
        if self.new_single_track and self.z_single_track is not None:
            self.xEst, self.PEst = ekf_update_odom(self.xEst, self.PEst, self.z_single_track, R_single_track)
            self.new_single_track = False
        if self.new_double_track and self.z_double_track is not None:
            self.xEst, self.PEst = ekf_update_odom(self.xEst, self.PEst, self.z_double_track, R_double_track)
            self.new_double_track = False
        
        # Optionally update with IMU and GPS data
        # if self.new_imu and self.z_imu is not None:
        #     self.xEst, self.PEst = ekf_update_imu(self.xEst, self.PEst, self.z_imu, R_imu)
        #     self.new_imu = False
        if self.new_gps and self.z_gps is not None:
            self.xEst, self.PEst = ekf_update_gps(self.xEst, self.PEst, self.z_gps, R_GPS)
            self.new_gps = False
        
        self.publish_estimate()
        
    def publish_estimate(self):
        # Publish updated estimate as Odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"  # Adjust as needed

        # Set position from state vector
        odom_msg.pose.pose.position.x = self.xEst[0, 0]
        odom_msg.pose.pose.position.y = self.xEst[1, 0]
        odom_msg.pose.pose.position.z = self.xEst[2, 0]

        # Convert Euler angles (roll, pitch, yaw) to quaternion
        roll = self.xEst[3, 0]
        pitch = self.xEst[4, 0]
        yaw = self.xEst[5, 0]
        q = quaternion_from_euler(roll, pitch, yaw)
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]

        # Set linear and angular velocities from state vector
        odom_msg.twist.twist.linear.x = self.xEst[6, 0]
        odom_msg.twist.twist.linear.y = self.xEst[7, 0]
        odom_msg.twist.twist.linear.z = self.xEst[8, 0]
        odom_msg.twist.twist.angular.x = self.xEst[9, 0]
        odom_msg.twist.twist.angular.y = self.xEst[10, 0]
        odom_msg.twist.twist.angular.z = self.xEst[11, 0]

        # Update covariance for x and y (flattened 6x6 covariance matrix)
        cov = np.zeros((6,6))
        cov[0,0] = self.PEst[0, 0]  # Variance for x
        cov[0,1] = self.PEst[0, 1]  # Covariance x-y
        cov[1,0] = self.PEst[1, 0]  # Covariance y-x
        cov[1,1] = self.PEst[1, 1]  # Variance for y
        odom_msg.pose.covariance = cov.flatten().tolist()
        print(f'Current Px variance: {self.PEst[0, 0]}, Py variance: {self.PEst[1, 1]}')
        
        self.ekf_pub.publish(odom_msg)
        
def main(args=None):
    rclpy.init(args=args)
    ekf_node = EKFFullNode()
    rclpy.spin(ekf_node)
    ekf_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
