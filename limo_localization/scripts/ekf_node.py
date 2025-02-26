#!/usr/bin/env python3

import yaml
import os
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
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
# Measurement noise covariance for odometry (12x12): [p (3), v (3), rpy (3), omega (3)]
covs = load_covariances()
if covs is not None:
    # For the EKF odometry update, we use the 'yaw_rate' covariance
    R_odom = np.array(covs.get("yaw_rate"))
    print("Loaded R_odom from YAML.")
else:
    # Fallback default
    R_odom = np.diag([
        0.01, 0.01, 0.01, # Position noise
        np.deg2rad(1.0), np.deg2rad(1.0), np.deg2rad(1.0), # Orientation noise (radians)
        0.01, 0.01, 0.01, # Linear velocity noise
        np.deg2rad(1.0), np.deg2rad(1.0), np.deg2rad(1.0) # Angular velocity noise (rad/s)
    ]) ** 2
    print("!YAML not loaded; using default R_odom.")

# Process noise covariance Q (15x15)
Q = np.diag([
    0.05, 0.05, 0.05,            # position noise
    np.deg2rad(1.0), np.deg2rad(1.0), np.deg2rad(1.0),  # orientation noise (rad)
    0.1, 0.1, 0.1,               # linear velocity noise
    np.deg2rad(0.5), np.deg2rad(0.5), np.deg2rad(0.5),  # angular velocity noise (rad/s)
    0.2, 0.2, 0.2                # linear acceleration noise
]) ** 2

# Measurement noise covariance for GPS (6x6): [p (3), v (3)]
R_GPS = np.diag([2.0, 2.0, 2.0, 0.0, 0.0, 0.0]) ** 2
# Measurement noise covariance for IMU (9x9): [orientation (3), angular velocity (3), linear acceleration (3)]
R_imu = np.diag([
    np.deg2rad(1.0), np.deg2rad(1.0), np.deg2rad(1.0),
    np.deg2rad(0.5), np.deg2rad(0.5), np.deg2rad(0.5),
    0.2, 0.2, 0.2
]) ** 2


# Helper functions: Rotation Matrix, Jacobian, and their derivatives

def R_from_euler(roll, pitch, yaw):
    """
    คำนวณ Rotation Matrix จาก Euler angles โดยใช้สูตร:
    R = Rz(yaw) * Ry(pitch) * Rx(roll)
    """
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
    """
    คำนวณเมทริกซ์แปลง J ที่แปลง angular velocity ให้เป็น Euler angle rates สำหรับ ZYX Euler angles
    """
    roll = float(roll)
    pitch = float(pitch)
    cos_pitch = math.cos(pitch)
    if abs(cos_pitch) < 1e-4:
        cos_pitch = 1e-4  # ป้องกัน division by zero
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

# Dynamic model and Jacobian for the prediction step

def dynamic_model(x, dt, R_from_euler, J_from_euler, u_alpha):
    """
    คำนวณ state ใหม่จาก state ปัจจุบันด้วยสมการ:
      pₖ₊₁ = pₖ + R(rₖ) (vₖ*dt + 0.5*aₖ*dt²)
      rₖ₊₁ = rₖ + J(rₖ)*ωₖ*dt
      vₖ₊₁ = vₖ + aₖ*dt
      ωₖ₊₁ = ωₖ + u_α*dt
      aₖ₊₁ = aₖ
    """
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
    # >>>>> MODIFIED: z now contains 12 measurements: position, orientation, linear velocity, angular velocity <<<<<
    H = np.zeros((12, 15))
    # Map position (state indices 0:3)
    H[0:3, 0:3] = np.eye(3)
    # Map orientation (roll, pitch, yaw) from state indices 3:6
    H[3:6, 3:6] = np.eye(3)
    # Map linear velocity from state indices 6:9
    H[6:9, 6:9] = np.eye(3)
    # Map angular velocity from state indices 9:12
    H[9:12, 9:12] = np.eye(3)

    zPred = H @ xEst
    y = z - zPred

    # >>>>> MODIFIED: Normalize the orientation residual (rows 3 to 5) <<<<<
    for i in range(3, 6):
        y[i, 0] = normalize_angle(y[i, 0])
        
    S = H @ PEst @ H.T + R_odom
    K = PEst @ H.T @ np.linalg.inv(S)
    xEst_new = xEst + K @ y
    PEst_new = (np.eye(15) - K @ H) @ PEst
    return xEst_new, PEst_new


def normalize_angle(angle):
    """
    ปรับมุมให้อยู่ในช่วง [-pi, pi].
    """
    return (angle + np.pi) % (2 * np.pi) - np.pi

def ekf_update_imu(xEst, PEst, z, R_imu):
    # สำหรับ IMU: วัด [roll, pitch, yaw, ω_x, ω_y, ω_z, a_x, a_y, a_z]^T
    H = np.zeros((9, 15))
    H[0:3, 3:6] = np.eye(3)      # orientation
    H[3:6, 9:12] = np.eye(3)     # angular velocity
    H[6:9, 12:15] = np.eye(3)    # linear acceleration
    H[8:14] = 0
    zPred = H @ xEst
    y = z - zPred  # innovation
    
    # Normalize the orientation innovation (roll, pitch, yaw)
    for i in range(3):
        y[i, 0] = normalize_angle(y[i, 0])
    
    S = H @ PEst @ H.T + R_imu
    K = PEst @ H.T @ np.linalg.inv(S)
    xEst_new = xEst + K @ y
    PEst_new = (np.eye(15) - K @ H) @ PEst
    
    # Normalize the updated orientation state as well
    for i in range(3):
        xEst_new[3 + i, 0] = normalize_angle(xEst_new[3 + i, 0])
    
    return xEst_new, PEst_new

def ekf_update_gps(xEst, PEst, z, R_GPS):
    # For GPS: measure [p_x, p_y, p_z, v_x, v_y, v_z]^T
    H = np.zeros((6, 15))
    # Here, you need to map the state vector to the measured states.
    # For example, if the state vector contains position at indices 0:3
    # and velocity at indices 6:9, then you can set:
    H[0:3, 0:3] = np.eye(3)
    H[3:6, 6:9] = np.eye(3)
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
        self.z_odom = None
        self.new_odom = False
        self.z_imu = None
        self.new_imu = False
        self.u_alpha = np.zeros((3,1))
        
        #gps_varibles
        self.new_gps = None
        
        self.odom_sub = self.create_subscription(Odometry, '/odometry/yaw_rate', self.odom_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.gps_sub = self.create_subscription(Odometry, '/gps/odom', self.gps_callback, 10)

        self.ekf_pub = self.create_publisher(PoseStamped, '/ekf_pose', 10)
        self.timer = self.create_timer(self.dt, self.timer_callback)
        
        
    def odom_callback(self, msg:Odometry):
        # Extract position (unchanged)
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        pz = msg.pose.pose.position.z

        # >>>>> MODIFIED: Extract orientation from quaternion and convert to Euler angles <<<<<
        q = msg.pose.pose.orientation
        quat = [q.x, q.y, q.z, q.w]
        (roll, pitch, yaw) = euler_from_quaternion(quat)

        # Extract linear velocity (unchanged)
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vz = msg.twist.twist.linear.z

        # >>>>> MODIFIED: Extract angular velocity <<<<<
        wx = msg.twist.twist.angular.x
        wy = msg.twist.twist.angular.y
        wz = msg.twist.twist.angular.z

        # >>>>> MODIFIED: Combine into a 12x1 measurement vector <<<<<
        # Order: [position (3), orientation (3), linear velocity (3), angular velocity (3)]
        self.z_odom = np.array([[px], [py], [pz],
                                [roll], [pitch], [yaw],
                                [vx], [vy], [vz],
                                [wx], [wy], [wz]])
        self.new_odom = True

        
    def imu_callback(self, msg):
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
        
    def gps_callback(self, msg:Odometry):
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        pz = msg.pose.pose.position.z
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vz = msg.twist.twist.linear.z
        self.z_gps = np.array([[px], [py], [pz], [vx], [vy], [vz]])
        self.new_gps = True
        
    def timer_callback(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            dt = self.dt
        self.last_time = current_time
        
        # Prediction step
        self.xEst, self.PEst = ekf_predict(self.xEst, self.PEst, dt, Q, R_from_euler, J_from_euler,
                                           dR_droll, dR_dpitch, dR_dyaw,
                                           dJ_droll, dJ_dpitch, dJ_dyaw, self.u_alpha)
        
        # Update step if new measurements are available
        if self.new_odom and self.z_odom is not None:
            self.xEst, self.PEst = ekf_update_odom(self.xEst, self.PEst, self.z_odom, R_odom)
            self.new_odom = False
        # if self.new_imu and self.z_imu is not None:
        #     self.xEst, self.PEst = ekf_update_imu(self.xEst, self.PEst, self.z_imu, R_imu)
        #     self.new_imu = False
        if self.new_gps and self.z_gps is not None:
            self.xEst, self.PEst = ekf_update_gps(self.xEst, self.PEst, self.z_gps, R_GPS)
            self.new_gps = False
        
        self.publish_estimate()
        
    def publish_estimate(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.pose.position.x = self.xEst[0, 0]
        msg.pose.position.y = self.xEst[1, 0]
        msg.pose.position.z = self.xEst[2, 0]
        roll = self.xEst[3, 0]
        pitch = self.xEst[4, 0]
        yaw = self.xEst[5, 0]
        q = quaternion_from_euler(roll, pitch, yaw)
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]
        self.ekf_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    ekf_node = EKFFullNode()
    rclpy.spin(ekf_node)
    ekf_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

