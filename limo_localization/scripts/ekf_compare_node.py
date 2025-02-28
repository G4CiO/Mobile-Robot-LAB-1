#!/usr/bin/env python3
"""
ekf_comparision_node.py

This node performs EKF fusion using four sensor combinations:
  1. GPS + yaw_rate
  2. GPS + single_track
  3. GPS + double_track
  4. GPS + yaw_rate, single_track, double_track

It loads covariance matrices from a YAML file (using the workspace finder),
runs full EKF math (dynamic model, Jacobian, prediction, update), and
visualizes the results in a single row of 4 subplots with:
  - Ground Truth: solid black line
  - EKF Estimate: long dashed red line (dash pattern [10, 5])
  - Wheel Odometry: short dashed blue line (dash pattern [2, 2])
  - GPS: green circles (alpha=0.2)

Additionally, it publishes the fused EKF state as an Odometry message on '/ekf_odom'.
Adjust parameters as needed.
"""

import os
import yaml
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import numpy as np
import math
import matplotlib.pyplot as plt

# -------------------- Workspace & YAML Covariance Loading -------------------- #
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
    data_dir = os.path.join(workspace_dir, "src", "Mobile-Robot-LAB-1", "limo_localization", "config")
    data_dir = os.path.abspath(data_dir)
    yaml_file = os.path.join(data_dir, "covariances.yaml")
    if not os.path.exists(yaml_file):
        print(f"Error: YAML file not found at {yaml_file}")
        return None
    with open(yaml_file, "r") as f:
        data = yaml.safe_load(f)
    return data.get("covariances", None)

covs = load_covariances()
if covs is not None:
    R_yaw_rate = np.array(covs.get("yaw_rate"))
    R_single_track = np.array(covs.get("single_track"))
    R_double_track = np.array(covs.get("double_track"))
    print("Loaded odometry covariances from YAML.")
else:
    R_yaw_rate = np.diag([
        0.01, 0.01, 0.01,
        np.deg2rad(1.0), np.deg2rad(1.0), np.deg2rad(1.0),
        0.01, 0.01, 0.01,
        np.deg2rad(1.0), np.deg2rad(1.0), np.deg2rad(1.0)
    ]) ** 2
    R_single_track = R_yaw_rate.copy()
    R_double_track = R_yaw_rate.copy()
    print("!YAML not loaded; using default odometry covariances.")

# Process noise covariance Q (15x15)
Q = np.diag([
    0.1, 0.1, 0.1,   # position noise
    0.1, 0.1, 0.1,   # orientation noise (rad)
    0.1, 0.1, 0.1,   # linear velocity noise
    0.1, 0.1, 0.1,   # angular velocity noise (rad/s)
    0.1, 0.1, 0.1    # linear acceleration noise
]) ** 2

# Measurement noise covariances
R_GPS = np.diag([1.0, 1.0, 1.0]) ** 2
R_imu = np.diag([
    np.deg2rad(1.0), np.deg2rad(1.0), np.deg2rad(1.0),
    np.deg2rad(0.5), np.deg2rad(0.5), np.deg2rad(0.5),
    0.2, 0.2, 0.2
]) ** 2

# -------------------- EKF Math Functions -------------------- #
def normalize_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

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
                   [0, 1, 0],
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
    for i in range(3, 6):
        y[i, 0] = normalize_angle(y[i, 0])
    S = H @ PEst @ H.T + R_odom
    K = PEst @ H.T @ np.linalg.inv(S)
    xEst_new = xEst + K @ y
    PEst_new = (np.eye(15) - K @ H) @ PEst
    return xEst_new, PEst_new

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
        xEst_new[3+i, 0] = normalize_angle(xEst_new[3+i, 0])
    return xEst_new, PEst_new

def ekf_update_gps(xEst, PEst, z, R_GPS):
    H = np.zeros((3, 15))
    H[0:3, 0:3] = np.eye(3)
    zPred = H @ xEst
    y = z - zPred
    S = H @ PEst @ H.T + R_GPS
    K = PEst @ H.T @ np.linalg.inv(S)
    xEst_new = xEst + K @ y
    PEst_new = (np.eye(15) - K @ H) @ PEst
    return xEst_new, PEst_new

# -------------------- EKF Comparison Node with Plotting & Publishing -------------------- #
class EKFComparisonNode(Node):
    def __init__(self):
        super().__init__('ekf_comparision_node')
        self.dt = 0.1
        self.last_time = self.get_clock().now()

        # Initialize 4 parallel EKF filters (each 15x1 state, 15x15 covariance)
        self.xEst1 = np.zeros((15,1))  # Mode 1: GPS + yaw_rate
        self.xEst2 = np.zeros((15,1))  # Mode 2: GPS + single_track
        self.xEst3 = np.zeros((15,1))  # Mode 3: GPS + double_track
        self.xEst4 = np.zeros((15,1))  # Mode 4: GPS + all sensors

        self.PEst1 = np.eye(15)
        self.PEst2 = np.eye(15)
        self.PEst3 = np.eye(15)
        self.PEst4 = np.eye(15)

        # Constant input acceleration (assumed zero)
        self.u_alpha = np.zeros((3,1))

        # Use YAML loaded covariances (or defaults) for odometry update
        self.R_yaw_rate = R_yaw_rate
        self.R_single_track = R_single_track
        self.R_double_track = R_double_track

        # Subscribers for sensor topics
        self.create_subscription(Odometry, '/gps/odom', self.gps_callback, 10)
        self.create_subscription(Odometry, '/odometry/yaw_rate', self.yaw_rate_callback, 10)
        self.create_subscription(Odometry, '/odometry/single_track', self.single_track_callback, 10)
        self.create_subscription(Odometry, '/odometry/double_track', self.double_track_callback, 10)
        self.create_subscription(Odometry, '/odometry/ground_truth', self.gt_callback, 10)

        # Publish EKF estimate using Odometry message
        self.ekf_pub = self.create_publisher(Odometry, '/ekf_odom', 10)

        # Latest measurements
        self.z_gps = None
        self.z_yaw_rate = None
        self.z_single_track = None
        self.z_double_track = None
        self.gt_xy = None  # Ground truth position (x, y)

        # Histories for plotting
        self.gt_history = []      # Ground truth history
        self.gps_history = []     # GPS history
        self.odom_history1 = []   # Wheel odometry from yaw_rate (Mode 1)
        self.odom_history2 = []   # From single_track (Mode 2)
        self.odom_history3 = []   # From double_track (Mode 3)
        self.odom_history4 = []   # For Mode 4 (using yaw_rate as representative)
        self.odom_history4_yaw = []
        self.odom_history4_single = []
        self.odom_history4_double = []

        self.ekf_history1 = []    # EKF estimate history for Mode 1
        self.ekf_history2 = []    # Mode 2
        self.ekf_history3 = []    # Mode 3
        self.ekf_history4 = []    # Mode 4

        # Set up matplotlib subplots (1 row, 4 columns)
        self.fig, self.axes = plt.subplots(1, 4, figsize=(20, 5))
        self.fig.suptitle('EKF Fusion Comparison with Ground Truth')
        for ax in self.axes:
            ax.set_xlabel('X [m]')
            ax.set_ylabel('Y [m]')
        plt.ion()
        plt.show()

        # Timer for EKF update and plotting
        self.timer = self.create_timer(self.dt, self.timer_callback)

    # ---------- Sensor Callbacks ---------- #
    def gps_callback(self, msg: Odometry):
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        pz = msg.pose.pose.position.z
        self.z_gps = np.array([[px], [py], [pz]])

    def yaw_rate_callback(self, msg: Odometry):
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        pz = msg.pose.pose.position.z
        q = msg.pose.pose.orientation
        (roll, pitch, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
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

    def single_track_callback(self, msg: Odometry):
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        pz = msg.pose.pose.position.z
        q = msg.pose.pose.orientation
        (roll, pitch, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
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

    def double_track_callback(self, msg: Odometry):
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        pz = msg.pose.pose.position.z
        q = msg.pose.pose.orientation
        (roll, pitch, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
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

    def gt_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.gt_xy = (x, y)

    # ---------- Timer Callback: EKF Update, Plotting & Publishing ---------- #
    def timer_callback(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            dt = self.dt
        self.last_time = current_time

        # Prediction for each mode
        self.xEst1, self.PEst1 = ekf_predict(self.xEst1, self.PEst1, dt, Q,
                                              R_from_euler, J_from_euler,
                                              dR_droll, dR_dpitch, dR_dyaw,
                                              dJ_droll, dJ_dpitch, dJ_dyaw, self.u_alpha)
        self.xEst2, self.PEst2 = ekf_predict(self.xEst2, self.PEst2, dt, Q,
                                              R_from_euler, J_from_euler,
                                              dR_droll, dR_dpitch, dR_dyaw,
                                              dJ_droll, dJ_dpitch, dJ_dyaw, self.u_alpha)
        self.xEst3, self.PEst3 = ekf_predict(self.xEst3, self.PEst3, dt, Q,
                                              R_from_euler, J_from_euler,
                                              dR_droll, dR_dpitch, dR_dyaw,
                                              dJ_droll, dJ_dpitch, dJ_dyaw, self.u_alpha)
        self.xEst4, self.PEst4 = ekf_predict(self.xEst4, self.PEst4, dt, Q,
                                              R_from_euler, J_from_euler,
                                              dR_droll, dR_dpitch, dR_dyaw,
                                              dJ_droll, dJ_dpitch, dJ_dyaw, self.u_alpha)

        # Update with GPS measurement (if available)
        if self.z_gps is not None:
            self.xEst1, self.PEst1 = ekf_update_gps(self.xEst1, self.PEst1, self.z_gps, R_GPS)
            self.xEst2, self.PEst2 = ekf_update_gps(self.xEst2, self.PEst2, self.z_gps, R_GPS)
            self.xEst3, self.PEst3 = ekf_update_gps(self.xEst3, self.PEst3, self.z_gps, R_GPS)
            self.xEst4, self.PEst4 = ekf_update_gps(self.xEst4, self.PEst4, self.z_gps, R_GPS)

        # Update with wheel odometry measurements
        if self.z_yaw_rate is not None:
            self.xEst1, self.PEst1 = ekf_update_odom(self.xEst1, self.PEst1, self.z_yaw_rate, self.R_yaw_rate)
            self.xEst4, self.PEst4 = ekf_update_odom(self.xEst4, self.PEst4, self.z_yaw_rate, self.R_yaw_rate)
            self.odom_history1.append((self.z_yaw_rate[0,0], self.z_yaw_rate[1,0]))
            self.odom_history4_yaw.append((self.z_yaw_rate[0,0], self.z_yaw_rate[1,0]))

        if self.z_single_track is not None:
            self.xEst2, self.PEst2 = ekf_update_odom(self.xEst2, self.PEst2, self.z_single_track, self.R_single_track)
            self.xEst4, self.PEst4 = ekf_update_odom(self.xEst4, self.PEst4, self.z_single_track, self.R_single_track)
            self.odom_history2.append((self.z_single_track[0,0], self.z_single_track[1,0]))
            self.odom_history4_single.append((self.z_single_track[0,0], self.z_single_track[1,0]))

        if self.z_double_track is not None:
            self.xEst3, self.PEst3 = ekf_update_odom(self.xEst3, self.PEst3, self.z_double_track, self.R_double_track)
            self.xEst4, self.PEst4 = ekf_update_odom(self.xEst4, self.PEst4, self.z_double_track, self.R_double_track)
            self.odom_history3.append((self.z_double_track[0,0], self.z_double_track[1,0]))
            self.odom_history4_double.append((self.z_double_track[0,0], self.z_double_track[1,0]))


        # Record histories for plotting
        if self.gt_xy is not None:
            self.gt_history.append(self.gt_xy)
        if self.z_gps is not None:
            self.gps_history.append((self.z_gps[0,0], self.z_gps[1,0]))
        self.ekf_history1.append((self.xEst1[0,0], self.xEst1[1,0]))
        self.ekf_history2.append((self.xEst2[0,0], self.xEst2[1,0]))
        self.ekf_history3.append((self.xEst3[0,0], self.xEst3[1,0]))
        self.ekf_history4.append((self.xEst4[0,0], self.xEst4[1,0]))

        self.update_plots()
        self.publish_estimate()

    def update_plots(self):
        for ax in self.axes:
            ax.cla()
            ax.set_xlabel('X [m]')
            ax.set_ylabel('Y [m]')

        def extract_xy(history):
            if len(history) == 0:
                return [], []
            xs, ys = zip(*history)
            return xs, ys

        # Mode 1: GPS + yaw_rate
        ax = self.axes[0]
        ax.set_title('GPS + yaw_rate')
        xs, ys = extract_xy(self.gt_history)
        ax.plot(xs, ys, '-', color='k', label='Ground Truth')
        xs, ys = extract_xy(self.ekf_history1)
        line1, = ax.plot(xs, ys, '--', color='r', label='EKF Estimate')
        line1.set_dashes([10, 5])
        xs, ys = extract_xy(self.odom_history1)
        line2, = ax.plot(xs, ys, '--', color='b', label='Wheel Odometry')
        line2.set_dashes([2, 2])
        xs, ys = extract_xy(self.gps_history)
        ax.plot(xs, ys, 'o', color='g', alpha=0.2, label='GPS')
        ax.legend()

        # Mode 2: GPS + single_track
        ax = self.axes[1]
        ax.set_title('GPS + single_track')
        xs, ys = extract_xy(self.gt_history)
        ax.plot(xs, ys, '-', color='k', label='Ground Truth')
        xs, ys = extract_xy(self.ekf_history2)
        line1, = ax.plot(xs, ys, '--', color='r', label='EKF Estimate')
        line1.set_dashes([10, 5])
        xs, ys = extract_xy(self.odom_history2)
        line2, = ax.plot(xs, ys, '--', color='b', label='Wheel Odometry')
        line2.set_dashes([2, 2])
        xs, ys = extract_xy(self.gps_history)
        ax.plot(xs, ys, 'o', color='g', alpha=0.2, label='GPS')
        ax.legend()

        # Mode 3: GPS + double_track
        ax = self.axes[2]
        ax.set_title('GPS + double_track')
        xs, ys = extract_xy(self.gt_history)
        ax.plot(xs, ys, '-', color='k', label='Ground Truth')
        xs, ys = extract_xy(self.ekf_history3)
        line1, = ax.plot(xs, ys, '--', color='r', label='EKF Estimate')
        line1.set_dashes([10, 5])
        xs, ys = extract_xy(self.odom_history3)
        line2, = ax.plot(xs, ys, '--', color='b', label='Wheel Odometry')
        line2.set_dashes([2, 2])
        xs, ys = extract_xy(self.gps_history)
        ax.plot(xs, ys, 'o', color='g', alpha=0.2, label='GPS')
        ax.legend()

        # Mode 4: GPS + all sensors
        ax = self.axes[3]
        ax.set_title('GPS + all Wheel Odometry')
        xs, ys = extract_xy(self.gt_history)
        ax.plot(xs, ys, '-', color='k', label='Ground Truth')
        xs, ys = extract_xy(self.ekf_history4)
        line1, = ax.plot(xs, ys, '--', color='r', label='EKF Estimate')
        line1.set_dashes([10, 5])
        # Plot yaw_rate odometry for Mode 4
        xs, ys = extract_xy(self.odom_history4_yaw)
        line_yaw, = ax.plot(xs, ys, '--', color='b', label='Yaw Rate')
        line_yaw.set_dashes([2, 2])

        # Plot single_track odometry for Mode 4
        xs, ys = extract_xy(self.odom_history4_single)
        line_single, = ax.plot(xs, ys, '--', color='c', label='Single Track')
        line_single.set_dashes([2, 2])

        # Plot double_track odometry for Mode 4
        xs, ys = extract_xy(self.odom_history4_double)
        line_double, = ax.plot(xs, ys, '--', color='m', label='Double Track')
        line_double.set_dashes([2, 2])
        xs, ys = extract_xy(self.gps_history)
        ax.plot(xs, ys, 'o', color='g', alpha=0.2, label='GPS')
        ax.legend()

        self.fig.tight_layout()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def publish_estimate(self):
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        odom_msg.pose.pose.position.x = self.xEst4[0, 0]
        odom_msg.pose.pose.position.y = self.xEst4[1, 0]
        odom_msg.pose.pose.position.z = self.xEst4[2, 0]

        roll = self.xEst4[3, 0]
        pitch = self.xEst4[4, 0]
        yaw = self.xEst4[5, 0]
        q = quaternion_from_euler(roll, pitch, yaw)
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]

        # Publish a simple 6x6 covariance for the pose (only x and y)
        cov = np.zeros((6,6))
        cov[0,0] = self.PEst4[0, 0]
        cov[0,1] = self.PEst4[0, 1]
        cov[1,0] = self.PEst4[1, 0]
        cov[1,1] = self.PEst4[1, 1]
        odom_msg.pose.covariance = cov.flatten().tolist()
        self.ekf_pub.publish(odom_msg)
        self.get_logger().info(f"Published EKF estimate with Px variance: {self.PEst4[0,0]}")

def main(args=None):
    rclpy.init(args=args)
    node = EKFComparisonNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    plt.ioff()
    plt.show()

if __name__ == '__main__':
    main()
