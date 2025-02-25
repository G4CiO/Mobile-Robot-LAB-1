#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Quaternion
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import numpy as np
import math

class EKFNode(Node):
    def __init__(self):
        super().__init__('ekf_odom_node')
        self.get_logger().info("15-State EKF odometry node has started.")

        # State vector: [px, py, pz, roll, pitch, yaw, vx, vy, vz, ωx, ωy, ωz, ax, ay, az]
        self.xEst = np.zeros((15, 1))
        self.PEst = np.eye(15)

        # Process noise covariance Q (15x15)
        self.Q = np.diag([
            0.01, 0.01, 0.01,                           # position noise (m)
            np.deg2rad(0.5), np.deg2rad(0.5), np.deg2rad(0.5),  # orientation noise (rad)
            0.05, 0.05, 0.05,                           # linear velocity noise (m/s)
            np.deg2rad(0.1), np.deg2rad(0.1), np.deg2rad(0.1),  # angular velocity noise (rad/s)
            0.02, 0.02, 0.02                            # acceleration noise (m/s²)
        ]) ** 2

        # Measurement noise covariance for odometry full measurement (12x12)
        # Measurement vector: [x, y, z, roll, pitch, yaw, vx, vy, vz, ωx, ωy, ωz]
        self.R_odom_full = np.block([
            [np.diag([2.0, 2.0, 2.0, np.deg2rad(5.0), np.deg2rad(5.0), np.deg2rad(5.0)]), np.zeros((6,6))],
            [np.zeros((6,6)), np.diag([0.1, 0.1, 0.1, np.deg2rad(0.1), np.deg2rad(0.1), np.deg2rad(0.1)])]
        ])

        # Flags and measurement storage
        self.new_odom = False
        # z_odom is 12x1: [x; y; z; roll; pitch; yaw; vx; vy; vz; ωx; ωy; ωz]
        self.z_odom = None

        # Subscribers using your topics
        self.gps_sub = self.create_subscription(
            Odometry, '/gps/odom', self.gps_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/yaw_rate', self.odom_callback, 10
        )

        # Publisher for EKF estimate (publish pose)
        self.ekf_pub = self.create_publisher(Odometry, '/ekf/odom', 10)

        # Timer for prediction & update
        self.dt = 0.01
        self.last_time = self.get_clock().now()
        self.timer = self.create_timer(self.dt, self.timer_callback)

    # --- Helper Functions ---
    def rotation_matrix(self, r):
        """Compute rotation matrix R from Euler angles (roll, pitch, yaw)"""
        r = np.array(r).flatten()  # Ensure 1D array
        rx, ry, rz = r[0], r[1], r[2]
        cx, sx = np.cos(rx), np.sin(rx)
        cy, sy = np.cos(ry), np.sin(ry)
        cz, sz = np.cos(rz), np.sin(rz)
        R = np.array([
            [cz*cy, cz*sy*sx - sz*cx, cz*sy*cx + sz*sx],
            [sz*cy, sz*sy*sx + cz*cx, sz*sy*cx - cz*sx],
            [-sy,   cy*sx,           cy*cx]
        ], dtype=float)
        return R

    def J_matrix(self, r):
        """Compute J matrix for converting angular velocity to Euler angle rates (ZYX Euler)"""
        r = np.array(r).flatten()
        rx, ry = r[0], r[1]
        sx, cx = np.sin(rx), np.cos(rx)
        sy, cy = np.sin(ry), np.cos(ry)
        ty = np.tan(ry)
        J = np.array([
            [1,      sx*ty,      cx*ty],
            [0,      cx,         -sx],
            [0,      sx/cy,      cx/cy]
        ], dtype=float)
        return J

    # --- Measurement Callbacks ---
    def gps_callback(self, msg: Odometry):
        # Extract pose from /gps/odom
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        quat = [msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w]
        # Convert quaternion to Euler angles (we only need yaw; optionally roll and pitch)
        (roll, pitch, yaw) = euler_from_quaternion(quat)
        # Here, we choose to use x,y,z and yaw from GPS.
        # You may also include roll and pitch if desired.
        self.z_gps = np.array([[x], [y], [z], [yaw]])
        # We will later fuse these into the full measurement vector.
        # For now, flag that a new GPS measurement is available.
        self.new_odom = True  # We'll fuse both topics into one update

    def odom_callback(self, msg: Odometry):
        # Extract full pose and twist from /odometry/yaw_rate
        # Even though the topic name is "yaw_rate", assume the message contains full odometry.
        # Pose:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        quat = [msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(quat)
        # Twist:
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vz = msg.twist.twist.linear.z
        wx = msg.twist.twist.angular.x
        wy = msg.twist.twist.angular.y
        wz = msg.twist.twist.angular.z

        # Build the 12x1 measurement vector
        # Here we choose to fuse both the GPS pose and the odometry full pose/twist.
        # For instance, we can choose to use the odometry message as the full measurement.
        # The measurement vector order is:
        # [x, y, z, roll, pitch, yaw, vx, vy, vz, ωx, ωy, ωz]
        self.z_odom = np.array([[x],
                                  [y],
                                  [z],
                                  [roll],
                                  [pitch],
                                  [yaw],
                                  [vx],
                                  [vy],
                                  [vz],
                                  [wx],
                                  [wy],
                                  [wz]])
        self.new_odom = True

    # --- Dynamic Model and Jacobian (Prediction) ---
    def dynamic_model(self, x, dt):
        """
        Prediction for 15-state model:
          pₖ₊₁ = pₖ + R(rₖ)*(vₖ*dt + 0.5*aₖ*dt²)
          rₖ₊₁ = rₖ + J(rₖ)*(ωₖ*dt)
          vₖ₊₁ = vₖ + aₖ*dt
          ωₖ₊₁ = ωₖ   (assumed constant)
          aₖ₊₁ = aₖ   (assumed constant)
        """
        p = x[0:3]
        r = x[3:6]
        v = x[6:9]
        w = x[9:12]
        a = x[12:15]

        R_mat = self.rotation_matrix(r)
        J = self.J_matrix(r)

        p_new = p + R_mat @ (v * dt + 0.5 * a * dt**2)
        r_new = r + J @ (w * dt)
        v_new = v + a * dt
        w_new = w
        a_new = a

        x_new = np.vstack((p_new, r_new, v_new, w_new, a_new))
        return x_new

    def jacobian_F(self, x, dt):
        """Compute approximate Jacobian F for the dynamic model"""
        F = np.eye(15)
        r = x[3:6]
        R_mat = self.rotation_matrix(r)
        F[0:3, 6:9] = R_mat * dt
        F[0:3, 12:15] = R_mat * (0.5 * dt**2)
        J = self.J_matrix(r)
        F[3:6, 9:12] = J * dt
        return F

    def ekf_predict(self, xEst, PEst, dt):
        F = self.jacobian_F(xEst, dt)
        xPred = self.dynamic_model(xEst, dt)
        PPred = F @ PEst @ F.T + self.Q
        return xPred, PPred

    # --- Measurement Update ---
    def ekf_update_full(self, xEst, PEst, z):
        """
        Update using full odometry measurement (12 elements):
          z = [x, y, z, roll, pitch, yaw, vx, vy, vz, ωx, ωy, ωz]^T
        State vector (15x1) includes acceleration which is not measured.
        """
        H = np.zeros((12, 15))
        # Map pose measurements:
        H[0, 0] = 1.0   # x
        H[1, 1] = 1.0   # y
        H[2, 2] = 1.0   # z
        H[3, 3] = 1.0   # roll
        H[4, 4] = 1.0   # pitch
        H[5, 5] = 1.0   # yaw
        # Map twist measurements:
        H[6, 6] = 1.0   # vx
        H[7, 7] = 1.0   # vy
        H[8, 8] = 1.0   # vz
        H[9, 9] = 1.0   # ωx
        H[10, 10] = 1.0 # ωy
        H[11, 11] = 1.0 # ωz

        zPred = H @ xEst
        y = z - zPred

        # Normalize angle differences for roll, pitch, yaw (rows 3-5)
        for i in range(3, 6):
            y[i, 0] = (y[i, 0] + np.pi) % (2 * np.pi) - np.pi

        S = H @ PEst @ H.T + self.R_odom_full
        K = PEst @ H.T @ np.linalg.inv(S)
        xNew = xEst + K @ y
        for i in range(3, 6):
            xNew[i, 0] = (xNew[i, 0] + np.pi) % (2 * np.pi) - np.pi
        PNew = (np.eye(15) - K @ H) @ PEst
        return xNew, PNew

    def ekf_update(self):
        """
        In this example, we fuse the measurements from both topics.
        We'll assume that the odometry topic (/odometry/yaw_rate) provides full
        odometry measurements (pose and twist), and we use those values.
        Optionally, you could fuse GPS and odometry separately.
        """
        # Here we use the measurement from odometry (full 12 measurements)
        if self.z_odom is not None:
            self.xEst, self.PEst = self.ekf_update_full(self.xEst, self.PEst, self.z_odom)
            self.new_odom = False

    # --- Timer Callback ---
    def timer_callback(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            dt = self.dt
        self.last_time = current_time

        # Prediction
        self.xEst, self.PEst = self.ekf_predict(self.xEst, self.PEst, dt)

        # If a new full odometry measurement is available, update
        if self.new_odom and self.z_odom is not None:
            self.ekf_update()

        self.publish_estimate()

    def publish_estimate(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"

        # Publish position
        msg.pose.pose.position.x = float(self.xEst[0, 0])
        msg.pose.pose.position.y = float(self.xEst[1, 0])
        msg.pose.pose.position.z = float(self.xEst[2, 0])
        # Convert Euler (roll, pitch, yaw) to quaternion
        roll = float(self.xEst[3, 0])
        pitch = float(self.xEst[4, 0])
        yaw = float(self.xEst[5, 0])
        q = quaternion_from_euler(roll, pitch, yaw)
        msg.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        # Optionally, publish some twist values
        msg.twist.twist.linear.x = float(self.xEst[6, 0])
        msg.twist.twist.angular.z = float(self.xEst[11, 0])

        self.ekf_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = EKFNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
