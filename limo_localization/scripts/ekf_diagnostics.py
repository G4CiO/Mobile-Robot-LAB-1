#!/usr/bin/env python3
"""
Node นี้ใช้สำหรับ:
  - วัดประสิทธิภาพของ EKF โดยคำนวณ RMSE ระหว่าง /ekf_odom กับ /odometry/ground_truth
    และคำนวณค่า NIS (Normalized Innovation Squared) หากมีข้อมูล innovation
  - แสดง realtime plot ของข้อมูล fusion จาก:
      • /ekf_odom (เส้นสีแดง)
      • /odometry/ground_truth (เส้นสีเขียว)
      • /odometry/yaw_rate (เส้นสีน้ำเงิน)  --> Wheel odometry
      • /gps/odom (จุดสีดำ)
  - รับข้อความหยุดจาก /stop_collection เมื่อได้รับให้หยุดการคำนวณ (stop calculation)
    โดยจะหยุด timer การคำนวณและ plot พร้อมสรุปผล diagnostics ออกทาง terminal
    (**หมายเหตุ**: โหนดนี้จะไม่ publish คำสั่งหยุดรถ)
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from tf_transformations import euler_from_quaternion
import numpy as np
import math
import matplotlib.pyplot as plt

# ฟังก์ชันช่วยปรับมุมให้อยู่ในช่วง [-pi, pi]
def normalize_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

class EKFDiagnosticsPlotNode(Node):
    def __init__(self):
        super().__init__('ekf_diagnostics_plot_node')
        self.get_logger().info("EKF Diagnostics Plot Node started.")
        
        # Flag สำหรับควบคุมการเก็บข้อมูลและคำนวณ
        self.collecting_data = True

        # Subscribers สำหรับข้อมูลที่ต้อง plot
        self.ekf_odom_sub = self.create_subscription(
            Odometry, '/ekf_odom', self.ekf_odom_callback, 10)
        self.gt_sub = self.create_subscription(
            Odometry, '/odometry/ground_truth', self.gt_callback, 10)
        self.yaw_rate_sub = self.create_subscription(
            Odometry, '/odometry/yaw_rate', self.yaw_rate_callback, 10)
        self.gps_sub = self.create_subscription(
            Odometry, '/gps/odom', self.gps_callback, 10)
        # Subscriber สำหรับ stop message
        self.stop_sub = self.create_subscription(
            Empty, '/stop_collection', self.stop_callback, 10)

        # Buffers สำหรับ realtime plotting (เก็บตำแหน่ง x,y)
        self.ekf_x, self.ekf_y = [], []
        self.gt_x, self.gt_y = [], []
        self.wheel_x, self.wheel_y = [], []    # จาก /odometry/yaw_rate
        self.gps_x, self.gps_y = [], []

        # Buffers สำหรับเก็บค่า diagnostic
        self.pos_errors = []     # error ระหว่างตำแหน่งของ ekf กับ ground truth
        self.yaw_errors = []     # เก็บ squared error ของ yaw
        self.innovations = []    # หากมี publish innovation (ตัวอย่าง)

        # เก็บค่า ground truth ล่าสุด สำหรับใช้คำนวณ error
        self.latest_gt = None  # (x, y, yaw)

        # ตั้งค่า realtime plotting
        plt.ion()
        fig_width, fig_height = 6, 6  # กำหนดขนาดรูป
        self.fig, self.ax = plt.subplots(figsize=(fig_width, fig_height))
        base_fontsize = min(fig_width, fig_height) * 1.5
        self.ax.set_xlabel("X [m]", fontsize=base_fontsize * 0.8)
        self.ax.set_ylabel("Y [m]", fontsize=base_fontsize * 0.8)
        self.ax.set_title("Realtime Fusion Plot", fontsize=base_fontsize)
        # สร้าง line objects สำหรับ plot
        self.ekf_line, = self.ax.plot([], [], 'r-', label='EKF Odom')
        self.gt_line, = self.ax.plot([], [], 'g-', label='Ground Truth')
        self.wheel_line, = self.ax.plot([], [], 'b-', label='Wheel Odom')
        self.gps_scatter, = self.ax.plot([], [], 'ko', markersize=4, label='GPS Odom')
        self.ax.legend(loc='upper left', fontsize=base_fontsize * 0.7)

        # Timer สำหรับ update plotting (อัตรา update 5 Hz)
        self.update_rate = 5.0  # Hz
        self.plot_timer = self.create_timer(1.0 / self.update_rate, self.plot_update_callback)

        # Timer สำหรับวิเคราะห์ diagnostics (คำนวณ RMSE, NIS) ทุก 5 วินาที
        self.diag_timer = self.create_timer(5.0, self.diag_update_callback)

    # Callback สำหรับ /ekf_odom
    def ekf_odom_callback(self, msg: Odometry):
        if not self.collecting_data:
            return
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.ekf_x.append(x)
        self.ekf_y.append(y)
        # ดึง orientation จาก EKF
        q = msg.pose.pose.orientation
        _, _, ekf_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        # หากมีข้อมูล ground truth ล่าสุด ให้นำมาคำนวณ error
        if self.latest_gt is not None:
            gt_x, gt_y, gt_yaw = self.latest_gt
            err = np.sqrt((x - gt_x)**2 + (y - gt_y)**2)
            self.pos_errors.append(err)
            yaw_err = abs(normalize_angle(ekf_yaw - gt_yaw))
            self.yaw_errors.append(yaw_err**2)
        # สามารถเก็บ innovation ได้ที่นี่หากมี

    # Callback สำหรับ /odometry/ground_truth
    def gt_callback(self, msg: Odometry):
        if not self.collecting_data:
            return
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.gt_x.append(x)
        self.gt_y.append(y)
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.latest_gt = (x, y, yaw)

    # Callback สำหรับ /odometry/yaw_rate (wheel odometry)
    def yaw_rate_callback(self, msg: Odometry):
        if not self.collecting_data:
            return
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.wheel_x.append(x)
        self.wheel_y.append(y)

    # Callback สำหรับ /gps/odom
    def gps_callback(self, msg: Odometry):
        if not self.collecting_data:
            return
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.gps_x.append(x)
        self.gps_y.append(y)

    # Callback สำหรับ stop message
    def stop_callback(self, msg: Empty):
        if self.collecting_data:
            self.get_logger().info("Received stop_collection. Stopping diagnostics calculation.")
            self.collecting_data = False
            # ยกเลิก timer ที่เกี่ยวกับการคำนวณและ plot
            self.plot_timer.cancel()
            self.diag_timer.cancel()
            # Publish stop command (แบบ --once) ถ้าต้องการให้ publish cmd_vel=0 ให้ uncomment บรรทัดด้านล่าง
            # (ในที่นี้เราไม่ต้องการหยุดรถ)
            # stop_pub = self.create_publisher(Twist, '/cmd_vel', 10)
            # stop_msg = Twist()
            # stop_msg.linear.x = 0.0
            # stop_msg.angular.z = 0.0
            # stop_pub.publish(stop_msg)
            self.finalize_diagnostics()

    # Timer callback สำหรับ update realtime plot
    def plot_update_callback(self):
        self.ekf_line.set_xdata(self.ekf_x)
        self.ekf_line.set_ydata(self.ekf_y)
        self.gt_line.set_xdata(self.gt_x)
        self.gt_line.set_ydata(self.gt_y)
        self.wheel_line.set_xdata(self.wheel_x)
        self.wheel_line.set_ydata(self.wheel_y)
        self.gps_scatter.set_xdata(self.gps_x)
        self.gps_scatter.set_ydata(self.gps_y)
        all_x = self.ekf_x + self.gt_x + self.wheel_x + self.gps_x
        all_y = self.ekf_y + self.gt_y + self.wheel_y + self.gps_y
        if all_x and all_y:
            min_x, max_x = min(all_x), max(all_x)
            min_y, max_y = min(all_y), max(all_y)
            self.ax.set_xlim(min_x - 0.5, max_x + 0.5)
            self.ax.set_ylim(min_y - 0.5, max_y + 0.5)
        plt.draw()
        plt.pause(0.001)

    # Timer callback สำหรับ diagnostics (คำนวณ RMSE, NIS) ทุก 5 วินาที
    def diag_update_callback(self):
        if len(self.pos_errors) > 0:
            rmse_pos = np.sqrt(np.mean(np.array(self.pos_errors)**2))
            rmse_yaw = np.sqrt(np.mean(np.array(self.yaw_errors)))
            self.get_logger().info(f"RMSE Position: {rmse_pos:.4f} m, RMSE Yaw: {rmse_yaw:.4f} rad")
        else:
            self.get_logger().info("No error data yet for RMSE calculation.")
        if len(self.innovations) > 0:
            avg_nis = np.mean(self.innovations)
            self.get_logger().info(f"Average NIS: {avg_nis:.4f}")
        self.pos_errors = []
        self.yaw_errors = []
        self.innovations = []

    # หลังจากหยุดการคำนวณแล้ว ให้สรุปผลออกทาง terminal
    def finalize_diagnostics(self):
        if len(self.pos_errors) > 0:
            final_rmse_pos = np.sqrt(np.mean(np.array(self.pos_errors)**2))
            final_rmse_yaw = np.sqrt(np.mean(np.array(self.yaw_errors)))
        else:
            final_rmse_pos, final_rmse_yaw = 0.0, 0.0
        self.get_logger().info("===== Final Diagnostics Summary =====")
        self.get_logger().info(f"Final RMSE Position: {final_rmse_pos:.4f} m")
        self.get_logger().info(f"Final RMSE Yaw: {final_rmse_yaw:.4f} rad")
        
def main(args=None):
    rclpy.init(args=args)
    node = EKFDiagnosticsPlotNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
