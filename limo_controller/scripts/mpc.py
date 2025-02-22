#!/usr/bin/python3

from limo_controller.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
import time
import cvxpy
import math
import numpy as np
import sys
import pathlib
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
# from CubicSpline import cubic_spline_planner
# from utils.angle import angle_mod
import cubic_spline_planner
from angle import angle_mod
sys.path.append(str(pathlib.Path(__file__).parent.parent.parent))

# class State:
#     """
#     vehicle state class
#     """

#     def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
#         self.x = x
#         self.y = y
#         self.yaw = yaw
#         self.v = v
#         self.predelta = None

class MPCNode(Node):
    def __init__(self):
        super().__init__('mpc_node')
        self.NX = 4  # x = x, y, v, yaw
        self.NU = 2  # a = [accel, steer]
        self.T = 5  # horizon length

        # mpc parameters
        self.R = np.diag([0.01, 0.01])  # input cost matrix
        self.Rd = np.diag([0.01, 1.0])  # input difference cost matrix
        self.Q = np.diag([1.0, 1.0, 0.5, 0.5])  # state cost matrix
        self.Qf = self.Q  # state final matrix
        self.GOAL_DIS = 1.5  # goal distance
        self.STOP_SPEED = 0.5 / 3.6  # stop speed
        self.MAX_TIME = 500.0  # max simulation time
        self.state = np.array([0.0, 0.0, 0.0, 0.0])


        # iterative paramter
        self.MAX_ITER = 3  # Max iteration
        self.DU_TH = 0.1  # iteration finish param

        self.TARGET_SPEED = 1.5  # [m/s] target speed
        self.N_IND_SEARCH = 10  # Search index number

        self.DT = 0.01  # [s] time tick

        # Vehicle parameters
        self.LENGTH = 4.5  # [m]
        self.WIDTH = 2.0  # [m]
        self.BACKTOWHEEL = 1.0  # [m]
        self.WHEEL_LEN = 0.3  # [m]
        self.WHEEL_WIDTH = 0.2  # [m]
        self.TREAD = 0.7  # [m]
        self.WB = 0.2  # [m]

        self.MAX_STEER = np.deg2rad(30.0)  # maximum steering angle [rad]
        self.MAX_DSTEER = np.deg2rad(30.0)  # maximum steering speed [rad/s]
        self.MAX_SPEED = 5.0 # maximum speed [m/s]
        self.MIN_SPEED = -5.0  # minimum speed [m/s]
        self.MAX_ACCEL = 1.0  # maximum accel [m/ss]

        self.show_animation = True

        self.odom_sub = self.create_subscription(Odometry, '/odometry/ground_truth', self.odom_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(self.DT, self.timer_callback)  # 100 Hz

    def odom_callback(self, msg:Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        self.yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])[2]
        self.v = msg.twist.twist.linear.x
        self.state = np.array([
            self.x,
            self.y,
            self.yaw,
            self.v
        ])

    def pi_2_pi(self, angle):
        return angle_mod(angle)
    
    def get_linear_model_matrix(self, v, phi, delta):

        A = np.zeros((self.NX, self.NX))
        A[0, 0] = 1.0
        A[1, 1] = 1.0
        A[2, 2] = 1.0
        A[3, 3] = 1.0
        A[0, 2] = self.DT * math.cos(phi)
        A[0, 3] = - self.DT * v * math.sin(phi)
        A[1, 2] = self.DT * math.sin(phi)
        A[1, 3] = self.DT * v * math.cos(phi)
        A[3, 2] = self.DT * math.tan(delta) / self.WB

        B = np.zeros((self.NX, self.NU))
        B[2, 0] = self.DT
        B[3, 1] = self.DT * v / (self.WB * math.cos(delta) ** 2)

        C = np.zeros(self.NX)
        C[0] = self.DT * v * math.sin(phi) * phi
        C[1] = - self.DT * v * math.cos(phi) * phi
        C[3] = - self.DT * v * delta / (self.WB * math.cos(delta) ** 2)

        return A, B, C

    # def update_state(self, state, a, delta):

    #     # input check
    #     if delta >= self.MAX_STEER:
    #         delta = self.MAX_STEER
    #     elif delta <= -self.MAX_STEER:
    #         delta = -self.MAX_STEER

    #     state.x = state.x + state.v * math.cos(state.yaw) * self.DT
    #     state.y = state.y + state.v * math.sin(state.yaw) * self.DT
    #     state.yaw = state.yaw + state.v / self.WB * math.tan(delta) * self.DT
    #     state.v = state.v + a * self.DT

    #     if state.v > self.MAX_SPEED:
    #         state.v = self.MAX_SPEED
    #     elif state.v < self.MIN_SPEED:
    #         state.v = self.MIN_SPEED

    #     return state
    
    def get_nparray_from_matrix(self, x):
        return np.array(x).flatten()
    
    def calc_nearest_index(self, state, cx, cy, cyaw, pind):

        dx = [state[0] - icx for icx in cx[pind:(pind + self.N_IND_SEARCH)]]
        dy = [state[1] - icy for icy in cy[pind:(pind + self.N_IND_SEARCH)]]

        d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

        mind = min(d)

        ind = d.index(mind) + pind

        mind = math.sqrt(mind)

        dxl = cx[ind] - state[0]
        dyl = cy[ind] - state[1]

        angle = self.pi_2_pi(cyaw[ind] - math.atan2(dyl, dxl))
        if angle < 0:
            mind *= -1

        return ind, mind
    
    # Use update_state()
    def predict_motion(self, x0, oa, od, xref):
        xbar = xref * 0.0
        for i, _ in enumerate(x0):
            xbar[i, 0] = x0[i]

        for (ai, di, i) in zip(oa, od, range(1, self.T + 1)):
            state = self.state
            xbar[0, i] = state[0]
            xbar[1, i] = state[1]
            xbar[2, i] = state[3]
            xbar[3, i] = state[2]

        return xbar
    
# //////////////////////////////////////////////////////////////////////////////

    # Use calc_nearest_index()
    def calc_ref_trajectory(self, state, cx, cy, cyaw, ck, sp, dl, pind):
        xref = np.zeros((self.NX, self.T + 1))
        dref = np.zeros((1, self.T + 1))
        ncourse = len(cx)

        ind, _ = self.calc_nearest_index(self.state, cx, cy, cyaw, pind)

        if pind >= ind:
            ind = pind

        xref[0, 0] = cx[ind]
        xref[1, 0] = cy[ind]
        xref[2, 0] = sp[ind]
        xref[3, 0] = cyaw[ind]
        dref[0, 0] = 0.0  # steer operational point should be 0

        travel = 0.0

        for i in range(self.T + 1):
            travel += abs(state[3]) * self.DT
            dind = int(round(travel / dl))

            if (ind + dind) < ncourse:
                xref[0, i] = cx[ind + dind]
                xref[1, i] = cy[ind + dind]
                xref[2, i] = sp[ind + dind]
                xref[3, i] = cyaw[ind + dind]
                dref[0, i] = 0.0
            else:
                xref[0, i] = cx[ncourse - 1]
                xref[1, i] = cy[ncourse - 1]
                xref[2, i] = sp[ncourse - 1]
                xref[3, i] = cyaw[ncourse - 1]
                dref[0, i] = 0.0

        return xref, ind, dref
    
    # Use predict_motion(), linear_mpc_control()
    def iterative_linear_mpc_control(self, xref, x0, dref, oa, od):
        """
        MPC control with updating operational point iteratively
        """
        ox, oy, oyaw, ov = None, None, None, None

        if oa is None or od is None:
            oa = [0.0] * self.T
            od = [0.0] * self.T

        for i in range(self.MAX_ITER):
            xbar = self.predict_motion(x0, oa, od, xref)
            poa, pod = oa[:], od[:]
            oa, od, ox, oy, oyaw, ov = self.linear_mpc_control(xref, xbar, x0, dref)
            du = sum(abs(oa - poa)) + sum(abs(od - pod))  # calc u change value
            if du <= self.DU_TH:
                break
        else:
            print("Iterative is max iter")

        return oa, od, ox, oy, oyaw, ov
    
   # Use get_linear_model_matrix(), get_nparray_from_matrix()
    def linear_mpc_control(self, xref, xbar, x0, dref):
        """
        linear mpc control

        xref: reference point
        xbar: operational point
        x0: initial state
        dref: reference steer angle
        """

        x = cvxpy.Variable((self.NX, self.T + 1))
        u = cvxpy.Variable((self.NU, self.T))

        cost = 0.0
        constraints = []

        for t in range(self.T):
            cost += cvxpy.quad_form(u[:, t], self.R)

            if t != 0:
                cost += cvxpy.quad_form(xref[:, t] - x[:, t], self.Q)

            A, B, C = self.get_linear_model_matrix(
                xbar[2, t], xbar[3, t], dref[0, t])
            constraints += [x[:, t + 1] == A @ x[:, t] + B @ u[:, t] + C]

            if t < (self.T - 1):
                cost += cvxpy.quad_form(u[:, t + 1] - u[:, t], self.Rd)
                constraints += [cvxpy.abs(u[1, t + 1] - u[1, t]) <=
                                self.MAX_DSTEER * self.DT]

        cost += cvxpy.quad_form(xref[:, self.T] - x[:, self.T], self.Qf)

        constraints += [x[:, 0] == x0]
        constraints += [x[2, :] <= self.MAX_SPEED]
        constraints += [x[2, :] >= self.MIN_SPEED]
        constraints += [cvxpy.abs(u[0, :]) <= self.MAX_ACCEL]
        constraints += [cvxpy.abs(u[1, :]) <= self.MAX_STEER]

        prob = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
        prob.solve(solver=cvxpy.CLARABEL, verbose=False)

        if prob.status == cvxpy.OPTIMAL or prob.status == cvxpy.OPTIMAL_INACCURATE:
            ox = self.get_nparray_from_matrix(x.value[0, :])
            oy = self.get_nparray_from_matrix(x.value[1, :])
            ov = self.get_nparray_from_matrix(x.value[2, :])
            oyaw = self.get_nparray_from_matrix(x.value[3, :])
            oa = self.get_nparray_from_matrix(u.value[0, :])
            odelta = self.get_nparray_from_matrix(u.value[1, :])

        else:
            print("Error: Cannot solve mpc..")
            oa, odelta, ox, oy, oyaw, ov = None, None, None, None, None, None

        return oa, odelta, ox, oy, oyaw, ov
    
# //////////////////////////////////////////////////////////////////////////////

    def check_goal(self, state, goal, tind, nind):

        # check goal
        dx = state[0] - goal[0]
        dy = state[1] - goal[1]
        d = math.hypot(dx, dy)

        isgoal = (d <= self.GOAL_DIS)

        if abs(tind - nind) >= 5:
            isgoal = False

        isstop = (abs(state[3]) <= self.STOP_SPEED)

        if isgoal and isstop:
            return True

        return False

    # Use calc_nearest_index(), calc_ref_trajectory(), iterative_linear_mpc_control, update_state(), smooth_yaw()
    def do_simulation(self, cx, cy, cyaw, ck, sp, dl, state):
        """
        Simulation

        cx: course x position list
        cy: course y position list
        cy: course yaw position list
        ck: course curvature list
        sp: speed profile
        dl: course tick [m]

        """

        goal = [cx[-1], cy[-1]]

        # initial yaw compensation
        if state[2] - cyaw[0] >= math.pi:
            state[2] -= math.pi * 2.0
        elif state[2] - cyaw[0] <= -math.pi:
            state[2] += math.pi * 2.0

        time = 0.0
        x = [state[0]]
        y = [state[1]]
        yaw = [state[2]]
        v = [state[3]]
        t = [0.0]
        d = [0.0]
        a = [0.0]
        target_ind, _ = self.calc_nearest_index(self.state, cx, cy, cyaw, 0)

        odelta, oa = None, None

        cyaw = self.smooth_yaw(cyaw)

        while self.MAX_TIME >= time:
            xref, target_ind, dref = self.calc_ref_trajectory(
                self.state, cx, cy, cyaw, ck, sp, dl, target_ind)

            x0 = [state[0], state[1], state[3], state[2]]  # current state

            oa, odelta, ox, oy, oyaw, ov = self.iterative_linear_mpc_control(
                xref, x0, dref, oa, odelta)

            di, ai = 0.0, 0.0
            if odelta is not None:
                di, ai = odelta[0], oa[0]

            time = time + self.DT

            x.append(state[0])
            y.append(state[1])
            yaw.append(state[2])
            v.append(state[3])
            t.append(time)
            d.append(di)
            a.append(ai)

            if self.check_goal(state, goal, target_ind, len(cx)):
                print("Goal")
                break

        return t, x, y, yaw, v, d, a
        
    # Use pi_2_pi()
    def calc_speed_profile(self, cx, cy, cyaw, target_speed):

        speed_profile = [target_speed] * len(cx)
        direction = 1.0  # forward

        # Set stop point
        for i in range(len(cx) - 1):
            dx = cx[i + 1] - cx[i]
            dy = cy[i + 1] - cy[i]

            move_direction = math.atan2(dy, dx)

            if dx != 0.0 and dy != 0.0:
                dangle = abs(self.pi_2_pi(move_direction - cyaw[i]))
                if dangle >= math.pi / 4.0:
                    direction = -1.0
                else:
                    direction = 1.0

            if direction != 1.0:
                speed_profile[i] = - target_speed
            else:
                speed_profile[i] = target_speed

        speed_profile[-1] = 0.0

        return speed_profile

    def smooth_yaw(self, yaw):

        for i in range(len(yaw) - 1):
            dyaw = yaw[i + 1] - yaw[i]

            while dyaw >= math.pi / 2.0:
                yaw[i + 1] -= math.pi * 2.0
                dyaw = yaw[i + 1] - yaw[i]

            while dyaw <= -math.pi / 2.0:
                yaw[i + 1] += math.pi * 2.0
                dyaw = yaw[i + 1] - yaw[i]

        return yaw
    
    def get_straight_course3(self,dl):
        ax = [0.0, -10.0, -20.0, -40.0, -50.0, -60.0, -70.0]
        ay = [0.0, -1.0, 1.0, 0.0, -1.0, 1.0, 0.0]
        cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
            ax, ay, ds=dl)

        cyaw = [i - math.pi for i in cyaw]

        return cx, cy, cyaw, ck
        
    def get_switch_back_course(self, dl):
        ax = [0.0, 30.0, 6.0, 20.0, 35.0]
        ay = [0.0, 0.0, 20.0, 35.0, 20.0]
        cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
            ax, ay, ds=dl)
        ax = [35.0, 10.0, 0.0, 0.0]
        ay = [20.0, 30.0, 5.0, 0.0]
        cx2, cy2, cyaw2, ck2, s2 = cubic_spline_planner.calc_spline_course(
            ax, ay, ds=dl)
        cyaw2 = [i - math.pi for i in cyaw2]
        cx.extend(cx2)
        cy.extend(cy2)
        cyaw.extend(cyaw2)
        ck.extend(ck2)

        return cx, cy, cyaw, ck
    
    def timer_callback(self):
        print(__file__ + " start!!")
        start = time.time()

        dl = 1.0  # course tick
        # cx, cy, cyaw, ck = get_straight_course(dl)
        # cx, cy, cyaw, ck = get_straight_course2(dl)
        # cx, cy, cyaw, ck = get_straight_course3(dl)
        # cx, cy, cyaw, ck = get_forward_course(dl)
        cx, cy, cyaw, ck = self.get_switch_back_course(dl)

        sp = self.calc_speed_profile(cx, cy, cyaw, self.TARGET_SPEED)

        t, x, y, yaw, v, d, a = self.do_simulation(cx, cy, cyaw, ck, sp, dl, self.state)

        elapsed_time = time.time() - start
        print(f"calc time:{elapsed_time:.6f} [sec]")

        # Publish velocity command
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = v * math.tan(yaw) / self.WB
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = MPCNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
