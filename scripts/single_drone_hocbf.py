#!/usr/bin/env python3

import rospy
import sympy as sp
import numpy as np
import math
from cvxopt import solvers, matrix
import time

from nav_msgs.msg import Odometry
from multi_drone_control.msg import rate_thrust_cmd

# Constants
mass_drone = 0.68
mass_rotor = 0.009

m_T = mass_drone+(4*mass_rotor)  # total mass of the system
I_xx = 0.007  # moment of inertia of the quadcopter along the x axis
I_yy = 0.007  # moment of inertia of the quadcopter along the y axis

# State space of the system
x, y, z = sp.symbols("x y z")  # positions
v_x, v_y, v_z = sp.symbols("v_x v_y v_z")

theta, phi, psi = sp.symbols("theta phi psi")
""" orientation as euler angles in
orientation is described in the roll-pitch-yaw convention;
theta = roll; phi = pitch, psi = yaw """

f_T = sp.symbols("f_T")

X = sp.Matrix([
    [x],
    [y],
    [z],
    [v_x],
    [v_y],
    [v_z],
    [theta],
    [phi],
    [psi],
    [f_T],
])

# Input of the system
fdot_T = sp.symbols("fdot_T")
r, p = sp.symbols("r p")
""" r = roll rate in the body frame
p = pitch rate in the body frame
yawrate is zero so we won't consider it """

U = sp.Matrix([
    [fdot_T],
    [r],
    [p]
])

# drift function f
f = sp.Matrix([
    [v_x],
    [v_y],
    [v_z],
    [(f_T - 9.81)*((sp.cos(psi)*sp.sin(phi)*sp.cos(theta) + sp.sin(psi)*sp.sin(theta)))/m_T],
    [(f_T - 9.81)*(sp.sin(psi)*sp.sin(phi)*sp.cos(theta) - sp.cos(psi)*sp.sin(theta))/m_T],
    [(f_T - 9.81)*(sp.cos(phi)*sp.cos(theta))/m_T],
    [0],
    [0],
    [0],
    [0]
])

# input function g
g = sp.Matrix([
    [0, 0, 0],
    [0, 0, 0],
    [0, 0, 0],
    # [((sp.cos(psi)*sp.sin(phi)*sp.cos(theta) + sp.sin(psi)*sp.sin(theta)))/m_T, 0, 0],
    # [(sp.sin(psi)*sp.sin(phi)*sp.cos(theta) - sp.cos(psi)*sp.sin(theta))/m_T, 0, 0],
    # [(sp.cos(phi)*sp.cos(theta))/m_T, 0, 0],
    [0, 0, 0],
    [0, 0, 0],
    [0, 0, 0],
    [0, 0, sp.sin(theta)/sp.cos(phi)],
    [0, 0, sp.cos(theta)],
    [0, 1, sp.tan(phi)*sp.sin(theta)],
    [1, 0, 0]
])

t = sp.symbols("t")  # symbol for time


def lie_derivative(barrier_fn, vector_field, n=1):
    fn_ = barrier_fn
    for n in range(0, n):
        grad = sp.diff(fn_, X)
        fn_ = grad.dot(vector_field)

    return fn_


def Lf(barrier_fn, order=1):
    return lie_derivative(barrier_fn, f, order)


def quaternion_to_euler_angles(q):
    angles = [0, 0, 0]

    # roll (x-axis rotation)
    sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
    angles[0] = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = math.sqrt(1 + 2 * (q.w * q.y - q.x * q.z))
    cosp = math.sqrt(1 - 2 * (q.w * q.y - q.x * q.z))
    angles[1] = 2 * math.atan2(sinp, cosp) - math.pi / 2

    # yaw (z-axis rotation)
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    angles[2] = math.atan2(siny_cosp, cosy_cosp)

    return angles


dist = (sp.Matrix([x, y, z]) - sp.Matrix([2, 0, 1]))
b1 = (50.1 - (50/14)*t) - dist.dot(dist)
b1 = 8*sp.exp(-t/3) + 0.1 - dist.dot(dist)
b2 = (347.93*sp.exp(-0.418*t) + 2) - dist.dot(dist)
b = -sp.ln(sp.exp(b1) + sp.exp(b2))
b = b1

# Variables to keep track of states
pos_x = None
pos_y = None
pos_z = None
vel_x = None
vel_y = None
vel_z = None
roll = None
pitch = None
yaw = None
f_T_real = None


def odom_callback(data):
    global pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, roll, pitch, yaw
    pos_x = data.pose.pose.position.x
    pos_y = data.pose.pose.position.y
    pos_z = data.pose.pose.position.z
    vel_x = data.twist.twist.linear.x
    vel_y = data.twist.twist.linear.y
    vel_z = data.twist.twist.linear.z
    # orientation = squat.Quaternion(w=data.pose.pose.orientation.w,
    #                                x=data.pose.pose.orientation.x,
    #                                y=data.pose.pose.orientation.y,
    #                                z=data.pose.pose.orientation.z)
    q = data.pose.pose.orientation
    [roll, pitch, yaw] = quaternion_to_euler_angles(q)

    return


def eval_expr(e, t_actual):
    return e.subs({x: pos_x, y: pos_y, z: pos_z,
                   v_x: vel_x, v_y: vel_y,
                   v_z: vel_z, psi: yaw,
                   phi: pitch, theta: roll,
                   f_T: f_T_real, t: t_actual})


def main():
    global pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, roll, pitch, yaw, f_T_real
    pub = rospy.Publisher('/hummingbird0/drone_cmd', rate_thrust_cmd,
                          queue_size=10)
    _ = rospy.Subscriber("/hummingbird0/ground_truth/odometry",
                         Odometry, odom_callback)
    rospy.init_node('drone_high_level_hocbf', anonymous=False)
    rate = rospy.Rate(50)  # 50hz

    taken_off = False
    M = np.array([[1.0, 2.0, 0.0], [-8.0, 3.0, 2.0], [0.0, 1.0, 1.0]],
                 dtype=float)
    P = M.T @ M  # positive semi definite matrix
    P = 2*np.eye(3)
    Q = np.array([0., 0., 0.], dtype=float).dot(M).reshape((3,))
    start = time.time()

    lhs_fdot_T = lie_derivative(lie_derivative(b, f, 2), g.col(0))
    lhs_p = lie_derivative(lie_derivative(b, f, 2), g.col(1))
    lhs_q = lie_derivative(lie_derivative(b, f, 2), g.col(2))

    alpha = 0.7
    # rhs = (Lf(b, 2) + sp.diff(b, t, 2) + alpha*sp.diff(b, t) + (alpha**2)*b + Lf(alpha*b) + sp.diff(alpha*b, t))

    rhs = lie_derivative(b, f, 3) + alpha * sp.diff(b, t, 2) + 2*(alpha**2)*sp.diff(b, t) + b*(alpha**3) + lie_derivative(alpha*sp.diff(b, t) + alpha*alpha*b, f) + lie_derivative(alpha*b, f, 2)

    while not rospy.is_shutdown():
        if not taken_off:
            print("taking off")
            take_off_msg = rate_thrust_cmd()
            take_off_msg.thrust.data = 7.9
            take_off_msg.roll_rate.data = 0.0
            take_off_msg.pitch_rate.data = 0.0
            pub.publish(take_off_msg)
            if pos_z >= 1.00:
                f_T_real = 5.5
                take_off_msg.thrust.data = f_T_real
                pub.publish(take_off_msg)
                taken_off = True
            rate.sleep()
            continue

        time_now = time.time() - start
        # lhs = - lie_derivative(lie_derivative(b, f), g*U).subs(
        #     {x: pos_x, y: pos_y, z: pos_z, v_z: vel_total, theta: roll,
        #      phi: pitch, psi: yaw, t: time_now}
        # ).evalf()

        # rhs = (Lf(b, 2) + sp.diff(b, t, 2) + 2*b*Lf(b) + 2*b*sp.diff(b, t) # +
        # Lf(b)**2 + sp.diff(b, t)**2 + b**4 + 2*Lf(b)*sp.diff(b, t) +
        # 2*(b**2)*Lf(b) + 2*(b**2)*sp.diff(b, t)).subs(
        #     {x: pos_x, y: pos_y, z: pos_z, v_z: vel_total,
        #      theta: roll, phi: pitch, psi: yaw, t: time_now}
        # ).evalf()
        # G = np.array([sp.Poly(lhs, f_T).coeffs()[0], sp.Poly(lhs, r).coeffs()[0], sp.Poly(lhs, p).coeffs()[0]], dtype=float)
        G = np.array([[eval_expr(lhs_fdot_T, time_now), eval_expr(lhs_p, time_now), eval_expr(lhs_q, time_now)],
                      [-1, 0, 0],
                      [1, 0, 0],], dtype=float)
        h = np.array([[eval_expr(rhs, time_now)],
                      [(7.0 - f_T_real)],
                      [(f_T_real)],], dtype=float)
        # problem = Problem(P, q, G, h, A, B, lb, ub)
        # solution = solve_problem(problem, solver="osqp")
        sol = solvers.qp(matrix(P), matrix(Q), matrix(G), matrix(h))

        fdot = sol['x'][0]
        f_T_real += float(fdot)/50.0
        p_inpt = sol['x'][1]
        q_inpt = sol['x'][2]
        max_ang_vel = 0.1

        if p_inpt > max_ang_vel:
            p_inpt = max_ang_vel
        elif p_inpt < -max_ang_vel:
            p_inpt = -max_ang_vel

        if q_inpt > max_ang_vel:
            q_inpt = max_ang_vel
        elif q_inpt < -max_ang_vel:
            q_inpt = -max_ang_vel

        cmd_msg = rate_thrust_cmd()
        print(f"the solution is {fdot, f_T_real, p_inpt, q_inpt}")
        cmd_msg.thrust.data = f_T_real
        cmd_msg.roll_rate.data = p_inpt
        cmd_msg.pitch_rate.data = q_inpt
        pub.publish(cmd_msg)

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
