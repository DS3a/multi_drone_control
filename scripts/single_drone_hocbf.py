#!/usr/bin/env python3

import rospy
import sympy as sp
import numpy as np
import squaternion as squat
from qpsolvers import Problem, solve_problem
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
v_z = sp.symbols("v_z")

theta, phi, psi = sp.symbols("theta phi psi")
""" orientation as euler angles in
orientation is described in the roll-pitch-yaw convention;
theta = roll; phi = pitch, psi = yaw """


X = sp.Matrix([
    [x],
    [y],
    [z],
    [v_z],
    [theta],
    [phi],
    [psi],
])

# Input of the system
f_T = sp.symbols("f_T")
r, p = sp.symbols("r p")
""" r = roll rate in the body frame
p = pitch rate in the body frame
yawrate is zero so we won't consider it """

U = sp.Matrix([
    [f_T],
    [r],
    [p]
])

# drift function f
f = sp.Matrix([
    [v_z*((sp.cos(psi)*sp.sin(phi)*sp.cos(theta) - sp.sin(psi)*sp.cos(theta)))],
    [v_z*(sp.sin(psi)*sp.sin(phi)*sp.cos(theta) + sp.cos(psi)*sp.cos(theta))],
    [v_z*(sp.cos(phi)*sp.cos(theta))],
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
    [1/m_T, 0, 0],
    [0, sp.cos(psi)/sp.cos(phi), - sp.sin(psi)/sp.cos(phi)],
    [0, sp.sin(psi)*sp.tan(phi), sp.cos(psi)],
    [0, -sp.tan(phi)*sp.cos(psi), sp.tan(phi)*sp.sin(psi)],
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


dist = (sp.Matrix([x, y, z]) - sp.Matrix([-2, -2, -2]))
b1 = (50.1 - (50/14)*t) - dist.dot(dist)
b2 = (347.93*sp.exp(-0.418*t) + 2) - dist.dot(dist)
b = -sp.ln(sp.exp(b1) + sp.exp(b2))
b = b1

# Variables to keep track of states
pos_x = None
pos_y = None
pos_z = None
vel_total = None
roll = None
pitch = None
yaw = None


def odom_callback(data):
    global pos_x, pos_y, pos_z, vel_total, roll, pitch, yaw
    pos_x = data.pose.pose.position.x
    pos_y = data.pose.pose.position.y
    pos_z = data.pose.pose.position.z
    vel_total = np.sqrt((data.twist.twist.linear.x ** 2) +
                        (data.twist.twist.linear.y ** 2) +
                        (data.twist.twist.linear.z ** 2))
    orientation = squat.Quaternion(w=data.pose.pose.orientation.w,
                                   x=data.pose.pose.orientation.x,
                                   y=data.pose.pose.orientation.y,
                                   z=data.pose.pose.orientation.z)
    [roll, pitch, yaw] = orientation.to_euler(degrees=False)

    return


def main():
    global pos_x, pos_y, pos_z, vel_total, roll, pitch, yaw
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
    q = np.array([0., 0., 0.], dtype=float).dot(M).reshape((3,))
    A = np.array([0., 0., 0.], dtype=float)
    B = np.array([0.], dtype=float)
    lb = np.array([3, -0.1, -0.1], dtype=float)
    ub = np.array([20, 0.1, 0.1], dtype=float)
    start = time.time()

    while not rospy.is_shutdown():
        if not taken_off:
            print("taking off")
            take_off_msg = rate_thrust_cmd()
            take_off_msg.thrust.data = 7.9
            take_off_msg.roll_rate.data = 0.0
            take_off_msg.pitch_rate.data = 0.0
            pub.publish(take_off_msg)
            if pos_z >= 1.00:
                take_off_msg.thrust.data = 4.7
                pub.publish(take_off_msg)
                taken_off = True
            rate.sleep()
            continue

        time_now = time.time() - start
        lhs = - lie_derivative(lie_derivative(b, f), g*U).subs(
            {x: pos_x, y: pos_y, z: pos_z, v_z: vel_total, theta: roll,
             phi: pitch, psi: yaw, t: time_now}
        ).evalf()

        rhs = (Lf(b, 2) + sp.diff(b, t, 2) + 2*b*Lf(b) + 2*b*sp.diff(b, t) +
               Lf(b)**2 + sp.diff(b, t)**2 + b**4 + 2*Lf(b)*sp.diff(b, t) +
               2*(b**2)*Lf(b) + 2*(b**2)*sp.diff(b, t)).subs(
                   {x: pos_x, y: pos_y, z: pos_z, v_z: vel_total,
                    theta: roll, phi: pitch, psi: yaw, t: time_now}
               ).evalf()
        G = np.array([sp.Poly(lhs, f_T).coeffs()[0], sp.Poly(lhs, r).coeffs()[0], sp.Poly(lhs, p).coeffs()[0]], dtype=float)
        h = np.array([rhs], dtype=float)
        problem = Problem(P, q, G, h, A, B, lb, ub)
        solution = solve_problem(problem, solver="osqp")
        cmd_msg = rate_thrust_cmd()
        print(f"the solution is {solution.x}")
        cmd_msg.thrust.data = solution.x[0]
        cmd_msg.roll_rate.data = solution.x[1]
        cmd_msg.pitch_rate.data = solution.x[2]
        pub.publish(cmd_msg)

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
