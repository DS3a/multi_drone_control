#!/usr/bin/env python3

import rospy
import sympy as sp
import numpy as np
from cvxopt import solvers, matrix
import time

from nav_msgs.msg import Odometry
from multi_drone_control.msg import attitude_thrust_cmd
from scipy.spatial.transform import Rotation

# Constants
mass_drone = 0.68
mass_rotor = 0.009

m_T = mass_drone + 4*mass_rotor

### State space of the system

p_x, p_y, p_z = sp.symbols("p_x p_y p_z")  # positions
v_x, v_y, v_z = sp.symbols("v_x, v_y, v_z")

X = sp.Matrix([
    p_x,
    p_y,
    p_z,
    v_x,
    v_y,
    v_z,
])

### Input of the system

a_x, a_y, a_z = sp.symbols("a_x a_y a_z")

U = sp.Matrix([
    a_x,
    a_y,
    a_z,
])

### The drift matrix

f = sp.Matrix([
    v_x,
    v_y,
    v_z,
    0,
    0,
    -9.81,
])

### The input matrix

g = sp.Matrix([
    [0, 0, 0],
    [0, 0, 0],
    [0, 0, 0],
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1],
])


def lie_derivative(barrier_fn, vector_field, n=1):
    fn_ = barrier_fn
    for n in range(0, n):
        grad = sp.diff(fn_, X)
        fn_ = grad.dot(vector_field)

    return fn_


def Lf(barrier_fn, order=1):
    return lie_derivative(barrier_fn, f, order)


t = sp.symbols("t")  # symbol for time
dist = (sp.Matrix([p_x, p_y, p_z]) - sp.Matrix([-2, 2, 2]))
b1 = (50.1 - (50/14)*t) - dist.dot(dist)
b1 = 8*sp.exp(-t/3) + 0.1 - dist.dot(dist)
b2 = (347.93*sp.exp(-0.418*t) + 2) - dist.dot(dist)
b = -sp.ln(sp.exp(b1) + sp.exp(b2))
b = b1

# barrier functions to enforce velocity constraints
nu_max = 0.3  # max amplitude of pitch or roll can be 0.3 radians
b_pitch = (v_z * np.tan(nu_max))**2 - v_x**2
b_roll = (v_z * np.tan(nu_max))**2 - v_y**2

v_max = 1.5  # max v_z can be 0.5 this constraint trickles down to v_x and v_y which have to be lesser than some ratio of v_z at all times
b_vz = (v_max**2) - v_z**2

# Variables to keep track of states
pos_x = None
pos_y = None
pos_z = None
vel_x = None
vel_y = None
vel_z = None


def odom_callback(data):
    global pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, roll, pitch, yaw
    pos_x = data.pose.pose.position.x
    pos_y = data.pose.pose.position.y
    pos_z = data.pose.pose.position.z
    vel_x = data.twist.twist.linear.x
    vel_y = data.twist.twist.linear.y
    vel_z = data.twist.twist.linear.z

    return


def eval_expr(e, t_actual=0):
    if e == 0:
        return 0
    return e.subs({p_x: pos_x, p_y: pos_y, p_z: pos_z,
                   v_x: vel_x, v_y: vel_y,
                   v_z: vel_z, t: t_actual})


def main():
    global pos_x, pos_y, pos_z, vel_x, vel_y, vel_z
    pub = rospy.Publisher('/hummingbird0/drone_cmd', attitude_thrust_cmd,
                          queue_size=10)
    _ = rospy.Subscriber("/hummingbird0/ground_truth/odometry",
                         Odometry, odom_callback)
    rospy.init_node('drone_high_level_hocbf', anonymous=False)
    rate = rospy.Rate(50)  # 50hz

    taken_off = False
    M = np.array([[1.0, 2.0, 0.0], [-8.0, 3.0, 2.0], [0.0, 1.0, 1.0]],
                 dtype=float)
    P = M.T @ M  # positive semi definite matrix
    P = 0.5*np.eye(3)
    Q = np.array([0., 0., 0.], dtype=float).reshape((3,))
    start = time.time()

    alpha = 0.9
    lhs_ax = -lie_derivative(lie_derivative(b, f, 1), g.col(0))
    lhs_az = -lie_derivative(lie_derivative(b, f, 1), g.col(2))
    lhs_ay = -lie_derivative(lie_derivative(b, f, 1), g.col(1))
    rhs = Lf(b, 2) + sp.diff(b, t, 2) + alpha*sp.diff(b, t) + alpha*alpha*b + Lf(alpha*b) + sp.diff(alpha*b, t)

    lhs_b_pitch_ax = -lie_derivative(b_pitch, g.col(0))
    lhs_b_pitch_ay = 0
    lhs_b_pitch_az = -lie_derivative(b_pitch, g.col(2))
    rhs_b_pitch = lie_derivative(b_pitch, f) + alpha*b_pitch

    lhs_b_roll_ax = 0
    lhs_b_roll_ay = -lie_derivative(b_roll, g.col(1))
    lhs_b_roll_az = -lie_derivative(b_roll, g.col(2))
    rhs_b_roll = lie_derivative(b_roll, f) + alpha*b_roll

    lhs_b_vz_az = -lie_derivative(b_vz, g.col(2))
    rhs_b_vz = lie_derivative(b_vz, f) + alpha*b_vz

    while not rospy.is_shutdown():
        if not taken_off:
            print("taking off")
            take_off_msg = attitude_thrust_cmd()
            take_off_msg.thrust.data = 7.9
            take_off_msg.roll.data = 0.0
            take_off_msg.pitch.data = 0.0
            take_off_msg.yaw.data = 0.0
            pub.publish(take_off_msg)
            if pos_z >= 1.00:
                f_T_real = 5.5
                take_off_msg.thrust.data = f_T_real
                pub.publish(take_off_msg)
                taken_off = True
            rate.sleep()
            continue

        time_now = time.time() - start
        G = np.array([[eval_expr(lhs_ax, time_now), eval_expr(lhs_ay, time_now), eval_expr(lhs_az, time_now)],  # HOCBF constraints
                      [eval_expr(lhs_b_roll_ax), eval_expr(lhs_b_roll_ay), eval_expr(lhs_b_roll_az)],  # roll angle constraint
                      [eval_expr(lhs_b_pitch_ax), eval_expr(lhs_b_pitch_ay), eval_expr(lhs_b_pitch_az)], #], dtype=float)  # pitch angle constraint
                      # [0, 0, eval_expr(lhs_b_v_z_max_az)],  # v_z_max constraint
                      [0, 0, eval_expr(lhs_b_vz_az)]], dtype=float)  # v_z min constraint

        h = np.array([[eval_expr(rhs, time_now)],  # HOCBF constraint
                      [eval_expr(rhs_b_roll)],  # roll angle constraint
                      [eval_expr(rhs_b_pitch)], #], dtype=float)  # pitch angle constraint
                      # [eval_expr(rhs_b_v_z_max)],  # v_z_max constraint
                      [eval_expr(rhs_b_vz)]], dtype=float)  # v_z_min constraint

        sol = solvers.qp(matrix(P), matrix(Q), matrix(G), matrix(h))

        a = np.array(sol['x'])  # the acceleration vector
        a = a*m_T  # convert this to forces
        f_t = np.linalg.norm(a)  # this is the thrust force requried
        a_cap = (a / np.linalg.norm(a)).reshape(3)  # the direction vector of a
        f_Z = np.array([0, 0, 1])
        v = np.cross(f_Z.reshape(3), a_cap.reshape(3))  # f_z x a
        c = np.dot(f_Z, a_cap)
        vx = np.cross(np.eye(3), v)
        R = np.eye(3) + vx + np.dot(vx, vx)*(1/(1+c))

        r = Rotation.from_matrix(R)
        angles = r.as_euler("zyx", degrees=False)

        cmd_msg = attitude_thrust_cmd()
        cmd_msg.thrust.data = f_t
        cmd_msg.yaw.data = angles[0]
        cmd_msg.pitch.data = angles[1]
        cmd_msg.roll.data = angles[2]
        print(f"the solution is {f_t, angles}")

        pub.publish(cmd_msg)

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
