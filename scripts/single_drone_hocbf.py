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
mass_drone = 0.825
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
    [1/m_T, 0, 0],
    [0, 1/m_T, 0],
    [0, 0, 1/m_T],
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
dist = ((sp.Matrix([p_x, p_y, p_z]) - sp.Matrix([10, 10, 1])))
# b1 = (50.1 - (50/14)*t) - sp.sqrt(dist.dot(dist))
b1 = 40*sp.exp(-t/10) + 0.1 - sp.sqrt(dist.dot(dist))
b2 = (347.93*sp.exp(-t/2.39) + 2) - sp.sqrt(dist.dot(dist))
b2 = (347.93*sp.exp(-t/15) + 2) - sp.sqrt(dist.dot(dist))

dist2 = (sp.Matrix([p_x, p_y, p_z]) - sp.Matrix([-1, -2, 3]))
# b2 = (347.93*sp.exp(-t/4) + 2) - sp.sqrt(dist.dot(dist2))
b3 = (2 + 400*sp.exp(-t/8)) - sp.sqrt(dist2.dot(dist2))
b = -sp.ln(sp.exp(-b1) + sp.exp(-b2) + sp.exp(-b3))
b = -sp.ln(sp.exp(-b1) + sp.exp(-b2))
# b = b1

# barrier functions to enforce velocity constraints
nu_max = 0.8  # max amplitude of pitch or roll can be 0.3 radians
# b_pitch_max = nu_max - sp.atan(v_x / v_z)
# b_pitch_min = nu_max + sp.atan(v_x / v_z)

b_pitch_max = nu_max - v_x
b_pitch_min = nu_max + v_x


# b_roll_max = nu_max - sp.atan(v_y / v_z)
# b_roll_min = nu_max + sp.atan(v_y / v_z)


b_roll_max = nu_max - v_y
b_roll_min = nu_max + v_y

v_max = 15.5
v_min = -5.0
b_v_z_max = v_max - v_z
b_v_z_min = -v_min + v_z


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
    received_initial_solution = False
    a = 0

    P = np.eye(3)
    Q = np.array([0., 0., 0.], dtype=float).reshape((3,))

    alpha = 0.99
    lhs_ax = -lie_derivative(lie_derivative(b, f, 1), g.col(0))
    lhs_ay = -lie_derivative(lie_derivative(b, f, 1), g.col(1))
    lhs_az = -lie_derivative(lie_derivative(b, f, 1), g.col(2))
    # rhs = Lf(b, 2) + sp.diff(b, t, 2) + alpha*sp.diff(b, t) + alpha*alpha*b + Lf(alpha*b) + sp.diff(alpha*b, t)
    rhs = Lf(b, 2) + sp.diff(b, t, 2) + 2*alpha*Lf(b) + alpha*sp.diff(b, t) + (alpha**2)*b
    # rhs = Lf(b, 2) + sp.diff(b, t, 2) + 2*b*Lf(b) + 2*b*sp.diff(b, t) + (Lf(b))**2 + b**4 + 2*(b**2)*Lf(b)
    # rhs = Lf(b, 2) + sp.diff(b, t, 2) + sp.diff(b, t)**2 + (alpha*b)**2 + Lf(alpha*b) + sp.diff(alpha*b, t)
    # rhs = Lf(b, 2) + sp.diff(b, t, 2) + sp.diff(b, t)**2 + b**4 + Lf(b**2) + sp.diff(b**2, t)
    # rhs = Lf(b, 2) + sp.diff(b, t, 2) + 2*b*Lf(b) + 2*b*sp.diff(b, t) + Lf(b)**2 + sp.diff(b, t)**2 + b**4 + 2*Lf(b)*sp.diff(b, t) + 2*(b**2)*Lf(b) + 2*(b**2)*sp.diff(b, t)
    # rhs = Lf(b, 2) + 2*b*Lf(b) + Lf(b)**2 + 2*(b**2)*Lf(b)+b**4

    alpha = 0.3
    pow = 1
    lhs_b_pitch_min_ax = -lie_derivative(b_pitch_min, g.col(0))
    lhs_b_pitch_min_ay = 0
    lhs_b_pitch_min_az = -lie_derivative(b_pitch_min, g.col(2))
    # rhs_b_pitch_min = lie_derivative(b_pitch_min, f) + alpha*b_pitch_min
    rhs_b_pitch_min = lie_derivative(b_pitch_min, f) + alpha*b_pitch_min**pow

    lhs_b_pitch_max_ax = -lie_derivative(b_pitch_max, g.col(0))
    lhs_b_pitch_max_ay = 0
    lhs_b_pitch_max_az = -lie_derivative(b_pitch_max, g.col(2))
    # rhs_b_pitch_max = lie_derivative(b_pitch_max, f) + alpha*b_pitch_max
    rhs_b_pitch_max = lie_derivative(b_pitch_max, f) + alpha*b_pitch_max**pow

    lhs_b_roll_min_ax = 0
    lhs_b_roll_min_ay = -lie_derivative(b_roll_min, g.col(1))
    lhs_b_roll_min_az = -lie_derivative(b_roll_min, g.col(2))
    # rhs_b_roll_min = lie_derivative(b_roll_min, f) + alpha*b_roll_min
    rhs_b_roll_min = lie_derivative(b_roll_min, f) + alpha*b_roll_min**pow

    lhs_b_roll_max_ax = 0
    lhs_b_roll_max_ay = -lie_derivative(b_roll_max, g.col(1))
    lhs_b_roll_max_az = -lie_derivative(b_roll_max, g.col(2))
    # rhs_b_roll_max = lie_derivative(b_roll_max, f) + alpha*b_roll_max
    rhs_b_roll_max = lie_derivative(b_roll_max, f) + alpha*b_roll_max**pow

    lhs_b_v_z_max_az = -lie_derivative(b_v_z_max, g.col(2))
    # rhs_b_v_z_max = lie_derivative(b_v_z_max, f) + alpha * b_v_z_max
    rhs_b_v_z_max = lie_derivative(b_v_z_max, f) + alpha*b_v_z_max**pow

    lhs_b_v_z_min_az = -lie_derivative(b_v_z_min, g.col(2))
    # rhs_b_v_z_min = lie_derivative(b_v_z_min, f) + alpha * b_v_z_min
    rhs_b_v_z_min = lie_derivative(b_v_z_min, f) + alpha*b_v_z_min**pow

    start = time.time()
    while not rospy.is_shutdown():
        if not taken_off:
            print("taking off")
            take_off_msg = attitude_thrust_cmd()
            take_off_msg.thrust.data = 7.5
            take_off_msg.roll.data = 0.0
            take_off_msg.pitch.data = 0.0
            take_off_msg.yaw.data = 0.0
            pub.publish(take_off_msg)
            if pos_z >= 0.25:
                taken_off = True
                start = time.time()
            rate.sleep()
            continue

        time_now = time.time() - start
        G = np.array([[eval_expr(lhs_ax, time_now), eval_expr(lhs_ay, time_now), eval_expr(lhs_az, time_now)],  #  ], dtype=float) # HOCBF constraints
                      [eval_expr(lhs_b_roll_max_ax), eval_expr(lhs_b_roll_max_ay), eval_expr(lhs_b_roll_max_az)],  # roll angle max constraint
                      [eval_expr(lhs_b_roll_min_ax), eval_expr(lhs_b_roll_min_ay), eval_expr(lhs_b_roll_min_az)],  # roll angle min constraint
                      [eval_expr(lhs_b_pitch_max_ax), eval_expr(lhs_b_pitch_max_ay), eval_expr(lhs_b_pitch_max_az)], #], dtype=float)  # pitch angle max constraint
                      [eval_expr(lhs_b_pitch_min_ax), eval_expr(lhs_b_pitch_min_ay), eval_expr(lhs_b_pitch_min_az)], #], dtype=float)  # pitch angle max constraint
                      [0, 0, eval_expr(lhs_b_v_z_max_az)],  # v_z_max constraint
                      [0, 0, eval_expr(lhs_b_v_z_min_az)]], dtype=float)  # v_z min constraint

        # print("rhs of hocbf is ", eval_expr(rhs, time_now))

        delta = 0.5
        h = np.array([[eval_expr(rhs, time_now)+2*delta],    #  ], dtype=float)  # HOCBF constraint
                      [eval_expr(rhs_b_roll_max)+delta],  # roll angle constraint
                      [eval_expr(rhs_b_roll_min)+delta],  # roll angle constraint
                      [eval_expr(rhs_b_pitch_max)+delta],  # ], dtype=float)  # pitch angle constraint
                      [eval_expr(rhs_b_pitch_min)+delta],  # ], dtype=float)  # pitch angle constraint
                      [eval_expr(rhs_b_v_z_max)],  # v_z_max constraint
                      [eval_expr(rhs_b_v_z_min)]], dtype=float)  # v_z_min constraint

        solvers.options['show_progress'] = False
        sol = solvers.qp(matrix(P), matrix(Q), matrix(G), matrix(h))

        lpf_const = 0.0
        if received_initial_solution:
            a = (lpf_const * a) + (1 - lpf_const)*np.array(sol['x'])
        else:
            a = np.array(sol['x'])  # the acceleration vector

        if a[2] < 0:
            a[2] = 0
        # elif a[2] > 1:
        #     a[2] = 1

        # if a[0] < -1:
        #     a[0] = -1
        # elif a[0] > 1:
        #     a[0] = 1

        # if a[1] < -1:
        #     a[1] = -1
        # elif a[1] > 1:
        #     a[1] = 1



        # a = a*m_T  # convert this to forces
        f_t = np.linalg.norm(a)  # this is the thrust force requried
        if f_t > 7.3:
            f_t = 7.3
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
        print(f"the solution is {f_t, angles} at time {int(time_now)}")

        pub.publish(cmd_msg)

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
