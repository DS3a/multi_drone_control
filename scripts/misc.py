#!/usr/bin/env python3
import math



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


def hello():
    print("hello world")

# Converting thrust vector to Thrust, roll pitch and yaw

# f_t = np.linalg.norm(a)  # this is the thrust force requried
# if f_t > 7.3:
#     f_t = 7.3
# a_cap = (a / np.linalg.norm(a)).reshape(3)  # the direction vector of a
# f_Z = np.array([0, 0, 1])
# v = np.cross(f_Z.reshape(3), a_cap.reshape(3))  # f_z x a
# c = np.dot(f_Z, a_cap)
# vx = np.cross(np.eye(3), v)

# R = np.eye(3) + vx + np.dot(vx, vx)*(1/(1+c))

# r = Rotation.from_matrix(R)
# angles = r.as_euler("zyx", degrees=False)

# cmd_msg = attitude_thrust_cmd()
# cmd_msg.thrust.data = f_t
# cmd_msg.yaw.data = angles[0]
# cmd_msg.pitch.data = angles[1]
# cmd_msg.roll.data = angles[2]
# print(f"the solution is {f_t, angles} at time {int(time_now)}")
