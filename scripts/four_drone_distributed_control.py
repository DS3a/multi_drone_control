#!/usr/bin/env python3

import rospy
import numpy as np
from scipy.spatial.transform import Rotation

from multi_drone_control.msg import attitude_thrust_cmd
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry


required_thrust_vector = np.array([0, 0, 0])

payload_roll = 0.0
payload_pitch = 0.0

KP = 18

PITCH_KP = KP
ROLL_KP = KP


def payload_odom_callback(data):
    global payload_roll, payload_pitch

    r_quat = data.pose.pose.orientation
    r = Rotation.from_quat([r_quat.x, r_quat.y, r_quat.z, r_quat.w])
    angles = r.as_euler("zyx", degrees=False)
    payload_roll = angles[2]
    payload_pitch = angles[1]
    # print(payload_roll, payload_pitch)


def thrust_vector_callback(data):
    # Publish the received thrust vector to 4 different topics
    global required_thrust_vector
    required_thrust_vector = np.array([data.x, data.y, data.z])


def get_angles_from_thrust(thrust_vec):
    # f_t = np.linalg.norm(thrust_vec)
    fr_cap = thrust_vec / np.linalg.norm(thrust_vec)
    f_Z = np.array([0, 0, 1])
    v = np.cross(f_Z, fr_cap)
    c = np.dot(f_Z, fr_cap)
    vx = np.cross(np.eye(3), v)
    R = np.eye(3) + vx + np.dot(vx, vx)*(1/(1+c))

    r = Rotation.from_matrix(R)
    angles = r.as_euler("zyx", degrees=False)
    return angles


def publish_cmd(hb, thrust_vector):
    f_t = np.linalg.norm(thrust_vector)
    angles = get_angles_from_thrust(thrust_vector)
    angles[np.isnan(angles)] = 0
    print(angles)
    cmd_msg = attitude_thrust_cmd()
    cmd_msg.thrust.data = f_t
    cmd_msg.yaw.data = angles[0]
    cmd_msg.pitch.data = angles[1]
    cmd_msg.roll.data = angles[2]

    hb.publish(cmd_msg)


if __name__ == '__main__':
    # Initialize ROS node
    rospy.init_node('thrust_vector_subscriber', anonymous=True)

    # Subscribe to the "/payload/thrust_vector" topic
    rospy.Subscriber("/payload/thrust_vector", Vector3, thrust_vector_callback)

    rospy.Subscriber("/payload/ground_truth/odometry", Odometry,
                     payload_odom_callback)

    # Publishers for the 4 different topics
    hb_0 = rospy.Publisher("/hummingbird_0/drone_cmd",
                           attitude_thrust_cmd, queue_size=2)
    hb_1 = rospy.Publisher("/hummingbird_1/drone_cmd",
                           attitude_thrust_cmd, queue_size=2)
    hb_2 = rospy.Publisher("/hummingbird_2/drone_cmd",
                           attitude_thrust_cmd, queue_size=2)
    hb_3 = rospy.Publisher("/hummingbird_3/drone_cmd",
                           attitude_thrust_cmd, queue_size=2)

    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        hb_0_z = 0
        hb_1_z = 0
        hb_2_z = 0
        hb_3_z = 0

        hb_0_z = -(PITCH_KP * payload_pitch) - (ROLL_KP * payload_roll)
        hb_1_z = -(PITCH_KP * payload_pitch) + (ROLL_KP * payload_roll)
        hb_2_z = +(PITCH_KP * payload_pitch) + (ROLL_KP * payload_roll)
        hb_3_z = +(PITCH_KP * payload_pitch) - (ROLL_KP * payload_roll)

        # f_t = np.linalg.norm(required_thrust_vector)
        # angles = get_angles_from_thrust(required_thrust_vector)
        # angles[np.isnan(angles)] = 0
        # print(angles)
        # cmd_msg = attitude_thrust_cmd()
        # cmd_msg.thrust.data = f_t
        # cmd_msg.yaw.data = angles[0]
        # cmd_msg.pitch.data = angles[1]
        # cmd_msg.roll.data = angles[2]

        # hb_0.publish(cmd_msg)
        # hb_1.publish(cmd_msg)
        # hb_2.publish(cmd_msg)
        # hb_3.publish(cmd_msg)

        publish_cmd(hb_0, required_thrust_vector + np.array([0, 0, hb_0_z]))
        publish_cmd(hb_1, required_thrust_vector + np.array([0, 0, hb_1_z]))
        publish_cmd(hb_2, required_thrust_vector + np.array([0, 0, hb_2_z]))
        publish_cmd(hb_3, required_thrust_vector + np.array([0, 0, hb_3_z]))

        rate.sleep()
