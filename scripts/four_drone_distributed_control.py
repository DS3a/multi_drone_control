#!/usr/bin/env python3

import rospy
import numpy as np
from scipy.spatial.transform import Rotation

from multi_drone_control.msg import attitude_thrust_cmd
from geometry_msgs.msg import Vector3


required_thrust_vector = np.array([0, 0, 0])


def thrust_vector_callback(data):
    # Publish the received thrust vector to 4 different topics
    global required_thrust_vector
    required_thrust_vector = np.array([data.x, data.y, data.z])


if __name__ == '__main__':
    # Initialize ROS node
    rospy.init_node('thrust_vector_subscriber', anonymous=True)

    # Subscribe to the "/payload/thrust_vector" topic
    rospy.Subscriber("/payload/thrust_vector", Vector3, thrust_vector_callback)

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
        f_t = np.linalg.norm(required_thrust_vector)
        fr_cap = required_thrust_vector / f_t
        f_Z = np.array([0, 0, 1])
        v = np.cross(f_Z, fr_cap)
        c = np.dot(f_Z, fr_cap)
        vx = np.cross(np.eye(3), v)
        R = np.eye(3) + vx + np.dot(vx, vx)*(1/(1+c))

        r = Rotation.from_matrix(R)
        angles = r.as_euler("zyx", degrees=False)
        cmd_msg = attitude_thrust_cmd()
        cmd_msg.thrust.data = f_t
        cmd_msg.yaw.data = angles[0]
        cmd_msg.pitch.data = angles[1]
        cmd_msg.roll.data = angles[2]

        hb_0.publish(cmd_msg)
        hb_1.publish(cmd_msg)
        hb_2.publish(cmd_msg)
        hb_3.publish(cmd_msg)

        rate.sleep()
