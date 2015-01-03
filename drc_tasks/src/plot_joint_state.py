#!/usr/bin/env python

import rospy
import time
import numpy as np
import math
import matplotlib.pyplot as plt
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

name = []
v0 = []
v1 = []
v2 = []
v3 = []
t = []
st = 0

def callback (msg):
    global st
    if st == 0:
        st = msg.header.stamp
        for x in msg.name:
            name.append(x);
    v0.append(msg.velocity[30]);
    v1.append(msg.velocity[31]);
    v2.append(msg.velocity[32]);
    v3.append(msg.velocity[33]);
    # v0.append(msg.position[30]);
    # v1.append(msg.position[31]);
    # v2.append(msg.position[32]);
    # v3.append(msg.position[33]);


    t.append(msg.header.stamp.to_sec() - st.to_sec())

def listener():
    rospy.spin()

def draw_plot():
    plt.plot(t, v0, color="red", linewidth=1.0, linestyle="-", label=name[30])
    plt.plot(t, v1, color="green", linewidth=1.0, linestyle="-", label=name[31])
    plt.plot(t, v2, color="blue", linewidth=1.0, linestyle="-", label=name[32])
    plt.plot(t, v3, color="yellow", linewidth=1.0, linestyle="-", label=name[33])

    plt.xlabel('t[sec]')
    plt.legend(loc='bottom right')
    plt.show()

if __name__ == '__main__':
    rospy.init_node('jointstate_draw', anonymous=True)
    rospy.Subscriber('joint_states', JointState, callback)
    listener()
