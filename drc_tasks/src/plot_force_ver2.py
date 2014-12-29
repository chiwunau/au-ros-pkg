#!/usr/bin/env python

import rospy
import time
import numpy as np
import math
import matplotlib.pyplot as plt
from std_msgs.msg import Float64
from geometry_msgs.msg import WrenchStamped

f_sum= []
f_x = []
f_y = []
f_z = []
t = []
st = 0

def callback(msg):
    global st
    if st == 0:
        st = msg.header.stamp
    f_x.append(msg.wrench.force.x)
    f_y.append(msg.wrench.force.y)
    f_z.append(msg.wrench.force.z)
    f_sum.append(math.sqrt(pow(msg.wrench.force.x,2) + pow(msg.wrench.force.y,2) +pow(msg.wrench.force.z,2)))
    t.append(msg.header.stamp.to_sec() - st.to_sec())

def listener():
    rospy.init_node('python_plot', anonymous=True)
    # rospy.Subscriber("righ", Float64, callback) 
    rospy.Subscriber("left_endeffector/wrench", WrenchStamped, callback)
    rospy.spin()

def draw_plot():
   # plt.plot(t,f_x,'r', t,f_y,'g', t, f_z, 'b', t, f_sum, 'k')
    plt.plot(t, f_x, color="red", linewidth=1.0, linestyle="--", label="Fx")
    plt.plot(t, f_y, color="green", linewidth=1.0, linestyle="--", label="Fy")
    plt.plot(t, f_z, color="blue", linewidth=1.0, linestyle="--", label="Fz")
    plt.plot(t, f_sum, color="black", linewidth=1.0, linestyle="-", label="|F|")
    plt.ylabel('F[N]')
    plt.xlabel('t[sec]')
    plt.legend(loc='upper right')
    plt.show()

if __name__ == '__main__':
    listener()
