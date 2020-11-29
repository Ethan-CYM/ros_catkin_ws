#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function
import rtde_control
import rtde_receive

import time
import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
from math import pi
import numpy as np

from geometry_msgs.msg import *

rtde_c = rtde_control.RTDEControlInterface("192.168.0.104")
rtde_r = rtde_receive.RTDEReceiveInterface("192.168.0.104")

fext = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
firstFlag = True

def sensor(date):
    global fext
    f = date.wrench.force
    t = date.wrench.torque
    fext[0] = f.x
    fext[1] = f.y
    fext[2] = f.z
    fext[3] = t.x
    fext[4] = t.y
    fext[5] = t.z
    print(fext[2])

def main():
    global fext
    global firstFlag
    velocity = 0.5
    acceleration = 0.5
    dt = 1.0/500  # 2ms
    lookahead_time = 0.1
    gain = 100

    Q = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ]
    # Q1 = [90.0/180.0*pi, -54.71/180.0*pi, 72.57/180.0*pi, -107.97/180.0*pi, -90.35/180.0*pi, 90.21/180.0*pi]
    # Q2 = [90/180.0*pi, -50.55/180.0*pi, 75.98/180.0*pi, -114.51/180.0*pi, -90.06/180.0*pi, 90.36/180.0*pi]
    # Q2 = [89.66/180.0*pi, -49.74/180.0*pi, 78.05/180.0*pi, -118.16/180.0*pi, -89.41/180.0*pi, 90.12/180.0*pi]
    Q1 = [90.0/180.0*pi, -54.71/180.0*pi, 72.57/180.0*pi, -107.97/180.0*pi, -90.35/180.0*pi, 90.8/180.0*pi]
    Q2 = [90/180.0*pi, -50.55/180.0*pi, 75.98/180.0*pi, -114.51/180.0*pi, -90.06/180.0*pi, 90.0/180.0*pi]
    Q2 = [89.66/180.0*pi, -49.74/180.0*pi, 78.05/180.0*pi, -118.16/180.0*pi, -89.41/180.0*pi, 90.2/180.0*pi]
    Qd = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    Kp = +0.1
    Kf = +0.00
    fext0 = [0.0, 0.0, 0.0, 0.0, 0.0, 1.14]

    try:
        rospy.init_node("ur_move", anonymous=True, disable_signals=True)
        rospy.Subscriber("netft_data_B", WrenchStamped, sensor)

        rtde_c.moveJ(Q1)

        time.sleep(1)

        while not rospy.is_shutdown():
            Q = rtde_r.getActualQ()

            Qd[0] = (Q2[0]-Q[0])*Kp
            Qd[1] = (Q2[1]-Q[1])*Kp
            Qd[2] = (Q2[2]-Q[2])*Kp
            Qd[3] = (Q2[3]-Q[3])*Kp
            Qd[4] = (Q2[4]-Q[4])*Kp
            Qd[5] = (Q2[5]-Q[5])*Kp
            Qd[5] = Qd[5]+(fext[5]-fext0[5])*Kf
            rtde_c.speedJ(Qd,acceleration,dt)
            time.sleep(0.002)
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        rtde_c.servoStop()
        rtde_c.stopScript()
        raise

if __name__ == '__main__':
    main()
    rospy.signal_shutdown("out mian KeyboardInterrupt")
    rtde_c.servoStop()
    rtde_c.stopScript()
