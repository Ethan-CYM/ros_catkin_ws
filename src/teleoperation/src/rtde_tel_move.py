#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function
import rtde_control
import time
import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
from math import pi

# rtde_c = rtde_control.RTDEControlInterface("127.0.0.1")
rtde_c = rtde_control.RTDEControlInterface("192.168.0.104")

# Parameters
magFlag = False
urQ = [pi/2, -3.5*pi/9, 2*pi/3, -pi, -2*pi/4, 0]

def ControlParamterCallback(data):
    global magFlag
    global urQ
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    try:
        urQ = data.points[0].positions
        magFlag = True
    except KeyboardInterrupt:
        raise
    except:
        raise

def main():
    global magFlag
    global urQ
    velocity = 0.5
    acceleration = 0.5
    dt = 1.0/500  # 2ms
    lookahead_time = 0.1
    gain = 100
    try:
        rospy.init_node("ur_move", anonymous=True, disable_signals=True)
        rospy.Subscriber("ur_control_parameter", JointTrajectory, ControlParamterCallback)

        rtde_c.moveJ(urQ)

        while not rospy.is_shutdown():
            if(magFlag == True):
                magFlag = False
                rtde_c.servoJ(urQ, velocity, acceleration, dt, lookahead_time, gain)
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        rtde_c.servoStop()
        rtde_c.stopScript()
        raise

if __name__ == '__main__':
    main()
