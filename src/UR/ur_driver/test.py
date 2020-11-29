#!/usr/bin/env python
from __future__ import print_function
import time
import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
import numpy as np
from math import pi
import rtde_control
import time

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
Q1 = [2.2,-0,-1.57,0,0,0]
Q2 = [1.5,0,-1.57,0,0,0]
Q3 = [1.5,-0.2,-1.57,0,0,0]
Q4 = [0,-pi/2,0,-pi/2,0,0]
QNum = 60
TNUM = 5

Q5 = np.zeros((6,QNum))
client = None
rtde_c = rtde_control.RTDEControlInterface("127.0.0.1")
# Parameters
acceleration = 0.5
dt = 1.0/500  # 2ms
joint_q = [-1.54, -1.83, -2.28, -0.59, 1.60, 0.023]
joint_speed = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# Move to initial joint position with a regular moveJ
rtde_c.moveJ(joint_q)

# Move to initial joint position with a regular moveJ
rtde_c.moveJ(joint_q)

def OnceMove():
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    d = 0.0
    g.trajectory.points = []
    for i in range(QNum):
        if i < QNum/2 :
            Q5[:,i] = Q4
            Q5[2,i] = Q4[2] + i*pi/180
            Q5[3,i] = Q4[3] + i*pi/180
        else:
            Q5[:,i] = Q4
            Q5[2,i] = Q4[2] + (QNum-i)*pi/180
            Q5[3,i] = Q4[3] + (QNum-i)*pi/180
        d += 0.1
        g.trajectory.points.append(
            JointTrajectoryPoint(positions = Q5[:,i], velocities=[0]*6, time_from_start = rospy.Duration(d)))
    #print(g.trajectory.points)
    client.send_goal(g)
    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise

def dynamicMove():
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    d = 0.0
    g.trajectory.points = []
    for i in range(QNum):
        if i < QNum/2 :
            Q5[:,i] = Q4
            Q5[2,i] = Q4[2] + i*pi/180
            Q5[3,i] = Q4[3] + i*pi/180
        else:
            Q5[:,i] = Q4
            Q5[2,i] = Q4[2] + (QNum-i)*pi/180
            Q5[3,i] = Q4[3] + (QNum-i)*pi/180
        d += 0.1
        g.trajectory.points.append(
            JointTrajectoryPoint(positions = Q5[:,i], velocities=[0]*6, time_from_start = rospy.Duration(d)))
        if i % TNUM == (TNUM - 1):
            g.trajectory.points.append(
                JointTrajectoryPoint(positions = Q5[:,i], velocities=[0]*6, time_from_start = rospy.Duration(d + 0.02)))
            start=time.time()
            client.send_goal(g)
            end=time.time()
            print('Running time: %s Seconds'%(end-start))
            time.sleep(d)
            del g.trajectory.points[:]
            d = 0.02
            g.trajectory.points.append(
                JointTrajectoryPoint(positions = Q5[:,i], velocities=[0]*6, time_from_start = rospy.Duration(d)))
    #print(g.trajectory.points)
    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise

def endVelMove():
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    d = 0.0
    g.trajectory.points = []
    for i in range(QNum):
        if i < QNum/2 :
            Q5[:,i] = Q4
            Q5[2,i] = Q4[2] + i*pi/180
            Q5[3,i] = Q4[3] + i*pi/180
        else:
            Q5[:,i] = Q4
            Q5[2,i] = Q4[2] + (QNum-i)*pi/180
            Q5[3,i] = Q4[3] + (QNum-i)*pi/180
        if i % TNUM == 0 and i > 0:
            g.trajectory.points[TNUM-1].velocities = [0, 0 ,(Q5[2,i] - Q5[2,i-2]) / 0.2, (Q5[2,i] - Q5[2,i-2]) / 0.2, 0 ,0]
            print(g.trajectory.points[TNUM-1].velocities)
            client.send_goal(g)
            time.sleep(d)
            del g.trajectory.points[:]
            d = 0
        d += 0.1
        g.trajectory.points.append(
            JointTrajectoryPoint(positions = Q5[:,i], velocities=[0]*6, time_from_start = rospy.Duration(d)))
    client.send_goal(g)
    #print(g.trajectory.points)
    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise

def unsafeMove():
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    d = 0.0
    g.trajectory.points = []
    for i in range(QNum):
        if i < QNum/2 :
            Q5[:,i] = Q4
            Q5[2,i] = Q4[2] + i*pi/180
            Q5[3,i] = Q4[3] + i*pi/180
        else:
            Q5[:,i] = Q4
            Q5[2,i] = Q4[2] + (QNum-i)*pi/180
            Q5[3,i] = Q4[3] + (QNum-i)*pi/180
        d += 0.1
        g.trajectory.points.append(
            JointTrajectoryPoint(positions = Q5[:,i], velocities=[0]*6, time_from_start = rospy.Duration(d)))
        if i % TNUM == (TNUM - 1):
            client.send_goal(g)
            time.sleep(d - 0.2)
            del g.trajectory.points[:]
            d = 0
            d += 0.1
            g.trajectory.points.append(
                JointTrajectoryPoint(positions = Q5[:,i-1], velocities=[0]*6, time_from_start = rospy.Duration(d)))
            d += 0.1
            g.trajectory.points.append(
                JointTrajectoryPoint(positions = Q5[:,i], velocities=[0]*6, time_from_start = rospy.Duration(d)))
    #print(g.trajectory.points)
    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise

def initMove():
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    g.trajectory.points = [
        JointTrajectoryPoint(positions=Q4, velocities=[0]*6, time_from_start=rospy.Duration(2.0))]
    client.send_goal(g)
    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise

def move1():
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    g.trajectory.points = [
        JointTrajectoryPoint(positions=Q1, velocities=[0]*6, time_from_start=rospy.Duration(2.0)),
        JointTrajectoryPoint(positions=Q2, velocities=[0]*6, time_from_start=rospy.Duration(3.0)),
        JointTrajectoryPoint(positions=Q3, velocities=[-0.1]*6, time_from_start=rospy.Duration(4.0))]
    client.send_goal(g)
    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise

def main():
    global client
    try:
        rospy.init_node("test_move", anonymous=True, disable_signals=True)
#        client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
        client = actionlib.SimpleActionClient("arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
        print("Waiting for server...")

	    # Execute 500Hz control loop for 2 seconds, each cycle is 2ms
        for i in range(1000):
            start = time.time()
            rtde_c.speedJ(joint_speed, acceleration, dt)
            joint_speed[0] += 0.0005
            joint_speed[1] += 0.0005
            end = time.time()
            duration = end - start
            if duration < dt:
                time.sleep(dt - duration)

        rtde_c.speedStop()
        rtde_c.stopScript()

        client.wait_for_server()
        print("Connected to server")
        initMove()
        #OnceMove()
        #dynamicMove()
        #endVelMove()
        #move1()
        unsafeMove()
        #move_repeated()
        #move_disordered()
        #move_interrupt()
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__': main()
