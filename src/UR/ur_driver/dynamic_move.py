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
