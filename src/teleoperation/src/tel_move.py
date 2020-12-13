#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function
import time
import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
from math import pi

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
# 全局变量
client = None
subFlag = False
g = FollowJointTrajectoryGoal()
g.trajectory = JointTrajectory()
g.trajectory.joint_names = JOINT_NAMES

# 遥操作轨迹点发送
def moveTeleoperation():
    global g
    try:
        joint_states = rospy.wait_for_message("joint_states", JointState)
        joints_pos = joint_states.position
        client.send_goal(g)
        #client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise
    except:
        raise

# 移动到初始位姿
def MoveInit():
    global g
    global subFlag
    urInitQ = [-pi/2, -5*pi/9, -7*pi/9, 0*pi/4, 2*pi/4, 0]
    try:
        del g.trajectory.points[:]
        g.trajectory.points.append(
            JointTrajectoryPoint(positions=urInitQ, velocities=[0]*6, time_from_start=rospy.Duration(4.0)))
        client.send_goal(g)
        time.sleep(4.0)
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise
    except:
        raise

# 遥操作UR轨迹点消息订阅器的回调函数
def ControlParamterCallback(data):
    global g
    global subFlag
    try:
        del g.trajectory.points[:]
        for i in range(len(data.points)):
            g.trajectory.points.append(
                JointTrajectoryPoint(positions=data.points[i].positions, velocities=[0]*6, time_from_start=data.points[i].time_from_start))
        subFlag = True
    except KeyboardInterrupt:
        raise
    except:
        raise

def main():
    global client
    global subFlag
    try:
        rospy.init_node("test_move", anonymous=True, disable_signals=True)
        client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
        # client = actionlib.SimpleActionClient("arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
        rospy.Subscriber("ur_control_parameter", JointTrajectory, ControlParamterCallback)

        print("Waiting for server...")
        client.wait_for_server()
        print("Connected to server")
        MoveInit()
        while not rospy.is_shutdown():
            if(subFlag == True):
                subFlag = False
                moveTeleoperation()
    except KeyboardInterrupt:
        #stop_thread(add_thread)
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__':
    main()
