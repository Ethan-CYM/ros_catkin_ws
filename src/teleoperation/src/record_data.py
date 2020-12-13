#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function
import rtde_control
import rtde_receive
import rospy
import time
import sys
import threading
from threading import _Timer
from Queue import Queue
from sensor_msgs.msg import JointState
# from omni_msgs.msg import OmniState

rtde_r = rtde_receive.RTDEReceiveInterface("192.168.0.104")

actualQ = Queue()
actualV = Queue()
actualC = Queue()
targetQ = Queue()
targetV = Queue()
targetC = Queue()
TouchQ = Queue()
timestart =  time.time()
timeinversial =  time.time()
a_ur_joint_degree = open("ur_joint_actual_degree.txt", mode='w')
a_ur_joint_degree.write('time  shoulder_pan_joint  shoulder_lift_joint  elbow_joint  wrist_1_joint  wrist_2_joint  wrist_3_joint\n')
a_ur_joint_velocity = open("ur_joint_actual_velocity.txt", mode='w')
a_ur_joint_velocity.write('time  shoulder_pan_joint  shoulder_lift_joint  elbow_joint  wrist_1_joint  wrist_2_joint  wrist_3_joint\n')
a_ur_joint_current = open("ur_joint_actual_current.txt", mode='w')
a_ur_joint_current.write('time  shoulder_pan_joint  shoulder_lift_joint  elbow_joint  wrist_1_joint  wrist_2_joint  wrist_3_joint\n')
t_ur_joint_degree = open("ur_joint_target_degree.txt", mode='w')
t_ur_joint_degree.write('time  shoulder_pan_joint  shoulder_lift_joint  elbow_joint  wrist_1_joint  wrist_2_joint  wrist_3_joint\n')
t_ur_joint_velocity = open("ur_joint_target_velocity.txt", mode='w')
t_ur_joint_velocity.write('time  shoulder_pan_joint  shoulder_lift_joint  elbow_joint  wrist_1_joint  wrist_2_joint  wrist_3_joint\n')
t_ur_joint_current = open("ur_joint_target_current.txt", mode='w')
t_ur_joint_current.write('time  shoulder_pan_joint  shoulder_lift_joint  elbow_joint  wrist_1_joint  wrist_2_joint  wrist_3_joint\n')
touch_joint_degree = open("touch_joint_degree.txt", mode='w')
touch_joint_degree.write('time  waist  shoulder  elbow  yaw  pitch  roll\n')


def touchJointCallback(data):
    global TouchQ
    global timestart
    timenow =  time.time()
    s = str(timenow - timestart) + "  "
    for i in range(len(data.position)):
        s = s + str(data.position[i]).replace('[','').replace(']','').replace("'",'').replace(',','') + "  "
    s = s + "\n"
    TouchQ.put(s)


# def touchStateCallback(data):
#     # print(data)
#     a = data

def getURData():
    global actualQ
    global actualV
    global actualC
    global targetQ
    global targetV
    global targetC
    global timestart
    global timeinversial
    try:
        while not rospy.is_shutdown():
            timenow =  time.time()
            timeinversial = time.time()
            s = str(timenow - timestart) + "  "
            data = rtde_r.getActualQ()
            for i in range(len(data)):
                s = s + str(data[i]).replace('[','').replace(']','').replace("'",'').replace(',','') + "  "
            s = s + "\n"
            actualQ.put(s)

            timenow =  time.time()
            s = str(timenow - timestart) + "  "
            data = rtde_r.getActualQd()
            for i in range(len(data)):
                s = s + str(data[i]).replace('[','').replace(']','').replace("'",'').replace(',','') + "  "
            s = s + "\n"
            actualV.put(s)

            timenow =  time.time()
            s = str(timenow - timestart) + "  "
            data = rtde_r.getActualCurrent()
            for i in range(len(data)):
                s = s + str(data[i]).replace('[','').replace(']','').replace("'",'').replace(',','') + "  "
            s = s + "\n"
            actualC.put(s)

            timenow =  time.time()
            s = str(timenow - timestart) + "  "
            data = rtde_r.getTargetQ()
            for i in range(len(data)):
                s = s + str(data[i]).replace('[','').replace(']','').replace("'",'').replace(',','') + "  "
            s = s + "\n"
            targetQ.put(s)

            timenow =  time.time()
            s = str(timenow - timestart) + "  "
            data = rtde_r.getTargetQd()
            for i in range(len(data)):
                s = s + str(data[i]).replace('[','').replace(']','').replace("'",'').replace(',','') + "  "
            s = s + "\n"
            targetV.put(s)

            timenow =  time.time()
            s = str(timenow - timestart) + "  "
            data = rtde_r.getTargetCurrent()
            for i in range(len(data)):
                s = s + str(data[i]).replace('[','').replace(']','').replace("'",'').replace(',','') + "  "
            s = s + "\n"
            targetC.put(s)
            timeinversial =time.time() - timeinversial
            time.sleep(0.008 - timeinversial)
    except KeyboardInterrupt:
            raise

# class RepeatingTimer(_Timer):
#     def run(self):
#         while not self.finished.is_set():
#             self.function(*self.args, **self.kwargs)
#             self.finished.wait(self.interval)

def writeURData():
        global actualQ
        global actualV
        global actualC
        global targetQ
        global targetV
        global targetC
        global a_ur_joint_degree
        global a_ur_joint_velocity
        global a_ur_joint_current
        global t_ur_joint_degree
        global t_ur_joint_velocity
        global t_ur_joint_current
        try:
            while not rospy.is_shutdown():
                print(actualQ.qsize())
                if ~actualQ.empty():
                    s = actualQ.get()
                    a_ur_joint_degree.write(s)
                if ~actualV.empty():
                    s = actualV.get()
                    a_ur_joint_velocity.write(s)
                if ~actualC.empty():
                    s = actualC.get()
                    a_ur_joint_current.write(s)
                if ~targetQ.empty():
                    s = targetQ.get()
                    t_ur_joint_degree.write(s)
                if ~targetV.empty():
                    s = targetV.get()
                    t_ur_joint_velocity.write(s)
                # print(targetC.qsize())
                if ~targetC.empty():
                    s = targetC.get()
                    t_ur_joint_current.write(s)

        except KeyboardInterrupt:
            raise

def main():
    global TouchQ
    global touch_joint_degree
    rospy.init_node("record_data", anonymous=True, disable_signals=True)
    rospy.Subscriber("/phantom/joint_states", JointState, touchJointCallback)
    # rospy.Subscriber("/phantom/state", OmniState, touchStateCallback)
    sentinel = threading.Event()
    sentinel.set()
    # timerGetURData = RepeatingTimer(0.0078, getURData)
    tGetURData = threading.Thread(target=getURData)
    tGetURData.setDaemon(True)
    tGetURData.start()
    twriteURData = threading.Thread(target=writeURData)
    twriteURData.setDaemon(True)
    twriteURData.start()
    
    try:
        while not rospy.is_shutdown():
            print(TouchQ.qsize())
            if ~TouchQ.empty():
                s = TouchQ.get()
                touch_joint_degree.write(s)

    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        a_ur_joint_degree.close()
        a_ur_joint_velocity.close()
        a_ur_joint_current.close()
        t_ur_joint_degree.close()
        t_ur_joint_velocity.close()
        t_ur_joint_current.close()
        touch_joint_degree.close()
        raise

if __name__ == '__main__':
    main()
