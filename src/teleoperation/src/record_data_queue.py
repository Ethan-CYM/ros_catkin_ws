#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function
import time
import random
import sys
import threading
from Queue import Queue
import rtde_control
import rtde_receive
import rospy
from sensor_msgs.msg import JointState
from omni_msgs.msg import OmniState

class Producer(threading.Thread):
    def __init__(self, t_name, queue):
        threading.Thread.__init__(self,name=t_name)
        self.data=queue
    def run(self):
        for i in range(10):    #随机产生10个数字 ，可以修改为任意大小
            randomnum=random.randint(1,99)
            print("%s: %s is producing %d to the queue!" % (time.ctime(), self.getName(), randomnum))
            self.data.put(randomnum)  #将数据依次存入队列
            time.sleep(1)
        print("%s: %s finished!" %(time.ctime(), self.getName()))

#Consumer thread
class Consumer_even(threading.Thread):
    def __init__(self,t_name,queue):
        threading.Thread.__init__(self,name=t_name)
        self.data=queue
    def run(self):
        while 1:
            try:
                val_even = self.data.get(1,5)  #get(self, block=True, timeout=None) ,1就是阻塞等待,5是超时5秒
                if val_even%2==0:
                    print("%s: %s is consuming. %d in the queue is consumed!" % (time.ctime(),self.getName(),val_even))
                    time.sleep(2)
                else:
                    self.data.put(val_even)
                    time.sleep(2)

            except KeyboardInterrupt:
                print("Main thread catch KeyboardInterrupt.")
                raise
            except:     #等待输入，超过5秒  就报异常
                print("%s: %s finished!" %(time.ctime(),self.getName()))
                break
class Consumer_odd(threading.Thread):
    def __init__(self,t_name,queue):
        threading.Thread.__init__(self, name=t_name)
        self.data=queue
    def run(self):
        while 1:
            try:
                val_odd = self.data.get(1,5)
                if val_odd%2!=0:
                    print("%s: %s is consuming. %d in the queue is consumed!" % (time.ctime(), self.getName(), val_odd))
                    time.sleep(2)
                else:
                    self.data.put(val_odd)
                    time.sleep(2)
            except KeyboardInterrupt:
                print("Main thread catch KeyboardInterrupt.")
                raise
            except:
                print("%s: %s finished!" % (time.ctime(), self.getName()))
                break
#Main thread
def main():
    sentinel = threading.Event()
    sentinel.set()
    queue = Queue()
    producer = Producer('Pro.', queue)
    consumer_even = Consumer_even('Con_even.', queue)
    consumer_odd = Consumer_odd('Con_odd.',queue)
    producer.setDaemon(True)
    consumer_even.setDaemon(True)
    consumer_odd.setDaemon(True)
    producer.start()
    consumer_even.start()
    consumer_odd.start()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        sentinel.clear()
        print("Main thread catch KeyboardInterrupt.")
        raise


if __name__ == '__main__':
    main()
