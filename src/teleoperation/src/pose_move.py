import rtde_control
import rtde_receive
import time

rtde_c = rtde_control.RTDEControlInterface("127.0.0.1")
rtde_r = rtde_receive.RTDEReceiveInterface("127.0.0.1")
init_q = rtde_r.getActualQ()


# Target in the Z-Axis of the TCP
target = rtde_r.getActualTCPPose()
print(target)
target[4] -= 3
print(target)
# Move asynchronously in cartesian space to target, we specify asynchronous behavior by setting the async parameter to
# 'True'. Try to set the async parameter to 'False' to observe a default synchronous movement, which cannot be stopped
# by the stopL function due to the blocking behaviour.
rtde_c.moveL(target, 0.25, 0.5, True)
time.sleep(3)
# Stop the movement before it reaches target
# rtde_c.stopL(2)

# Move back to initial joint configuration
target[4] += 3
rtde_c.moveL(target, 0.25, 0.5, True)
time.sleep(3)
# rtde_c.stopL(2)
# Stop the RTDE control script
rtde_c.stopScript()
