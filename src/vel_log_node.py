#!/usr/bin/env python

## Simple velocity logger that listens to std_msgs/Int16
##   published to the 'lwheel' and 'rwheel' topics.
## After five seconds of not receiving data from either
##   topic, stops collection and produces graphs of:
##     ticks over time
##     m/s over time
##     velocity over time
##     acceleration over time

import rospy
from std_msgs.msg import Int16
import numpy
import matplotlib.pyplot as plot

last_recieved_data = 0 # the rospy Time when data was last received

lwheel = []
rwheel = []

demaS = 0
demaB = 0
demaLastS = 0
demaLastB = 0

alpha = 0.19
beta = 0.041

def demaInit():
    global demaS
    global demaB
    global demaLastS
    global demaLastB
    demaS = 0
    demaB = 0
    demaLastS = 0
    demaLastB = 0

def demaFilter(val):
    global demaS
    global demaB
    global demaLastS
    global demaLastB
    demaS = (alpha * val) + ((1 - alpha) * (demaLastS + demaLastB))
    demaB = (beta * (demaS - demaLastS)) + ((1 - beta) * demaLastB)
    demaLastS = demaS
    demaLastB = demaB
    return demaS + demaB

def left_callback(data):
    global lwheel
    global last_recieved_data
    last_recieved_data = rospy.Time.now()
    #rospy.loginfo(rospy.get_caller_id() + 'right: %s', data.data)
    lwheel.append(data.data)

def right_callback(data):
    global rwheel
    global last_recieved_data
    last_recieved_data = rospy.Time.now()
    #rospy.loginfo(rospy.get_caller_id() + 'left: %s', data.data)
    rwheel.append(-1 * data.data)

# from a list of ticks 15ms apart, returns a list of "instantaneous" velocities
def velocities(ticks):
    ticks_in_meter = 1390.0
    ms_between_ticks = 15.0
    diff_ticks = numpy.diff(ticks)
    meters_traveled = [tick/ticks_in_meter for tick in diff_ticks]
    raw_vel = [(dist*1000.0)/ms_between_ticks for dist in meters_traveled]
    demaInit()
    return [demaFilter(a) for a in raw_vel]

# from a list of velocity values 15ms apart, returns a list of "instantaneous" accelerations
def accelerations(velocities):
    ms_between_ticks = 15.0
    diff_vel = numpy.diff(velocities)
    raw_acc = [(vel*1000.0)/ms_between_ticks for vel in diff_vel]
    demaInit()
    return [demaFilter(a) for a in raw_acc]

def listener():
    global lwheel
    global rwheel
    global last_recieved_data
    numpy.set_printoptions(threshold=numpy.nan)
    rospy.init_node('vel_log', anonymous=True)
    rospy.Subscriber('lwheel', Int16, left_callback)
    rospy.Subscriber('rwheel', Int16, right_callback)

    last_recieved_data = rospy.Time.now()

    # collect data until it has been more than five
    #   seconds since the last data was received
    while rospy.Time.now() - last_recieved_data < rospy.Duration.from_sec(5):
         rospy.sleep(rospy.Duration(1, 0))

    # plot raw encoder ticks over time
    #plot.plot(lwheel, label="Left pos")
    #plot.plot(rwheel, label="Right pos")
    
    # plot velocity between intervals over time
    lvels = velocities(lwheel)
    rvels = velocities(rwheel)
    LVeltime = numpy.arange(0.0, 15.0*(len(lvels))/1000, 0.015)
    RVeltime = numpy.arange(0.0, 15.0*(len(rvels))/1000, 0.015)



    plot.plot(LVeltime,lvels, label="Left vel")
    plot.plot(RVeltime,rvels, label="Right vel")
    
    # plot acceleration between intervals over time
    lacc = accelerations(lvels)
    racc = accelerations(rvels)
    LAcctime = numpy.arange(0.0149, 15.0*(len(lacc))/1000.0, 0.015)
    RAcctime = numpy.arange(0.0, 15.0*(len(racc))/1000.0, 0.015)
    print(RAcctime.size)
    print(len(racc))
    

    print(LAcctime.size)
    print(len(lacc))
    plot.plot(LAcctime,lacc, label="Left acc")
    plot.plot(RAcctime,racc, label="Right acc")

    print("Left vel max: ", max(lvels), ", Right vel max: ", max(rvels))
    print("Left acc max: ", max(lacc), ", Right acc max: ", max(racc))

    plot.legend()
    plot.show()

if __name__ == '__main__':
    listener()
