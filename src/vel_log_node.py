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

def left_callback(data):
    global lwheel
    last_recieved_data = rospy.Time.now()
    rospy.loginfo(rospy.get_caller_id() + 'right: %s', data.data)
    lwheel.append(data.data)

def right_callback(data):
    global rwheel
    last_recieved_data = rospy.Time.now()
    rospy.loginfo(rospy.get_caller_id() + 'left: %s', data.data)
    rwheel.append(data.data)

# from a list of ticks 15ms apart, returns a list of "instantaneous" velocities
def velocities(ticks):
    ticks_in_meter = 1390
    ms_between_ticks = 15
    diff_ticks = numpy.diff(ticks)
    meters_traveled = [diff_ticks/ticks_in_meter for tick in diff_ticks]
    return [((meters_traveled*1000)/ms_between_ticks) for meters_traveled in meters_traveled]

# from a list of velocity values 15ms apart, returns a list of "instantaneous" accelerations
def accelerations(velocities):
    ms_between_values = 15
    return [(velocity*1000)/ms_between_values for velocity in velocities]

def listener():
    global lwheel
    global rwheel
    rospy.init_node('vel_log', anonymous=True)
    rospy.Subscriber('lwheel', Int16, left_callback)
    rospy.Subscriber('rwheel', Int16, right_callback)

    last_recieved_data = rospy.Time.now()

    # collect data until it has been more than five
    #   seconds since the last data was received
    while rospy.Time.now() - last_recieved_data < rospy.Duration.from_sec(5):
         rospy.sleep(rospy.Duration(1, 0))

    # plot raw encoder ticks over time
    plot.plot(lwheel)
    plot.plot(rwheel)
    plot.show()

    # plot velocity between intervals over time
    lvels = velocities(lwheel)
    rvels = velocities(rwheel)
    plot.plot(lvels)
    plot.plot(rvels)
    plot.show()

    # plot acceleration between intervals over time
    lacc = accelerations(lvels)
    racc = accelerations(rvels)
    plot.plot(lacc)
    plot.plot(racc)
    plot.show()

if __name__ == '__main__':
    listener()
