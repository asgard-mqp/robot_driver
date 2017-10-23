#!/usr/bin/env python

## Simple velocity logger that listens to std_msgs/Int16
##   published to the 'lwheel' and 'rwheel' topics.
## After 1000 data points from both, produces a graph of
##   the data.

import rospy
from std_msgs.msg import Int16
import numpy
import matplotlib.pyplot as plot

last_recieved_data = 0

lwheel = []
rwheel = []

def left_callback(data):
    global lwheel
    last_recieved_data = rospy.Time.now()
    rospy.loginfo(rospy.get_caller_id() + 'right: %s', data.data)
    lwheel.append(data.data)
    #numpy.append(lwheel, 0)#[rospy.Time.now().to_sec(), data.data], 0)

def right_callback(data):
    global rwheel
    last_recieved_data = rospy.Time.now()
    rospy.loginfo(rospy.get_caller_id() + 'left: %s', data.data)
    numpy.append(rwheel, 0)#[rospy.Time.now().to_sec(), data.data], 0)
    rwheel.append(data.data)

def meters_per_second(ticks):
    ticks_in_meter = 1390
    meters_traveled = [tick/ticks_in_meter for tick in ticks]
    

def listener():
    global lwheel
    global rwheel
    rospy.init_node('vel_log', anonymous=True)
    rospy.Subscriber('lwheel', Int16, left_callback)
    rospy.Subscriber('rwheel', Int16, right_callback)

    last_recieved_data = rospy.Time.now()

    # collect data until it has been more than a minute
    # since the last data was recieved, exit this loop
    while rospy.Time.now() - last_recieved_data < rospy.Duration.from_sec(5):
         rospy.sleep(rospy.Duration(1, 0))

    #rospy.loginfo(rospy.get_caller_id() + 'left: %s', numpy.array_str(lwheel))
    #rospy.loginfo(rospy.get_caller_id() + 'right: %s', numpy.array_str(rwheel))

    #print(lwheel)
    #print(rwheel)

    # plot raw encoder ticks over time
    plot.plot(lwheel)
    plot.plot(rwheel)
    plot.show()

    # plot m/s over time
    plot.plot(meters_per_second(lwheel))
    plot.plot(meters_per_second(rwheel))
    plot.show()
    
    # plot acceleration over time
    ldelta = numpy.diff(lwheel)
    rdelta = numpy.diff(rwheel)
    lvel = [(delta*1000)/15 for delta in ldelta]
    rvel = [(delta*1000)/15 for delta in rdelta]
    plot.plot(lvel)
    plot.plot(rvel)
    plot.show()
    
    # plot velocity over time
    

if __name__ == '__main__':
    listener()

