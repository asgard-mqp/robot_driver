#!/usr/bin/env python

## Simple velocity logger that listens to std_msgs/Int16
##   published to the 'lwheel' and 'rwheel' topics.
## After 1000 data points from both, produces a graph of
##   the data.

import rospy
from std_msgs.msg import Int16
import numpy
import matplotlib.pyplot as plot

last_recieved_data = rospy.Time.now()

lwheel = []
rwheel = []

def left_callback(data):
    last_recieved_data = rospy.Time.now()
    rospy.loginfo(rospy.get_caller_id() + 'right: %s', data.data)
    numpy.append(lwheel, [rospy.Time.now().to_sec(), data.data], 2)

def right_callback(data):
    last_recieved_data = rospy.Time.now()
    rospy.loginfo(rospy.get_caller_id() + 'left: %s', data.data)
    time = rospy.Time.now()
    time = time.to_sec()
    numpy.append(rwheel, [rospy.Time.now().to_sec(), data.data], 2)

def listener():
    rospy.init_node('vel_log', anonymous=True)
    rospy.Subscriber('lwheel', Int16, left_callback)
    rospy.Subscriber('rwheel', Int16, right_callback)
    
    # collect data until it has been more than a minute 
    # since the last data was recieved, exit this loop
    while rospy.Time.now() - last_recieved_data > rospy.Duration.fromsec(60):
        rospy.spinOnce()
    
    # plot raw encoder ticks over time
    plot.matshow(lwheel, cmap='Blues')
    plot.matshow(rwheel, cmap='Greens')
    plot.show()

if __name__ == '__main__':
    listener()

