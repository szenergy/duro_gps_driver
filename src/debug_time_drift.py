#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import TimeReference

pub = rospy.Publisher("debug_time_drift", Float64, queue_size=10)

def callback(data):
    global pub
    time_diff = (data.header.stamp - data.time_ref).to_sec() 
    rospy.loginfo("Time difference: %15.3f", time_diff)
    pub.publish(time_diff)
    
def listener():
    rospy.init_node("duro_time_drift_debugger", anonymous=True)
    rospy.logwarn("ONLY TEST PURPOSE " + rospy.get_caller_id())
    rospy.Subscriber("/gps/duro/time_ref", TimeReference, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
