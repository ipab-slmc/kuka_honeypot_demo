#!/usr/bin/env python

# Republishes joint states from "/bag" prefix topic and fixes the time stamps

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import *
import tf


def callback(data, pub):
    data.header.stamp = rospy.Time.now()
    pub.publish(data)

def listener():

    pub = rospy.Publisher('/joint_states', sensor_msgs.msg.JointState, queue_size=10)
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('/bag/joint_states', sensor_msgs.msg.JointState, callback, pub)

    rospy.spin()

if __name__ == '__main__':
    listener()

