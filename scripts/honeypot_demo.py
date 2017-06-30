#!/usr/bin/env python                                
import rospy
import math
import tf
from sensor_msgs.msg import JointState
from ipab_lwr_msgs.msg import FriCommandJointStiffness
from visualization_msgs.msg import Marker
import numpy as np

class demo:

    def __init__(self):
        self.jointState = None
        self.bowlPosition = [0.7,0.0,0.15]
        self.radius = 0.15
        self.lowStiffness = [1,1,1,1,1,1,1]
        self.highStiffness = [50,50,50,50,50,80,10]

        self.listener = tf.TransformListener()
        rospy.Subscriber("/joint_states", JointState, self.callback) #"/kuka_lwr_state" ? Not working!?
        self.pubCommand = rospy.Publisher('/lwr/commandJointStiffness', FriCommandJointStiffness, queue_size=10)
        self.pubBowl = rospy.Publisher('/bowl_marker', Marker, queue_size=10)

        self.setupBowlMarker()

    def setupBowlMarker(self):
        self.marker = Marker()
        self.marker.header.frame_id = 'base'
        self.marker.ns = 'Bowl'
        self.marker.id = 0
        self.marker.type = 2    
        self.marker.action = 0
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = 0.0
        self.marker.pose.orientation.w = 1.0
        self.marker.pose.position.x = self.bowlPosition[0]
        self.marker.pose.position.y = self.bowlPosition[1]
        self.marker.pose.position.z = self.bowlPosition[2]
        self.marker.scale.x = self.radius
        self.marker.scale.y = self.radius
        self.marker.scale.z = self.radius
        self.marker.color.a = 1.0
        self.marker.color.r = 0.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0

    def callback(self, data):
        self.jointState = data.position

    def checkInBowl(self, eff):
        diff = np.array(eff)-np.array(self.bowlPosition)
        d = np.linalg.norm(diff)
        dz = diff[2]
        return d<self.radius and dz<0

    def run(self):
        rospy.loginfo('Waiting for transforms')
        self.listener.waitForTransform('world_frame','spoon_tip', rospy.Time(), rospy.Duration(4.0))
        rate = rospy.Rate(100.0) #guess
        rospy.loginfo('Starting')

        inbowl=False
        inbowlPrev=False

        while not rospy.is_shutdown():
            now = rospy.Time()
            self.listener.waitForTransform('world_frame','spoon_tip', now, rospy.Duration(4.0))
            (trans,rot) = self.listener.lookupTransform('world_frame','spoon_tip', now)
        
            if self.jointState != None:
                message=FriCommandJointStiffness()
                message.jointPosition = self.jointState
                message.jointDamping=tuple([0.7]*7)
                inbowl = self.checkInBowl(trans)
                if(inbowl != inbowlPrev and inbowl):
                    rospy.loginfo('In the bowl')
                if(inbowl != inbowlPrev and not inbowl):
                    rospy.loginfo('Out of the bowl')
                if inbowl:
                    message.jointStiffness = self.highStiffness
                else:
                    message.jointStiffness = self.lowStiffness
                self.marker.header.stamp = now
                self.pubBowl.publish(self.marker)
                self.pubCommand.publish(message)
                inbowlPrev = inbowl
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('kuka_honeypot', anonymous=True)
    honeypot = demo()
    honeypot.run()
