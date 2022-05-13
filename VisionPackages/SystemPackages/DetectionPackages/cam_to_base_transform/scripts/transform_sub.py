#!/usr/bin/env python
from operator import truediv
import rospy
import sys
import tf
import tf2_ros
from std_msgs.msg import Float32MultiArray

def callback(msg):
    rospy.loginfo("Transform: " + str(msg.data))
    rospy.loginfo("Transform type: " + str(type(msg.data)))

def main():
    rospy.init_node('tf2_subscriber', anonymous=True)
    sub = rospy.Subscriber('cam_to_base_transform', Float32MultiArray, callback)
    rospy.spin()

if __name__ == '__main__':
    main()