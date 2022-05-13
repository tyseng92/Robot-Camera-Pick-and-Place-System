#!/usr/bin/env python
import rospy
import sys
import tf
import tf2_ros
import geometry_msgs.msg
from std_msgs.msg import Float32MultiArray

def main():
    rospy.init_node('tf2_broadcaster', anonymous=True)
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    pub = rospy.Publisher('cam_to_base_transform', Float32MultiArray, queue_size=1)
    rate = rospy.Rate(10.0)
    # trans = None
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform("camera_01_depth_optical_frame", "base_link", rospy.Time())
            print("transform")
            #trans_rb = tfBuffer.lookup_transform('base_link','ref_link',rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            print("Exception:", e.message)
            rate.sleep()
            continue
        msg = Float32MultiArray()
        msg.data = [trans.transform.translation.x, 
                trans.transform.translation.y, 
                trans.transform.translation.z,
                trans.transform.rotation.x,
                trans.transform.rotation.y,
                trans.transform.rotation.z,
                trans.transform.rotation.w]
        pub.publish(msg)        
        rate.sleep()

if __name__ == '__main__':
        main()