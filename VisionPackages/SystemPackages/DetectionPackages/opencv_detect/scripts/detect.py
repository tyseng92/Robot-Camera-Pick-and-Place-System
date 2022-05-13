#!/usr/bin/env python

import rospy
from bounding_box import ObjLocation    
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from opencv_detect.msg import Pose, PoseList
import datetime
import sys, os

class Detector():
    def __init__(self):
        self.pub = rospy.Publisher("/object_detect/bounding_boxes", PoseList, queue_size=10)
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber("/multiastra_subscriber1/astra/cpp_image_node/image_roi", Image, self.find_bboxes)
        self.obj = ObjLocation()
        self.poseList = PoseList()
        self.itm_list = rospy.get_param('/opencv_detect/item_list')
        rospy.loginfo("Initialized..")

    def find_bboxes(self,data):
        rospy.loginfo("In callback function..")

        a = datetime.datetime.now()
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        #cv2.namedWindow('test')
        #cv2.imshow('test',cv_image)
        #cv2.waitKey(1)

        (rows,cols,channels) = cv_image.shape
        #print("rows: {} cols: {}".format(str(rows), str(cols)))
        # run bounding box script

        item_list = self.obj.find_coordinate(cv_image)        
        print(item_list)

        if len(self.itm_list) != len(item_list):
            rospy.logerr("Number of detected items doest not equal to the length of param list in delta_socket!")
            rospy.set_param("/opencv_detect/cv_param/item_num_bool", False)
            #cv2.destroyWindow('Display')
            #os._exit(0)            
            return
        else:
            rospy.set_param("/opencv_detect/cv_param/item_num_bool", True)
            rospy.loginfo("Correct number of detected items")

        self.construct_msg(item_list)

        # output box info to publisher
        #self.pub.publish(self.poseList)

        b = datetime.datetime.now()
        print("time: " + str(b-a))
        #rospy.loginfo("Published message")

    def construct_msg(self, ilist):
        plist = []
        for k, v in ilist.iteritems():
            pose = Pose()
            pose.Class = k
            pose.cx = v[0]
            pose.cy = v[1]
            pose.angle = v[2]
            plist.append(pose) 

        self.poseList.pose_list = plist

    def timer_publish(self, event):
        self.pub.publish(self.poseList)
        rospy.loginfo("Published message")

def main():
    detector = Detector()
    rospy.init_node('opencv_contour', anonymous=True, disable_signals=True)
    try:
        rospy.loginfo("start to spin..")
        rospy.Timer(rospy.Duration(1/1), detector.timer_publish)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.sleep(5)
        print ("Interrupted by user, shutting down..")
        sys.exit(0)

if __name__ == '__main__':
    main()
