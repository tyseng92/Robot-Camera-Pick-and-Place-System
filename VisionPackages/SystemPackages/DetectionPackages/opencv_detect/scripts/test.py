#!/usr/bin/env python

import rospy
from bounding_box import ObjLocation    
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from opencv_detect.msg import Pose, PoseList
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import datetime
import sys, os

import open3d as o3d
import numpy as np
import math
from pyntcloud import PyntCloud
from pandas import DataFrame
import scipy.linalg as linalg

import matplotlib.pyplot as plt
from collections import Counter
import HSVdetect
#import volumecal

from ctypes import *



class Detector():
    def __init__(self):
        self.pub = rospy.Publisher("/object_detect/bounding_boxes", PoseList, queue_size=10)
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber("/astra/cpp_image_node/image_roi", Image, self.image_cb)
        self.sub2 = rospy.Subscriber("/camera_01/depth_registered/points", PointCloud2, self.pcl_cb)
        self.obj = ObjLocation()
        self.poseList = PoseList()
        self.itm_list = rospy.get_param('/opencv_detect/item_list')
        self.volume = rospy.get_param('/opencv_detect/cv_param/volume')
        self.origin_point = rospy.get_param('/opencv_detect/cv_param/origin_point')
        self.theta = rospy.get_param('/opencv_detect/cv_param/theta')
        self.cloudroi = rospy.get_param('/opencv_detect/cv_param/roirange')
        self.radius = rospy.get_param('/opencv_detect/cv_param/radius')
 

        rospy.loginfo("Initialized..")
        self.cv_image = None
        self.center_point = np.array([0,0,0])

        self.convert_rgbUint32_to_tuple = lambda rgb_uint32: ((rgb_uint32 & 0x00ff0000)>>16, (rgb_uint32 & 0x0000ff00)>>8, (rgb_uint32 & 0x000000ff))
        self.convert_rgbFloat_to_tuple = lambda rgb_float: self.convert_rgbUint32_to_tuple(int(cast(pointer(c_float(rgb_float)), POINTER(c_uint32)).contents.value))

    def convertCloudFromRosToOpen3d(self, cloud_data,field_names):
        # Check empty
        open3d_cloud = o3d.geometry.PointCloud()
        if len(cloud_data)==0:
            print("Converting an empty cloud")
            return None

        # Set open3d_cloud
        if "rgb" in field_names:
            IDX_RGB_IN_FIELD=3 # x, y, z, rgb
            
            # Get xyz
            xyz = [(x,y,z) for x,y,z,rgb in cloud_data ] # (why cannot put this line below rgb?)

            # Get rgb
            # Check whether int or float
            if type(cloud_data[0][IDX_RGB_IN_FIELD])==float: # if float (from pcl::toROSMsg)
                rgb = [self.convert_rgbFloat_to_tuple(rgb) for x,y,z,rgb in cloud_data ]
            else:
                rgb = [self.convert_rgbUint32_to_tuple(rgb) for x,y,z,rgb in cloud_data ]

            # combine
            open3d_cloud.points = o3d.utility.Vector3dVector(np.array(xyz))
            open3d_cloud.colors = o3d.utility.Vector3dVector(np.array(rgb)/255.0)
        else:
            xyz = [(x,y,z) for x,y,z in cloud_data ] # get xyz
            open3d_cloud.points = o3d.utility.Vector3dVector(np.array(xyz))

        # return
        return open3d_cloud

    def pcl_cb(self, ros_cloud):
        print("start pcl processing")

        field_names=[field.name for field in ros_cloud.fields]
        ros_pointcloud = list(pc2.read_points(ros_cloud, skip_nans=True, field_names = field_names))

        o3d_cloud = self.convertCloudFromRosToOpen3d(ros_pointcloud,field_names)  # convert to open3d

        np_cloud = np.asarray(o3d_cloud.points)
        print("start pcl processing")
        o3d.visualization.draw_geometries([o3d_cloud])


    def image_cb(self, ros_image):
        rospy.loginfo("In Image callback function..")

        a = datetime.datetime.now()
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError as e:
            print(e)

        HeightS, WidthS, ChannelS = self.cv_image.shape
        print("big image:  ", HeightS, WidthS, ChannelS)
        inputpixelidx = np.array([100,100])
        flag = self.colordetect(inputpixelidx,self.radius)

        print("flag:   ", flag)
        

   
 

    def image2pixel(self,pointincamera):
        print("try")
        

    def colordetect(self, inputpixelidx, radius):
       
        flag = 0
        inputpixelidx = np.array([80,150])
        
        imagecopy = self.cv_image.copy()
        imageROI = imagecopy[inputpixelidx[0]-radius:inputpixelidx[0]+radius, inputpixelidx[1]-radius:inputpixelidx[1]+radius] 

        print("sfdfdf:  ", inputpixelidx[0],inputpixelidx[1])

        HeightS, WidthS, ChannelS = imageROI.shape
        print("dhuueurh:  ", HeightS, WidthS, ChannelS)

        for row in range(HeightS): 
            for col in range(WidthS):
                dists = math.sqrt((row - radius)**2 + (col - radius)**2)
                
                if dists > radius:
                    imageROI[row, col] = 0

        cv2.imshow('ss',imageROI)


        point_color = (0, 0, 255)
        thickness = 4 
        #cv2.circle(cv_image, [inputpixelidx[1],inputpixelidx[0]], radius, point_color, thickness)
        cv2.imshow('rr',self.cv_image)

        color = HSVdetect.get_color(imageROI)
        print(color)
        if len(color) >=3:
            flag = 1
            print("correct")
        else:
            flag = 0

        return flag




    '''
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
        self.pub.publish(self.poseList)

        b = datetime.datetime.now()
        print("time: " + str(b-a))
        rospy.loginfo("Published message")

        '''

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

def main():
    detector = Detector()
    rospy.init_node('opencv_contour', anonymous=True, disable_signals=True)
    try:
        rospy.loginfo("start to spin..")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.sleep(5)
        print ("Interrupted by user, shutting down..")
        sys.exit(0)

if __name__ == '__main__':
    main()
