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
import volumecal



class Detector():
    def __init__(self):
        self.pub = rospy.Publisher("/object_detect/bounding_boxes", PoseList, queue_size=10)
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber("/astra/cpp_image_node/image_roi", Image, self.find_bboxes)
        self.sub2 = rospy.Subscriber("/camera_01/depth_registered/points", PointCloud2, self.pcl_cb)
        self.obj = ObjLocation()
        self.poseList = PoseList()
        self.itm_list = rospy.get_param('/opencv_detect/item_list')
        self.volume = rospy.get_param('/opencv_detect/cv_param/volume')
        self.origin_point = rospy.get_param('/opencv_detect/cv_param/origin_point')
        self.theta = rospy.get_param('/opencv_detect/cv_param/theta')
        self.cloudroi = rospy.get_param('/opencv_detect/cv_param/roirange')


        rospy.loginfo("Initialized..")
        self.cloud = None


    def convertCloudFromRosToOpen3d(self, cloud_data):
        # Check empty
        open3d_cloud = open3d.PointCloud()
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
                rgb = [convert_rgbFloat_to_tuple(rgb) for x,y,z,rgb in cloud_data ]
            else:
                rgb = [convert_rgbUint32_to_tuple(rgb) for x,y,z,rgb in cloud_data ]

            # combine
            open3d_cloud.points = open3d.Vector3dVector(np.array(xyz))
            open3d_cloud.colors = open3d.Vector3dVector(np.array(rgb)/255.0)
        else:
            xyz = [(x,y,z) for x,y,z in cloud_data ] # get xyz
            open3d_cloud.points = open3d.Vector3dVector(np.array(xyz))

        # return
        return open3d_cloud

    def pcl_cb(self, ros_point_cloud):


        field_names=[field.name for field in ros_cloud.fields]
        ros_pointcloud = list(pc2.read_points(ros_point_cloud, skip_nans=True, field_names = field_names))

        o3d_cloud = convertCloudFromRosToOpen3d(ros_pointcloud)  # convert to open3d

        np_cloud = np.asarray(o3d_cloud.points)

        #calibrition between camera and reference frame
        Rotation = volumecal.eulerAnglesToRotationMatrix(self.theta)
        Translation = origin_point
        Translation = np.asarray(Translation)

        #get roi point cloud
        leftupper = self.cloudroi[0]
        rightbottom = self.cloudroi[1]
        bounding_ploy = np.array([
                          [leftupper[0],leftupper[1], 0],
                          [rightbottom[0], leftupper[1], 0],
                          [rightbottom[0], rightbottom[1], 0],
                          [leftupper[0], rightbottom[1], 0]
                         ], dtype = np.float32).reshape([-1, 3]).astype("float64")

        bounding_polygon = np.array(bounding_ploy, dtype = np.float64)
        vol = o3d.visualization.SelectionPolygonVolume()

        #The Z-axis is used to define the height of the selected region
        vol.orthogonal_axis = "Z"
        vol.axis_max = np_source[:,2].max()
        vol.axis_min =np_source[:,2].min()

        vol.bounding_polygon = o3d.utility.Vector3dVector(bounding_polygon)
        pcd_food = vol.crop_point_cloud(pcd_source)

        xyz_load = np.asarray(pcd_food.points)

        # convert the point respect to reference frame
        TranslationM = np.tile(Translation, (np.size(xyz_load,0),1))
        pcd_transfrom = np.dot( Rotation, xyz_load.T) + TranslationM.T
        pcd_transfrom = pcd_transfrom.T

        # point2grid
        pointGrid, pointNum = volumecal.voxel_conv(pcd_transfrom, 0.05)

        Max_grid = pointGrid[pointNum.index(max(pointNum))]
        Max_grid = np.array(Max_grid, dtype=np.float64)
        print("max gird:    ", Max_grid[:,2].max())
        print("max grid point:   ", Max_grid[np.argmax(Max_grid[:,2])])
        center_point = Max_grid[np.argmax(Max_grid[:,2])]
        print("center point:   ",center_point)
        indexfoodtrans = ismember(pcd_transfrom,center_point, axis='row')
        centerori = xyz_load[indexfoodtrans]
        centerori = np.squeeze(centerori)
        #indexfoodtrans = np.argwhere(pcd_transfrom==center_point)
        #centerori = pcd_transfrom[indexfoodtrans]
        print("center point index:   ",indexfoodtrans)

        print("center point origin:   ",centerori)
        print("center point origin1:   ",centerori[0])

        InnerCircle = volumecal.InnerCircle(pcd_transfrom,center_point, 0.3)

        print("InnerCircle:   ",len(InnerCircle))

        InnerCircle = np.array(InnerCircle, dtype=np.float64)


        Depth = volumecal.depthcalc(InnerCircle, self.volume, 0.01)
        print("depth:   ",Depth)


 

        # process your pcl
        self.cloud = pcl_data_from_subscriber

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
        print "Interrupted by user, shutting down.."
        sys.exit(0)

if __name__ == '__main__':
    main()
