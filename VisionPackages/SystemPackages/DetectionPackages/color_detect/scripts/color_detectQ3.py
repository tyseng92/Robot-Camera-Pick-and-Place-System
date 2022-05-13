#!/usr/bin/env python

import rospy

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
from scipy import signal
import collections
from std_msgs.msg import Float32MultiArray
from scipy.spatial.transform import Rotation as R
import ros_numpy
from time import time

from ctypes import *



class Detector():
    def __init__(self):
        self.roi_path = rospy.get_param('/color_detect/cv_param/roi_path')
        self.roi_image = None
        self.cv_image = None
        self.read_yaml()

        self.pub = rospy.Publisher("/color_detect/center_list", PoseList, queue_size=1)
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber("/multiastra_subscriber1/astra/cpp_image_node/image_roi", Image, self.image_cb)
        self.sub2 = rospy.Subscriber("/camera_01/depth_registered/points", PointCloud2, self.pcl_cb, queue_size=1)
        self.sub3 = rospy.Subscriber('cam_to_base_transform', Float32MultiArray, self.trans_cb)
        #self.centerList = CenterList()
        self.poseList = PoseList()
        self.volume = rospy.get_param('/color_detect/cv_param/volume')
        self.H = rospy.get_param('/color_detect/cv_param/H')
        self.origin_point = rospy.get_param('/color_detect/cv_param/origin_point')
        self.theta = rospy.get_param('/color_detect/cv_param/theta')
        self.cloudroi = rospy.get_param('/color_detect/cv_param/cloudroi')
        self.radiusrario = rospy.get_param('/color_detect/cv_param/radiusrario')
        #self.roiimage = rospy.get_param('/color_detect/cv_param/roiimage')


        rospy.loginfo("Initialized..")
        self.target_point = []
        self.pubcount = 0
        self.convert_rgbUint32_to_tuple = lambda rgb_uint32: ((rgb_uint32 & 0x00ff0000)>>16, (rgb_uint32 & 0x0000ff00)>>8, (rgb_uint32 & 0x000000ff))
        self.convert_rgbFloat_to_tuple = lambda rgb_float: self.convert_rgbUint32_to_tuple(int(cast(pointer(c_float(rgb_float)), POINTER(c_uint32)).contents.value))
        self.trans_cam2base = []


    def read_yaml(self):
        fs = cv2.FileStorage(str(self.roi_path), cv2.FILE_STORAGE_READ)
        x = int(fs.getNode("x").real())
        y = int(fs.getNode("y").real())
        w = int(fs.getNode("width").real())
        h = int(fs.getNode("height").real())
        fs.release()
        self.roi_image = [x,y,w,h]
        self.cv_image = np.zeros((self.roi_image[3], self.roi_image[2], 3), dtype=np.uint8)

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

            # Get rgb/home/zyadan/catkin_ws/src/color_detect/scripts/detect.py
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

    def trans_cb(self, msg):
        self.trans_cam2base = list(msg.data)


    def pcl_cb(self, ros_cloud):
        print("start pcl processing")
        start_time = time()
        self.pubcount = self.pubcount + 1
        field_names=[field.name for field in ros_cloud.fields]
        ros_pointcloud = list(pc2.read_points(ros_cloud, skip_nans=True, field_names = field_names))

        pc_ros = ros_numpy.numpify(ros_cloud)
        heightros = pc_ros.shape[0]
        widthros = pc_ros.shape[1]
        print("widthros, heightros:  ", widthros, heightros)
        with open('/home/rrc2/catkin_ws/src/color_detect/scripts/pickingpoint.txt', 'a') as f:
            f.write(str(widthros)+str(',')+str(heightros)+str('\n'))
        np_allpoints = np.zeros((heightros*widthros, 3),dtype = np.float32)
        np_allpoints[:,0] = np.resize(pc_ros['x'], heightros*widthros)
        np_allpoints[:,1] = np.resize(pc_ros['y'], heightros*widthros)
        np_allpoints[:,2] = np.resize(pc_ros['z'], heightros*widthros)
        np.savetxt('/home/rrc2/catkin_ws/src/color_detect/np_points.txt', np_allpoints, delimiter=',')

        o3d_cloud = self.convertCloudFromRosToOpen3d(ros_pointcloud,field_names)  # convert to open3d
        
        np_cloud = np.asarray(o3d_cloud.points)

        #o3d.visualization.draw_geometries([pcdros])

        # transformation from camera to robot base
        Translation = self.trans_cam2base[0:3]
        Quaternion = self.trans_cam2base[3:]
        Rotation = R.from_quat(Quaternion)
        Rotation = Rotation.as_matrix()
        Rotation = np.asarray(Rotation)
        Translation = np.asarray(Translation)
        print("Translation: ", Translation)
        print("Rotation; ", Rotation)
        
        # GetROI=0 ROI predefined, 1: Get ROI by the function
        GetROI = 0
        if GetROI:
            #get roi point cloud
            picked_points = volumecal.pick_points(o3d_cloud)
            leftupper = o3d_cloud.points[picked_points[0]]
            rightbottom = o3d_cloud.points[picked_points[1]]
            print("leftupper rightbottom:  ", leftupper,rightbottom)
        else:
            leftupper = self.cloudroi[0]
            rightbottom = self.cloudroi[1]
            print("start pcl processing:  ", leftupper,rightbottom)


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
        vol.axis_max = np_cloud[:,2].max()
        vol.axis_min =np_cloud[:,2].min()

        vol.bounding_polygon = o3d.utility.Vector3dVector(bounding_polygon)
        pcd_food = vol.crop_point_cloud(o3d_cloud)
        print("crop roi point cloud ....")
        #o3d.visualization.draw_geometries([pcd_food])
        xyz_load = np.asarray(pcd_food.points)

        # convert the point respect to reference frame
        TranslationM = np.tile(Translation, (np.size(xyz_load,0),1))
        pcd_transfrom = np.dot(Rotation, xyz_load.T) + TranslationM.T
        pcd_transfrom = pcd_transfrom.T
   
        # point2grid
        pointGrid, heightGrid = volumecal.voxel_conv(pcd_transfrom, 0.05)

        print("heightGridbefore:  ",len(heightGrid),len(pointGrid))
        
        Target = 1
        IfTarget = 0
        num=1
        numm = 1
        
        while Target:
            num=num+1
            if IfTarget == 0:
                numm=numm+1
                #print("heightGrid:", heightGrid)
                if len(heightGrid)==0:
                    print("...................no proper food....................")
                    break
                maxgridindex = heightGrid.index(max(heightGrid))
               # print("maxgridindex:    ", maxgridindex)
                Max_gridlist = pointGrid[maxgridindex]
                Max_grid = np.array(Max_gridlist, dtype=np.float64)
                #print("Max_grid:    ", Max_grid)
                #print("maxgridindex:    ", maxgridindex)
               # print("Max_gridlist:  ", Max_gridlist)

                del pointGrid[maxgridindex]
                del heightGrid[maxgridindex]
            
             #   pointGrid.remove(Max_gridlist.all())
              #  heightGrid.remove(max(heightGrid))
                
                center_point = Max_grid[np.argmax(Max_grid[:,2])]
                print("center point:   ",center_point)
                indexfoodtrans = HSVdetect.ismember(pcd_transfrom,center_point, axis='row')
                centerori = xyz_load[indexfoodtrans]
                centerori = np.squeeze(centerori)

                print("center point index:   ",indexfoodtrans)
                print("center point origin:   ",centerori)

                InnerCircle = volumecal.InnerCircle(pcd_transfrom,center_point, 0.06)

                print("InnerCircle:   ",len(InnerCircle))

                InnerCircle = np.array(InnerCircle, dtype=np.float64)

                #Height_baselink = volumecal.depthcalc(InnerCircle, self.volume, 0.01)
                Depth = volumecal.depthcalcbyh(InnerCircle, self.H, center_point[2])
                print("depth:   ",Depth)

                indexinPC = HSVdetect.ismember(np_allpoints, centerori, axis='row')

                pixel_u = np.array(indexinPC) % widthros
                pixel_v = np.array(indexinPC) // widthros

                print("pixel in image:  ", pixel_u, pixel_v)
                print("pixel in roi:  ", pixel_u-self.roi_image[0], pixel_v-self.roi_image[1])

                centerinsource = np_allpoints[indexinPC]
                centerinsource = np.squeeze(centerinsource)

               # print("last:   ",indexinPC)
                print("centerinsource:",centerinsource)
                #Depth =center_point[2] - Height_baselink

                inputpixelidx = np.zeros(2)
                inputpixelidx[0] = pixel_u-self.roi_image[0]
                inputpixelidx[1] = pixel_v-self.roi_image[1]
                inputpixelidx = inputpixelidx.astype(np.int32)
                radius = self.radiusrario/centerinsource[2]
                print("inputpixelidx:    ",inputpixelidx, radius)
                radius = radius.astype(np.int32)
                end_time1 = time()
                IfTarget, detectedcolor = self.autocolordetect(inputpixelidx,radius)
                print("Iftarget:   ", IfTarget, detectedcolor)

                self.target_point = []

                self.target_point.append([inputpixelidx[0], inputpixelidx[1], Depth,0])

                print("target_point:    ", self.target_point)
                if IfTarget ==1:
                    with open('/home/rrc2/catkin_ws/src/color_detect/scripts/pickingpoint.txt', 'a') as f:
                        f.write(str(inputpixelidx[0])+str(',')+str(inputpixelidx[1])+str('\n'))
                
            else:
                Target = 0
                self.construct_msg(self.target_point)
                end_time2 = time()
                print("pcl time:   ", end_time1- start_time)
                print("pcl+image time:   ", end_time2-start_time)  
                print("Target: ", Target)
                # output coordinate and depth info to publisher
                if self.pubcount > 0:
                    print(".................publish the picking point..................")
                    self.pub.publish(self.poseList)
                    self.pubcount =0
              
            print("num:   ", num, numm)




       


    def image_cb(self, ros_image):
     #   rospy.loginfo("In Image callback function..")

        a = datetime.datetime.now()
        try:
            image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data)
            self.cv_image[:,:,0],self.cv_image[:,:,1],self.cv_image[:,:,2] = image[:,:,0],image[:,:,1],image[:,:,2] # change rgb to opencv bgr
        except CvBridgeError as e:
            print(e)

        HeightS, WidthS, ChannelS = self.cv_image.shape
        #print("big image:  ", HeightS, WidthS, ChannelS)

        # cv2.imshow("Image ", self.cv_image)
        # cv2.waitKey(10000)
        #cv2.imwrite('/home/zyadan/catkin_ws/src/color_detect/src/color/image.jpg',self.cv_image)


        

    def image2pixel(self,pointincamera):
        print("try")
        pointincamera = np.array(pointincamera, dtype=np.float64)
        Astra_Intrinsic = np.array([[577.0617065429688, 0.0, 317.3138122558594], 
                                    [0.0, 577.0617065429688, 239.10760498046875],
                                    [0.0, 0.0, 1.0]])
        pixelpoint = np.dot(Astra_Intrinsic,pointincamera.T)
        pixelpoint = pixelpoint.astype(np.int32)

        return pixelpoint 

        

    def colordetect(self, inputpixelidx, radius):
       
        flag = 0
        #inputpixelidx = np.array([80,150])
        
        imagecopy = self.cv_image.copy()
        Hs, Ws, Cs = imagecopy.shape
        #imageROI = imagecopy[np.max(0,inputpixelidx[0]-radius):np.min(Ws,inputpixelidx[0]+radius), np.max(0,inputpixelidx[1]-radius):np.min(Hs,inputpixelidx[1]+radius)]
        imageROI = imagecopy[inputpixelidx[0]-radius:inputpixelidx[0]+radius, inputpixelidx[1]-radius:inputpixelidx[1]+radius] 
        #cv2.imwrite('/home/zyadan/catkin_ws/src/color_detect/src/color/roi.jpg',imageROI)

        cv2.imshow("imageROI", imageROI)
        print("inputpixelidx:  ", inputpixelidx[0],inputpixelidx[1])

        HeightS, WidthS, ChannelS = imageROI.shape
        if HeightS==0 or WidthS==0:
            color=[]
            return flag, color
        print("imageROI info:  ", HeightS, WidthS, ChannelS)

        for row in range(HeightS): 
            for col in range(WidthS):
                dists = math.sqrt((row - radius)**2 + (col - radius)**2)
                
                if dists > radius:
                    imageROI[row, col] = 0

        cv2.imshow('ss',imageROI)
        cv2.waitKey(10)


        point_color = (0, 0, 255)
        thickness = 4 
        #cv2.circle(cv_image, [inputpixelidx[1],inputpixelidx[0]], radius, point_color, thickness)
        cv2.imshow('rr',self.cv_image)

        color = HSVdetect.get_color(imageROI)
        redin = 0
        for col in color:
            if col == "red" or col =="red2":
                redin = redin +1
        if redin>1:
            color.remove("red2")

        if len(color) >=3:
            flag = 1
            print("correct")
        else:
            flag = 0

        return flag, color


    def autocolordetect(self, inputpixelidx, radius):
       
        flag = 0
        #inputpixelidx = np.array([80,150])
        
        imagecopy = self.cv_image.copy()
        Hs, Ws, Cs = imagecopy.shape
        #imageROI = imagecopy[np.max(0,inputpixelidx[0]-radius):np.min(Ws,inputpixelidx[0]+radius), np.max(0,inputpixelidx[1]-radius):np.min(Hs,inputpixelidx[1]+radius)]
        imageROI = imagecopy[inputpixelidx[1]-radius:inputpixelidx[1]+radius, inputpixelidx[0]-radius:inputpixelidx[0]+radius] 
        #cv2.imwrite('/home/zyadan/catkin_ws/src/color_detect/src/color/roi.jpg',imageROI)

        print("input pixel:  ", inputpixelidx[0],inputpixelidx[1])
        print("input roi pixel:  ", inputpixelidx[0]-radius, inputpixelidx[1]-radius, inputpixelidx[0]+radius, inputpixelidx[1]+radius)

        HeightS, WidthS, ChannelS = imageROI.shape
        if (inputpixelidx[0]-radius)<=0 or (inputpixelidx[1]-radius)<=0 or (inputpixelidx[0]+radius)>=Ws or (inputpixelidx[1]+radius)>=Hs:
            color=[]
            return flag, color
        print("imageROI info:  ", HeightS, WidthS, ChannelS)
        outpixelnum = 0
        for row in range(HeightS): 
            for col in range(WidthS):
                dists = math.sqrt((row - radius)**2 + (col - radius)**2)
                
                if dists > radius:
                    imageROI[col,row] = 0
                    outpixelnum +=1
        imagecopy = cv2.rectangle(imagecopy, (inputpixelidx[0]-radius, inputpixelidx[1]-radius), (inputpixelidx[0]+radius, inputpixelidx[1]+radius), (0,0,255), 1)
        
        pixelnum = HeightS * WidthS - outpixelnum
        

        color,_,_ = HSVdetect.auto_colorsegment(imageROI, pixelnum)

        color_new = list(dict.fromkeys(color))
        for i in range(len(color)):
            if color[i] =="red2":
                color[i] = "red"
        color_new = list(dict.fromkeys(color))      
        if len(color_new) >=3:
            flag = 1
            print("correct")
        else:
            flag = 0

        # imagecopy = cv2.resize(imagecopy, (633, 573))  
        # cv2.imshow('ss',imagecopy)
        # cv2.waitKey(10)
        # print("color detected: ", color)

        if flag ==1:
          #  imagecopy = cv2.resize(imagecopy, (633, 573)) #large 
            imagecopy = cv2.resize(imagecopy, (486, 615))#small
            cv2.imshow('ss',imagecopy)
            cv2.waitKey(10)

        return flag, color_new




    





    '''
    def construct_msg(self, ilist):
        plist = []
        center = Center()
        center.x = ilist[0]
        center.y = ilist[1]
        center.z = ilist[2]
        center.depth = ilist[3]
        plist.append(center) 

        self.centerList.center_list = plist

    '''

    def construct_msg(self, ilist):
        plist = []
        pose = Pose()
        pose.Class = "object_1"
        pose.cx = ilist[0][0]
        pose.cy = ilist[0][1]
        pose.cz = ilist[0][2]
        pose.angle = 0
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
