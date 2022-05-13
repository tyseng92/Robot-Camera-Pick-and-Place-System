#!/usr/bin/env python

import numpy as np 
import time
import cv2 
#from auto_label import write
import rospy

class ObjLocation(object):
    def __init__(self):
        #cv2.namedWindow('threshold')
        #cv2.createTrackbar('Threshold','threshold',0,255,nothing)
        self.food_list = dict()
        # Image processing parameter
        opcv = rospy.get_param('/auto_label/cv_param')
        self.thd, self.blk, self.ksize, self.ksize_2, self.boxmin, self.boxmax, self.bXr, self.bYt, self.bXl, self.bYb = opcv['thd'], opcv['blk'], opcv['ksize'], opcv['ksize_2'], opcv['boxmin'], opcv['boxmax'], opcv['bXr'], opcv['bYt'], opcv['bXl'], opcv['bYb']

    def nothing(self, x):
        pass

    # create max box
    def maxbox_single(self, b_list):
        max_list = [1280,720,0,0]
        for point in b_list:
            # In here get min of the coordinates for bx and by
            for i in range(0,2): 
                if max_list[i] > point[i]:
                    max_list[i] = point[i] 
            # In here get max of the coordinates for bx+bw and by+bh
            for i in range(2,4): 
                if max_list[i] < point[i]:
                    max_list[i] = point[i] 
        return max_list[0], max_list[1], max_list[2]-max_list[0], max_list[3]-max_list[1]  

    # create max box
    def maxboxes(self, b_list):
        correct_box = []
        #outside_box = [] 
        #max_list = b_list.pop(0)
        outside_box = b_list
        print outside_box
        while not rospy.is_shutdown() and outside_box:
            max_list = outside_box.pop(0)
            print outside_box
            boxes = outside_box
            outside_box = []   # initialize list
            for box in boxes:
                # condition for non-intersection of boxes
                left = box[0] < max_list[0] and box[2] < max_list[0]
                right = box[0] > max_list[2] and box[2] > max_list[2]
                bottom = box[1] > max_list[3] and box[3] > max_list[3]
                top = box[1] < max_list[1] and box[3] < max_list[1]
                # if box intersect to each other
                if not (left or right or bottom or top):        
                    # In here get min of the coordinates for bx and by
                    for i in range(0,2): 
                        if max_list[i] > box[i]:
                            max_list[i] = box[i] 
                    # In here get max of the coordinates for bx+bw and by+bh
                    for i in range(2,4): 
                        if max_list[i] < box[i]:
                            max_list[i] = box[i]
                else:
                    outside_box.append(box)
            #mlist = [max_list[0], max_list[1], max_list[2]-max_list[0], max_list[3]-max_list[1]]
            correct_box.append(max_list)

        return correct_box   

    def find_coordinate(self, path, track=False):

        img = cv2.imread(path, cv2.IMREAD_UNCHANGED)

        height, width, channels = img.shape
        #print (height, width, channels)

        imgblur = cv2.bilateralFilter(img,17,75,75)
        #cv2.imshow("blur_img", imgblur)

        # Adaptive thresholding blk=11, thd=2
        threshed_img = cv2.adaptiveThreshold(cv2.cvtColor(imgblur, cv2.COLOR_BGR2GRAY),255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY_INV,self.blk,self.thd)       

        # morphological transformation
        ksize = self.ksize
        ksize_2 = self.ksize_2
        #kernel = np.ones((ksize,ksize),np.uint8)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(ksize,ksize))
        kernel_0 = np.ones((0,0),np.uint8)
        kernel_2 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(ksize_2,ksize_2))
        #opening_img = cv2.morphologyEx(threshed_img, cv2.MORPH_OPEN, kernel_0)
        #dilation = cv2.dilate(threshed_img,kernel,iterations = 1)
        closing_img = cv2.morphologyEx(threshed_img, cv2.MORPH_CLOSE, kernel)
        #erosion_img = cv2.erode(closing_img,kernel_2,iterations = 1)
        #opening_img = cv2.morphologyEx(closing_img, cv2.MORPH_OPEN, kernel_2)
        contour_img = closing_img
        #contour_img = threshed_img
        #contour_img = dilation

        contour_img[:,:self.bXl] = 0
        contour_img[:,width-self.bXr:] = 0
        contour_img[:self.bYt,:] = 0
        contour_img[height-self.bYb:,:] = 0
        # Find contours and get the external one
        image, contours, hier = cv2.findContours(contour_img,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        #image, contours, hier = cv2.findContours(contour_img,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)

        #num = 0
        box_list = []
        moment_list = []
        angle_list = []
        for i, c in enumerate(contours):
            
            # get the bounding rect
            bx, by, bw, bh = cv2.boundingRect(c)
            # filter box smaller than boxlim length
            if bw < self.boxmin or bh < self.boxmin: 
                continue

            # filter box bigger than boxmax length
            if bw > self.boxmax or bh > self.boxmax: 
                continue

            # filter boxes appear at sides of photo
            if bx < self.bXl or bx+bw > width-self.bXr or by < self.bYt or by+bh> height-self.bYb:
                continue

            if track:
                # draw a green rectangle to visualize the bounding rect
                cv2.rectangle(img, (bx, by), (bx+bw, by+bh), (0, 255, 0), 2)
                
                continue

            # store valid boxes in list to compare for max box
            box_list.append([bx,by,bx+bw,by+bh])
 
            # get centroids
            M = cv2.moments(c)
            moment_list.append(M)
            
            # get orientation
            rect = cv2.minAreaRect(c)  # output: ( top-left corner(x,y), (width, height), angle of rotation ) 
            if rect[1][0] < rect[1][1]:
                angle = rect[2] + 180 
            else:
                angle = rect[2] + 90
            angle_list.append(angle)
            
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            cv2.drawContours(img,[box],0,(0,0,255),2)

        if track:        
            img = cv2.line(img,(width-self.bXr,0),(width-self.bXr,530),(0,0,255),5)
            img = cv2.line(img,(0,self.bYt),(522,self.bYt),(0,0,255),5)
            img = cv2.line(img,(0,height-self.bYb),(522,height-self.bYb),(0,0,255),5)
            img = cv2.line(img,(self.bXl,0),(self.bXl,530),(0,0,255),5)

            #img = cv2.line(img,(0,360),(1280,360),(255,0,255),5)
            #img = cv2.line(img,(640,0),(640,720),(255,0,255),5)

            #img2 = cv2.resize(img, (920, 500))

            cv2.imshow("blur_img", imgblur)
            cv2.imshow("contour_img", contour_img) 
            cv2.imshow('threshold',img)
            
            return   

        print("Angle_list: " + str(angle_list))
        #print(box_list)
        #x,y,w,h = self.maxbox(box_list)
        
        #mboxes = self.maxboxes(box_list)

        for i, bx in enumerate(box_list):
            x = bx[0]
            y = bx[1]
            xw = bx[2]
            yh = bx[3]
            # draw a green rectangle to visualize the bounding rect
            cv2.rectangle(img, (x, y), (xw, yh), (0, 255, 0), 2)
            name = "object_"+str(i)
            self.food_list[name] = [str(x), str(y), str(xw), str(yh)]

        #for i, bx in enumerate(mboxes):
        #    x = bx[0]
        #    y = bx[1]
        #    xw = bx[2]
        #    yh = bx[3]
            # draw a green rectangle to visualize the bounding rect
        #    cv2.rectangle(img, (x, y), (xw, yh), (0, 255, 0), 2)
        #    name = "object_"+str(i)
        #    self.food_list[name] = [str(x), str(y), str(xw), str(yh)]

        if len(box_list)==0:
            return None
            #multiple food
            #if num < len(self.food_name):
                #self.food_list[self.food_name[num]] = [str(x), str(y), str(x+w), str(y+h)]
                #print("num of food: " + str(len(self.food_list)) + ' ' + str(x) + ' ' +str(y))
                #num=num+1
      
        #print("num of contours: " + str(len(contours)))
        #write(self.food_list, path)
        return self.food_list

        cv2.drawContours(img, contours, -1, (255, 255, 0), 1)

        #return centroid
        M = moment_list[0]
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        cv2.circle(img,(cx,cy), 5, (0,0,255), -1)

        img2 = cv2.resize(img, (920, 500)) 
        cv2.namedWindow('Label')
        cv2.imshow('Label',img2)
        cv2.moveWindow('Label',200,200)
        cv2.waitKey(2000)
        #cv2.waitKey(-1)
        
        #cv2.imshow('threshold',img2) 
        #cv2.imshow("contours", img)
        #time.sleep(0.5)

        return [cx, cy, angle_list[0]] 
        #return [x+((xw-x)/2), y+((yh-y)/2), grasp]

    def trackbar(self, img_path):
        #opcv = rospy.get_param('/object_detect/cv_param')
        #thd, blk, boxmin, boxmax, bXr, bYt, bXl, bYb = opcv['thd'], opcv['blk'], opcv['boxmin'], opcv['boxmax'], opcv['bXr'], opcv['bYt'], opcv['bXl'], opcv['bYb']

        cv2.namedWindow('threshold')
        
        cv2.createTrackbar('Threshold','threshold',0,255,self.nothing)
        cv2.createTrackbar('AdaptBlockSize','threshold',0,522,self.nothing)
        cv2.createTrackbar('KernelSize','threshold',0,522,self.nothing)
        cv2.createTrackbar('KernelSize_2','threshold',0,522,self.nothing)
        cv2.createTrackbar('BoxSizeMin','threshold',0,522,self.nothing)
        cv2.createTrackbar('BoxSizeMax','threshold',0,522,self.nothing)
        cv2.createTrackbar('EdgeCancelXr','threshold',0,522,self.nothing)
        cv2.createTrackbar('EdgeCancelYt','threshold',0,530,self.nothing)
        cv2.createTrackbar('EdgeCancelXl','threshold',0,522,self.nothing)
        cv2.createTrackbar('EdgeCancelYb','threshold',0,530,self.nothing)

        # Initialize variables
        cv2.setTrackbarPos('Threshold','threshold', self.thd)
        cv2.setTrackbarPos('AdaptBlockSize','threshold', self.blk) #501
        cv2.setTrackbarPos('KernelSize','threshold', self.ksize)
        cv2.setTrackbarPos('KernelSize_2','threshold', self.ksize_2)
        cv2.setTrackbarPos('BoxSizeMin','threshold', self.boxmin) #133
        cv2.setTrackbarPos('BoxSizeMax','threshold', self.boxmax)
        cv2.setTrackbarPos('EdgeCancelXr','threshold', self.bXr)
        cv2.setTrackbarPos('EdgeCancelYt','threshold', self.bYt)
        cv2.setTrackbarPos('EdgeCancelXl','threshold', self.bXl)
        cv2.setTrackbarPos('EdgeCancelYb','threshold', self.bYb)

        # Track bar loop
        while not rospy.is_shutdown():     
            k = cv2.waitKey(1) & 0xFF
            if k == 27:
                break
            # Threshold image
            self.thd = cv2.getTrackbarPos('Threshold','threshold')
            self.blk = cv2.getTrackbarPos('AdaptBlockSize','threshold')
            self.ksize = cv2.getTrackbarPos('KernelSize','threshold')
            self.ksize_2 = cv2.getTrackbarPos('KernelSize_2','threshold')
            self.boxmin = cv2.getTrackbarPos('BoxSizeMin','threshold')
            self.boxmax = cv2.getTrackbarPos('BoxSizeMax','threshold')
            self.bXr = cv2.getTrackbarPos('EdgeCancelXr','threshold')
            self.bYt = cv2.getTrackbarPos('EdgeCancelYt','threshold')
            self.bXl = cv2.getTrackbarPos('EdgeCancelXl','threshold')
            self.bYb = cv2.getTrackbarPos('EdgeCancelYb','threshold')
     
            if self.blk%2 == 0 or self.blk <= 1:
                continue
        
            self.find_coordinate(img_path, track=True)

            time.sleep(0.5)

        cv2.destroyAllWindows()

    def display_center(self, Img):
        while not rospy.is_shutdown():     
            k = cv2.waitKey(1) & 0xFF
            if k == 27:
                break
            img = Img.display()
            img = cv2.line(img,(0,360),(1280,360),(255,0,255),5)
            img = cv2.line(img,(640,0),(640,720),(255,0,255),5)
        cv2.destroyAllWindows()
    
