import cv2
import os

from PIL import Image, ImageEnhance
import numpy as np
import random
import threading, os, time

path = os.path.join(os.environ["HOME"],"Desktop/image_database/yolo/DATABASE_DEC/zed/img_database_broccoli")
pathN = os.path.join(os.environ["HOME"],"Desktop/image_database/yolo/DATABASE_DEC/zed/temp")
if not os.path.exists(path):
    os.makedirs(path)

width = 328
height = 328

widthN = 328
heightN = 328

ratioW = float(width)/widthN
ratioH = float(height)/heightN


first = 1
last = 10
lastImgInData =2

item_name = "Broccoli"
item_nameN = "Broccoli"

# offset

fx = 10
fy = 30

total = last - first + 1

def delete(filepath):
    if os.path.exists(filepath):
        os.remove(filepath)

def res_txt(i, num):
    lname = item_name + "_"  + str(i) + ".txt"
    label = os.path.join(path, lname)
    lname2 = item_nameN + "_" + str(num) + ".txt"
    label2 = os.path.join(pathN, lname2)
    with open(label) as f:
        flist = f.read().splitlines()
        delete(label2)
        for fl in flist:
            ulist = fl.split()
            rlist = convert(ulist)
            with open(label2, 'a') as f:  
                f.write(' '.join(rlist) + str('\n'))
                #f.write('\n')

def randomColor(image):

    random_factor = np.random.randint(0, 21)/10.
    color_image = ImageEnhance.Color(image).enhance(random_factor) 
    random_factor = np.random.randint(20, 31) / 10.
    brightness_image = ImageEnhance.Brightness(color_image).enhance(random_factor) 
    random_factor = np.random.randint(10, 21) / 10.
    contrast_image = ImageEnhance.Contrast(brightness_image).enhance(random_factor)  
    random_factor = np.random.randint(0, 31) / 10.
    return ImageEnhance.Sharpness(brightness_image).enhance(random_factor) 
                
# x, y, x0, y0 are normalized value    
def convert(ulist):
    x0 = float(ulist[1])
    y0 = float(ulist[2])

  #  print x0, y0
    width0 = float(ulist[3])
    height0 = float(ulist[4])    
    x = round((x0*float(width)-fx)/ float(widthN) ,6)
    y = round((y0*float(height)-fy) / float(heightN) ,6)
    widthB = round(width0 * ratioH ,6)
    heightB = round(height0 * ratioW ,6)
 #   print("x,y: {} {}".format(x,y))

    return [ulist[0], str(x), str(y), str(widthB), str(heightB)]
    


        

for i in range(first, last+1):
    imgname = item_name + "_"  + str(i) + ".jpg"
    imgpath = os.path.join(path, imgname)
    img=Image.open(imgpath)
    
    
 #   res = img[fy:(328+fy),fx:(fx+328)]
  #  res=cv2.resize(img,(328,328),interpolation=cv2.INTER_CUBIC)

    res = randomColor(img)
    num = i 
    rname = item_nameN + "_"  + str(num) + ".jpg"
    rpath = os.path.join(pathN, rname) 
    res.save(rpath)
  #  cv2.imwrite(rpath,res)
   # res_txt(i, num)

        
print "Done"
