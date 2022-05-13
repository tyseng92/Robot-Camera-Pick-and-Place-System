#!/usr/bin/env python

from bounding_box import ObjLocation
import os
import cv2

path = os.path.join(os.environ["HOME"],"Desktop/image_database/yolo/an_img_db/single")
first = 1
last = 368

def label(i, cls, x, y, w, h, W, H):
    lname = "Image_" + str(i) + ".txt"
    label = os.path.join(path, lname)
    #print x, y, w, h, W, H
    print i
    with open(label, "w") as f:
        f.write(str(cls)+ ' ' +str(round(x/float(W), 6))+ ' ' +str(round(y/float(H), 6))+ ' ' +str(round(w/float(W), 6))+ ' ' +str(round(h/float(H), 6)))  

def cls(i):
        
    if i >= 1 and i <= 7:
        return 0        
    if i >= 8 and i <= 15:
        return 1
    if i >= 16 and i <= 22:
        return 2
    if i >= 23 and i <= 30:
        return 3
    if i >= 31 and i <= 37:
        return 4
    if i >= 38 and i <= 45:
        return 5
    if i >= 46 and i <= 52:
        return 0        
    if i >= 53 and i <= 60:
        return 1
    if i >= 61 and i <= 67:
        return 2
    if i >= 68 and i <= 75:
        return 3
    if i >= 76 and i <= 82:
        return 4
    if i >= 83 and i <= 90:
        return 5
    if i >= 91 and i <= 97:
        return 0        
    if i >= 98 and i <= 105:
        return 1
    if i >= 106 and i <= 112:
        return 2
    if i >= 113 and i <= 120:
        return 3
    if i >= 121 and i <= 127:
        return 4
    if i >= 128 and i <= 135:
        return 5
    if i >= 136 and i <= 142:
        return 0        
    if i >= 143 and i <= 150:
        return 1
    if i >= 151 and i <= 157:
        return 2
    if i >= 158 and i <= 165:
        return 3
    if i >= 166 and i <= 172:
        return 4
    if i >= 173 and i <= 181:
        return 5
    if i >= 182 and i <= 188:
        return 0        
    if i >= 189 and i <= 196:
        return 1
    if i >= 197 and i <= 203:
        return 2
    if i >= 204 and i <= 211:
        return 3
    if i >= 212 and i <= 218:
        return 4
    if i >= 219 and i <= 226:
        return 5
    if i >= 227 and i <= 233:
        return 0        
    if i >= 234 and i <= 241:
        return 1
    if i >= 242 and i <= 248:
        return 2
    if i >= 249 and i <= 256:
        return 3
    if i >= 257 and i <= 263:
        return 4
    if i >= 264 and i <= 271:
        return 5
    if i >= 272 and i <= 278:
        return 0        
    if i >= 279 and i <= 286:
        return 0
    if i >= 287 and i <= 293:
        return 1
    if i >= 294 and i <= 301:
        return 2
    if i >= 302 and i <= 308:
        return 3
    if i >= 309 and i <= 316:
        return 4
    if i >= 317 and i <= 323:
        return 5        
    if i >= 324 and i <= 331:
        return 0        
    if i >= 332 and i <= 338:
        return 1
    if i >= 339 and i <= 346:
        return 2
    if i >= 347 and i <= 353:
        return 3
    if i >= 354 and i <= 361:
        return 4
    if i >= 362 and i <= 368:
        return 5
       

def main():
    fail = []
    more = []
    for i in range(first, last+1):
        imgname = "Image_" + str(i) + ".jpg"
        imgpath = os.path.join(path, imgname)
        img = cv2.imread(imgpath)
        (H, W) = img.shape[:2]
        obj = ObjLocation()
        #obj.trackbar(imgpath)
        blist = obj.find_coordinate(imgpath)
        if len(blist) > 1:
            print("More: " + str(i))
            more.append(i)
            continue
        if blist == None:
            print("Fail: " + str(i))
            fail.append(i)
            continue
        for n, l in blist.iteritems():
            global x, y, w, h
            x = (int(l[0]) + int(l[2]))/2
            y = (int(l[1]) + int(l[3]))/2
            w = int(l[2]) - int(l[0])
            h = int(l[3]) - int(l[1])
        c = cls(i)
        
        label(i, c, x, y, w, h, W, H)
    print("Fail: "+ str(fail))
    print("More: "+ str(more))

    
def main_track():
    imgname = "Image_" + str(1) + ".jpg"
    imgpath = os.path.join(path, imgname)
    obj = ObjLocation()
    obj.trackbar(imgpath)    

if __name__ == "__main__":
    main()
    #main_track()


