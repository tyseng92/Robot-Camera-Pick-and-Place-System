# this script is to create padding on rectangular image, changing from rectangle image to square image for rotation purpose.

import cv2 
import numpy 
import os

# source folder
path = os.path.join(os.environ["HOME"],"Desktop/image_database/yolo/test_padding/label")
if not os.path.exists(path):
    os.makedirs(path)

# destination folder
path2 = os.path.join(os.environ["HOME"],"Desktop/image_database/yolo/test_padding/padding")
if not os.path.exists(path):
    os.makedirs(path)

width = 397
height = 276


first = 1365
last = 1375

def delete(filepath):
    if os.path.exists(filepath):
        os.remove(filepath)

def pad_val():
    if width > height:
        return (width-height)/2
    else:
        return (height-width)/2

def pad_txt(i):
    lname = "Image_" + str(i) + ".txt"
    label = os.path.join(path, lname)
    lname2 = "Image_" + str(i) + ".txt"
    label2 = os.path.join(path2, lname2)
    with open(label) as f:
        flist = f.read().splitlines()
        delete(label2)
        for fl in flist:
            ulist = fl.split()
            rlist = convert(ulist)
            with open(label2, 'a') as f:  
                f.write(' '.join(rlist) + str('\n'))
            
def convert(ulist):
    x0 = float(ulist[1])
    y0 = float(ulist[2])
    w0 = float(ulist[3])
    h0 = float(ulist[4])

    print x0, y0, w0, h0

    pad = pad_val()

    if width > height:
        H = 2*pad+height   
        h = round((int(h0*height))/float(H), 6)     
        y = round((int(y0*height)+pad)/float(H), 6)
        return [ulist[0], ulist[1], str(y), ulist[3], str(h)]
    else:   
        W = 2*pad+width  
        w = round((int(w0*width))/float(W), 6)       
        x = round((int(x0*width)+pad)/float(W), 6)
        return [ulist[0], str(x), ulist[2], str(w), ulist[4]]
        

for i in range(first, last+1):
    imgname = "Image_" + str(i) + ".jpg"
    imgpath = os.path.join(path, imgname)
    img = cv2.imread(imgpath)
    
    pad = int(pad_val())
    print("pad: {}".format(pad)) 
    replicate = cv2.copyMakeBorder(img,pad,pad,0,0,cv2.BORDER_REPLICATE)
        
    # display img info
    #(h, w) = replicate.shape[:2]
    #print("h,w: {} {}".format(h,w))

    #cv2.imshow("example", replicate)
    #cv2.waitKey()
    
    # save padded images
    save_path = os.path.join(path2, imgname) 
    cv2.imwrite(save_path, replicate)
    pad_txt(i)


        
print "Done"
