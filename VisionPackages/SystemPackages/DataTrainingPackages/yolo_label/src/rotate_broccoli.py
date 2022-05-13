import cv2
import os

path = os.path.join(os.environ["HOME"],"Desktop/image_database/yolo/DATABASE_DEC/zed_new/img_database_broccoli_5")
if not os.path.exists(path):
    os.makedirs(path)

width = 328
height = 328


first = 1309
last = 1328
lastImgInData = 1328

item_name = "Broccoli"
# offset

fx = 0
fy = 0

total = last - first + 1

def delete(filepath):
    if os.path.exists(filepath):
        os.remove(filepath)

def rot_txt(i, num, angle):
    lname = item_name + "_" + str(i) + ".txt"
    label = os.path.join(path, lname)
    lname2 = item_name + "_" + str(num) + ".txt"
    label2 = os.path.join(path, lname2)
    with open(label) as f:
        flist = f.read().splitlines()
        delete(label2)
        for fl in flist:
            ulist = fl.split()
            rlist = convert(ulist, angle)
            with open(label2, 'a') as f:  
                f.write(' '.join(rlist) + str('\n'))
                #f.write('\n')
                
# x, y, x0, y0 are normalized value    
def convert(ulist, angle):
    x0 = float(ulist[1])
    y0 = float(ulist[2])
    print x0, y0
    if angle == 90:
        x = round(y0 - fx/float(height), 6)
        y = round(1.0 - x0 + fy/float(width), 6)
        swap = {'15':'21','16':'22','17':'15','18':'16','19':'17','20':'18','21':'19','22':'20','23':'25','24':'26','25':'23','26':'24','27':'29','28':'30','29':'27','30':'28'}
        cls = swap[ulist[0]] 
        return [cls, str(x), str(y), ulist[4] , ulist[3]]
        
    elif angle == 180:
        x = round(1.0 - x0 - fx/float(height), 6)
        y = round(1.0 - y0 + fy/float(width), 6)
        swap = {'15':'19','16':'20','17':'21','18':'22','19':'15','20':'16','21':'17','22':'18','23':'23','24':'24','25':'25','26':'26','27':'27','28':'28','29':'29','30':'30'}
        cls = swap[ulist[0]]
        return [cls, str(x), str(y), ulist[3] , ulist[4]]
        
    elif angle == 270:
        x = round(1.0 - y0 - fx/float(height), 6)
        y = round(x0 + fy/float(width), 6)
        swap = {'15':'17','16':'18','17':'19','18':'20','19':'21','20':'22','21':'15','22':'16','23':'25','24':'26','25':'23','26':'24','27':'29','28':'30','29':'27','30':'28'}      
        cls = swap[ulist[0]] 
        return [cls, str(x), str(y), ulist[4] , ulist[3]]
        

for i in range(first, last+1):
    imgname = item_name + "_" + str(i) + ".jpg"
    imgpath = os.path.join(path, imgname)
    img = cv2.imread(imgpath)
    
    (h, w) = img.shape[:2]
    print("Current index: " + str(i))
    print("h,w: {} {}".format(h,w))
    center = (w/2, h/2)
    scale = 1.0
    period = 0
    
    for angle in [90, 180, 270]:
        M = cv2.getRotationMatrix2D(center, angle, scale)
        if angle == 90 or angle == 270:
            rotated = cv2.warpAffine(img, M, (h, w))
        elif angle == 180:
            rotated = cv2.warpAffine(img, M, (w, h))
        num = lastImgInData + (total*period) + (i - first + 1)
        period += 1
        rname = item_name + "_" + str(num) + ".jpg"
        rpath = os.path.join(path, rname) 
        cv2.imwrite(rpath,rotated)
        rot_txt(i, num, angle)
        print("Index: {}",format(i))
        
print "Done"
