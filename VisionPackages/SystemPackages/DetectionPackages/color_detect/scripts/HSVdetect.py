import numpy as np
import cv2
import collections
import matplotlib.pyplot as plt
from scipy import signal


def getColorList():
    dict = collections.defaultdict(list)
 
    # black
    #lower_black = np.array([0, 0, 0])
    #upper_black = np.array([180, 255, 46])
    #color_list = []
    #color_list.append(lower_black)
    #color_list.append(upper_black)
    #dict['black'] = color_list
 
    # gray
    # lower_gray = np.array([0, 0, 46])
    # upper_gray = np.array([180, 43, 220])
    # color_list = []
    # color_list.append(lower_gray)
    # color_list.append(upper_gray)
    # dict['gray']=color_list
 
    # # white
    # lower_white = np.array([0, 0, 221])
    # upper_white = np.array([180, 30, 255])
    # color_list = []
    # color_list.append(lower_white)
    # color_list.append(upper_white)
    # dict['white'] = color_list
 
    # red
    lower_red = np.array([156, 43, 46])
    upper_red = np.array([180, 255, 255])
    color_list = []
    color_list.append(lower_red)
    color_list.append(upper_red)
    dict['red']=color_list
 
    # red2
    lower_red = np.array([0, 43, 46])
    upper_red = np.array([10, 255, 255])
    color_list = []
    color_list.append(lower_red)
    color_list.append(upper_red)
    dict['red2'] = color_list
 
    #orange
    lower_orange = np.array([11, 43, 46])
    upper_orange = np.array([15, 255, 255])#25,15
    color_list = []
    color_list.append(lower_orange)
    color_list.append(upper_orange)
    dict['orange'] = color_list
 
    #yellow
    lower_yellow = np.array([16, 43, 46])#26,16
    upper_yellow = np.array([27, 255, 255])
    color_list = []
    color_list.append(lower_yellow)
    color_list.append(upper_yellow)
    dict['yellow'] = color_list
 
    #green
    lower_green = np.array([28, 43, 46])#35,28
    upper_green = np.array([77, 255, 255])
    color_list = []
    color_list.append(lower_green)
    color_list.append(upper_green)
    dict['green'] = color_list
 
    # cyan
    lower_cyan = np.array([78, 43, 46])
    upper_cyan = np.array([99, 255, 255])
    color_list = []
    color_list.append(lower_cyan)
    color_list.append(upper_cyan)
    dict['cyan'] = color_list
 
    # blue
    lower_blue = np.array([100, 43, 46])
    upper_blue = np.array([124, 255, 255])
    color_list = []
    color_list.append(lower_blue)
    color_list.append(upper_blue)
    dict['blue'] = color_list
 
    # purple
    lower_purple = np.array([125, 43, 46])
    upper_purple = np.array([155, 255, 255])
    color_list = []
    color_list.append(lower_purple)
    color_list.append(upper_purple)
    dict['purple'] = color_list
 
    return dict

#The upper and lower bound is slightly changed and remove white, black and gray
def getColorListQ3():
    dict = collections.defaultdict(list)
 
    # red
    lower_red = np.array([156, 43, 46])
    upper_red = np.array([180, 255, 255])
    color_list = []
    color_list.append(lower_red)
    color_list.append(upper_red)
    dict['red']=color_list
 
    # red2
    lower_red = np.array([0, 43, 46])
    upper_red = np.array([10, 255, 255])
    color_list = []
    color_list.append(lower_red)
    color_list.append(upper_red)
    dict['red2'] = color_list
 
    #orange
    lower_orange = np.array([11, 43, 46])
    upper_orange = np.array([15, 255, 255])#25,15
    color_list = []
    color_list.append(lower_orange)
    color_list.append(upper_orange)
    dict['orange'] = color_list
 
    #yellow
    lower_yellow = np.array([16, 43, 46])#26,18
    upper_yellow = np.array([28, 255, 255])#34,28
    color_list = []
    color_list.append(lower_yellow)
    color_list.append(upper_yellow)
    dict['yellow'] = color_list
 
    #green
    lower_green = np.array([29, 43, 46])#35,29
    upper_green = np.array([77, 255, 255])
    color_list = []
    color_list.append(lower_green)
    color_list.append(upper_green)
    dict['green'] = color_list
 
    # cyan
    lower_cyan = np.array([78, 43, 46])
    upper_cyan = np.array([99, 255, 255])
    color_list = []
    color_list.append(lower_cyan)
    color_list.append(upper_cyan)
    dict['cyan'] = color_list
 
    # blue
    lower_blue = np.array([100, 43, 46])
    upper_blue = np.array([124, 255, 255])
    color_list = []
    color_list.append(lower_blue)
    color_list.append(upper_blue)
    dict['blue'] = color_list
 
    # purple
    lower_purple = np.array([125, 43, 46])
    upper_purple = np.array([155, 255, 255])
    color_list = []
    color_list.append(lower_purple)
    color_list.append(upper_purple)
    dict['purple'] = color_list
 
    return dict

# check whether white is in 
def ifwhitein(img_hsv, pixelnum):
    lower_white = np.array([0, 0, 221])
    upper_white = np.array([180, 30, 255])

    mask = cv2.inRange(img_hsv, lowerb=lower_white, upperb=upper_white)
    binary = cv2.threshold(mask, 127, 255, cv2.THRESH_BINARY)[1]
    binary = cv2.dilate(binary, None, iterations=2)

    cnts, hiera = cv2.findContours(binary.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    sum = 0
    for c in cnts:
        sum += cv2.contourArea(c)
    colorper = round(sum / pixelnum, 2)
    colorname = 'white'
#    color_list.append(np.array(colorper))

    return colorname, colorper

# get color based on the fixed boundary of colors
def get_color(frame):
    print('go in get_color')
    hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    maxsum =  200
    color = list()
    color_dict = getColorList()

    for d in color_dict:
        mask = cv2.inRange(hsv,color_dict[d][0],color_dict[d][1])

        #cv2.imwrite('/home/zyadan/catkin_ws/src/color_detect/src/color/'+d+'.jpg',mask)
        binary = cv2.threshold(mask, 127, 255, cv2.THRESH_BINARY)[1]
        binary = cv2.dilate(binary,None,iterations=2)
        print(d)
        _, cnts, hiera = cv2.findContours(binary.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        sum = 0
        for c in cnts:
            sum+=cv2.contourArea(c)
        if sum > maxsum :
            
            color.append(d)
 
    return color

# to check whether the element is in M matrix 
def ismember(M,element, axis='row'):
    indexinA = np.argwhere(M==element)
    sizeB = np.size(element)
    if axis == 'row':
        indexinAB = [i[0] for i in indexinA]
        indexinAD = dict(collections.Counter(indexinAB))
        RepValue = [key for key,value in indexinAD.items() if value ==sizeB ]

    if axis == 'single':
        print('single')

    return RepValue

# choose the main color in th detected colordict
def get_detectedcolor(frame,pixelnum, detected_colordict):
    print('get color based detected boundary')
    hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    maxpercent = 0.20
    color = list()
    colorpercent = list()
    color_dict = detected_colordict

    for d in color_dict:
        mask = cv2.inRange(hsv, color_dict[d][0], color_dict[d][1])

        binary = cv2.threshold(mask, 127, 255, cv2.THRESH_BINARY)[1]
        binary = cv2.dilate(binary, None, iterations=2)
        print(d)
        cnts, hiera = cv2.findContours(binary.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        sum = 0
        for c in cnts:
            sum += cv2.contourArea(c)
        colorper = round(sum / pixelnum, 2)

        if colorper > maxpercent:
            color.append(d)
            colorpercent.append(colorper)

    return color, colorpercent


def auto_colorsegment(image, pixelnum, s_min = 90, s_max = 255, v_min = 46, v_max = 255):
        Height, Width, Channel = image.shape
        img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        img_h = img_hsv[..., 0]
        img_s = img_hsv[..., 1]
        img_v = img_hsv[..., 2]

        fig = plt.gcf()  # 分通道显示图片
        fig.set_size_inches(20, 20)

        # plt.imshow(img_hsv)
        # plt.axis('off')
        # plt.title('HSV')
        img_s = np.array(img_s)
        img_v = np.array(img_v)

        mask = np.zeros([Height, Width], dtype=np.uint8)
        for i in range(Height):
            for j in range(Width):
                if np.logical_and(np.logical_and(img_s[i, j] > s_min, img_s[i, j] < s_max),
                                np.logical_and(img_v[i, j] > v_min, img_v[i, j] < v_max)):
                    mask[i, j] = 1


        img_hmask = cv2.add(img_h, np.zeros(np.shape(img_h), dtype=np.uint8), mask=mask)
        # cv2.imshow('img_h', img_hmask)
        # c = cv2.waitKey(40)

        H_hist = cv2.calcHist([img_hmask], [0], mask, [181], [0, 181])

        H_hist_np = np.array(H_hist)
        H_hist_np = np.squeeze(H_hist_np)
        # print("H_peak:  ", H_hist_np)
        # Sprint("img_h:  ", img_h)
        H_peaks, _ = signal.find_peaks(H_hist_np.T, distance=3, height=40)  # distance表极大值点的距离至少大于等于10个水平单位
        print("H_peak:  ", np.size(H_peaks, 0))

        # plt.plot(H_hist)
        # plt.plot(H_peaks, H_hist_np[H_peaks], "x")
        # plt.plot(np.zeros_like(H_hist_np), "--", color="gray")
        # plt.show()
        detected_colordict = collections.defaultdict(list)
        num_dist = {1:'first',2:'second',3:'third',4:'forth',5:'fifth',6:'sixth'}
        color = list()
        colorpercent = list()

        for i in range(np.size(H_peaks, 0)):
            if i == 0:
                h_min = min([x for x in range(H_peaks[i]) if H_hist_np[x] > 2])
                if i < np.size(H_peaks, 0)-1:
                    h_max = np.int64((H_peaks[i] + H_peaks[i + 1]) / 2)
                else:
                    h_max = max([x for x in range(H_peaks[i], np.size(H_hist_np)) if H_hist_np[x] > 2])
            elif i == np.size(H_peaks, 0) - 1:
                h_min = np.int64((H_peaks[i] + H_peaks[i - 1]) / 2)
                h_max =  max([x for x in range(H_peaks[i], np.size(H_hist_np)) if H_hist_np[x] > 2])
            else:
                h_min = np.int64((H_peaks[i] + H_peaks[i - 1]) / 2)
                h_max = np.int64((H_peaks[i] + H_peaks[i + 1]) / 2)

            lower_hsv = np.array([h_min, s_min, v_min])  # 设定目标颜色上下限
            upper_hsv = np.array([h_max, s_max, v_max])
            print("lower and upper hsv: ", lower_hsv, upper_hsv)
            color_list = []
            color_list.append(lower_hsv)
            color_list.append(upper_hsv)

            mask = cv2.inRange(img_hsv, lowerb=lower_hsv, upperb=upper_hsv)
            binary = cv2.threshold(mask, 127, 255, cv2.THRESH_BINARY)[1]
            binary = cv2.dilate(binary, None, iterations=2)
            maxpercent = 0.15

            cnts, hiera = cv2.findContours(binary.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            sum = 0.01
            for c in cnts:
                sum += cv2.contourArea(c)
            colorper = round(sum / pixelnum, 2)
            # colorname = HSVdetect.color_recog_interval(h_min, h_max)
            colorname = color_recog_peak(H_peaks[i])

            if colorper > maxpercent:
                color.append(colorname)
                colorpercent.append(colorper)
                detected_colordict[colorname] = color_list
            if i == np.size(H_peaks, 0) - 1:
                colorname, colorper = ifwhitein(img_hsv, pixelnum)
                if colorper > maxpercent:
                    color.append(colorname)
                    colorpercent.append(colorper)

                    # white
                    lower_white = np.array([0, 0, 221])
                    upper_white = np.array([180, 30, 255])
                    color_list = []
                    color_list.append(lower_white)
                    color_list.append(upper_white)
                    detected_colordict['white'] = color_list


            # result = cv2.bitwise_and(image, image, mask=mask)
            # name = "result" + str(i) + ".png"
            # cv2.imwrite(name, result)
            # cv2.imshow('result', result)
            # cv2.waitKey(40)

        return color, colorpercent, detected_colordict


# get key from the value of a dictionary
def get_key(dict, value):
    return [k for k, v in dict.items() if v == value]

# decide the color name based on the interval of the fixed bound of colors (have a bug is There will be the same interval, then will get different names)
def color_belong_interval(peak_hvalue, a1,a2,color_dict):
    color_inter={}
    for color in color_dict:
        b1 = color_dict[color][0][0]
        b2 = color_dict[color][1][0]
        if a1> b2 or a2<b1:
            continue
        else:
            interval = b2-max(a1, b1)- (b2-min(a2,b2))
            color_inter[color] = interval
    print("11111:  ", color_inter.values())
    print("22222:  ",max(color_inter.values()))
    if np.size(max(color_inter.values()))>1:
        colorbelongstr = color_belong_peak(peak_hvalue, color_dict)
    colorbelong = get_key(color_inter,max(color_inter.values()))
    colorbelongstr = ''.join(colorbelong)
    return  colorbelongstr

# decide the color name based on the peak's position
def color_belong_peak(peak_hvalue, color_dict):
    for color in color_dict:
        b1 = color_dict[color][0][0]
        b2 = color_dict[color][1][0]
        if peak_hvalue>=b1 and peak_hvalue<=b2:
            colorname = color
            break
    return colorname


def color_recog_interval(peak_hvalue, detecthmin, detecthmax):
    color_dict = getColorList()
    colorname  = color_belong_interval(peak_hvalue, detecthmin,detecthmax,color_dict)

    return  colorname

def color_recog_peak(peak_hvalue):
    color_dict = getColorList()
    colorname = color_belong_peak(peak_hvalue, color_dict)
    return colorname




 
if __name__ == '__main__':
    img = cv2.imread('roi.jpg')
    
    color = get_color(img)
    print(color)
    if len(color) >=3:
        print("correct")


    center_pixel = [320, 337]
    diameter = 30


    