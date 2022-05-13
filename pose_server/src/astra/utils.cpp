#include "pose_server/astra/pose_srv_astra.h"

// Note: Search (Ctrl+F) for @ADD_ITEM to modify the source code for new items.

namespace pose_server{

void ObjectPose::set_ratio(){
    ratio_min = ratio;
    ratio_max = 1 - ratio;
}

Point2D ObjectPose::find_center(int xmin, int xmax, int ymin, int ymax){
    int cx = (xmin + xmax)/2;
    int cy = (ymin + ymax)/2;
    return {cx, cy};
}


Point2D ObjectPose::midpoint(Point2D p1, Point2D p2){
    return {(p1.x+p2.x)/2, (p1.y+p2.y)/2};
}

Point2D ObjectPose::centroid(Point2D p1, Point2D p2, Point2D p3){
    return {(p1.x+p2.x+p3.x)/3,(p1.y+p2.y+p3.y)/3};
}

float ObjectPose::dist(Point2D p1, Point2D p2){
    return sqrt(
        (p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y)
    );
}

Angles ObjectPose::cosine(Point2D p1, Point2D p2, Point2D p3){
    float p = dist(p1, p2);
    float q = dist(p2, p3);
    float r = dist(p3, p1);

    float a = acos((q*q + r*r - p*p)/(2*q*r));
    float b = acos((p*p + r*r - q*q)/(2*p*r));
    float c = acos((q*q + p*p - r*r)/(2*q*p));

    a = a* 180 / PI;
    b = b* 180 / PI;
    c = c* 180 / PI;

    return {a,b,c};
}

void ObjectPose::read_yaml(){
        cv::FileStorage fs(PATH, cv::FileStorage::READ);
        //std::cout << "PATH: " << PATH << '\n';
        if( fs.isOpened() ){
            fs["x"] >> offset.x;
            fs["y"] >> offset.y;
            fs["width"] >> offset.width;
            fs["height"] >> offset.height;
        }
        else std::cout << "Error: can not load the yaml file from image_database!\n";
}

void ObjectPose::save_data(std::string data, std::string filePath){
    ROS_INFO("Save data!");
    data += "\n";
    std::ofstream sfile;
    std::string SVPATH = filePath;
    sfile.open(SVPATH, std::ios::out | std::ios::app);
    sfile << data;
    sfile.close();
}
}  /* END of namespace*/




