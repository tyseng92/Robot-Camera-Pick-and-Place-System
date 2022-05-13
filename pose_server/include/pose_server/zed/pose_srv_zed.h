#ifndef POSE_SRV_H
#define POSE_SRV_H

#include "pose_server/zed/data_structure_zed.h"
#include "ros/ros.h"

//#include "pose_server/Pose.h"
#include <std_msgs/UInt64MultiArray.h>
#include "pose_server/ImgToDepth.h"
#include "pose_server/PoseSrv.h"
#include <cstdlib>

#include <std_msgs/Int8.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
//#include <darknet_ros_msgs/BoundingBoxes.h>
//#include <darknet_ros_msgs/BoundingBox.h>
#include <pose_server/BoundingBoxes.h>
#include <pose_server/BoundingBox.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <signal.h>
#include <iostream>
#include <math.h>

#include <unistd.h> 
#include <sys/socket.h> 
#include <stdlib.h> 
#include <netinet/in.h> 

#include <cmath>

#define PI 3.14159265
#define PORT 8080 

namespace pose_server{

class ObjectPose{
  public:
    ObjectPose(ros::NodeHandle *nh);
    ~ObjectPose();

    void set_ratio();
    void obj_cb(const std_msgs::Int8::ConstPtr &fd_obj);
    void box_cb(const pose_server::BoundingBoxes::ConstPtr &msg);
    void img_cb(const sensor_msgs::Image::ConstPtr &img);
    bool pose_cb(pose_server::PoseSrv::Request  &req, pose_server::PoseSrv::Response &res);
    void getTransform(std::vector<Box> Boxes);
    void getTransformSp(std::vector<Box> Boxes);
    void height_pb(std::vector<Box> *Boxes);
    void ignore_side(std::vector<Box> *Boxes);
    void finalbox(std::vector<Box> *Boxes);
    void bruteForceMinTri(std::vector<Box> *Boxes);
    void get_dgn_pt();
    Point2D find_center(int xmin, int xmax, int ymin, int ymax);
    float dist(Point2D p1, Point2D p2);
    Point2D midpoint(Point2D p1, Point2D);
    Point2D centroid(Point2D p1, Point2D p2, Point2D p3);
    Angles cosine(Point2D p1, Point2D p2, Point2D p3);
    void finalBoxesDisplay();
    void read_yaml();
    void point_display();
    Point3D uv_to_xyz(float u, float v, bool offset_display=true, std_msgs::UInt64MultiArray *pt=NULL);
    void adjustDepth();
    void cal_rpy(bool centroid);
    void convert_to_msg(bool centroid);
    void categorize(bool centroid);
    void initialize();

  protected:
    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::Subscriber sub2;
    ros::Subscriber sub3;
    ros::Publisher pub;
    ros::Publisher pub2;
    ros::ServiceClient client;
    ros::ServiceServer service;
    
    std::string RECTPATH;
    std::string PATH;
    float s_pb;
    float cam_ground_distance;
    float max_tray_height;
    int tray_num; 
    float w_ratio;
    float h_ratio;
    float inc_ang_y;
    float inc_ang_z;
    int tri_len;
    int tri_ang;
    float offset_brc_b;
    cv::Rect2d offset;
    double ratio;
    double ratio_min;
    double ratio_max;
    int num;
    std::vector<Box> Boxes_Potato;
    std::vector<Box> Boxes_Tomato;
    std::vector<Box> Boxes_Broccoli;
    Box box;
    float max_pb;
    std::vector<bool> no_obj;
    int max_index;
    Box FinalBox;
    Triangle min_tri;
    std::vector<Triangle> tri;

    Point2D center_pt;
    std_msgs::UInt64MultiArray pt;
    std_msgs::UInt64MultiArray pt2;
    Point2D Fmin_pt; 
    Point2D Fmax_pt;


    Point2D dgn_pt;
    Point2D dgn_pt_2;

    Point3D dgn_pt3d;
    Point3D dgn_pt3d_2;
    Point3D center_pt3d;
    Point3D tri_pt3d_1;
    Point3D tri_pt3d_2;
    Point3D tri_pt3d_3;
    Point3D centroid_pt3d;
    Rot3D rot_3d;
    std::string object;
    std::string className;
    bool isNaN;

    int offset_x;
    int offset_y;
    int off_u;
    int off_v;
    bool run_boxcb;

    tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped object_transform;
    std::map<std::string, int> mapStringNum;
    //pcl::PointCloud<pcl::PointXYZ> cloud;

    int server_fd;
    int new_socket;
    char buffer[1024];
    const char *data;

};
}
#endif /* POSE_SRV_H */
