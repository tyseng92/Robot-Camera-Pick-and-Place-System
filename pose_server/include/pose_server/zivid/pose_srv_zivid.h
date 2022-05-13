#ifndef POSE_SRV_H
#define POSE_SRV_H

#include "pose_server/zivid/data_structure_zivid.h"
#include "ros/ros.h"

//#include "pose_server/Pose.h"
#include <std_msgs/UInt64MultiArray.h>
#include "pose_server/ImgToDepth.h"
#include "pose_server/PoseZividSrv.h"
#include "pose_server/Capture.h"
#include "pose_server/CaptureAssistantSuggestSettings.h"
#include <cstdlib>

#include <std_msgs/Int8.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
//#include <darknet_ros_msgs/BoundingBoxes.h>
//#include <darknet_ros_msgs/BoundingBox.h>
#include <pose_server/BoundingBoxes.h>
#include <pose_server/BoundingBox.h>
#include <pose_server/Pose.h>
#include <pose_server/PoseList.h>
#include <pose_server/ObjectCount.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <signal.h>
#include <iostream>
#include <math.h>

#include <unistd.h> 
#include <sys/socket.h> 
#include <stdlib.h> 
#include <netinet/in.h> 

#include <cmath>
#include <fstream>
#include <string>

#define PI 3.14159265
#define PORT 8080 

namespace pose_server{

class ObjectPose{
  public:
    ObjectPose(ros::NodeHandle *nh);
    ~ObjectPose();

    void set_ratio();
    void obj_cb(const pose_server::ObjectCount::ConstPtr &fd_obj);
    void box_cb(const pose_server::BoundingBoxes::ConstPtr &msg);
    void img_cb(const sensor_msgs::Image::ConstPtr &img);
    void boxcv_cb(const pose_server::PoseList::ConstPtr &msg);
    bool pose_cb(pose_server::PoseZividSrv::Request  &req, pose_server::PoseZividSrv::Response &res);
    void getTransform(std::vector<Box> Boxes);
    void getTransformLine(std::vector<Box> Boxes);
    void getTransformTri(std::vector<Box> Boxes);
    void getTransform_cv(BoxCv Box);
    void height_pb(std::vector<Box> *Boxes);
    void ignore_side(std::vector<Box> *Boxes);
    void finalbox(std::vector<Box> *Boxes);
    void bruteForceMinLine(std::vector<Box> *Boxes);
    void bruteForceMinTri(std::vector<Box> *Boxes);
    void reuse_transform(int type);
    void get_dgn_pt();
    Point2D find_center(int xmin, int xmax, int ymin, int ymax);
    float dist(Point2D p1, Point2D p2);
    Point2D midpoint(Point2D p1, Point2D);
    Point2D centroid(Point2D p1, Point2D p2, Point2D p3);
    Angles cosine(Point2D p1, Point2D p2, Point2D p3);
    void finalBoxesDisplay();
    void read_yaml();
    void point_display();
    Point3D display2D(float u, float v, bool offset_display, std_msgs::UInt64MultiArray *pt);
    Point3D uv_to_xyz(float u, float v, bool offset_display=true, std_msgs::UInt64MultiArray *pt=NULL);
    void adjustDepth(int type, Point3D *point);
    void cal_rpy(int type);
    void convert_to_msg(int type);
    //void wo_transform();
    void categorize(int type);
    void initialize();
    void callZivid();
    Point3D point_transform(Point3D pt);
    void save_data(std::string data, std::string filePath);

  protected:
    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::Subscriber sub2;
    ros::Subscriber sub3;
    ros::Subscriber sub4;
    ros::Publisher pub;
    ros::Publisher pub2;
    ros::Publisher pub3;
    ros::ServiceClient client;
    ros::ServiceClient client2;
    ros::ServiceClient client3;
    ros::ServiceServer service;
    
    std::string RECTPATH;
    std::string PATH;
    float max_time;
    float s_pb;
    float cam_ground_distance;
    float max_tray_height;
    int tray_num; 
    float w_ratio;
    float h_ratio;
    int h_offset;
    float inc_ang_x;
    float inc_ang_y;
    bool detect_line;
    bool detect_tri;
    int line_len;
    int tri_len;
    int tri_ang;
    float offset_depth_cv;
    float base_depth;
    XmlRpc::XmlRpcValue offset_depth;
    std::vector<std::string> cat_name;
    cv::Rect2d offset;
    double ratio;
    double ratio_min;
    double ratio_max;
    int num;
    std::vector<std::vector<Box>> All_Boxes;
    std::vector<BoxCv> All_Boxes_cv;
    XmlRpc::XmlRpcValue categories;
    Box box;
    BoxCv boxcv;
    float max_pb;
    std::vector<bool> no_obj;
    int max_index;
    Box FinalBox;
    Triangle min_tri;
    Line min_line;
    std::vector<Triangle> tri;
    std::vector<Line> line;

    Point2D center_pt;
    std_msgs::UInt64MultiArray pt;
    std_msgs::UInt64MultiArray pt2;
    std_msgs::UInt64MultiArray pt3;
    Point2D Fmin_pt; 
    Point2D Fmax_pt;


    Point2D dgn_pt;
    Point2D dgn_pt_2;

    Point3D dgn_pt3d;
    Point3D dgn_pt3d_2;
    Point3D center_pt3d;
    Point3D line_pt3d_1;
    Point3D line_pt3d_2;
    Point3D mid_pt3d;
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

    bool dataReady;
    tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped object_transform;
    std::vector<geometry_msgs::TransformStamped> store_transform;
    //std::vector<std::string> pack_name;
    //std::vector<geometry_msgs::TransformStamped> confirm_transform;
    //Transform trans;
    std::map<std::string, int> mapStringNum;
    std::vector<std::string> class_name;
    //pcl::PointCloud<pcl::PointXYZ> cloud;

    int server_fd;
    int new_socket;
    char buffer[1024];
    const char *data;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;

};
}
#endif /* POSE_SRV_H */
