#ifndef DEPTH_SRV_H
#define DEPTH_SRV_H

#include "pose_server/astra/data_structure_astra.h"
#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include "pose_server/ImgToDepth.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>

#include <signal.h>
#include <iostream>

namespace depth_server{

class ObjectDepth{
  public:
    ObjectDepth(ros::NodeHandle *nh);
    ~ObjectDepth();

    void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input);
    bool depth_cb(pose_server::ImgToDepth::Request  &req, pose_server::ImgToDepth::Response &res);
    Point3D calNanPt(float u, float v);

  protected:
    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::ServiceServer service;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    //pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud;
    int spread;
};
}
#endif /* DEPTH_SRV_H */