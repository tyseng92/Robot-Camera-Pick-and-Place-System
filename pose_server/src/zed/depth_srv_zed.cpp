#include "pose_server/zed/depth_srv_zed.h"

namespace depth_server{

ObjectDepth::ObjectDepth(ros::NodeHandle *nh):n(*nh){
    sub = n.subscribe("/zed/zed_node/point_cloud/cloud_registered", 1, &ObjectDepth::cloud_cb, this);
    service = n.advertiseService("depth_server", &ObjectDepth::depth_cb, this);    
    ROS_INFO("Starting ObjectDepth node...");
}

ObjectDepth::~ObjectDepth(){
    ROS_INFO("ObjectDepth is closed.");
}

void ObjectDepth::cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input){
    // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    pcl::fromROSMsg(*input, cloud);
    //outputCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    //std::vector<int> indices;
    //pcl::removeNaNFromPointCloud(cloud, *outputCloud, indices);
}

bool ObjectDepth::depth_cb(pose_server::ImgToDepth::Request  &req, pose_server::ImgToDepth::Response &res){
    
    //ROS_INFO("Generating 3D coordinate of the requested point in point cloud..");
    
    // index from image is mapped onto point cloud
    int i = (req.u) + (req.v)*cloud.width;
    
    // 3D coordinates from point cloud using depth value
    res.x = (float)cloud.points[i].x;
    res.y = (float)cloud.points[i].y;
    res.z = (float)cloud.points[i].z;

    //res.x = (float)cloud.at(req.u, req.v).x;
    //res.y = (float)cloud.at(req.u, req.v).y;
    //res.z = (float)cloud.at(req.u, req.v).z;
    
    return true;
}
}


