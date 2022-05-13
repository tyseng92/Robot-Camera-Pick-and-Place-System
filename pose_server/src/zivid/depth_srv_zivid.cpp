#include "pose_server/zivid/depth_srv_zivid.h"

namespace depth_server{

    ObjectDepth::ObjectDepth(ros::NodeHandle *nh):n(*nh){
        sub = n.subscribe("/zivid_camera/points", 1, &ObjectDepth::cloud_cb, this);
        service = n.advertiseService("/depth_server_zivid", &ObjectDepth::depth_cb, this); 
        n.param("/pose_server/depth_spread", spread, 1);   
        ROS_INFO("Starting ObjectDepth node...");
    }

    ObjectDepth::~ObjectDepth(){
        ROS_INFO("ObjectDepth is closed.");
    }

    void ObjectDepth::cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input){
        // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
        pcl::fromROSMsg(*input, cloud);
        std::cout << "stored point cloud!\n";
        //outputCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        //std::vector<int> indices;
        //pcl::removeNaNFromPointCloud(cloud, *outputCloud, indices);
    }

    bool ObjectDepth::depth_cb(pose_server::ImgToDepth::Request  &req, pose_server::ImgToDepth::Response &res){
        
        //ROS_INFO("Generating 3D coordinate of the requested point in point cloud..");
        std::cout << "processing: " << req.u << " " << req.v << "\n";
        // index from image is mapped onto point cloud
        int i = (req.u) + (req.v)*cloud.width;

        // 3D coordinates from point cloud using depth value, time 1000 to convert m to mm.
        res.x = (float)(cloud.points[i].x);
        res.y = (float)(cloud.points[i].y);
        res.z = (float)(cloud.points[i].z);

        std::cout << "res.x: " << res.x << "\n";
        std::cout << "res.y: " << res.y << "\n";
        std::cout << "res.z: " << res.z << "\n";
        if(res.z != res.z){
            ROS_WARN("First Depth point has NaN value.");
            Point3D rept = calNanPt(req.u, req.v);
            res.x = rept.x;
            res.y = rept.y;
            res.z = rept.z;
            std::cout << "res.x recalculated: " << res.x << "\n";
            std::cout << "res.y recalculated: " << res.y << "\n";
            std::cout << "res.z recalculated: " << res.z << "\n";
        }
        //res.x = (float)cloud.at(req.u, req.v).x;
        //res.y = (float)cloud.at(req.u, req.v).y;
        //res.z = (float)cloud.at(req.u, req.v).z;
        
        return true;
    }

    Point3D ObjectDepth::calNanPt(float u, float v){
        int U = (int)u;
        int V = (int)v;
        float x_total = 0;
        float y_total = 0;
        float z_total = 0;
        int count = 0;

        for(int i = U-spread; i <= U+spread; i++){
            for(int j = V-spread; j <= V+spread; j++){
                int k = i + j*cloud.width;
                float x_val = (float)cloud.points[k].x;
                float y_val = (float)cloud.points[k].y;
                float z_val = (float)cloud.points[k].z;
                if (x_val==x_val && y_val==y_val && z_val==z_val){
                    x_total += x_val;
                    y_total += y_val;
                    z_total += z_val;
                    count++;
                }
            }
        }
        return {(float)(x_total/count), (float)(y_total/count), (float)(z_total/count)};
    }
}


