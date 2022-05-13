#include "pose_server/astra/pose_srv_astra.h"

// Note: Search (Ctrl+F) for @ADD_ITEM to modify the source code for new items.

namespace pose_server{

ObjectPose::ObjectPose(ros::NodeHandle *nh):n(*nh), tfListener(tfBuffer){
    sub = n.subscribe("/darknet_ros_astra/found_object", 1, &ObjectPose::obj_cb, this);
    sub2 = n.subscribe("/darknet_ros_astra/bounding_boxes", 1, &ObjectPose::box_cb, this);
    sub3 = n.subscribe("/darknet_ros_astra/detection_image", 1, &ObjectPose::img_cb, this);
    //sub4 = n.subscribe("/object_detect/bounding_boxes", 1, &ObjectPose::boxcv_cb, this);
    sub4 = n.subscribe("/color_detect/center_list", 1, &ObjectPose::boxcv_cb, this);
    pub = n.advertise<std_msgs::UInt64MultiArray>("/pose_server/FinalBoxCenter", 100);
    pub2 = n.advertise<std_msgs::UInt64MultiArray>("/pose_server/FinalLineMid", 100);
    pub3 = n.advertise<std_msgs::UInt64MultiArray>("/pose_server/FinalTriCentroid", 100);
    client = n.serviceClient<pose_server::ImgToDepth>("/depth_server_astra");
    client3 = n.serviceClient<pose_server::Capture>("/astra_camera/capture");
    n.param("/pose_server/dgn_pt_ratio", ratio, 0.4);
    n.param("/pose_server/tray_num", tray_num, 1);
    n.param("/pose_server/w_ratio", w_ratio, (float)0.2);
    n.param("/pose_server/h_ratio", h_ratio, (float)0.167);
    n.param("/pose_server/h_offset", h_offset, 0);
    n.param("/astra_subscriber/detect_line", detect_line, false);
    n.param("/astra_subscriber/detect_tri", detect_tri, false);
    n.param("/pose_server/line_len", line_len, 80);
    n.param("/pose_server/tri_len", tri_len, 80);
    n.param("/pose_server/tri_ang", tri_ang, 90);
    n.getParam("/pose_server/offset", offset_depth);
    n.param("/pose_server/offset_cv", offset_depth_cv, (float)0.0);
    n.param("/pose_server/base_depth", base_depth, (float)0.0);
    n.getParam("/pose_server/categories", categories);
    n.getParam("/pose_server/class_name", class_name);
    n.getParam("/pose_server/cat_name", cat_name);

    set_ratio();
    char* home = std::getenv("HOME");
    std::string s(home);
    std::string rectpath = "/Desktop/image_database/yolo/DATABASE_DEC/astra/roi.yml";
    
    n.param<std::string>("/pose_server/rectpath", RECTPATH, rectpath);
    PATH = home + RECTPATH;
    isNaN = false;
    //dataReady = false;
    max_time = 0;

    //All_Boxes.reserve(categories.size());
    for(int i = 0; i < categories.size(); i++){
        All_Boxes.push_back(std::vector<Box>());
    }
    std::cout << "cat_size: " << categories.size() << "\n";

    ROS_INFO("Starting ObjectPose node...");
}

ObjectPose::~ObjectPose(){
    ROS_INFO("ObjectPose is closed.");
}

void ObjectPose::obj_cb(const pose_server::ObjectCount::ConstPtr &fd_obj){
    num = fd_obj->count;
}

//@ADD_ITEM
// find the box with highest probability. 
void ObjectPose::box_cb(const pose_server::BoundingBoxes::ConstPtr &msg){
    ros::Time begin = ros::Time::now();

    std::cout << "1================================\n";

    for (int i = 0; i < categories.size(); i++){
        All_Boxes[i].clear();
    }

    box = {};
    initialize();

    // store the msg in local memory 
    for(auto it = msg->bounding_boxes.begin(); it != msg->bounding_boxes.end(); ++it){
        box.cls = (*it).Class;
        box.pb = (*it).probability;
        box.xmin = (*it).xmin;
        box.ymin = (*it).ymin;
        box.xmax = (*it).xmax;
        box.ymax = (*it).ymax;

        // All_Boxes will store all of the boxes info of the items
        bool hvClass = false;
        //std::cout << "Check range: \n";
        for(int i = 0; i < categories.size(); i++){
            int l = categories[i].size()-1;
            if(mapStringNum[box.cls] >= mapStringNum[categories[i][0]] && mapStringNum[box.cls] <= mapStringNum[categories[i][l]]){
                //std::cout << "range: "<< mapStringNum[categories[i][0]] << " " << mapStringNum[categories[i][l]] << "\n";
                All_Boxes[i].push_back(box);
                hvClass = true; 
                break;
            }
        }
        if(hvClass==false){
            ROS_ERROR("Class number out of range error!!");
            exit(-1);
        }
    }

    // Make sure the sequence for displaying of data points in image_database packages is correct, if the getTransform functions below are modified.
    store_transform.clear();
    for (int i = 0; i < categories.size(); i++){
        getTransform(All_Boxes[i]);
        if(detect_line && !detect_tri)getTransformLine(All_Boxes[i]);
        if(detect_tri)getTransformTri(All_Boxes[i]);
    }

    //wo_transform();
    //confirm_transform = pack_transform;
 
    // If all of the getTransform does not detect any boxes, there will be no display shown, returning out of the box_cb.
    if (std::all_of(no_obj.begin(), no_obj.end(), [](bool v) { return v; })){
        no_obj.clear();
        std::cout << "2!================================\n";
        return;
    }
    no_obj.clear();

    // send points for display in image database
    point_display();
    
    //dataReady = true;
    ros::Time end = ros::Time::now();

    ros::Duration cb_time = end - begin;
    if(cb_time.toSec() > max_time){
        max_time = cb_time.toSec();
    }
    //ROS_INFO("Pose End Time: ", (double)end);
    save_data(std::to_string(begin.toSec()), "/home/rrc/Desktop/time_results/PoseBeginTime.txt");
    save_data(std::to_string(end.toSec()), "/home/rrc/Desktop/time_results/PoseEndTime.txt");
    std::cout << "End Pose Time: " << end << "\n";
    ROS_INFO("Total call back time: %f ", (double)cb_time.toSec());
    ROS_INFO("Max call back time: %f ", (double)max_time);
    std::cout << "2================================\n";
}

// in case we need to process the image with bounding box
void ObjectPose::img_cb(const sensor_msgs::Image::ConstPtr &img){

}

}  /* END of namespace*/




