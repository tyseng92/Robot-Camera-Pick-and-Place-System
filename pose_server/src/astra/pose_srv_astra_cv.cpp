#include "pose_server/astra/pose_srv_astra.h"

// Note: Search (Ctrl+F) for @ADD_ITEM to modify the source code for new items.

namespace pose_server{

void ObjectPose::boxcv_cb(const pose_server::PoseList::ConstPtr &msg){
    ros::Time begin = ros::Time::now();

    std::cout << "1cv================================\n";

    All_Boxes_cv.clear();
    boxcv = {};

    // store the msg in local memory 
    for(auto it = msg->pose_list.begin(); it != msg->pose_list.end(); ++it){
        boxcv.cls = (*it).Class;
        boxcv.cx = (*it).cx;
        boxcv.cy = (*it).cy;
        boxcv.angle = (*it).angle;

        All_Boxes_cv.push_back(boxcv);

    }
    if(All_Boxes_cv.size()==0){
        std::cout << "2!cv================================\n";
        return;
    }
    
    for(int i = 0; i < All_Boxes_cv.size(); i++){
        getTransform_cv(All_Boxes_cv[i]);
    }
    
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
    std::cout << "2cv================================\n";
}

void ObjectPose::getTransform_cv(BoxCv Box){
    std::cout << "T1cv================================\n";
    int type_num = 0;

    center_pt3d = uv_to_xyz(Box.cx, Box.cy, false);

    // adjust depth of the 3d point
    center_pt3d.z = (center_pt3d.z + base_depth)/2 - offset_depth_cv;
    //center_pt3d.z = base_depth;

    rot_3d.rz = Box.angle*PI/180;
    object = Box.cls;

    std::cout << "center_pt3d x: " << center_pt3d.x << "\n";
    std::cout << "center_pt3d y: " << center_pt3d.y << "\n";
    std::cout << "center_pt3d z: " << center_pt3d.z << "\n";
    std::cout << "rot_3d.rz: " << rot_3d.rz << "\n";

    // Ignore Nan value from depth measurement.
    if(isNaN){
        isNaN = false;
        std::cout << "T2!!!cv================================\n";
        return;
    }
    // Convert to PoseStamped message with quaternion
    convert_to_msg(type_num);
 
    // publish the pose of object when requested by client
    br.sendTransform(object_transform);
    std::cout << "T2cv================================\n";
}

}  /* END of namespace*/




