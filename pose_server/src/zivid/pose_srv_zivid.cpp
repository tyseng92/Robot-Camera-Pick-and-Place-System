#include "pose_server/zivid/pose_srv_zivid.h"

// Note: Search (Ctrl+F) for @ADD_ITEM to modify the source code for new items.

namespace pose_server{

ObjectPose::ObjectPose(ros::NodeHandle *nh):n(*nh), tfListener(tfBuffer){
    sub = n.subscribe("/darknet_ros_zivid/found_object", 1, &ObjectPose::obj_cb, this);
    sub2 = n.subscribe("/darknet_ros_zivid/bounding_boxes", 1, &ObjectPose::box_cb, this);
    sub3 = n.subscribe("/darknet_ros_zivid/detection_image", 1, &ObjectPose::img_cb, this);
    sub4 = n.subscribe("/object_detect/bounding_boxes", 1, &ObjectPose::boxcv_cb, this);
    pub = n.advertise<std_msgs::UInt64MultiArray>("/pose_server/FinalBoxCenter", 100);
    pub2 = n.advertise<std_msgs::UInt64MultiArray>("/pose_server/FinalLineMid", 100);
    pub3 = n.advertise<std_msgs::UInt64MultiArray>("/pose_server/FinalTriCentroid", 100);
    client = n.serviceClient<pose_server::ImgToDepth>("/depth_server_zivid");
    client2 = n.serviceClient<pose_server::CaptureAssistantSuggestSettings>("/zivid_camera/capture_assistant/suggest_settings");
    client3 = n.serviceClient<pose_server::Capture>("/zivid_camera/capture");
    //service = n.advertiseService("pose_server", &ObjectPose::pose_cb, this);
    n.param("/pose_server/dgn_pt_ratio", ratio, 0.4);
    n.param("/pose_server/tray_num", tray_num, 1);
    n.param("/pose_server/w_ratio", w_ratio, (float)0.2);
    n.param("/pose_server/h_ratio", h_ratio, (float)0.167);
    n.param("/pose_server/h_offset", h_offset, 0);
    n.param("/zivid_subscriber/detect_line", detect_line, false);
    n.param("/zivid_subscriber/detect_tri", detect_tri, false);
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
    std::string rectpath = "/Desktop/image_database/yolo/DATABASE_DEC/zivid/roi.yml";
    
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

void ObjectPose::set_ratio(){
    ratio_min = ratio;
    ratio_max = 1 - ratio;
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

void ObjectPose::getTransform(std::vector<Box> Boxes){
    std::cout << "T1================================\n";
    if(Boxes.size() == 0){
        std::cout << "T2!================================\n";
        return;
    }
    int type_num = 0;
    // For transformation object naming.
    //className = Boxes[0].cls;
    // check All_Boxes contents
    //for(std::vector<Box>::iterator it = Boxes.begin(); it != Boxes.end(); ++it){
    //    std::cout << "class :" << (*it).cls << "\n";
    //}
    // adjust box probability of box near the side of tray
    height_pb(&Boxes);
    
    // Avoid gripper hits the side of the tray
    ignore_side(&Boxes);
    
    // find the final box
    finalbox(&Boxes);

    // get the item name for final box
    categorize(type_num);
    if (max_pb == FLT_MIN){
        std::cout << "T2!!================================\n";
        return;
    }
    
    // instead of using the coordinate of roi, this function set an offset, changing the coordinate of the final box to camera full image.
    finalBoxesDisplay();

    // get 2 ends of sausage which has highest probability
    get_dgn_pt();

    // x, y, and z depth of two ends 
    dgn_pt3d = uv_to_xyz(dgn_pt.x, dgn_pt.y, true, &pt);
    dgn_pt3d_2 = uv_to_xyz(dgn_pt_2.x, dgn_pt_2.y, true, &pt);

    // x, y, and z depth of center point
    center_pt3d = uv_to_xyz(center_pt.x, center_pt.y, true, &pt);
    adjustDepth(type_num, &center_pt3d);

    std::cout << "center_pt3d x: " << center_pt3d.x << "\n";
    std::cout << "center_pt3d y: " << center_pt3d.y << "\n";
    std::cout << "center_pt3d z: " << center_pt3d.z << "\n";

    //calculate yaw and pitch angle of the object
    cal_rpy(type_num);

    // Ignore Nan value from depth measurement.
    if(isNaN){
        isNaN = false;
        std::cout << "T2!!!================================\n";
        return;
    }
    // Convert to PoseStamped message with quaternion
    convert_to_msg(type_num);

    // pack object name for delta socket to use
    //if (pack_name.size()==tray_num){
    //    pack_name.push_back(object_transform.child_frame_id);
    //}
    //else{
    //    pack_name.clear();
    //    pack_name.push_back(object_transform.child_frame_id);
    //}
    
    // store transform for getTransfromLine and getTransformTri.
    store_transform.push_back(object_transform); 
    // publish the pose of object when requested by client
    br.sendTransform(object_transform);

    std::cout << "T2================================\n";

}

void ObjectPose::getTransformLine(std::vector<Box> Boxes){
    std::cout << "D1================================\n";
    if(Boxes.size() == 0){
        std::cout << "D2!================================\n";
        return;
    }
    int type_num = 1;
    // Avoid gripper hits the side of the tray
    ignore_side(&Boxes);

    bruteForceMinLine(&Boxes);
    
    // For transformation object naming.
    className = Boxes[0].cls;
    categorize(type_num);

    if (line.size() == 0){
        std::cout << "D2!!================================\n";
        reuse_transform(type_num);
        return;
    }
    line.clear();
    ROS_INFO("Finish calculating min line.");

    Point2D c_pt = midpoint(min_line.p1, min_line.p2);
    std::cout << "midpoint: " << c_pt.x << " "<< c_pt.y << '\n';


    line_pt3d_1 = uv_to_xyz(min_line.p1.x, min_line.p1.y, true, &pt2);
    line_pt3d_2 = uv_to_xyz(min_line.p2.x, min_line.p2.y, true, &pt2);
    //mid_pt3d = uv_to_xyz(c_pt.x, c_pt.y, true, &pt2);
    display2D(c_pt.x, c_pt.y, true, &pt2);
    
    // Get the average depth of three of the triangle items
    mid_pt3d = {(line_pt3d_1.x+line_pt3d_2.x)/2, (line_pt3d_1.y+line_pt3d_2.y)/2, (line_pt3d_1.z+line_pt3d_2.z)/2};
    adjustDepth(type_num, &mid_pt3d);

    cal_rpy(type_num);
    

    // Ignore Nan value from depth measurement.
    if(isNaN){
        isNaN = false;
        std::cout << "D2!!!================================\n";
        return;
    }
    // Convert to PoseStamped message with quaternion for centroid of 3 items
    convert_to_msg(type_num);
    br.sendTransform(object_transform);
    std::cout << "D2================================\n";
}

void ObjectPose::getTransformTri(std::vector<Box> Boxes){
    std::cout << "C1================================\n";
    if(Boxes.size() == 0){
        std::cout << "C2!================================\n";
        return;
    }
    int type_num = 2;

    // Avoid gripper hits the side of the tray
    ignore_side(&Boxes);

    bruteForceMinTri(&Boxes);

    // For transformation object naming.
    className = Boxes[0].cls;
    categorize(type_num);

    if (tri.size() == 0){
        std::cout << "C2!!================================\n";
        reuse_transform(type_num);
        return;
    }
    tri.clear();
    ROS_INFO("Finish calculating min triangle.");

    Point2D c_pt = centroid(min_tri.p1, min_tri.p2, min_tri.p3);
    std::cout << "Centroid: " << c_pt.x << " "<< c_pt.y << '\n';


    tri_pt3d_1 = uv_to_xyz(min_tri.p1.x, min_tri.p1.y, true, &pt3);
    tri_pt3d_2 = uv_to_xyz(min_tri.p2.x, min_tri.p2.y, true, &pt3);
    tri_pt3d_3 = uv_to_xyz(min_tri.p3.x, min_tri.p3.y, true, &pt3);
    centroid_pt3d = uv_to_xyz(c_pt.x, c_pt.y, true, &pt3);
    
    // Get the average depth of three of the triangle items
    centroid_pt3d = {(tri_pt3d_1.x+tri_pt3d_2.x+tri_pt3d_3.x)/3, (tri_pt3d_1.y+tri_pt3d_2.y+tri_pt3d_3.y)/3, (tri_pt3d_1.z+tri_pt3d_2.z+tri_pt3d_3.z)/3};
    adjustDepth(type_num, &centroid_pt3d);

    cal_rpy(type_num);
    

    // Ignore Nan value from depth measurement.
    if(isNaN){
        isNaN = false;
        std::cout << "C2!!!================================\n";
        return;
    }
    // Convert to PoseStamped message with quaternion for centroid of 3 items
    convert_to_msg(type_num);
    br.sendTransform(object_transform);
    std::cout << "C2================================\n";
}

// in case we need to process the image with bounding box
void ObjectPose::img_cb(const sensor_msgs::Image::ConstPtr &img){

}

///*
void ObjectPose::bruteForceMinLine(std::vector<Box> *Boxes){
    for(std::size_t i=0; i<Boxes->size(); ++i){
        for(std::size_t j=i+1; j<Boxes->size(); ++j){
            // refer to this link: https://stackoverflow.com/questions/1910712/dereference-vector-pointer-to-access-element
            // Boxes->at(i).xmin can be used as well 
            Point2D p1 = find_center((*Boxes)[i].xmin, (*Boxes)[i].xmax, (*Boxes)[i].ymin, (*Boxes)[i].ymax);
            Point2D p2 = find_center((*Boxes)[j].xmin, (*Boxes)[j].xmax, (*Boxes)[j].ymin, (*Boxes)[j].ymax);

            if(dist(p1,p2) > 0 && dist(p1,p2) < line_len){
                if ((*Boxes)[i].near_side == true || (*Boxes)[j].near_side == true){
                    continue;
                }
                //std::cout << "Index: " << i << " " << j << " " << k << "\n";
                //std::cout << "Triangle pts: " <<  p1.x << ' ' << p1.y << ' ' << p2.x << ' ' << p2.y << ' ' << p3.x << ' ' << p3.y << '\n';
                line.push_back({p1,p2});
            }
        }
    }
    //std::cout << "tri size: " << tri.size() << "\n";
    // If no triangle detected, this function will be returned.
    if (line.size() == 0){
        ROS_WARN("No line between two items is detected!");
        no_obj.push_back(true);
        return;
    }
    else no_obj.push_back(false);

    float min = FLT_MAX;
    // Line with min length
    for(std::size_t i=0; i<line.size(); ++i){
        float length = dist(line[i].p1, line[i].p2);
        if(length < min){
            min = length;
            min_line = {line[i].p1, line[i].p2};
        }
    }
}
//*/

void ObjectPose::bruteForceMinTri(std::vector<Box> *Boxes){
    //for(std::size_t i = 0; i<Boxes->size(); ++i){
    //    Point2D p = find_center((*Boxes)[i].xmin, (*Boxes)[i].xmax, (*Boxes)[i].ymin, (*Boxes)[i].ymax);
    //    std::cout << "Boxes: " << p.x << " " << p.y << "\n";
    //}
    
    //for(std::size_t i = 0; i<Boxes_Potato.size(); ++i){
    //    Point2D p = find_center(Boxes_Potato[i].xmin, Boxes_Potato[i].xmax, Boxes_Potato[i].ymin, Boxes_Potato[i].ymax);
    //    std::cout << "Boxesv: " << p.x << " " << p.y << "\n";
    //}

    float min = FLT_MAX;
    for(std::size_t i=0; i<Boxes->size(); ++i){
        for(std::size_t j=i+1; j<Boxes->size(); ++j)
            for(std::size_t k=j+1; k<Boxes->size(); ++k){
                // refer to this link: https://stackoverflow.com/questions/1910712/dereference-vector-pointer-to-access-element
                // Boxes->at(i).xmin can be used as well 
                Point2D p1 = find_center((*Boxes)[i].xmin, (*Boxes)[i].xmax, (*Boxes)[i].ymin, (*Boxes)[i].ymax);
                Point2D p2 = find_center((*Boxes)[j].xmin, (*Boxes)[j].xmax, (*Boxes)[j].ymin, (*Boxes)[j].ymax);
                Point2D p3 = find_center((*Boxes)[k].xmin, (*Boxes)[k].xmax, (*Boxes)[k].ymin, (*Boxes)[k].ymax);
                Angles ang = cosine(p1,p2,p3);
                // Edge and angle constraint for triangle.
                if(dist(p1,p2)< tri_len && dist(p2,p3)< tri_len && dist(p3,p1)< tri_len){
                    if(ang.a < tri_ang && ang.b < tri_ang && ang.c < tri_ang){
                        // if any of the edge point near side, ignore it.
                        if ((*Boxes)[i].near_side == true || (*Boxes)[j].near_side == true || (*Boxes)[k].near_side == true){
                            continue;
                        }
                        //std::cout << "Index: " << i << " " << j << " " << k << "\n";
                        //std::cout << "Triangle pts: " <<  p1.x << ' ' << p1.y << ' ' << p2.x << ' ' << p2.y << ' ' << p3.x << ' ' << p3.y << '\n';
                        tri.push_back({p1,p2,p3});
                    }
                }
            }
    }
    //std::cout << "tri size: " << tri.size() << "\n";
    // If no triangle detected, this function will be returned.
    if (tri.size() == 0){
        ROS_WARN("No triangle is detected!");
        no_obj.push_back(true);
        return;
    }
    else no_obj.push_back(false);

    // Triangle with min area
    for(std::size_t i=0; i<tri.size(); ++i){
        int x1 = tri[i].p1.x;
        int x2 = tri[i].p2.x;
        int x3 = tri[i].p3.x;
        int y1 = tri[i].p1.y;
        int y2 = tri[i].p2.y;
        int y3 = tri[i].p3.y;

        float area = fabs(x1*(y2-y3)+x2*(y3-y1)+x3*(y1-y2))/2;
        if (area < min){
            min = area;
            //std::cout << "Area: " << min << "\n"; 
            min_tri = {tri[i].p1, tri[i].p2, tri[i].p3};
        }
    }
    
    //std::cout << "Min Triangle: " << min_tri.p1.x << " " << min_tri.p1.y << "\n" 
    //<< min_tri.p2.x << " " << min_tri.p2.y << "\n" 
    //<< min_tri.p3.x << " " << min_tri.p3.y << "\n";
}

// if no transformation is found in getTransformLine or getTransformTri, transformation from getTransform will be used, retrieving from store_transform container.
void ObjectPose::reuse_transform(int type){
    for(auto it = store_transform.begin(); it != store_transform.end(); ++it){      
        if((*it).child_frame_id == object){
            //std::cout << "className when no line found: " << object << "\n";
            geometry_msgs::TransformStamped s_transform = (*it);
            if(type == 1){
                s_transform.child_frame_id = "2_" + object;
            }
            else if(type == 2){
                s_transform.child_frame_id = "3_" + object;
            }
            br.sendTransform(s_transform);
        }
    }
}

// Note: Remember to draw ROI around trays only using image_database packages.
void ObjectPose::ignore_side(std::vector<Box> *Boxes){
    int W = offset.width;
    int H = offset.height;

    //for(std::vector<Box>::iterator it = Boxes->begin(); it != Boxes->end(); ++it){
    //    std::cout << "Original prob " << (*it).cls << ": " << (*it).pb << '\n';
    //}
    for(std::vector<Box>::iterator it = Boxes->begin(); it != Boxes->end(); ++it){
        (*it).near_side = false;
        Point2D c_pt = find_center((*it).xmin, (*it).xmax, (*it).ymin, (*it).ymax);
        // if the box is near the side of tray, reduce its probability of being pick up as final box.  
        //if (c_pt.x < 1*W/6 || c_pt.x > 5*W/6 || c_pt.y < 1*H/5 || c_pt.y > 4*H/5){
        
        // create outer boundary of the roi with trays
        if (c_pt.x < w_ratio*(W/tray_num) || c_pt.x > W-(w_ratio)*(W/tray_num) || c_pt.y < h_ratio*H+h_offset || c_pt.y > (1-h_ratio)*H+h_offset){
            (*it).near_side = true;
        }
        else{
            // create vertical boundary between trays within roi
            for (int i= 2; i <= tray_num; i++){
                if (c_pt.x > (W/tray_num)*((i-1)-w_ratio) && c_pt.x < (W/tray_num)*((i-1)+w_ratio)){
                    (*it).near_side = true;
                }               
            }
        }
    }
}

void ObjectPose::height_pb(std::vector<Box> *Boxes){
    for(std::vector<Box>::iterator it = Boxes->begin(); it != Boxes->end(); ++it){
        Point2D c_pt = find_center((*it).xmin, (*it).xmax, (*it).ymin, (*it).ymax);
        // increase probability of the highest object where height is scanned using depth 
        Point3D d_pt = uv_to_xyz(c_pt.x, c_pt.y, false);

        // Reduce height probability of broccoli due to its trunk facing upwards
        //if (mapStringNum[(*it).cls]>= 27 && mapStringNum[(*it).cls]<= 30){
        //    d_pt.z -= offset_brc_b; 
        //} 

        (*it).height = d_pt.z;
        //std::cout << "Height prob " << (*it).cls << ": " << (*it).height << ' ' << (*it).near_side << '\n';
    
        //if (mapStringNum[(*it).cls]>= 6 && mapStringNum[(*it).cls]<= 13){
        //std::cout << "cam_ground: " << cam_ground_distance << '\n'; 
        //}
    }



  //  std::cout << "cam_ground: " << cam_ground_distance << '\n';
}

void ObjectPose::finalbox(std::vector<Box> *Boxes){
    max_pb = FLT_MIN;
    max_index = -1;
    //std::cout << "min pb: " << max_pb << "\n";
    for(std::vector<Box>::iterator it = Boxes->begin(); it != Boxes->end(); ++it){
        //std::cout << "height: " << (*it).height << "\n";
        if (max_pb < (*it).height){
            if ((*it).near_side == true){
                continue;
            }
            //std::cout << "it_height: " << (*it).height << "\n";
            max_pb = (*it).height;
            max_index = it - Boxes->begin();
        }   
    }
    std::cout << "max_index: " << max_index << '\n';
    //std::cout << "max_pb: " << max_pb << '\n';
    if (max_pb == FLT_MIN){
        std::cout << "No final box is detected!" << '\n';
        no_obj.push_back(true);
        return;
    } 
    else no_obj.push_back(false);
    FinalBox = (*Boxes)[max_index];

    //if (mapStringNum[FinalBox.cls]>= 6 && mapStringNum[FinalBox.cls]<= 13){
    //    std::cout << "Choosen Box: " << FinalBox.cls << '\n';
    //}

 //   std::cout << "Choosen Box: " << FinalBox.cls << '\n';

    center_pt = find_center(FinalBox.xmin, FinalBox.xmax, FinalBox.ymin, FinalBox.ymax);
    
    // get the most frequent chosen final box
    //weight_finalbox()
    
    // u and v of two ends of min and max 
    Fmin_pt = {FinalBox.xmin, FinalBox.ymin}; 
    Fmax_pt = {FinalBox.xmax, FinalBox.ymax};    
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

void ObjectPose::finalBoxesDisplay(){
    read_yaml();
    pt.data.push_back(Fmin_pt.x + offset.x);
    pt.data.push_back(Fmin_pt.y + offset.y);
    pt.data.push_back(Fmax_pt.x + offset.x);
    pt.data.push_back(Fmax_pt.y + offset.y);
}

void ObjectPose::point_display(){
    // publish center point
    //for(std::vector<size_t>::iterator it = pt2.data.begin(); it != pt2.data.end(); ++it){
    //    std::cout << "pt2 " <<  it-pt2.data.begin() << ":" << (*it) << std::endl;
    //}
    pub.publish(pt);
    pt.data.clear();
    if(detect_line && !detect_tri){
        pub2.publish(pt2);
        pt2.data.clear();
    }
    if(detect_tri){
        pub3.publish(pt3);
        pt3.data.clear();
    }
}
//@ADD_ITEM
void ObjectPose::initialize(){
    // set the mapStringNum value
    int num = 0;
    for (std::string name: class_name){
        mapStringNum[name]=num++;
    }

    // uncomment print to check for mapStringNum
    std::vector<std::pair<std::string, int>> map2vec;
    for (auto& it : mapStringNum){
        map2vec.push_back(it);
    }
    // sort the map2vec container in ascending number order.
    sort(map2vec.begin(), map2vec.end(), [](std::pair<std::string, int>& a, std::pair<std::string, int>& b){return a.second < b.second;});
    //std::cout << "mapStringNum:\n";
    for (auto elem : map2vec){
        //std::cout << elem.first << " " << elem.second << "\n";
    } 
}

//@ADD_ITEM
void ObjectPose::get_dgn_pt(){

    int x1, y1, x2, y2;
    //std::cout << "mapStringNum[FinalBox.cls]: " << mapStringNum[FinalBox.cls] << '\n';
    //std::cout << "ratio_min: " << ratio_min << '\n';
    //std::cout << "ratio_max: " << ratio_max << '\n';
    
    switch(mapStringNum[FinalBox.cls]){ 
        // 0 degree 
        case 0:      
        case 6:
        case 21:
        case 23:
        case 27:
        case 31:
            x1 = (Fmin_pt.x + Fmax_pt.x)/2 ;
            y1 = Fmin_pt.y + (Fmax_pt.y - Fmin_pt.y)*ratio_min;
            x2 = x1;
            y2 = Fmin_pt.y + (Fmax_pt.y - Fmin_pt.y)*ratio_max;
            break;
        // 45 degree
        case 1:
        case 2:
        case 7:
        case 22:
        case 24:
        case 28:
        case 32:
            x1 = Fmin_pt.x + (Fmax_pt.x - Fmin_pt.x)*ratio_max;
            y1 = Fmin_pt.y + (Fmax_pt.y - Fmin_pt.y)*ratio_min;
            x2 = Fmin_pt.x + (Fmax_pt.x - Fmin_pt.x)*ratio_min;
            y2 = Fmin_pt.y + (Fmax_pt.y - Fmin_pt.y)*ratio_max;
            break;
        // 90 degree 
        case 3:
        case 8:
        case 15:
        case 25:
        case 29:
        case 33:
            x1 = Fmin_pt.x + (Fmax_pt.x - Fmin_pt.x)*ratio_max;
            y1 = (Fmin_pt.y + Fmax_pt.y)/2 ;
            x2 = Fmin_pt.x + (Fmax_pt.x - Fmin_pt.x)*ratio_min;
            y2 = y1;
            break;
        // 135 degree
        case 4:
        case 5:
        case 9:
        case 16:
        case 26:
        case 30:
        case 34:
            x1 = Fmin_pt.x + (Fmax_pt.x - Fmin_pt.x)*ratio_max;
            y1 = Fmin_pt.y + (Fmax_pt.y - Fmin_pt.y)*ratio_max;
            x2 = Fmin_pt.x + (Fmax_pt.x - Fmin_pt.x)*ratio_min;
            y2 = Fmin_pt.y + (Fmax_pt.y - Fmin_pt.y)*ratio_min;
            break;
        // 180 degree
        case 10:
        case 17:
            x1 = (Fmin_pt.x + Fmax_pt.x)/2 ;
            y1 = Fmin_pt.y + (Fmax_pt.y - Fmin_pt.y)*ratio_max;
            x2 = x1;
            y2 = Fmin_pt.y + (Fmax_pt.y - Fmin_pt.y)*ratio_min;
            break;
        // 225 degree
        case 11:
        case 18:
            x1 = Fmin_pt.x + (Fmax_pt.x - Fmin_pt.x)*ratio_min;
            y1 = Fmin_pt.y + (Fmax_pt.y - Fmin_pt.y)*ratio_max;
            x2 = Fmin_pt.x + (Fmax_pt.x - Fmin_pt.x)*ratio_max;
            y2 = Fmin_pt.y + (Fmax_pt.y - Fmin_pt.y)*ratio_min;
            break;
        // 270 degree
        case 12:
        case 19:
            x1 = Fmin_pt.x + (Fmax_pt.x - Fmin_pt.x)*ratio_min;
            y1 = (Fmin_pt.y + Fmax_pt.y)/2 ;
            x2 = Fmin_pt.x + (Fmax_pt.x - Fmin_pt.x)*ratio_max;
            y2 = y1;
            break;
        // 315 degree
        case 13:
        case 20:
            x1 = Fmin_pt.x + (Fmax_pt.x - Fmin_pt.x)*ratio_min;
            y1 = Fmin_pt.y + (Fmax_pt.y - Fmin_pt.y)*ratio_min;
            x2 = Fmin_pt.x + (Fmax_pt.x - Fmin_pt.x)*ratio_max;
            y2 = Fmin_pt.y + (Fmax_pt.y - Fmin_pt.y)*ratio_max;
            break;
        // no degree (no directions)
        case 14:
            x1 = (Fmin_pt.x + Fmax_pt.x)/2 ;
            y1 = (Fmin_pt.y + Fmax_pt.y)/2 ;
            x2 = x1;
            y2 = y1;

        default :
            break;
    }
    dgn_pt = {x1, y1};
    dgn_pt_2 = {x2, y2};
}

Point3D ObjectPose::display2D(float u, float v, bool offset_display, std_msgs::UInt64MultiArray *pt){
  read_yaml();
  off_u = u + offset.x;
  off_v = v + offset.y;

  // Display points for pt, pt2, and pt3 output.
  if(offset_display){
    (*pt).data.push_back(off_u);
    (*pt).data.push_back(off_v);
  }
}

Point3D ObjectPose::uv_to_xyz(float u, float v, bool offset_display, std_msgs::UInt64MultiArray *pt)
{
  //usage: object_depth_client u v

  pose_server::ImgToDepth srv;

  display2D(u, v, offset_display, pt);
  
  srv.request.u = off_u;
  srv.request.v = off_v;
  //std::cout << "================================\n";
  //std::cout << "uv: " << u << ' ' << v << '\n';
  //std::cout << "offset: " << offset.x << ' ' << offset.y << '\n';
  //std::cout << "srv: " << srv.request.u << ' ' << srv.request.v << '\n';
  //std::cout << "-----------------------------------------------\n";
  Point3D pt3D;
  if (client.call(srv))
  {
      // readings are in meter.
      pt3D.x =  (float)srv.response.x;
      pt3D.y =  (float)srv.response.y;
      pt3D.z =  (float)srv.response.z;
      //ROS_INFO("\nx: %f", (float)srv.response.x);
      //ROS_INFO("\ny: %f", (float)srv.response.y);
      //ROS_INFO("\nz: %f", (float)srv.response.z);
      //if(srv.response.x == nan){exit(-1);}
  }
  else
  {
      ROS_ERROR("Failed to call service depth_server!!!");
      exit(-1);
      //return {};
  }
    
  //isnan(x)? std::cout << "Its NaN" << "\n";
  //while(x==nan)
  if(pt3D.z!=pt3D.z){
     ROS_WARN("NaN value detected.");
     isNaN = true;
   } 

  // transform point from zivid_optical_frame to wo_frame
  pt3D = point_transform(pt3D);
  return pt3D;
}

Point3D ObjectPose::point_transform(Point3D pt){
    //tf2_ros::Buffer tfBuffer; 
    //tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::PointStamped initial_pt; 
    geometry_msgs::PointStamped transformed_pt; 
    
    initial_pt.header.frame_id = "zivid_optical_frame";
    initial_pt.point.x = pt.x;
    initial_pt.point.y = pt.y;
    initial_pt.point.z = pt.z;

    while(n.ok()){
        try{
            tfBuffer.transform(initial_pt, transformed_pt, "wo_frame");
        }
        catch(tf2::TransformException &ex){
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        //std::cout << "Done transform!" << "\n";
        break;
    }
    return {(float)transformed_pt.point.x, (float)transformed_pt.point.y, (float)transformed_pt.point.z};
}

//@ADD_ITEM
// use this function after the centroid point is calculated.
void ObjectPose::adjustDepth(int type, Point3D *point){
    int cls;
    if(type == 0) cls = mapStringNum[FinalBox.cls];
    else cls = mapStringNum[className];
    
    //std::cout << "Check range and offset: \n"; 
    for(int i = 0; i < categories.size(); i++){
        for(int j = 0; j < categories[i].size(); j+=2){
            double offset_dpt = static_cast<double>(offset_depth[i][j/2]);
            int range_1 = mapStringNum[categories[i][j]];
            int range_2 = mapStringNum[categories[i][j+1]];
            if(cls >= range_1 && cls <= range_2){
                // special case for broccoli_b, reset center point to average of the two diagonal points.
                if(range_1 == 27 && range_2 == 30){
                    point->z = (dgn_pt3d.z + dgn_pt3d_2.z)/2; 
                }
                point->z -= (float)offset_dpt; 
                std::cout << i << " " << j << ": " << range_1 << " " << range_2 << "\n";
                std::cout << "offset: " << offset_dpt << "\n"; 
                return;
            }
        }
    }
}

void ObjectPose::cal_rpy(int type){
    float del_x;
    float del_y;
    float del_z;

    // calculated from wo_frame. One item: type = 0, two items: type = 1, three items: type = 2. 
    if(type == 0){
        del_x = (dgn_pt3d.x - dgn_pt3d_2.x);
        del_y = (dgn_pt3d.y - dgn_pt3d_2.y);
        del_z = (dgn_pt3d.z - dgn_pt3d_2.z);
    }
    else if(type == 1){
        //del_x = (line_pt3d_1.x - mid_pt3d.x);
        //del_y = (line_pt3d_1.y - mid_pt3d.y);
        //del_z = (line_pt3d_1.z - mid_pt3d.z);

        del_x = (line_pt3d_1.x - line_pt3d_2.x);
        del_y = (line_pt3d_1.y - line_pt3d_2.y);
        del_z = (line_pt3d_1.z - line_pt3d_2.z);
    }
    else if(type == 2){
        del_x = (tri_pt3d_1.x - centroid_pt3d.x);
        del_y = (tri_pt3d_1.y - centroid_pt3d.y);
        del_z = (tri_pt3d_1.z - centroid_pt3d.z);
    }

    rot_3d.rz = atan2(del_y,del_x);
    std::cout << "del_x: " << del_x << "\n";
    std::cout << "del_y: " << del_y << "\n";
    std::cout << "rz: " << rot_3d.rz*180/PI << "\n";
    // angle offset
    //rot_3d.rx = 0;//3.142;
    //rot_3d.rx -= 0.873;

    // Use cos(inc_ang) for inclination angle of the camera 
    // rot_3d.rz = atan2((dgn_pt3d.y - dgn_pt3d_2.y),(dgn_pt3d.x - dgn_pt3d_2.x))-PI/2;
    float xy_dist = sqrt(del_x*del_x+del_y*del_y);
    // Since sqrt or fabs output positive value, atan2 will only output angles at quadrant I and IV, suitable for item with bidirection (e.g. sausage)
    rot_3d.ry = atan2(del_z, xy_dist);

    //if (mapStringNum[FinalBox.cls]>= 6 && mapStringNum[FinalBox.cls]<= 13){
    //    std::cout << "rx: " << rot_3d.rx*180/PI << "ry: " << rot_3d.ry*180/PI << " rz: " << rot_3d.rz*180/PI << '\n';
    //}

  //  std::cout << "rx: " << rot_3d.rx*180/PI << " rz: " << rot_3d.rz*180/PI << '\n';
}

void ObjectPose::convert_to_msg(int type){
    //categorize(type);
    object_transform.header.stamp = ros::Time::now();
    // "zed_wo_frame" has same frame orientation as the "frame_link". center_pt3d and centroid_pt3d are based on "zed_left_camera_frame". When published into "zed_wo_frame", position and orientation of object_transform must be rearranged. 
    object_transform.header.frame_id = "wo_frame";
    if(type == 0){
        object_transform.child_frame_id = object;
        // Rearranged position based on "base_link"
        object_transform.transform.translation.x = center_pt3d.x;
        object_transform.transform.translation.y = center_pt3d.y;
        object_transform.transform.translation.z = center_pt3d.z;
    }
    else if(type == 1){
        object_transform.child_frame_id = "2_" + object;
        // Rearranged position based on "base_link"
        object_transform.transform.translation.x = mid_pt3d.x;
        object_transform.transform.translation.y = mid_pt3d.y;
        object_transform.transform.translation.z = mid_pt3d.z;
    }
    else if(type == 2){
        object_transform.child_frame_id = "3_" + object;
        // Rearranged position based on "base_link"
        object_transform.transform.translation.x = centroid_pt3d.x;
        object_transform.transform.translation.y = centroid_pt3d.y;
        object_transform.transform.translation.z = centroid_pt3d.z;
    }
    tf2::Quaternion q_new;
    // setEuler: Rotate about Y, X and Z using Euler angle rotation.  setEulerZYX: Rotate about Z, Y, and X using Euler angle rotation. setRPY: Rotate about X, Y, and Z using Fixed angle rotation.

    q_new.setEulerZYX(rot_3d.rz, 0, 0);
    
    q_new.normalize();
    object_transform.transform.rotation.x = q_new.x();
    object_transform.transform.rotation.y = q_new.y();
    object_transform.transform.rotation.z = q_new.z();
    object_transform.transform.rotation.w = q_new.w();

    //if (mapStringNum[FinalBox.cls]== 14){
    //std::cout << "xPos: " << center_pt3d.x << "\tyPos: " << center_pt3d.y << "\tzPos: " << center_pt3d.z << '\n';
    std::cout << "Quaternion: \n"<< object_transform.transform.rotation << std::endl;
    //}

   //std::cout << "Quaternion: "<< object_transform.transform.rotation << " rz: "<< -rot_3d.rz*180/PI <<  std::endl;
}

//@ADD_ITEM
// get the object name for the final chosen item 
void ObjectPose::categorize(int type){
    int cls;
    if(type == 0) cls = mapStringNum[FinalBox.cls];
    else cls = mapStringNum[className];
    
    for(int i = 0; i < categories.size(); i++){
        int l = categories[i].size()-1;
        int range_1 = mapStringNum[categories[i][0]];
        int range_2 = mapStringNum[categories[i][l]];
        if(cls >= range_1 && cls <= range_2){
            object = cat_name[i];
            ROS_INFO("%s object.", object.c_str());
            break;
        }
    }
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

void ObjectPose::callZivid(){
    ros::Time begin;
    ros::Time end;
    ros::Duration duration; 
    double cap_duration;
    pose_server::CaptureAssistantSuggestSettings cass_srv;
        cass_srv.request.max_capture_time.sec = 1;
        cass_srv.request.max_capture_time.nsec = 200000000;
        cass_srv.request.ambient_light_frequency = 0;
        begin = ros::Time::now();
        if(client2.call(cass_srv)){
            end = ros::Time::now();
            duration = end - begin;
            cap_duration = duration.toSec();
            ROS_ERROR("Successfully call CaptureAssistantSuggestSettings service in zivid_camera. Total capture time: %f", cap_duration);
        } 
          else
        {
            ROS_ERROR("Failed to call CaptureAssistantSuggestSettings service in zivid_camera.");
            exit(-1);
        }
    
    pose_server::Capture cap_srv;
        begin = ros::Time::now();
        if(client3.call(cap_srv)){
            end = ros::Time::now();
            duration = end - begin;
            cap_duration = duration.toSec();
            ROS_ERROR("Successfully call Capture service in zivid_camera. Total capture time: %f", cap_duration);
        } 
          else
        {
            ROS_ERROR("Failed to call Capture service in zivid_camera.");
            exit(-1);
        }
}


/*
bool ObjectPose::pose_cb(pose_server::Pose::Request  &req, pose_server::Pose::Response &res){
    if(req.get_pose == true){
        // return pose as geometry_msgs/PoseStamped
        res.pose_stmp.header.stamp = ros::Time::now();
        res.pose_stmp.header.frame_id = "/object";
        res.pose_stmp.pose.position.x =;
        res.pose_stmp.pose.position.y =;
        res.pose_stmp.pose.position.z =;
        res.pose_stmp.pose.orientation.x =;
        res.pose_stmp.pose.orientation.y =;
        res.pose_stmp.pose.orientation.z =;
        res.pose_stmp.pose.orientation.w =;

        ROS_INFO("sending back response: [%ld]", (long int)res.sum);
    return true;
    }
}*/

}  /* END of namespace*/




