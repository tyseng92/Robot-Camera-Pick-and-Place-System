#include "pose_server/zivid/pose_srv_zivid.h"

namespace pose_server{

ObjectPose::ObjectPose(ros::NodeHandle *nh):n(*nh), tfListener(tfBuffer){
    sub = n.subscribe("/darknet_ros_zivid/found_object", 1, &ObjectPose::obj_cb, this);
    sub2 = n.subscribe("/darknet_ros_zivid/bounding_boxes", 1, &ObjectPose::box_cb, this);
    sub3 = n.subscribe("/darknet_ros_zivid/detection_image", 1, &ObjectPose::img_cb, this);
    pub = n.advertise<std_msgs::UInt64MultiArray>("/pose_server/FinalBoxCenter", 100);
    pub2 = n.advertise<std_msgs::UInt64MultiArray>("/pose_server/FinalTriCentroid", 100);
    client = n.serviceClient<pose_server::ImgToDepth>("depth_server_zivid");
    client2 = n.serviceClient<pose_server::CaptureAssistantSuggestSettings>("/zivid_camera/capture_assistant/suggest_settings");
    client3 = n.serviceClient<pose_server::Capture>("/zivid_camera/capture");
    service = n.advertiseService("pose_server", &ObjectPose::pose_cb, this);
    n.param("/pose_server/depth_pt_ratio", ratio, 0.4);
    n.param("/pose_server/cam_ground_distance", cam_ground_distance, (float)1.0);
    n.param("/pose_server/max_tray_height", max_tray_height, (float)0.08);
    n.param("/pose_server/tray_num", tray_num, 1);
    n.param("/pose_server/w_ratio", w_ratio, (float)0.2);
    n.param("/pose_server/h_ratio", h_ratio, (float)0.167);
    n.param("/pose_server/h_offset", h_offset, 0);
    n.param("/pose_server/inc_ang_x", inc_ang_x, (float)0.2758);
    n.param("/pose_server/inc_ang_y", inc_ang_y, (float)0.002134);
    n.param("/pose_server/tri_len", tri_len, 80);
    n.param("/pose_server/tri_ang", tri_ang, 90);
    n.param("/pose_server/offset_brc_b", offset_brc_b, float(0.025));
    set_ratio();

    char* home = std::getenv("HOME");
    std::string s(home);
    std::string rectpath = "/Desktop/image_database/yolo/DATABASE_DEC/zivid/roi.yml";
    
    n.param<std::string>("/pose_server/rectpath", RECTPATH, rectpath);
    PATH = home + RECTPATH;
    isNaN = false;
    //dataReady = false;
    max_time = 0;

    ROS_INFO("Starting ObjectPose node...");
}

ObjectPose::~ObjectPose(){
    ROS_INFO("ObjectPose is closed.");
}

void ObjectPose::set_ratio(){
    ratio_min = ratio;
    ratio_max = 1 - ratio;
}

void ObjectPose::obj_cb(const std_msgs::Int8::ConstPtr &fd_obj){
    num = fd_obj->data;
}

// find the box with highest probability. 
void ObjectPose::box_cb(const pose_server::BoundingBoxes::ConstPtr &msg){
    ros::Time begin = ros::Time::now();

    std::cout << "1===============================================\n";
    Boxes_Potato.clear();
    Boxes_Tomato.clear();
    Boxes_Broccoli.clear();
    Boxes_Sausage.clear();
    //Boxes_Omelette.clear();
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

        // Sausage
        if(mapStringNum[box.cls] >= 0 && mapStringNum[box.cls] <= 5){
            Boxes_Sausage.push_back(box);
        }
        // Potato
        else if(mapStringNum[box.cls] >= 6 && mapStringNum[box.cls] <= 13){
            Boxes_Potato.push_back(box);
        }
        // Tomato
        else if(mapStringNum[box.cls]==14){
            Boxes_Tomato.push_back(box);
        }
        // Broccoli
        else if(mapStringNum[box.cls]>= 15 && mapStringNum[box.cls]<=30){
            Boxes_Broccoli.push_back(box);
        }
        // Omelette
        else if(mapStringNum[box.cls]>= 31 && mapStringNum[box.cls]<=34){
            Boxes_Omelette.push_back(box);
        }
        else{
            ROS_ERROR("Class number out of range error!!");
            exit(-1);
        }
    }

    //All_Boxes.push_back(Boxes_Sausage);
    //All_Boxes.push_back(Boxes_Broccoli);
    //All_Boxes.push_back(Boxes_Tomato);
    //All_Boxes.push_back(Boxes_Potato);
    //All_Boxes.push_back(Boxes_Potato);

    // Check for image_database data sequence if the getTransform functions below are modified.
    getTransform(Boxes_Potato);
    getTransform(Boxes_Broccoli);
    getTransform(Boxes_Sausage);
    getTransform(Boxes_Tomato);
    //getTransform(Boxes_Omelette);
    //getTransformSp(Boxes_Potato);
    //getTransformSp(Boxes_Tomato);
    //getTransformSp(Boxes_Broccoli);
    //getTransformSp(Boxes_Sausage);

    //wo_transform();
    //confirm_transform = pack_transform;
 
    // If all of the getTransform does not detect any boxes, there will be no display shown, returning out of the box_cb.r
    if (std::all_of(no_obj.begin(), no_obj.end(), [](bool v) { return v; })){
        no_obj.clear();
        std::cout << "2!===============================================\n";
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
    ROS_INFO("Total call back time: %f ", (double)cb_time.toSec());
    ROS_INFO("Max call back time: %f ", (double)max_time);
    std::cout << "2===============================================\n";
}


void ObjectPose::getTransform(std::vector<Box> Boxes){
    //std::cout << "T1===============================================\n";

    // adjust box probability of box near the side of tray
    height_pb(&Boxes);

    // Avoid gripper hits the side of the tray
    ignore_side(&Boxes);
    
    // find the final box
    finalbox(&Boxes);
    if (max_pb == 0){
        //std::cout << "T2!===============================================\n";
        return;
    }
    
    // instead of using the coordinate of roi, this function set an offset, changing the coordinate of the final box to camera full image.
    finalBoxesDisplay();

    // get 2 ends of sausage which has highest probability
    get_dgn_pt();

    // x, y, and z depth of two ends
    dgn_pt3d = uv_to_xyz(dgn_pt.x, dgn_pt.y, true , &pt);
    dgn_pt3d_2 = uv_to_xyz(dgn_pt_2.x, dgn_pt_2.y, true, &pt);

    // x, y, and z depth of center point
    center_pt3d = uv_to_xyz(center_pt.x, center_pt.y, true, &pt);

    std::cout << "center_pt3d x: " << center_pt3d.x << "\n";
    std::cout << "center_pt3d y: " << center_pt3d.y << "\n";
    std::cout << "center_pt3d z: " << center_pt3d.z << "\n";

    // For food item with different center point
    adjustDepth();

    //calculate yaw and pitch angle of the object
    cal_rpy(false);

    // Ignore Nan value from depth measurement.
    if(isNaN){
        isNaN = false;
        //std::cout << "T2!!===============================================\n";
        return;
    }
    // Convert to PoseStamped message with quaternion
    convert_to_msg(false);

    // pack object name for delta socket to use
    //if (pack_name.size()==tray_num){
    //    pack_name.push_back(object_transform.child_frame_id);
    //}
    //else{
    //    pack_name.clear();
    //    pack_name.push_back(object_transform.child_frame_id);
    //}

    // publish the pose of object when requested by client
    br.sendTransform(object_transform);

    //std::cout << "T2===============================================\n";

}

void ObjectPose::getTransformSp(std::vector<Box> Boxes){
    //std::cout << "C1===============================================\n";

    // Avoid gripper hits the side of the tray
    ignore_side(&Boxes);

    bruteForceMinTri(&Boxes);
    if (tri.size() == 0){return;}
    tri.clear();
    ROS_INFO("Finish calculating min triangle.");

    Point2D c_pt = centroid(min_tri.p1, min_tri.p2, min_tri.p3);
    std::cout << "Centroid: " << c_pt.x << " "<< c_pt.y << '\n';

    tri_pt3d_1 = uv_to_xyz(min_tri.p1.x, min_tri.p1.y, true, &pt2);
    tri_pt3d_2 = uv_to_xyz(min_tri.p2.x, min_tri.p2.y, true, &pt2);
    tri_pt3d_3 = uv_to_xyz(min_tri.p3.x, min_tri.p3.y, true, &pt2);
    centroid_pt3d = uv_to_xyz(c_pt.x, c_pt.y, true, &pt2);

    // Get the average depth of three of the triangle items
    centroid_pt3d.z = (tri_pt3d_1.z+tri_pt3d_2.z+tri_pt3d_3.z)/3;
    
    cal_rpy(true);
    
    // For transformation object naming.
    className = Boxes[0].cls;
    // Ignore Nan value from depth measurement.
    if(isNaN){
        isNaN = false;
        return;
    }
    // Convert to PoseStamped message with quaternion for centroid of 3 items
    convert_to_msg(true);
    br.sendTransform(object_transform);
    //std::cout << "C2===============================================\n";
}

// in case we need to process the image with bounding box
void ObjectPose::img_cb(const sensor_msgs::Image::ConstPtr &img){

}

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
        std::cout << "No triangle is detected!" << '\n';
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
        if (mapStringNum[(*it).cls]>= 27 && mapStringNum[(*it).cls]<= 30){
            d_pt.z += offset_brc_b; 
        }

        // Use cos(inc_ang) for inclination angle of the camera 
        float trans_depth = d_pt.z*cos(inc_ang_x)-(-d_pt.y)*(sin(inc_ang_x));
        //std::cout << "trans_depth: " << trans_depth << "\n";
        //std::cout << "cam_ground_distance: " << cam_ground_distance << "\n";
        float h_pb = (cam_ground_distance-trans_depth)/max_tray_height;  // height of object/max obj height for probability calculation
        // height is used as main factor of choosing the final box
        (*it).height = h_pb*10;  

        std::cout << "Height prob " << (*it).cls << ": " << (*it).height << ' ' << (*it).near_side << '\n';
    
        //if (mapStringNum[(*it).cls]>= 6 && mapStringNum[(*it).cls]<= 13){
        //std::cout << "cam_ground: " << cam_ground_distance << '\n'; 
        //}
    }



  //  std::cout << "cam_ground: " << cam_ground_distance << '\n';
}

void ObjectPose::finalbox(std::vector<Box> *Boxes){
    max_pb = 0;
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
    std::cout << "max_pb: " << max_pb << '\n';
    if (max_pb == 0){
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
    //for(std::vector<size_t>::iterator it = pt.data.begin(); it != pt.data.end(); ++it){
    //    std::cout << "pt " <<  it-pt.data.begin() << ":" << (*it) << std::endl;
    //}
    pub.publish(pt);
    pub2.publish(pt2);
    pt.data.clear();
    pt2.data.clear();
}

void ObjectPose::initialize(){
    mapStringNum["stick_0"]=0;
    mapStringNum["stick_30"]=1;
    mapStringNum["stick_60"]=2;
    mapStringNum["stick_90"]=3;
    mapStringNum["stick_120"]=4;
    mapStringNum["stick_150"]=5;
    mapStringNum["potato_0"]=6;
    mapStringNum["potato_45"]=7;
    mapStringNum["potato_90"]=8;
    mapStringNum["potato_135"]=9;
    mapStringNum["potato_180"]=10;
    mapStringNum["potato_225"]=11;
    mapStringNum["potato_270"]=12;
    mapStringNum["potato_315"]=13;
    mapStringNum["tomato"]=14;
    mapStringNum["broccoli_s_0"]=15;
    mapStringNum["broccoli_s_45"]=16;
    mapStringNum["broccoli_s_90"]=17;
    mapStringNum["broccoli_s_135"]=18;
    mapStringNum["broccoli_s_180"]=19;
    mapStringNum["broccoli_s_225"]=20;
    mapStringNum["broccoli_s_270"]=21;
    mapStringNum["broccoli_s_315"]=22;
    mapStringNum["broccoli_t_0"]=23;
    mapStringNum["broccoli_t_45"]=24;
    mapStringNum["broccoli_t_90"]=25;
    mapStringNum["broccoli_t_135"]=26;
    mapStringNum["broccoli_b_0"]=27;
    mapStringNum["broccoli_b_45"]=28;
    mapStringNum["broccoli_b_90"]=29;
    mapStringNum["broccoli_b_135"]=30;
    mapStringNum["omelette_0"]=31;
    mapStringNum["omelette_45"]=32;
    mapStringNum["omelette_90"]=33;
    mapStringNum["omelette_135"]=34;
    mapStringNum["output_tray"]=35;

}

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

Point3D ObjectPose::uv_to_xyz(float u, float v, bool offset_display, std_msgs::UInt64MultiArray *pt)
{
  //usage: object_depth_client u v

  pose_server::ImgToDepth srv;
  read_yaml();
  off_u = u + offset.x;
  off_v = v + offset.y;

  // Display points for pt and pt2 output.
  if(offset_display){
    (*pt).data.push_back(off_u);
    (*pt).data.push_back(off_v);
  }
  
  srv.request.u = off_u;
  srv.request.v = off_v;
  //std::cout << "===============================================\n";
  //std::cout << "uv: " << u << ' ' << v << '\n';
  //std::cout << "offset: " << offset.x << ' ' << offset.y << '\n';
  //std::cout << "srv: " << srv.request.u << ' ' << srv.request.v << '\n';
  //std::cout << "-----------------------------------------------\n";
  float x,y,z;
  if (client.call(srv))
  {
      // time 1000 to get mm.
      x =  (float)srv.response.x;
      y =  (float)srv.response.y;
      z =  (float)srv.response.z;
      //ROS_INFO("\nx: %f", (float)srv.response.x);
      //ROS_INFO("\ny: %f", (float)srv.response.y);
      //ROS_INFO("\nz: %f", (float)srv.response.z);
      //if(srv.response.x == nan){exit(-1);}
  }
  else
  {
      ROS_ERROR("Failed to call service depth_server");
      exit(-1);
      //return {};
  }
    
  //isnan(x)? std::cout << "Its NaN" << "\n";
  //while(x==nan)
  if(x!=x){
     ROS_WARN("NaN value detected.");
     isNaN = true;
   } 
  
  return {x, y, z};
}

// use this function after the centroid point is calculated.
void ObjectPose::adjustDepth(){
    // For food item with different center point.
    // broccoli_b, reset center point to average of the two diagonal points.
    if (mapStringNum[FinalBox.cls]>= 27 && mapStringNum[FinalBox.cls]<= 30){
        center_pt3d.z = (dgn_pt3d.z + dgn_pt3d_2.z)/2; 
    }
}

void ObjectPose::cal_rpy(bool centroid){
    float del_x;
    float del_y;
    float del_z;

    if(!centroid){
        //del_x = (dgn_pt3d.x*cos(inc_ang_x) - dgn_pt3d_2.x*cos(inc_ang_x));
        //del_y = (dgn_pt3d.y - dgn_pt3d_2.y);
        //del_z = (dgn_pt3d.z - dgn_pt3d_2.z);
        del_x = (dgn_pt3d.x - dgn_pt3d_2.x);
        del_y = (dgn_pt3d.y - dgn_pt3d_2.y);
        del_z = (dgn_pt3d.z*cos(inc_ang_x) - dgn_pt3d_2.z*cos(inc_ang_x));
//        std::cout << "(atan2) del_z: " << del_z << " del_y: " <<  del_y << " del_x: " <<  del_x << '\n';
    }
    else{
        //del_x = (tri_pt3d_1.x*cos(inc_ang_x) - centroid_pt3d.x*cos(inc_ang_x));
        //del_y = (tri_pt3d_1.y - centroid_pt3d.y);
        //del_z = (tri_pt3d_1.z - centroid_pt3d.z);
        del_x = (tri_pt3d_1.x - centroid_pt3d.x);
        del_y = (tri_pt3d_1.y - centroid_pt3d.y);
        del_z = (tri_pt3d_1.z*cos(inc_ang_x) - centroid_pt3d.z*cos(inc_ang_x));
    }

    rot_3d.rz = atan2(del_y,del_x);
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

void ObjectPose::convert_to_msg(bool centroid){
    categorize(centroid);
    object_transform.header.stamp = ros::Time::now();
    // "zed_wo_frame" has same frame orientation as the "frame_link". center_pt3d and centroid_pt3d are based on "zed_left_camera_frame". When published into "zed_wo_frame", position and orientation of object_transform must be rearranged. 
    object_transform.header.frame_id = "zivid_wo_frame";
    if(!centroid){
        object_transform.child_frame_id = object;
        // Rearranged position based on "base_link"
        object_transform.transform.translation.x = -center_pt3d.x/1000;
        object_transform.transform.translation.y = center_pt3d.y/1000;
        object_transform.transform.translation.z = -center_pt3d.z/1000;
    }
    else {
        object_transform.child_frame_id = "3_" + object;
        // Rearranged position based on "base_link"
        object_transform.transform.translation.x = -centroid_pt3d.x/1000;
        object_transform.transform.translation.y = centroid_pt3d.y/1000;
        object_transform.transform.translation.z = -centroid_pt3d.z/1000;
    }
    tf2::Quaternion q_new, q_rot, q_org;
    
    // setEuler: Rotate about Y, X and Z using Euler angle rotation.  setEulerZYX: Rotate about Z, Y, and X using Euler angle rotation. setRPY: Rotate about X, Y, and Z using Fixed angle rotation.

    // Quaternion adjusted with camera incline angle.
    q_rot.setRPY(inc_ang_x, inc_ang_y, 0);

    // Quaternion adjusted with object orientation.
    q_org.setEulerZYX(-rot_3d.rz, 0, 0);
    //q_org.setEulerZYX(rot_3d.rx, rot_3d.rz, 0);
    //q_new.setEuler(0, rot_3d.rx, 0);
    
    //q_new = q_org;
    // Rotated q_org by q_rot, change into q_new
    q_new = q_rot*q_org;
    
    q_new.normalize();
    object_transform.transform.rotation.x = q_new.x();
    object_transform.transform.rotation.y = q_new.y();
    object_transform.transform.rotation.z = q_new.z();
    object_transform.transform.rotation.w = q_new.w();

    //if (mapStringNum[FinalBox.cls]== 14){
    //std::cout << "xPos: " << center_pt3d.x << "\tyPos: " << center_pt3d.y << "\tzPos: " << center_pt3d.z << '\n';
    //std::cout << "Quaternion: \n"<< object_transform.transform.rotation << std::endl;
    //}

   //std::cout << "Quaternion: "<< object_transform.transform.rotation << " rz: "<< -rot_3d.rz*180/PI <<  std::endl;
}

void ObjectPose::categorize(bool centroid){
    int cls;
    if(!centroid) cls = mapStringNum[FinalBox.cls];
    else cls = mapStringNum[className];
    
    if (cls >= 0 && cls <= 5){
        ROS_INFO("Sausage object.");
        object = "sausage";
    }
    else if (cls >= 6 && cls <= 13){
        ROS_INFO("Potato object.");
        object = "potato";
    }
    else if (cls == 14){
        ROS_INFO("Tomato object.");
        object = "tomato";
    }
    else if (cls >= 15 && cls <= 30){
        ROS_INFO("Broccoli object.");
        object = "broccoli";
    }
    else if (cls >= 31 && cls <= 34){
        ROS_INFO("Omelette object.");
        object = "omelette";
    }
}

// get correct transformation from wo_frame for object in pack_name and record them.
/*
void ObjectPose::wo_transform(){
    //confirm_transform.clear();
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped transformStamped;
    for(auto it = pack_name.begin(); it != pack_name.end(); ++it){
        while (n.ok()){ 
            try{
                std::cout << "test transform "<< (*it) << "\n";
                transformStamped = tfBuffer.lookupTransform((*it), "wo_frame",
                                        ros::Time(0));
            }
            catch (tf2::TransformException &ex) {
                ROS_WARN("%s",ex.what());
                ros::Duration(1.0).sleep();
                continue;
            }
            break;
        }
        confirm_transform.push_back(transformStamped);
    }
}
*/
bool ObjectPose::pose_cb(pose_server::PoseZividSrv::Request  &req, pose_server::PoseZividSrv::Response &res){
    if(req.get_pose == true){
        ROS_INFO("Receive request from client.");
        
        // call zivid driver to capture the data.
        //callZivid();

        // wait for data to be processed.
        //while(!dataReady){
        //    ros::Duration(0.1).sleep();
        //    dataReady = false;
        //}
        /*
        res.sdata.push_back("Broccoli");
        res.data.push_back(0.123456);
        res.data.push_back(6.001);
        res.data.push_back(5.002);
        //res.data.push_back((*it).rx);
        res.data.push_back(10.123456);
        
        
        res.sdata.push_back("Potato");
        res.data.push_back(-0.123456);
        res.data.push_back(0.55);
        res.data.push_back(5.002);
        //res.data.push_back((*it).rx);
        res.data.push_back(10.123456);
        
        res.sdata.push_back("Sausage");
        res.data.push_back(0.123456);
        res.data.push_back(-0.55);
        res.data.push_back(5.002);
        //res.data.push_back((*it).rx);
        res.data.push_back(10.123456);

        res.sdata.push_back("Tomato");
        res.data.push_back(0.123456);
        res.data.push_back(0.55);
        res.data.push_back(5.00);
        //res.data.push_back((*it).rx);
        res.data.push_back(10.123456);
        
        //std::cout << "name: " << (*it).name << std::endl;
        //ROS_INFO("sending back response: x=[%f] y=[%f] z=[%f] az=[%f]", (float)(*it).x, (float)(*it).y, (float)(*it).z, (float)(*it).rz);
        */
        /*
        wo_transform();
        for(auto it = confirm_transform.begin(); it != confirm_transform.end(); ++it){
            res.sdata.push_back((*it).child_frame_id);
            res.data.push_back((*it).transform.translation.x*1000);
            res.data.push_back((*it).transform.translation.y*1000);
            res.data.push_back((*it).transform.translation.z*1000);
            //res.data.push_back((*it).transform.rotation.x);
            res.data.push_back((*it).transform.rotation.z);
            std::cout << "name: " << (*it).child_frame_id << std::endl;
            ROS_INFO("sending back response: x=[%f] y=[%f] z=[%f] az=[%f]", (float)(*it).transform.translation.x*1000, (float)(*it).transform.translation.y*1000, (float)(*it).transform.translation.z*1000, (float)(*it).transform.rotation.z);
        }
        ROS_INFO("----------------------------------------------------\n");
        */
        //ROS_INFO("sending back response: x=[%f] y=[%f] z=[%f] ax=[%f] az=[%f]", (float)res.x, (float)res.y, (float)res.z, (float)res.rx, (float)res.rz);
        return true;
    }
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
            transformed_pt = tfBuffer.transform(initial_pt, "wo_frame");
        }
        catch(tf2::TransformException &ex){
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        break;
    }
    return {(float)transformed_pt.point.x, (float)transformed_pt.point.y, (float)transformed_pt.point.z};
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




