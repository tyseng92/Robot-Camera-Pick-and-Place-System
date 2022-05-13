#include "pose_server/astra/pose_srv_astra.h"

// Note: Search (Ctrl+F) for @ADD_ITEM to modify the source code for new items.
// Note: Search (Ctrl+F) for @CHANGE_FRAME to change frames.
namespace pose_server{

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

Point3D ObjectPose::uv_to_xyz(float u, float v, bool offset_display, std_msgs::UInt64MultiArray *pt){
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

  // transform point from camera_01_depth_optical_frame to wo_frame
  pt3D = point_transform(pt3D);
  return pt3D;
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

//@CHANGE_FRAME
Point3D ObjectPose::point_transform(Point3D pt){
    //tf2_ros::Buffer tfBuffer; 
    //tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::PointStamped initial_pt; 
    geometry_msgs::PointStamped transformed_pt; 
    
    initial_pt.header.frame_id = "camera_01_depth_optical_frame";
    initial_pt.point.x = pt.x;
    initial_pt.point.y = pt.y;
    initial_pt.point.z = pt.z;

    while(n.ok()){
        try{
            //tfBuffer.transform(initial_pt, transformed_pt, "wo_frame");
            //tfBuffer.transform(initial_pt, transformed_pt, "ref_link");
            tfBuffer.transform(initial_pt, transformed_pt, "base_link");
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

//@CHANGE_FRAME
void ObjectPose::convert_to_msg(int type){
    //categorize(type);
    object_transform.header.stamp = ros::Time::now();
    // "zed_wo_frame" has same frame orientation as the "frame_link". center_pt3d and centroid_pt3d are based on "zed_left_camera_frame". When published into "zed_wo_frame", position and orientation of object_transform must be rearranged. 
    //object_transform.header.frame_id = "wo_frame";
    object_transform.header.frame_id = "base_link";
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

}  /* END of namespace*/




