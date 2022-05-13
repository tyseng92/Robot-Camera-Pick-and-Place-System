#include "pose_server/astra/pose_srv_astra.h"

// Note: Search (Ctrl+F) for @ADD_ITEM to modify the source code for new items.

namespace pose_server{

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

}  /* END of namespace*/




