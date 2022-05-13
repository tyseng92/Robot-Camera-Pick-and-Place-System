#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <std_msgs/UInt64MultiArray.h>
#include <std_msgs/Int8.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <signal.h>
#include <sstream>
#include <fstream>
#include <iostream>
#include <cstdlib>
#include <stdio.h>
#include <time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <stdlib.h>
#include "image_viewer_capture/GetFrame.h"

class MultiSubscriber{    
public:
    MultiSubscriber(ros::NodeHandle *nh):it(*nh){
        // cam properties, use relative name
        nh->param<std::string>("img_topic", cam_topic, "/camera_01/rgb/image_rect_color");
        nh->param<std::string>("pcl_topic", pcl_topic, "/camera_01/depth_registered/points");
        nh->param<std::string>("tray_topic", tray_topic, "/camera_01/tray_id");
        nh->param<std::string>("window_name", WINDOW_NAME, "Window 1");
        nh->param<std::string>("cap_button", CAP_BUTTON, "1");
        nh->param<std::string>("roi_button", ROI_BUTTON, "q");
        nh->param<std::string>("imgpath", IMGPATH, "/Desktop/image_database/yolo/PHASE_2/cam1/image/");
        nh->param<std::string>("pclpath", PCLPATH, "/Desktop/image_database/yolo/PHASE_2/cam1/pcd/");
        nh->param<std::string>("filepath", FILEPATH, "/Desktop/image_database/yolo/PHASE_2/cam1/num.txt");
        nh->param<std::string>("rectpath_1", RECTPATH_1, "/Desktop/image_database/yolo/PHASE_2/cam1/roi_1.yml");
        nh->param<std::string>("rectpath_2", RECTPATH_2, "/Desktop/image_database/yolo/PHASE_2/cam1/roi_2.yml");
        nh->param("roi/x", roi_x, 168);
        nh->param("roi/y", roi_y, 160);
        nh->param("roi/width", roi_w, 224);
        nh->param("roi/height", roi_h, 224);
        // common properties
        nh->param<std::string>("item_name", ITEMNAME, "Image");
        nh->param("capture", capture, true);
        nh->param("res_width", res_w, 640);
        nh->param("res_ratio", res_r, (float)1.333);
        nh->param("win_x", win_x, 0);
        nh->param("win_y", win_y, 0);
        nh->param<std::string>("gtpath", GTPATH, "/Desktop/image_database/yolo/PHASE_2/cam_groundtruth/pcd/");
        OPENCV_WINDOW = WINDOW_NAME;
        B_CAP = CAP_BUTTON;
        B_ROI = ROI_BUTTON;
        char* home = std::getenv("HOME");
        std::string shome(home);
        PATH = shome + IMGPATH + ITEMNAME + "/";
        PclPATH = shome + PCLPATH + ITEMNAME + "/";
        GtPATH = shome + GTPATH + ITEMNAME + "/";
        FPATH = shome + FILEPATH;
        RectPATH_1 = shome + RECTPATH_1;
        RectPATH_2 = shome + RECTPATH_2;
        // pub and sub 
        sub = it.subscribe(cam_topic, 1, &MultiSubscriber::imageCallback, this);
        sub2 = nh->subscribe(pcl_topic, 1, &MultiSubscriber::cloud_cb, this);
        sub3 = nh->subscribe(tray_topic, 1, &MultiSubscriber::roi_cb, this);
        //sub3 = nh->subscribe("/phoxi_camera/pointcloud", 1, &MultiSubscriber::phoxi_cb, this);
        pub = nh->advertise<std_msgs::UInt64MultiArray>("/astra/cpp_image_node/mouse_point", 1000);
        pub2 = it.advertise("/astra/cpp_image_node/image_roi", 1);
        client = nh->serviceClient<image_viewer_capture::GetFrame>("/phoxi_camera/get_frame");

        if(capture)cv::namedWindow(OPENCV_WINDOW);

        // Uncomment this line will cause "Attempt to unlock mutex that was not locked" error in Ubuntu 18
        // Link: https://stackoverflow.com/questions/33858902/attempt-to-unlock-mutex-that-was-not-locked-aborted-when-using-waitkey
        //cv::startWindowThread();

        //count = 1;
        once = 1;
        not_first_cb = false;
        max_time = 0;
        tray_id = 1;

        std::cout << "PATH: " << PATH << std::endl;
        std::cout << "PclPATH: " << PclPATH << std::endl;
        std::cout << "FPATH: " << FPATH << std::endl;
        std::cout << "RectPATH_1: " << RectPATH_1 << std::endl;
        std::cout << "RectPATH_2: " << RectPATH_2 << std::endl;
        std::cout << "ITEMNAME: " << ITEMNAME << "\n" << std::endl;

        set_tray_id(tray_id);

        std::cout << "Multiastra is started.\n";
        ROS_INFO("Starting node...");

        infile.open(FPATH);
        infile >> count;
        infile.close();

        // roi.x = roi_x;
        // roi.y = roi_y;
        // roi.width = roi_w;
        // roi.height = roi_h;
        
    }
    
    ~MultiSubscriber(){
        cv::destroyWindow(OPENCV_WINDOW);
        std::cout << "Multiastra is closed.\n";
        //ROS_INFO("Image Database is closed.");
    }
    
    void write_yaml(std::string rectpath){
        // cv::FileStorage fs;
        std::cout << "rectpath: " << rectpath << std::endl;
        // fs.write(rectpath, cv::FileStorage::WRITE);
        // if( fs.isOpened() ){
        //     fs << "x" << roi.x << "y" << roi.y;
        //     fs << "width" << roi.width << "height" << roi.height;
        //     fs.release();
        // }
        // else std::cout << "Error: can not save the rect\n";
        cv::FileStorage fs(rectpath, cv::FileStorage::WRITE);
        fs << "x" << roi.x << "y" << roi.y;
        fs << "width" << roi.width << "height" << roi.height;
        fs.release();
    }

    void read_yaml(std::string rectpath){
        cv::FileStorage fs;
        std::cout << "rectpath: " << rectpath << "\n";
        fs.open(rectpath, cv::FileStorage::READ);
        if( fs.isOpened() ){
            fs["x"] >> roi.x;
            fs["y"] >> roi.y;
            fs["width"] >> roi.width;
            fs["height"] >> roi.height;
        }
        else std::cout << "Error: can not load the rect\n";
    }

    // https://stackoverflow.com/questions/25748404/how-to-use-cvsetmousecallback-in-class
    static void mouse_cb(int event, int x, int y, int flags, void* userdata){
        MultiSubscriber* db = reinterpret_cast<MultiSubscriber*>(userdata);
        db->mouse_cb(event, x, y);
    }

    void mouse_cb(int event, int x, int y){
        if(event == cv::EVENT_LBUTTONDOWN)
        {  
            std_msgs::UInt64MultiArray pt;
            pt.data.clear();
            pt.data.push_back(x);
            pt.data.push_back(y);
            pub.publish(pt);
            std::cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;
        }
    }

    void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input){
        // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
        //pcl::fromROSMsg(*input, cloud); // without rgb 
        pcl::fromROSMsg(*input, rgb_cloud);  // with rgb
        //std::cout << "stored point cloud!\n";
    }

    // void phoxi_cb(const sensor_msgs::PointCloud2ConstPtr& input){
    //     // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    //     pcl::fromROSMsg(*input, phoxi_cloud);
    //     std::cout << "stored point cloud for phoxi!\n";
    // }

    void roi_cb(const std_msgs::Int8::ConstPtr& id){
        tray_id = id->data;
        set_tray_id(tray_id);
    }

    void set_tray_id(int id){
        if (id == 1){
            read_yaml(RectPATH_1);
        }
        else if (id == 2){
            read_yaml(RectPATH_2);
        }
        else {
            std::cout << "Tray ID is not included." << "\n";
        }
        std::cout << "Tray_ID: " << id  << "\n";
    }

    void call_get_frame(){
        image_viewer_capture::GetFrame srv;
        srv.request.in = -1;
        if (client.call(srv)){
            std::cout << "Message: " << srv.response.message << "\n";
            std::cout << "Success: " << srv.response.success << "\n";
        }
        else
        {
            ROS_ERROR("Failed to call service \"/phoxi_camera/get_frame\"!!!");
            exit(-1);
            //return {};
        }
    }
    

    void save_pcl(int count){
        pclname = PclPATH + ITEMNAME + "_" + std::to_string(count) + ".pcd";
        gtname = GtPATH + ITEMNAME + "_" + std::to_string(count) + ".pcd";
        //pcl::io::savePCDFileBinary(pclname, cloud);
        pcl::io::savePCDFileBinary(pclname, rgb_cloud);  
        //pcl::io::savePCDFileASCII(pclname, cloud); 
        //cl::io::savePCDFile(pclname, cloud);
        //if (OPENCV_WINDOW == "Astra Cam 1"){
            // ros topic of "/phoxi_camera/pointcloud" is too slow in receiving the point cloud from phoxi camera 
            //call_get_frame();
            //pcl::io::savePCDFileBinary(gtname, phoxi_cloud);
        //}
    }

    char show_cvimage(std::string window_name, cv::Mat& output_img, int wait_key){
        cv::resize(output_img, output_img, cv::Size(res_w,res_w/res_r));
        cv::imshow(window_name, output_img);
        char key = cv::waitKey(wait_key);
        cv::setMouseCallback(window_name, mouse_cb, this);
        return key;
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg){
        //if (ros::isShuttingDown()){std::cout << "shut down\n";}

        try
        {   // Show video display through camera, convert from ros msg to opencv image format using pointer          
            //cv::imshow(OPENCV_WINDOW, cv_bridge::toCvShare(msg, "bgr8")->image);
            //ROS_INFO("image...");
            
            double time = ros::Time::now().toSec();
            //ROS_INFO("image time: %f", time );
            // Copy image from ros msgs for ROI display
            cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
            
            cv_ptr->image = cv_ptr->image(roi);
            pub2.publish(cv_ptr->toImageMsg());

            // restore from roi to original
            cv_ptr=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
            

            // Display resolution of image
            if (once == 1){
                ROS_INFO("Left Rectified image received from ASTRA - Size: %dx%d",
             msg->width, msg->height);
             once = 0;
            }

            // Display resolution of image
            if (once == 1){
                ROS_INFO("Left Rectified image received from ASTRA - Size: %dx%d",
             msg->width, msg->height);
             once = 0;
            }

            if(capture){
                // Press 'f' to save the current frame into .jpg files
                char key = show_cvimage(OPENCV_WINDOW, cv_ptr->image, 100);
                //char key = cv::waitKey(1);
                if (key == B_CAP[0]) {    
                    // create another copy of ros image for modification.
                    //cv_ptr=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
                    set_tray_id(tray_id);
                    //do{
                    // Store img index number 
                    imgname = PATH + ITEMNAME + "_" + std::to_string(count) + ".jpg";
                    cv::Mat img_roi = cv_ptr->image(roi);
                    cv::imwrite(imgname, img_roi);
                    ROS_INFO("Image Captured: %s", imgname.c_str());

                    save_pcl(count);

                    count += 1;
                    outfile.open(FPATH);
                    outfile << count; 
                    outfile.close();
                        
                    //}while(count % 5 == 0);
                }
                // Press 'r' to select roi area 
                else if (key == B_ROI[0]){
                    //cv_ptr=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
                    roi = cv::selectROI(cv_ptr->image);
                    write_yaml(RectPATH_1);
                    cv::destroyWindow("ROI selector");
                }
            }

        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // calculate time
        if(not_first_cb){
            end = ros::Time::now();
            ros::Duration cb_time = end - begin;
            if(cb_time.toSec() > max_time){
                max_time = cb_time.toSec();
            }
            //ROS_INFO("Total call back time: %f ", (double)cb_time.toSec());
            //ROS_INFO("Max call back time: %f ", (double)max_time);
        }

        not_first_cb = true;
        begin = ros::Time::now();
    }
    
    
protected:

    image_transport::ImageTransport it;
    image_transport::Subscriber sub;
    ros::Subscriber sub2;
    ros::Subscriber sub3;
    ros::Publisher pub;
    image_transport::Publisher pub2;
    ros::ServiceClient client;

    std::string imgname;
    std::string pclname;
    std::string gtname;
    cv_bridge::CvImagePtr cv_ptr;
    int count;
    int once;
    cv::Rect2d roi;
    
    cv::Point pt;

    std::string cam_topic;
    std::string pcl_topic;
    std::string tray_topic;
    bool capture;
    ros::Time begin;
    ros::Time end;
    float max_time;
    bool not_first_cb;
    int res_w;
    float res_r;
    int win_x;
    int win_y;
    int roi_x;
    int roi_y;
    int roi_w;
    int roi_h;
    int tray_id;

    std::string WINDOW_NAME;
    std::string CAP_BUTTON;
    std::string ROI_BUTTON;
    std::string IMGPATH;
    std::string FILEPATH;
    std::string RECTPATH_1;
    std::string RECTPATH_2;
    std::string PCLPATH;
    std::string GTPATH;

    std::string PATH;
    std::string FPATH;
    std::string RectPATH_1;
    std::string RectPATH_2;
    std::string PclPATH;
    std::string GtPATH;
    std::string OPENCV_WINDOW;
    std::string B_CAP;
    std::string B_ROI;

    std::string ITEMNAME;

    std::ifstream infile;
    std::ofstream outfile;
    //pcl::PointCloud<pcl::PointXYZ> cloud;
    //pcl::PointCloud<pcl::PointXYZ> phoxi_cloud;
    pcl::PointCloud<pcl::PointXYZRGB> rgb_cloud;

};

void mySigintHandler(int sig)
{
  // Do some custom action.
  // For example, publish a stop message to some other nodes.
  ROS_INFO("Shutting down...");
  // All the default sigint handler does is call shutdown()
  ros::shutdown();
}

int main(int argc, char **argv){
    //capture_images(argc, argv);
    ///initialize node
    ros::init(argc, argv, "multiastra_subscriber", ros::init_options::AnonymousName);
    //start node
    // Check this link for relative parameter problem: https://answers.ros.org/question/11098/roscpp-relative-parameter/
    ros::NodeHandle nh("~");

    MultiSubscriber ms(&nh);
    signal(SIGINT, mySigintHandler);
    ros::spin();
    return 0;
}
