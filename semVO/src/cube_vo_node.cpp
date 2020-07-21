
/*******************************************************
 * please first launch KITTIOdomTest.cpp in VINS_Fusion of vins_estimator
 *
 *******************************************************/

#include <iostream>
#include <queue>
#include <thread>
#include <mutex>
#include <cmath>
#include <string>
#include <ros/ros.h>
// sync time
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <boost/bind.hpp>

#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include "landmark.h"
#include "feature_tracker.h"
#include "DataManager.h"
#include "Frame.h"
#include "estimator.h"
#include "visual_odometry.h"

using namespace std;
using namespace Eigen;


//Estimator estimator;
VisualOdometry::Ptr vo(new VisualOdometry);
Camera::Ptr camera(new Camera);

queue<darknet_ros_msgs::BoundingBoxes> frame_bboxes_buf;
queue<sensor_msgs::Image> img_buf;
std::mutex m_buf;

#define __DEBUG__(msg) msg;
void left_image_callback(const sensor_msgs::Image& msg){
    m_buf.lock();
    img_buf.push(msg);
    m_buf.unlock();
}

void frame_bboxes_callback(const darknet_ros_msgs::BoundingBoxes& msg){
    m_buf.lock();
    frame_bboxes_buf.push(msg);
    m_buf.unlock();
}

cv::Mat getImageFromMsg(const sensor_msgs::Image &msg){
    cv_bridge::CvImagePtr cv_ptr;
    if(msg.encoding == "8UC1"){
        sensor_msgs::Image img;
        img.header = msg.header;
        img.height = msg.height;
        img.width = msg.width;
        img.is_bigendian = msg.is_bigendian;
        img.step = msg.step;
        img.data = msg.data;
        img.encoding = "BGR8";
        cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    }
    else
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat img = cv_ptr->image.clone();
    return img;
}

void image_frame_bboxes_callback(const sensor_msgs::Image image, const darknet_ros_msgs::BoundingBoxes frame_bboxes)
{

}

void sync_process()
{
    while(ros::ok())
    {
        cv::Mat image; darknet_ros_msgs::BoundingBoxes frame_bboxes;
        std_msgs::Header image_header, frame_bboxes_header;
        double image_time = 0, frame_bboxes_time = 0;
        m_buf.lock();
        if(!img_buf.empty() && !frame_bboxes_buf.empty())
        {
            vo->checkImageAndBboxesAligned(img_buf, frame_bboxes_buf.front()));


            image_time = img_buf.front().header.stamp.toSec();
            image_header = img_buf.front().header;
            image = getImageFromMsg(img_buf.front());
            img_buf.pop();
            if(!image.empty())
                estimator.inputImage(image_time, image);
            frame_bboxes_time = frame_bboxes_buf.front().header.stamp.toSec();
            frame_bboxes_header = frame_bboxes_buf.front().header;
            frame_bboxes = frame_bboxes_buf.front();
            frame_bboxes_buf.pop();
            if(!frame_bboxes_buf.front().bounding_boxes.empty())
                estimator.inputFrameBboxes(frame_bboxes_time, frame_bboxes);
        }
        m_buf.unlock();
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

int main(int argc, char** argv)
{

    //TODO: 写成单独node形式， 最好不在vins-fusion代码中写，独立出来

    ros::init(argc, argv, "cube_vo");
    ROS_INFO("cube_vo");
    ros::NodeHandle nh("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

//    Landmark landmark;


//    DataManager dataManager;
//
//    // TODO all topics used
//    // [A] camera pose
//    string camera_pose_topic = string("/vins_estimator/camera_pose");// '/camera_pose'
//    ROS_INFO("[VO] Subscribe to camera_pose_topic: %s", camera_pose_topic.c_str());
//    ros::Subscriber sub_camera_pose = nh.subscribe(camera_pose_topic, 1000, &DataManager::camera_pose_callback, &dataManager );
//
//
//    // Raw left and right images
//    string left_image_topic = string("/left_image_topic");// 'image_track'
//    ROS_INFO("[VO] Subscribe to left_image_topic: %s", left_image_topic.c_str());
//    ros::Subscriber sub_left_image = nh.subscribe(left_image_topic, 1000, &DataManager::leftImage_callback, &dataManager);
//    string right_image_topic = string("/right_image_topic");
//    ROS_INFO("[VO] Subscribe to right_image_topic: %s", right_image_topic.c_str());
//    ros::Subscriber sub_right_image = nh.subscribe(right_image_topic, 1000, &DataManager::rightImage_callback, &dataManager);
//
//    // Keyframe image
//    string keyframe_image_topic = string("/keyframe_image_topic");
//    ROS_INFO("[VO] Subscribe to keyframe_image_topic: %s", keyframe_image_topic.c_str());
//    ros::Subscriber sub_keyframe_image = nh.subscribe(keyframe_image_topic, 1000, &DataManager::keyframe_image_callback, &dataManager);
//
//    // keyframe camera pose
//    string keyframe_pose_topic = string("/keyframe_pose");
//    ROS_INFO("[VO] Subscribe to keyframe_pose_topic: %s", keyframe_pose_topic.c_str());
//    ros::Subscriber sub_keyframe_pose = nh.subscribe(keyframe_pose_topic, 1000, &DataManager::keyframe_pose_callback, &dataManager);
//
//    // Bboxes of keyframe image from darknet_ros yolov3
//    string keyframe_bboxes_topic = string("/keyframe_bboxes_topic");
//    ROS_INFO("[VO] Subscribe to keyframe_bboxes_topic: %s", keyframe_bboxes_topic.c_str());
//    ros::Subscriber sub_keyframe_bboxes = nh.subscribe(keyframe_bboxes_topic, 1000, &DataManager::keyframe_bboxes_callback, &dataManager);
//
//    // Bboxes of frame image from darknet_ros yolov3
//    string frame_bboxes_topic = string("/frame_bboxes_topic");
//    ROS_INFO("[VO] Subscribe to frame_bboxes_topic: %s", frame_bboxes_topic.c_str());
//    ros::Subscriber sub_frame_bboxes = nh.subscribe(frame_bboxes_topic, 1000, &DataManager::frame_bboxes_callback, &dataManager);
//
//    //------------------------------- sub topics Finished ------------------------------------//
//    // TODO pub all **CUBES** including keyframes and frames image
//    string pub_cube_makers = "/cube_makers";
//    ROS_INFO("main: Publisher pub_cube_makers: %s", pub_cube_makers.c_str());
//    ros::Publisher cube_makers_pub = nh.advertise<visualization_msgs::MarkerArray>(pub_cube_makers, 1000);


    // TODO: sub的img and keyframe_camera_pose放在DataManager中 然后放到Frame类中做 Detect Cube
    // TODO: input one-bye-one image to tracking()

    // TODO detect image
//    ros::Publisher pubLeftImage = nh.advertise<sensor_msgs::Image>("/leftImage", 1000);
//    ros::Publisher pubRightImage = nh.advertise<sensor_msgs::Image>("/rightImage", 1000);
//
//    if(argc != 3){
//        printf("please intput: rosrun vins kitti_odom_test [config file] [data folder] \n");
//        return 1;
//    }
//
//    string config_file = argv[1];
//    printf("config_file: %s\n", argv[1]);
//    string sequence = argv[2];
//    printf("read sequence: %s\n", argv[2]);
//    string dataPath = sequence + "/";

     //Bboxes of frame image from darknet_ros yolov3
    string frame_bboxes_topic = string("/darknet_ros/bounding_boxes");
    ROS_INFO("[VO] Subscribe to frame_bboxes_topic: %s", frame_bboxes_topic.c_str());
    ros::Subscriber sub_frame_bboxes = nh.subscribe(frame_bboxes_topic, 1000, frame_bboxes_callback);

//    string detection_image_topic = string("/darknet_ros/detection_image");
//    ROS_INFO("[VO] Subscribe to detection_image_topic: %s", detection_image_topic.c_str());
//    ros::Subscriber sub_detection_image = nh.subscribe(detection_image_topic, 1000, &feature_tracker::detection_image_callback, &tracking);

    string left_image_topic = string("/leftRGBImage");
    ROS_INFO("[VO] Subscribe to left_image_topic: %s", left_image_topic.c_str());
    ros::Subscriber sub_left_image = nh.subscribe(left_image_topic, 1000, left_image_callback);

    //// synchronize image and bboxes time
    message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/leftRGBImage", 1000);
    message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> frame_bboxes_sub(nh, "/darknet_ros/bounding_boxes", 1000);
    message_filters::TimeSynchronizer<sensor_msgs::Image, darknet_ros_msgs::BoundingBoxes> sync(image_sub, frame_bboxes_sub, 10);
    sync.registerCallback(boost::bind(&image_frame_bboxes_callback, _1, _2));


    std::thread sync_thread{sync_process};

//    tracking.startingDetectCuboid();

#ifdef DEBUG
    printf("[VO] Subscriber Finished!\n");
#endif

    ros::spin();

    return 0;
}
