
/*******************************************************
 * please first launch KITTIOdomTest.cpp in VINS_Fusion of vins_estimator
 *
 *******************************************************/

#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include "landmark.h"
#include "Tracking.h"
#include "DataManager.h"
#include "Frame.h"


#define DEBUG

using namespace std;
using namespace Eigen;

//Estimator estimator;


int main(int argc, char** argv)
{

    //TODO: 写成单独node形式， 最好不在vins-fusion代码中写，独立出来

    ros::init(argc, argv, "cube_vo");
    ROS_INFO("cube_vo");
    ros::NodeHandle nh("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    Landmark landmark;
    Tracking tracking;

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

    if(argc != 2){
        printf("please intput: rosrun vins kitti_odom_test [data folder] \n");
        return 1;
    }
    string sequence = argv[1];
    printf("read sequence: %s\n", argv[1]);
    string dataPath = sequence + "/";

    // load image list
    FILE* file;
    file = std::fopen((dataPath + "times.txt").c_str(), "r");
    if(file == NULL)
    {
        printf("cannot find file: %stimes.txt\n", dataPath.c_str());
        ROS_BREAK();
        return 0;
    }
    double imageTime;
    vector<double> imageTimeList;
    while ( fscanf(file, "%lf", &imageTime) != EOF)
    {
        imageTimeList.push_back(imageTime);
    }
    std::fclose(file);

    string leftImagePath, rightImagePath;
    cv::Mat imLeft, imRight;

    for (std::size_t i = 0; i < imageTimeList.size(); ++i) {
        if(ros::ok())
        {
            printf("\n process image %d\n", (int)i);
            stringstream ss;
            ss << setfill('0') << setw(6) << i;
            leftImagePath = dataPath + "image_0/" + ss.str() + ".png";

            imLeft = cv::imread(leftImagePath, CV_LOAD_IMAGE_GRAYSCALE);

            tracking.DetectCuboid(imLeft);
        }
        else
            break;
    }




#ifdef DEBUG
    printf("[VO] Subscriber Finished!\n");
#endif

    return 0;
}
