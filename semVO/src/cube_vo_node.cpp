
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
Landmark landmark;
Tracking tracking;

int main(int argc, char** argv)
{

    //TODO: 写成单独node形式， 最好不在vins-fusion代码中写，独立出来

    ros::init(argc, argv, "cube_vo");
    ROS_INFO("cube_vo");
    ros::NodeHandle nh("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    DataManager dataManager;

    // TODO all topics used
    // [A] camera pose
    string camera_pose_topic = string("/vins_estimator/camera_pose");// '/camera_pose'
    ROS_INFO("[VO] Subscribe to camera_pose_topic: %s", camera_pose_topic.c_str());
    ros::Subscriber sub_camera_pose = nh.subscribe(camera_pose_topic, 1000, &DataManager::camera_pose_callback, &dataManager );


    // Raw left and right images
    string left_image_topic = string("/left_image_topic");// 'image_track'
    ROS_INFO("[VO] Subscribe to left_image_topic: %s", left_image_topic.c_str());
    ros::Subscriber sub_left_image = nh.subscribe(left_image_topic, 1000, &DataManager::leftImage_callback, &dataManager);
    string right_image_topic = string("/right_image_topic");
    ROS_INFO("[VO] Subscribe to right_image_topic: %s", right_image_topic.c_str());
    ros::Subscriber sub_right_image = nh.subscribe(right_image_topic, 1000, &DataManager::rightImage_callback, &dataManager);

    // Keyframe image
    string keyframe_image_topic = string("/keyframe_image_topic");
    ROS_INFO("[VO] Subscribe to keyframe_image_topic: %s", keyframe_image_topic.c_str());
    ros::Subscriber sub_keyframe_image = nh.subscribe(keyframe_image_topic, 1000, &DataManager::keyframe_image_callback, &dataManager);

    // keyframe camera pose
    string keyframe_pose_topic = string("/keyframe_pose");
    ROS_INFO("[VO] Subscribe to keyframe_pose_topic: %s", keyframe_pose_topic.c_str());
    ros::Subscriber sub_keyframe_pose = nh.subscribe(keyframe_pose_topic, 1000, &DataManager::keyframe_pose_callback, &dataManager);

    // Bboxes of keyframe image from darknet_ros yolov3
    string keyframe_bboxes_topic = string("/keyframe_bboxes_topic");
    ROS_INFO("[VO] Subscribe to keyframe_bboxes_topic: %s", keyframe_bboxes_topic.c_str());
    ros::Subscriber sub_keyframe_bboxes = nh.subscribe(keyframe_bboxes_topic, 1000, &DataManager::keyframe_bboxes_callback, &dataManager);

    // Bboxes of frame image from darknet_ros yolov3
    string frame_bboxes_topic = string("/frame_bboxes_topic");
    ROS_INFO("[VO] Subscribe to frame_bboxes_topic: %s", frame_bboxes_topic.c_str());
    ros::Subscriber sub_frame_bboxes = nh.subscribe(frame_bboxes_topic, 1000, &DataManager::frame_bboxes_callback, &dataManager);

    //------------------------------- sub topics Finished ------------------------------------//
    // TODO pub all **CUBES** including keyframes and frames image
    string pub_cube_makers = "/cube_makers";
    ROS_INFO("main: Publisher pub_cube_makers: %s", pub_cube_makers.c_str());
    ros::Publisher cube_makers_pub = nh.advertise<visualization_msgs::MarkerArray>(pub_cube_makers, 1000);


    // TODO: sub的img and keyframe_camera_pose放在DataManager中 然后放到Frame类中做 Detect Cube
    // TODO: input one-bye-one image to tracking()


#ifdef DEBUG
    printf("[VO] Subscriber Finished!\n");
#endif

    return 0;
}
