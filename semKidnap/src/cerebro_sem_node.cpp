//
// Created by jixingwu on 2020/5/19.
// This file is used to relocalize for semantic mapping
//
#include <string>
#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Header.h>

#include "DataManager.h"
#include "Cerebro.h"
using namespace std;

int main(int argc, char** argv)
{
    //--- ROS INIT ---//
    ros::init(argc, argv, "cerebro_sem_node");
    ros::NodeHandle nh("~");

    //--- DataManager ---//
    DataManager dataManager = DataManager(nh);
    //[A] input data and parameters
    // TODO: set abstrack cameras;

    Cerebro cer(nh);
    cer.setDataManager(&dataManager);
    cer.setPublishers("/cerebro");


    // TODO: all topics used.
    // [A.00] Camera Pose
    // w_T_c: Pose of camera in world-cordinate system
    string camera_pose_topic = string("/vins_estimator/camera_pose");
    ROS_INFO("Subscribe to camera_pose_topic: %s", camera_pose_topic.c_str());
    ros::Subscriber sub_odometry_sem = nh.subscribe(camera_pose_topic, 1000, &DataManager::camera_pose_callback, &dataManager );


    // [A.01.0] Keyframe Pose
    // keyframe pose. Caution, these poses are of imu (NOT of camera)

    // [A.01.1] Cube and Cube Pose
    // sub semantic mapping topic, which pub Cube Pose and genre is /visualization/MakerArray or /Marker
    string cube_visual_topic = string("/vins_semantics/cube_visual");
    ROS_INFO("Subscribe to cube_visual_topic: %s", cube_visual_topic.c_str());
    ros::Subscriber sub_cube = nh.subscribe(cube_visual_topic, 1000, &DataManager::cube_visual_callback, &dataManager);

    string cube_pose_topic = string("/vins_semantics/cube_pose");
    ROS_INFO("Subscribe to cube_pose_topic: %s", cube_pose_topic.c_str());
    ros::Subscriber sub_cube_pose = nh.subscribe(cube_pose_topic, 1000, &DataManager::cube_pose_callback, &dataManager);

    // [A.02] Raw Image
    // sub all raw KITTI image by topics
    string raw_image_topic = string("/raw_image_topic");
    ROS_INFO("Subscribe to raw_image_topic: %s", raw_image_topic.c_str());
    ros::Subscriber sub_raw_image = nh.subscribe(raw_image_topic, 1000, &DataManager::raw_image_callback, &dataManager);

    // [A.02.1] Additional Image
    // stereo pairs if have

    // [A.02.2] Additional Image
    // depth 16UC1 image

    // [B.0] Camera params
    // Additional Cameras (yaml)
    Eigen::Matrix3d camera_calibs;//calib

    // [B.1] Camera baseline
    // set stereo baseline transform ie. right_T_left from yaml file
    Eigen::MatrixXd camera_extr;// extrinsic_1_T_0

    // [C] imu_T_cam
    // imu_T_cam: imu camera extrinsic calib. Will store this just in case there is a need
    string extrinsic_cam_imu_topic = string("/vins_estimator/extrinsic");
    ROS_INFO("Subscribe to extrinsic_cam_imu_topic: %s", extrinsic_cam_imu_topic.c_str());
    ros::Subscriber sub_extrinsic = nh.subscribe(extrinsic_cam_imu_topic, 1000, &DataManager::extrinsic_cam_imu_callback, &dataManager);

    // [D] PointCloud(all): has 5 channels
    // [E] Tracked features

    // --- Start Thread ---//

    // [A] Data associate thread: looks at the callback buffers and sets the data in the std::map
#define __THREAD_SWITCH_ON__ 1
#if __THREAD_SWITCH_ON__
    dataManager.data_association_thread_enable();
    std::thread data_association_th(&DataManager::data_association_thread, &dataManager, 15);

    dataManager.trial_thread_enable();
    dataManager.trial_thread_disable();
    std::thread dm_trial_th(&DataManager::trial_thread, &dataManager);

    dataManager.clean_up_useless_images_thread_enable();
    std::thread dm_cleanup_th(&DataManager::clean_up_useless_images_thread, &dataManager);

#endif


    //[B.00] Kidnap Message
    // TODO: Kidnap Message Publisher.
    // indicate semantic mapping kidnap in places
    // take a node named "feature_tracker" for tracking semantic cuboids.
    // pub "/rcvd_flag_sem"? and "/rcvd_flag_header_sem"? to recognize kidnap.
    string pub_topic_sem = "/feature_tracker/rcvd_flag_sem";
    ROS_INFO("main: Publisher pub_topic_sem: %s", pub_topic_sem.c_str());
    ros::Publisher rcvd_flag_sem_pub = nh.advertise<std_msgs::Bool>(pub_topic_sem, 1000);// set queue_size
    // we publish std_msgs::Header to the pose-graph-solver
    // serve as carrier of timestamp,
    /// we can eliminate(消除) the odometry edges from the cost function.
    string pub_topic_sem_header = "/feature_tracker/rcvd_flage_sem_header";
    ROS_INFO("main: Publisher pub_topic_sem_header: %s", pub_topic_sem_header.c_str());
    ros::Publisher kidnap_indicator_sem_header_pub = nh.advertise<std_msgs::Header>(pub_topic_sem_header, 1000);
    dataManager.setKidnapIndicatorPublishers(rcvd_flag_sem_pub, kidnap_indicator_sem_header_pub);


    //[B.01]
    // ---松耦合方式---
    // TODO: Kidnap Message Subcriber.
    // receiving and processing the kidnap places to make relocalization.
    // sub two kinds of topics for kidnap message,
    // a. Bool, b. Header. What's the Usage or Functions?
    // be "/rcvd_flag" or "/rcvd_flag_header" to distinguishing features way.
    string kidnap_bool_sem_topic = "/feature_tracker/rcvd_flag_sem";
    ROS_INFO("Subscribe to kidnap_bool_topic: %s", kidnap_bool_sem_topic.c_str());
    ros::Subscriber sud_kidnap_bool_sem = nh.subscribe(kidnap_bool_sem_topic, 1000, &Cerebro::kidnap_bool_sem_callback, &cer);

    string kidnap_header_sem_topic = "/feature_tracker/rcvd_flag_header_sem";
    ROS_INFO("Subscribe to kidnap_header_topic: %s", kidnap_header_sem_topic.c_str());
    ros::Subscriber sub_kidnap_header_sem = nh.subscribe(kidnap_header_sem_topic, 1000, &Cerebro::kidnap_header_sem_callback, &cer);

    //[C.00]
    // loop candidates producer

    //[C.01]
    // loop candidates consumer

    //[D]
    // Kidnap Identification Thread

    // [E]
    // Visualization

    ros::spin();

    // CTRL+C, Stop all Thread!
#define __THREAD_SWTICH_OFF__ 1
#if __THREAD_SWTICH_OFF__
    dataManager.data_association_thread_disable();
    dataManager.trial_thread_disable();
    dataManager.clean_up_useless_images_thread_disable();

    //TODO: disable cer thread and join() wait thread procession;
#endif
}


