//
// Created by jixingwu on 2020/5/21.
//

#ifndef SRC_DATAMANAGER_H
#define SRC_DATAMANAGER_H

#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <queue>
#include <ostream>
#include <iomanip>
#include <map>
#include <iterator>
#include <ctime>


#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <memory> //needed for std::shared_ptr
#include <boost/make_shared.hpp>
//opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>

// ros
#include <ros/ros.h>
#include <ros/package.h>

// ros msg
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/Image.h>
// #include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Bool.h>

#include "DataNode.h"
#include "ImageDataManager.h"
#include "DataSemNode.h"

#include "utils/PoseManipUtils.h"
#include "utils/RawFileIO.h"
#include "utils/SafeQueue.h"
#include "utils/ElapsedTime.h"
#include "utils/TermColor.h"

// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace Eigen;
using namespace std;

//typedef std::map<ros::Time, DataNode* > t__DataNode;
typedef std::map<ros::Time, DataSemNode*> t__DataSemNode;
class DataManager
{
public:
    DataManager(ros::NodeHandle &nh);
    DataManager(const DataManager &obj);

    bool isPose0Avaliable() const { return pose_0_available; }

private:
    bool pose_0_available = false;
    ros::Time pose_0;// time of 1st pose
private:
    ros::NodeHandle nh; // TODO: Not sure whether this will be needed here.
    ros::Publisher rcvd_flag_sem_pub;
    ros::Publisher kidnap_indicator_header_sem_pub;
    bool is_kidnap_indicator_set = false;

public:
    bool isKidnapIndicatorPubSet() const;
    void setKidnapIndicatorPublishers(ros::Publisher& pub_bool, ros::Publisher& pub_header);

public:
    ///////
    /////// Callbacks
    //////
    void camera_pose_callback(const nav_msgs::Odometry::ConstPtr msg);
    void keyframe_pose_callback(const nav_msgs::Odometry::ConstPtr msg);
    void cube_visual_callback(const visualization_msgs::Marker::ConstPtr msg);
    void cube_pose_callback(const nav_msgs::Odometry::ConstPtr msg );

    void raw_image_callback(const sensor_msgs::ImageConstPtr& msg);
    void stereo_image_callback(const sensor_msgs::ImageConstPtr& msg);
    void depth_image_callback(const sensor_msgs::ImageConstPtr& msg);

    void extrinsic_cam_imu_callback(const nav_msgs::Odometry::ConstPtr& msg);
    void ptcld_callback(const sensor_msgs::PointCloud::ConstPtr msg);
    void tracked_feat_callback(const sensor_msgs::PointCloud::ConstPtr msg);

private:
    ros::Time last_image_time = ros::Time();
    // callback-buffers
    std::queue<sensor_msgs::ImageConstPtr> img_buf;
    std::queue<nav_msgs::OdometryConstPtr> pose_buf;
    std::queue<nav_msgs::OdometryConstPtr> cube_pose_buf;
    string print_queue_size(int verbose ) const;


    /////
    ///// Threads
    /////
private:
    std::atomic<bool> b_data_association_thread;
public:
    void data_association_thread(int max_loop_rat_in_hz);
    void data_association_thread_enable(){b_data_association_thread = true;}
    void data_association_thread_disable(){b_data_association_thread = false;}

private:
    atomic<bool> b_trial_thread;
public:
    // Just a trial thread
    void trial_thread();
    void trial_thread_enable() { b_trial_thread = true; }
    void trial_thread_disable() { b_trial_thread = false; }

private:
    atomic<bool> b_clean_up_useless_images_thread;
public:
    // Thread to deallocate images which have no pose info (aka useless images)
    void clean_up_useless_images_thread();
    void clean_up_useless_images_thread_enable() { b_clean_up_useless_images_thread = true; }
    void clean_up_useless_images_thread_disable() { b_clean_up_useless_images_thread = false; }

private:
    std::shared_ptr<ImageDataManager> img_data_mgr = std::make_shared<ImageDataManager>();
    std::shared_ptr<t__DataSemNode> data_map = std::make_shared<t__DataSemNode>();
public:
    void print_datamap_status(string fname) const;
    std::shared_ptr<t__DataSemNode> getDataMapRef(){ return data_map; }
};

#endif //SRC_DATAMANAGER_H
