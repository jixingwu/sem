//
// Created by jixingwu on 2020/6/18.
//

#ifndef SRC_DATAMANAGER_H
#define SRC_DATAMANAGER_H

#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <Eigen/Core>

#include <queue>
#include <mutex>
#include <thread>
#include <Eigen/Core>
#include <utility>

#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <cv_bridge/cv_bridge.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "Object_landmark.h"
//#include "Frame.h"
#include "g2o_Object.h"
#include "utils/TermColor.h"
#include "DataNode.h"

using namespace std;
using namespace Eigen;

typedef std::map<ros::Time, DataNode*> t__DataNode;

class DataManager
{
public:
//    DataManager(ros::NodeHandle &nh);
    DataManager();

    bool isPose0Avaliable() const {return pose_0_available;}

private:
    ros::NodeHandle nh;
    bool pose_0_available = false;
    ros::Time pose_0;// time of 1st pose

public:
    //------------------------ callback -------------------------//
    void camera_pose_callback(const nav_msgs::Odometry &msg);
    void leftImage_callback(const sensor_msgs::ImageConstPtr &msg);
    void rightImage_callback(const sensor_msgs::ImageConstPtr &msg);// Not sure whether this will be used
    void keyframe_image_callback(const sensor_msgs::ImageConstPtr &msg);
    void keyframe_pose_callback(const nav_msgs::Odometry &msg);
    void keyframe_bboxes_callback(const darknet_ros_msgs::BoundingBoxes &msg);
    void frame_bboxes_callback(const darknet_ros_msgs::BoundingBoxes &msg);

public:
    // ---------------------- publisher ------------------------//
    bool isCubeMakersPublisherPubSet() const;
    void setCubeMakerPublisher(ros::Publisher& pub);
    ros::Publisher cube_makers_pub;

private:
    bool is_cube_makers_set = false;

private:
    queue<nav_msgs::Odometry> pose_buf;
    queue<nav_msgs::Odometry> keypose_buf;
    queue<darknet_ros_msgs::BoundingBoxes> keyframe_bboxes_buf;
    queue<darknet_ros_msgs::BoundingBoxes> frame_bboxes_buf;
    queue<sensor_msgs::ImageConstPtr> img_buf;
    queue<sensor_msgs::ImageConstPtr> keyimg_buf;
    std::mutex m_buf;

private:
    std::shared_ptr<t__DataNode> data_map = std::make_shared<t__DataNode>();

public:
    std::shared_ptr<t__DataNode> getDataMapRef(){ return data_map; }

public:
    queue<nav_msgs::Odometry> getCameraPose(){ return pose_buf; }
    queue<nav_msgs::Odometry> getKeyframePose(){ return keypose_buf; }
    queue<darknet_ros_msgs::BoundingBoxes> getKeyframeBboxes(){return keyframe_bboxes_buf;}
    queue<darknet_ros_msgs::BoundingBoxes> getFrameBboxes(){return frame_bboxes_buf;}
    queue<sensor_msgs::ImageConstPtr> getFrameImage(){ return img_buf; }
    queue<sensor_msgs::ImageConstPtr> getKeyframeImage(){ return keyimg_buf; }


};
#endif //SRC_DATAMANAGER_H
