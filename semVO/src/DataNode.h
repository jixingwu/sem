//
// Created by jixingwu on 2020/6/18.
//

#ifndef SRC_DATANODE_H
#define SRC_DATANODE_H
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

using namespace Eigen;
using namespace std;
class DataNode
{
public:
    DataNode(ros::Time stamp) : stamp(stamp)
    {
        is_key_frame = false;
        m_wTc = false;
    }

    void setLeftImageFromMsg(const sensor_msgs::Image &msg);
    void setKeyframeImageFromMsg(const sensor_msgs::Image &msg);
    void setKeyframeBboxes(const darknet_ros_msgs::BoundingBoxes &msg);
    void setFrameBboxes(const darknet_ros_msgs::BoundingBoxes &msg);

    bool isLeftImageAvailable() const { return m_img; }
    bool isKeyframeImageAvailable() const { return m_keyImg; }
    bool isFrameBboxesAvailable() const { return m_frameBboxes; }
    bool isKeyframeBboxesAvailable() const { return m_keyframeBboxes; }

    const cv::Mat& getLeftImage() const;
    const cv::Mat& getKeyframeImage() const;
    const darknet_ros_msgs::BoundingBoxes getframeBboxes() const;
    const darknet_ros_msgs::BoundingBoxes getKeyframeBboxes() const;



private:
    std::atomic<bool> is_key_frame;
    bool m_wTc;

    const ros::Time stamp;
    mutable std::mutex m;

    cv::Mat img;
    ros::Time t_img;
    std::atomic<bool> m_img;

    cv::Mat keyImg;
    ros::Time t_keyImg;
    std::atomic<bool> m_keyImg;

    darknet_ros_msgs::BoundingBoxes frameBboxes;
    ros::Time t_frameBboxes;
    std::atomic<bool> m_frameBboxes;

    darknet_ros_msgs::BoundingBoxes keyframeBboxes;
    ros::Time t_keyframeBboxes;
    std::atomic<bool> m_keyframeBboxes;

};


#endif //SRC_DATANODE_H
