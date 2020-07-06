//
// Created by jixingwu on 2020/6/5.
//

#ifndef SRC_DATASEMNODE_H
#define SRC_DATASEMNODE_H
#include <iostream>
#include <thread>
#include <mutex>
#include <atomic>


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
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"

// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace Eigen;
#include <opencv2/core/eigen.hpp>

#include "utils/PoseManipUtils.h"
#include "utils/MiscUtils.h"
#include "utils/TermColor.h"

class DataSemNode
{
public:
    DataSemNode(ros::Time stamp): stamp(stamp)
    {
        is_key_frame = false;
        m_wTc = false;
    }
    void setPose(const Matrix4d __wTc);
    void setPose(const ros::Time __t, const Matrix4d __wTc);
    void setCubeFromMsg(const visualization_msgs::MarkerArrayPtr msg);
    void setCubePoseFromMsg(const nav_msgs::OdometryConstPtr msg);

private:
    const ros::Time stamp;
    mutable std::mutex m;
    std::atomic<bool> is_key_frame;

    Matrix4d wTc;
    MatrixXd wTc_covariance; //6*6
    ros::Time t_wTc;
    std::atomic<bool> m_wTc;
};
#endif //SRC_DATASEMNODE_H
