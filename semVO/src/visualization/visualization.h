//
// Created by jixingwu on 2020/8/28.
//

#ifndef SRC_VISUALIZATION_H
#define SRC_VISUALIZATION_H

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <fstream>

#include "src/visual_odometry/visual_odometry.h"
#include "src/visual_odometry/mapcube.h"


void registerPub(ros::NodeHandle &n);

void pubTrackImage(const cv::Mat &imgTrack, double t);

void pubOdometry(const VisualOdometry &vo, const std_msgs::Header &header);

void pubCuboids(MapCube *mapCube, Vector3d rgbcolor, double t);

void outputCuboids(MapCube *mapCube);





#endif //SRC_VISUALIZATION_H
