//
// Created by jixingwu on 2020/8/28.
//

#include "visualization.h"

ros::Publisher pub_odometry;
ros::Publisher pub_path;
ros::Publisher pub_camera_pose;
ros::Publisher pub_image_track, pub_2d_cuboid_image;
ros::Publisher pub_cube;
nav_msgs::Path path;

void registerPub(ros::NodeHandle &n)
{
    pub_path = n.advertise<nav_msgs::Path>("path", 1000);
    pub_odometry = n.advertise<nav_msgs::Odometry>("odometry", 1000);
    pub_camera_pose = n.advertise<nav_msgs::Odometry>("camera_pose", 1000);
    pub_image_track = n.advertise<sensor_msgs::Image>("image_track", 1000);
    pub_cube = n.advertise<visualization_msgs::MarkerArray>("cubes_raw_frame",1000);
    pub_2d_cuboid_image = n.advertise<sensor_msgs::Image>("/cubes_project_image", 1000);
}