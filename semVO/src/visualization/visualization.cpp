//
// Created by jixingwu on 2020/8/28.
//

#include "visualization.h"
#include "visualization_msgs/MarkerArray.h"

ros::Publisher pub_odometry;
ros::Publisher pub_path;
ros::Publisher pub_camera_pose;
ros::Publisher pub_image_track, pub_2d_cuboid_image;
ros::Publisher pub_cube, pub_cube_array;

nav_msgs::Path path;

void registerPub(ros::NodeHandle &n)
{
    ROS_DEBUG("Finshed pub register!");
    pub_path = n.advertise<nav_msgs::Path>("/path", 1000);
    pub_odometry = n.advertise<nav_msgs::Odometry>("/odometry", 1000);
    pub_camera_pose = n.advertise<nav_msgs::Odometry>("/camera_pose", 1000);
    pub_image_track = n.advertise<sensor_msgs::Image>("/image_track", 1000);
    pub_cube = n.advertise<visualization_msgs::Marker>("/cubes_raw",1000);
    pub_2d_cuboid_image = n.advertise<sensor_msgs::Image>("/cubes_project_image", 1000);
    pub_cube_array = n.advertise<visualization_msgs::MarkerArray>("/cube_raw_array", 1000);
}

void pubCuboidsArray()
{

}

void outputCuboids(MapCube *mapCube)
{
    ROS_DEBUG("cube scale: %f, %f, %f",
              mapCube->scale_.x(), mapCube->scale_.y(), mapCube->scale_.z());
    ROS_DEBUG("cube pose.trans: %f, %f, %f",
              mapCube->pose_.translation().x(), mapCube->pose_.translation().y(), mapCube->pose_.translation().z());
    ROS_DEBUG("cube pose.quate: %f, %f, %f, %f",
              mapCube->pose_.unit_quaternion().x(), mapCube->pose_.unit_quaternion().y(),
              mapCube->pose_.unit_quaternion().z(), mapCube->pose_.unit_quaternion().z());

}




void pubCuboids( MapCube *mapCube, Vector3d rgbcolor, double t)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time(t);

    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = mapCube->pose_.translation().x();
    marker.pose.position.y = mapCube->pose_.translation().y();
    marker.pose.position.z = mapCube->pose_.translation().z();
    marker.pose.orientation.x = mapCube->pose_.unit_quaternion().x();
    marker.pose.orientation.y = mapCube->pose_.unit_quaternion().y();
    marker.pose.orientation.z = mapCube->pose_.unit_quaternion().z();
    marker.pose.orientation.w = mapCube->pose_.unit_quaternion().w();

    marker.scale.x = mapCube->scale_.x();
    marker.scale.y = mapCube->scale_.y();
    marker.scale.z = mapCube->scale_.z();
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;


    ROS_DEBUG_NAMED("pubCuboids", "%d: begin to pub cuboids!", mapCube->id_);
    pub_cube.publish(marker);


}

void pubTrackImage(const cv::Mat& img, double t)
{
    std_msgs::Header header;
    header.frame_id = "world";
    header.stamp = ros::Time(t);
    sensor_msgs::ImagePtr imgTrackMsg = cv_bridge::CvImage(header, "bgr8", img).toImageMsg();
    pub_image_track.publish(imgTrackMsg);

}

