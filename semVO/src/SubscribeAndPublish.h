//
// Created by jixingwu on 2020/3/8.
//

#ifndef SRC_SUBSCRIBEANDPUBLISH_H
#define SRC_SUBSCRIBEANDPUBLISH_H



#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <Eigen/Core>
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

class SubscribeAndPublish
{
private:
    ros::NodeHandle n;
//    ros::Rate loop_rate(5);
    ros::Subscriber sub_image_track;
    ros::Subscriber sub_keyframe_image;
    ros::Subscriber sub_keyframe_bboxes;
    ros::Subscriber sub_frame_bboxes;
    ros::Subscriber sub_camtransToGround;

    ros::Publisher pub_frame_raw_cube;

public:

    SubscribeAndPublish()
    {
        sub_image_track = n.subscribe<sensor_msgs::Image>("/leftImage", 100, &SubscribeAndPublish::leftImage_callback, this);
        sub_keyframe_image = n.subscribe<sensor_msgs::Image>("/keyframe_image", 100, &SubscribeAndPublish::keyframe_image_callback, this);
        sub_keyframe_bboxes = n.subscribe("/keyframe_bboxes", 100, &SubscribeAndPublish::keyframe_bboxes_callback, this);
        sub_frame_bboxes = n.subscribe("/frame_bboxes", 100, &SubscribeAndPublish::frame_bboxes_callback, this);
        sub_camtransToGround = n.subscribe("/cam_trans_to_ground",100, &SubscribeAndPublish::camToGround_callback, this );
        pub_frame_raw_cube = n.advertise<visualization_msgs::MarkerArray>("/cube_raw_image", 10);
    }
    ~SubscribeAndPublish(){}

    void leftImage_callback(const sensor_msgs::ImageConstPtr &img_msg);
    void keyframe_image_callback(const sensor_msgs::ImageConstPtr &img_msg);
    void keyframe_bboxes_callback(const darknet_ros_msgs::BoundingBoxes &keyframe_bboxes);
    void frame_bboxes_callback(const darknet_ros_msgs::BoundingBoxes &frame_bboxes);
    void camToGround_callback(const nav_msgs::Odometry &msg);

//    void publish_all_poses(std::vector<object_landmark*> cube_landmark_history);
    void publish_image(string dataPath);



};

#endif //SRC_SUBSCRIBEANDPUBLISH_H
