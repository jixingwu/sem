
/*******************************************************
 * please first launch KITTIOdomTest.cpp in VINS_Fusion of vins_estimator
 *
 *******************************************************/

#include <iostream>
#include <queue>
#include <thread>
#include <mutex>
#include <cmath>
#include <string>
#include <ros/ros.h>
// sync time
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <boost/bind.hpp>

#include "sophus/se3.h"

#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include "landmark.h"
#include "feature_tracker.h"
#include "DataManager.h"
#include "Frame.h"
#include "estimator.h"
#include "visual_odometry.h"
#include "config.h"

#define DEBUG

using namespace std;
using namespace Eigen;


//Estimator estimator;
VisualOdometry::Ptr vo(new VisualOdometry);
Camera::Ptr camera(new Camera);

queue<darknet_ros_msgs::BoundingBoxes> frame_bboxes_buf;
queue<sensor_msgs::Image> img_buf;
queue<nav_msgs::Odometry> pose_buf;
std::mutex m_buf;

#define __DEBUG__(msg) msg;
void image_frame_bboxes_pose_callback(const sensor_msgs::Image image, const darknet_ros_msgs::BoundingBoxes frame_bboxes,
        nav_msgs::Odometry pose)
{
    m_buf.lock();
    img_buf.push(image);
    frame_bboxes_buf.push(frame_bboxes);
    pose_buf.push(pose);
    m_buf.unlock();

}
cv::Mat getImageFromMsg(const sensor_msgs::Image &msg){
    cv_bridge::CvImagePtr cv_ptr;
    if(msg.encoding == "8UC1"){
        sensor_msgs::Image img;
        img.header = msg.header;
        img.height = msg.height;
        img.width = msg.width;
        img.is_bigendian = msg.is_bigendian;
        img.step = msg.step;
        img.data = msg.data;
        img.encoding = "BGR8";
        cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    }
    else
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat img = cv_ptr->image.clone();
    return img;
}

void sync_process()
{
    while(ros::ok())
    {
        cv::Mat image;
        darknet_ros_msgs::BoundingBoxes frame_bboxes;
        nav_msgs::Odometry pose; Eigen::Quaterniond q, init_q; Eigen::Vector3d t, init_t;
        std_msgs::Header image_header, frame_bboxes_header;
        double image_time = 0, frame_bboxes_time = 0;
        m_buf.lock();
        if(!img_buf.empty() && !frame_bboxes_buf.empty() && !pose_buf.empty())
        {
            image_time = img_buf.front().header.stamp.toSec();
            image_header = img_buf.front().header;
            image = getImageFromMsg(img_buf.front());
            img_buf.pop();

            frame_bboxes_time = frame_bboxes_buf.front().header.stamp.toSec();
            frame_bboxes_header = frame_bboxes_buf.front().header;
            frame_bboxes = frame_bboxes_buf.front();
            frame_bboxes_buf.pop();

            pose = pose_buf.front();
            if(!pose.header.frame_id.empty())
            {
                t.x() = pose.pose.pose.position.x; t.y() = pose.pose.pose.position.y; t.z() = pose.pose.pose.position.z;
                q.x() = pose.pose.pose.orientation.x; q.y() = pose.pose.pose.orientation.y; q.z() = pose.pose.pose.orientation.z(); q.w() = pose.pose.pose.orientation.w;
                pose_buf.pop();
            }


            Frame::Ptr pFrame = Frame::createFrame();

            pFrame->time_stamp_ = image_time;
            pFrame->rgb_image_ = image;
            pFrame->bboxes_ = frame_bboxes;
            pFrame->T_c_w_ = Sophus::SE3(q,t);

            vo->addFrame(pFrame);
        }
        m_buf.unlock();
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

int main(int argc, char** argv)
{
    //TODO: 写成单独node形式， 最好不在vins-fusion代码中写，独立出来

    ros::init(argc, argv, "cube_vo");
    ROS_INFO("cube_vo");
    ros::NodeHandle nh("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    //// synchronize image and bboxes time
    string left_image_topic = string("/leftRGBImage");
    ROS_INFO("[VO] Subscribe to left_image_topic: %s", left_image_topic.c_str());
    message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, left_image_topic, 1000);

    string frame_bboxes_topic = string("/darknet_ros/bounding_boxes");
    ROS_INFO("[VO] Subscribe to frame_bboxes_topic: %s", frame_bboxes_topic.c_str());
    message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> frame_bboxes_sub(nh, frame_bboxes_topic, 1000);

    string camera_pose_topic = string("/vins/camera_pose");
    ROS_INFO("[VO] Subscribe to camera_pose_topic: %s", camera_pose_topic.c_str());
    message_filters::Subscriber<nav_msgs::Odometry> camera_pose_sub(nh, camera_pose_topic, 1000);

    message_filters::TimeSynchronizer<sensor_msgs::Image, darknet_ros_msgs::BoundingBoxes, nav_msgs::Odometry> sync(
            image_sub, frame_bboxes_sub, camera_pose_sub, 1000);
    sync.registerCallback(boost::bind(&image_frame_bboxes_pose_callback, _1, _2, _3));

    std::thread sync_thread{sync_process};

#ifdef DEBUG
    printf("[VO] Subscriber Finished!\n");
#endif

    ros::spin();

    return 0;
}
