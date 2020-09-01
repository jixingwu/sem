
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
#include <ros/console.h>// import ROS_DEBUG(), ROS_DEBUG_ONCE() etc.
// sync time
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <boost/bind.hpp>

//#include "sophus/se3.h"
//#include <sophus/se3.h>
#include <sophus/se3.h>
#include <sophus/so3.h>

#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>

#include "visual_odometry/frame.h"
#include "visual_odometry/visual_odometry.h"
#include "visualization/config.h"
#include "visual_odometry/TermColor.h"
#include "visualization/visualization.h"

#define DEBUG

using namespace std;
using namespace Eigen;

queue<darknet_ros_msgs::BoundingBoxesConstPtr> frame_bboxes_buf;
queue<sensor_msgs::ImageConstPtr> img0_buf;
queue<nav_msgs::OdometryConstPtr> pose_buf;
std::mutex m_buf;

#define __DEBUG__(msg) msg;

cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &msg);

void img0_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    img0_buf.push(img_msg);
    m_buf.unlock();
}

void frame_bboxes_callback(const darknet_ros_msgs::BoundingBoxesConstPtr & bboxes_msg)
{
    m_buf.lock();
    frame_bboxes_buf.push(bboxes_msg);
    m_buf.unlock();
}

void pose_callback(const nav_msgs::OdometryConstPtr &pose_msg)
{
    m_buf.lock();
    pose_buf.push(pose_msg);
    m_buf.unlock();
}


cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &msg){
    cv_bridge::CvImagePtr cv_ptr;
    if(msg->encoding == "8UC1"){
        sensor_msgs::Image img;
        img.header = msg->header;
        img.height = msg->height;
        img.width = msg->width;
        img.is_bigendian = msg->is_bigendian;
        img.step = msg->step;
        img.data = msg->data;
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
    VisualOdometry::Ptr vo(new VisualOdometry);
    Camera::Ptr camera(new Camera);

    while(ros::ok())
    {
        cv::Mat image;
        darknet_ros_msgs::BoundingBoxesConstPtr frame_bboxes;
        nav_msgs::OdometryConstPtr pose; Eigen::Quaterniond q, init_q; Eigen::Vector3d t, init_t;
        std_msgs::Header image_header, frame_bboxes_header;
        double image_time = 0, frame_bboxes_time = 0;
        m_buf.lock();

        if(!img0_buf.empty() && !frame_bboxes_buf.empty() && !pose_buf.empty())
        {
            ROS_DEBUG_ONCE("------ received console msgs-------");
            if (img0_buf.front()->header.stamp < pose_buf.front()->header.stamp)
            {
                ROS_WARN("this image is not inited!! ");
                img0_buf.pop();
            }
            else if(img0_buf.front()->header.stamp == pose_buf.front()->header.stamp)
            {
                ROS_INFO("image stamp is equal to pose!!!");
                if(img0_buf.front()->header.stamp == frame_bboxes_buf.front()->image_header.stamp)
                {
                    ROS_INFO("image stamp is equal to bboxes!!!");
                    image_time = img0_buf.front()->header.stamp.toSec();
                    image_header = img0_buf.front()->header;
                    image = getImageFromMsg(img0_buf.front());
                    img0_buf.pop();

                    frame_bboxes_time = frame_bboxes_buf.front()->header.stamp.toSec();
                    frame_bboxes_header = frame_bboxes_buf.front()->header;
                    frame_bboxes = frame_bboxes_buf.front();
                    frame_bboxes_buf.pop();

                    pose = pose_buf.front();
                    if(!pose->header.frame_id.empty())
                    {
                        t.x() = pose->pose.pose.position.x; t.y() = pose->pose.pose.position.y; t.z() = pose->pose.pose.position.z;
                        q.x() = pose->pose.pose.orientation.x; q.y() = pose->pose.pose.orientation.y; q.z() = pose->pose.pose.orientation.z; q.w() = pose->pose.pose.orientation.w;
                        pose_buf.pop();
                    }

                    Frame::Ptr pFrame = Frame::createFrame();
                    pFrame->time_stamp_ = image_time;
                    pFrame->rgb_image_ = image;
                    pFrame->bboxes_ = frame_bboxes;
                    pFrame->T_c_w_ = Sophus::SE3(q,t);
                    __DEBUG__( ROS_INFO("curr frame id is %ld", pFrame->id_);)
                    vo->addFrame(pFrame);

                }
                else
                {
                    ROS_WARN("bboxes aliasing!! pop bboxes");
                    frame_bboxes_buf.pop();
//                    continue;
                }
            } else{
                ROS_ERROR("not initialization!!");
                ros::shutdown();
            }

//            cout<<TermColor::iBLUE()<<img0_buf.size()<<"\t"<<frame_bboxes_buf.size()<<"\t"<<pose_buf.size()<<TermColor::RESET()<<endl;
//            cout<<TermColor::iRED()<<"img time: "<< img0_buf.front()->header.stamp<<endl
//            <<"bboxes time: "<< frame_bboxes_buf.front()->image_header.stamp<<endl
//            <<"pose time: "<< pose_buf.front()->header.stamp<<TermColor::RESET()<<endl;
//            img0_buf.pop(); frame_bboxes_buf.pop(); pose_buf.pop();
        }
        else{
            __DEBUG__(ROS_DEBUG_ONCE("------- not all received -----");
                    cout<<img0_buf.size()<<"\t"<<frame_bboxes_buf.size()<<"\t"<<pose_buf.size()<<endl;)
        }
        m_buf.unlock();
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
    return;
}

int main(int argc, char** argv)
{
    //TODO: 写成单独node形式， 最好不在vins-fusion代码中写，独立出来

    ros::init(argc, argv, "cube_vo");
    ROS_INFO("cube_vo");
    ros::NodeHandle nh("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    if(argc > 0)
        Config::setParameterFile(argv[1]);
    else
        return 1;

    registerPub(nh);

    //// synchronize image and bboxes time
    string left_image_topic = string("/leftImageRGB");
    ROS_INFO("[VO] Subscribe to left_image_topic: %s", left_image_topic.c_str());
    ros::Subscriber sub_left_image = nh.subscribe(left_image_topic, 100, img0_callback);

    string frame_bboxes_topic = string("/darknet_ros/bounding_boxes");
    ROS_INFO("[VO] Subscribe to frame_bboxes_topic: %s", frame_bboxes_topic.c_str());
//    message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> frame_bboxes_sub(nh, frame_bboxes_topic, 100);
    ros::Subscriber sub_frame_bboxes = nh.subscribe(frame_bboxes_topic, 100, frame_bboxes_callback);

    string camera_pose_topic = string("/vins_estimator/camera_pose");
    ROS_INFO("[VO] Subscribe to camera_pose_topic: %s", camera_pose_topic.c_str());
//    message_filters::Subscriber<nav_msgs::Odometry> camera_pose_sub(nh, camera_pose_topic, 100);
    ros::Subscriber sub_pose = nh.subscribe(camera_pose_topic, 100, pose_callback);

//    string detection_image_topic = string("/darknet_ros/detection_image");
//    ROS_INFO("[VO] Subscribe to detection_image_topic: %s", detection_image_topic.c_str());
//    ros::Subscriber sub_detec = nh.subscribe(detection_image_topic, 100, detec_callback);

    std::thread sync_thread{sync_process};
#ifdef DEBUG
    printf("[VO] Subscriber Finished!\n");
#endif
    ros::spin();

    return 0;
}
