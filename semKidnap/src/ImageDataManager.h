//
// Created by jixingwu on 2020/5/28.
//

#ifndef SRC_IMAGEDATAMANAGER_H
#define SRC_IMAGEDATAMANAGER_H

// This class now stroes the image data and useless iamges are stored to
// file to conserve ram.
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <utility> //std::pair
#include <fstream>

// threading
#include <thread>
#include <mutex>
#include <atomic>

//opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>

#include <ros/ros.h>

using namespace std;

class ImageDataManager
{
public:
    ImageDataManager();
    //bool initStashDir();
    ~ImageDataManager();

    bool setImage(const string ns, const ros::Time t, const cv::Mat img);
    bool setNewImageFromMsg(const string ns, const sensor_msgs::ImageConstPtr msg);
    bool getImage(const string ns, const ros::Time t, cv::Mat& outImg);

    bool rmImage(const string ns, const ros::Time t);
    bool stashImage(const string ns, const ros::Time t);

    bool isImageRetrivable(const string ns, const ros::Time t) const;

    bool print_status(string fname) const;
    bool print_status() const;

};

#endif //SRC_IMAGEDATAMANAGER_H
