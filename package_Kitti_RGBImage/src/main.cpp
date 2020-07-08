//
// Created by jixingwu on 2020/7/8.
// package kitti color image to ros bag
//

#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

using namespace std;

int main(int argc, char** argv){
    ros::init(argc, argv, "package_RGB");
    ros::NodeHandle nh("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    ros::Publisher pubLeftImage = nh.advertise<sensor_msgs::Image>("/leftRGBImage", 1000);
    ros::Publisher pubRightImage = nh.advertise<sensor_msgs::Image>("/rightRGBImage", 1000);

    if(argc != 2){
        printf("please intput: rosrun package_RGB package_test [data folder] \n"
               "for example: rosrun package_RGB package_test "
               "/media/tony-ws1/disk_D/kitti/odometry/sequences/00 \n");
        return 1;
    }

    string sequence = argv[1];
    printf("read sequence: %s\n", argv[1]);
    string dataPath = sequence + "/";

    //load image list
    FILE* file;
    file = std::fopen((dataPath + "times.txt").c_str(), "r");
    if (file == NULL){
        printf("cannot fild file: %stimes.txt\n", dataPath.c_str());
        ROS_BREAK();
        return 0;
    }

    double imageTime;
    vector<double> imageTimeList;
    while (fscanf(file, "%lf", &imageTime) != EOF){
        imageTimeList.push_back(imageTime);
    }
    std::fclose(file);

    string leftImagePath, rightImagePath;
    cv::Mat imLeft, imRight;
    cv::Mat imLeftdst, imRightdst;

    cv_bridge::CvImage left_msg;
    cv_bridge::CvImage right_msg;

    for (std::size_t i = 0; i < imageTimeList.size(); ++i) {
        if(ros::ok()){
            printf("\nprocess image: %d\n", (int)i);
            stringstream ss;
            ss<<setfill('0') << setw(6) << i;
            leftImagePath = dataPath + "image_2/" + ss.str() + ".png";
            rightImagePath = dataPath + "image_3/" + ss.str() + ".png";

//            printf("%lu  %f \n", i, imageTimeList[i]);
//            printf("%s\n", leftImagePath.c_str() );
//            printf("%s\n", rightImagePath.c_str() );

//            imLeft = cv::imread(leftImagePath, CV_LOAD_IMAGE_COLOR);
            imLeft = cv::imread(leftImagePath, 1);
//            sensor_msgs::ImagePtr imLeftMsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", imLeft).toImageMsg();
//            imLeftMsg->header.stamp = ros::Time(imageTimeList[i]);
//            pubLeftImage.publish(imLeftMsg);
            left_msg.header.stamp = ros::Time(imageTimeList[i]);
            left_msg.encoding = sensor_msgs::image_encodings::BGR8;
            left_msg.image = imLeft;
            pubLeftImage.publish(left_msg.toImageMsg());

//            imRight = cv::imread(rightImagePath, CV_LOAD_IMAGE_COLOR);
            imRight = cv::imread(rightImagePath, 1);
//            sensor_msgs::ImagePtr imRightMsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", imRight).toImageMsg();
//            imRightMsg->header.stamp = ros::Time(imageTimeList[i]);
//            pubRightImage.publish(imRightMsg);
            right_msg.header.stamp = ros::Time(imageTimeList[i]);
            right_msg.encoding = sensor_msgs::image_encodings::BGR8;
            right_msg.image = imRight;
            pubRightImage.publish(right_msg.toImageMsg());

//            cv::pyrDown(imLeft, imLeftdst, cv::Size)

            cv::imshow("leftImage", imLeft);
            cv::imshow("rightImage", imRight);
            cv::waitKey(2);
        }
        else
            break;
    }
    return 0;
}
