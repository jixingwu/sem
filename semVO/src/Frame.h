//
// Created by jixingwu on 2020/4/1.
//

#ifndef SRC_FRAME_H
#define SRC_FRAME_H
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "GraphMatching.h"

#include "detect_3d_cuboid/matrix_utils.h"
#include "detect_3d_cuboid/detect_3d_cuboid.h"

#include "line_lbd/line_lbd_allclass.h"
using namespace std;
using namespace cv;

class Frame
{
public:

    //相机内参, KITTI
    Mat K = ( Mat_<double> (3,3) <<
            529.5000, 0, 365.0000,
            0, 529.5000, 265.0000,
            0, 0, 1.0000);
    detect_3d_cuboid detect_cuboid_obj;
    line_lbd_detect line_lbd_obj;

public:

    Frame()= default;
    ~Frame()= default;
    //tra
    void triangulation (
            const vector<Point2f>& points1,
            const vector<Point2f>& points2,
            const cv::Mat& R, const cv::Mat& t,
            vector<cv::Point3d>& points
            );//Output: points;
    void pose_estimation_2d2d(
            const vector<Point2f>& points1,
            const vector<Point2f>& points2,
            Mat& R, Mat& t
            );//Output: R, t;
    Point2f pixel2cam(const Point2d& p, const Mat& K);

    void setCubParameter();



};

//class BboxMatching : public GraphMatching
//{
//
//};

#endif //SRC_FRAME_H
