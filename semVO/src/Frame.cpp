//
// Created by jixingwu on 2020/4/1.
//

#include "Frame.h"

Point2f Frame::pixel2cam(const Point2d& p, const Mat& K)
{
    return Point2f
            (
                    (p.x - K.at<double>(0,2)) / K.at<double>(0,0),
                    (p.y - K.at<double>(1,2))/ K.at<double>(1,1)
                    );
}

void Frame::triangulation(
        const vector<Point2f>& points1,
        const vector<Point2f>& points2,
        const cv::Mat &R, const cv::Mat &t,
        vector<cv::Point3d>& points)
{
    Mat T1 = (Mat_<float>(3,4)<<
            1,0,0,0,
            0,1,0,0,
            0,0,1,0);
    Mat T2 = (Mat_<float> (3,4) <<
            R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), t.at<double>(0,0),
            R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), t.at<double>(1,0),
            R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), t.at<double>(2,0)
    );

    vector<Point2f> pts_1, pts_2;
    for (int i = 0; i < min(points1.size(), points2.size()); ++i) {
        pts_1.push_back(pixel2cam(points1[i], K));
        pts_2.push_back(pixel2cam(points2[i], K));
    }

    Mat pts_4d;
    cv::triangulatePoints(T1, T2, points1, points2, pts_4d);

    //转换成非齐次坐标
    for (int i = 0; i < pts_4d.cols; ++i) {
        Mat x = pts_4d.col(i);
        x /= x.at<float>(3,0);//归一化
        Point3d p(
                x.at<float>(0,0),
                x.at<float>(1,0),
                x.at<float>(2,0)
                );
        points.push_back(p);
    }
    //-- 估计运动

    //-- 三角化

    //-- 验证三角化点与特征点的重投影关系
}

void Frame::pose_estimation_2d2d(
        const vector<Point2f>& points1,
        const vector<Point2f>& points2,
        Mat &R, Mat &t)
{
    //-- 计算基础矩阵
    Mat fundamental_matrix;
    fundamental_matrix = cv::findFundamentalMat(points1, points2, CV_FM_8POINT);
    cout<<"fundamental_matirx is\n"<<fundamental_matrix<<endl;

    //-- 计算本质矩阵
    Point2d principal_point(365.0000, 265.0000);//相机主点
    double focal_length = 529.5000;//相机焦距
    Mat essential_matrix;
    essential_matrix = findEssentialMat(points1, points2, focal_length, principal_point);
    cout<<"essential_matrix is\n"<<essential_matrix<<endl;

    //-- 计算单应矩阵
    Mat homography_matrix;
    homography_matrix = findHomography(points1, points2, RANSAC, 3);
    cout<<"homography_matrix is \n"<<homography_matrix<<endl;

    //-- 从本质矩阵中恢复旋转各平移信息
    recoverPose(essential_matrix, points1, points2, R, t, focal_length, principal_point);
    cout<<"R is \n"<<R<<endl;
    cout<<"t is \n"<<t<<endl;

}

void Frame::setCubParameter() {

    //set detector parameters
    Eigen::Matrix3d calib;
    calib<<535.4,  0,  320.1,   // for KITTI cabinet data.
            0,  539.2, 247.6,
            0,      0,     1;
    detect_cuboid_obj.whether_plot_detail_images = false;
    detect_cuboid_obj.whether_plot_final_images = false;
    detect_cuboid_obj.print_details = false;
    detect_cuboid_obj.set_calibration(calib);
    detect_cuboid_obj.whether_sample_bbox_height = false;
    detect_cuboid_obj.nominal_skew_ratio = 2;
    detect_cuboid_obj.whether_save_final_images = true;

    line_lbd_obj.use_LSD = true;
    line_lbd_obj.line_length_thres = 15;
}
