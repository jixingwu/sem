//
// Created by jixingwu on 2020/7/20.
//

#ifndef SRC_CAMERA_H
#define SRC_CAMERA_H
#include <Eigen/Core>
#include <Eigen/Geometry>
using Eigen::Vector2d;
using Eigen::Vector3d;

// for Sophus
#include <sophus/se3.h>
using Sophus::SE3;

// for cv
#include <opencv2/core/core.hpp>
using cv::Mat;

// for eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
using namespace Eigen;

// std
#include <vector>
#include <list>
#include <memory>
#include <string>
#include <iostream>
#include <set>
#include <unordered_map>
#include <map>

class Camera
{
public:
    typedef std::shared_ptr<Camera> Ptr;
//    double   fx_, fy_, cx_, cy_;  // Camera intrinsics
private:
    double fx_ = 718.856;//fx
    double fy_ = 718.856;//fy
    double cx_ = 607.1928;//cx
    double cy_ = 185.2157;//cy

    double calDepth(const Vector2d& p_p, Matrix3d R, Vector3d t, Matrix3d inMatrix);
    void setExternalParams(Quaterniond Q, Vector3d t);
    void setExternalParams(Matrix3d R, Vector3d t);

public:
    Camera();
    ~Camera();

    // coordinate transform: world, camera, pixel
    Vector3d world2camera( const Vector3d& p_w, const SE3& T_c_w );
    Vector3d camera2world( const Vector3d& p_c, const SE3& T_c_w );
    Vector2d camera2pixel( const Vector3d& p_c );
    Vector3d pixel2camera( const Vector2d& p_p, const SE3& T_c_w);
    Vector3d pixel2world ( const Vector2d& p_p, const SE3& T_c_w);
    Vector2d world2pixel ( const Vector3d& p_w, const SE3& T_c_w );



};

using namespace std;

#endif //SRC_CAMERA_H
