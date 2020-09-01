//
// Created by jixingwu on 2020/7/20.
//

#include "src/camera_models/camera.h"
#include "src/visualization/config.h"

Camera::Camera()
{
//    fx_ = Config::get<double>("camera.fx");
//    fy_ = Config::get<double>("camera.fy");
//    cx_ = Config::get<double>("camera.cx");
//    cy_ = Config::get<double>("camera.cy");

//    fx_ = 718.856;//fx
//    fy_ = 718.856;//fy
//    cx_ = 607.1928;//cx
//    cy_ = 185.2157;//cy
//cout<<"------------"<<fx_<<fy_<<cx_<<cy_<<endl;

}
Camera::~Camera() {}

Vector3d Camera::world2camera(const Vector3d &p_w, const SE3 &T_c_w) {
    return T_c_w*p_w;
}

Vector3d Camera::camera2world(const Vector3d &p_c, const SE3 &T_c_w) {
    return T_c_w.inverse()*p_c;
}

Vector2d Camera::camera2pixel(const Vector3d &p_c) {
    return Vector2d(
            fx_*p_c(0,0) / p_c(2,0) + cx_,
            fy_*p_c(1,0) / p_c(2,0) + cy_
            );
}

Vector3d Camera::pixel2camera(const Vector2d &p_p, const SE3& T_c_w) {
    Matrix3d inMatrix;
    inMatrix<< fx_, 0, cx_, 0, fy_, cy_, 0, 0 ,1;
    Matrix3d R = T_c_w.rotation_matrix();
    Vector3d t = T_c_w.translation();

    double depth = calDepth(p_p, R, t, inMatrix);
//    cout<<"Depth: "<<depth<<endl;

    return Vector3d (
            ( p_p ( 0,0 )-cx_ ) *depth/fx_,
            ( p_p ( 1,0 )-cy_ ) *depth/fy_,
            depth
    );
}
Vector2d Camera::world2pixel(const Vector3d &p_w, const SE3 &T_c_w) {
    return camera2pixel( world2camera(p_w, T_c_w));
}

Vector3d Camera::pixel2world ( const Vector2d& p_p, const SE3& T_c_w )
{
    return camera2world ( pixel2camera ( p_p, T_c_w ), T_c_w );
}

double Camera::calDepth(const Vector2d &p_p, Matrix3d R, Vector3d t, Matrix3d inMatrix)
{
    Vector3d p_p_q(p_p(0), p_p(1), 1);
    Vector3d rightMatrix = R.inverse()*inMatrix.inverse()*p_p_q;
    Vector3d leftMatrix = R.innerStride()*t;
    return leftMatrix(2,0)/rightMatrix(2,0);
}