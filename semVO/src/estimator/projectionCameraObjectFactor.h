//
// Created by jixingwu on 2020/9/1.
//

#ifndef SRC_PROJECTIONCAMERAOBJECTFACTOR_H
#define SRC_PROJECTIONCAMERAOBJECTFACTOR_H
#include <ceres/ceres.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <utility>

#include "src/detect_3d_cuboid/detect_3d_cuboid.h"
#include "src/visual_odometry/mapcube.h"
/*
 * :public SizedCostFunction<1 //number of residuals
 *                          1 //size of first parameter>
 */
typedef Eigen::Matrix<double, 9, 1> Vector9d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
using namespace Eigen;
using namespace std;

struct ProjectionCameraObjectFactor // : public ceres::SizedCostFunction<9, 6, 3> // e_co_3D: 9维
{
public:
    ProjectionCameraObjectFactor(MapCube landmark, Sophus::SE3 pose)
    {
        T_w_c_landmark_ = Matrix4d::Identity();
        T_w_c_landmark_.topLeftCorner(3,3) = landmark.pose_.rotation_matrix();
        T_w_c_landmark_.col(3).topRows(3) = landmark.pose_.translation();

        dimen_landmark_ = landmark.scale_;

        T_w_c_pose_ = Matrix4d::Identity();
        T_w_c_pose_.topLeftCorner(3,3) = pose.rotation_matrix();
        T_w_c_pose_.col(3).topRows(3) = pose.translation();
    }
    // 输入观测量:landmark(T_o, d) and pose(T_c), 待估计量cuboid T_om d_m
    // T_o: (R,t)/SE3, d: vector3d.
    // equ: [log(Tc^-1 * To * Tom^-1), d - dm]

    /*Sorry to bother I figured out the issue:
Changing this fixed it:

    Matrix<T,4,4> f = nodepose.cast<T> ();


and

Matrix<T,4,4> delta = f.inverse() * npose ;
Matrix<T,3,3> R = delta.topLeftCorner(3,3);
Quaternion<T> delta_q( R );

 * */

    template<typename T>
    bool operator()(const T* const kquat, const T* const ktrans, const T* const kdimen, T* residuals) const
    {
        // q: wxyz, t:xyz, d: length width height;
        // quat, trans --> T_w_c
        Eigen::Map<const Eigen::Matrix<T, 3, 1>>    trans(ktrans);
        Eigen::Map<const Eigen::Quaternion<T>>      quat(kquat);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>>    dimen(kdimen);
        Eigen::Matrix<T, 4, 4>                      T_w_c;

        T_w_c = Matrix<T, 4, 4>::Identity();
        T_w_c.topLeftCorner(3, 3) = quat.toRotationMatrix();
        T_w_c.col(3).topRows(3) = trans;

        // class member --> T
        Matrix<T, 4, 4> T_w_c_landmark = T_w_c_landmark_.cast<T>();
        Matrix<T, 4, 4> T_w_c_pose = T_w_c_pose_.cast<T>();
        Matrix<T, 3, 1> dimen_landmark = dimen_landmark_.cast<T>();

        Matrix<T, 4, 4> delta = T_w_c_pose.inverse() * T_w_c_landmark * T_w_c.inverse();
        Matrix<T, 3, 3> rotation_matrix = delta.topLeftCorner(3,3);// \HERE: an error
        Quaternion<T>   delta_q(rotation_matrix);

        Eigen::Map<Matrix<T, 10, 1>> res(residuals);// res = [q(xyzw), t, d]

        res.topRows(4)  = Matrix<T, 4, 1>(delta_q.x(), delta_q.y(), delta_q.z(), delta_q.w());
        res.middleRows(4, 3) = delta.col(3).topRows(3);
        res.bottomRows(3) = (dimen_landmark - dimen).cwiseAbs();

//        cout<<"dimen_land:\n"<<dimen_landmark<<endl;
//        cout<<"dimen\n"<<dimen<<endl;
//        cout<<"res:\n"<<res<<endl;
//        T a = residuals[3];
//        cout<<"residuals:\n"<<res<<endl;
        return true;
    }

private:

    Matrix4d T_w_c_landmark_;
    Vector3d dimen_landmark_;
    Matrix4d T_w_c_pose_;
};

#endif //SRC_PROJECTIONCAMERAOBJECTFACTOR_H
