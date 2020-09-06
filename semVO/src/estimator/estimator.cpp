//
// Created by jixingwu on 2020/9/1.
//
#include "estimator.h"
#include "projectionCameraObjectFactor.h"
#include <ceres/ceres.h>


#include "src/visual_odometry/frame.h"

Estimator::Estimator() {

}

Estimator::~Estimator() {

}

void Estimator::optimization()
{
    // 定义Cost Function模型，书写一个类，在类中定义带末班
    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    loss_function = new ceres::HuberLoss(1.0);

    // 观测量：landmark and camera pose, 待估计量：
    Frame::Ptr refFrame = vo.ref_;// as landmark
    Frame::Ptr currFrame = vo.curr_;

    vector<MapCube*> v_currMapCube = vo.curr_->local_cuboids_;

    // 向问题中添加误差项
    // problem.add......
    for(int ii = 0; ii < min(refFrame->local_cuboids_.size(), currFrame->local_cuboids_.size()); ++ii)
    {
        MapCube *init_cube = currFrame->local_cuboids_[ii];

        Eigen::Quaterniond  init_quat = init_cube->pose_.unit_quaternion();
        Eigen::Vector3d     init_trans = init_cube->pose_.translation();
        Eigen::Vector3d     init_dimen = init_cube->scale_;

        // q: xyzw, t:xyz, d: length width height;
        double *para_quat = Vector4d(init_quat.x(), init_quat.y(), init_quat.z(), init_quat.w()).data();
        double *para_trans = init_trans.data();
        double *para_dimen = init_dimen.data();

        ProjectionCameraObjectFactor *f_td = new ProjectionCameraObjectFactor( *refFrame->local_cuboids_[ii], currFrame->T_c_w_ );// landmark and pose

        problem.AddResidualBlock(
                new ceres::AutoDiffCostFunction<ProjectionCameraObjectFactor, 10, 4, 3, 3>(f_td),
                loss_function,
                para_quat, para_trans, para_dimen
                );
    }


    // 配置求解器
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    options.gradient_tolerance = 1e-16;
    options.function_tolerance = 1e-16;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    cout << summary.FullReport() <<endl;
    return;
}
