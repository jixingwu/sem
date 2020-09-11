//
// Created by jixingwu on 2020/9/1.
//
#include "estimator.h"
#include <ros/console.h>


Estimator::Estimator() {

}

Estimator::~Estimator() {

}
#define __DEBUG__(msg) ;
#define __DEBUG_OPT__(msg) ;
void Estimator::optimization(const SemMap::Ptr& kmap, Frame::Ptr& currFrame)
{
    ROS_DEBUG_NAMED("Estimator::optimization()", "opt: %lu", currFrame->id_);
    __DEBUG_OPT__(cout<<TermColor::iWHITE()<<"starting opt !!"<<endl;)
    // 定义Cost Function模型，书写一个类，在类中定义带末班
    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    loss_function = new ceres::HuberLoss(1.0);

    // 观测量：landmark and camera pose, 待估计量：currFrame

//    auto v_currMapCube = vo.curr_->local_cuboids_;
    __DEBUG_OPT__(cout<<"curr.id: "<<currFrame->id_<<endl;)

    // 向问题中添加误差项
    // problem.add......
    for (auto &init_cube : currFrame->local_cuboids_)
    {
        __DEBUG_OPT__(cout<<"init_cube id: "<<init_cube->id_<<endl;)
        for(const auto& map_cube : kmap->map_cubes_)
        {
            auto iter = kmap->map_cubes_.find(init_cube->id_);
            if (iter == kmap->map_cubes_.end()) continue;

            Eigen::Quaterniond  init_quat = init_cube->pose_.unit_quaternion();
            Eigen::Vector3d     init_trans = init_cube->pose_.translation();
            Eigen::Vector3d     init_dimen = init_cube->scale_;

            // q: xyzw, t:xyz, d: length width height;
            double *para_quat = Vector4d(init_quat.x(), init_quat.y(), init_quat.z(), init_quat.w()).data();
            double *para_trans = init_trans.data();
            double *para_dimen = init_dimen.data();

            auto *f_td = new ProjectionCameraObjectFactor( map_cube.second, currFrame->T_c_w_ );// landmark and pose

            problem.AddResidualBlock(
                    new ceres::AutoDiffCostFunction<ProjectionCameraObjectFactor, 10, 4, 3, 3>(f_td),
                    loss_function,
                    para_quat, para_trans, para_dimen
            );

        }
        if(problem.NumResiduals() != 0)
        {
            // 配置求解器
            ceres::Solver::Options options;
            options.linear_solver_type = ceres::DENSE_QR;
            options.minimizer_progress_to_stdout = true;
//            options.gradient_tolerance = 1e-16;
//            options.function_tolerance = 1e-16;


            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);

            cout << summary.BriefReport() <<endl;
        }
        else
        {
            cout<<"problem residual num is zero!!, camera may occur to jump, please be cared!!\n";
        }


    }


    __DEBUG__(cout<<"ending opt!"<<TermColor::RESET()<<endl;)
    return;
}
