//
// Created by jixingwu on 2020/9/1.
//
#include "estimator.h"
#include <ceres/ceres.h>

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

    // 向问题中添加误差项
    // problem.add......

    // 配置求解器
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
}
