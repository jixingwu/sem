//
// Created by jixingwu on 2020/9/1.
//

#ifndef SRC_ESTIMATOR_H
#define SRC_ESTIMATOR_H
#include "src/visual_odometry/visual_odometry.h"

class Estimator
{

public:
    Estimator();
    ~Estimator();

    void optimization();

private:
    VisualOdometry vo;
};

#endif //SRC_ESTIMATOR_H
