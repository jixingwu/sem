//
// Created by jixingwu on 2020/9/1.
//

#ifndef SRC_ESTIMATOR_H
#define SRC_ESTIMATOR_H
#include "projectionCameraObjectFactor.h"
#include "src/visual_odometry/TermColor.h"
#include <ceres/ceres.h>
#include "src/visual_odometry/frame.h"
#include "src/visual_odometry/map.h"

class Estimator
{

public:
    Estimator();
    ~Estimator();

    void optimization(const SemMap::Ptr& kmap, Frame::Ptr& curr);// update curr local_cuboids_, ref as landmark
};

#endif //SRC_ESTIMATOR_H
