//
// Created by jixingwu on 2020/2/13.
//

#ifndef SRC_LANDMARK_H
#define SRC_LANDMARK_H

#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <sensor_msgs/Image.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include "detect_3d_cuboid/detect_3d_cuboid.h"
#include "detect_3d_cuboid/object_3d_util.h"

#include <Eigen/Core>

class Landmark
{
public:
    Landmark();
    ~Landmark();

};



#endif //SRC_LANDMARK_H
