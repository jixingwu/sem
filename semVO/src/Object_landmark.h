#pragma once

#include <vector>

#include "g2o_Object.h"

//maybe rename as cube.h
class object_landmark{
public:

    g2o::cuboid cube_meas;  //cube_value
    g2o::VertexCuboid* cube_vertex;
    double meas_quality;  // [0,1] the higher, the better

};