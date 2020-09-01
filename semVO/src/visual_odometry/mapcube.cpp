//
// Created by jixingwu on 2020/7/20.
//

#include "mapcube.h"

MapCube::MapCube()
        : id_(-1), pose_(Sophus::SE3(Eigen::Quaterniond(1,0,0,0), Vector3d(0,0,0)))
        , scale_(Vector3d(0,0,0)), observed_times_(0)
{

}

MapCube::Ptr MapCube::createMapCube()
{
    static long factory_id = 0;
    return MapCube::Ptr(
            new MapCube( factory_id++, Sophus::SE3(Eigen::Quaterniond(1,0,0,0), Vector3d(0,0,0)) , Vector3d(0,0,0) )
    );
}
