//
// Created by jixingwu on 2020/7/20.
//

#include "mapcube.h"

MapCube::MapCube()
        : id_(-1), pos_(Vector3d(0,0,0)), norm_(Vector3d(0,0,0)), observed_times_(0)
{

}

MapCube::MapCube ( long id, Vector3d position, Vector3d norm )
        : id_(id), pos_(position), norm_(norm), observed_times_(0)
{

}

MapCube::Ptr MapCube::createMapCube()
{
    static long factory_id = 0;
    return MapCube::Ptr(
            new MapCube( factory_id++, Vector3d(0,0,0), Vector3d(0,0,0) )
    );
}
