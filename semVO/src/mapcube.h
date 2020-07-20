//
// Created by jixingwu on 2020/7/20.
//

#ifndef SRC_MAPCUBE_H
#define SRC_MAPCUBE_H
#include "camera.h"
class Frame;
class MapCube
{
public:
    typedef shared_ptr<MapCube> Ptr;
    unsigned long id_;
    Vector3d        pos_;// Position in world
    Vector3d        norm_;// Normal of viewing direction
    Mat             descriptor_;
    int             observed_times_;// being observed by feature matching algo.
    int             matched_times_;// being an inliner in pose estimation

    // TODO: cube所需要的参数

    MapCube();
    MapCube(long id, Vector3d position, Vector3d norm);

    static MapCube::Ptr createMapCube();
};

#endif //SRC_MAPCUBE_H
