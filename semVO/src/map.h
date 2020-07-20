//
// Created by jixingwu on 2020/7/20.
//

#ifndef SRC_MAP_H
#define SRC_MAP_H
#include "frame.h"
#include "mapcube.h"

class Map
{
public:
    typedef shared_ptr<Map> Ptr;
    map<unsigned long, MapCube::Ptr> map_cubes_;// all landmarks
    map<unsigned long, Frame::Ptr>   keyframes_;   // all frames

    Map(){}

    void insertKeyFrame(Frame::Ptr frame);
    void insertMapCube(MapCube::Ptr map_cube);
};

#endif //SRC_MAP_H
