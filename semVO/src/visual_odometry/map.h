//
// Created by jixingwu on 2020/7/20.
//

#ifndef SRC_MAP_H
#define SRC_MAP_H
#include "frame.h"
#include "mapcube.h"

class SemMap
{
public:
    typedef shared_ptr<SemMap> Ptr;
    unordered_map< unsigned long, MapCube> map_cubes_;// all landmarks
    unordered_map< unsigned long, Frame::Ptr >   keyframes_;   // all frames

    SemMap(){}

    void insertKeyFrame(const Frame::Ptr& frame);
    void insertMapCube(const MapCube &map_cube);
};

#endif //SRC_MAP_H
