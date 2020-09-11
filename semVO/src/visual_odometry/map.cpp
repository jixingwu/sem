//
// Created by jixingwu on 2020/7/20.
//

#include "map.h"

void SemMap::insertKeyFrame(const Frame::Ptr& frame)
{
//    cout<< "Key frame size = "<< keyframes_.size() << endl;
    if ( keyframes_.find(frame->id_) == keyframes_.end() )
    {
        keyframes_.insert( make_pair(frame->id_, frame) );
    }
    else
    {
        keyframes_[ frame->id_ ] = frame;
    }
}

void SemMap::insertMapCube(const MapCube &map_cube)
{
//    cout<< "Map cube size = "<< map_cubes_.size()<<endl;
    if(map_cubes_.find(map_cube.id_) == map_cubes_.end() )
        map_cubes_.insert( make_pair(map_cube.id_, map_cube));
    else
        map_cubes_[map_cube.id_] = map_cube;
}
