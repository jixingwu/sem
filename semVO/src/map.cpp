//
// Created by jixingwu on 2020/7/20.
//

#include "map.h"

void MapObject::insertKeyFrame(Frame::Ptr frame)
{
    cout<< "Key frame size = "<< keyframes_.size() << endl;
    if ( keyframes_.find(frame->id_) == keyframes_.end() )
    {
        keyframes_.insert( make_pair(frame->id_, frame) );
    }
    else
    {
        keyframes_[ frame->id_ ] = frame;
    }
}

void MapObject::insertMapCube(MapCube::Ptr map_cube)
{
    if(map_cubes_.find(map_cube->id_) == map_cubes_.end() )
        map_cubes_.insert( make_pair(map_cube->id_, map_cube));
    else
        map_cubes_[map_cube->id_] = map_cube;
}
