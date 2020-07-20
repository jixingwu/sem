//
// Created by jixingwu on 2020/7/20.
//

#ifndef SRC_FRAME_H
#define SRC_FRAME_H

#include "camera.h"
#include "Object_landmark.h"
class MapCube;
class Frame
{
public:
    typedef std::shared_ptr<Frame> Ptr;
    unsigned long   id_;  // id of this frame
    double          time_stamp_; // when it is recorded
    SE3             T_c_w; // transform from world to camera
    Camera::Ptr     camera_;
    Mat             color_, depth_; // color and depth image

    // cube parameter
    int                             cube_num_; // cube number of this frame
    std::vector<object_landmark*>   observed_cuboids_;// cube sequence of this frame

public:
    Frame();
    Frame(long id, double time_stamp=0, SE3 T_c_w=SE3(), Camera::Ptr camera=nullptr, Mat color=Mat(), Mat depth=Mat() );
    ~Frame();

    static Frame::Ptr createFrame();

    double findDepth();

    Vector3d getCamCenter() const;

    bool isInFrame();
};

#endif //SRC_FRAME_H
