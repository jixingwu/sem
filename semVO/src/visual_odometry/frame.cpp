//
// Created by jixingwu on 2020/7/20.
//

#include "frame.h"

#include <utility>

Frame::Frame()
: id_(-1), time_stamp_(-1), camera_(nullptr), is_key_frame_(false)
{
}

Frame::Frame ( long id, double time_stamp, const SE3& T_c_w, Camera::Ptr camera, Mat color, Mat depth )
        : id_(id), time_stamp_(time_stamp), T_c_w_(T_c_w), camera_(std::move(camera)), is_key_frame_(false)
{
}

Frame::~Frame()
{
}

Frame::Ptr Frame::createFrame()
{
    static long factory_id = 0;
    return Frame::Ptr( new Frame(factory_id++) );
}

void Frame::setPose(const SE3 &T_c_w)
{
    T_c_w_ = T_c_w;
}