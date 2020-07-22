//
// Created by jixingwu on 2020/7/20.
//

#include "camera.h"
#include "config.h"

Camera::Camera()
{
    fx_ = Config::get<float>("camera.fx");
    fy_ = Config::get<float>("camera.fy");
    cx_ = Config::get<float>("camera.cx");
    cy_ = Config::get<float>("camera.cy");

}
