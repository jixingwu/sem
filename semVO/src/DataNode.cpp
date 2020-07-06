//
// Created by jixingwu on 2020/6/18.
//

#include "DataNode.h"

void DataNode::setLeftImageFromMsg(const sensor_msgs::Image &msg) {
    std::lock_guard<std::mutex> lk(m);
}
