//
// Created by jixingwu on 2020/5/21.
//

#include "Cerebro.h"

//--------------------------------------------------------//
// NetVLAD Neural Networks is a way of whole image descriptor computer.
// this thread is used descriptor computing.
// INPUT: dataManager->data_map.cube_pose instead of keyframe_pose.
// USAGE:
// OUTPUT: whole descriptor
//--------------------------------------------------------//

void Cerebro::descriptor_computer_thread()
{

}

void Cerebro::loopcandidata_consumer_thread()
{
    assert(m_dataManager_available);
    assert(b_loopcandidata_consumer);
    cout<< TermColor::GREEN() << "[Cerebro::loopcandidate_consumer_thread] Start thread" << TermColor::RESET() << endl;

    bool stereogeome_status = true;
    if(!stereogeome_status){
        assert(false && "[Cerebor::loop] cannot init_stereogeom\n");
        return;
    }

    // init pt-feature-matcher

    ros::Rate rate(1);
    int prev_count = 0;
    int new_count = 0;
    ElapsedTime timer;

    while(b_loopcandidata_consumer)
    {

    }
}

const int Cerebro::foundLoops_count() const {
    std::lock_guard<std::mutex> lk(m_foundLoops);
    return foundLoops.size();
}
