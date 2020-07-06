//
// Created by jixingwu on 2020/5/21.
//

#ifndef SRC_CEREBRO_H
#define SRC_CEREBRO_H

#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <vector>

#include "utils/TermColor.h"
#include "utils/ElapsedTime.h"
#include "utils/Plot2Mat.h"

#include "utils/CameraGeometry.h"
#include "utils/PointFeatureMatching.h"

#include <std_msgs/Bool.h>
#include <std_msgs/Header.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace Eigen;
using namespace std;

#include "DataManager.h"

class Cerebro
{
public:
    Cerebro(ros::NodeHandle& nh);// TODO: remove if argument
    void setDataManager(DataManager* dataManager);
    void setPublishers(const string base_topic_name);

private:
    bool m_dataManager_available = false;
    DataManager *dataManager;
    ros::NodeHandle nh;

    bool m_pub_available = false;
    ros::Publisher pub_loopedge;
    //-------------------------- END Constructor ----------------------------//

    //-------------------------- Descriptor Computation Thread -----------------------------//
public:
    /*****************************************************************************************************
     * This monitors the dataManager->data_map and makes sure the descriptor are update
     * descriptors are computed by an external ros-service, in the future can have
     * more similar threads to compute object bounding boxes, text and other perception related services
    *****************************************************************************************************/
    void descriptor_computer_thread_enable(){ b_descriptor_computer_thread = true;}
    void descriptor_computer_thread_disable(){ b_descriptor_computer_thread = false;}
    void descriptor_computer_thread();

private:
    atomic<bool> b_descriptor_computer_thread;
    atomic<bool> connected_to_descriptor_server;
    atomic<bool> descriptor_size_available;
    atomic<int> descriptor_size;

    // stroage for intelligence
    mutable std::mutex m_wholeImageComputedList;
    vector<ros::Time> wholeImageComputedList;
    void wholeImageComputedList_pushback(const ros::Time __tx);
public:
    const int wholeImageComputedList_size() const;// size of the list. threadsafe
    const ros::Time wholeImageComputedList_at(int k) const;// returns kth element of the list. threadsafe
    //-------------------- END Descriptor Computation Thread ----------------------------------//

public:
    const int foundLoops_count() const;

private:
    mutable std::mutex m_foundLoops;
    vector< std::tuple<ros::Time, ros::Time, double> > foundLoops; // a list containing loop pairs. this is populated by `run()`

    //-------------------- Geometry Thread ---------------------------------------------------//
    //calls this->foundloop_cout() and this->foundLoop_i() and uses dataManager
    // to geometric verify and to compute the poses of loop-pairs
public:
    void loopcandidate_consumer_enable(){b_loopcandidata_consumer=true;}
    void loopcandidata_consumer_disable(){b_loopcandidata_consumer= false;}
    void loopcandidata_consumer_thread();

private:
    atomic<bool> b_loopcandidata_consumer;

public:
    //kidnap callbacks
    void kidnap_bool_sem_callback(const std_msgs::BoolConstPtr& rcvd_flag);
    void kidnap_header_sem_callback(const std_msgs::HeaderConstPtr& rcvd_header);
};
#endif //SRC_CEREBRO_H
