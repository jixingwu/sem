//
// Created by jixingwu on 2020/5/21.
//

#include "DataManager.h"

DataManager::DataManager(ros::NodeHandle &nh)
{
    this->nh = nh;
}

DataManager::DataManager(const DataManager &obj)
{
    cout << "[DataManager.cpp] Copy constructor allocating ptr.\n";
}

///////////////////////////////////////////////////////////////////////
////////////////////Kidnap Indicator Publisher/////////////////////////
///////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////
//////////////////// call backs ///////////////////////////////////////
///////////////////////////////////////////////////////////////////////

#define __DATAMANAGER_CALLBACK_PRINT__(msg);

void DataManager::camera_pose_callback(const nav_msgs::Odometry::ConstPtr msg)
{
    if(pose_0_available == false){
        pose_0 = msg->header.stamp;
        pose_0_available = true;
    }

    __DATAMANAGER_CALLBACK_PRINT__(cout << TermColor::BLUE() << "[cerebro/camera_pose_callback]"
    << msg->header.stamp << "\t" << msg->header.stamp - pose_0 << TermColor::RESET() << endl; )
    // push this to queue. Another thread will associate the data
    pose_buf.push(msg);
    return;
}

void DataManager::cube_pose_callback(const nav_msgs::Odometry::ConstPtr msg) {
    __DATAMANAGER_CALLBACK_PRINT__( cout << TermColor::iBLUE() << "[cerebro/cube_pose_callback]"
    << msg->header.stamp << "\t" << msg->header.stamp - pose_0 << TermColor::RESET() << endl;)
    // push this to queue. Another thread will associate the data
    cube_pose_buf.push(msg);
    return;
}

void DataManager::raw_image_callback(const sensor_msgs::ImageConstPtr &msg) {
    __DATAMANAGER_CALLBACK_PRINT__( cout<< TermColor::GREEN() << "[cerebro/raw_image_callback]"
    << msg->header.stamp << "\t" << msg->header.stamp-pose_0 << TermColor::RESET() << endl;)

    if(this->last_image_time != ros::Time() &&
    (msg->header.stamp.toSec() - this->last_image_time.toSec() > 1.0 || msg->header.stamp.toSec() < this->last_image_time.toSec())
    )
    {
        cout<< TermColor::iBLUE()
        << "---------------[cerebro/raw_image_callback]"
        << "curr_image.stamp - prev_image.stamp > 1.0\n"
        << "I will publish a FALSE t=" << 10000 << "then wait for 500ms and publish TRUE t="<< 10000
        << " TODO: Complete this implementation and verify correctness."
        << TermColor::RESET() << endl;

        // TODO: Publish FALSE, sleep 500ms and Publish TRUE
    }

    img_buf.push(msg);
    this->last_image_time = msg->header.stamp;
    return;

}

string DataManager::print_queue_size(int verbose) const {

    std::stringstream buffer;

    if(verbose == 1){
        buffer << "img_buf=" << img_buf.size() << "\t";
        buffer << "pose_buf=" << pose_buf.size() << "\t";
        buffer << "cube_pose_buf=" << cube_pose_buf.size() << "\t";
        buffer << endl;
    }

    if(verbose == 2){
        buffer << "img_buf=" << img_buf.size() << " (";
        if(img_buf.size() > 0){
            buffer << std::fixed << std::setprecision(4) << img_buf.front()->header.stamp-pose_0 << "-->";
            buffer << std::fixed << std::setprecision(4) << img_buf.back()->header.stamp-pose_0 << ";";
        }
        buffer << ")\t";

        buffer << "pose_buf=" << pose_buf.size() << " (";
        if(pose_buf.size() > 0){
            buffer << std::fixed << std::setprecision(4) << pose_buf.front()->header.stamp-pose_0 << "-->";
            buffer << std::fixed << std::setprecision(4) << pose_buf.back()->header.stamp-pose_0 << ";";
        }
        buffer << ")\t";

        buffer << "cube_pose_buf=" << cube_pose_buf.size() << " (";
        if(cube_pose_buf.size() > 0){
            buffer << std::fixed << std::setprecision(4) << cube_pose_buf.front()->header.stamp-pose_0 << "-->";
            buffer << std::fixed << std::setprecision(4) << cube_pose_buf.back()->header.stamp-pose_0 << ";";
        }
        buffer << ")\t";
        buffer << "\n";
    }

    if(verbose == 3 ){
        buffer << "img_buf=" << img_buf.size() << "\t";
        buffer << "pose_buf=" << pose_buf.size() << "\t";
        buffer << "cube_pose_buf=" << cube_pose_buf.size() << "\t";
        buffer << endl;

        if(img_buf.size() > 0)
            buffer << "img_buf.back.t=" << img_buf.back()->header.stamp-pose_0 << "\t";
        if(pose_buf.size() > 0)
            buffer << "pose_buf.back.t=" << pose_buf.back()->header.stamp-pose_0 << "\t";
        if(cube_pose_buf.size() > 0)
            buffer << "cube_buf.back.t=" << cube_pose_buf.back()->header.stamp-pose_0 << "\t";
        buffer << endl;

        if(img_buf.size() > 0)
            buffer << "img_buf.front.t=" << img_buf.front()->header.stamp-pose_0 << "\t";
        if(pose_buf.size() > 0)
            buffer << "pose_buf.front.t=" << pose_buf.front()->header.stamp-pose_0 << "\t";
        if(cube_pose_buf.size() > 0)
            buffer << "cube_buf.front.t=" << cube_pose_buf.front()->header.stamp-pose_0 << "\t";
        buffer << endl;
    }

    return buffer.str();
}

//////////////////////////////// Callback ends //////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////// Thread mains ////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
#define  __XOUT_MYFIEL__
void DataManager::trial_thread() {
    cout<< TermColor::GREEN() << "Start DataManager::trial_thread" << TermColor::RESET() << endl;
    ros::Rate looprate(10);

#if 0

#endif

    while(b_trial_thread)
    {
        img_data_mgr->print_status("/dev/pts/23");
        this->print_datamap_status("/dev/pts/24");
        looprate.sleep();
    }

    cout << TermColor::RED() << "END DataManager::trial_thread "<< TermColor::RESET() << endl;
}

#define __CLEAN_UP_COUT__(msg);
void DataManager::clean_up_useless_images_thread()
{

}

#define __DataManager_data_association_thread__(msg);
void DataManager::data_association_thread(int max_loop_rat_in_hz)
{
    assert(max_loop_rat_in_hz > 0 && max_loop_rat_in_hz < 200 && "[DataManager::data_association_thread] I am expecting the loop rate to be between 1-50");
    cout << TermColor::GREEN() << "[DataManager::data_association_thread] Start thread" << TermColor::RESET() << endl;
    assert(b_data_association_thread && "You have not enabled thread execution. Call function DataManager::data_association_thread_enable() before you spawn this thread\n");
    float requested_loop_time_ms = 1000. / max_loop_rat_in_hz;

    while(b_data_association_thread)
    {
        auto loop_start_at = std::chrono::high_resolution_clock::now();

        //--------------------- Process here ----------------------------------
        __DataManager_data_association_thread__(
                cout << "--\n";
                cout << print_queue_size(2);
                cout << "\t \t Size_of_data_map = " + std::to_string(data_map->size()) + "\n";
        )

        while(img_buf.size() > 0 )
        {
            sensor_msgs::ImageConstPtr img_msg = img_buf.front();
            img_buf.pop();
            __DataManager_data_association_thread__(
                    cout << TermColor::GREEN() << ">>>>>>> Added a new DataNode in data_map with poped() raw image t="
                    << img_msg->header.stamp << "####> ie." << img_msg->header.stamp-pose_0 << TermColor::RESET() << endl;
            )
            DataNode* n = new DataNode(img_msg->header.stamp);
            img_data_mgr->setNewImageFromMsg("left_image", img_msg);
            data_map->insert(std::make_pair(img_msg->header.stamp, n));
        }
    // additional raw images
    // depth images
    // dequeue all poses(Cube Pose) and set them to data_map
        while(cube_pose_buf.size() > 0)
        {
            nav_msgs::OdometryConstPtr cube_pose_msg = cube_pose_buf.front();

            ros::Time t = cube_pose_msg->header.stamp;
            __DataManager_data_association_thread__(
                    cout<<">> Attempt adding poped() pose in data_map with t=" << cube_pose_msg->header.stamp << "ie. #####" << cube_pose_msg->header.stamp-pose_0 << endl;
            )
            // find the DataNode with this timestamp
            //berks__old
            // TODO: add cube poses to map
            if(data_map->count(t) > 0){
                // a Node seem to exist with this t
                data_map->at(t)->setCubePoseFromMsg(cube_pose_msg);
                cube_pose_buf.pop();
            }
            else{
                if( t > data_map->rbegin()->first){
                    __DataManager_data_association_thread__(
                            cout << "\tpose's t was not yet found in datamap. data_map->rbegin()->first=" << data_map->rbegin()->first << " ";
                            cout << "this means a node doesnt exists yet for this pose. Usually this does not happen, but it occurs when Image data manager took too long to insert (thread blocking) and in the meantime more poses got available.\nI will not pop the queue in this.\n";
                );
                    break;
                }
                __DataManager__data_association_thread__( cout << "\tsince the key (for associating pose with data_map) was not found in data_map do range_search\n"; )

                for (auto  __it = data_map->begin(); __it != data_map->end() ; ++__it) {
                    ros::Duration diff = __it->first-t;
                    if( (diff.sec == 0 && abs(diff.nsec) < 1000000) || (diff.sec == -1 && diff.nsec > (100000000-1000000)))
                        break;
                }

                if(__it == data_map->end()){
                    __DataManager__data_association_thread__(cout << TermColor::RED() << "\t`data_association_thread`:pose:(not fouind) range search failed AAA FATAL \n";)
                    assert(false && "\t not found\n");
                    exit(2);
                }
                else
                    data_map->at(__it->first)->setCubePoseFromMsg(cube_pose_msg);
            }



        }

        //-------------------------Done processing----------------------------
        auto loop_end_at = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> ellapsed = loop_end_at-loop_start_at;

        int sleep_for = int(requested_loop_time_ms - (float)ellapsed.count());
        __DataManager_data_association_thread__(cout << "Loop iteration done in "<< ellapsed.count() << "ms; sleep for=" << sleep_for << "ms" << endl;)

        if(sleep_for > 0)
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_for));
        else{
            __DataManager_data_association_thread__(cout << "Queueing in thread 'data_association_thread'\n";)
            ROS_WARN( "Queueing in thread `data_association_thread`. requested_loop_time_ms=%f; elapsed=%f. If this occurs occasionally, it might be because of image_manager thread getting blocked by cleanup thread. It is perfectly normal.", requested_loop_time_ms, ellapsed.count() );
        }

        cout << TermColor::RED() << "[DataManager::data_association_thread] Finished thread" << TermColor::RESET() << endl;

    }

}

// TODO 寻找深度学习网络如何引入参数 at descriptor computer thread()中

























