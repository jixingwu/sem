//
// Created by jixingwu on 2020/7/20.
//

#ifndef SRC_FRAME_H
#define SRC_FRAME_H

#include "camera.h"
#include "Object_landmark.h"
#include "detect_3d_cuboid/detect_3d_cuboid.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "mapcube.h"
class MapCube;
class Frame
{
public:
    typedef std::shared_ptr<Frame> Ptr;
    unsigned long   id_;  // id of this frame
    double          time_stamp_; // when it is recorded
    SE3             T_c_w_; // transform from world to camera
    Camera::Ptr     camera_;
    bool            is_key_frame_;

    /// image and bboxes timestamp have been aligned
    cv::Mat rgb_image_;
    darknet_ros_msgs::BoundingBoxesConstPtr bboxes_;

    // cube parameter
    int                             cube_num_; // cube number of this frame
    std::vector<ObjectSet*>         frame_cuboids_; // cube raw proposal, 处理前的cube，由多个cuboid类型组成
    std::vector<MapCube*>           local_cuboids_;// cube sequence of this frame, 处理后的cube类型，该类型可用于构建SemMap
    std::vector<object_landmark*>   observed_cuboids_;// cube sequence of this frame

public:
    Frame();
    Frame(long id, double time_stamp=0, SE3 T_c_w=SE3(), Camera::Ptr camera=nullptr, Mat color=Mat(), Mat depth=Mat() );
    ~Frame();

    static Frame::Ptr createFrame();

    double findDepth();

    Vector3d getCamCenter() const;

    bool isInFrame();

    void setPose( const SE3& T_c_w);

    // check if a point is in this frame
    bool isInFrame( const Vector3d& pt_world);
};

#endif //SRC_FRAME_H
