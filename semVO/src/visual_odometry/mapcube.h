//
// Created by jixingwu on 2020/7/20.
// 一种'feature'的描述子

#ifndef SRC_MAPCUBE_H
#define SRC_MAPCUBE_H
#include "src/camera_models/camera.h"
#include "frame.h"

#include "src/detect_3d_cuboid/detect_3d_cuboid.h"
using namespace std;

class MapCube
{
public:
    typedef shared_ptr<MapCube> Ptr;
    int id_;

    int             observed_times_;// being observed by feature matching algo.
    int             matched_times_;// being an inliner in pose estimation

    // -------------- for cube param----------------
    SE3             pose_;
    SE3             worldPose_;
    Vector3d        scale_;
    string          object_class;        // detected object's class
    cv::Rect        bbox_2d_;            // (int) yolo detected 2D bbox_2d x y w h
    Eigen::Vector4d bbox_vec_;           // xmin, ymin , width, height

    double meas_quality_;                // [0,1] the higher, the better
    Eigen::MatrixXi box_corners_2d_;     // 2*8 on image usually for local cuboids on reference frame.
    Eigen::MatrixXi edge_markers_;       // in order to plot 2d cuboids with 8 corners.
    cv::Rect bbox_2d_tight_;             // tighted 2d object, used to find points association.
    int object_id_in_localF_;            // object id in reference frame's local objects;

    // Frames observing the object and associated localcuboid index in frame
    std::unordered_map< size_t, Frame::Ptr> mObservations_;
    Frame::Ptr *moRefF_;         // Reference Frame, first frame see this
    Frame::Ptr *mLatestF_;       // latest frame see this
    int nObs_;                   // num of frame observations

    // Bad flag
    bool mbBad_;

public:

    MapCube();
    MapCube(long id, SE3 pose, Vector3d scale) :
     id_(id), pose_(pose), scale_(scale), observed_times_(0) {}

    static MapCube::Ptr createMapCube();
};
#endif //SRC_MAPCUBE_H
