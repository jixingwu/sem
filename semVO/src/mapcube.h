//
// Created by jixingwu on 2020/7/20.
//

#ifndef SRC_MAPCUBE_H
#define SRC_MAPCUBE_H
#include "camera.h"
#include "map.h"
#include "frame.h"
#include "detect_3d_cuboid/detect_3d_cuboid.h"
class Frame;
class MapCube : public cuboid
{
public:
    typedef shared_ptr<MapCube> Ptr;
    unsigned long id_;

    int             observed_times_;// being observed by feature matching algo.
    int             matched_times_;// being an inliner in pose estimation

    // MapObject
    // -------------for local MapObject --------------
    SE3 pose_;
    SE3 worldPose_;
    Vector3d scale_;
    cv::Rect bbox_2d_;                   // (int) yolo detected 2D bbox_2d x y w h
    Eigen::Vector4d bbox_vec_;           // center, width, height

    double meas_quality_;                // [0,1] the higher, the better
    Eigen::MatrixXi box_corners_2d_;     // 2*8 on image usually for local cuboids on reference frame.
    Eigen::MatrixXi edge_markers_;       // in order to plot 2d cuboids with 8 corners.
    cv::Rect bbox_2d_tight_;             // tighted 2d object, used to find points association.
    int object_id_in_localF_;            // object id in reference frame's local objects;

// Frames observing the object and associated localcuboid index in frame
    std::unordered_map<Frame::Ptr*, size_t> mObservations_;
    Frame::Ptr *moRefF_;         // Reference Frame, first frame see this
    Frame::Ptr *mLatestF_;       // latest frame see this
    int nObs_;                   // num of frame observations

    // Bad flag
    bool mbBad_;

public:

    MapCube();
    MapCube(long id, SE3 pose, Vector3d scale);

    static MapCube::Ptr createMapCube();

    // -------------for local MapObject --------------


};

//class MapObject
//{
//public:
//    MapObject(Map *pMap, bool update_index = false);
//};

#endif //SRC_MAPCUBE_H
