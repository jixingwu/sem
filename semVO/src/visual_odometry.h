//
// Created by jixingwu on 2020/7/21.
//

#ifndef SRC_VISUAL_ODOMETRY_H
#define SRC_VISUAL_ODOMETRY_H

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <iostream>
#include <mutex>
#include <vector>
#include <Eigen/Core>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "darknet_ros_msgs/BoundingBoxes.h"
#include "darknet_ros_msgs/BoundingBox.h"

#include "detect_3d_cuboid/detect_3d_cuboid.h"
#include "detect_3d_cuboid/object_3d_util.h"
#include "line_lbd/line_lbd_allclass.h"

#include "GraphMatching.h"
#include "DataManager.h"
#include "Frame.h"
#include "Converter.h"

#include "g2o_Object.h"
#include "Object_landmark.h"
#include "Thirdparty/g2o/g2o/types/se3quat.h"
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"

#include "camera.h"
#include "map.h"
#include "frame.h"

class VisualOdometry
{
public:
    typedef shared_ptr<VisualOdometry> Ptr ;
    enum VOState{
        INITIALIZING = -1,
        OK = 0,
        LOST
    };

    VOState state_;
    MapObject::Ptr map_;
    Frame::Ptr ref_;
    Frame::Ptr curr_;
    SE3 T_C_w_estimated_;
    int num_inliers_;
    int num_lost_;

    // set cube class and queue parameter
    detect_3d_cuboid *detect_cuboid_obj;
    line_lbd_detect line_lbd_obj;
    g2o::SE3Quat fixed_init_cam_pose_Twc;
    vector<tracking_frame*> all_frames;
    int frame_index = 0;
    g2o::SparseOptimizer graph;

    SE3 InitToGround;


public:
    VisualOdometry();
    ~VisualOdometry();

    bool addFrame( Frame::Ptr frame);   //add a new frame

public:
    // inner operation
    void inputImage(const cv::Mat image);
    void inputBboxes(const darknet_ros_msgs::BoundingBoxes);

    void setGenerateCubeParameter();
//    void generateCubeProposal(const cv::Mat& raw_image,  const darknet_ros_msgs::BoundingBoxes& frame_bboxes);
    void generateCubeProposal();
    void cubeProposalScoring();

    void addKeyFrame();

};

#endif //SRC_VISUAL_ODOMETRY_H
