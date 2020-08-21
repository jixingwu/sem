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
#include <queue>
#include <Eigen/Core>
#include <ros/ros.h>
#include <ros/console.h>// import ROS_DEBUG(), ROS_DEBUG_ONCE() etc.

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "darknet_ros_msgs/BoundingBoxes.h"
#include "darknet_ros_msgs/BoundingBox.h"

#include "detect_3d_cuboid/detect_3d_cuboid.h"
#include "detect_3d_cuboid/object_3d_util.h"
#include "line_lbd/line_lbd_allclass.h"

#include "GraphMatching.h"
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
#include "mapcube.h"

class VisualOdometry
{
public:
    typedef shared_ptr<VisualOdometry> Ptr ;
    enum VOState{
        INITIALIZING = -1,
        OK = 0,
        LOST
    };

    VOState             state_; // current VO status
    SemMap::Ptr         map_;   // map with all frames and map points



    Frame::Ptr          ref_;// = new Frame();   // reference key-frame
    Frame::Ptr          curr_;  // current frame

    vector<ObjectSet>                       cubes_curr_;    //cubes in current frame, 3D
    vector<darknet_ros_msgs::BoundingBoxes> bboxes_curr_;   // bboxes in current frame, 2D

    vector<pair<MapCube::Ptr, MapCube::Ptr>>                                            match_3dcubes_;         // matched 3d cuboids
    vector<pair<int, int>>                                                              match_3d_index_;        // matched 3d ref and curr cuboids index
    vector<pair<darknet_ros_msgs::BoundingBoxes, darknet_ros_msgs::BoundingBoxes>>      match_2dbboxes_;        // matched 2d bboxes
    vector<pair<int, int>>                                                              match_2d_index_;        // matched 2d ref and curr bboxes index

    queue<pair<double, map<int, pair<int, int>, pair<MapCube::Ptr, MapCube::Ptr>>>> MatchedCubeBuf;
    // （当前时间t, ref与curr匹配到的多个cubes(cube提取出的数量num_id，pair(这对cube在ref和curr中的index), pair(一对匹配到的cube))

    SE3 T_c_w_estimated_;
    int num_inliers_;
    int num_lost_ = 0;

    // parameters
    int max_num_lost_ = 10;

    // set cube class and queue parameter
    detect_3d_cuboid *detect_cuboid_obj;
    line_lbd_detect line_lbd_obj;
//    g2o::SE3Quat fixed_init_cam_pose_Twc;
//    vector<tracking_frame*> all_frames;
//    int frame_index = 0;
//    g2o::SparseOptimizer graph;

    Eigen::Matrix3d Kalib;
    SE3 InitToGround; // world2camera T_c_w

    GraphMatching<TopoMetric_c, Node_c> graphMatching_h;// 用来进行cube匹配构造相似度矩阵

public:
    VisualOdometry();
    ~VisualOdometry();

    bool addFrame( Frame::Ptr frame );   //add a new frame

public:
    // inner operation
    void inputImage(const cv::Mat image);
    void inputBboxes(const darknet_ros_msgs::BoundingBoxes);

    void generateCubeProposal();
    void addKeyFrame();

    void saveImage(cv::String dest, cv::Mat image, size_t id, std::string imageName);

    /* cubeMatching()
     * 功能：对ref和curr中提取到的cube进行匹配，输出一对匹配到的cube和对应的index
     *
     * 算法描述：ref中的bbox（被提取到cube的）通过pose约束投影到当前帧curr中，
     * 通过强先验（同类物体）寻找‘最近的bbox1’
     *
     * 验证：通过3D cube映射验证，约束有 pose_, scale_
     */
    void cubeMatching();
    void addMapCubes();

    bool checkKeyFrame();
    bool checkReceivedPose();
    bool isBboxesInImage(Vector4d v, cv::Mat image);

};

#endif //SRC_VISUAL_ODOMETRY_H
