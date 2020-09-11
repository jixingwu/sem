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

#include "src/detect_3d_cuboid/detect_3d_cuboid.h"
#include "src/detect_3d_cuboid/object_3d_util.h"
#include "src/line_lbd/line_lbd_allclass.h"

#include "GraphMatching.h"
//#include "Frame.h"
//#include "Converter.h"
//
//#include "g2o_Object.h"
//#include "Object_landmark.h"
#include "Thirdparty/g2o/g2o/types/se3quat.h"
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"


#include "src/camera_models/camera.h"
#include "map.h"
#include "frame.h"
#include "mapcube.h"
#include "src/estimator/estimator.h"

typedef Eigen::Matrix<double, 9, 1> Vector9d;
typedef Eigen::Matrix<double, 9, 9> Matrix9d;
typedef Eigen::Matrix<double, 3, 8> Matrix38d;
typedef Eigen::Matrix<double, 10, 1> Vector10d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;

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
    queue<Frame::Ptr>   frameBuf;

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

    Eigen::Matrix3d Kalib;
    SE3 InitToGround; // world2camera T_c_w

    GraphMatching<TopoMetric_c, Node_c> graphMatching_h;// 用来进行cube匹配构造相似度矩阵
    bool isCubeMatching = false;
    Eigen::VectorXi retmatch, retmatchinverse; //相似度矩阵匹配结果及其转置

public:
    VisualOdometry();
    ~VisualOdometry();

    bool addFrame( Frame::Ptr frame );   //add a new frame

    // inner operation
    void inputImage(const cv::Mat image);
    void inputBboxes(const darknet_ros_msgs::BoundingBoxes);

    void generateCubeProposal();
    /*
     * addKeyFrame()
     * 仅将第一帧的cube放入到地图中，其他帧的cube通过optimizeMap() by inserted
     */
    void addKeyFrame();

    void saveImage(cv::String dest, cv::Mat image, size_t id, std::string imageName);

    /* cubeMatching()
     * 功能：对ref和curr中提取到的cube进行匹配，输出一对匹配到的cube和对应的index
     * 描述：ref中的bbox（被提取到cube的）通过pose约束投影到当前帧curr中，通过强先验（同类物体）寻找‘最近的bbox1’
     * 输出：retmatch的size是ref topo点数量，内容是匹配到curr中的index
     * 验证：通过3D cube映射验证，约束有 pose_, scale_
     */
    void cubeMatching();
    /* trackCubes()
     * 描述：将ref_ and curr_相同的cube的id相同，并且id是从0顺序递增
     * 输入：两帧ref_ curr_和bbox的匹配结果retmatch
     * 输出：修改了curr.local_cuboids_.id，从0递增
     * 代码逻辑：如果是第一帧，那么设置cube的id从0开始递增；找retmatch中ref与curr匹配到的id，将curr对应设置为ref的id
     * 对于没有匹配到的作为新的cube新增其id值。
     */
    void trackCubes();

    Estimator estimator;
    void optimizeCube();

    void addMapCubes();

    bool checkKeyFrame();
    bool checkReceivedPose();
    bool isBboxesInImage(Vector4d v, cv::Mat image);

};

#endif //SRC_VISUAL_ODOMETRY_H
