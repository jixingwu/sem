//
// Created by jixingwu on 2020/3/16.
//

#ifndef SRC_TRACKING_H
#define SRC_TRACKING_H

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
#include "Object_landmark.h"

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

typedef Eigen::Matrix<double, 3, 8> Matrix38d;
typedef Eigen::Matrix<double, 4, 2> Matrix42d;
//typedef Eigen::Matrix<double, 9, 1> Vector9d;


class Tracking
{
private:

    ros::NodeHandle n;

    GraphMatching<TopoMetric_c, Node_c> graphmatching_h;
//    DataManager dataManager;

public:

    Tracking();
    ~Tracking();
    void inputImage(const cv::Mat& img);
    void inputImageMsg();// INPUT: img_buf
    void inputCamPose(Eigen::Matrix4d cam_pose);
    void inputFrameBboxes2trian(darknet_ros_msgs::BoundingBoxes frame_bboxes,
                                darknet_ros_msgs::BoundingBoxes frame_bboxes_next);
    void bboxes2CenterPpoints2f(darknet_ros_msgs::BoundingBoxes frame_bboxes, vector<cv::Point2f>& points);

    double computeError(Matrix42d keyframeCoor, Matrix42d frameCoor);

public:
    cv::Mat InitToGround;//    Eigen::Matrix4d cam_transToGround;
    Eigen::Matrix3d Kalib;
    detect_3d_cuboid *detect_cuboid_obj;
    double obj_det_2d_thre;
    ros::Time img_t;

    line_lbd_detect line_lbd_obj;
    std::vector<ObjectSet> frames_cuboid;

    Eigen::MatrixXd first_truth_frame_pose;
    g2o::SparseOptimizer graph;
    g2o::SE3Quat fixed_init_cam_pose_Twc;

    vector<object_landmark*> cube_pose_opti_history;
    vector<object_landmark*> cube_pose_raw_detected_history;
    vector<tracking_frame*> all_frames;
    g2o::VertexCuboid* vCube;

    int frame_index = 0;
    queue<darknet_ros_msgs::BoundingBoxes> frame_bboxes_buf;
//    darknet_ros_msgs::BoundingBoxes frame_bboxes;
//    darknet_ros_msgs::BoundingBoxes keyframe_bboxes;

    std::mutex m_buf;
    queue<cv::Mat> img_buf;

    ros::Time bboxes_t0;
    bool bboxes_t0_available = false;
    bool is_bboxes_t0_available() {return bboxes_t0_available;}

public:
    void Track();// main tracking function. input sensor dataset;
    void CreateNewKeyFrame(cv::Mat img, uint32_t imgID);
    void DetectCuboid(const cv::Mat& raw_image,  const darknet_ros_msgs::BoundingBoxes& frame_bboxes);
    void GenerateCuboid();
    void AssociateCuboid();
    bool MatchCuboid(darknet_ros_msgs::BoundingBoxes keyframe_bboxes, darknet_ros_msgs::BoundingBoxes frame_bboxes);//keyframe_bboxes, frame_bboxes;
    cv::Mat setImageFromMsg(const sensor_msgs::ImageConstPtr msg);
    visualization_msgs::MarkerArray cuboids_to_marker(cuboid* raw_cuboid, Vector3d rgbcolor);
    void cuboid_corner_to_marker(const Matrix38d& cube_corners, visualization_msgs::Marker& marker, int bodyOrfront);

    void frame_bboxes_callback(const darknet_ros_msgs::BoundingBoxes& msg);
    void detection_image_callback(const sensor_msgs::Image& msg);
    void left_image_callback(const sensor_msgs::Image& msg);

    bool is_frame_bboxes_empty(){return frame_bboxes_buf.empty();}


//    cv::Mat TrackMonocular(const cv::Mat &im, const double &timestamp, int msg_seq_id=-1);// System::

};

#endif //SRC_TRACKING_H
