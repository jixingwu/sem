//
// Created by jixingwu on 2020/7/21.
//

#include "visual_odometry.h"
#include "config.h"

typedef Eigen::Matrix<double, 7, 1> Vector7d;
typedef Eigen::Matrix<double, 5, 1> Vector5d;

//void Estimator::inputImage(double t, const cv::Mat &_img) {
//
//}
#define __DEBUG__(msg) msg;
VisualOdometry::VisualOdometry():
state_(INITIALIZING), ref_(nullptr), curr_(nullptr), map_(new MapObject), num_inliers_(0), num_lost_(0)
{
    // 创建vo对象后就把generate cube的参数设置完成
    // set initial camera pose wrt ground. by default camera parallel to ground, height=1.7 (kitti)
    Eigen::Quaterniond init_q; Eigen::Vector3d init_t;
    init_q.x() = Config::get<double>("init.qx");
    init_q.y() = Config::get<double>("init.qy");
    init_q.z() = Config::get<double>("init.qz");
    init_q.w() = Config::get<double>("init.qw");
    init_t.x() = Config::get<double>("init.x");
    init_t.y() = Config::get<double>("init.y");
    init_t.z() = Config::get<double>("init.z");

    detect_cuboid_obj = new detect_3d_cuboid();
    detect_cuboid_obj->whether_plot_detail_images = false;
    detect_cuboid_obj->whether_plot_final_images = false;
    detect_cuboid_obj->print_details = false;
    detect_cuboid_obj->whether_sample_bbox_height = false;
    detect_cuboid_obj->nominal_skew_ratio = 2;
    detect_cuboid_obj->whether_save_final_images = true;

    line_lbd_obj.use_LSD = true;
    line_lbd_obj.line_length_thres = 15;

    // graph optimization
    g2o::BlockSolverX::LinearSolverType* linearSolver;
    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();
    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    graph.setAlgorithm(solver);    graph.setVerbose(false);

    Eigen::Quaterniond init_cam_pose_q(init_q.w(), init_q.x(), init_q.y(), init_q.z());
    Eigen::Vector3d init_cam_pose_v(init_t.x(), init_t.y(), init_t.z());

    Vector7d cam_pose; cam_pose<< init_t.x(), init_t.y(), init_t.z(), init_q.x(), init_q.y(), init_q.z(), init_q.w();//0 0 1.7000000 -0.7071 0 0 0.7071
    fixed_init_cam_pose_Twc = g2o::SE3Quat(cam_pose);
    __DEBUG__(cout<< TermColor::iRED()<< "fixed_init_cam_pose_Twc: "  << fixed_init_cam_pose_Twc <<TermColor::RESET() << endl;)

    InitToGround = SE3(init_q, init_t);
}
VisualOdometry::~VisualOdometry() {}

void VisualOdometry::addKeyFrame() {

}

bool VisualOdometry::addFrame(Frame::Ptr frame)
{
    switch (state_) {
        case INITIALIZING:
        {
            state_ = OK;
            curr_ = ref_ = frame;
            // extract features from first frame and add them into mat
//            generateCubeProposal(frame->rgb_image_, frame->bboxes_);
            generateCubeProposal();// OUTPUT: frame_cuboids as a 'vector<ObjectSet>' type
//            cubeProposalScoring(); // update frame_cuboids
            addKeyFrame();
            break;
        }
        case OK:
        {
            curr_ = frame;
            curr_->T_c_w_ = ref_->T_c_w_;//讲当前帧pose设置为上一帧pose
            generateCubeProposal();
//            cubeProposalScoring();
            // TODO:估计相机pose, 将vins的camera pose与initTransToWorld相乘；// 无需估计vins会pub
            // TODO: check camera pose
            // if true, 更新当前帧的pose. if else state_ = LOST;
        }
        case LOST:
        {

        }
    }
    return true;
}

#define __DEBUG__(msg) msg;
void VisualOdometry::setGenerateCubeParameter()
{}

void VisualOdometry::generateCubeProposal()
{
    const cv::Mat &raw_rgb_image = curr_->rgb_image_;
    const darknet_ros_msgs::BoundingBoxes frame_bboxes = *curr_->bboxes_;

    g2o::SE3Quat curr_cam_pose_Twc;
    Eigen:Vector3d pan_init;
    pan_init << 0, 0, 0;
    g2o::SE3Quat odom_val(Quaterniond(1,0,0,0), pan_init); // from previous frame to current frame

    Eigen::Matrix3d Kalib;
    Kalib <<ref_->camera_->fx_,  0,  ref_->camera_->cx_,   // for KITTI cabinet data.
            0,  ref_->camera_->fy_, ref_->camera_->cy_,
            0,      0,     1;// fx fy cx cy

    detect_cuboid_obj->set_calibration(Kalib);

//    if(frame_index == 0)
//        curr_cam_pose_Twc = fixed_init_cam_pose_Twc;
//    else{
//        g2o::SE3Quat prev_pose_Tcw = all_frames[frame_index-1]->cam_pose_Tcw;
//        if (frame_index>1)  // from third frame, use constant motion model to initialize camera.
//        {
//            g2o::SE3Quat prev_prev_pose_Tcw = all_frames[frame_index-2]->cam_pose_Tcw;
//            odom_val = prev_pose_Tcw*prev_prev_pose_Tcw.inverse();
//        }
//        curr_cam_pose_Twc = (odom_val*prev_pose_Tcw).inverse();
//    }

    tracking_frame* currframe = new tracking_frame();
    currframe->frame_img = frame_index;
//    all_frames[frame_index] = currframe;
    all_frames.push_back(currframe);

    bool has_detected_cuboid = false;
    g2o::cuboid cube_local_meas; double proposal_error;

    // edge detection
    cv::Mat all_lines_mat;
    line_lbd_obj.detect_filter_lines(raw_rgb_image, all_lines_mat);
    Eigen::MatrixXd all_lines_raw(all_lines_mat.rows, 4);
    for (int rr = 0; rr < all_lines_mat.rows; ++rr) {
        for (int cc = 0; cc < 4; ++cc) {
            all_lines_raw(rr,cc) = all_lines_mat.at<float>(rr,cc);
        }
    }

    assert( all_lines_raw.size() != 0 && "Eigen all_lines_raw is empty");

    std::vector<Vector4d> good_object_bbox;
//    frame_bboxes = frame_bboxes_buf.front();
//    frame_bboxes_buf.pop();
    __DEBUG__(
            cout<< TermColor::iBLUE() << "[feature_tracker/DetectCuboid] frame_bboxes size: "<< frame_bboxes.bounding_boxes.size() << TermColor::RESET() << endl;
    )
    const int bboxes_length = frame_bboxes.bounding_boxes.size();
    Eigen::Matrix<double, Dynamic, Dynamic> all_object_coor(bboxes_length, 5);

    int img_width = raw_rgb_image.cols;
    int img_length = raw_rgb_image.rows;

    // remove some 2d bboxes too close to boundary
    int boundary_threshold = 20;
    int count = 0;
    for (auto & bounding_boxes : frame_bboxes.bounding_boxes)
    {
        int xmin = bounding_boxes.xmin;
        int ymin = bounding_boxes.ymin;
        int xmax = bounding_boxes.xmax;
        int ymax = bounding_boxes.ymax;
        double prob = bounding_boxes.probability;
        int width = xmax - xmin;
        int length = ymax - ymin;

        Vector5d object_bbox;
        object_bbox << xmin, ymin, width, length, prob;//01234

        if((object_bbox(0) < boundary_threshold) || (object_bbox(0) + object_bbox(2) > img_width - boundary_threshold)
           || (object_bbox(1) < boundary_threshold) || (object_bbox(1) + object_bbox(3) > img_length - boundary_threshold))
            continue;
//        good_object_bbox.push_back(object_bbox);
        for (int cc = 0; cc < 5; ++cc) {
            all_object_coor(count, cc) = object_bbox(cc);
        }
        count++;
    }

//    all_object_coor.resize(5,5);

    assert( all_object_coor.size() != 0 && " all_object_corr is empty");

    SE3 frame_pose_to_init = curr_->T_c_w_;
    SE3 frame_pose_to_ground = frame_pose_to_init * InitToGround;
    Matrix4d transToWorld = frame_pose_to_ground.matrix();

    __DEBUG__( cout<< "frame_pose_to_ground: "<< frame_pose_to_ground<<endl;
                cout<<"transToWorld: "<< transToWorld<<endl;)
//    detect_cuboid_obj->whether_sample_cam_roll_pitch = (frame_index!=0);
//    if(detect_cuboid_obj->whether_sample_cam_roll_pitch)
//        transToWorld = fixed_init_cam_pose_Twc.to_homogeneous_matrix();
//    else
//        transToWorld = curr_cam_pose_Twc.to_homogeneous_matrix();


    assert(all_object_coor.size() != 0 && "bboxes_buf is empty");

    __DEBUG__(
            cout<< TermColor::iRED()
                << "[feature_tracker/DetectCuboid] raw_rgb_image.width: " << raw_rgb_image.cols << endl
                << "all_object_coor.size: "<<all_object_coor.size()<< endl
                << "all_lines_raw.size: " << all_lines_raw.size()<< TermColor::RESET() << endl;

    )
    detect_cuboid_obj->detect_cuboid(raw_rgb_image, transToWorld, all_object_coor, all_lines_raw, curr_->frame_cuboids_);
//    currframe->cuboids_2d_img = detect_cuboid_obj->cuboids_2d_img;

    __DEBUG__(
            cout<< TermColor::iBLUE()<<"[tracking/DetectCuboid()] detected frames_cuboids size: " << curr_->frame_cuboids_.size()<< TermColor::RESET() <<endl;
            cvNamedWindow("raw_rgb_image");
            cvMoveWindow("raw_rgb_image", 20, 300);
            cv::imshow("raw_rgb_image",raw_rgb_image);

            cvNamedWindow("detect_cuboid_obj->cuboids_2d_img");
            cvMoveWindow("detect_cuboid_obj->cuboids_2d_img", 20, 300);
            cv::imshow("detect_cuboid_obj->cuboids_2d_img",detect_cuboid_obj->cuboids_2d_img);
            cv::waitKey(0);
    )
//    frame_index++;
}

void VisualOdometry::cubeProposalScoring()
{
//    vector<ObjectSet>  frame_cuboids = curr_->frame_cuboids_;
//    for(int object_id = 0; object_id < frame_cuboids.size(); object_id++)
//    {
//        if(frame_cuboids[object_id].empty()) continue;
//        Vector3d raw_object_errors = Vector3d(frame_cuboids[object_id][0]->edge_distance_error);
//
//    }
}