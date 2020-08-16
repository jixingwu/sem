//
// Created by jixingwu on 2020/7/21.
//

#include "visual_odometry.h"
#include "config.h"
#include "Converter.h"

typedef Eigen::Matrix<double, 7, 1> Vector7d;
typedef Eigen::Matrix<double, 5, 1> Vector5d;
typedef Eigen::Matrix<int, 5, 1> Vector5i;

//void Estimator::inputImage(double t, const cv::Mat &_img) {
//
//}
#define __DEBUG__(msg);
#define __TRACIN_DEBUG__(msg) msg;
VisualOdometry::VisualOdometry():
        state_(INITIALIZING), ref_(nullptr), curr_(nullptr), map_(new SemMap), num_inliers_(0), num_lost_(0)
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
//    g2o::BlockSolverX::LinearSolverType* linearSolver;
//    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();
//    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);
//    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
//    graph.setAlgorithm(solver);    graph.setVerbose(false);
//
//    Eigen::Quaterniond init_cam_pose_q(init_q.w(), init_q.x(), init_q.y(), init_q.z());
//    Eigen::Vector3d init_cam_pose_v(init_t.x(), init_t.y(), init_t.z());
//
//    Vector7d cam_pose; cam_pose<< init_t.x(), init_t.y(), init_t.z(), init_q.x(), init_q.y(), init_q.z(), init_q.w();//0 0 1.7000000 -0.7071 0 0 0.7071
//    fixed_init_cam_pose_Twc = g2o::SE3Quat(cam_pose);
//    __DEBUG__(cout<< TermColor::iRED()<< "fixed_init_cam_pose_Twc: "  << fixed_init_cam_pose_Twc <<TermColor::RESET() << endl;)

    init_q.x() = -0.7071;
    init_q.y() = init_q.z() = 0;
    init_q.w() = 0.7071;
    init_t.x() = init_t.y() = 0;
    init_t.z() = 1.7;

    InitToGround = SE3(init_q, init_t);
    __DEBUG__(cout<< "InitToGround: "<< InitToGround<< endl;)
}
VisualOdometry::~VisualOdometry() {}

void VisualOdometry::addKeyFrame()
{
    if(map_->keyframes_.empty())
    {
        // first key-frame, add all cube object into map
        for (size_t ii = 0; ii < curr_->frame_cuboids_.size(); ++ii)
        {
            MapCube::Ptr map_cube = MapCube::createMapCube();
            map_->insertMapCube(map_cube);
        }
    }

}

void VisualOdometry::cubeMatching()
{

}

bool VisualOdometry::addFrame(Frame::Ptr frame)
{
    switch (state_) {
        case INITIALIZING:
        {
            state_ = OK;
            curr_ = ref_ = frame;
            // OUTPUT: frame_cuboids as a 'vector<ObjectSet>' type
            // equal to extractKeyPoints() and computeDescriptors()
            generateCubeProposal();
            addKeyFrame();// the first frame is a key-frame
            break;
        }
        case OK:
        {
            curr_ = frame;
            curr_->T_c_w_ = ref_->T_c_w_;//讲当前帧pose设置为上一帧pose
            generateCubeProposal();
            cubeMatching();// equal to featureMatching();
            // TODO:估计相机pose poseEstimationPnP() to 将vins的camera pose与initTransToWorld相乘；
            // if true, 更新当前帧的pose. if else state_ = LOST;
            if(checkReceivedPose())
            {

            }else{ // bad estimation due to various reasons
                num_lost_++;
                if(num_lost_ > max_num_lost_)
                {
                    state_ = LOST;
                }
                return false;
            }
            break;
        }
        case LOST:
        {
            ROS_ERROR("vo has lost.");
            ros::shutdown();
            break;
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

    Kalib.setIdentity();
//    Kalib(0,0) = ref_->camera_->fx_;
//    Kalib(0,2) = ref_->camera_->cx_;
//    Kalib(1,1) = ref_->camera_->fy_;
//    Kalib(1,2) = ref_->camera_->cy_;
    Kalib(0,0) = 718.856;
    Kalib(1,1) = 718.856;
    Kalib(0,2) = 607.1928;
    Kalib(1,2) = 185.2157;
    __DEBUG__(cout<<"Kalib: "<< Kalib << endl;)
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
    int boundary_threshold = 40;
    int count = 0;
    for (auto & bounding_boxes : frame_bboxes.bounding_boxes)
    {
        double xmin = bounding_boxes.xmin;
        double ymin = bounding_boxes.ymin;
        double xmax = bounding_boxes.xmax;
        double ymax = bounding_boxes.ymax;
        double prob = bounding_boxes.probability;
        double width = xmax - xmin;
        double length = ymax - ymin;

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

    Sophus::SE3 frame_pose_to_init = curr_->T_c_w_;
    Sophus::SE3 frame_pose_to_ground = frame_pose_to_init * InitToGround;
    Matrix4d transToWorld = frame_pose_to_ground.matrix();

    __DEBUG__( cout<< "frame_pose_to_ground_se3(): "<< frame_pose_to_ground.log()<<endl;
                cout<<"transToWorld: "<< transToWorld<<endl;)
//    detect_cuboid_obj->whether_sample_cam_roll_pitch = (frame_index!=0);
//    if(detect_cuboid_obj->whether_sample_cam_roll_pitch)
//        transToWorld = fixed_init_cam_pose_Twc.to_homogeneous_matrix();
//    else
//        transToWorld = curr_cam_pose_Twc.to_homogeneous_matrix();


    assert(all_object_coor.size() != 0 && "bboxes_buf is empty");

    __DEBUG__(
//            cout<< TermColor::iRED()
//                << "[feature_tracker/DetectCuboid] raw_rgb_image.width: " << raw_rgb_image.cols << endl
//               << "all_object_coor.size: "<<all_object_coor.size()<< endl
//               << "all_lines_raw.size: " << all_lines_raw.size()<< TermColor::RESET() << endl;

    )
    vector<ObjectSet> all_obj_cubes;
    detect_cuboid_obj->detect_cuboid(raw_rgb_image, transToWorld, all_object_coor, all_lines_raw, all_obj_cubes);
//    currframe->cuboids_2d_img = detect_cuboid_obj->cuboids_2d_img;


    __DEBUG__(
            cout<< TermColor::iBLUE()<<"[tracking/DetectCuboid()] detected all_obj_cubes size: " << all_obj_cubes.size()<< TermColor::RESET() <<endl;
            cvNamedWindow("detect_cuboid_obj->cuboids_2d_img");
            cvMoveWindow("detect_cuboid_obj->cuboids_2d_img", 20, 300);
            cv::imshow("detect_cuboid_obj->cuboids_2d_img",detect_cuboid_obj->cuboids_2d_img);
            cv::waitKey(2);

//            cv::String dest_ = "/home/jixingwu/catkin_ws/src/sem/semVO/image_results/";
//            cv::String savedfilename_;
//            savedfilename_ = dest_ + std::to_string(curr_->id_) + ".jpg";
//            cout<<"curr_.id_ = "<<curr_->id_<<endl;
//            cout<<"savedfilename = "<<savedfilename_<<endl;
//            cv::imwrite(savedfilename_, detect_cuboid_obj->cuboids_2d_img);
    )

    // cp and analyze results.
    curr_->local_cuboids_.clear();
    // previous frame_pose_to_init = curr_->T_c_w_;
    // transToWorld_se3;
    for (int ii = 0; ii < (int)all_obj_cubes.size(); ++ii)
    {
        if(!all_obj_cubes[ii].empty())
        {
            cuboid *raw_cuboid = all_obj_cubes[ii][0];
            Vector9d cube_pose;// Vector3d t, roll yall pitch, Vector3d scale
            cube_pose<< raw_cuboid->pos[0], raw_cuboid->pos[1], raw_cuboid->pos[2], 0, 0, raw_cuboid->rotY,
                raw_cuboid->scale[0], raw_cuboid->scale[1], raw_cuboid->scale[2];
            Eigen::AngleAxisd rollAngle(AngleAxisd(cube_pose[3], Vector3d::UnitX()));
            Eigen::AngleAxisd pitchAngle(AngleAxisd(cube_pose[4], Vector3d::UnitY()));
            Eigen::AngleAxisd yawAngle(AngleAxisd(cube_pose[5], Vector3d::UnitZ()));
            Eigen::Quaterniond pose_qua = rollAngle * pitchAngle * yawAngle;
            Sophus::SE3 pose = Sophus::SE3(pose_qua, cube_pose.head(3));
            Vector3d scale = cube_pose.tail(3);

            // measurement in local camera frame! important
            MapCube *newcuboid = new MapCube();
            newcuboid->pose_ = pose;
            newcuboid->scale_ = scale;
            newcuboid->bbox_2d_ = cv::Rect(raw_cuboid->rect_detect_2d[0], raw_cuboid->rect_detect_2d[1], raw_cuboid->rect_detect_2d[2], raw_cuboid->rect_detect_2d[3]);
            newcuboid->bbox_vec_ = Vector4d((double)newcuboid->bbox_2d_.x + (double)newcuboid->bbox_2d_.width/2, (double)newcuboid->bbox_2d_.y + (double) newcuboid->bbox_2d_.height/2,
                                           (double)newcuboid->bbox_2d_.width, (double)newcuboid->bbox_2d_.height);
            newcuboid->box_corners_2d_ = raw_cuboid->box_corners_2d;
            newcuboid->bbox_2d_tight_ = cv::Rect(raw_cuboid->rect_detect_2d[0] + raw_cuboid->rect_detect_2d[2] / 10.0,
                                                raw_cuboid->rect_detect_2d[1] + raw_cuboid->rect_detect_2d[3] / 10.0,
                                                raw_cuboid->rect_detect_2d[2] * 0.8, raw_cuboid->rect_detect_2d[3] * 0.8);
            get_cuboid_draw_edge_markers(newcuboid->edge_markers_, raw_cuboid->box_config_type, false);
            newcuboid->moRefF_ = curr_;
            newcuboid->object_id_in_localF_ = curr_->local_cuboids_.size();
            newcuboid->worldPose_ = frame_pose_to_init;

            double obj_cam_dist = std::min(std::max(newcuboid->pose_.translation()(2), 10.0), 30.0); // cut into [a,b]
            double obj_means_quality = (60.0 - obj_cam_dist) / 40.0;
            newcuboid->meas_quality_ = obj_means_quality;

            if(newcuboid->meas_quality_ < 0.1)
                ROS_WARN_STREAM("Abnormal measure quality!!: " << newcuboid->meas_quality_);
            curr_->local_cuboids_.push_back(newcuboid);
        }
    }

}

