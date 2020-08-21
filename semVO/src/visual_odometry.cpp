//
// Created by jixingwu on 2020/7/21.
//

#include "visual_odometry.h"
#include "config.h"
#include "Converter.h"
#include "include/TermColor.h"

#include "GraphMatching.h"//用作bbox的匹配

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
//            addKeyFrame();// the first frame is a key-frame
            break;
        }
        case OK:
        {
            curr_ = frame;// 当前帧更新
            generateCubeProposal();
            cubeMatching();// equal to featureMatching();
            // TODO:估计相机pose poseEstimationPnP() to 将vins的camera pose与initTransToWorld相乘；
//             if true, 更新当前帧的pose. if else state_ = LOST;
//            if(checkReceivedPose())
//            {
//
//            }
//            else
//            { // bad estimation due to various reasons
//                num_lost_++;
//                if(num_lost_ > max_num_lost_)
//                {
//                    state_ = LOST;
//                }
//                return false;
//            }
            ref_ = curr_;
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


bool VisualOdometry::isBboxesInImage(Vector4d v, cv::Mat image)
{
    int img_width = image.cols;
    int img_height = image.rows;
    if(v(0)>0 && v(1)>0 && v(2)<img_width && v(3)<img_height)
        return true;
    else
        return false;

}
#define __DEBUG__(msg) ;
#define __DEBUG_GENERATE__(msg) msg;
void VisualOdometry::generateCubeProposal()
{
    __DEBUG_GENERATE__(cout<< TermColor::iBLUE() <<"starting detecting cubes ..."<<endl;)
    const cv::Mat &raw_rgb_image = curr_->rgb_image_;
    const darknet_ros_msgs::BoundingBoxes frame_bboxes = *curr_->bboxes_;

    Kalib.setIdentity();
//    Kalib(0,0) = ref_->camera_->fx_;
//    Kalib(0,2) = ref_->camera_->cx_;
//    Kalib(1,1) = ref_->camera_->fy_;
//    Kalib(1,2) = ref_->camera_->cy_;
    Kalib(0,0) = 718.856;//fx
    Kalib(1,1) = 718.856;//fy
    Kalib(0,2) = 607.1928;//cx
    Kalib(1,2) = 185.2157;//cy
    detect_cuboid_obj->set_calibration(Kalib);

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

    __DEBUG__(
            cout<< "[feature_tracker/DetectCuboid] frame_bboxes size: "<< frame_bboxes.bounding_boxes.size() << endl;
    )
    const int bboxes_length = frame_bboxes.bounding_boxes.size();
    Eigen::Matrix<double, Dynamic, Dynamic> all_object_coor(bboxes_length, 5);

    int img_width = raw_rgb_image.cols;
    int img_length = raw_rgb_image.rows;

    // remove some 2d bboxes too close to boundary
    int boundary_threshold = 20;
    int count = 0;
    vector<string> v_class;
    for (auto & bounding_boxes : frame_bboxes.bounding_boxes)
    {
        double xmin = bounding_boxes.xmin;
        double ymin = bounding_boxes.ymin;
        double xmax = bounding_boxes.xmax;
        double ymax = bounding_boxes.ymax;
        double prob = bounding_boxes.probability;
        double width = xmax - xmin;
        double length = ymax - ymin;
        string object_class = bounding_boxes.Class;

        Vector5d object_bbox;
        object_bbox << xmin, ymin, width, length, prob;//01234

        v_class.push_back(object_class);

        if((object_bbox(0) < boundary_threshold) || (object_bbox(0) + object_bbox(2) > img_width - boundary_threshold)
           || (object_bbox(1) < boundary_threshold) || (object_bbox(1) + object_bbox(3) > img_length - boundary_threshold))
            continue;
//        if(!isBboxesInImage(Vector4d(xmin, ymin, width, length), raw_rgb_image))
//            continue;
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


    assert(all_object_coor.size() != 0 && "bboxes_buf is empty");

    vector<ObjectSet> all_obj_cubes;
    detect_cuboid_obj->detect_cuboid(raw_rgb_image, transToWorld, all_object_coor, v_class, all_lines_raw, all_obj_cubes);

    __DEBUG__(
            cout<<"[tracking/DetectCuboid()] detected all_obj_cubes size: " << all_obj_cubes.size()<<endl;
            cvNamedWindow("detect_cuboid_obj->cuboids_2d_img");
            cvMoveWindow("detect_cuboid_obj->cuboids_2d_img", 20, 300);
            cv::imshow("detect_cuboid_obj->cuboids_2d_img",detect_cuboid_obj->cuboids_2d_img);
            cv::waitKey(2);

            cv::String dest_ = "/home/jixingwu/catkin_ws/src/sem/semVO/image_results/";
            cv::String savedfilename_;
            char frame_index[256];
            sprintf(frame_index,"%06lu", curr_->id_);
            savedfilename_ = dest_ + frame_index + ".jpg";
            cout<<"curr_.id_ = "<<curr_->id_<<"\t frame_index = "<<frame_index<<endl;
            cout<<"savedfilename = "<<savedfilename_<<endl;
            cv::imwrite(savedfilename_, detect_cuboid_obj->cuboids_2d_img);
    )

    // cp and analyze results.对于每个检测到的raw cube，将其转换为可添加到SemMap中的类型——MapCube
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
            newcuboid->object_class = raw_cuboid->object_class;
            newcuboid->bbox_2d_ = cv::Rect(raw_cuboid->rect_detect_2d[0], raw_cuboid->rect_detect_2d[1], raw_cuboid->rect_detect_2d[2], raw_cuboid->rect_detect_2d[3]);
            newcuboid->bbox_vec_ = Vector4d((double)newcuboid->bbox_2d_.x, (double)newcuboid->bbox_2d_.y,
                                           (double)newcuboid->bbox_2d_.width, (double)newcuboid->bbox_2d_.height);// xmin, ymin, width, height
            newcuboid->box_corners_2d_ = raw_cuboid->box_corners_2d;
            newcuboid->bbox_2d_tight_ = cv::Rect(raw_cuboid->rect_detect_2d[0] + raw_cuboid->rect_detect_2d[2] / 10.0,
                                                raw_cuboid->rect_detect_2d[1] + raw_cuboid->rect_detect_2d[3] / 10.0,
                                                raw_cuboid->rect_detect_2d[2] * 0.8, raw_cuboid->rect_detect_2d[3] * 0.8);
            get_cuboid_draw_edge_markers(newcuboid->edge_markers_, raw_cuboid->box_config_type, false);
            newcuboid->moRefF_ = &curr_;
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

    __DEBUG__(cout<<"curr_.local_cuboids_.size(): "<<curr_->local_cuboids_.size()<<endl;)
    __DEBUG_GENERATE__(cout<<"ending detection ..."<<TermColor::RESET()<<endl;)

}
#define __DEBUG_MATCH__(msg) msg;
void VisualOdometry::cubeMatching()
{
    __DEBUG_MATCH__(cout<<TermColor::iRED()<<"staring matching cubes ..."<<endl;)
    __DEBUG_MATCH__(cout<<"curr_ frame id: "<< curr_->id_<<endl;)

    int M = ref_->local_cuboids_.size(), N = curr_->local_cuboids_.size();
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> simMatrix;
    simMatrix.resize(M, N);
//    simMatrix.Constant(-1);

    //ref_ 去匹配 curr_, 每个M去找N
    __DEBUG_MATCH__(cout<<"M: "<<M<<"\t N: "<<N<<endl;)
    for (int simMatrixM = 0; simMatrixM < M; ++simMatrixM)
    {
        Vector4d refBboxesXY = ref_->local_cuboids_[simMatrixM]->bbox_vec_;// xmin ymin  width height
        string refBboxesClass = ref_->local_cuboids_[simMatrixM]->object_class; // Class
        __DEBUG_MATCH__(cout<<"ref_ bboxes"<< refBboxesXY <<endl;)

        //TODO: 添加运动约束
        //// ref_中的bboxes的像素坐标系下的(xmin, ymin)，映射到curr_的世界坐标系下，得到refPixel2CurrWorld
        Vector2d refBboxesXYmin = refBboxesXY.head(2), refBboxesXYmax = refBboxesXY.head(2) + refBboxesXY.tail(2);
        __DEBUG_MATCH__(
                cout<<"refBboxesXYmin: "<<refBboxesXYmin<<"\n refBbboxesXYmax: "<<refBboxesXYmax<<endl;
                cout<<"ref_ T_c_w: "<<ref_->T_c_w_<<"\n curr_ T_c_w: "<<curr_->T_c_w_<<endl;
        )

        auto *camera_ = new Camera();
        Vector3d refPixel2CurrWorldXYmin = camera_->pixel2world(refBboxesXYmin, ref_->T_c_w_);
        Vector2d currWorld2CurrPixelXYmin = camera_->world2pixel(refPixel2CurrWorldXYmin, curr_->T_c_w_);
        Vector3d refPixel2CurrWorldXYmax = camera_->pixel2world(refBboxesXYmax, ref_->T_c_w_);
        Vector2d currWorld2CurrPixelXYmax = camera_->world2pixel(refPixel2CurrWorldXYmax, curr_->T_c_w_);

        __DEBUG_MATCH__(
                cout<<"refBboxesXY(xmin, ymin, width, height) in ref frame: "<<refBboxesXY<<endl;
        )

        Vector4d _currBboxesXY;
        Vector2d _currBboxesWH = currWorld2CurrPixelXYmax - currWorld2CurrPixelXYmin;
        _currBboxesXY = Vector4d(currWorld2CurrPixelXYmin(0),currWorld2CurrPixelXYmin(1),
                                 _currBboxesWH(0), _currBboxesWH(1));
        __DEBUG_MATCH__(
                cout<<" _currBboxesXY(xmin, ymin, width, height) in curr frame: "<<_currBboxesXY<<endl;
                )
        __DEBUG_MATCH__(
                cvNamedWindow("ref image");
                cvMoveWindow("ref image", 20, 300);
                cv::rectangle(ref_->rgb_image_, ref_->local_cuboids_[simMatrixM]->bbox_2d_,
                              cv::Scalar(255,0,0), 5, cv::LINE_8, 0);
//                cv::imshow("ref image", ref_->rgb_image_);
//                cv::waitKey(300);
                saveImage("/home/jixingwu/catkin_ws/src/sem/semVO/image/0/",ref_->rgb_image_, simMatrixM, "ref image");
                )
        // TODO: _currBboxesXY match with currBboxesXY
        //// @param _currBboxesXY    ref帧中的bboxes映射到curr帧中的参数
        //// @param currBboxesXY    curr帧中的每个bboxes
        for (int simMatrixN = 0; simMatrixN < N; ++simMatrixN)
        {
            __DEBUG_MATCH__(
                    cvNamedWindow("curr_ image");
                    cvMoveWindow("curr_ image", 20, 300);
//                    cv::rectangle(curr_->rgb_image_, curr_->local_cuboids_[simMatrixN]->bbox_2d_,
//                                  cv::Scalar(255,0,0), 1, cv::LINE_8, 0);
                    cv::Rect _currRect(_currBboxesXY(0), _currBboxesXY(1), _currBboxesXY(2), _currBboxesXY(3));
                    cv::rectangle(curr_->rgb_image_, _currRect, cv::Scalar(0,255,0), 5, cv::LINE_8, 0);
//                    cv::imshow("curr_ image", ref_->rgb_image_);
//                    cv::waitKey(300);
                    saveImage("/home/jixingwu/catkin_ws/src/sem/semVO/image/1/", curr_->rgb_image_, simMatrixN, "curr image" );
                    )

            Vector4d currBboxesXY = curr_->local_cuboids_[simMatrixN]->bbox_vec_; // ordinary
            string currBboxesClass = curr_->local_cuboids_[simMatrixN]->object_class; // Class

            //// check whether ref bboxes is in the image
            if(!isBboxesInImage(refBboxesXY, ref_->rgb_image_) || !isBboxesInImage(currBboxesXY, curr_->rgb_image_))
            {
                ROS_WARN("ref bboxes is not in the image");
                continue;
            }

            double error;
            if(ref_->local_cuboids_[simMatrixM]->object_class == curr_->local_cuboids_[simMatrixN]->object_class)
            {
                // TODO: xmin, ymin, width, height的范数，不一定在相同数量级内
                // 相对误差 = (_x - x)/x
                error = Vector4d(_currBboxesXY - currBboxesXY).cwiseQuotient(currBboxesXY).norm();//分量平方和的平方根
                simMatrix(M, N) = exp(-1 * error);
            }
            else
                simMatrix(M, N) = 0;
        }
    }
    Eigen::MatrixXd simMat_simhorn = graphMatching_h.sinkhorn(simMatrix);
    Eigen::VectorXi retmatch(graphMatching_h.prior_graph->num_of_nodes()), retmatchinverse(graphMatching_h.visited_graph->num_of_nodes());
    retmatch.setConstant(-1);
    retmatchinverse.setConstant(-1);

    // 构建出的相似度矩阵的匹配结果
    Eigen::MatrixXd sim_results;
    sim_results = graphMatching_h.getBestMatchFromSimMat(simMat_simhorn, retmatch, retmatchinverse);
    graphMatching_h.clear();

    __DEBUG_MATCH__(
            cout<<"上一帧与第"<<curr_->id_<<"帧bboxes的匹配结果矩阵sim_results: \n"<<sim_results<<endl;
            cout<<"retmatch: "<<retmatch<<endl;
            cout<<TermColor::RESET()<<endl;)



}

void VisualOdometry::saveImage(cv::String dest, cv::Mat image, size_t id, std::string imageName)
{
//    cv::String dest_ = "/home/jixingwu/catkin_ws/src/sem/semVO/image_results/";
    cv::String savedfilename_;
    char frame_index[256];
    sprintf(frame_index,"%06lu", id);
    savedfilename_ = dest + frame_index + ".jpg";
    cout<<"save "<<imageName<<"image into "<<dest<<endl;
    cout<<"file name is: "<<savedfilename_<<endl;
    cv::imwrite(savedfilename_, image);
}
