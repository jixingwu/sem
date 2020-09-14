//
// Created by jixingwu on 2020/7/21.
//

#include <ros/console.h>
#include "visual_odometry.h"
#include "src/visualization/config.h"
#include "src/visualization/visualization.h"
#include "TermColor.h"
#include "src/detect_3d_cuboid/object_3d_util.h"

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

    init_q.x() = -0.7071;
    init_q.y() = init_q.z() = 0;
    init_q.w() = 0.7071;
    init_t.x() = init_t.y() = 0;
    init_t.z() = 1.7;

    InitToGround = SE3(init_q, init_t);
    __DEBUG__(cout<< "InitToGround: "<< InitToGround<< endl;)
}
VisualOdometry::~VisualOdometry() {}

#define __DEBUG_ADDKEYFRAME__(msg) ;
void VisualOdometry::addKeyFrame()
{
    __DEBUG_ADDKEYFRAME__(cout<<TermColor::iCYAN()<<"start add keyframe!"<<endl;)

    for(auto map_cube : curr_->local_cuboids_)
        map_->insertMapCube(*map_cube);

    map_->insertKeyFrame(curr_);
    __DEBUG_ADDKEYFRAME__(cout<<"map key-frame num: "<<map_->keyframes_.size()<<TermColor::RESET()<<endl;)

}

bool VisualOdometry::addFrame(Frame::Ptr frame)
{
    switch (state_) {
        case INITIALIZING:
        {
            state_ = OK;
            curr_ = frame;

            generateCubeProposal();
            trackCubes();
            addKeyFrame();// the first frame is a key-frame
            visualization();
            ref_ = curr_;
            break;
        }
        case OK:
        {
            curr_ = frame;// 当前帧更新
            generateCubeProposal();
            cubeMatching();// equal to featureMatching();
            trackCubes();// matching 后tracking得到的匹配，设置为相同的id
            optimizeCube();
            addKeyFrame();
            visualization();
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
#define __DEBUG__(msg);
#define __DEBUG_GENERATE__(msg);
void VisualOdometry::generateCubeProposal()
{
    ROS_DEBUG_NAMED("generateCubeProposal()", "%lu: staring detecing cubes", curr_->id_);

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

    __DEBUG_GENERATE__(
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

    __DEBUG_GENERATE__( cout<< "frame_pose_to_ground_se3(): "<< frame_pose_to_ground.log()<<endl;
                cout<<"transToWorld: "<< transToWorld<<endl;)


    assert(all_object_coor.size() != 0 && "bboxes_buf is empty");

    vector<ObjectSet> all_obj_cubes;
    detect_cuboid_obj->detect_cuboid(raw_rgb_image, transToWorld, all_object_coor, v_class, all_lines_raw, all_obj_cubes);

    if(all_object_coor.size() == 0)
        ROS_WARN("bbox is empty!");

    if(all_obj_cubes.empty())
        ROS_WARN("cubes is empty!");

    pubTrackImage(detect_cuboid_obj->cuboids_2d_img, curr_->time_stamp_);
    bool isSavedImage = true;
    if(isSavedImage)
    {

        cvNamedWindow("detect_cuboid_obj->cuboids_2d_img");
        cvMoveWindow("detect_cuboid_obj->cuboids_2d_img", 20, 300);
        cv::imshow("detect_cuboid_obj->cuboids_2d_img", detect_cuboid_obj->cuboids_2d_img);
        cv::waitKey(2);

        cv::String dest_ = "/home/jixingwu/catkin_ws/src/sem/semVO/image_results/";
        cv::String savedfilename_;
        char frame_index[256];
        sprintf(frame_index, "%06lu", curr_->id_);
        savedfilename_ = dest_ + frame_index + ".jpg";
//    cout<<"curr_.id_ = "<<curr_->id_<<endl;

        cv::imwrite(savedfilename_, detect_cuboid_obj->cuboids_2d_img);
    }


    // cp and analyze results.对于每个检测到的raw cube，将其转换为可添加到SemMap中的类型——MapCube
//    curr_->local_cuboids_.clear();
    // previous frame_pose_to_init = curr_->T_c_w_;
    // transToWorld_se3;

    cout<<"all_obj_cubes size: "<<all_obj_cubes.size()<<endl;
    for (int ii = 0; ii < (int)all_obj_cubes.size(); ++ii)
    {
        if(!all_obj_cubes[ii].empty())
        {
            cout<<"----";
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
            newcuboid->id_ = -1; // 未设置id的cube
            newcuboid->object_class = raw_cuboid->object_class;
            newcuboid->bbox_2d_ = cv::Rect(raw_cuboid->rect_detect_2d[0], raw_cuboid->rect_detect_2d[1], raw_cuboid->rect_detect_2d[2], raw_cuboid->rect_detect_2d[3]);
            newcuboid->bbox_vec_ = Vector4d((double)newcuboid->bbox_2d_.x, (double)newcuboid->bbox_2d_.y,
                                           (double)newcuboid->bbox_2d_.width, (double)newcuboid->bbox_2d_.height);// xmin, ymin, width, height
            newcuboid->box_corners_2d_ = raw_cuboid->box_corners_2d;
            newcuboid->box_config_type_ = raw_cuboid->box_config_type;
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

    ROS_DEBUG_NAMED("generateCubeProposal()", "detected cube num: %zu", curr_->local_cuboids_.size());

    cout<<"curr_.local_cuboids_.size(): "<<curr_->local_cuboids_.size()<<endl;
    ROS_DEBUG_NAMED("generateCubeProposal()", "ending detection!");
    for(auto &mapCube:curr_->local_cuboids_)
    {
        ROS_DEBUG_ONCE("frame id: %lu", curr_->id_);
        outputCuboids(mapCube);
    }


}
#define __DEBUG_MATCH__(msg) ;
void VisualOdometry::cubeMatching()
{
    ROS_DEBUG_NAMED("cubeMatching()", "%lu: staring matching cubes", curr_->id_);
    __DEBUG__(cout<<TermColor::iRED()<<"staring matching cubes ..."<<endl;)
    __DEBUG__(cout<<"curr_ frame id: "<< curr_->id_<<endl;)

    int M = ref_->local_cuboids_.size(), N = curr_->local_cuboids_.size();
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> simMatrix;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> simError;
    simMatrix.resize(M, N);
    simError.resize(M, N);
//    simMatrix.Constant(-1);
    if( min(M, N) <= 0 ){
        isCubeMatching = false;
        return;
    }

    //ref_ 去匹配 curr_, 每个M去找N
    __DEBUG__(cout<<"M: "<<M<<"\t N: "<<N<<endl;)
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
                cout<<"ref_ T_c_w: "<<ref_->T_c_w_.log()<<"\n curr_ T_c_w: "<<curr_->T_c_w_.log()<<endl;
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
        bool issavedImage = true;
        if(issavedImage)
        {
            cvNamedWindow("ref image");
            cvMoveWindow("ref image", 20, 300);
            cv::Mat img = ref_->rgb_image_;
            cv::rectangle(img, ref_->local_cuboids_[simMatrixM]->bbox_2d_,
                          cv::Scalar(255, 0, 0), 5, cv::LINE_8, 0);
            saveImage("/home/jixingwu/catkin_ws/src/sem/semVO/image/0/", img, ref_->id_, "ref image");

            cvNamedWindow("curr_ image");
            cvMoveWindow("curr_ image", 20, 300);
            cv::Rect _currRect(_currBboxesXY(0), _currBboxesXY(1), _currBboxesXY(2), _currBboxesXY(3));
            img = curr_->rgb_image_;
            cv::rectangle(img, _currRect, cv::Scalar(0, 255, 0), 5, cv::LINE_8, 0);
            saveImage("/home/jixingwu/catkin_ws/src/sem/semVO/image/1/", img, curr_->id_, "curr image");
        }

        // TODO: _currBboxesXY match with currBboxesXY
        //// @param _currBboxesXY    ref帧中的bboxes映射到curr帧中的参数
        //// @param currBboxesXY    curr帧中的每个bboxes
        for (int simMatrixN = 0; simMatrixN < N; ++simMatrixN)
        {
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
                Vector4d def = _currBboxesXY - currBboxesXY;
                error = def.cwiseQuotient(currBboxesXY).norm();// cwiseQuotient()逐元素除法，分量平方和的平方根
                __DEBUG_MATCH__(
                        cout<<"currBboxesXY:\n"<<currBboxesXY<<endl;
                        cout<<"def:\n"<<def<<endl;
                        )
//                simError(simMatrixM, simMatrixN) = error;
                if(error >= 1.0){ // 用1来限制过大的error
                    simMatrix(simMatrixM, simMatrixN) = 0;
                }else{
                    simMatrix(simMatrixM, simMatrixN) = exp(-1 * error);
                }

            }
            else{
//                simError(simMatrixM, simMatrixN) = 100000;
                simMatrix(simMatrixM, simMatrixN) = 0;
            }

        }
    }

    __DEBUG_MATCH__(
            cout<<"simMatrix:\n"<<simMatrix<<endl;
//            cout<<"simError: \n"<<simError<<endl;
            )
    Eigen::MatrixXd simMat_simhorn = graphMatching_h.sinkhorn(simMatrix);
    retmatch.resize(M);        // resize 第1张图的topo节点数
    retmatchinverse.resize(N); // resize 第2张图的topo节点数

    // 构建出的相似度矩阵的匹配结果
    Eigen::MatrixXd sim_results;
    sim_results = graphMatching_h.getBestMatchFromSimMat(simMat_simhorn, retmatch, retmatchinverse);

    isCubeMatching = true;
    graphMatching_h.clear();



    __DEBUG_MATCH__(
            cout<<"simMat_simhorn: \n"<<simMat_simhorn<<endl;
            cout<<"sim_results: \n"<<sim_results<<endl;
            cout<<"retmatch: "<<retmatch<<endl;
            )
    __DEBUG__(cout<<"ending matching cubes ..."<<TermColor::RESET()<<endl;)
    ROS_DEBUG_NAMED("cubeMatching()", "ending matching cubes!");

    return;
}

void VisualOdometry::saveImage(cv::String dest, cv::Mat image, size_t id, std::string imageName)
{
//    cv::String dest_ = "/home/jixingwu/catkin_ws/src/sem/semVO/image_results/";
    cv::String savedfilename_;
    char frame_index[256];
    sprintf(frame_index,"%06lu", id);
    savedfilename_ = dest + frame_index + ".jpg";
//    cout<<"save "<<imageName<<"image into "<<dest<<endl;
//    cout<<"file name is: "<<savedfilename_<<endl;
    cv::imwrite(savedfilename_, image);
}

#define __DEBUG_TRACK__(msg);

void VisualOdometry::trackCubes()
{
    __DEBUG__(cout<<TermColor::iGREEN()<<"starting tracking ..."<<TermColor::RESET()<<endl;)
    ROS_DEBUG_NAMED("trackCubes()", "%lu: starting tracker", curr_->id_);

    if (curr_->local_cuboids_.empty())
    {
        ROS_WARN("curr frame of local cuboids is empty!!");
        return;
    }
    __DEBUG_TRACK__(cout<<"curr_ id: "<<curr_->id_<<endl;)

    // 对于未被设置id的MapCube的id_ = -1;
    if(curr_->id_ == 0)
    {
        for (int ii = 0; ii < curr_->local_cuboids_.size(); ++ii) {
            curr_->local_cuboids_[ii]->id_ = ii;
        }
        __DEBUG_TRACK__(
                cout<<"first frame curr_ id:";
                for(auto cuboid:curr_->local_cuboids_)
                    cout<<cuboid->id_<<endl;
                )
        return;
    }

    // 完全映射
    // 不完全映射
    __DEBUG_TRACK__(
            cout<<TermColor::iGREEN()<<"retmatch: \n"<<retmatch<<endl;
            cout<<"ref_ id: ";
            for (auto cuboid : ref_->local_cuboids_)
                cout<<cuboid->id_<<endl;

            )

    vector<int> id_v;
    vector<int> id_vall;

    for (int ii = 0; ii < retmatch.size(); ++ii)
    {
        if(retmatch[ii] != -1)
        {
            curr_->local_cuboids_[retmatch[ii]]->id_ = ref_->local_cuboids_[ii]->id_;
            id_v.push_back(curr_->local_cuboids_[retmatch[ii]]->id_);
        }
        id_vall.push_back(ref_->local_cuboids_[ii]->id_);
    }

    for(auto & local_cuboid : curr_->local_cuboids_)
    {
        auto result = find(id_vall.begin(), id_vall.end(), local_cuboid->id_);
        if(result == id_vall.end())//没找到
        {
            auto p = max_element(id_vall.begin(), id_vall.end());
            local_cuboid->id_ = *p + 1;
            id_vall.push_back(local_cuboid->id_);
        }
    }
    __DEBUG_TRACK__(
            cout<<"curr_ id: ";
            for (auto cuboid:curr_->local_cuboids_) {
                cout<<cuboid->id_<<endl;
            }
            cout<<TermColor::RESET()<<endl;
            )

    __DEBUG__(cout<<TermColor::iGREEN()<<"ending tracking"<<TermColor::RESET()<<endl;)
    return;
}

void VisualOdometry::optimizeCube()
{
    ROS_DEBUG_NAMED("optimizeCube()", "%lu: starting opt", curr_->id_);
    estimator.optimization(map_, curr_);

}

void VisualOdometry::visualization()
{
    for(auto &mapCube : curr_->local_cuboids_)
    {
        pubCuboids(mapCube, Vector3d(0,1.0,0), curr_->time_stamp_);
    }
}