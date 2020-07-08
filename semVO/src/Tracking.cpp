//
// Created by jixingwu on 2020/3/16.
//

#include "Tracking.h"
//#include "SubscribeAndPublish.h"



#include <queue>
#include <mutex>
#include <thread>
#include <Eigen/Core>
#include <utility>

using namespace std;
using namespace Eigen;

typedef Matrix<double, 7, 1> Vector7d;
typedef Matrix<double, 5, 1> Vector5d;
//#define __TRACKING_DEBUG__(msg) msg;
#define __TRACKING_DEBUG__(msg);
Tracking::Tracking() {

//    InitToGround = cv::Mat::eye(4, 4, CV_32F);
//    // set initial camera pose wrt ground. by default camera parallel to ground, height=1.7 (kitti)
//    double init_x, init_y, init_z, init_qx, init_qy, init_qz, init_qw;
//    n.param<double>("init_x", init_x, 0);
//    n.param<double>("init_y", init_y, 0);
//    n.param<double>("init_z", init_z, 1.7);
//    n.param<double>("init_qx", init_qx, -0.7071);
//    n.param<double>("init_qy", init_qy, 0);
//    n.param<double>("init_qz", init_qz, 0);
//    n.param<double>("init_qw", init_qw, 0.7071);
//    Eigen::Quaternionf pose_quat(init_qw, init_qx, init_qy, init_qz);
//    Eigen::Matrix3f rot = pose_quat.toRotationMatrix(); // 	The quaternion is required to be normalized
//    for (int row = 0; row < 3; row++)
//        for (int col = 0; col < 3; col++)
//            InitToGround.at<float>(row, col) = rot(row, col);
//
//    InitToGround.at<float>(0, 3) = init_x;
//    InitToGround.at<float>(1, 3) = init_y;
//    InitToGround.at<float>(2, 3) = init_z;

    // set initial camera pose wrt ground. by default camera parallel to ground, height=1.7 (kitti)
    double init_x, init_y, init_z, init_qx, init_qy, init_qz, init_qw;
    n.param<double>("init_x", init_x, 0);
    n.param<double>("init_y", init_y, 0);
    n.param<double>("init_z", init_z, 1.7);
    n.param<double>("init_qx", init_qx, -0.7071);
    n.param<double>("init_qy", init_qy, 0);
    n.param<double>("init_qz", init_qz, 0);
    n.param<double>("init_qw", init_qw, 0.7071);
    Kalib << 718.856,  0,  607.1928,   // for KITTI cabinet data.
            0,  718.856, 185.2157,
            0,      0,     1;// fx fy cx cy

    detect_cuboid_obj = new detect_3d_cuboid();
    detect_cuboid_obj->print_details = false;
    detect_cuboid_obj->set_calibration(Kalib);

    detect_cuboid_obj->whether_plot_detail_images = true;
    detect_cuboid_obj->whether_plot_final_images = true;
    detect_cuboid_obj->print_details = true;
    detect_cuboid_obj->set_calibration(Kalib);
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

//    fixed_init_cam_pose_Twc(first_truth_frame_pose.tail<7>());
    Eigen::Quaterniond init_cam_pose_q(init_qw, init_qx, init_qy, init_qz);
    Eigen::Vector3d init_cam_pose_v(init_x, init_y, init_z);

    Vector7d cam_pose; cam_pose<< init_x, init_y, init_z, init_qx, init_qy, init_qz, init_qw;//0 0 1.7000000 -0.7071 0 0 0.7071
//    fixed_init_cam_pose_Twc = g2o::SE3Quat(init_cam_pose_q, init_cam_pose_v);
    fixed_init_cam_pose_Twc = g2o::SE3Quat(cam_pose);
    __TRACKING_DEBUG__(cout<< TermColor::iRED()<< "fixed_init_cam_pose_Twc: "  << fixed_init_cam_pose_Twc <<TermColor::RESET() << endl;)

}
Tracking::~Tracking() {}

#define __TRACK_DEBUG_PRINT__(msg);

void Tracking::Track() {// 有sub vins_estiamtor 的tracked_image topics 因此无需init
    // 假设已经初始化完成
//    queue<sensor_msgs::ImageConstPtr> key_buf = keyimg_buf;
//    if(!key_buf.empty()){
//        __TRACK_DEBUG_PRINT__(cout<<"[Tracking/Track()] Keyframe image of size: %d"<< keyimg_buf.size() << endl;)
//    }
//    while(key_buf.size() > 0){
//        sensor_msgs::ImageConstPtr img_msg = key_buf.front();
//        key_buf.pop();
//        img_t = img_msg->header.stamp;
//        __TRACK_DEBUG_PRINT__(cout<<"[Tracking/Track()] poped() raw image t="<< img_msg->header.stamp << "#######> ie." << endl;)
//        cv::Mat img = setImageFromMsg(img_msg);
//        CreateNewKeyFrame(img, img_msg->header.seq);
//    }




}

cv::Mat Tracking::setImageFromMsg(const sensor_msgs::ImageConstPtr msg) {
    auto tmp = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::Mat image = tmp.clone();
    assert( image.rows>0 && image.cols>0 && !image.empty() && "In Tracking::setImageFromMsg image that is being set from sensor_msgs::ImageConstPtr is invalid");
    return image;
}

void Tracking::CreateNewKeyFrame(cv::Mat img, uint32_t imgID) {
    // output in terminal imageId
//    queue<sensor_msgs::ImageConstPtr>
    __TRACK_DEBUG_PRINT__(cout<<"[Tracking/CreateNewKeyFrame()] imageID = "<< imgID << "#######> ie." << endl;)
    // whether detect object
//    DetectCuboid(img);// TODO: fix DetectCuboid()
}


void Tracking::inputImageMsg() {
    //for(auto& msg : img_buf)
}

void Tracking::inputImage(const cv::Mat& img)
{
//    cv::Mat raw_img = img.clone();
//    m_buf.lock();
//    if(!keyframe_bboxes_buf.empty() && !frame_bboxes_buf.empty())
//    {
//        keyframe_bboxes = keyframe_bboxes_buf.front();
//        frame_bboxes = frame_bboxes_buf.front();
//        keyframe_bboxes_buf.pop();
//        frame_bboxes_buf.pop();
//
//        DetectCuboid(raw_img);
//        MatchCuboid(keyframe_bboxes, frame_bboxes);
//        if(frame_bboxes_buf.size() >= 2)
//            inputFrameBboxes2trian(frame_bboxes, frame_bboxes_buf.front());
//    }
//    m_buf.unlock();

}

//void Tracking::inputFrameBboxes2trian(darknet_ros_msgs::BoundingBoxes frame_bboxes,
//        darknet_ros_msgs::BoundingBoxes frame_bboxes_next)
//{
//    std::vector<cv::Point2f> points1, points2;
////    for (const auto & bboxes : frame_bboxes.bounding_boxes)
////    {
////        cv::Point2f points;
////        points.x = static_cast<float>((bboxes.xmin + bboxes.xmax)/2);
////        points.y = static_cast<float>((bboxes.ymin + bboxes.ymax)/2);
////        points1.push_back(points);
////    }
//    bboxes2CenterPpoints2f(frame_bboxes, points1);
//    bboxes2CenterPpoints2f(frame_bboxes_next, points2);
//
//    //-- 估计两张图像间运动
//    cv::Mat R, t;
//    framer.pose_estimation_2d2d(points1, points2, R, t);
//
//    //-- 三角化
//    std::vector<cv::Point3d> points_3d;
//    framer.triangulation(points1, points2, R, t, points_3d);
//
//    //验证三角化点与特征点的重投影关系
//    for (int i = 0; i < points_3d.size(); ++i)
//    {
//        cv::Point2d pt1_cam = framer.pixel2cam(points1[i], framer.K);
//        cv::Point2d pt1_cam_3d(
//                points_3d[i].x/points_3d[i].z,
//                points_3d[i].y/points_3d[i].z
//                );
//        cout<<"point in the first camera frame: "<<pt1_cam<<endl;
//        cout<<"point projected from 3D "<<pt1_cam_3d<<", d="<<points_3d[i].z<<endl;
//        //第二张图
//        cv::Point2f pt2_cam = framer.pixel2cam(points2[i], framer.K);
//        cv::Mat pt2_trans = R*(Mat_<double>(3,1)<<
//                points_3d[i].x, points_3d[i].y, points_3d[i].z)+t;
//        pt2_trans /= pt2_trans.at<double>(2,0);
//
//        cout<<"point in the second camera frame: "<<pt2_cam<<endl;
//        cout<<"point reprojected from second frame: "<<pt2_trans.t()<<endl;
//    }
//}

void Tracking::bboxes2CenterPpoints2f(darknet_ros_msgs::BoundingBoxes frame_bboxes,
                                std::vector<cv::Point2f>& Points)
{
    for (const auto & bboxes : frame_bboxes.bounding_boxes)
    {
        cv::Point2f points;
        points.x = static_cast<float>((bboxes.xmin + bboxes.xmax)/2);
        points.y = static_cast<float>((bboxes.ymin + bboxes.ymax)/2);
        Points.push_back(points);
    }
}

void Tracking::inputCamPose(Eigen::Matrix4d cam_pose)
{
    //cam_transToGround = cam_pose;
}

bool Tracking::MatchCuboid(darknet_ros_msgs::BoundingBoxes keyframe_bboxes, darknet_ros_msgs::BoundingBoxes frame_bboxes)
{

    int M = keyframe_bboxes.bounding_boxes.size();
    int N = frame_bboxes.bounding_boxes.size();

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> simMatrix;
    simMatrix.resize(M, N);

    Matrix42d keyframeCoor, frameCoor;

    for (int simMatrixM = 0; simMatrixM < M; ++simMatrixM) {
        for (int simMatrixN = 0; simMatrixN < N; ++simMatrixN) {

            int length = keyframe_bboxes.bounding_boxes[M].xmin - keyframe_bboxes.bounding_boxes[M].xmax;
            int width  = keyframe_bboxes.bounding_boxes[M].ymin - keyframe_bboxes.bounding_boxes[M].ymax;

            int x0 = keyframe_bboxes.bounding_boxes[M].xmin;
            int y0 = keyframe_bboxes.bounding_boxes[M].ymin;
            int x1 = x0 + length;
            int y1 = y0;
            int x2 = keyframe_bboxes.bounding_boxes[M].xmax;
            int y2 = keyframe_bboxes.bounding_boxes[M].ymax;
            int x3 = x2;
            int y3 = y2 - width;
            keyframeCoor<<x0, y0, x1, y1, x2, y2, x3, y3;

            length = frame_bboxes.bounding_boxes[N].xmin - frame_bboxes.bounding_boxes[N].xmax;
            width  = frame_bboxes.bounding_boxes[N].ymin - frame_bboxes.bounding_boxes[N].ymax;

            x0 = frame_bboxes.bounding_boxes[N].xmin;
            y0 = frame_bboxes.bounding_boxes[N].ymin;
            x1 = x0 + length;
            y1 = y0;
            x2 = keyframe_bboxes.bounding_boxes[N].xmax;
            y2 = keyframe_bboxes.bounding_boxes[N].ymax;
            x3 = x2;
            y3 = y2 - width;
            frameCoor<<x0, y0, x1, y1, x2, y2, x3, y3;

            double error = computeError(keyframeCoor, frameCoor);
            simMatrix(M, N) = exp(-1 * error);
        }
    }

    Eigen::MatrixXd sim_results;
    graphmatching_h.inputSimMatrix(simMatrix, sim_results);

    cout<<"--------------------/"<<endl;
    cout<<"关键帧与帧间图片的匹配结果sim_result:\n"<<sim_results<<endl;
}

template <typename GraphT, typename NodeT>
void GraphMatching<GraphT, NodeT>::inputSimMatrix(Eigen::MatrixXd& sim_mat, Eigen::MatrixXd& sim_results)
{
    Eigen::MatrixXd simMat_simhorn;
    simMat_simhorn = sinkhorn(sim_mat);

    Eigen::VectorXi retmatch(prior_graph->num_of_nodes()) , retmatchinverse(visited_graph->num_of_nodes());
    retmatch.setConstant(-1);
    retmatchinverse.setConstant(-1);

    sim_results = getBestMatchFromSimMat(simMat_simhorn, retmatch, retmatchinverse);
}
template <typename T>
double squareOpenRoot4(T x1, T y1, T x2, T y2 )
{
    return sqrt( pow((x1-x2),2)+ pow((y1-y2),2) );
}


double Tracking::computeError(Matrix42d keyframeCoor, Matrix42d frameCoor)
{
    //边长
    double d01 = squareOpenRoot4<double>(keyframeCoor(0,1), keyframeCoor(0,2), keyframeCoor(1,1), keyframeCoor(1,2));
    double d01_= squareOpenRoot4<double>(frameCoor(0,1), frameCoor(0,2), frameCoor(1,1), frameCoor(1,2));
    double d12 = squareOpenRoot4<double>(keyframeCoor(1,1), keyframeCoor(1,2), keyframeCoor(2,1), keyframeCoor(2,2));
    double d12_= squareOpenRoot4<double>(frameCoor(1,1), frameCoor(1,2), frameCoor(2,1), frameCoor(2,2));
    double d23 = squareOpenRoot4<double>(keyframeCoor(2,1), keyframeCoor(2,2), keyframeCoor(3,1), keyframeCoor(3,2));
    double d23_= squareOpenRoot4<double>(frameCoor(2,1), frameCoor(2,2), frameCoor(3,1), frameCoor(3,2));
    double d30 = squareOpenRoot4<double>(keyframeCoor(3,1), keyframeCoor(3,2), keyframeCoor(0,1), keyframeCoor(0,2));
    double d30_= squareOpenRoot4<double>(frameCoor(3,1), frameCoor(3,2), frameCoor(0,1), frameCoor(0,2));
    //对角
    double d02 = squareOpenRoot4<double>(keyframeCoor(0,1), keyframeCoor(0,2), keyframeCoor(2,1), keyframeCoor(2,2));
    double d02_= squareOpenRoot4<double>(frameCoor(0,1), frameCoor(0,2), frameCoor(2,1), frameCoor(2,2));

    return sqrt( pow( (d02 - d02_) ,2 ) ) / (d01 + d12 + d23 + d30 + d01_ + d12_ + d23_ + d30_);
}

#define __TRACKING_CALLBACK_PRINT__(msg) msg;
void Tracking::frame_bboxes_callback(const darknet_ros_msgs::BoundingBoxes& msg) {
    if(bboxes_t0_available == false)
    {
        bboxes_t0 = msg.header.stamp;
        bboxes_t0_available = true;
    }

    __TRACKING_CALLBACK_PRINT__(
            cout << TermColor::iBLUE() << "[Tracking/frame_bboxes_callback] frame_bboxes t= "
            << msg.header.stamp - bboxes_t0 << TermColor::RESET() << endl;
            )
//    m_buf.lock();
//    frame_bboxes_buf.push(msg);
//    m_buf.unlock();
    if(!img_buf.empty()){
        cv::Mat image = img_buf.front();
        img_buf.pop();
        DetectCuboid(image, msg);
    }


    return;
}

void Tracking::detection_image_callback(const sensor_msgs::Image &msg) {
    __TRACKING_CALLBACK_PRINT__(
            cout << TermColor::iGREEN() << "[Tracking/detection_image_callback] detection_image t= "
                 << msg.header.stamp - bboxes_t0 << TermColor::RESET() << endl;
    )
    return;
}

void Tracking::left_image_callback(const sensor_msgs::Image& msg){

    __TRACKING_CALLBACK_PRINT__(
            cout<< TermColor::iBLUE() << "[Tracking/left_image_callback] image t= "
            << msg.header.stamp - bboxes_t0 << TermColor::RESET() << endl;
            )
    cv_bridge::CvImagePtr cv_ptr;
    if(msg.encoding == "8UC1"){
        sensor_msgs::Image img;
        img.header = msg.header;
        img.height = msg.height;
        img.width = msg.width;
        img.is_bigendian = msg.is_bigendian;
        img.step = msg.step;
        img.data = msg.data;
        img.encoding = "mono8";
        cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    cv::Mat img = cv_ptr->image.clone();
    m_buf.lock();
    img_buf.push(img);
    m_buf.unlock();

    return;
}

//void Tracking::GenerateCuboid() {
//    cv::Mat img;
//}

#define __TRACKING_DETECTCUBOID_DEBUG__(msg) msg;

void Tracking::DetectCuboid(const cv::Mat& raw_rgb_image, const darknet_ros_msgs::BoundingBoxes& frame_bboxes)// get 'Keyframe *pKF' from vins_fusion, also a signal image
{
    g2o::SE3Quat curr_cam_pose_Twc;
    Eigen:Vector3d pan_init;
    pan_init << 0, 0, 0;
    g2o::SE3Quat odom_val(Quaterniond(1,0,0,0), pan_init); // from previous frame to current frame


    if(frame_index == 0)
        curr_cam_pose_Twc = fixed_init_cam_pose_Twc;
    else{
        g2o::SE3Quat prev_pose_Tcw = all_frames[frame_index-1]->cam_pose_Tcw;
        if (frame_index>1)  // from third frame, use constant motion model to initialize camera.
        {
            g2o::SE3Quat prev_prev_pose_Tcw = all_frames[frame_index-2]->cam_pose_Tcw;
            odom_val = prev_pose_Tcw*prev_prev_pose_Tcw.inverse();
        }
        curr_cam_pose_Twc = (odom_val*prev_pose_Tcw).inverse();
    }

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
    __TRACKING_DETECTCUBOID_DEBUG__(
            cout<< TermColor::iBLUE() << "[Tracking/DetectCuboid] frame_bboxes size: "<< frame_bboxes.bounding_boxes.size() << TermColor::RESET() << endl;
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

    Matrix4d transToWorld;
    detect_cuboid_obj->whether_sample_cam_roll_pitch = (frame_index!=0);
    if(detect_cuboid_obj->whether_sample_cam_roll_pitch)
        transToWorld = fixed_init_cam_pose_Twc.to_homogeneous_matrix();
    else
        transToWorld = curr_cam_pose_Twc.to_homogeneous_matrix();


    __TRACKING_DETECTCUBOID_DEBUG__(
            cout<< TermColor::iBLUE() <<"[tracking/DetectCuboid()] transToWorld: "<< transToWorld<< TermColor::RESET() <<endl;
            cout<<"[tracking/DetectCuboid()] all_object_coor: "<< all_object_coor.size()<<endl;
            //cout<<"[tracking/DetectCuboid()] all_lines_raw: "<< all_lines_raw<<endl;
//            cvNamedWindow("img");
//            cvMoveWindow("img", 20, 300);
//            cv::imshow("img", raw_rgb_image);
//            cv::waitKey(0);
            cout<< TermColor::iBLUE() << "[tracking/DetectCuboid()] fixed_init_cam_pose_Twc:" << fixed_init_cam_pose_Twc<< TermColor::RESET() << endl;
    )

    assert(all_object_coor.size() != 0 && "bboxes_buf is empty");

    __TRACKING_DETECTCUBOID_DEBUG__(
            cout<< TermColor::iRED()
            << "[Tracking/DetectCuboid] raw_rgb_image.width: " << raw_rgb_image.cols << endl
            << "transToWorld: "<< transToWorld <<endl
            << "all_object_coor.size: "<<all_object_coor.size()<< endl
            << "all_lines_raw.size: " << all_lines_raw.size()<< TermColor::RESET() << endl;
            cvNamedWindow("raw_rgb_image");
            cvMoveWindow("raw_rgb_image", 20, 300);
            cv::imshow("raw_rgb_image",raw_rgb_image);
            cv::waitKey(0);
            )
    detect_cuboid_obj->detect_cuboid(raw_rgb_image, transToWorld, all_object_coor, all_lines_raw, frames_cuboid);
    currframe->cuboids_2d_img = detect_cuboid_obj->cuboids_2d_img;

    __TRACKING_DETECTCUBOID_DEBUG__(
            cout<< TermColor::iBLUE()<<"[tracking/DetectCuboid()] detected frames_cuboids size: " << frames_cuboid.size()<< TermColor::RESET() <<endl;
            cvNamedWindow("currframe.cuboids_2d_img");
            cvMoveWindow("currframe.cuboids_2d_img", 20, 300);
            cv::imshow("currframe.cuboids_2d_img",currframe->cuboids_2d_img);
            cv::waitKey(0);
    )
    frame_index++;



//    cv::Mat pop_pose_to_ground = InitToGround;
//    std::vector<ObjectSet> all_obj_cubes;
//    std::vector<double> all_box_confidence;
//    std::vector<int> truth_tracklet_ids;
//
//
//
////    line_lbd_obj.detect_filter_lines(raw_image, all)
////    detect_cuboid_obj = framer.detect_cuboid_obj;
//
//
//
//
////    cv::Mat frame_pose_to_init;// get vins fusion camera pose inverse
//    cv::Mat frame_pose_to_ground = camera_pose;
//    frame_pose_to_ground = InitToGround * frame_pose_to_ground;
//
//    Eigen::Matrix4f cam_transToGround = Converter::toMatrix4f(frame_pose_to_ground);
//
//    // TODO: transToGround 解决raw_image的pose问题
//    detect_cuboid_obj->detect_cuboid(raw_image, cam_transToGround.cast<double>(), all_object_coor, all_lines_raw, frames_cuboid);
//
//    g2o::SE3Quat frame_pose_to_init = Converter::toSE3Quat(camera_pose);
//    g2o::SE3Quat InitToGround_se3 = Converter::toSE3Quat(InitToGround);
//
//    g2o::cuboid cube_local_meas;
////    g2o::SE3Quat curr_cam_pose_Twc = Converter::toSE3Quat();
//
//    has_detected_cuboid = !frames_cuboid.empty();
//    if(has_detected_cuboid)
//    {
//        for (int ii = 0; ii < (int)frames_cuboid.size(); ++ii)
//        {
//
//            cuboid *raw_cuboid = frames_cuboid[ii][0];
//            g2o::cuboid cube_ground_value;// [x y z yaw l w h]
//            Vector9d cube_pose;
//            cube_pose << raw_cuboid->pos[0], raw_cuboid->pos[1], raw_cuboid->pos[2], 0, 0, raw_cuboid->rotY,
//            raw_cuboid->scale[0], raw_cuboid->scale[1], raw_cuboid->scale[2];
//            cube_ground_value.fromMinimalVector(cube_pose);
//
////            cube_local_meas = cube_ground_value.transform_to(pop_pose_to_ground);
//
//            // TODO pub frames_cuboid###########################################################
//            //// raw_cuboid
//            // MakerArray.size() = frames_cuboid.size * 2
//            visualization_msgs::MarkerArray frame_long_markers;
//            visualization_msgs::MarkerArray frame_markers;
//            std::vector<object_landmark*> cube_pose_raw_detected_history(frames_cuboid[ii].size(), nullptr);
//
////            for (int jj = 0; jj < frames_cuboid[ii].size(); ++jj) {
////                cuboid *raw_cuboid = frames_cuboid[ii][jj];
////                frame_markers = cuboids_to_marker(raw_cuboid, Vector3d(0,0,1));// return MakerArray.size()=2
////                for frame_long_markers = [frame_markers, frame_markers, ...]
////                object_landmark *tempcuboids2 = new object_landmark();
////                tempcuboids2->cube_vertex = new g2o::VertexCuboid();// ye
////                cuboid *raw_cuboid = all_obj_cubes[ii][0];
////                g2o::cuboid cube_ground_value;
////                Vector
////            }
//
////            dataManager.cube_makers_pub(frame_long_markers);
//
//                // measurement in local frame! important
//
//
//        }
//    }
}

visualization_msgs::MarkerArray Tracking::cuboids_to_marker(cuboid *raw_cuboid, Vector3d rgbcolor) {
    visualization_msgs::MarkerArray plane_markers; visualization_msgs::Marker marker;
    if (raw_cuboid == nullptr) return plane_markers;

    marker.header.frame_id = "/world"; marker.header.stamp = img_t;//ros::Time::now();
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP; marker.action = visualization_msgs::Marker::ADD;
    marker.color.r = rgbcolor(0); marker.color.g = rgbcolor(1); marker.color.b = rgbcolor(2); marker.color.a = 1.0;
    marker.scale.x = 0.02;

}



//// one cuboid need front and back markers...
//void cuboid_corner_to_marker(const Matrix38d& cube_corners, visualization_msgs::Marker& marker, int bodyOrfront)
//{
//    Eigen::VectorXd edge_pt_ids;
//    if (bodyOrfront==0) { // body edges
//        edge_pt_ids.resize(16); edge_pt_ids<<1,2,3,4,1,5,6,7,8,5,6,2,3,7,8,4;edge_pt_ids.array()-=1;
//    }else { // front face edges
//        edge_pt_ids.resize(5); edge_pt_ids<<1,2,6,5,1;edge_pt_ids.array()-=1;
//    }
//    marker.points.resize(edge_pt_ids.rows());
//    for (int pt_id=0; pt_id<edge_pt_ids.rows(); pt_id++)
//    {
//        marker.points[pt_id].x = cube_corners(0, edge_pt_ids(pt_id));
//        marker.points[pt_id].y = cube_corners(1, edge_pt_ids(pt_id));
//        marker.points[pt_id].z = cube_corners(2, edge_pt_ids(pt_id));
//    }
//}
//
//// one cuboid need front and back markers...  rgbcolor is 0-1 based
//visualization_msgs::MarkerArray cuboids_to_marker(object_landmark* obj_landmark, Vector3d rgbcolor)
//{
//    visualization_msgs::MarkerArray plane_markers;  visualization_msgs::Marker marker;
//    if (obj_landmark==nullptr)
//        return plane_markers;
//
//    marker.header.frame_id="/cuboid";
//    marker.header.stamp=ros::Time::now();
//    marker.id = 0; //0
//    marker.type = visualization_msgs::Marker::LINE_STRIP;
//    marker.action = visualization_msgs::Marker::ADD;
//    marker.color.r = rgbcolor(0); marker.color.g = rgbcolor(1); marker.color.b = rgbcolor(2); marker.color.a = 1.0;
//    marker.scale.x = 0.02;
//
//    g2o::cuboid cube_opti = obj_landmark->cube_vertex->estimate();
//    Eigen::MatrixXd cube_corners = cube_opti.compute3D_BoxCorner();
//
//    for (int ii=0;ii<2;ii++) // each cuboid needs two markers!!! one for all edges, one for front facing edge, could with different color.
//    {
//        marker.id++;
//        cuboid_corner_to_marker(cube_corners,marker, ii);
//        plane_markers.markers.push_back(marker);
//    }
//    return plane_markers;
//}

