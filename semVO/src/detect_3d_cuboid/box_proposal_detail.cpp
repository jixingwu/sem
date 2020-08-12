// std c
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <ctime>

// opencv pcl
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"

// ros
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Core>

// ours
#include "matrix_utils.h"
#include "object_3d_util.h"
#include "profiler.hpp"
#include "detect_3d_cuboid.h"

using namespace std;
// using namespace cv;
using namespace Eigen;

void detect_3d_cuboid::set_calibration(const Matrix3d &Kalib)
{
	cam_pose.Kalib = Kalib;
	cam_pose.invK = Kalib.inverse();
}

void detect_3d_cuboid::set_cam_pose(const Matrix4d &transToWolrd)
{
	cam_pose.transToWolrd = transToWolrd;
	cam_pose.rotationToWorld = transToWolrd.topLeftCorner<3, 3>();//左上子矩阵
	Vector3d euler_angles;
	quat_to_euler_zyx(Quaterniond(cam_pose.rotationToWorld), euler_angles(0), euler_angles(1), euler_angles(2));
	cam_pose.euler_angle = euler_angles;
	cam_pose.invR = cam_pose.rotationToWorld.inverse();
	cam_pose.projectionMatrix = cam_pose.Kalib * transToWolrd.inverse().topRows<3>(); // project world coordinate to camera
	cam_pose.KinvR = cam_pose.Kalib * cam_pose.invR;
	cam_pose.camera_yaw = cam_pose.euler_angle(2);
	//TODO relative measure? not good... then need to change transToWolrd.
}

//void detect_3d_cuboid::detect_cuboid(const cv::Mat &rgb_img, const Eigen::Matrix4d &transToWolrd,
//                                     const Eigen::MatrixXd &obj_bbox_coors, Eigen::MatrixXd edges,
//                                     std::vector<ObjectSet> &all_object_cuboids) {
//
//}

void detect_3d_cuboid::detect_cuboid(const cv::Mat &rgb_img, const Matrix4d &transToWolrd, const MatrixXd &obj_bbox_coors,
									 Eigen::MatrixXd all_lines_raw, std::vector<ObjectSet> &all_object_cuboids)
{
    cv::Mat merge_lines_img = rgb_img.clone();
    // 绘制上边缘采样点
    typedef cv::Point_<int> Point2i;
    std::vector<Point2i> simple_points;
    cv::Mat image_point = rgb_img;

    set_cam_pose(transToWolrd);
	cam_pose_raw = cam_pose;

	cv::Mat gray_img;
	if (rgb_img.channels() == 3)
		cv::cvtColor(rgb_img, gray_img, CV_BGR2GRAY);
	else
		gray_img = rgb_img;

	int img_width = rgb_img.cols;
	int img_height = rgb_img.rows;

	int num_2d_objs = obj_bbox_coors.rows();
	all_object_cuboids.resize(num_2d_objs);

	vector<bool> all_configs;
	all_configs.push_back(consider_config_1);
	all_configs.push_back(consider_config_2);

	// parameters for cuboid generation
	double vp12_edge_angle_thre = 15;
	double vp3_edge_angle_thre = 10;	// 10  10  parameters
	double shorted_edge_thre = 20;		// if box edge are too short. box might be too thin. most possibly wrong.
	bool reweight_edge_distance = true; // if want to compare with all configurations. we need to reweight

	// parameters for proposal scoring
	bool whether_normalize_two_errors = true;
	double weight_vp_angle = 0.8;
	double weight_skew_error = 1.5;
	// if also consider config2, need to weight two erros, in order to compare two configurations

	//STEP [1.确保边缘线段的两个端点是从左到右存储的]
	align_left_right_edges(all_lines_raw); // this should be guaranteed when detecting edges
	//显示边缘检测的图
	if (whether_plot_detail_images)
	{
		cv::Mat output_img;
		plot_image_with_edges(rgb_img, output_img, all_lines_raw, cv::Scalar(255, 0, 0));
		cv::imshow("Raw detected Edges", output_img);
		cv::waitKey(0);
	}

	//STEP 【2.世界坐标系和相机坐标系】
	// find ground-wall boundary edges
	Vector4d ground_plane_world(0, 0, 1, 0); // treated as column vector % in my pop-up code, I use [0 0 -1 0]. here I want the normal pointing innerwards, towards the camera to match surface normal prediction
	Vector4d ground_plane_sensor = cam_pose.transToWolrd.transpose() * ground_plane_world;

	//逐个bbox处理
	//       int object_id=1;
//	cout<<"num_2d_objs: "<<num_2d_objs<<endl;
	for (int object_id = 0; object_id < obj_bbox_coors.rows(); object_id++)
	{
//        cout<<"----------8"<<endl;
//	    std::cout<<"object id  "<<object_id<<std::endl;
//		ca::Profiler::tictoc("One 3D object total time");
		//计算左上角点的坐标xy
		int left_x_raw = static_cast<int>(obj_bbox_coors(object_id, 0));
		int top_y_raw = static_cast<int>(obj_bbox_coors(object_id, 1));
		int obj_width_raw = static_cast<int>(obj_bbox_coors(object_id, 2));
		int obj_height_raw = static_cast<int>(obj_bbox_coors(object_id, 3));
		//计算右下角点的坐标
		int right_x_raw = left_x_raw + obj_width_raw;
		int down_y_raw = top_y_raw + obj_height_raw;
//        cout<<"----------9"<<endl;
//        cout<<"left_x, top_y, right_x, down_y: "<<left_x_raw<<"\t"<<top_y_raw<<"\t"<<right_x_raw<<"\t"<<down_y_raw<<endl;
		//绘制检测框
		if(left_x_raw<0 || top_y_raw<0 || right_x_raw<0 || down_y_raw<0)
            continue;
        rectangle(	rgb_img,
                      cv::Point(left_x_raw, top_y_raw),
                      cv::Point(right_x_raw, down_y_raw),
                      cv::Scalar(0,255,255),
                      2,
                      8);

        /// @param down_expand_sample_all
		std::vector<int> down_expand_sample_all;
		down_expand_sample_all.push_back(0);
		if (whether_sample_bbox_height) // 2D object detection might not be accurate
		{
			int down_expand_sample_ranges = max(min(20, obj_height_raw - 90), 20);
			down_expand_sample_ranges = min(down_expand_sample_ranges, img_height - top_y_raw - obj_height_raw - 1); // should lie inside the image  -1 for c++ index
			if (down_expand_sample_ranges > 10)																		 // if expand large margin, give more samples.
				down_expand_sample_all.push_back(round(down_expand_sample_ranges / 2));
			down_expand_sample_all.push_back(down_expand_sample_ranges);//(0, 10, 20)
		}

//        for (int i = 0; i < down_expand_sample_all.size(); ++i) {
//            cout<<down_expand_sample_all[i]<<endl;
//        }
		// NOTE later if in video, could use previous object yaw..., also reduce search range
//        cout<<"----------10"<<endl;
		double yaw_init = cam_pose.camera_yaw - 90.0 / 180.0 * M_PI; // yaw init is directly facing the camera, align with camera optical axis
		std::vector<double> obj_yaw_samples;
//        cout<<"----------11"<<endl;
		linespace<double>(yaw_init - 45.0 / 180.0 * M_PI, yaw_init + 45.0 / 180.0 * M_PI, 6.0 / 180.0 * M_PI, obj_yaw_samples);
//        cout<<"111111"<<endl;

		MatrixXd all_configs_errors(400, 9);
		MatrixXd all_box_corners_2ds(800, 8);   // initialize a large eigen matrix
		int valid_config_number_all_height = 0; // all valid objects of all height samples
		ObjectSet raw_obj_proposals;
		raw_obj_proposals.reserve(100);
		// 	    int sample_down_expan_id=1;
		for (int sample_down_expan_id = 0; sample_down_expan_id < down_expand_sample_all.size(); sample_down_expan_id++)
		{
			int down_expand_sample = down_expand_sample_all[sample_down_expan_id];// = 0, 10, 20
			int obj_height_expan = obj_height_raw + down_expand_sample;
			int down_y_expan = top_y_raw + obj_height_expan;
			//宽度不变，高度变高，求取对角线长度
//			cout<<"obj_width_raw * obj_width_raw + obj_height_expan * obj_height_expan: "<<obj_width_raw * obj_width_raw + obj_height_expan * obj_height_expan<<endl;
//			if((obj_width_raw * obj_width_raw + obj_height_expan * obj_height_expan) == 0) continue;
			double obj_diaglength_expan = sqrt(obj_width_raw * obj_width_raw + obj_height_expan * obj_height_expan);

//			cout<<"obj_diaglength_expan: "<<obj_diaglength_expan<<endl;
			// STEP5 【上边缘采样】
			//【顶边上的采样点的x坐标】，如果边太大，则提供更多样本。为所有边缘至少提供10个，

			// sample points on the top edges, if edge is too large, give more samples. give at least 10 samples for all edges. for small object, object pose changes lots
			int top_sample_resolution = round(min(20, obj_width_raw / 10)); //  25 pixels

			if(top_sample_resolution<1)
			    break;

			std::vector<int> top_x_samples;
            // NOTE 边界框的最左边left_x_raw+5  到右边right_x_raw-5，每隔top_sample_resolution(20像素)的距离采样一个点
			linespace<int>(left_x_raw + 5, right_x_raw - 5, top_sample_resolution, top_x_samples);
//			cout<<"2222"<<endl;
			MatrixXd sample_top_pts(2, top_x_samples.size());//储存顶边采样点
			for (int ii = 0; ii < top_x_samples.size(); ii++)
			{
				sample_top_pts(0, ii) = top_x_samples[ii];
				sample_top_pts(1, ii) = top_y_raw;

                simple_points.push_back(Point2i(top_x_samples[ii], top_y_raw));
//                circle(image_point, simple_points[ii], 3, cv::Scalar(255,0,0),-1,8,0);
			}


			//扩大边界范围
			// expand some small margin for distance map  [10 20]
			int distmap_expand_wid = min(max(min(20, obj_width_raw - 100), 10), max(min(20, obj_height_expan - 100), 10));//20
			int left_x_expan_distmap = max(0, left_x_raw - distmap_expand_wid);//727
			int right_x_expan_distmap = min(img_width - 1, right_x_raw + distmap_expand_wid);
			int top_y_expan_distmap = max(0, top_y_raw - distmap_expand_wid);
			int down_y_expan_distmap = min(img_height - 1, down_y_expan + distmap_expand_wid);
			int height_expan_distmap = down_y_expan_distmap - top_y_expan_distmap;////扩大后高度 安全控制
			int width_expan_distmap = right_x_expan_distmap - left_x_expan_distmap;////扩大后宽度 安全控制
			Vector2d expan_distmap_lefttop = Vector2d(left_x_expan_distmap, top_y_expan_distmap);//左上角坐标
			Vector2d expan_distmap_rightbottom = Vector2d(right_x_expan_distmap, down_y_expan_distmap);//右下角坐标

//            if(sample_down_expan_id == 0)
//                // NOTE 绘制bbox的边界.
//                rectangle(	rgb_img,
//                              cv::Point(left_x_expan_distmap, top_y_expan_distmap),
//                              cv::Point(right_x_expan_distmap, down_y_expan_distmap),
//                              cv::Scalar(255,0,0),
//                              2,
//                              8);

            //STEP 【6 线段处理】
            //STEP 【6.2 找到扩大后的边界框内的线段】
			// find edges inside the object bounding box
			/// @param all_lines_inside_object: 存储所有在扩大后的边界框内的线段
			MatrixXd all_lines_inside_object(all_lines_raw.rows(), all_lines_raw.cols()); // first allocate a large matrix, then only use the toprows to avoid copy, alloc
			int inside_obj_edge_num = 0;

			//检测所有扩大后的边框内的线段
			for (int edge_id = 0; edge_id < all_lines_raw.rows(); edge_id++)
			    //判断all_lines_raw矩阵中第edge_id线段的另一个端点.head<2>是否在区域中, x(1: n)
				if (check_inside_box(all_lines_raw.row(edge_id).head<2>(), expan_distmap_lefttop, expan_distmap_rightbottom))
                    //判断all_lines_raw矩阵中第edge_id线段的另一个端点.tail<2>是否在区域中, x(end-n+1: end)
					if (check_inside_box(all_lines_raw.row(edge_id).tail<2>(), expan_distmap_lefttop, expan_distmap_rightbottom))
					{
						all_lines_inside_object.row(inside_obj_edge_num) = all_lines_raw.row(edge_id);
						inside_obj_edge_num++;
					}
            //STEP 【6.2 在找到物体的边缘线之后合并边，并剔除短边，小区域的边缘合并应该更快】
			// merge edges and remove short lines, after finding object edges.  edge merge in small regions should be faster than all.
			///@param 线段合并与筛选参数
			double pre_merge_dist_thre = 20;
			double pre_merge_angle_thre = 5;
			double edge_length_threshold = 30;
			MatrixXd all_lines_merge_inobj;//合并之后的线段存储矩阵
			merge_break_lines(all_lines_inside_object.topRows(inside_obj_edge_num), all_lines_merge_inobj, pre_merge_dist_thre,
							  pre_merge_angle_thre, edge_length_threshold);

            // 显示筛选之后的边缘线段.
             cv::Mat output_img;
             plot_image_with_edges(merge_lines_img, output_img, all_lines_merge_inobj, cv::Scalar(0,255,0));
//             cvNamedWindow("merge_lines_img");
//             cvMoveWindow("merge_lines_img",500, 300);
//             cv::imshow("merge_lines_img", output_img);
//             cv::waitKey(0);

             //STEP 【7. 计算角度、中点、canny边缘检测，距离变换】
			// compute edge angels and middle points
			VectorXd lines_inobj_angles(all_lines_merge_inobj.rows());
			MatrixXd edge_mid_pts(all_lines_merge_inobj.rows(), 2);
			for (int i = 0; i < all_lines_merge_inobj.rows(); i++)
			{
				lines_inobj_angles(i) = std::atan2(all_lines_merge_inobj(i, 3) - all_lines_merge_inobj(i, 1), all_lines_merge_inobj(i, 2) - all_lines_merge_inobj(i, 0)); // [-pi/2 -pi/2]
				edge_mid_pts.row(i).head<2>() = (all_lines_merge_inobj.row(i).head<2>() + all_lines_merge_inobj.row(i).tail<2>()) / 2;
			}

			// TODO could canny or distance map outside sampling height to speed up!!!!   Then only need to compute canny onces.
			// detect canny edges and compute distance transform  NOTE opencv canny maybe different from matlab. but roughly same
//			cout<<"----------"<<left_x_expan_distmap<<"\t"<<top_y_expan_distmap<<"\t"<<width_expan_distmap<<"\t"<<height_expan_distmap<<"\t"<<endl;

			if (left_x_expan_distmap<0 || top_y_expan_distmap<0 || width_expan_distmap<0 || height_expan_distmap<0)
                continue;

			cv::Rect object_bbox = cv::Rect(left_x_expan_distmap, top_y_expan_distmap, width_expan_distmap, height_expan_distmap); //
			cv::Mat im_canny;
			cv::Canny(gray_img(object_bbox), im_canny, 80, 200); // low thre, high thre    im_canny 0 or 255   [80 200  40 100]
			cv::Mat dist_map;
			cv::distanceTransform(255 - im_canny, dist_map, CV_DIST_L2, 3); // dist_map is float datatype
//            cout<<"----------1"<<endl;
			if (whether_plot_detail_images)
			{
				cv::imshow("im_canny", im_canny);
				cv::Mat dist_map_img;
				cv::normalize(dist_map, dist_map_img, 0.0, 1.0, cv::NORM_MINMAX);
				cv::imshow("normalized distance map", dist_map_img);
				cv::waitKey(0);
			}

			//【8 生成立方体】
			// Generate cuboids
			MatrixXd all_configs_error_one_objH(200, 9);//误差
			MatrixXd all_box_corners_2d_one_objH(400, 8);//2D坐标
			int valid_config_number_one_objH = 0;//有效测量次数

			//STEP 【8.1采样相机的roll pitch角】
			std::vector<double> cam_roll_samples;
			std::vector<double> cam_pitch_samples;
			whether_sample_cam_roll_pitch = false;
			if (whether_sample_cam_roll_pitch)
			{
				linespace<double>(cam_pose_raw.euler_angle(0) - 6.0 / 180.0 * M_PI, cam_pose_raw.euler_angle(0) + 6.0 / 180.0 * M_PI, 3.0 / 180.0 * M_PI, cam_roll_samples);
				linespace<double>(cam_pose_raw.euler_angle(1) - 6.0 / 180.0 * M_PI, cam_pose_raw.euler_angle(1) + 6.0 / 180.0 * M_PI, 3.0 / 180.0 * M_PI, cam_pitch_samples);
			}
			else
			{
				cam_roll_samples.push_back(cam_pose_raw.euler_angle(0));
				cam_pitch_samples.push_back(cam_pose_raw.euler_angle(1));
			}
			// different from matlab. first for loop yaw, then for configurations.
			// 	      int obj_yaw_id=8;
//            cout<<"----------2"<<endl;
//			cout<<"roll, pitch, yaw: "<<cam_roll_samples.size()<<"\t"<<cam_pitch_samples.size()<<"\t"<<obj_yaw_samples.size()<<endl;
			for (int cam_roll_id = 0; cam_roll_id < cam_roll_samples.size(); cam_roll_id++)
				for (int cam_pitch_id = 0; cam_pitch_id < cam_pitch_samples.size(); cam_pitch_id++)
					for (int obj_yaw_id = 0; obj_yaw_id < obj_yaw_samples.size(); obj_yaw_id++)
					{
 						if (whether_sample_cam_roll_pitch)
						{
							Matrix4d transToWolrd_new = transToWolrd;
							transToWolrd_new.topLeftCorner<3, 3>() = euler_zyx_to_rot<double>(cam_roll_samples[cam_roll_id], cam_pitch_samples[cam_pitch_id], cam_pose_raw.euler_angle(2));
							set_cam_pose(transToWolrd_new);
							ground_plane_sensor = cam_pose.transToWolrd.transpose() * ground_plane_world;
						}

						double obj_yaw_esti = obj_yaw_samples[obj_yaw_id];

						//STEP【8.2计算三个消失点】
						///@param vp_1, vp_2, vp_3 三个消失点
						Vector2d vp_1, vp_2, vp_3;
						getVanishingPoints(cam_pose.KinvR, obj_yaw_esti, vp_1, vp_2, vp_3); // for object x y z  axis

						circle(image_point, cv::Point(vp_1(0)/5, vp_1(1))/5, 3, cv::Scalar(255,0,0),-1,8,0);

                        MatrixXd all_vps(3, 2);
						all_vps.row(0) = vp_1;
						all_vps.row(1) = vp_2;
						all_vps.row(2) = vp_3;
						//输出采样的航偏角

						//STEP【8.3寻找形成消失点的两条边】
						MatrixXd all_vp_bound_edge_angles = VP_support_edge_infos(all_vps, edge_mid_pts, lines_inobj_angles,
																				  Vector2d(vp12_edge_angle_thre, vp3_edge_angle_thre));
						// 		  int sample_top_pt_id=15;
//						cout<<"sample_top_pts: "<<sample_top_pts.cols()<<endl;
						for (int sample_top_pt_id = 0; sample_top_pt_id < sample_top_pts.cols(); sample_top_pt_id++)
						{
						    //STEP【8.4.1采样得到立方体上边缘的第一个点】
						    ///@param corner_1_top 当前采样点
							Vector2d corner_1_top = sample_top_pts.col(sample_top_pt_id);
							bool config_good = true;
							///@param vp_1_position 消失点1的位置
							int vp_1_position = 0; // 0 initial as fail,  1  on left   2 on right

                            // STEP 【8.4.2 计算立方体上边缘的第二个点.】
                            /// @param corner_2_top 消失点1-上边缘采样点射线与边界框左右边界的交点.
							Vector2d corner_2_top = seg_hit_boundary(vp_1, corner_1_top, Vector4d(right_x_raw, top_y_raw, right_x_raw, down_y_expan));
							if (corner_2_top(0) == -1)
							{ // vp1-corner1 doesn't hit the right boundary. check whether hit left
							    corner_2_top = seg_hit_boundary(vp_1, corner_1_top, Vector4d(left_x_raw, top_y_raw, left_x_raw, down_y_expan));
							    if (corner_2_top(0) != -1) // vp1-corner1 hit the left boundary   vp1 on the right
									vp_1_position = 2;
							}
							else // vp1-corner1 hit the right boundary   vp1 on the left
								vp_1_position = 1;

							config_good = vp_1_position > 0;
							if (!config_good)
							{
								if (print_details)
									printf("Configuration fails at corner 2, outside segment\n");
								continue;
							}
							if ((corner_1_top - corner_2_top).norm() < shorted_edge_thre)
							{
								if (print_details)
									printf("Configuration fails at edge 1-2, too short\n");
								continue;
							}
							//输出立方体上边缘第一二个点的位置
//							int config_ind=0; // have to consider config now.
							for (int config_id = 1; config_id < 3; config_id++) // configuration one or two of matlab version
							{
								if (!all_configs[config_id - 1])
									continue;
								Vector2d corner_3_top, corner_4_top;
								if (config_id == 1)
								{
									if (vp_1_position == 1) // then vp2 hit the left boundary
										corner_4_top = seg_hit_boundary(vp_2, corner_1_top, Vector4d(left_x_raw, top_y_raw, left_x_raw, down_y_expan));
									else // or, then vp2 hit the right boundary
										corner_4_top = seg_hit_boundary(vp_2, corner_1_top, Vector4d(right_x_raw, top_y_raw, right_x_raw, down_y_expan));
									if (corner_4_top(1) == -1)
									{
										config_good = false;
										if (print_details)
											printf("Configuration %d fails at corner 4, outside segment\n", config_id);
										continue;
									}
									if ((corner_1_top - corner_4_top).norm() < shorted_edge_thre)
									{
										if (print_details)
											printf("Configuration %d fails at edge 1-4, too short\n", config_id);
										continue;
									}
									// compute the last point in the top face
									corner_3_top = lineSegmentIntersect(vp_2, corner_2_top, vp_1, corner_4_top, true);
									if (!check_inside_box(corner_3_top, Vector2d(left_x_raw, top_y_raw), Vector2d(right_x_raw, down_y_expan)))
									{ // check inside boundary. otherwise edge visibility might be wrong
										config_good = false;
										if (print_details)
											printf("Configuration %d fails at corner 3, outside box\n", config_id);
										continue;
									}
									if (((corner_3_top - corner_4_top).norm() < shorted_edge_thre) || ((corner_3_top - corner_2_top).norm() < shorted_edge_thre))
									{
										if (print_details)
											printf("Configuration %d fails at edge 3-4/3-2, too short\n", config_id);
										continue;
									}
								}
								if (config_id == 2)
								{
									if (vp_1_position == 1) // then vp2 hit the left boundary
										corner_3_top = seg_hit_boundary(vp_2, corner_2_top, Vector4d(left_x_raw, top_y_raw, left_x_raw, down_y_expan));
									else // or, then vp2 hit the right boundary
										corner_3_top = seg_hit_boundary(vp_2, corner_2_top, Vector4d(right_x_raw, top_y_raw, right_x_raw, down_y_expan));
									if (corner_3_top(1) == -1)
									{
										config_good = false;
										if (print_details)
											printf("Configuration %d fails at corner 3, outside segment\n", config_id);
										continue;
									}
									if ((corner_2_top - corner_3_top).norm() < shorted_edge_thre)
									{
										if (print_details)
											printf("Configuration %d fails at edge 2-3, too short\n", config_id);
										continue;
									}
									// compute the last point in the top face
									corner_4_top = lineSegmentIntersect(vp_1, corner_3_top, vp_2, corner_1_top, true);
									if (!check_inside_box(corner_4_top, Vector2d(left_x_raw, top_y_expan_distmap), Vector2d(right_x_raw, down_y_expan_distmap)))
									{
										config_good = false;
										if (print_details)
											printf("Configuration %d fails at corner 4, outside box\n", config_id);
										continue;
									}
									if (((corner_3_top - corner_4_top).norm() < shorted_edge_thre) || ((corner_4_top - corner_1_top).norm() < shorted_edge_thre))
									{
										if (print_details)
											printf("Configuration %d fails at edge 3-4/4-1, too short\n", config_id);
										continue;
									}
								}
								// compute first bottom points    computing bottom points is the same for config 1,2
								Vector2d corner_5_down = seg_hit_boundary(vp_3, corner_3_top, Vector4d(left_x_raw, down_y_expan, right_x_raw, down_y_expan));
								if (corner_5_down(1) == -1)
								{
									config_good = false;
									if (print_details)
										printf("Configuration %d fails at corner 5, outside segment\n", config_id);
									continue;
								}
								if ((corner_3_top - corner_5_down).norm() < shorted_edge_thre)
								{
									if (print_details)
										printf("Configuration %d fails at edge 3-5, too short\n", config_id);
									continue;
								}
								Vector2d corner_6_down = lineSegmentIntersect(vp_2, corner_5_down, vp_3, corner_2_top, true);
								if (!check_inside_box(corner_6_down, expan_distmap_lefttop, expan_distmap_rightbottom))
								{
									config_good = false;
									if (print_details)
										printf("Configuration %d fails at corner 6, outside box\n", config_id);
									continue;
								}
								if (((corner_6_down - corner_2_top).norm() < shorted_edge_thre) || ((corner_6_down - corner_5_down).norm() < shorted_edge_thre))
								{
									if (print_details)
										printf("Configuration %d fails at edge 6-5/6-2, too short\n", config_id);
									continue;
								}
								Vector2d corner_7_down = lineSegmentIntersect(vp_1, corner_6_down, vp_3, corner_1_top, true);
								if (!check_inside_box(corner_7_down, expan_distmap_lefttop, expan_distmap_rightbottom))
								{ // might be slightly different from matlab
									config_good = false;
									if (print_details)
										printf("Configuration %d fails at corner 7, outside box\n", config_id);
									continue;
								}
								if (((corner_7_down - corner_1_top).norm() < shorted_edge_thre) || ((corner_7_down - corner_6_down).norm() < shorted_edge_thre))
								{
									if (print_details)
										printf("Configuration %d fails at edge 7-1/7-6, too short\n", config_id);
									continue;
								}
								Vector2d corner_8_down = lineSegmentIntersect(vp_1, corner_5_down, vp_2, corner_7_down, true);
								if (!check_inside_box(corner_8_down, expan_distmap_lefttop, expan_distmap_rightbottom))
								{
									config_good = false;
									if (print_details)
										printf("Configuration %d fails at corner 8, outside box\n", config_id);
									continue;
								}
								if (((corner_8_down - corner_4_top).norm() < shorted_edge_thre) || ((corner_8_down - corner_5_down).norm() < shorted_edge_thre) || ((corner_8_down - corner_7_down).norm() < shorted_edge_thre))
								{
									if (print_details)
										printf("Configuration %d fails at edge 8-4/8-5/8-7, too short\n", config_id);
									continue;
								}

								MatrixXd box_corners_2d_float(2, 8);
								box_corners_2d_float << corner_1_top, corner_2_top, corner_3_top, corner_4_top, corner_5_down, corner_6_down, corner_7_down, corner_8_down;
								MatrixXd box_corners_2d_float_shift(2, 8);
								box_corners_2d_float_shift.row(0) = box_corners_2d_float.row(0).array() - left_x_expan_distmap;
								box_corners_2d_float_shift.row(1) = box_corners_2d_float.row(1).array() - top_y_expan_distmap;

								MatrixXi visible_edge_pt_ids, vps_box_edge_pt_ids;
								double sum_dist;
								if (config_id == 1)
								{
									visible_edge_pt_ids.resize(9, 2);
									visible_edge_pt_ids << 1, 2, 2, 3, 3, 4, 4, 1, 2, 6, 3, 5, 4, 8, 5, 8, 5, 6;
									vps_box_edge_pt_ids.resize(3, 4);
									vps_box_edge_pt_ids << 1, 2, 8, 5, 4, 1, 5, 6, 4, 8, 2, 6; // six edges. each row represents two edges [e1_1 e1_2   e2_1 e2_2;...] of one VP
									visible_edge_pt_ids.array() -= 1;
									vps_box_edge_pt_ids.array() -= 1; //change to c++ index
									sum_dist = box_edge_sum_dists(dist_map, box_corners_2d_float_shift, visible_edge_pt_ids);
								}
								else
								{
									visible_edge_pt_ids.resize(7, 2);
									visible_edge_pt_ids << 1, 2, 2, 3, 3, 4, 4, 1, 2, 6, 3, 5, 5, 6;
									vps_box_edge_pt_ids.resize(3, 4);
									vps_box_edge_pt_ids << 1, 2, 3, 4, 4, 1, 5, 6, 3, 5, 2, 6; // six edges. each row represents two edges [e1_1 e1_2   e2_1 e2_2;...] of one VP
									visible_edge_pt_ids.array() -= 1;
									vps_box_edge_pt_ids.array() -= 1;
									sum_dist = box_edge_sum_dists(dist_map, box_corners_2d_float_shift, visible_edge_pt_ids, reweight_edge_distance);
								}
								double total_angle_diff = box_edge_alignment_angle_error(all_vp_bound_edge_angles, vps_box_edge_pt_ids, box_corners_2d_float);
								all_configs_error_one_objH.row(valid_config_number_one_objH).head<4>() = Vector4d(config_id, vp_1_position, obj_yaw_esti, sample_top_pt_id);
								all_configs_error_one_objH.row(valid_config_number_one_objH).segment<3>(4) = Vector3d(sum_dist / obj_diaglength_expan, total_angle_diff, down_expand_sample);
								if (whether_sample_cam_roll_pitch)
									all_configs_error_one_objH.row(valid_config_number_one_objH).segment<2>(7) = Vector2d(cam_roll_samples[cam_roll_id], cam_pitch_samples[cam_pitch_id]);
								else
									all_configs_error_one_objH.row(valid_config_number_one_objH).segment<2>(7) = Vector2d(cam_pose_raw.euler_angle(0), cam_pose_raw.euler_angle(1));
								all_box_corners_2d_one_objH.block(2 * valid_config_number_one_objH, 0, 2, 8) = box_corners_2d_float;
								valid_config_number_one_objH++;
								if (valid_config_number_one_objH >= all_configs_error_one_objH.rows())
								{
									all_configs_error_one_objH.conservativeResize(2 * valid_config_number_one_objH, NoChange);
									all_box_corners_2d_one_objH.conservativeResize(4 * valid_config_number_one_objH, NoChange);
								}
							} //end of config loop
						}	 //end of top id
					}		  //end of yaw

//            cout<<"----------3"<<endl;
			MatrixXd all_corners = all_box_corners_2d_one_objH.topRows(2*valid_config_number_one_objH);

			VectorXd normalized_score;
			vector<int> good_proposal_ids;
			fuse_normalize_scores_v2(all_configs_error_one_objH.col(4).head(valid_config_number_one_objH), all_configs_error_one_objH.col(5).head(valid_config_number_one_objH),
									 normalized_score, good_proposal_ids, weight_vp_angle, whether_normalize_two_errors);
//            cout<<"----------4"<<endl;
			for (int box_id = 0; box_id < good_proposal_ids.size(); box_id++)
			{
				int raw_cube_ind = good_proposal_ids[box_id];

				if (whether_sample_cam_roll_pitch)
				{
					Matrix4d transToWolrd_new = transToWolrd;
					transToWolrd_new.topLeftCorner<3, 3>() = euler_zyx_to_rot<double>(all_configs_error_one_objH(raw_cube_ind, 7), all_configs_error_one_objH(raw_cube_ind, 8), cam_pose_raw.euler_angle(2));
					set_cam_pose(transToWolrd_new);
					ground_plane_sensor = cam_pose.transToWolrd.transpose() * ground_plane_world;
				}

				cuboid *sample_obj = new cuboid();
				change_2d_corner_to_3d_object(all_box_corners_2d_one_objH.block(2 * raw_cube_ind, 0, 2, 8), all_configs_error_one_objH.row(raw_cube_ind).head<3>(),
											  ground_plane_sensor, cam_pose.transToWolrd, cam_pose.invK, cam_pose.projectionMatrix, *sample_obj);
				// 		  sample_obj->print_cuboid();
				if ((sample_obj->scale.array() < 0).any())
					continue; // scale should be positive
				sample_obj->rect_detect_2d = Vector4d(left_x_raw, top_y_raw, obj_width_raw, obj_height_raw);
				sample_obj->edge_distance_error = all_configs_error_one_objH(raw_cube_ind, 4); // record the original error
				sample_obj->edge_angle_error = all_configs_error_one_objH(raw_cube_ind, 5);
				sample_obj->normalized_error = normalized_score(box_id);
				double skew_ratio = sample_obj->scale.head(2).maxCoeff() / sample_obj->scale.head(2).minCoeff();
				sample_obj->skew_ratio = skew_ratio;
				sample_obj->down_expand_height = all_configs_error_one_objH(raw_cube_ind, 6);
				if (whether_sample_cam_roll_pitch)
				{
					sample_obj->camera_roll_delta = all_configs_error_one_objH(raw_cube_ind, 7) - cam_pose_raw.euler_angle(0);
					sample_obj->camera_pitch_delta = all_configs_error_one_objH(raw_cube_ind, 8) - cam_pose_raw.euler_angle(1);
				}
				else
				{
					sample_obj->camera_roll_delta = 0;
					sample_obj->camera_pitch_delta = 0;
				}

				raw_obj_proposals.push_back(sample_obj);
			}
		} // end of differnet object height sampling
//        cout<<"----------5"<<endl;
		// %finally rank all proposals. [normalized_error   skew_error]
		int actual_cuboid_num_small = std::min(max_cuboid_num, (int)raw_obj_proposals.size());
		VectorXd all_combined_score(raw_obj_proposals.size());
		for (int box_id = 0; box_id < raw_obj_proposals.size(); box_id++)
		{
			cuboid *sample_obj = raw_obj_proposals[box_id];
			double skew_error = weight_skew_error * std::max(sample_obj->skew_ratio - nominal_skew_ratio, 0.0);
			if (sample_obj->skew_ratio > max_cut_skew)
				skew_error = 100;
			double new_combined_error = sample_obj->normalized_error + weight_skew_error * skew_error;
			all_combined_score(box_id) = new_combined_error;
		}
//        cout<<"----------6"<<endl;
		std::vector<int> sort_idx_small(all_combined_score.rows());
		iota(sort_idx_small.begin(), sort_idx_small.end(), 0);
		sort_indexes(all_combined_score, sort_idx_small, actual_cuboid_num_small);
		for (int ii = 0; ii < actual_cuboid_num_small; ii++) // use sorted index
		{
			all_object_cuboids[object_id].push_back(raw_obj_proposals[sort_idx_small[ii]]);
		}
//        cout<<"----------7 object_id: "<<object_id<<endl;
//		ca::Profiler::tictoc("One 3D object total time");
	} // end of different objects
//    cout<<"----------8"<<endl;
	if (whether_plot_final_images || whether_save_final_images)
	{
		cv::Mat frame_all_cubes_img = rgb_img.clone();
		for (int object_id = 0; object_id < all_object_cuboids.size(); object_id++)
			if (all_object_cuboids[object_id].size() > 0)
			{
				plot_image_with_cuboid(frame_all_cubes_img, all_object_cuboids[object_id][0]);
			}
		if (whether_save_final_images)
			cuboids_2d_img = frame_all_cubes_img;
		if (whether_plot_final_images)
		{
			cv::imshow("frame_all_cubes_img", frame_all_cubes_img);
			cv::waitKey(0);
		}
	}
}
