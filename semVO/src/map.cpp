////
//// Created by jixingwu on 2020/3/11.
////
//
//#include "map.h"
//#include "SubscribeAndPublish.h"
//using namespace Eigen;
//using namespace std;
//
////void SubscribeAndPublish::keyframe_image_callback(const sensor_msgs::ImageConstPtr &img_msg)
////{
////    cv::Mat keyframe_image = getImageFromMsg(img_msg);
////}
////
////void SubscribeAndPublish::leftImage_callback(const sensor_msgs::ImageConstPtr &img_msg)
////{
////    cv::Mat leftImage = getImageFromMsg(img_msg);
////}
////
////
////
////void SubscribeAndPublish::keyframe_bboxes_callback(const darknet_ros_msgs::BoundingBoxes &keyframe_bboxes)
////{
////
////}
////
////void SubscribeAndPublish::frame_bboxes_callback(const darknet_ros_msgs::BoundingBoxes &frame_bboxes)
////{
////
////}
//
//
//// one cuboid need front and back markers...  rgbcolor is 0-1 based
//visualization_msgs::MarkerArray cuboids_to_marker(object_landmark* obj_landmark, Vector3d rgbcolor)
//{
//    visualization_msgs::MarkerArray plane_markers;  visualization_msgs::Marker marker;
//    if (obj_landmark==nullptr)
//        return plane_markers;
//
//    marker.header.frame_id="/world";  marker.header.stamp=ros::Time::now();
//    marker.id = 0; //0
//    marker.type = visualization_msgs::Marker::LINE_STRIP;   marker.action = visualization_msgs::Marker::ADD;
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
//
//void cuboid_corner_to_marker(const Matrix38d& cube_corners,visualization_msgs::Marker& marker, int bodyOrfront)
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
//cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
//{
//    cv_bridge::CvImageConstPtr ptr;
//    if (img_msg->encoding == "8UC1")
//    {
//        sensor_msgs::Image img;
//        img.header = img_msg->header;
//        img.height = img_msg->height;
//        img.width = img_msg->width;
//        img.is_bigendian = img_msg->is_bigendian;
//        img.step = img_msg->step;
//        img.data = img_msg->data;
//        img.encoding = "mono8";
//        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
//    }
//    else
//        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
//
//    cv::Mat img = ptr->image.clone();
//    return img;
//}
//
//void SubscribeAndPublish::publish_all_poses(std::vector<object_landmark*> cube_landmark_history)
//{
//    visualization_msgs::MarkerArray finalcube_makers = cuboids_to_marker(cube_landmark_history.back(), Eigen::Vector3d(0,1,0));
//
//    while(n.ok())
//    {
//        pub_frame_raw_cube.publish(finalcube_makers);
//        ros::Rate loop_rate(5);
//        ros::spinOnce();
//        loop_rate.sleep();
//    }
//}
//
//void SubscribeAndPublish::publish_image(string dataPath)
//{
//    FILE *file;
//    file = std::fopen((dataPath + "times.txt").c_str(), "r");
//    if(file == NULL)
//    {
//        printf("cannot find file: %stimes.txt\n", dataPath.c_str());
//        ROS_BREAK();
//    }
//    double imageTime;
//    std::vector<double> imageTimeList;
//    while(fscanf(file, "%lf", &imageTime) != EOF)
//    {
//        imageTimeList.push_back(imageTime);
//    }
//    std::fclose(file);
//
//    string leftImagePath, rightImagePath;
//    cv::Mat imLeft, imRight;
//    for (size_t i = 0; i < imageTimeList.size(); ++i)
//    {
//        if(ros::ok())
//        {
//            printf("\n process image %d\n", (int)i);
//            stringstream ss;
//            ss<<setfill('0')<<setw(6)<<i;
//            leftImagePath = dataPath + "image_0/" + ss.str() + ".png";
//            rightImagePath = dataPath + "image_1/" + ss.str() + ".png";
//
//            imLeft = cv::imread(leftImagePath, CV_LOAD_IMAGE_GRAYSCALE);
//            sensor_msgs::ImagePtr imLeftMsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", imLeft).toImageMsg();
//            imLeftMsg->header.stamp = ros::Time(imageTimeList[i]);
//            pub_left_image.publish(imLeftMsg);
//
//            imRight = cv::imread(rightImagePath, CV_LOAD_IMAGE_GRAYSCALE);
//            sensor_msgs::ImagePtr imRightMsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", imRight).toImageMsg();
//            imRightMsg->header.stamp = ros::Time(imageTimeList[i]);
//            pub_right_image.publish(imRightMsg);
//        }
//
//    }
//}
//
//
