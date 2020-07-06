//
// Created by jixingwu on 2020/6/5.
//

#include "DataSemNode.h"

void DataSemNode::setCubePoseFromMsg(const nav_msgs::OdometryConstPtr msg) {
    std::lock_guard<std::mutex> lk(m);

    PoseManipUtils::geometry_msgs_Pose_to_eigenmat(msg->pose.pose, wTc);

    wTc_covariance = MatrixXd::Zero(6,6);
    for (int i = 0; i < 36; ++i) {
        wTc_covariance(int(i/6), i%6) = msg->pose.covariance[i];
    }

    t_wTc = msg->header.stamp;
    m_wTc = true;
}

void DataSemNode::setPose(const Matrix4d __wTc) {
    std::lock_guard<std::mutex> lk(m);

    wTc_covariance = MatrixXd::Zero(6, 6);
    wTc = Matrix4d(__wTc);

    m_wTc = true;
}

void DataSemNode::setPose(const ros::Time __t, const Matrix4d __wTc) {
    std::lock_guard<std::mutex> lk(m);

    wTc_covariance = MatrixXd::Zero(6,6);
    wTc = Matrix4d(__wTc);

    t_wTc = __t;
    m_wTc = true;
}
