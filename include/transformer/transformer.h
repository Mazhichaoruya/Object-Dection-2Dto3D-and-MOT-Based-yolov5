//
// Created by mzc on 2021/1/29.
//

#ifndef CLOUD_SAVE_TRANSFORMER_H
#define CLOUD_SAVE_TRANSFORMER_H
#include "include.h"
//获取相机外参数转换矩阵
extern Eigen::Isometry3d M_out;
extern Eigen::Vector4d M_in;
extern Eigen::Transform<float, 3, 2, 0> TR;
Eigen::Isometry3d getTFMatrix(const Eigen::Vector3d& rotate/*roll, pitch, yaw*/, const Eigen::Vector3d& translate);
cv::Mat ConvertDepthToPointCLoud(const cv::Mat& depth, cv::Mat &mask,Eigen::Vector4d intrinsics, Eigen::Isometry3d T_rc);
Eigen::Vector3d PointConvertToPix(pcl::PointXYZI &point,Eigen::Vector4d intrinsics, Eigen::Isometry3d T_rc);
#endif //CLOUD_SAVE_TRANSFORMER_H
