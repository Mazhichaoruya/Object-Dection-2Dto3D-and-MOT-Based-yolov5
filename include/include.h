//
// Created by mzc on 2021/1/27.
//

#ifndef CLOUD_SAVE_INCLUDE_H
#define CLOUD_SAVE_INCLUDE_H

#include "Eigen/Dense"
#include "opencv2/core.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/video/tracking.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include "map"
#include "algorithm"
#include "kalman/kalman.h"
#include "transformer/public_type.h"
#define MAX_X 4.5
#define MAX_Y 3.5
#define MAX_Z 2.0
#define IOU_Threshold_max 0.6
#define IOU_Threshold_min 0.01//IOU匹配阈值
#define Fx 541.28 //焦距
#define Fy 540.30
#define Mid_h 640
#define Mid_v 360
#define Pose 0
#define S_w 0.6//人的尺寸假设 高1.7m 宽0.6m
#define S_h 1.7

extern costmap::StdStamp time_stamp;

struct middata{
    float max_x,max_y,max_z;
    float min_x,min_y,min_z;
    float sum_x,sum_y,sum_z;
};

struct Position{
    float x;
    float y;
    float z;
};
struct Size{
    float l;
    float w;
    float h;
};
struct Velocity{
    float vx;
    float vy;
    float vz;
};
struct Detection{
    int ClassID;//类别编号
    int ID;//track编号
    Position p;//中心点坐标
    Size s;//物体尺寸
    std::vector<pcl::PointXYZI> points;//类别点云
    middata viz;//可视化使用--恢复尺寸的坐标
    std::string classname;//类别名
    cv::Rect Bbox;//目标2D区域
    bool Information_3D;//是否含有测量的3D信息
};

struct track{
    int ID;//track ID
    int Nomatched;//丢失匹配时间
    int lost3D;//丢失3D信息时间
    bool enable;//有效标志位 --true有效（report均为有效track ）
    bool Information_3D;//tracker中是否含有测量预测的3D信息
    int ClassID;//类别编号
    int age;//已创建周期
    Size s;//跟踪目标尺寸
    Position p,pe;//跟踪目标的中心位置
    Velocity v,ve;//目标当前速度
    cv::Rect Bbox;//tracker 2D区域
    Kalmanf kf;//单个目标对应的kf
    middata viz;//用于3D可视化
    std::vector<pcl::PointXYZI> points;//3D点云信息
    std::string classname;//类别名称
    std::vector<cv::Point3f> trace;//3D中心点历史轨迹
};
struct  track_report{
    int ID;//跟踪ID号
    costmap::StdStamp TimeStamp;//时间戳信息
    int classID;//类别ID
    std::string classname;//类别名称
    int age;//生存周期
    bool Information_3D;//3D信息标志位
    cv::Point3f position,velocity,size;//3D信息下 位置 速度 尺寸
    cv::Point3f position_e,velocity_e;//2D 估计信息 位置 和速度 定性表示 ：上下左右 速度朝向
    cv::Rect  bbox;//2D RGB图像跟踪bbox
    std::vector<cv::Point3f> trace,pointcloud;//历史轨迹 当前点云信息
};
#endif //CLOUD_SAVE_INCLUDE_H
