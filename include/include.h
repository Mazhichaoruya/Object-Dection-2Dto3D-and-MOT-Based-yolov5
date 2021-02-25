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
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include "map"
#include "algorithm"
#include "kalman/kalman.h"

#define MAX_X 4.0
#define MAX_Y 4.0
#define MAX_Z 4.0
#define IOU_Threshold_max 0.5
#define IOU_Threshold_min 0.1//IOU匹配阈值
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
    int ClassID;
    int ID;
    Position p;
    Size s;
    std::vector<pcl::PointXYZI> points;
    middata viz;
    std::string classname;
};

struct track{
    int ID;//track ID
    int Nomatched;
    bool enable;
    int ClassID;
    int age;//已创建周期
    Size s;//跟踪目标尺寸
    Position p;//跟踪目标的中心位置
    Velocity v;//目标当前速度
    Kalmanf kf;
    middata viz;
    std::vector<pcl::PointXYZI> points;
    std::string classname;
    std::vector<cv::Point3f> trace;
};
#endif //CLOUD_SAVE_INCLUDE_H
