//
// Created by mzc on 2021/2/22.
//

#ifndef DETECTOR_DETECTOR3D_H
#define DETECTOR_DETECTOR3D_H
#include "include.h"
#include "detector/detector.h"
#include "matcher/Iou_matching.h"
#include "matcher/matcher.h"
class detector3d {
private:
    std::vector<Detection> objects;
    pcl::PointCloud<pcl::PointXYZI>::Ptr Output3D;
    cv::Mat Output2D;
//    DBSCAN clusterDB;
    Detector Detector2D;
    Eigen::Vector4d M_in;
    Eigen::Isometry3d M_out;
public:
    detector3d(Eigen::Vector4d intrinsics,Eigen::Isometry3d T_rc,std::string weight_path,std::string classname_path);
    void Reclustering(pcl::PointCloud<pcl::PointXYZI>::Ptr pPntCloud,cv::Rect ro,cv::Rect rc,int classID,std::vector<pcl::PointXYZI> &objectpoints);
    std::vector<Detection> ReportResults();
    void SetInput(cv::Mat &img,pcl::PointCloud<pcl::PointXYZI>::Ptr &pointcloud);
    void destory();
    void showimg();
};


#endif //DETECTOR_DETECTOR3D_H
