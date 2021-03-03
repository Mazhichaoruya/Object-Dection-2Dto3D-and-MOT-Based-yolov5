//
// Created by mzc on 2021/1/29.
//

#ifndef CLOUD_SAVE_VISUALIZER_H
#define CLOUD_SAVE_VISUALIZER_H
#include "include.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include "opencv2/viz.hpp"
#include "tracker/tracker.h"
template <typename PointCloudPtrType>
void show_point_cloud(PointCloudPtrType cloud, std::string display_name) {//pcl点云可视化
    pcl::visualization::CloudViewer viewer(display_name);
    viewer.showCloud(cloud);
//    viewer.wasStopped(1000);
    while (!viewer.wasStopped())
    {
//        if (cv::waitKey(0))
//            break;
    }
}
void showImg(cv::Mat &color, std::map<int,track> &tracks);
void showCloud(cv::viz::Viz3d& win, std::map<int,track> &tracks, pcl::PointCloud<pcl::PointXYZI>::Ptr &pointcloud);
void viewerRunner(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer);
void show_point_cloud_cv(cv::Mat &pointcloud,std::string displayname);
void show_tracks_update(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer,std::map<int,track> &tracks, pcl::PointCloud<pcl::PointXYZI>::Ptr &pointcloud);
#endif //CLOUD_SAVE_VISUALIZER_H
