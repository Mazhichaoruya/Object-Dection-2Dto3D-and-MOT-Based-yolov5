//
// Created by mzc on 2021/1/29.
//
#ifndef CLOUD_SAVE_CLUSTER_H
#define CLOUD_SAVE_CLUSTER_H

#include "include.h"
#include "transformer/transformer.h"
#include "DBSCAN/DBSCAN_kdtree.h"
#include "DBSCAN/DBSCAN_precomp.h"
#include "DBSCAN/DBSCAN_simple.h"
#include "PCKMeans/pckmeans.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common.h>
#include <pcl/segmentation/extract_clusters.h>
#include "visualizer/visualizer.h"
struct Oneclass{
    pcl::PointCloud<pcl::PointXYZI>::Ptr ponits3d;
    cv::Rect images;
    pcl::PointXYZ centerpoint;
};//聚类簇包含 点云和映射图区域
struct Resize_limt{
    int x_min,y_min,x_max,y_max;
};//边界类
struct ResultsOfCluster{
  std::vector<pcl::PointIndices> cluster_indices;//聚类点云
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_results;//全部点云 用于可视化
  std::map<int,Oneclass> pointcloud_clustered;//分类别存储点云
  cv::Mat ClusterRebuildImg;//重建的类别可视化图像
  int clustering_nums;//聚类类别数目
};
class DBSCAN{
private:
    ResultsOfCluster Roc;//聚类结果
    Eigen::Vector4d Intrinsics;//彩色相机内参
    Eigen::Isometry3d T_pose;//彩色相机外参
    cv::Size Imagesize;//扩展图像大小
    Resize_limt image_boundary;//在RGB像素坐标系下的边界
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloudf;//降采样后的全部点云
public:
    DBSCAN();
    DBSCAN(Eigen::Vector4d intrinsics,Eigen::Isometry3d T_rc);
    void DBSCAN_Cluster(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,cv::Size size);
    std::map<int,Oneclass> ReportClusteredResults();
    pcl::PointCloud<pcl::PointXYZI>::Ptr returnOriginpcl();
    pcl::PointCloud<pcl::PointXYZI>::Ptr returnClusterpcl();

};
#endif //CLOUD_SAVE_CLUSTER_H
