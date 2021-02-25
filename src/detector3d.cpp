//
// Created by mzc on 2021/2/22.
//

#include "detector/detector3d.h"
#include "kalman/kalman.h"
#include "cluster/cluster.h"
detector3d::detector3d(Eigen::Vector4d intrinsics,Eigen::Isometry3d T_rc,std::string weight_path,std::string classname_path) {
    Detector2D=Detector(weight_path,classname_path);
    M_in=intrinsics;
    M_out=T_rc;
}
void detector3d::Reclustering(pcl::PointCloud<pcl::PointXYZI>::Ptr pPntCloud,cv::Rect ro,cv::Rect rc,int classID,std::vector<pcl::PointXYZI> &objectpoints) {
    auto clusterKM = KMeans(M_in,M_out);
    clusterKM.SetK(3);
    clusterKM.SetInputCloud(pPntCloud,ro,rc);
    auto pointclouds = clusterKM.Cluster();
    for (auto p:pointclouds.at(0)) {
        pcl::PointXYZI pI;
        pI.x = p.pnt.x;
        pI.y = p.pnt.y;
        pI.z = p.pnt.z;
        pI.intensity = classID;
        objectpoints.push_back(pI);
        Output3D->points.push_back(pI);
    }
}
std::vector<Detection> detector3d::ReportResults() {
//    std::cout<<"num:"<<objects.size()<<std::endl;
    for (auto &object:objects){
        middata m{0,0,0,10,10,10,0,0,0};
        for (auto p:object.points){
            if (m.max_x<p.x)
                m.max_x=p.x;
            if (m.max_y<p.y)
                m.max_y=p.y;
            if (m.max_z<p.z)
                m.max_z=p.z;
            if (m.min_x>p.x)
                m.min_x=p.x;
            if (m.min_y>p.y)
                m.min_y=p.y;
            if (m.min_z>p.z)
                m.min_z=p.z;
            m.sum_x+=p.x;
            m.sum_y+=p.y;
            m.sum_z+=p.z;
        }
        object.p={m.sum_x/object.points.size(),m.sum_y/object.points.size(),m.sum_z/object.points.size()};
        object.s={m.max_x-m.min_x,m.max_y-m.min_y,m.max_y-m.min_y};
        object.viz=m;
    }
    return objects;
}
void detector3d::SetInput(cv::Mat &img, pcl::PointCloud<pcl::PointXYZI>::Ptr &pointcloud) {
    auto clusterDB=DBSCAN(M_in,M_out);
    clusterDB.DBSCAN_Cluster(pointcloud,img.size());
    Output2D=Detector2D.Detection(img);
    Output3D.reset(new pcl::PointCloud<pcl::PointXYZI>);
    Output3D=pointcloud;
    auto c=clusterDB.ReportClusteredResults();
    auto d=Detector2D.ReportObjects();
    Matcher matcher(c,d);
    auto result=matcher.BoxIouMatch();
    objects.clear();//清空objects向量
    for (auto res:result) {
        Detection object;
        object.ClassID=d.at(res.first.at(0)).classID;
        object.classname=d.at(res.first.at(0)).name;
        if (abs(res.second) < IOU_Threshold_min) {
//                cout<<"failed"<<endl;
            continue;
        }
        if (abs(res.second) > IOU_Threshold_max ||
            abs(res.second) > (IOU_Threshold_min + IOU_Threshold_max) / 2 && res.second < 0) {
            for (auto p:c.at(res.first.at(1)).ponits3d->points) {
                p.intensity = d.at(res.first.at(0)).classID + 1;
                Output3D->points.push_back(p);
                object.points.push_back(p);
            }
        }
        if (abs(res.second) < IOU_Threshold_max && abs(res.second) > IOU_Threshold_min &&
            res.second > 0) {//半监督聚类提升效果
            Reclustering(c.at(res.first.at(1)).ponits3d,d.at(res.first.at(0)).area, c.at(res.first.at(1)).images,d.at(res.first.at(0)).classID+1,object.points);
        }
        if (object.points.size()>0)
            objects.push_back(object);
    }

}
void detector3d::destory() {
    Detector2D.destory();
}
void detector3d::showimg(){
    cv::imshow("Color",Output2D);
    cv::waitKey(1);
}