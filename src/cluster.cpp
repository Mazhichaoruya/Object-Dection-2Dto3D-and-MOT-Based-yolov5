//
// Created by mzc on 2021/1/30.
//
#include "include.h"
#include "cluster/cluster.h"
#include "cluster/PCKMeans/pckmeans.h"
DBSCAN::DBSCAN(Eigen::Vector4d intrinsics,Eigen::Isometry3d T_rc){
//构造函数 传入RGB相机的内外参数 图像尺寸大小
Intrinsics=intrinsics;
T_pose=T_rc;
Roc.cloud_results.reset(new pcl::PointCloud<pcl::PointXYZI>);
pointcloudf.reset(new pcl::PointCloud<pcl::PointXYZI>);
}
DBSCAN::DBSCAN() {

}
//DBSCAN密度聚类
void DBSCAN::DBSCAN_Cluster(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,cv::Size size) {
//      show_point_cloud(cloud,"Original--");
    //降采样 体素5cm
    Imagesize.height=size.height*2;
    Imagesize.width=size.width*2;
    image_boundary.x_max=Imagesize.width-size.width/2;
    image_boundary.y_max=Imagesize.height-size.height/2;
    image_boundary.x_min=-size.width/2;
    image_boundary.y_min=-size.height/2;


    pcl::VoxelGrid<pcl::PointXYZI> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.05f, 0.05f, 0.05f);
    vg.filter(*cloud);
    pointcloudf=cloud;
//      show_point_cloud(cloud,"After downsample--");
    //KD-tree加速的密度聚类
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new  pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(cloud);
    DBSCANKdtreeCluster<pcl::PointXYZI> ec;
    ec.setCorePointMinPts(10);
    ec.setClusterTolerance(0.1);
    ec.setMinClusterSize(20);
    ec.setMaxClusterSize(2000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(Roc.cluster_indices);
    int classindex=0;//标记类别编号
    Roc.clustering_nums=Roc.cluster_indices.size();
//    std::cout<<"After downsampled: "<<cloud->size()<<std::endl;
//    std::cout<<"Nums of Clusters: "<<Roc.cluster_indices.size()<<std::endl;
//      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_clustered(new pcl::PointCloud<pcl::PointXYZI>);
    Roc.ClusterRebuildImg=cv::Mat::zeros(Imagesize.height,Imagesize.width,CV_8UC(1));//重建聚类图
    for (auto it:Roc.cluster_indices) {
        Roc.pointcloud_clustered[classindex].ponits3d.reset(new pcl::PointCloud<pcl::PointXYZI>);//点云记得重置
        int minx = Imagesize.width, miny = Imagesize.width, maxx = 0,maxy =0 ;//寻找相机投影下的边框
        for (auto pit:it.indices){
            auto temp=cloud->points.at(pit);
            temp.intensity=classindex;
            Roc.cloud_results->points.push_back(temp);
            auto Pix=PointConvertToPix(temp,Intrinsics,T_pose);//Pix属于像平面
            Eigen::Vector2i pix={static_cast<int>(Pix.x())-image_boundary.x_min,static_cast<int>(Pix.y())-image_boundary.y_min};//pix转为 像素坐标
            if (Pix.x()>image_boundary.x_min&&Pix.x()<image_boundary.x_max&&Pix.y()>image_boundary.y_min&&Pix.y()<image_boundary.y_max&&Pix.z()>0){//像素区域内
                Roc.pointcloud_clustered[classindex].ponits3d->points.push_back(temp);
                Roc.pointcloud_clustered[classindex].centerpoint.x+=temp.x;
                Roc.pointcloud_clustered[classindex].centerpoint.y+=temp.y;
                Roc.pointcloud_clustered[classindex].centerpoint.z+=temp.z;
                Roc.ClusterRebuildImg.at<uint8_t>(pix.y(),pix.x())=(classindex+1)*255/Roc.clustering_nums;
                if (minx > pix.x()) minx = pix.x();if (miny > pix.y()) miny = pix.y();if (maxx < pix.x()) maxx = pix.x();if (maxy < pix.y()) maxy = pix.y();//寻找边界
            }
        }
        if (minx <640) minx = 640;if (miny < 320) miny = 320;if (maxx >1920) maxx = 1920;if (maxy >1080) maxy = 1080;//限制矩形框大小位于RGB图像范围内
        if (!Roc.pointcloud_clustered[classindex].ponits3d->points.empty()){
            Roc.pointcloud_clustered[classindex].images=cv::Rect(minx,miny,(maxx - minx), (maxy - miny));
            Roc.pointcloud_clustered[classindex].centerpoint.x/=Roc.pointcloud_clustered[classindex].ponits3d->points.size();
            Roc.pointcloud_clustered[classindex].centerpoint.y/=Roc.pointcloud_clustered[classindex].ponits3d->points.size();
            Roc.pointcloud_clustered[classindex].centerpoint.z/=Roc.pointcloud_clustered[classindex].ponits3d->points.size();
        }
        classindex++;
//        std::cout<<classindex<<std::endl;
    }
    //可视化
/*
    for (auto img:Roc.pointcloud_clustered) {
        //绘制聚类边界框
        cv::rectangle(Roc.ClusterRebuildImg,img.second.images,255,1);
    }
    cv::resize(Roc.ClusterRebuildImg,Roc.ClusterRebuildImg,cv::Size(1280,720));
    cv::imshow("Clustered Image",Roc.ClusterRebuildImg);//可视化点云聚类后在像平面的图像
    cv::waitKey(0);
    */
//    show_point_cloud(Roc.cloud_results,"After DBSCAN--");//可视化点云

}
std::map<int,Oneclass> DBSCAN::ReportClusteredResults(){
    return Roc.pointcloud_clustered;
}
pcl::PointCloud<pcl::PointXYZI>::Ptr DBSCAN::returnOriginpcl(){
    return pointcloudf;
}
pcl::PointCloud<pcl::PointXYZI>::Ptr DBSCAN::returnClusterpcl(){
    return Roc.cloud_results;
}
//PCKMeans

KMeans::KMeans(Eigen::Vector4d intrinsics, Eigen::Isometry3d T_rc){
    Intrinsics=intrinsics;
    T_pose=T_rc;
}
//KMeans::KMeans(){
//
//}
bool KMeans::InitKCenter()//m_k ==3
{
    mv_center.resize(m_k);
    int size = mv_pntcloud.size();
    /*
    ////随机初始化
     srand(unsigned(time(NULL)));
     for (int i =0; i< m_k;i++)
     {
         int seed = random()%(size+1);
         mv_center[i].x = mv_pntcloud[seed].pnt.x;
         mv_center[i].y = mv_pntcloud[seed].pnt.y;
         mv_center[i].z = mv_pntcloud[seed].pnt.z;
     }
     */
    pcl::PointXYZI point3d;
    st_pointxyz center;//目标点云中心点
    center.x=0;center.y=0;center.z=0;int sizeofobpoint=0;
    std::vector<Eigen::Vector3d> vec_pix;
    float last_in=MAXFLOAT,last_out_max=0.0,last_in_max=0.0,centerx,centery;
    centerx=rectobj.x+rectobj.width/2;centery=rectobj.y+rectobj.height/2;
    float r1=(abs(rectclu.y-rectobj.y)+rectclu.height/2)*(abs(rectclu.y-rectobj.y)+rectclu.height/2)+(abs(rectclu.x-rectobj.x)+rectclu.width/2)*(abs(rectclu.x-rectobj.x)+rectclu.width/2);
    float r2=(rectobj.height/2)*(rectobj.height/2)+(rectobj.width/2)*(rectobj.width/2);
    for(auto p:mv_pntcloud){
        point3d.x = p.pnt.x;
        point3d.y = p.pnt.y;
        point3d.z = p.pnt.z;
        Eigen::Vector3d pix=PointConvertToPix(point3d,Intrinsics,T_pose);
        vec_pix.push_back(pix);
        if (pix.x()>=rectobj.x&&pix.y()>=rectobj.y&&pix.x()<rectobj.x+rectobj.width&&pix.y()<rectobj.y+rectobj.height){
            center.x+=p.pnt.x;
            center.y+=p.pnt.y;
            center.z+=p.pnt.z;
            sizeofobpoint++;
        }
    }
    center.x/=sizeofobpoint; center.y/=sizeofobpoint; center.z/=sizeofobpoint;
    for(int i=0;i<mv_pntcloud.size();++i) {
        auto distance=DistBetweenPoints(mv_pntcloud.at(i).pnt,center);
        if (vec_pix.at(i).x()>=rectobj.x&&vec_pix.at(i).y()>=rectobj.y&&vec_pix.at(i).x()<rectobj.x+rectobj.width&&vec_pix.at(i).y()<rectobj.y+rectobj.height){//落在目标检测框内的点云
            mv_pntcloud.at(i).connection_constraint={-2,2,0};  //目标 框内增加必连约束 0为目标类 1为非目标点 2为其他类
            if(distance<last_in){//目标类起始点是目标框内最近点
                mv_center.at(0)=mv_pntcloud.at(i).pnt;
                last_in=distance;
            }
            if(distance>last_in_max){//其他点选择目标区域内最远的点
                mv_center.at(2)=mv_pntcloud.at(i).pnt;
                last_in_max=distance;
            }
            mv_pntcloud.at(i).weight=1-((vec_pix.at(i).x()-centerx)*(vec_pix.at(i).x()-centerx)+(vec_pix.at(i).y()-centery)*(vec_pix.at(i).y()-centery))/r2;//在目标区域内的约束权重以单位圆均匀分布
        }
        else{
            mv_pntcloud.at(i).connection_constraint={2,-2,0};//目标 框外增加勿连约束
            if(distance>last_out_max){ //非目标点--框外最远点作为起始点
                mv_center.at(1)=mv_pntcloud.at(i).pnt;
                last_out_max=distance;
            }
            mv_pntcloud.at(i).weight=((vec_pix.at(i).x()-centerx)*(vec_pix.at(i).x()-centerx)+(vec_pix.at(i).y()-centery)*(vec_pix.at(i).y()-centery))/r1;//在目标区域外的约束权重以单位圆均匀分布
        }
    }
    return true;
}
bool KMeans::SetInputCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr pPntCloud,cv::Rect ro,cv::Rect rc)
{
    rectclu=rc;
    rectobj=ro;
    size_t pntCount = (size_t) pPntCloud->points.size();
    for (size_t i = 0; i< pntCount;++i)
    {
        st_point point;
        point.pnt.x = pPntCloud->points[i].x;
        point.pnt.y = pPntCloud->points[i].y;
        point.pnt.z = pPntCloud->points[i].z;
        point.groupID = 0;

        mv_pntcloud.push_back(point);
    }

    return true;
}
std::vector<VecPoint_t> KMeans::Cluster()
{
    InitKCenter();
    std::vector<st_pointxyz>v_center(mv_center.size());
    size_t pntCount = mv_pntcloud.size();

    do
    {
        for (size_t i = 0;i < pntCount;++i)
        {
            double min_dist = DBL_MAX;
            int pnt_grp = 0;   //聚类群组索引号
            for (size_t j =0;j <m_k;++j)
            {
//                double dist = DistBetweenPoints(mv_pntcloud[i].pnt, mv_center[j]);//
                double dist = DistBetweenPoints(mv_pntcloud[i].pnt, mv_center[j])+mv_pntcloud.at(i).connection_constraint[j]*mv_pntcloud.at(i).weight;//增加成对约束权重因素
                if (min_dist - dist > 0.000001)
                {
                    min_dist = dist;
                    pnt_grp = j;
                }
            }
            m_grp_pntcloud[pnt_grp].push_back(st_point(mv_pntcloud[i].pnt,pnt_grp)); //将该点和该点群组的索引存入聚类中
        }

        //保存上一次迭代的中心点
        for (size_t i = 0; i<mv_center.size();++i)
        {
            v_center[i] = mv_center[i];
        }

        mv_center=UpdateGroupCenter(m_grp_pntcloud,mv_center);
        if ( !ExistCenterShift(v_center, mv_center))
        {
            break;
        }
        for (size_t i = 0; i < m_k; ++i){
            m_grp_pntcloud[i].clear();
        }

    }while(true);

    return m_grp_pntcloud;
}
double KMeans::DistBetweenPoints(st_pointxyz &p1, st_pointxyz &p2)
{
    double dist = 0;
    double x_diff = 0, y_diff = 0, z_diff = 0;

    x_diff = p1.x - p2.x;
    y_diff = p1.y - p2.y;
    z_diff = p1.z - p2.z;
//    dist = sqrt(x_diff * x_diff + y_diff * y_diff + z_diff * z_diff);
    dist = x_diff * x_diff + y_diff * y_diff + z_diff * z_diff;//不必使用开方运算增加运算量

    return dist;
}
std::vector<st_pointxyz> KMeans::UpdateGroupCenter(std::vector<VecPoint_t> &grp_pntcloud, std::vector<st_pointxyz> center)
{
    for (size_t i = 0; i < m_k; ++i)
    {
        float x = 0, y = 0, z = 0;
        size_t pnt_num_in_grp = grp_pntcloud[i].size();

        for (size_t j = 0; j < pnt_num_in_grp; ++j)
        {
            x += grp_pntcloud[i][j].pnt.x;
            y += grp_pntcloud[i][j].pnt.y;
            z += grp_pntcloud[i][j].pnt.z;
        }
        x /= pnt_num_in_grp;
        y /= pnt_num_in_grp;
        z /= pnt_num_in_grp;
        center[i].x = x;
        center[i].y = y;
        center[i].z = z;
    }
    return center;

}
//是否存在中心点移动
bool KMeans::ExistCenterShift(std::vector<st_pointxyz> &prev_center, std::vector<st_pointxyz> &cur_center)
{
    for (size_t i = 0; i < m_k; ++i)
    {
        double dist = DistBetweenPoints(prev_center[i], cur_center[i]);
        if (dist > DIST_NEAR_ZERO)
        {
            return true;
        }
    }

    return false;
}

bool KMeans::SaveFile(const char *prex_name)
{
    for (int i = 0; i < m_k; ++i)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr p_pnt_cloud(new pcl::PointCloud<pcl::PointXYZ> ());

        for (size_t j = 0, grp_pnt_count = m_grp_pntcloud[i].size(); j < grp_pnt_count; ++j)
        {
            pcl::PointXYZ pt;
            pt.x = m_grp_pntcloud[i][j].pnt.x;
            pt.y = m_grp_pntcloud[i][j].pnt.y;
            pt.z = m_grp_pntcloud[i][j].pnt.z;

            p_pnt_cloud->points.push_back(pt);
        }

        p_pnt_cloud->width = (int)m_grp_pntcloud[i].size();
        p_pnt_cloud->height = 1;

        char newFileName[256] = {0};
        char indexStr[16] = {0};

        strcat(newFileName, "szFileName");
        strcat(newFileName, "-");
        strcat(newFileName, prex_name);
        strcat(newFileName, "-");
        sprintf(indexStr, "%d", i + 1);
        strcat(newFileName, indexStr);
        strcat(newFileName, ".pcd");
        pcl::io::savePCDFileASCII(newFileName, *p_pnt_cloud);
    }

    return true;
}