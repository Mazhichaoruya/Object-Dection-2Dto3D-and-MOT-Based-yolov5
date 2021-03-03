//
// Created by mzc on 2021/1/30.
//
#include "include.h"
#include "visualizer/visualizer.h"
#include "filter/imagefilter.h"
#include "transformer/public_type.h"
//可视化工具函数

static cv::viz::Color color_table[15] = {
        //cv::viz::Color::white(),
        cv::viz::Color::red(),cv::viz::Color::green(), cv::viz::Color::blue(),
        cv::viz::Color::orange_red(), cv::viz::Color::indigo(), cv::viz::Color::pink(),
        cv::viz::Color::navy(),cv::viz::Color::olive(), cv::viz::Color::maroon(),
        cv::viz::Color::teal(),cv::viz::Color::rose(), cv::viz::Color::azure(),
        cv::viz::Color::lime(),cv::viz::Color::gold(), cv::viz::Color::brown()
};

void show_point_cloud_cv(cv::Mat &pointcloud,std::string displayname){
//    cv::imshow("2D_"+displayname,pointcloud);
    cv::viz::WCloud Cloud(pointcloud);
    cv::viz::Viz3d viewer3d("3D_"+displayname);
    viewer3d.showWidget(displayname,Cloud);
//    cv::waitKey(0);
    while (!viewer3d.wasStopped()) {
    viewer3d.spinOnce(1, true);
    }
}
void showCloud(cv::viz::Viz3d& win, std::map<int,track> &tracks, pcl::PointCloud<pcl::PointXYZI>::Ptr &pointcloud)
{
    win.removeAllWidgets();
    cv::viz::WCloudCollection collection;
    cv::Size size{int(pointcloud->points.size()),1};
    cv::Mat cloud(size, CV_32FC3, cv::Scalar::all(0));
    for (int i = 0; i < pointcloud->points.size(); ++i) {
        cloud.at<cv::Vec3f>(0,i)={pointcloud->points.at(i).x,pointcloud->points.at(i).y,pointcloud->points.at(i).z};
    }
    collection.addCloud(cloud);
    std::vector<cv::viz::WCube> cubes;
    std::vector<cv::viz::WArrow> arrows;
    std::vector<cv::viz::WText3D> texts;
    std::vector<cv::viz::WPolyLine> traces;
//    std::vector<cv::viz::WSphere> spheres;
    for (auto trc:tracks){
        //分track添加cube arrow text
        cv::viz::WPolyLine trace(trc.second.trace,color_table[trc.second.ID%15]);
        if (trc.second.Information_3D&&trc.second.Nomatched==0){
//            cv::viz::WSphere sphere(cv::Point3f(trc.second.p.x,trc.second.p.y,trc.second.p.z),trc.second.s.w/2,trc.second.s.h/2,color_table[trc.second.ID%15]);
            cv::viz::WCube cube(cv::Point3f(trc.second.viz.min_x,trc.second.viz.min_y,trc.second.viz.min_z),cv::Point3f(trc.second.viz.max_x,trc.second.viz.max_y,trc.second.viz.max_z),true,color_table[trc.second.ID%15]);
            cv::viz::WArrow arrow(cv::Point3f(trc.second.p.x,trc.second.p.y,trc.second.p.z),cv::Point3f(trc.second.p.x+trc.second.v.vx,trc.second.p.y+trc.second.v.vy,trc.second.p.z+trc.second.v.vz),0.01,color_table[trc.second.ID%15]);
            cv::viz::WText3D text(trc.second.classname,cv::Point3f(trc.second.viz.max_x,trc.second.viz.max_y,trc.second.viz.max_z),0.05,true,color_table[trc.second.ID%15]);
            cubes.push_back(cube);arrows.push_back(arrow);texts.push_back(text);traces.push_back(trace);
//            spheres.push_back(sphere);
        } else{
            continue;
//            cv::viz::WArrow arrow(cv::Point3f(trc.second.p.x,trc.second.p.y,trc.second.p.z),cv::Point3f(trc.second.p.x+trc.second.ve.vx/10,trc.second.p.y+trc.second.ve.vy/10,trc.second.p.z+trc.second.ve.vz/10),0.01,color_table[15-trc.second.ID%15]);
//            cv::viz::WText3D text(trc.second.classname,cv::Point3f(trc.second.p.x,trc.second.p.y,trc.second.p.z),0.05,true,color_table[trc.second.ID%15]);
//            cv::viz::WSphere sphere(cv::Point3f(trc.second.p.x,trc.second.p.y,trc.second.p.z),0.3,0.3,color_table[15-trc.second.ID%15]);
//            arrows.push_back(arrow);texts.push_back(text);traces.push_back(trace);spheres.push_back(sphere);
        }
        int length=trc.second.points.size();
        cv::Mat part(cv::Size{length,1}, CV_32FC3, cv::Scalar::all(0));
        for (int i = 0; i < trc.second.points.size(); ++i) {
            part.at<cv::Vec3f>(0,i)={trc.second.points.at(i).x,trc.second.points.at(i).y,trc.second.points.at(i).z};
        }
        collection.addCloud(part, color_table[trc.second.ID%15]);
    }
//    win.setBackgroundColor();
    win.showWidget("merge", collection);
    win.showWidget("coord", cv::viz::WCoordinateSystem());
    for (int i = 0; i < traces.size(); ++i) {
        win.showWidget("cube"+std::to_string(i),cubes.at(i));
        win.showWidget("arrow"+std::to_string(i),arrows.at(i));
        win.showWidget("ID"+std::to_string(i),texts.at(i));
        win.showWidget("Trace"+std::to_string(i),traces.at(i));
//        win.showWidget("sphere"+std::to_string(i),spheres.at(i));
    }
    win.spinOnce(0);
}
void showImg(cv::Mat &color, std::map<int,track> &tracks){
    for(auto trc:tracks){
        int plus=0;
        std::string plus_show,age;
        plus=trc.second.ve.vz/0.1;
        for (int i = 0; i < abs(int(plus)); ++i) {
            if (plus>0)
              plus_show=plus_show+"+";
            else
                plus_show=plus_show+"-";
        }
        age=" age"+std::to_string(trc.second.age);
       cv::rectangle(color,trc.second.Bbox,color_table[trc.first%15],1) ;
       cv::putText(color, plus_show, cv::Point((trc.second.Bbox.x+trc.second.Bbox.width/2), (trc.second.Bbox.y+trc.second.Bbox.height/2)), cv::FONT_HERSHEY_PLAIN, 1.2, color_table[trc.first%15], 2);//图像上添加labels
        cv::putText(color, trc.second.classname+age, cv::Point(trc.second.Bbox.x, trc.second.Bbox.y), cv::FONT_HERSHEY_PLAIN, 1.2, color_table[trc.first%15], 2);//图像上添加labels
//        cv::arrowedLine(color,cv::Point{(trc.second.Bbox.x+trc.second.Bbox.width/2), (trc.second.Bbox.y+trc.second.Bbox.height/2)},cv::Point{(int )(trc.second.Bbox.x+trc.second.Bbox.width/2.0+trc.second.ve.vx), (int )(trc.second.Bbox.y+trc.second.Bbox.height/2.0+trc.second.ve.vy)},
//                       color_table[trc.first%15],3);
    }
    cv::imshow("Track 2D",color);
    cv::waitKey(1);
}
/*****/
//PCL
boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud){
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setCameraPosition(0, 0, 0, 0, 0, 0, 0, 0, -1);
    viewer->setBackgroundColor(0,0,0);
    viewer->addPointCloud<pcl::PointXYZI> (cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->initCameraParameters ();
    return (viewer);
}

void viewerRunner(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer){
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100));
    }
}
void show_tracks_update(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer,std::map<int,track> &tracks, pcl::PointCloud<pcl::PointXYZI>::Ptr &pointcloud){
//    viewer->removeAllPointClouds();
    viewer=simpleVis(pointcloud);
//    viewer->updatePointCloud<pcl::PointXYZI>(pointcloud,"pointcloud");
    for (auto trc:tracks){
        viewer->addCube(trc.second.viz.min_x,trc.second.viz.max_x,trc.second.viz.min_y,trc.second.viz.max_z,trc.second.viz.max_z,trc.second.viz.min_x,0.5,0.5,0.5,"cube");
    }
}


//坐标转换工具
Eigen::Isometry3d getTFMatrix(const Eigen::Vector3d& rotate/*roll, pitch, yaw*/, const Eigen::Vector3d& translate){
    Eigen::AngleAxisd roll(rotate(0), Eigen::Vector3d(1,0,0));
    Eigen::AngleAxisd pitch(rotate(1), Eigen::Vector3d(0,1,0));
    Eigen::AngleAxisd yaw(rotate(2), Eigen::Vector3d(0,0,1));
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    R = yaw * pitch * roll;
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.rotate(R);
    T.pretranslate(Eigen::Vector3d(translate(0), translate(1), translate(2)));
    return T;
}
//深度图转化为点云
cv::Mat ConvertDepthToPointCLoud(const cv::Mat& depth, cv::Mat &mask,Eigen::Vector4d intrinsics, Eigen::Isometry3d T_rc)
{
    cv::Mat cloud(depth.size(), CV_32FC3, cv::Scalar::all(0));
    const float fx = intrinsics(0);
    const float fy = intrinsics(1);
    const float cx = intrinsics(2);
    const float cy = intrinsics(3);
    int step = 1;
    for(int i = 0; i < depth.rows; i++)
    {
        const short* depth_data = depth.ptr<short>(i);
        for(int j = 0;j<depth.cols;j++)
        {
            double pixels_distance =depth_data[j] * 0.001 ;
            if(pixels_distance > 0.05 && pixels_distance < MAX_X)
            {
                mask.at<uchar>(i, j) = 255;
            }
        }
    }
    mask = morphProcess(mask, cv::MORPH_ELLIPSE, cv::Size(10, 10), cv::MORPH_ERODE);//点云腐蚀处理
//     cv::imshow("mask", mask);
//     cv::waitKey(0);
//    for (int i = 0; i < mask.rows; ++i) {
//        for (int j = 0; j < mask.cols; ++j) {
//            std::cout<<mask.at<uint8_t>(i,j)<<"\t";
//        }
//
//    }
    for(int i = 0; i < depth.rows; i+=step)
    {
        const short* depth_data = depth.ptr<short>(i);
        cv::Vec3f *cloud_data = cloud.ptr<cv::Vec3f>(i);
        for(int j = 0; j < depth.cols; j+=step)
        {
//            if(!mask.at<uchar>(i, j))
//                continue;
            double pixels_distance =depth_data[j] * 0.001 ;
            //横点距离相机的距离s
            if(pixels_distance > 0.05 && pixels_distance < MAX_X)
            {
                cv::Vec3f pt;
                float point_x = (j-cx)*pixels_distance/fx;
                float point_y = (i-cy)*pixels_distance/fy;
                float point_z = pixels_distance;
                Eigen::Vector3d point(point_z, -point_x, -point_y);
                Eigen::Vector3d point_trans = T_rc * point;
                pt[0]= point_trans(0);
                pt[1] = point_trans(1);
                pt[2] = point_trans(2);
                if(pt[2] > MAX_Z)continue;
                if(pt[0] > MAX_X)continue;
                if(fabs(pt[1]) > MAX_Y) continue;
                cloud_data[j] = pt;
            }
        }
    }
    return cloud;
}

//点云转化为平面像素
Eigen::Vector3d PointConvertToPix(pcl::PointXYZI &point,Eigen::Vector4d intrinsics, Eigen::Isometry3d T_rc){
    Eigen::Matrix<double,3,3> Intrinsics;
    Eigen::Vector3d Point;
    Point<<point.x,point.y,point.z;
//    Point<<-point.y,-point.z,point.x;
    Intrinsics<<intrinsics(0),0,intrinsics(2),0,intrinsics(1),intrinsics(3),0,0,1;
    Point=T_rc.inverse()*Point;
    auto Pointcopy=Point;
    Point<<-Pointcopy(1),-Pointcopy(2),Pointcopy(0);
    Point=Intrinsics*Point;
    auto Depth=Point(2);Point=Point/Point(2);Point(2)=Depth;
    return Point;
}

//图像腐蚀
cv::Mat morphProcess(const cv::Mat& src, int shape, cv::Size size, int op){
    cv::Mat dst;
    cv::Mat elem = cv::getStructuringElement(shape, size);
    cv::morphologyEx(src, dst, op, elem);
    return dst;
}
//获取雷达数据 和odem数据
bool loadLidarData(const std::string& filename, costmap::LaserScan& ldata, costmap::LocateState& vpose){
    std::ifstream ifs;
    ifs.open(filename, std::ios::binary);
    if(!ifs.is_open())
        return false;
    int flag = 0;
    ifs.read((char*)(&flag), sizeof(int));
    uint64_t counter = 0;
    ifs.read((char*)(&counter), sizeof(uint64_t));
    int size;
    ifs.read((char*)(&size), sizeof(int));
    ldata.ranges.clear();
    float num = 0.0, angular = 0.0;
    for(int i = 0;i<size;i++)
    {
        ifs.read((char*)(&num), sizeof(float));
        ifs.read((char*)(&angular), sizeof(float));
        costmap::NumWithAngular item;
        item.angular = angular;
        item.num = num;
        ldata.ranges.push_back(item);
    }
    ifs.read((char*)(&vpose.pose_timestamped.pose.x), sizeof(float));
    ifs.read((char*)(&vpose.pose_timestamped.pose.y), sizeof(float));
    ifs.read((char*)(&vpose.pose_timestamped.pose.th), sizeof(float));
    ifs.close();
    return true;
}