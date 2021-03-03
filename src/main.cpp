#include <iostream>
#include "include.h"
#include "detector/detector.h"
#include "tracker/tracker.h"
#include "detector/detector3d.h"
#include "visualizer/visualizer.h"
#include "transformer/transformer.h"
#include "thread"
#include "transformer/public_type.h"
#include "time.h"
int read_files_in_dir(const char *p_dir_name, std::vector<std::string> &file_names){
    //文件路径下读取文件到Vector
    DIR *p_dir = opendir(p_dir_name);
    if (p_dir == nullptr) {
        return -1;
    }
    struct dirent* p_file = nullptr;
    while ((p_file = readdir(p_dir)) != nullptr) {
        if (strcmp(p_file->d_name, ".") != 0 &&
            strcmp(p_file->d_name, "..") != 0) {
            //std::string cur_file_name(p_dir_name);
            //cur_file_name += "/";
            //cur_file_name += p_file->d_name;
            std::string cur_file_name(p_file->d_name);
            file_names.push_back(cur_file_name);
        }
    }
    closedir(p_dir);
    return 0;
}

//
//使用文件名做时间戳同步
std::map<int,std::array<std::string,4>> time_Synchronize(std::vector<std::string> &fl,std::vector<std::string> &fr,std::vector<std::string> &fm,std::vector<std::string> &fp){
    std::vector<std::vector<std::string>> filesall{fl,fr,fm,fp};
    std::map<int,std::array<std::string,4>> results;
    std::array<std::map<int,std::string>,4> map3;
    int index=0;
    for (auto fs:filesall) {
        for (auto f:fs){
            std::string time;
            std::reverse(f.begin(),f.end());
            for (int i = 8; i < 8+7; ++i) {
                time.push_back(f.at(i));
            }
            std::reverse(time.begin(),time.end());
            std::reverse(f.begin(),f.end());
            int timeint=std::stoi(time);
//            cout<<time<<endl;
            map3.at(index)[timeint]=f;
        }
        index++;
    }
    for (auto &itp : map3.at(3)){
        bool findl= false,findr= false,findm=false;
        results[itp.first].at(3)=itp.second;//取雷达pose作为基准
        for(auto &itl:map3.at(0)){
            if (std::abs(itp.first-itl.first)<15) {
                results[itp.first].at(0) = itl.second;
                map3.at(0).erase(itl.first);
                findl= true;
                break;
            }
        }
        if (!findl){
            results.erase(itp.first);
            std::cout<<"findl_wrong"<<std::endl;
            continue;
        }
        for (auto &itr:map3.at(1)){
            if (std::abs(itp.first-itr.first)<15){
                results[itp.first].at(1)=itr.second;
                map3.at(1).erase(itr.first);
                findr= true;
                break;
            }
        }
        if (!findr){
            results.erase(itp.first);
            std::cout<<"findr_wrong"<<std::endl;
            continue;
        }
        for (auto &itm:map3.at(2)){
            if (std::abs(itp.first-itm.first)<15){
                results[itp.first].at(2)=itm.second;
                map3.at(1).erase(itm.first);
                findm= true;
                break;
            }
        }
        if (!findm){
            results.erase(itp.first);
            std::cout<<"findm_wrong"<<std::endl;
        }
//        if (results[itp.first].at(0).empty()){
//            cout<<"wrong time matched--l"<<endl;
//            results.erase(itp.first);
//            continue;
//        }
//        if (results[itp.first].at(1).empty()){
//            cout<<"wrong time matched--r"<<endl;
//            results.erase(itp.first);
//        }
//        if (results[itp.first].at(2).empty()){
//            cout<<"wrong time matched--m"<<endl;
//            results.erase(itp.first);
//        }
    }
    cout<<"num of files:"<<results.size()<<endl;
    return results;
}

//boost::mutex updateModelMutex;
//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
//boost::thread vthread(&viewerRunner,viewer);
cv::viz::Viz3d object_cloud("object_cloud");

//初始化卡尔曼滤波器参数
cv::Mat A(StateNum,StateNum,CV_32FC1,cv::Scalar(0)),H(MeasureNum,StateNum,CV_32FC1,cv::Scalar(0)),Q(StateNum,StateNum,CV_32FC1,cv::Scalar(0)),R(MeasureNum,MeasureNum,CV_32FC1,cv::Scalar(0)),P(StateNum,StateNum,CV_32FC1,cv::Scalar(0));
void set_kalman_matrix(cv::Mat &A,cv::Mat &H,cv::Mat &P,cv::Mat &Q,cv::Mat &R){
    for (int i = 0; i < A.rows; ++i) {
        for (int j = 0; j < A.cols; ++j) {
            if (i==j)
                A.at<float>(i,j)=1.0;
            if (i<StateNum/2){
                if (StateNum-j+i==7)
                    A.at<float>(i,j)=0.03;//根据时间间隔设置
            }
        }
    }
    for (int i = 0; i < H.rows; ++i) {
        for (int j = 0; j < H.cols; ++j) {
            if (i==j)
                H.at<float>(i,j)=1.0;
        }
    }
    setIdentity(Q, cv::Scalar::all(1e-4));            //系统噪声方差矩阵Q
    setIdentity(R, cv::Scalar::all(1e-3));        //测量噪声方差矩阵R
    setIdentity(P, cv::Scalar::all(.1));                  //后验错误估计协方差矩阵P
    /*
    for (int i = 0; i < A.rows; ++i) {
        for (int j = 0; j < A.cols; ++j) {
            std::cout<<A.at<float>(i,j)<<" ";
        }
        std::cout<<std::endl;
    }
     */
}
Eigen::Isometry3d M_out;
Eigen::Vector4d M_in;
Eigen::Transform<float, 3, 2, 0> TR;
costmap::StdStamp time_stamp;//时间戳 使用雷达位姿
int main() {
    //相机外参
    Eigen::Vector3d L_rot(0.0,-0.61,0.61);//左相机旋转姿态
    Eigen::Vector3d L_tra(0.18105,0.08143,0.23012);//左相机位置
    auto L_out=getTFMatrix(L_rot,L_tra);
    Eigen::Vector3d R_rot(3.14159,-0.61,-0.61);//右边相机旋转姿态 --欧拉角
    Eigen::Vector3d R_tra(0.18105,-0.08143,0.23012);//右边相机位置
    auto R_out=getTFMatrix(R_rot,R_tra);
    Eigen::Vector3d M_rot(0.0,0,0);//中间彩色相机旋转姿态 --欧拉角
//    Eigen::Vector3d M_tra(0.221,0.01,0.780);//中间相机位置
    Eigen::Vector3d M_tra(0.221,0.01,0.950);//中间相机位置
    M_out=getTFMatrix(M_rot,M_tra);
    //左右深度相机内参
    Eigen::Vector4d L_in( 429.128, 429.128,420.038, 241.847);
    Eigen::Vector4d R_in( 429.128, 429.128,420.038, 241.847);
    //彩色相机内参
    M_in    =   Eigen::Vector4d(541.2853541608224,540.3261715156995,652.0247381734374,399.9668022776777);//标定所得相机内参 相机1
//    M_in    =   Eigen::Vector4d(500.2527835897041,716.1117740787954,499.7620978089695,379.9548440735398);//标定所得相机内参 相机2
//    Eigen::Vector4d M_in(519.537841796875,539.1375122070312,625.8279638589884,399.0869303889303);//标定所得点到相机平面转换矩阵
//    Eigen::Vector4d M_in(541.3,540.3,640.0,360.0);//标定所得点到相机平面转换矩阵

    //图像读取
    //文件名路径读取
    //标定数据路径****
//    std::string rootpath="../Data/";
//    std::string samepath="depth/";
//    std::string pathL=rootpath+"L_camera/",pathR=rootpath+"R_camera/";
    //数据集数据路径
    std::string rootpath="../Data/";
    char *pathL="../Data/left_rgbd/";
    char *pathR="../Data/right_rgbd/";
    char *pathM="../Data/rgb/";
    char *pathP="../Data/laser/";
    std::string weight="../engine/";//模型路径
    std::string classname_path="../engine/road.names";//类别名路径
    std::vector<std::string> filenamels,filenamers,filenamems,filelidars;
    //标定数据文件读取
    /*
    std::ifstream file_l(pathL+"list.txt"),file_r(pathR+"list.txt");
    if (file_l.fail()&&file_r.fail()){
        std::cout<<"the file is vaild!"<<std::endl;
        return -1;
    } else
        std::cout<<"the file is opened!"<<std::endl;
    std::string filenamel,filenamer;
    while (std::getline(file_l,filenamel)){
        filenamels.push_back(pathL+samepath+filenamel+".png");
    }
    while (std::getline(file_r,filenamer)){
        filenamers.push_back(pathR+samepath+filenamer+".png");
    }
     */
    // 数据集数据读取
    if (read_files_in_dir(pathL, filenamels) < 0||read_files_in_dir(pathR, filenamers)||read_files_in_dir(pathM,filenamems)||read_files_in_dir(pathP,filelidars)) {
        std::cout << "read_rgbd&lider_in_dir failed." << std::endl;
        return -1;
    }
    auto files=time_Synchronize(filenamels,filenamers,filenamems,filelidars);
//    std::cout<<filenamels.size()<<"--"<<filenamers.size()<<std::endl;  //文件数

    set_kalman_matrix(A,H,P,Q,R);
    detector3d detector(M_in,M_out,weight,classname_path);
    Tracker tracker(A,H,P,Q,R,0.9,2,3);


    bool  init= false;
    for (auto file:files) {//读取左右相机的深度图
        costmap::LaserScan lidar_data;
        costmap::LocateState robot_pose;
        pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZI>), pointCloudf(
                new pcl::PointCloud<pcl::PointXYZI>);
        //读取数据集图像
        cv::Mat Depthl = cv::imread(rootpath + "left_rgbd/" + file.second.at(0),
                                    cv::IMREAD_ANYDEPTH);//读取 16位读取 默认8位读取
        cv::Mat Depthr = cv::imread(rootpath + "right_rgbd/" + file.second.at(1),
                                    cv::IMREAD_ANYDEPTH);//读取 16位读取 默认8位读取
        cv::Mat Color = cv::imread(rootpath + "rgb/" + file.second.at(2));//读取彩色图像
        loadLidarData(rootpath + "laser/" + file.second.at(3),lidar_data,robot_pose);
        //标定相机图像
//        cv::Mat Depthl=cv::imread(filenamels.back(),cv::IMREAD_ANYDEPTH);//读取 16位读取 默认8位读取
//        cv::Mat Depthr=cv::imread(filenamers.back(),cv::IMREAD_ANYDEPTH);//读取 16位读取 默认8位读取
        Eigen::Vector3f  ea(0.0,0.0,robot_pose.pose_timestamped.pose.th);
        Eigen::Matrix3f rotation_matrix ;
        rotation_matrix = Eigen::AngleAxisf(ea[0], Eigen::Vector3f::UnitZ()) *
                           Eigen::AngleAxisf(ea[1], Eigen::Vector3f::UnitY()) *
                           Eigen::AngleAxisf(ea[2], Eigen::Vector3f::UnitX());
        Eigen::Vector3f t(robot_pose.pose_timestamped.pose.x,robot_pose.pose_timestamped.pose.y,0.0);
//        std::cout<<"robot_T:"<<t;
//        std::cout<<"robot_R:"<<ea;
        TR = Eigen::Transform<float, 3, Eigen::Affine>::Identity();
        TR.rotate(rotation_matrix);TR.pretranslate(t);//位姿变换
        time_stamp=robot_pose.pose_timestamped.header.stamp;
        cv::Mat maskl(Depthl.size(), CV_8UC1, cv::Scalar(0));
        cv::Mat maskr(Depthr.size(), CV_8UC1, cv::Scalar(0));
        auto cloudL = ConvertDepthToPointCLoud(Depthl, maskl, L_in, L_out);//深度图转化为点云并且转为相机坐标系
        auto cloudR = ConvertDepthToPointCLoud(Depthr, maskr, R_in, R_out);
        for (int i = 0; i < cloudL.rows; ++i) {//逐个像素转化为点云数据
            for (int j = 0; j < cloudL.cols; ++j) {
                pcl::PointXYZI point;
                point.x = cloudL.at<cv::Vec3f>(i, j)(0);
                point.y = cloudL.at<cv::Vec3f>(i, j)(1);
                point.z = cloudL.at<cv::Vec3f>(i, j)(2);
                point.intensity = 0;//坐标相机点标记为0
                pointCloud->points.push_back(point);
                if (maskl.at<uint8_t>(i, j) == 255)
                    pointCloudf->points.push_back(point);
                point.x = cloudR.at<cv::Vec3f>(i, j)(0);
                point.y = cloudR.at<cv::Vec3f>(i, j)(1);
                point.z = cloudR.at<cv::Vec3f>(i, j)(2);
                point.intensity = 0;//右边相机点标记为1
                pointCloud->points.push_back(point);
                if (maskr.at<uint8_t>(i, j) == 255)
                    pointCloudf->points.push_back(point);
            }
        }
        auto startd=std::clock();
        detector.SetInput(Color,pointCloudf);
//        auto endd=std::clock();
//        cout<<"time-detect= "<<(float)(endd-startd)/CLOCKS_PER_SEC*1000<<endl;
//        detector.showimg();
        auto det=detector.ReportResults();
//        std::cout<<det.size()<<endl;
        if (det.size()<=0)
            continue;
//        for(auto &de:det){
//            if (de.Information_3D)
//                cout<<"Detection:"<<de.classname<<" pos:"<<de.p.x<<" "<<de.p.y<<" "<<de.p.z<<" size:"<<de.s.l<<" "<<de.s.w<<" "<<de.s.h<<" rect:"<<de.Bbox<<endl;
//            for (auto &point:de.points){
//                std::cout<<"Before TR:"<<point.x<<" "<<point.y<<" "<<point.z<<endl;
//                point=pcl::transformPoint(point,TR);
//                std::cout<<"After TR:"<<point.x<<" "<<point.y<<" "<<point.z<<endl;
//            }
//        }
        //将点云转移到世界坐标系
        if (Pose)
            for(auto &p:pointCloudf->points){
                p=pcl::transformPoint(p,TR);
            }
        if (!init){
            tracker.trackinit(det);
            init= true;
        } else{
//            auto start=std::clock();
            tracker.predict();
            tracker.update(det);
//            auto end=std::clock();
//            cout<<"time-track= "<<(float)(end-start)/CLOCKS_PER_SEC*1000<<endl;
            auto tracks=tracker.show_tracks();
            showCloud(object_cloud,tracks,pointCloudf);
            showImg(Color,tracks);
            auto reports=tracker.report_tracks();
        }
        for(auto &de:det){
            cout<<"Detection:"<<de.classname<<de.ID<<" pos:"<<de.p.x<<" "<<de.p.y<<" "<<de.p.z<<" size:"<<de.s.l<<" "<<de.s.w<<" "<<de.s.h<<" rect:"<<de.Bbox<<endl;
        }
    }
    detector.destory();
    return 0;
}
