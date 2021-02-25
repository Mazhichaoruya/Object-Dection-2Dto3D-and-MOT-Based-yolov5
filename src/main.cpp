#include <iostream>
#include "include.h"
#include "detector/detector.h"
#include "tracker/tracker.h"
#include "detector/detector3d.h"
#include "visualizer/visualizer.h"
#include "transformer/transformer.h"
#include "thread"
#include "transformer/public_type.h"
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
std::map<int,std::array<std::string,3>> time_Synchronize(std::vector<std::string> &fl,std::vector<std::string> &fr,std::vector<std::string> &fm){
    std::vector<std::vector<std::string>> filesall{fl,fr,fm};
    std::map<int,std::array<std::string,3>> results;
    std::array<std::map<int,std::string>,3> map3;
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
    for (auto &itm : map3.at(2)){
        results[itm.first].at(2)=itm.second;
        for(auto &itl:map3.at(0)){
            if (std::abs(itm.first-itl.first)<15) {
                results[itm.first].at(0) = itl.second;
                map3.at(0).erase(itl.first);
                break;
            }
        }
        if (results[itm.first].at(0).empty()){
            cout<<"wrong time matched--l"<<endl;
            results.erase(itm.first);
            continue;
        }
        for (auto &itr:map3.at(1)){
            if (std::abs(itm.first-itr.first)<15){
                results[itm.first].at(1)=itr.second;
                map3.at(1).erase(itr.first);
                break;
            }
        }
        if (results[itm.first].at(1).empty()){
            cout<<"wrong time matched--r"<<endl;
            results.erase(itm.first);
        }
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
            if (i==0&&j==6||i==1&&j==7||i==2&&j==8)
                A.at<float>(i,j)=0.03;//根据时间间隔设置
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
}
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
    auto M_out=getTFMatrix(M_rot,M_tra);
    //左右深度相机内参
    Eigen::Vector4d L_in( 429.128, 429.128,420.038, 241.847);
    Eigen::Vector4d R_in( 429.128, 429.128,420.038, 241.847);
    //彩色相机内参
    Eigen::Vector4d M_in(541.2853541608224,540.3261715156995,652.0247381734374,399.9668022776777);//标定所得相机内参
//    Eigen::Vector4d M_in(519.537841796875,539.1375122070312,625.8279638589884,399.0869303889303);//标定所得点到相机平面转换矩阵
//    Eigen::Vector4d M_in(541.3,540.3,640.0,360.0);//标定所得点到相机平面转换矩阵

    //图像读取
    //文件名路径读取
    //标定数据路径****
//    std::string rootpath="../Data/";
//    std::string samepath="depth/";
//    std::string pathL=rootpath+"L_camera/",pathR=rootpath+"R_camera/";
    //数据集数据路径
    std::string rootpath="../Data/Images2rgbd/";
    char *pathL="../Data/Images2rgbd/left_rgbd/";
    char *pathR="../Data/Images2rgbd/right_rgbd/";
    char *pathM="../Data/Images2rgbd/rgb/";
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
    if (read_files_in_dir(pathL, filenamels) < 0||read_files_in_dir(pathR, filenamers)||read_files_in_dir(pathM,filenamems)) {
        std::cout << "read_rgbdimages_in_dir failed." << std::endl;
        return -1;
    }
    auto files=time_Synchronize(filenamels,filenamers,filenamems);
//    std::cout<<filenamels.size()<<"--"<<filenamers.size()<<std::endl;  //文件数



    set_kalman_matrix(A,H,P,Q,R);
    detector3d detector(M_in,M_out,weight,classname_path);
    Tracker tracker(A,H,P,Q,R,0.9,2,3);
    bool  init= false;
    for (auto file:files) {//读取左右相机的深度图
        pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZI>), pointCloudf(
                new pcl::PointCloud<pcl::PointXYZI>);
        //读取数据集图像
        cv::Mat Depthl = cv::imread(rootpath + "left_rgbd/" + file.second.at(0),
                                    cv::IMREAD_ANYDEPTH);//读取 16位读取 默认8位读取
        cv::Mat Depthr = cv::imread(rootpath + "right_rgbd/" + file.second.at(1),
                                    cv::IMREAD_ANYDEPTH);//读取 16位读取 默认8位读取
        cv::Mat Color = cv::imread(rootpath + "rgb/" + file.second.at(2));//读取彩色图像
        //标定相机图像
//        cv::Mat Depthl=cv::imread(filenamels.back(),cv::IMREAD_ANYDEPTH);//读取 16位读取 默认8位读取
//        cv::Mat Depthr=cv::imread(filenamers.back(),cv::IMREAD_ANYDEPTH);//读取 16位读取 默认8位读取
        cv::Mat maskl(Depthl.size(), CV_8UC1, cv::Scalar(0));
        cv::Mat maskr(Depthr.size(), CV_8UC1, cv::Scalar(0));
        auto cloudL = ConvertDepthToPointCLoud(Depthl, maskl, L_in, L_out);//深度图转化为点云并且转为世界坐标系
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
        detector.SetInput(Color,pointCloudf);
        detector.showimg();
        auto det=detector.ReportResults();
//        std::cout<<det.size()<<endl;
        if (det.size()<=0)
            continue;
        for(auto de:det){
            cout<<"Detection:"<<de.classname<<" pos:"<<de.p.x<<" "<<de.p.y<<" "<<de.p.z<<" size:"<<de.s.l<<" "<<de.s.w<<" "<<de.s.h<<endl;
        }
        if (!init){
            tracker.trackinit(det);
            init= true;
        } else{
//            vthread.join();
            tracker.predict();
            tracker.update(det);
            auto tracks=tracker.report_tracks();
            showCloud(object_cloud,tracks,pointCloudf);

        }
    }
    detector.destory();
    return 0;
}
