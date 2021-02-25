//
// Created by mzc on 2021/2/18.
//
#include "kalman/kalman.h"

Kalmanf::Kalmanf(int statenum, int measurenum) {
   stateNum=statenum;
   measureNum=measurenum;
   KF=cv::KalmanFilter(stateNum,measureNum,0);
//   std::cout<<"kalman init"<<std::endl;
}
Kalmanf::Kalmanf() {//Do nothing
}
void Kalmanf::KalmanFilterSetup(cv::Mat A,cv::Mat H,cv::Mat P,cv::Mat Q,cv::Mat R) {
    KF=cv::KalmanFilter(stateNum, measureNum, 0);
//    KF.transitionMatrix = (cv::Mat_<float>(StateNum, StateNum) <<
//            1.0f,  0.0f,  0.0f,  0.0f, 0.0f,  0.0f,  1.0f, 0.0f, 0.0f,
//            0.0f,  1.0f,  0.0f,  0.0f, 0.0f,  0.0f,  0.0f, 1.0f, 0.0f,
//            0.0f,  0.0f,  1.0f,  0.0f, 0.0f,  0.0f,  0.0f, 0.0f, 1.0f,
//            0.0f,  0.0f,  0.0f,  1.0f, 0.0f,  0.0f,  0.0f, 0.0f, 0.0f,
//            0.0f,  0.0f,  0.0f,  0.0f, 1.0f,  0.0f,  0.0f, 0.0f, 0.0f,
//            0.0f,  0.0f,  0.0f,  0.0f, 0.0f,  1.0f,  0.0f, 0.0f, 0.0f,
//            0.0f,  0.0f,  0.0f,  0.0f, 0.0f,  0.0f,  1.0f, 0.0f, 0.0f,
//            0.0f,  0.0f,  0.0f,  0.0f, 0.0f,  0.0f,  0.0f, 1.0f, 0.0f,
//            0.0f,  0.0f,  0.0f,  0.0f, 0.0f,  0.0f,  0.0f, 0.0f, 1.0f
//
//    );
//    setIdentity(KF.measurementMatrix);
//    setIdentity(KF.processNoiseCov, cv::Scalar::all(1e-4));
//    setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1e-3));
//    setIdentity(KF.errorCovPost, cv::Scalar::all(.1));
    KF.transitionMatrix=A;                            //转移矩阵A
    KF.measurementMatrix=H;                           //测量矩阵H
    KF.processNoiseCov=Q;                            //系统噪声方差矩阵Q
    KF.measurementNoiseCov=R;                        //测量噪声方差矩阵R
    KF.errorCovPost=P;                                //后验错误估计协方差矩阵P
//    std::cout<<"H:"<<std::endl;
//    for (int i = 0; i < KF.measurementMatrix.rows; ++i) {
//        for (int j = 0; j <KF.measurementMatrix.cols ; ++j) {
//            std::cout<<KF.measurementMatrix.at<float>(i,j)<<" ";
//        }
//        std::cout<<std::endl;
//    }
    measurement = cv::Mat::zeros(measureNum, 1, CV_32F);
//    std::cout<<"kalman setup para"<<std::endl;
}
void Kalmanf::Kalmaninitstate(std::array<float, StateNum> position){
    for (int i = 0; i < StateNum; ++i) {
        KF.statePost.at<float>(i)=position.at(i);
    }
//    randn(KF.statePost, cv::Scalar::all(0), cv::Scalar::all(0.1));
//    std::cout<<"init state"<<std::endl;
//    for (int i = 0; i < KF.statePost.rows; ++i) {
//        std::cout<<KF.statePost.at<float>(i)<<" ";
//    }
}
std::array<float,StateNum> Kalmanf::Kalmanprediction(){
    cv::Mat result=KF.predict();
    std::array<float,StateNum> point;
//    std::cout<<"Predict"<<" ";
    for (int i = 0; i < stateNum; ++i) {
        point.at(i)=result.at<float>(i);
//        std::cout<<point.at(i)<<" ";
    }
//    std::cout<<"kalman predict"<<std::endl;
    return point;
}
void Kalmanf::kalmanUpdate(std::array<float, MeasureNum> newpoint) {
//    std::cout<<"update"<<" ";
    for (int i = 0; i < MeasureNum; ++i) {
        measurement.at<float>(i)=newpoint.at(i);
//        std::cout<<newpoint.at(i)<<" ";

    }
//    for (int i = 0; i < measurement.rows; ++i) {
//        std::cout<<measurement.at<float>(i)<<" ";
//    }
//    std::cout<<std::endl;
    auto state=KF.correct(measurement);
//    for (int i = 0; i < StateNum; ++i) {
//        std::cout<<state.at<float>(i)<<" ";
//    }
//    std::cout<<"measure update"<<std::endl;

}