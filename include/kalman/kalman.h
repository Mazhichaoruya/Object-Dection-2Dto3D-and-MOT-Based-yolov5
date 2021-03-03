//
// Created by mzc on 2021/2/18.
//

#ifndef TRACKER_KALMAN_H
#define TRACKER_KALMAN_H
#include "iostream"
#include "opencv2/core.hpp"
#include "opencv2/opencv.hpp"
#define StateNum 14 //px py pz bbox x y width height vx vy vz vu vv vw vh
#define MeasureNum 7 //px py pz bbox x y width height
class Kalmanf{
private:
   int stateNum;                                      //状态值
    int measureNum;                                    //测量值
    cv::KalmanFilter KF;
    cv::Mat measurement;
public:
    Kalmanf();
    Kalmanf(int stateNum,int measureNum);
    void KalmanFilterSetup(cv::Mat A,cv::Mat H,cv::Mat P,cv::Mat Q,cv::Mat R);//初始化设置KF滤波器
    void Kalmaninitstate(std::array<float,StateNum> position);//初始化状态
    std::array<float,StateNum> Kalmanprediction();//预测当前位置
    void kalmanUpdate(std::array<float, MeasureNum> newpoint);//更新状态量
};

#endif //TRACKER_KALMAN_H
