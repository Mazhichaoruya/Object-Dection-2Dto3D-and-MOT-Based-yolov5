//
// Created by mzc on 2021/2/18.
//

#ifndef TRACKER_TRACKER_H
#define TRACKER_TRACKER_H
#include "include.h"
#include "kalman/kalman.h"
#include "matcher/Iou_matching.h"

class Tracker{
private:
    std::map<int,track> tracks;
    std::map<int,std::array<float,StateNum>> kalman_output;
    int max_age;//最大生存周期
    float max_Iou;//1-Max_IOU
    int trackIdNow;//记录当前ID数
    int death_period,birth_period;//生存与死亡周期
    cv::Mat A,H,Q,R,P;
    void output3d();
public:
    Tracker(cv::Mat A,cv::Mat H,cv::Mat P,cv::Mat Q,cv::Mat R,float max_iou,int birth,int death);
    void new_track(track &tr);
    void remove_track(int trackId);
    void trackinit(std::vector<Detection> &detections);
    void predict();
    void update(std::vector<Detection> &detections);
    void check_state(int trackId);
    std::vector<track_report> report_tracks();
    std::map<int,track> show_tracks();
};


#endif //TRACKER_TRACKER_H
