//
// Created by mzc on 2021/2/18.
//

#ifndef TRACKER_IOU_MATCHING_H
#define TRACKER_IOU_MATCHING_H
#include "include.h"
#include "hungarian.h"
#include "tracker/tracker.h"
class Matcher3D{
private:
    std::vector<std::vector<float>> Cost;
public:
    void Matching(std::vector<int> &assignments);
    float Iou_caculate(std::array<float,StateNum> t,Detection d);//计算3dbox的IOU
    std::vector<std::vector<float>> Iou_cost(std::map<int,std::array<float,StateNum>> t_boxes,std::vector<Detection> det_boxes,float max_iou_distance);
};
#endif //TRACKER_IOU_MATCHING_H
