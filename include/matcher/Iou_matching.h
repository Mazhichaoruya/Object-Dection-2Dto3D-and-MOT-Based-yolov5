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
    float Iou_caculate3D(std::array<float,StateNum> t,Detection d);//计算3dbox的IOU
    float Iou_caculate2D(std::array<float,StateNum> t,Detection d,bool Infor3D);//计算2dbox的IOU + points的欧氏距离（如果存在）
    std::vector<std::vector<float>> Iou_cost_3D(std::map<int,std::array<float,StateNum>> t_boxes,std::vector<Detection> det_boxes,float max_iou_distance);
    std::vector<std::vector<float>> Iou_cost_2D(std::map<int,std::array<float,StateNum>> t_boxes,std::vector<Detection> det_boxes,std::map<int,track> tracks);
};
#endif //TRACKER_IOU_MATCHING_H
