//
// Created by mzc on 2021/2/18.
//
#include "matcher/Iou_matching.h"
float Matcher3D::Iou_caculate3D(std::array<float,StateNum> t,Detection d) {
    auto area1=t.at(3)*t.at(4)*t.at(5);auto area2=d.s.l*d.s.w*d.s.h;
    float x1=std::max(t.at(0)-t.at(3)/2,d.p.x-d.s.l/2);
    float y1=std::max(t.at(1)-t.at(4)/2,d.p.y-d.s.w/2);
    float z1=std::max(t.at(2)-t.at(5)/2,d.p.z-d.s.h/2);
    float x2=std::min(t.at(0)+t.at(3)/2,d.p.x+d.s.l/2);
    float y2=std::min(t.at(1)+t.at(4)/2,d.p.y+d.s.w/2);
    float z2=std::min(t.at(2)+t.at(5)/2,d.p.z+d.s.h/2);
    if (x1>=x2||y1>=y2||z1>=z2)
        return 0;
    float inter_area=(x2-x1)*(y2-y1)*(z2-z1);
    return inter_area/(area1+area2-inter_area);
}
std::vector<std::vector<float>> Matcher3D::Iou_cost_3D(std::map <int,std::array<float,StateNum>> t_boxes, std::vector <Detection> det_boxes, float max_iou_distance) {
    int row=0;
    for(auto iter=t_boxes.begin(); iter!=t_boxes.end(); iter++){
        Cost.emplace_back(std::vector<float>());
        for(int col = 0; col < det_boxes.size(); col++){
            float cost = 1 - Iou_caculate3D(iter->second, det_boxes[col]);
            if (cost > max_iou_distance){cost = 1.0;}
            Cost[row].push_back(cost);
        }
        row++;
    }
    return Cost;
}
float Matcher3D::Iou_caculate2D(std::array<float, StateNum> t, Detection d,bool Infor3D) {
    float distance=0.0;
    cv::Rect rect1{(int)t.at(3),(int)t.at(4),(int)t.at(5),(int)t.at(6)};
//    std::cout<<"text--rect1:"<<rect1<<std::endl;
    cv::Rect rect2=d.Bbox;
    float Iou=(rect1 & rect2).area()*1.0/(rect1 | rect2).area();
    if (d.Information_3D&&Infor3D){
        distance=(t.at(0)-d.p.x)*(t.at(0)-d.p.x)+(t.at(1)-d.p.y)*(t.at(1)-d.p.y)+(t.at(2)-d.p.z)*(t.at(2)-d.p.z);
        if (distance>1.0){//distance大于一米 权重为0
            distance=0;
        } else{
            //小于1m 作为权重加入ioucost，0.5占比
            distance=(1-distance);
        }
        return distance*0.3+Iou*0.7;
    } else{
//        std::cout<<"text--IOu:"<<Iou<<std::endl;
        return Iou;
    }
}
std::vector<std::vector<float>> Matcher3D::Iou_cost_2D(std::map<int, std::array<float, 14>> t_boxes,std::vector<Detection> det_boxes,std::map<int,track> tracks) {
    int row=0;
    for(auto iter=t_boxes.begin(); iter!=t_boxes.end(); iter++){
        Cost.emplace_back(std::vector<float>());
        for(int col = 0; col < det_boxes.size(); col++){
            float cost = 1 - Iou_caculate2D(iter->second, det_boxes[col],tracks.at(iter->first).Information_3D);
//            if (cost > max_iou_distance){cost = 1.0;}
            Cost[row].push_back(cost);
        }
        row++;
    }
    return Cost;
}
void Matcher3D::Matching(std::vector<int> &assignments) {
    HungarianAlgorithm HungAlgo;
    HungAlgo.Solve(Cost, assignments);
}