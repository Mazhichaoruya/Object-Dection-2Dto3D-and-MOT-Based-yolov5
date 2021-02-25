//
// Created by mzc on 2021/2/1.
//

#ifndef CLOUD_SAVE_MATCHER_H
#define CLOUD_SAVE_MATCHER_H
#include "include.h"
#include "cluster/cluster.h"
#include "detector/detector.h"
class Matcher{
private:
std::map<int,Oneclass> clusters;
std::vector<Object> objects;
std::map<float,std::array<int,2>> IOUs;
std::map<std::array<int,2>,float> Us,Is,results;
std::map<int,int> Priority;
int biasx,biasy;
public:
    void GetValidArea();//获取有效区域
    Matcher(std::map<int,Oneclass> &Clu,std::vector<Object> &obj);
    std::map<std::array<int,2>,float> BoxIouMatch();
};


#endif //CLOUD_SAVE_MATCHER_H
