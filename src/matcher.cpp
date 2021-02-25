//
// Created by mzc on 2021/2/1.
//
#include "matcher/matcher.h"

Matcher::Matcher(std::map<int,Oneclass> &Clu, std::vector<Object> &obj) {
   clusters=Clu;//聚类结果
   biasx=640;biasy=360;
   for(auto p:obj){
       p.area.x+=biasx;//转为点云映射重构平面像素坐标系
       p.area.y+=biasy;
       objects.push_back(p);//检测结果
   }
}
void Matcher::GetValidArea() {
    std::vector<float> distance;
    for (auto iter:clusters) {
        distance.push_back(iter.second.centerpoint.z);
    }
    std::sort(distance.begin(),distance.end());//优先级以到相机平面的距离为标准 越近（数值越小）优先级越高
    for(auto iter:clusters){
        Priority[iter.first]=std::find(distance.begin(),distance.end(),iter.second.centerpoint.z)-distance.begin();//获取优先级序列
    }

}
std::map<std::array<int,2>,float> Matcher::BoxIouMatch() {
    //获取交并集--Iou
    GetValidArea();
    for (int i = 0; i < objects.size(); ++i) {// 按照目标序号依次匹配
        std::map<int,cv::Rect> Im;
        for (auto clu:clusters) {
        Im[clu.first] = objects.at(i).area & clu.second.images;//存储当前目标匹配下左右的重合区
        Us[{i,clu.first}]=(objects.at(i).area|clu.second.images).area();//存储当前匹配的并集区域
    }
        std::vector<float> vec_sort;//最大Iou排序向量
        for (auto clu:clusters){
            Is[{i,clu.first}]=Im.at(clu.first).area();
            for(auto im:Im){
                if(Priority[clu.first]>Priority[im.first]&&Im.at(clu.first).area()&&clu.first!=im.first)//如果重叠部分优先级更高 要减去重叠区域大小
                    Is[{i,clu.first}]-=(Im[clu.first]&im.second).area();//交集的实际区域
            }
            IOUs[Is[{i,clu.first}]/Us[{i,clu.first}]]={i,clu.first};    //计算IOU
            vec_sort.push_back(Is[{i,clu.first}]/Us[{i,clu.first}]);
        }
        std::sort(vec_sort.begin(),vec_sort.end());
        results[IOUs[vec_sort.at(vec_sort.size()-1)]]=vec_sort.at(vec_sort.size()-1);//输出结果
    }
    for (auto &res:results) {
        if (objects.at(res.first.at(0)).area.area()>clusters.at(res.first.at(1)).images.area())
            res.second=-res.second;
    }
    return results;
}