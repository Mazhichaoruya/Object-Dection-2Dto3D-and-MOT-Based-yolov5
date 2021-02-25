//
// Created by mzc on 2021/2/18.
//
#include "tracker/tracker.h"
Tracker::Tracker(cv::Mat a,cv::Mat h,cv::Mat p,cv::Mat q,cv::Mat r,float max_iou,int birth,int death) {
    trackIdNow=0;
    max_Iou=max_iou;
    max_age=100;
    birth_period=birth;
    death_period=death;
    A=a;Q=q;H=h;R=r;P=p;
}
void Tracker::new_track(track &tr) {
    tr.age=1;
    tr.Nomatched=0;
    tr.ID=trackIdNow;
    tr.kf=Kalmanf(StateNum,MeasureNum);
    tr.kf.KalmanFilterSetup(A,H,P,Q,R);
    tr.kf.Kalmaninitstate({tr.p.x,tr.p.y,tr.p.z,tr.s.l,tr.s.w,tr.s.w,0.0,0.0,0.0});
    tr.trace.push_back(cv::Point3f (tr.p.x,tr.p.y,tr.p.z));
    tracks.insert(std::pair<int,track>(trackIdNow,tr));
    trackIdNow++;
}
void Tracker::trackinit(std::vector<Detection> &detections) {
    for (auto detection:detections){
        track temp;
        temp.p=detection.p;temp.s=detection.s;temp.v={0.0,0.0,0.0};
        temp.ClassID=detection.ClassID;
        new_track(temp);
    }
}
void Tracker::predict() {
    kalman_output.clear();
    for (auto track:tracks) {
        kalman_output.insert(std::pair<int,std::array<float,StateNum>>(track.first,track.second.kf.Kalmanprediction()));
    }
}
void Tracker::update(std::vector<Detection> &detections){
    std::vector<int> dIds;
    std::vector<int> tIds;
    for(auto track:tracks){
        tIds.push_back(track.first);
    }
    Matcher3D matcher3d;
    auto cost_matrix=matcher3d.Iou_cost(kalman_output,detections,max_Iou);
    if (cost_matrix.empty())//修复track为0时无法产生代价矩阵的bug
        cout<<"cost martix empty!"<<std::endl;
     else
        matcher3d.Matching(dIds);
    for (int i = 0; i < detections.size(); ++i) {
        //detection遍历 判断是否需要新加入trace
        int det_idx=i;
        int trackID;
        auto iter=std::find(dIds.begin(),dIds.end(),i);
        if(iter==dIds.end()){
            //新出现的目标未有trace对应
            track temp;
            trackID=trackIdNow;
            temp.p=detections.at(det_idx).p;
            temp.s=detections.at(det_idx).s;
            temp.ClassID=detections.at(det_idx).ClassID;
            new_track(temp);//新建一个trace
//            tracks.at(trackID).enable= false;
//            dIds.erase(dIds.begin()+trackindx);
//            tIds.push_back(trackID);
        }
            //DEC有对应的trace；即可能之前出现过
        else {
            int trackindx=iter-dIds.begin();//dec对应的tarck索引
            trackID=tIds.at(trackindx);
//            if (tracks.at(trackID).Nomatched>=0){
                //已经匹配上 更新坐标 尺寸
                tracks.at(trackID).p={kalman_output.at(trackID).at(0),kalman_output.at(trackID).at(1),kalman_output.at(trackID).at(2)};
                tracks.at(trackID).s={kalman_output.at(trackID).at(3),kalman_output.at(trackID).at(4),kalman_output.at(trackID).at(5)};
                tracks.at(trackID).v={kalman_output.at(trackID).at(6),kalman_output.at(trackID).at(7),kalman_output.at(trackID).at(8)};
                tracks.at(trackID).ClassID=detections.at(det_idx).ClassID;
                tracks.at(trackID).classname=detections.at(det_idx).classname+std::to_string(trackID);
                tracks.at(trackID).viz=detections.at(det_idx).viz;
                tracks.at(trackID).points=detections.at(det_idx).points;
                tracks.at(trackID).age++;
                tracks.at(trackID).trace.push_back(cv::Point3f (tracks.at(trackID).p.x,tracks.at(trackID).p.y,tracks.at(trackID).p.z));
//            if (tracks.at(trackID).age>=birth_period)
//                tracks.at(trackID).enable= true;
//            }
        }
    }
    for (int i = 0; i < dIds.size(); ++i) {
        //trace遍历---更新trace 以及判断是否trace已经过期
        int det_idx = dIds[i];
        int trackID = tIds[i];
        if (cost_matrix[i][det_idx] > max_Iou || cost_matrix[i][det_idx] < 0||det_idx<0) {//匹配失败
//            std::cout<<"nomatched"<<std::endl;
            tracks.at(trackID).Nomatched++;//不匹配时间
            if (tracks.at(trackID).Nomatched > death_period){
                //大于容忍时间销毁track
                remove_track(trackID);
//                cout<<"Track:"<<tIds.size()<<" Det:"<<dIds.size()<<endl;
                tIds.erase(std::find(tIds.begin(),tIds.end(),trackID));//修复tIds的index越界问题
            } else{
                tracks.at(trackID).p={kalman_output.at(trackID).at(0),kalman_output.at(trackID).at(1),kalman_output.at(trackID).at(2)};
                tracks.at(trackID).s={kalman_output.at(trackID).at(3),kalman_output.at(trackID).at(4),kalman_output.at(trackID).at(5)};
                tracks.at(trackID).v={kalman_output.at(trackID).at(6),kalman_output.at(trackID).at(7),kalman_output.at(trackID).at(8)};
                tracks.at(trackID).viz={kalman_output.at(trackID).at(0)+kalman_output.at(trackID).at(3)/2,kalman_output.at(trackID).at(1)+kalman_output.at(trackID).at(4)/2,kalman_output.at(trackID).at(2)+kalman_output.at(trackID).at(5)/2
                        ,kalman_output.at(trackID).at(0)-kalman_output.at(trackID).at(3)/2,kalman_output.at(trackID).at(1)-kalman_output.at(trackID).at(4)/2,kalman_output.at(trackID).at(2)-kalman_output.at(trackID).at(5)/2};
                tracks.at(trackID).points.clear();
                tracks.at(trackID).age++;
                tracks.at(trackID).trace.push_back(cv::Point3f (tracks.at(trackID).p.x,tracks.at(trackID).p.y,tracks.at(trackID).p.z));
            }

        }
        else {
//            std::cout<<"matched"<<std::endl;
            //trace匹配
            tracks.at(trackID).Nomatched = 0;//不匹配时间
            //更新tracks的数据
            detections.at(det_idx).ID = trackID;
            //测量更新
            tracks.at(trackID).kf.kalmanUpdate(
                    {detections.at(det_idx).p.x, detections.at(det_idx).p.y, detections.at(det_idx).p.z,
                     detections.at(det_idx).s.l, detections.at(det_idx).s.w, detections.at(det_idx).s.h});
            check_state(trackID);
        }
    }
}
void Tracker::remove_track(int trackId){
    tracks.erase(trackId);
}
std::map<int, track> Tracker::report_tracks() {
    auto returntracks=tracks;
    for (auto track:tracks){
        if (track.second.enable){
            std::cout<<"trackID"<<track.second.ID<<" class:"<<track.second.classname<<" pos:"<<track.second.p.x<<track.second.p.y<<" "
                     <<track.second.p.z<<" size:"<<track.second.s.l<<" "<<track.second.s.w<<" "<<track.second.s.h<<" velocity:"<<track.second.v.vx
                     <<" "<<track.second.v.vy<<" "<<track.second.v.vz<<std::endl;
        } else
            returntracks.erase(track.first);

    }
    return returntracks;
}
void Tracker::check_state(int trackId) {
    if (tracks.at(trackId).enable== false){
        if (tracks.at(trackId).age>=birth_period)
            tracks.at(trackId).enable= true;
    }
}