//
// Created by mzc on 2021/2/18.
//
#include "tracker/tracker.h"
#include "transformer/transformer.h"
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
    tr.lost3D=0;
    tr.Nomatched=0;
    tr.ID=trackIdNow;
    tr.Information_3D= false;
    tr.enable= false;
    tr.kf=Kalmanf(StateNum,MeasureNum);
    tr.kf.KalmanFilterSetup(A,H,P,Q,R);
    tr.kf.Kalmaninitstate({tr.p.x,tr.p.y,tr.p.z,(float )tr.Bbox.x,(float )tr.Bbox.y,(float )tr.Bbox.width,(float )tr.Bbox.height
                           ,tr.v.vx,tr.v.vy,tr.v.vz,tr.ve.vx,tr.ve.vy,tr.ve.vz,tr.ve.vz});
//    if (trackIdNow==3)
//    tr.trace.push_back(cv::Point3f (tr.p.x,tr.p.y,tr.p.z));
    tracks.insert(std::pair<int,track>(trackIdNow,tr));
    trackIdNow++;
}
void Tracker::trackinit(std::vector<Detection> &detections){
    for (auto detection:detections){
        track temp;
        temp.p=detection.p;temp.s=detection.s;
        temp.v={0.0,0.0,0.0};
        temp.Bbox=detection.Bbox;
        temp.ve={0.0,0.0,0.0};
        temp.ClassID=detection.ClassID;
        new_track(temp);
    }
}
void Tracker::predict() {
    kalman_output.clear();
    for (auto track:tracks) {
        auto predict=track.second.kf.Kalmanprediction();
//        if (track.second.enable)
            kalman_output.insert(std::pair<int,std::array<float,StateNum>>(track.first,predict));
    }
}
void Tracker::update(std::vector<Detection> &detections){
    std::vector<int> dIds;
    std::vector<int> tIds;
    for(auto track:tracks){
//        if (track.second.enable)
            tIds.push_back(track.first);
    }
    Matcher3D matcher3d;
    auto cost_matrix=matcher3d.Iou_cost_2D(kalman_output,detections,tracks);
    if (cost_matrix.empty())//修复track为0时无法产生代价矩阵的bug
        cout<<"cost martix empty!"<<std::endl;
     else
        matcher3d.Matching(dIds);
    for (int i = 0; i < detections.size(); ++i){
        //detection遍历 判断是否需要新加入trace
        int det_idx=i;
        int trackID;
        auto iter=std::find(dIds.begin(),dIds.end(),i);
        if(iter>=dIds.end()){
            //新出现的目标未有trace对应
            track temp;
            trackID=trackIdNow;
            temp.p=detections.at(det_idx).p;
            temp.s=detections.at(det_idx).s;
            temp.ve={0.0,0.0,0.0};
            temp.v={0,0,0};
            temp.Bbox=detections.at(det_idx).Bbox;//debug 消除不更新 kf的bug
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
            //已经匹配上 更新坐标 尺寸
            tracks.at(trackID).p={kalman_output.at(trackID).at(0),kalman_output.at(trackID).at(1),kalman_output.at(trackID).at(2)};
            tracks.at(trackID).Bbox={(int )kalman_output.at(trackID).at(3),(int)kalman_output.at(trackID).at(4),
                                     (int)kalman_output.at(trackID).at(5),(int )kalman_output.at(trackID).at(6)};
            tracks.at(trackID).v={kalman_output.at(trackID).at(7),kalman_output.at(trackID).at(8),kalman_output.at(trackID).at(9)};
            //图像像素估计速度 单位为比例 0-1
            if (tracks.at(trackID).enable){
                tracks.at(trackID).ve={kalman_output.at(trackID).at(10),kalman_output.at(trackID).at(11),
                                       std::min(std::abs(kalman_output.at(trackID).at(12)),abs(kalman_output.at(trackID).at(13)))==
                                       std::abs(kalman_output.at(trackID).at(12)) ? kalman_output.at(trackID).at(12)/kalman_output.at(trackID).at(5): kalman_output.at(trackID).at(13)/kalman_output.at(trackID).at(6)};
//                tracks.at(trackID).ve={(float )(tracks.at(trackID).ve.vz>0 ? 1.0:-1.0),(float )(-tracks.at(trackID).ve.vx>0 ? 1.0:-1.0),(float )(-tracks.at(trackID).ve.vy>0 ? 1.0:-1.0)};
            }
            else{
                tracks.at(trackID).ve={0,0,0};
            }
//            tracks.at(trackID).Bbox=detections.at(det_idx).Bbox;
            tracks.at(trackID).ClassID=detections.at(det_idx).ClassID;
            tracks.at(trackID).classname=detections.at(det_idx).classname+std::to_string(trackID);
            if (tracks.at(trackID).Information_3D){
                //具备直接的3D信息直接匹配
                if (detections.at(det_idx).Information_3D){
                    tracks.at(trackID).lost3D=0;
                    tracks.at(trackID).s    =   detections.at(det_idx).s;
                    tracks.at(trackID).viz=detections.at(det_idx).viz;
                    tracks.at(trackID).points=detections.at(det_idx).points;
                    tracks.at(trackID).trace.push_back(cv::Point3f (tracks.at(trackID).p.x,tracks.at(trackID).p.y,tracks.at(trackID).p.z));
                } else{//由3D切换为2D
                    tracks.at(trackID).lost3D++;
                    //稳定的track
                }
            } else{
                if (detections.at(det_idx).Information_3D){//由2D切换为3D
                    tracks.at(trackID).lost3D=0;
                    tracks.at(trackID).s    =   detections.at(det_idx).s;
                    tracks.at(trackID).p    =   detections.at(det_idx).p;
                    tracks.at(trackID).viz=detections.at(det_idx).viz;
                    tracks.at(trackID).points=detections.at(det_idx).points;
                    tracks.at(trackID).trace.push_back(cv::Point3f (detections.at(det_idx).p.x,detections.at(det_idx).p.y,detections.at(det_idx).p.z));//轨迹起始点为检测道到的当前位置
                    tracks.at(trackID).kf.Kalmaninitstate({detections.at(det_idx).p.x,detections.at(det_idx).p.y,detections.at(det_idx).p.z,(float )detections.at(det_idx).Bbox.x,(float )detections.at(det_idx).Bbox.y,(float )detections.at(det_idx).Bbox.width,(float )detections.at(det_idx).Bbox.height
                                                  ,0,0,0,0,0,0,0});
                    tracks.at(trackID).kf.Kalmanprediction();//重新初始化卡尔曼 加入3D的坐标
//                    std::cout<<"restate"<<trackID<<":"<<pridict.at(0)<<" "<<pridict.at(1)<<" "<<pridict.at(2)<<std::endl;
                }
            }
            //估计的位置输出
            tracks.at(trackID).pe={(kalman_output.at(trackID).at(3)+kalman_output.at(trackID).at(5)/2-Mid_h)/Mid_h,(kalman_output.at(trackID).at(4)+kalman_output.at(trackID).at(6)/2-Mid_v)/Mid_v,1.0};
            if (detections.at(det_idx).Information_3D){
                tracks.at(trackID).Information_3D= true;
            } else if (tracks.at(trackID).lost3D>3){//3D 变为2D tracker 需要重新初始化 丢失3帧以上的3D信息
                tracks.at(trackID).Information_3D= false;
                tracks.at(trackID).trace.clear();//轨迹清空
                tracks.at(trackID).kf.Kalmaninitstate({0,0,0,(float )detections.at(det_idx).Bbox.x,(float )detections.at(det_idx).Bbox.y,(float )detections.at(det_idx).Bbox.width,(float )detections.at(det_idx).Bbox.height
                                                              ,0,0,0,tracks.at(trackID).ve.vx,tracks.at(trackID).ve.vy,tracks.at(trackID).ve.vz,tracks.at(trackID).ve.vz});
                tracks.at(trackID).kf.Kalmanprediction();//重新初始化卡尔曼
            }
        }
    }
    for (int i = 0; i < tIds.size(); ++i){
        //trace遍历---更新trace 以及判断是否trace已经过期
        int det_idx = dIds[i];
        int trackID = tIds[i];
        tracks.at(trackID).age++;//无论如何增加age
        if (cost_matrix[i][det_idx] > max_Iou ||det_idx<0) {//匹配失败
//            std::cout <<trackID<<":nomatched:"<<cost_matrix[i][det_idx]<<std::endl;
            tracks.at(trackID).Nomatched++;//不匹配时间
            if (tracks.at(trackID).Nomatched >= death_period){
                //大于容忍时间销毁track
                remove_track(trackID);
                continue;
//                cout<<"Track:"<<tIds.size()<<" Det:"<<dIds.size()<<endl;
//                tIds.erase(std::find(tIds.begin(),tIds.end(),trackID));//修复tIds的index越界问题 //SB!!!
            }
            else{//刚匹配失败不足以被删除 进入等待 使用历史信息更新
                Detection det;//新建一个虚拟DET
                det.p={kalman_output.at(trackID).at(0),kalman_output.at(trackID).at(1),kalman_output.at(trackID).at(2)};
                det.Bbox={(int)kalman_output.at(trackID).at(3),(int)kalman_output.at(trackID).at(4)
                                         ,(int)kalman_output.at(trackID).at(5),(int )kalman_output.at(trackID).at(6)};
//                cout<<trackID<<":"<<det.Bbox.x<<" "<<det.Bbox.x<<" "<<det.Bbox.width<<" "<<det.Bbox.height<<endl;
                tracks.at(trackID).kf.kalmanUpdate(
                        {det.p.x, det.p.y, det.p.z,
                         (float )det.Bbox.x,(float )det.Bbox.y,(float )det.Bbox.width,(float )det.Bbox.height});
            }

        }
        else {
//            std::cout<<"matched"<<std::endl;
            //trace匹配
            tracks.at(trackID).Nomatched = 0;//不匹配时间
            //更新tracks的数据
            detections.at(det_idx).ID = trackID;
            //测量更新
            if (detections.at(det_idx).Information_3D== false&&tracks.at(trackID).Information_3D){//应对中途丢失3D信息的状态
                detections.at(det_idx).p={kalman_output.at(trackID).at(0),kalman_output.at(trackID).at(1),kalman_output.at(trackID).at(2)};
                std::cout<<"NO 3D Again"<<trackID<<" "<<kalman_output.at(trackID).at(0)<<" "<<kalman_output.at(trackID).at(1)<<" "<<kalman_output.at(trackID).at(2)<<std::endl;
            }
            tracks.at(trackID).kf.kalmanUpdate(
                    {detections.at(det_idx).p.x, detections.at(det_idx).p.y, detections.at(det_idx).p.z,
                     (float )detections.at(det_idx).Bbox.x,(float )detections.at(det_idx).Bbox.y,(float )detections.at(det_idx).Bbox.width,(float )detections.at(det_idx).Bbox.height});
        }
        check_state(trackID);
    }
}
void Tracker::remove_track(int trackId){
    tracks.erase(trackId);
}
std::vector<track_report> Tracker::report_tracks() {
    std::vector<track_report> reports;
    for (auto track:tracks){
        if (track.second.enable){
            track_report report;
            report.ID=track.second.ID;
            report.TimeStamp=time_stamp;
            report.classID=track.second.ClassID;
            report.age=track.second.age;
            report.Information_3D=track.second.Information_3D;
            report.bbox=track.second.Bbox;
            report.trace=track.second.trace;
            report.position={track.second.p.x,track.second.p.y,track.second.p.z};
            report.velocity={track.second.v.vx,track.second.v.vy,track.second.v.vz};
            report.size={track.second.s.l,track.second.s.w,track.second.s.h};
            report.position_e={track.second.pe.z,-track.second.pe.x,-track.second.pe.y};
            report.velocity_e={track.second.ve.vz,-track.second.ve.vx,-track.second.ve.vy};
            for(auto point:track.second.points){
                cv::Point3f p;
                p={point.x,point.y,point.z};
                report.pointcloud.push_back(p);
            }
            reports.push_back(report);
        }
    }
    return reports;
}
std::map<int,track> Tracker::show_tracks(){
    auto results=tracks;
    for (auto track:tracks){
        if (track.second.enable){
            std::cout<<track.second.classname<<(track.second.Information_3D ? "_3D ":"_2D ")<<" "<<" Age"<<track.second.age<<" pos:"<<track.second.p.x<<" "<<track.second.p.y<<" "
                     <<track.second.p.z<<" size:"<<track.second.s.l<<" "<<track.second.s.w<<" "<<track.second.s.h<<" velocity:"<<track.second.v.vx
                     <<" "<<track.second.v.vy<<" "<<track.second.v.vz<<" Rect:"<<track.second.Bbox<<" "<<"speed_pix:"<<track.second.ve.vx<<" "<<track.second.ve.vy<<" "<<track.second.ve.vz<<std::endl;
        } else
            results.erase(track.first);
    }
    return  results;
}
void Tracker::check_state(int trackId) {
    if (tracks.at(trackId).age>=birth_period){
            tracks.at(trackId).enable= true;
    } else{
        tracks.at(trackId).enable= false;
    }
}