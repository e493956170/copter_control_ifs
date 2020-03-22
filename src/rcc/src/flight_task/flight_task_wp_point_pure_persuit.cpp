#include "flight_task/flight_task.h"



int FIGHT_TASK_WP_POINT_PURE_PERSUIT::Pure_pusuit(FLY_PLAN_T &fly_plan
                                ,double &target_x
                                ,double &target_y
                                ,double &target_yaw){

    copter_local_pos_att_t attpos =_uav_ifs->get_pose();

    double k=_p->Pure_Pursuit_K;
    double v = sqrt(pow(_uav_ifs->get_pose().pos_vx,2)+pow(_uav_ifs->get_pose().pos_vy,2));
    double Lfc=_p->Pure_Pursuit_LFC;
    double min_dist=calc_dist(WP(attpos.pos_x,attpos.pos_y,0),fly_plan.wps[0]);
    int idx=0;
    for (int i = 1 ; i<fly_plan.wps.size();i++){
        double tmp = calc_dist(WP(attpos.pos_x,attpos.pos_y,0),fly_plan.wps[i]);
        if(tmp<min_dist){
            min_dist=tmp;
            idx=i;
        }
    }
    double Lf=k * v + Lfc;
    double L=0;
    while(Lf>L && (idx+1<fly_plan.size())){
        L+=calc_dist(fly_plan[idx+1],fly_plan[idx]);
        idx++;
    }
    target_x = fly_plan[idx].x;
    target_y = fly_plan[idx].y;
    _unity->current_idx=idx;
    if(idx==fly_plan.size()-1&&calc_dist(WP(attpos.pos_x,attpos.pos_y,0),*(fly_plan.wps.end()-1))<0.3){
        // target_yaw=angle_add(atan2(fly_plan[idx-1].y - fly_plan[idx].y,fly_plan[idx-1].x - fly_plan[idx].x),M_PI);
        rout("FIGHT_TASK_WP_POINT_PURE_PERPUIT:%f ,y:%f COMPLETE.",fly_plan[idx].x,fly_plan[idx].y);
        return 3;
    }
    return 0;
}



FLIGHT_TASK* FIGHT_TASK_WP_POINT_PURE_PERSUIT::loop(){
    ros::Rate Loop(100);
    _target_pos_flow->yaw  = _uav_ifs->get_pose().yaw;
    _target_pos_flow->x=_uav_ifs->get_pose().pos_x;
    _target_pos_flow->y=_uav_ifs->get_pose().pos_y;
    _target_pos_flow->z=_uav_ifs->get_pose().pos_z;
    double last_x=_uav_ifs->get_pose().pos_x,last_y=_uav_ifs->get_pose().pos_y,last_z=_uav_ifs->get_pose().pos_z,last_yaw=_uav_ifs->get_pose().yaw;
    _unity->flight_task_started=true;
    while(ros::ok()){
        if(_unity->copter_state==UNIVERSAL_STATE::COPTER_STATE::STATE_LOITER){
            _target_pos_flow->x = last_x;
            _target_pos_flow->y = last_y;
            _target_pos_flow->y = last_z;
            _target_pos_flow->yaw = last_yaw;
        }else{
            if(Pure_pusuit(*_sub_fly_wps,_target_pos_flow->x,_target_pos_flow->y,_target_pos_flow->yaw)==3){
                break;
            }
            last_x = _uav_ifs->get_pose().pos_x;
            last_y = _uav_ifs->get_pose().pos_y;
            last_z = _uav_ifs->get_pose().pos_z;
            last_yaw = _uav_ifs->get_pose().yaw;
        }
        Loop.sleep();
    }
    _unity->flight_task_started=false;
    return nullptr;
}
