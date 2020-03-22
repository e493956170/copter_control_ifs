#include "flight_task/flight_task.h"

FLIGHT_TASK* FIGHT_TASK_FACE_TO_TARGET::loop(){
    ros::Rate Loop(100);
    while(ros::ok()){
        _target_pos_flow->yaw = atan2(_p->target_pos_y-_uav_ifs->get_pose().pos_y,_p->target_pos_x-_uav_ifs->get_pose().pos_x);
        if(abs(include_angle_calc(_target_pos_flow->yaw,_uav_ifs->get_pose().yaw))<0.2){
            rout("FIGHT_TASK_FACE_TO_TARGET : COMPLETE.");
            break;
        }
        Loop.sleep();
    }
    return nullptr;

}
