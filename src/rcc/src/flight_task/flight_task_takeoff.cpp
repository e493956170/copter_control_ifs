#include "flight_task/flight_task.h"


FLIGHT_TASK* FLIGHT_TASK_TAKEOFF::loop(){
    ros::Rate Loop(100);
    _target_pos_flow->yaw  = _uav_ifs->get_pose().yaw;
    _target_pos_flow->x=_uav_ifs->get_pose().pos_x;
    _target_pos_flow->y=_uav_ifs->get_pose().pos_y;
    _target_pos_flow->z=target_z;
    while(ros::ok()){
        _target_pos_flow->z=target_z;
        if(abs(_uav_ifs->get_pose().pos_z-target_z)<0.2)
        {
            rout("FLIGHT_TASK_TAKEOFF : COMPLETE.");
            
            break;
        }
        Loop.sleep();
    }
    return nullptr;
}