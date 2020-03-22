#include "flight_task/flight_task.h"


FLIGHT_TASK* FIGHT_TASK_LAND::loop(){
    ros::Rate Loop(100);
    _target_pos_flow->z=0.1;

    while(ros::ok()){
        Loop.sleep();
        if(abs(_uav_ifs->get_pose().pos_z-_target_pos_flow->z)<0.1)
        {
            rout("FIGHT_TASK_LAND:COMPLETE.");
            break;
        }
    }
    return nullptr;
}