#include "route_tracker/wp_manager.h"
#include "flight_task/flight_task.h"



void WP_UPDATER::Create_Thread(FLY_PLAN_T &fly_plan 
						,positon_Local_NED_t &input_pos_flow){
    bool new_fly_plan_avaliable =false;
        
    ros::Rate loop_rate(50);

    std::stringstream ss;ss<<std::this_thread::get_id();
    rout("subwaypointsMangaerThread started .Thead id: %s",ss.str().c_str());
    FLIGHT_TASK_MANAGER flight_task_manager;

    FLIGHT_TASK_TAKEOFF takeoff(_p->target_pos_z);
    FIGHT_TASK_FACE_TO_TARGET face_to_target;
    FIGHT_TASK_WP_POINT_PURE_PERSUIT wp_point_pure_persuit(WP(_p->target_pos_x,_p->target_pos_y,_p->target_pos_z),"rrt");
    FIGHT_TASK_LAND land;
    
    flight_task_manager.add_task(&takeoff);
    flight_task_manager.add_task(&face_to_target);
    flight_task_manager.add_task(&wp_point_pure_persuit);
    flight_task_manager.add_task(&land);

    for(int i = 0;i<flight_task_manager.tasks_size();i++){
        flight_task_manager[i]->loop();
    }

    rout("FLIGHT TASK MANAGER HAS FINISHED ITS JOBS.");
    while(1) loop_rate.sleep();
}

