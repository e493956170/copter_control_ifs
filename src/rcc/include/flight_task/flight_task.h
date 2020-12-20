#ifndef __FLIGHT_TASK__
#define __FLIGHT_TASK__
#include "ros/ros.h"
#include "base/baseType.h"
#include "base/baseMethod.h"
#include "uav_link_ifs/uavcontrol_interface.h"
#include "flight_task/flow_attacher.h"


class FLIGHT_TASK:public __BASE_METHOD__{

public:
    virtual FLIGHT_TASK* loop()=0;
};

class FLIGHT_TASK_TAKEOFF:public FLIGHT_TASK{

    double target_z =0;
public:
    FLIGHT_TASK_TAKEOFF(double z){
        target_z = z;
    }
    FLIGHT_TASK* loop();
};

class FIGHT_TASK_FACE_TO_TARGET:public FLIGHT_TASK{
public:
    FLIGHT_TASK* loop();
};

class FIGHT_TASK_WP_POINT_PURE_PERSUIT:public FLIGHT_TASK{
    std::string planning_method="";
    HLWP target_wp;
    int Pure_pusuit(FlyPlan &fly_plan ,double &target_x,double &target_y,double &target_yaw);
public:

    FIGHT_TASK_WP_POINT_PURE_PERSUIT(HLWP wp,std::string _planning_method):target_wp(wp),planning_method(_planning_method){}
    FLIGHT_TASK* loop();
};
class FIGHT_TASK_LAND:public FLIGHT_TASK{
public:
    FLIGHT_TASK* loop();
};


class FLIGHT_TASK_MANAGER {
    std::vector<FLIGHT_TASK*> flight_tasks;

public:
    int tasks_size(){return flight_tasks.size();}
    FLIGHT_TASK* operator[](int idx){
        return flight_tasks[idx];
    }
    void add_task(FLIGHT_TASK* flight_task){
        flight_tasks.push_back(flight_task);
    }
    void add_task_parser(std::string cmd);//to do.

    ~FLIGHT_TASK_MANAGER(){
        for (int i =0;i<flight_tasks.size();i++){
            delete flight_tasks[i];
        }
    }
};


#endif