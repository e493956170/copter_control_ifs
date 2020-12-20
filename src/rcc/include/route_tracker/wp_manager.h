#ifndef _WP_MANAGER
#define _WP_MANAGER
#include "mutex"
#include <thread>
#include <functional>
#include <ros/ros.h>
#include "base/baseMethod.h"
#include "base/baseType.h"
#include "uav_link_ifs/uavcontrol_interface.h"
#include "base/cfg.h"
#include "map/occupied_map.h"


class WayPointsUpdater:public __BASE_METHOD__{
    std::string Path_Pursuit_Method ;
    Parameters *_p;
    UniversalState *_unity;
    std::mutex *mtx;
    UAVControlInterface *_mavlink;
public:
    WayPointsUpdater(Parameters *_p_,UniversalState *_unity_,std::mutex *_mtx_,UAVControlInterface *_mavlink_):_p(_p_),_unity(_unity_),mtx(_mtx_),_mavlink(_mavlink_){
        Path_Pursuit_Method=_p_->Path_Pursuit_Method;
    }

    void Create_Thread(FlyPlan &fly_plan,PositonLocalNED &input_pos_flow);

};


#endif 