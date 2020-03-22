#ifndef _WP_MANAGER
#define _WP_MANAGER
#include "mutex"
#include <thread>
#include <functional>
#include <ros/ros.h>
#include "baseMethod.h"
#include "baseType.h"
#include "uavcontrol_interface.h"
#include "cfg.h"
#include "occupied_map.h"


class WP_UPDATER:public __BASE_METHOD__{
    std::string Path_Pursuit_Method ;
    Parameters *_p;
    UNIVERSAL_STATE *_unity;
    std::mutex *mtx;
    UAVCONTROL_INTERFACE *_mavlink;
public:
    WP_UPDATER(Parameters *_p_,UNIVERSAL_STATE *_unity_,std::mutex *_mtx_,UAVCONTROL_INTERFACE *_mavlink_):_p(_p_),_unity(_unity_),mtx(_mtx_),_mavlink(_mavlink_){
        Path_Pursuit_Method=_p_->Path_Pursuit_Method;
    }

    void Create_Thread(FLY_PLAN_T &fly_plan,positon_Local_NED_t &input_pos_flow);

};


#endif 