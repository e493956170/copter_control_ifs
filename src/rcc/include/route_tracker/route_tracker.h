#ifndef ROUTE_CONTROL
#define ROUTE_CONTROL
#include <ros/ros.h>
#include "uav_link_ifs/uavcontrol_interface.h"
#include "mutex"
#include <thread>
#include <functional>
#include "base/cfg.h"
#include "base/baseMethod.h"
#include "base/baseType.h"
#include "map/occupied_map.h"
/*
	无人机路径控制类

*/
class Vel_Control_c:public __BASE_METHOD__{
public:

	struct {
		double roll=0;		double pitch=0;		double yaw=0;
	}copter_rpy;
	
	ros::Time update_time;
	double target_yaw=0;

	Vel_Control_c(Parameters *_p_,UNIVERSAL_STATE *_unity_,std::mutex *_mtx_,UAVCONTROL_INTERFACE *_mavlink_):_p(_p_),_unity(_unity_),mtx(_mtx_),_mavlink(_mavlink_){}

    Parameters *_p;
    UNIVERSAL_STATE *_unity;
    std::mutex *mtx;
	UAVCONTROL_INTERFACE *_mavlink;
};

class PID_2D_control:public Vel_Control_c{
public:
	double latest_x=0,latest_y=0,latest_z=0,latest_yaw=0;
	bool update_latest_local_pos();

    PID_2D_control(Parameters *_p_,UNIVERSAL_STATE *_unity_,std::mutex *_mtx_,UAVCONTROL_INTERFACE *_mavlink_):Vel_Control_c(_p_,_unity_,_mtx_,_mavlink_){
		pid_vx.Kp=_p_->V_Kp;
		pid_vx.Ki=_p_->V_Ki;
		pid_vx.Kd=_p_->V_Kd;
		pid_vx.max_output=_p_->max_fly_speed;
		pid_vx.max_i_sum=_p_->V_I_Max;

		pid_vy.Kp=_p_->V_Kp;
		pid_vy.Ki=_p_->V_Ki;
		pid_vy.Kd=_p_->V_Kd;
		pid_vy.max_output=_p_->max_fly_speed;
		pid_vy.max_i_sum=_p_->V_I_Max;

		pid_yaw_rate.Kp=_p_->Yaw_Kp;
		pid_yaw_rate.Ki=_p_->Yaw_Ki;
		pid_yaw_rate.Kd=_p_->Yaw_Kd;
		pid_yaw_rate.max_output=_p_->max_yaw_rad;
		pid_yaw_rate.max_i_sum=_p_->Yaw_I_Max;

		pid_ax.Kp=_p_->A_Kp;
		pid_ax.Ki=_p_->A_Ki;
		pid_ax.Kd=_p_->A_Kd;
		pid_ax.max_output=_p_->A_Max;
		pid_ay.Kp=_p_->A_Kp;
		pid_ay.Ki=_p_->A_Ki;
		pid_ay.Kd=_p_->A_Kd;
		pid_ay.max_output=_p_->A_Max;

		acc_control= _p_->acc_control;

		pid_z.Kp=_p->Vz_Kp;
		pid_z.Ki=_p->Vz_Ki;
		pid_z.Kd=_p->Vz_Kd;
		pid_z.max_output=_p->max_z_speed;
		pid_z.max_i_sum=_p_->Vz_I_Max;
	}

	~PID_2D_control(){}

	void Create_Thread(positon_Local_NED_t &input_pos_flow);

	void get_altitude_speed(double target_z ,double &target_vz){
		pid_z.error=target_z-latest_z;
		pid_z.error_sum+=pid_z.error;
		pid_z.i_sum_constrain();
		target_vz=pid_z.PID();
		pid_z.constrain(target_vz);
	}

	class PID_c{
	public:
		double Kp=0;
		double Ki=0;
		double Kd=0;
		double max_output=0;
		double error=0;
		double error_last = 0;
		double error_sum = 0;
		double max_i_sum = 0;
		double PID(){
    		double output = Kp*error+Ki*error_sum+Kd*(error-error_last);
			error_last=error;
			return output;
		}
		void constrain(double &target){
			target = target >  max_output ?  max_output:target;
			target = target < -max_output ? -max_output:target;
		}
		void i_sum_constrain(){
			error_sum = error_sum >  max_i_sum ?  max_i_sum:error_sum;
			error_sum = error_sum < -max_i_sum ? -max_i_sum:error_sum;
		}
	}pid_yaw_rate,pid_vx,pid_vy,pid_z,pid_ax,pid_ay;

	bool get_2d_control_speed(positon_Local_NED_t &input_pos_flow,velocity_Local_NED_t &output_flow);
	bool get_2d_control_speed(positon_Local_NED_t &input_pos_flow,vel_acc_Local_NED_t &output_flow);
	bool get_2d_control_speed_Init(positon_Local_NED_t &input_pos_flow,velocity_Local_NED_t &output_flow);

	
private:
	bool acc_control =false;
};



#endif 