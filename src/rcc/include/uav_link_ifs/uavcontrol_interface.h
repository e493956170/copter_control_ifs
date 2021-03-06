#ifndef __UAVCONTROL_INTERFACE_H__
#define __UAVCONTROL_INTERFACE_H__
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include "base/cfg.h"
#include "geometry_msgs/Twist.h"
#include "base/baseType.h"

class UAVControlInterface{
public:
	virtual void setPositionLocalNED(VelocityLocalNED vel_cmd)=0;
	virtual bool _link_down_get()=0;
	virtual bool set_mode(std::string mode)=0;
	virtual bool set_arm(bool arm)=0;
	virtual bool armed ()=0;
	virtual bool subscribe_sensor_flow()=0;
	virtual bool takeoff()=0;
	virtual bool flexible_cmd(std::string cmd)=0;
	virtual bool update_pose()=0;
	virtual std::string get_mode()=0;

	UAVLocalPositionAndAttitude get_pose() {return copter_local_pos_att;}
protected:
	UAVLocalPositionAndAttitude copter_local_pos_att;
	Parameters *_p;
	UniversalState *_unity;
	VelocityLocalBodyNED control_vel_cmd;
	VelocityLocalBodyNED avoid_vel_cmd;
	VelocityLocalBodyNED vel_cmd;
};


#endif