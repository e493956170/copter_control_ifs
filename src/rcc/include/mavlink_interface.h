#ifndef __MAVLINK_INTERFACE_H__
#define __MAVLINK_INTERFACE_H__
#include "uavcontrol_interface.h"
#include "ros/ros.h"
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include "cfg.h"
#include "geometry_msgs/Twist.h"
#include <mavros_msgs/State.h>
#include <mavros_msgs/Mavlink.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/State.h>

class MAVLINK_INTERFACE:public UAVCONTROL_INTERFACE{
public:
	MAVLINK_INTERFACE(Parameters *_p_,UNIVERSAL_STATE *unity_state);
    void setPositionLocalNED(velocity_Local_NED_t vel_cmd);
	void state_cb(const mavros_msgs::State::ConstPtr& msg){auto _link_down=msg->connected;current_state = *msg;}	
	void copter_info_handler(const nav_msgs::Odometry::ConstPtr &msg);
	bool subscribe_sensor_flow();
	bool takeoff();
    bool set_arm(bool arm);
    bool armed(){return current_state.armed;}
	bool set_mode(std::string mode);
	bool flexible_cmd(std::string cmd){};
	bool update_pose(){}//no need to update,auto update in callback;
	std::string get_mode();

private:
    bool Mavlink_Commmand(std::string type);
	mavros_msgs::State current_state;
    std::shared_ptr<ros::ServiceClient> set_mode_client_srv;
    std::shared_ptr<ros::ServiceClient> arming_client_srv;
    std::shared_ptr<ros::ServiceClient> commandlong_srv;
	std::shared_ptr<ros::Publisher> mavlink_pub;
	std::shared_ptr<ros::Publisher> vel_pub;
	std::shared_ptr<ros::Subscriber> pos_sub;
	std::shared_ptr<ros::Subscriber> state_sub;
	std::shared_ptr<ros::Subscriber> key_sub;
    bool _enable=false;
	bool _link_down=false;
    bool _link_down_get(){return _link_down;}
};

#endif