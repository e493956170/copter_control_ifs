#include "mavlink_interface.h"
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include "tf/transform_datatypes.h"

MAVLINK_INTERFACE::MAVLINK_INTERFACE(Parameters *_p_,UNIVERSAL_STATE *unity_state){
    _enable = true;
    _p=_p_;
    _unity=unity_state;
    ros::NodeHandle nh;
    mavlink_pub = std::make_shared<ros::Publisher>(nh.advertise<mavros_msgs::Mavlink>("mavlink/to", 1));
    vel_pub = std::make_shared<ros::Publisher>(nh.advertise<geometry_msgs::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 1, true));
    pos_sub  =std::make_shared<ros::Subscriber>(nh.subscribe<nav_msgs::Odometry>("/mavros/global_position/local",1,&MAVLINK_INTERFACE::copter_info_handler,this));
    state_sub = std::make_shared<ros::Subscriber>(nh.subscribe<mavros_msgs::State>("mavros/state",1,&MAVLINK_INTERFACE::state_cb,this));
    commandlong_srv = std::make_shared<ros::ServiceClient> (nh.serviceClient<mavros_msgs::CommandLong>("mavros/cmd/command"));
    arming_client_srv = std::make_shared<ros::ServiceClient>(nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming"));
    set_mode_client_srv = std::make_shared<ros::ServiceClient>(nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode"));
}

bool MAVLINK_INTERFACE::subscribe_sensor_flow(){
	// if(Mavlink_Commmand("dist_sensor"));
	//IMU数据订阅
	// if(Mavlink_Commmand("imu"));
	//空间定位数据订阅
	if(Mavlink_Commmand("location"));
	return true;
};

bool MAVLINK_INTERFACE::takeoff(){
    return Mavlink_Commmand("take_off");
}

bool MAVLINK_INTERFACE::set_arm(bool arm){
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = arm;
    ros::Rate loop(3);
    while( !armed()){
        if( arming_client_srv->call(arm_cmd) &&
            arm_cmd.response.success){
            ROS_INFO("SYSTEM ARMED");
            break;
        }
        loop.sleep();
    }
}
bool MAVLINK_INTERFACE::set_mode(std::string mode){
    mavros_msgs::SetMode mode_cmd;
    // mode_cmd.request.base_mode = 216 ;
    mode_cmd.request.custom_mode = mode;
    ros::Rate loop(3);
    while(ros::ok()){
        if(current_state.mode != mode&&set_mode_client_srv->call(mode_cmd)&&mode_cmd.response.mode_sent){
            ROS_INFO("MODE %s Enabled",mode.c_str());
            break;
        }
        loop.sleep();
    }
}



void MAVLINK_INTERFACE::setPositionLocalNED(velocity_Local_NED_t vel_cmd){

	if(!_enable||_link_down) return;

	geometry_msgs::Twist __vel_cmd;
	__vel_cmd.linear.x=vel_cmd.vx;
	__vel_cmd.linear.y=vel_cmd.vy;
	__vel_cmd.linear.z=vel_cmd.vz;
	
	// __vel_cmd.linear.x=0.5;
	// __vel_cmd.linear.y=0;
	// __vel_cmd.linear.z=0;

	__vel_cmd.angular.z=vel_cmd.yaw_rate;
	vel_pub->publish(__vel_cmd);

}

bool MAVLINK_INTERFACE::Mavlink_Commmand(std::string type){
	mavros_msgs::CommandLong stream_rate_cmd;
    stream_rate_cmd.response.success=false;
	if(type=="take_off"){
		int cnt=0;
		ros::Rate loop_rate(2);
		while(!stream_rate_cmd.response.success){
			if(++cnt>=2) {rout("HAND MANUAL NEED.ALREADY TAKE OFF?");break;}
			stream_rate_cmd.request.command=22;//
			stream_rate_cmd.request.confirmation=0;
			stream_rate_cmd.request.param1=0;//distance_sensor_id
			stream_rate_cmd.request.param2=0;//10hz  单位是ns
			stream_rate_cmd.request.param3=0;//10hz  单位是ns
			stream_rate_cmd.request.param4=0;//10hz  单位是ns
			stream_rate_cmd.request.param5=0;//10hz  单位是ns
			stream_rate_cmd.request.param6=0;//10hz  单位是ns
			stream_rate_cmd.request.param7=2;//10hz  单位是ns
			commandlong_srv->call(stream_rate_cmd);
			ROS_INFO("COMMAND TAKEOFF RESPOND %s.\r\nCheck if Mavros has already startd.",stream_rate_cmd.response.success?"Success":"Failed");
			loop_rate.sleep();
		}

        return true;
	}
	if(type=="dist_sensor"){
		ros::Rate loop_rate(10);
		while(!stream_rate_cmd.response.success){
			stream_rate_cmd.request.command=511;//mav_cmd_set_message_interval
			stream_rate_cmd.request.confirmation=0;
			stream_rate_cmd.request.param1=132;//distance_sensor_id
			stream_rate_cmd.request.param2=100000;//10hz  单位是ns
			commandlong_srv->call(stream_rate_cmd);
			ROS_INFO("COMMAND distance_sensor_id RESPOND %s.\r\nCheck if Mavros has already startd.",stream_rate_cmd.response.success?"Success":"Failed");
			loop_rate.sleep();
		}
        return true;
	}
	if(type=="imu"){
		ros::Rate loop_rate(10);
		while(!stream_rate_cmd.response.success){
			stream_rate_cmd.request.command=511;//mav_cmd_set_message_interval
			stream_rate_cmd.request.confirmation=0;
			stream_rate_cmd.request.param1=30;//attitude
			stream_rate_cmd.request.param2=20000;//50hz  单位是ns
			commandlong_srv->call(stream_rate_cmd);
			ROS_INFO("COMMAND ATTITUDE RESPOND %s.\r\nCheck if Mavros has already startd.",stream_rate_cmd.response.success?"Success":"Failed");
			loop_rate.sleep();
		}
        return true;
	}
	if(type=="location"){
		ros::Rate loop_rate(10);
		while(!stream_rate_cmd.response.success){
			stream_rate_cmd.request.command=511;//mav_cmd_set_message_interval
			stream_rate_cmd.request.confirmation=0;
			stream_rate_cmd.request.param1=33;//LAT_LON
			stream_rate_cmd.request.param2=100000;//10hz  单位是ns
			commandlong_srv->call(stream_rate_cmd);
			ROS_INFO("COMMAND LAT_LON RESPOND %s.\r\nCheck if Mavros has already startd.",stream_rate_cmd.response.success?"Success":"Failed");
			loop_rate.sleep();
		}
        return true;
	}
}
void MAVLINK_INTERFACE::copter_info_handler(const nav_msgs::Odometry::ConstPtr &msg){

	_unity->first_data_get=true;
	copter_local_pos_att.pos_x=msg->pose.pose.position.x;
	copter_local_pos_att.pos_y=msg->pose.pose.position.y;
	copter_local_pos_att.pos_z=msg->pose.pose.position.z;
	copter_local_pos_att.att_x=msg->pose.pose.orientation.x;
	copter_local_pos_att.att_y=msg->pose.pose.orientation.y;
	copter_local_pos_att.att_z=msg->pose.pose.orientation.z;
	copter_local_pos_att.att_w=msg->pose.pose.orientation.w;

	tf::Quaternion quat(copter_local_pos_att.att_x
												,copter_local_pos_att.att_y
												,copter_local_pos_att.att_z
												,copter_local_pos_att.att_w);

    tf::Matrix3x3(quat).getRPY(copter_local_pos_att.roll
												,copter_local_pos_att.pitch
												,copter_local_pos_att.yaw);

	copter_local_pos_att.ax=msg->twist.twist.linear.x;
	copter_local_pos_att.ay=msg->twist.twist.linear.y;
	copter_local_pos_att.az=msg->twist.twist.linear.z;
}

std::string MAVLINK_INTERFACE::get_mode(){
	return current_state.mode;
}




#include <ardupilotmega/mavlink.h>
namespace mavlink{


#define DEFAULT_SYS_ID 0
#define DEFAULT_COMPONENT 0
inline bool convert(const mavros_msgs::Mavlink &rmsg, mavlink_message_t &mmsg)
{
	if (rmsg.payload64.size() > sizeof(mmsg.payload64) / sizeof(mmsg.payload64[0])) {
		return false;
	}

	if (!rmsg.signature.empty() && rmsg.signature.size() != sizeof(mmsg.signature)) {
		return false;
	}

	// [[[cog:
	// for f in FIELD_NAMES:
	//     cog.outl("mmsg.%s = rmsg.%s;" % (f, f))
	// ]]]
	mmsg.magic = rmsg.magic;
	mmsg.len = rmsg.len;
	mmsg.incompat_flags = rmsg.incompat_flags;
	mmsg.compat_flags = rmsg.compat_flags;
	mmsg.seq = rmsg.seq;
	mmsg.sysid = rmsg.sysid;
	mmsg.compid = rmsg.compid;
	mmsg.msgid = rmsg.msgid;
	mmsg.checksum = rmsg.checksum;
	// [[[end]]] (checksum: 2ef42a7798f261bfd367bf4157b11ec0)
	std::copy(rmsg.payload64.begin(), rmsg.payload64.end(), mmsg.payload64);
	std::copy(rmsg.signature.begin(), rmsg.signature.end(), mmsg.signature);

	return true;
}
inline bool convert(const mavlink_message_t &mmsg, mavros_msgs::Mavlink &rmsg, uint8_t framing_status = mavros_msgs::Mavlink::FRAMING_OK)
{
	const size_t payload64_len = (mmsg.len + 7) / 8;

	rmsg.framing_status = framing_status;

	// [[[cog:
	// for f in FIELD_NAMES:
	//     cog.outl("rmsg.%s = mmsg.%s;" % (f, f))
	// ]]]
	rmsg.magic = mmsg.magic;
	rmsg.len = mmsg.len;
	rmsg.incompat_flags = mmsg.incompat_flags;
	rmsg.compat_flags = mmsg.compat_flags;
	rmsg.seq = mmsg.seq;
	rmsg.sysid = mmsg.sysid;
	rmsg.compid = mmsg.compid;
	rmsg.msgid = mmsg.msgid;
	rmsg.checksum = mmsg.checksum;
	// [[[end]]] (checksum: 4f0a50d2fcd7eb8823aea3e0806cd698)
	rmsg.payload64 = std::move(mavros_msgs::Mavlink::_payload64_type(mmsg.payload64, mmsg.payload64 + payload64_len));

	// copy signature block only if message is signed
	if (mmsg.incompat_flags & MAVLINK_IFLAG_SIGNED)
		rmsg.signature = std::move(mavros_msgs::Mavlink::_signature_type(mmsg.signature, mmsg.signature + sizeof(mmsg.signature)));
	else
		rmsg.signature.clear();

	return true;
}
}//mavlink