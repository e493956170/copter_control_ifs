#include "route_tracker.h"
#include "uavcontrol_interface.h"
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/Range.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>


bool PID_2D_control::update_latest_local_pos()
{   
    if(_mavlink!=nullptr){
        latest_x=_mavlink->get_pose().pos_x;
        latest_y=_mavlink->get_pose().pos_y;
        latest_z=_mavlink->get_pose().pos_z;
        latest_yaw=_mavlink->get_pose().yaw;
    }
    return true;
}


bool PID_2D_control::get_2d_control_speed(positon_Local_NED_t &input_pos_flow,velocity_Local_NED_t &output_flow){

    
    double &target_x=input_pos_flow.x;
    double &target_y=input_pos_flow.y;
    // double target_yaw=rect_coord_to_angle(target_x-latest_x,target_y-latest_y);
    
    // rout("target_x %f,target_y:%f,last_x:%f,last_y:%f,target_yaw%f，latest_yaw:%f",target_x,target_y,latest_x,latest_y,target_yaw,latest_yaw);
    target_yaw = input_pos_flow.yaw;
    double &target_vx=output_flow.vx;
    double &target_vy=output_flow.vy;
    double &target_yaw_rate=output_flow.yaw_rate;

    pid_yaw_rate.error=include_angle_calc(latest_yaw,target_yaw);
    // rout("pid_yaw_rate.error:%f",pid_yaw_rate.error);
    if(pid_yaw_rate.error_sum>10) pid_yaw_rate.error_sum=10;
    if(pid_yaw_rate.error_sum<-10) pid_yaw_rate.error_sum=-10;

    pid_vx.error=target_x-latest_x; //转换坐标系
    pid_vy.error=target_y-latest_y;
    pid_yaw_rate.error=include_angle_calc(latest_yaw,target_yaw);

    pid_vx.error_sum+=pid_vx.error;
    pid_vy.error_sum+=pid_vy.error;
    pid_yaw_rate.error_sum+=pid_vy.error_sum;

    pid_vx.i_sum_constrain();
    pid_vy.i_sum_constrain();
    pid_yaw_rate.i_sum_constrain();

    target_vx=pid_vx.PID();
    target_vy=pid_vy.PID();
    target_yaw_rate=pid_yaw_rate.PID();

    pid_vx.constrain(target_vx);
    pid_vy.constrain(target_vy);
    pid_yaw_rate.constrain(target_yaw_rate);
    // rout("%f %f",target_vx,target_vy);
    if(abs(pid_yaw_rate.error)>M_PI_4/2){
        double ratio = abs(pid_yaw_rate.error)-M_PI_4/2+1;
        target_vx/=ratio;target_vy/=ratio;
    }
    return true;
}

void PID_2D_control::Create_Thread(positon_Local_NED_t &input_pos_flow){
    
    ros::Rate *loop_rate = new ros::Rate(20);
    std::stringstream ss;ss<<"Path_PID_CONTROL_THREAD Started.Thread ID:"<<std::this_thread::get_id();rout("%s",ss.str().c_str());
    
    input_pos_flow.yaw  = _mavlink->get_pose().yaw;
    input_pos_flow.x=_mavlink->get_pose().pos_x;
    input_pos_flow.y=_mavlink->get_pose().pos_y;
    input_pos_flow.z=_mavlink->get_pose().pos_z;
    double last_x=0,last_y=0,last_yaw=0;

    rout("Mission \"Facing to Target\" : Doing.");

    while(ros::ok()){
        // rout("hello world RouteCOntrol");
        loop_rate->sleep();
        if(!_p->no_move){
            if(_mavlink->get_mode()!="OFFBOARD"){
                _mavlink->set_mode("OFFBOARD");
                _mavlink->set_arm(true);
            }
            update_latest_local_pos();
            velocity_Local_NED_t output_flow;
            get_2d_control_speed(input_pos_flow,output_flow);
            get_altitude_speed(input_pos_flow.z,output_flow.vz);
            _mavlink->setPositionLocalNED(output_flow);
        }
        // rout("attpos .x %f %f",mavlink.copter_local_pos_att.pos_x,mavlink.copter_local_pos_att.pos_y);
    }

}