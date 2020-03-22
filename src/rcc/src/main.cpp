#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/Range.h>
#include <pcl/io/pcd_io.h>
#include <pcl/tracking/particle_filter.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include "route_tinker.h"
#include "route_tracker.h"
#include <uavcontrol_interface.h>
#include <common/mavlink.h>
#include <mavros_msgs/Mavlink.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include "math.h"
#include "baseMethod.h"
#include "baseType.h"
#include "wp_manager.h"
#include "lidar_data_process.h"
#include <minimumsnap_route/service.h>
#include "cfg.h"
#include <mavlink_interface.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <mutex>
#include <thread>
#include <functional>
#include "flight_task/flow_attacher.h"
std::mutex mtx;
std::random_device rd;

UNIVERSAL_STATE unity_state;
#define rout ROS_INFO


void spinWrapper(){
	std::stringstream ss;ss<<std::this_thread::get_id();
	rout("rosSpin_thread started .Thead id: %s",ss.str().c_str());
	ros::spin();
}


void _copter_control_thread_Wrapper(PID_2D_control *p,positon_Local_NED_t &input_pos_flow){
	p->Create_Thread(input_pos_flow);
}
void _rrt_thread_Wrapper(Direct_RRT *p
								,PROBABILISTIC_MAP &GridMap
								,FLY_PLAN_T &current_route)
{
	p->Create_Thread(GridMap,current_route);
}

void _waypointsMangaerThread_Wrapper(WP_UPDATER *p,FLY_PLAN_T &fly_plan,positon_Local_NED_t &target_pos_flow){
	p->Create_Thread(fly_plan,target_pos_flow);
}

void system_init(Parameters *_p_,UNIVERSAL_STATE *_unity_,UAVCONTROL_INTERFACE *_uav_ifs_,positon_Local_NED_t *target_pos_flow,FLY_PLAN_T *_sub_fly_wps_){
	_p=_p_;
	_unity=_unity_;
	_uav_ifs=_uav_ifs_;
	_target_pos_flow=target_pos_flow;
	_sub_fly_wps=_sub_fly_wps_;
}	

int main (int argc,char **argv)
{
	ros::init (argc,argv,"RCC");

	string file_path;
	if(argc >2){
		string marker=argv[1];
		if(marker=="-c")
			file_path=argv[2];
	}else{
		file_path = "/home/az/config.txt";
	}
	fstream _file;
	_file.open(file_path, ios::in);
	if(!_file){
		cout<<"<"<<file_path<<"> didn't not exist!"<<endl;
		return 0;
	}
	Parameters parameters(file_path);
	if(!parameters.load_success) return 0;

	rout("Hello World.");

	UAVCONTROL_INTERFACE *uav_ifs = new MAVLINK_INTERFACE(&parameters,&unity_state);
	PROBABILISTIC_MAP grid_map(&mtx,&parameters,&unity_state,uav_ifs);
	Direct_RRT rrt(&rd,&mtx,&parameters,&unity_state,uav_ifs,&grid_map);
    LIDAR_DATA_PROCESS points_processer(&rd,&mtx,&parameters,&unity_state,uav_ifs,&grid_map);
	WP_UPDATER wp_updater(&parameters,&unity_state,&mtx,uav_ifs);//坐标Local ENU
	FLY_PLAN_T sub_fly_wps(WP(parameters.target_pos_x,parameters.target_pos_y,parameters.target_pos_z));
	PID_2D_control vel_control_p(&parameters,&unity_state,&mtx,uav_ifs);
	positon_Local_NED_t target_pos_flow;
	system_init(&parameters,&unity_state,uav_ifs,&target_pos_flow,&sub_fly_wps);



	if(parameters.show_path_map)
	{
		cv::namedWindow("Path_map",WINDOW_NORMAL);
		cv::resizeWindow("Path_map", 480, 480);
		cv::moveWindow("Path_map",1000,1000);
	}
	if(parameters.show_rt_map){
		cv::namedWindow("Origin_map",1);
		cv::resizeWindow("Origin_map", 480, 480);
		cv::moveWindow("Origin_map",1000,1000);	
	}
	if(parameters.show_point_cloud){
		cv::namedWindow("edgecloud",WINDOW_NORMAL);
		cv::resizeWindow("edgecloud", 480, 480);
		cv::moveWindow("edgecloud",1600,1000);		
	}

	cv::startWindowThread();
	ros::Rate *loop_rate_try = new ros::Rate(2);
	std::thread _ros_thread(spinWrapper);

	while(!unity_state.first_data_get) loop_rate_try->sleep();

	ros::Rate loop_rate(30);

	rout("Sleep .");

	for(int i =0;i<50;i++){  loop_rate.sleep();	}

	rout("System Start To Init.");

	std::thread _copter_control_thread(_copter_control_thread_Wrapper,&vel_control_p,std::ref(target_pos_flow));

	std::thread _rrt_thread(_rrt_thread_Wrapper,&rrt,std::ref(grid_map),std::ref(sub_fly_wps));

	std::thread _waypointsMangaerThread(_waypointsMangaerThread_Wrapper,&wp_updater,std::ref(sub_fly_wps),std::ref(target_pos_flow));

	_ros_thread.detach();

	_copter_control_thread.detach();

	_waypointsMangaerThread.detach();

	_rrt_thread.detach();

	while(ros::ok()){

		loop_rate.sleep();

	}
	rout("Program ended.");

	return 0;
}
