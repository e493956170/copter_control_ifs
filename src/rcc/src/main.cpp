#include <ros/ros.h>
#include "math.h"

#include "route_tinker/route_tinker.h"
#include "route_tracker/route_tracker.h"
#include "route_tracker/wp_manager.h"

#include "uav_link_ifs/uavcontrol_interface.h"
#include "uav_link_ifs/mavlink_interface.h"

#include "base/cfg.h"
#include "base/baseMethod.h"
#include "base/baseType.h"

#include "flight_task/flow_attacher.h"

#include "sensor/lidar_data_process.h"

#include <minimumsnap_route/service.h>

#include <opencv2/opencv.hpp>
#include <mutex>
#include <thread>
#include <functional>
#include "stdlib.h"
#include<boost/algorithm/string.hpp>

std::mutex mtx;
std::random_device rd;

std::shared_ptr<visualize_image> path_map_vis;
std::shared_ptr<visualize_image> origin_map_vis;
std::shared_ptr<visualize_image> edge_cloud_vis;

UniversalState unity_state;
#define rout ROS_INFO   //简化打印指令
#define PROJECT_NAME "my_catkin_ws"     //项目名称
#define STANDARD_CONFIG_FILE_SUFFIX "/settings/config.txt"         //项目配置文件标准地址


//轨迹控制包装函数
void _copter_control_thread_Wrapper(VelControlTracker2D *p,PositonLocalNED &input_pos_flow){
	p->Create_Thread(input_pos_flow);
}

//路径规划包装函数
void _rrt_thread_Wrapper(ConnectRRT *p
								,ProbabilisticMap &GridMap
								,FlyPlan &current_route){
	p->Create_Thread(GridMap,current_route);
}

//航点更新包装函数
void _waypointsMangaerThread_Wrapper(WayPointsUpdater *p,FlyPlan &fly_plan,PositonLocalNED &target_pos_flow){
	p->Create_Thread(fly_plan,target_pos_flow);
}

void system_init(Parameters *_p_,UniversalState *_unity_,UAVControlInterface *_uav_ifs_,PositonLocalNED *target_pos_flow,FlyPlan *fly_plan){
	_p=_p_;
	_unity=_unity_;
	_uav_ifs=_uav_ifs_;
	_target_pos_flow=target_pos_flow;
	_sub_fly_wps=fly_plan;
}	

int main (int argc,char **argv)
{
/**
 	@brief 初始化ROS节点
**/
	ros::init (argc,argv,"RCC"); 

/**
 	@brief 加载配置文件

**/
	string file_path;
	if(argc >2){
		string marker=argv[1];
		if(marker=="-c")
			file_path=argv[2];
	}else{
		file_path = string(getenv("ROS_PACKAGE_PATH"));
		std::vector<std::string> sv;
		boost::split(sv,file_path,boost::is_any_of(":"),boost::token_compress_on);
		for (int i=0;i<sv.size();i++){
			std::vector<string> tmp;
			boost::split(tmp,sv[i],boost::is_any_of("/"),boost::token_compress_on);
			if (tmp[tmp.size()-2]==string(PROJECT_NAME)){
				file_path = sv[i];
				break;
			}else{
				file_path="未找到路径";
				printf("配置文件载入失败：%s\r\n",file_path);
			}

		}
		file_path = file_path.substr(0,file_path.find_last_of("/"));
		file_path = file_path + STANDARD_CONFIG_FILE_SUFFIX;
	}

	printf("配置文件路径为：%s\r\n",file_path.c_str());
	fstream _file;
	_file.open(file_path, ios::in);
	if(!_file){
		
		printf("<%s> didn't not exist!\r\n",file_path.c_str());
		return 0;
	}
	Parameters parameters(file_path);
	if(!parameters.load_success) { rout("参数配置文件加载失败。");return 0;}

	UAVControlInterface *uav_ifs = new MAVLinkInterface(&parameters,&unity_state);//MAVLINK接口
	ProbabilisticMap grid_map(&mtx,&parameters,&unity_state,uav_ifs); //栅格地图
	ConnectRRT rrt(&rd,&mtx,&parameters,&unity_state,uav_ifs,&grid_map);  //RRT算法模块
    LidarDataProcess points_processer(&rd,&mtx,&parameters,&unity_state,uav_ifs,&grid_map);//传感器模块
	WayPointsUpdater wp_updater(&parameters,&unity_state,&mtx,uav_ifs);//坐标Local ENU  航点更新器
	FlyPlan fly_plan(WP(parameters.target_pos_x,parameters.target_pos_y,parameters.target_pos_z)); // 初始化飞行目标位置
	VelControlTracker2D vel_control_p(&parameters,&unity_state,&mtx,uav_ifs);//轨迹跟踪控制器 仅水平面
	PositonLocalNED target_pos_flow;  //位置控制变量
	system_init(&parameters,&unity_state,uav_ifs,&target_pos_flow,&fly_plan);  //参数初始化




	if(parameters.show_path_map)
	{
		path_map_vis = path_map_vis->init("path_map");
		// cv::namedWindow("Path_map",WINDOW_NORMAL);
		// cv::resizeWindow("Path_map", 480, 480);
		// cv::moveWindow("Path_map",1000,1000);
	}
	if(parameters.show_rt_map){
		origin_map_vis = origin_map_vis->init("origin_map");
		// cv::namedWindow("Origin_map",1);
		// cv::resizeWindow("Origin_map", 480, 480);
		// cv::moveWindow("Origin_map",1000,1000);	
	}
	if(parameters.show_point_cloud){
		edge_cloud_vis = edge_cloud_vis->init("edge_cloud");
		// cv::namedWindow("edgecloud",WINDOW_NORMAL);
		// cv::resizeWindow("edgecloud", 480, 480);
		// cv::moveWindow("edgecloud",1600,1000);		
	}

	// cv::startWindowThread();
	ros::Rate *loop_rate_try = new ros::Rate(2);

	while(!unity_state.first_data_get) loop_rate_try->sleep();

	ros::Rate loop_rate(30);

	for(int i =0;i<50;i++){  loop_rate.sleep();	}

	rout("System Start To Init.");

	printf("参数加载完毕，请输入需要前往的目的地坐标：x,y");

	std::thread _copter_control_thread(_copter_control_thread_Wrapper,&vel_control_p,std::ref(target_pos_flow));

	std::thread _rrt_thread(_rrt_thread_Wrapper,&rrt,std::ref(grid_map),std::ref(fly_plan));

	std::thread _waypointsMangaerThread(_waypointsMangaerThread_Wrapper,&wp_updater,std::ref(fly_plan),std::ref(target_pos_flow));


/**
 * @brief 线程Detach后独立运行 
 * 
 **/
	_copter_control_thread.detach();

	_waypointsMangaerThread.detach();

	_rrt_thread.detach();
/**
 * @brief 空循环 
 * 
 **/
	while(ros::ok()){
		std::stringstream ss;ss<<std::this_thread::get_id();
		rout("rosSpin_thread started .Thead id: %s",ss.str().c_str());
		ros::spin();
	}
	rout("程序结束。Program ended.");

	return 0;
}
