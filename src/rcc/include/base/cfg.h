#pragma once
#include <iostream>
#include <string>
#include <cstring>
#include <vector>
using namespace std;

struct CFG_J
{
    string key;//索引
    string value;//值
    CFG_J *next;//下个结点
};
class Config
{
private:
    string file_name;//文件名字
    CFG_J * head;//头指针
    int cfg_line;//配置行数
    int createHead();//创建一个链表头指针
    int freeJoin();//释放链表的节点
    int inputFile();//内存配置同步配置到文件
    int joinHead(string key, string value);//将某个配置加入到链表中
public:
    Config(string file_name);//构造函数
    ~Config();//析构函数
    int getLines();//获取配置数量
    int setCFG(string key, string value);//设置一个配置
    string getCFG(string key);//从内存获取某个配置的值
    int getCFG();//从文件获取所有的配置 加载入内存链表
    void printCfg();//打印配置链表内容
};

class {
public:
    float mass=2.5;
    float Jxx=0.03283;
    float Jyy=0.03283;
    float Jzz=0.06114;
    float r=0.275;
    double Ct=0.0004106;
    double Cm=0.0000007090;
    float diameterOfBlade=466.667;//mm
    float distOfShafts=550;//mm
    float udistOfShafts=388.967;//mm
    float height=250;//mm
    float protectScale=1.5;
    float pitch=0.;
    float roll=0.;
    float yaw=0.;
    float copterSafeSizeX=550.;//mm
    float copterSafeSizeY=300.;
    float copterSafeSizeZ=550.;


}copterInfo;
class Parameters{
public:
    Parameters(string file_path);
    ~Parameters(){
        delete(cfg);
    }
    Config *cfg;
    bool load_success=false;
    double 	target_pos_x=0;
	double 	target_pos_y=0;
	double 	target_pos_z=0;
    int dilate_first_x=0;
    int dilate_first_y=0;
    int dilate_second_x=0;
    int dilate_second_y=0;
    double  Pure_Pursuit_K=0;
    double  Pure_Pursuit_LFC=2;
    double A_Max=0;
    double  A_Kp=0;
    double  A_Ki=0;
    double  A_I_Max=0;
    double  A_Kd=0;
    double  Az_Kp=0;
    double  Az_Ki=0;
    double  Az_I_Max=0;
    double  Az_Kd=0;
    double  acc_control=0;
    double Vz_I_Max= 0;
    double Yaw_I_Max=0;
    double V_I_Max=0;

	double 	Yaw_Kp=0;
	double 	Yaw_Ki=0;
	double 	Yaw_Kd=0;
	double 	V_Kp=0;
	double 	V_Ki=0;
	double 	V_Kd=0;
	double 	Vz_Kp=0;
	double 	Vz_Ki=0;
	double 	Vz_Kd=0;
	double  max_fly_speed=0;
    double max_z_speed=0;
	double  max_yaw_rad=0;
	int map_size_x=0;
	int map_size_y=0;
    int map_expand_size = 0;
    double map_occupied_thresh=0.8;
    int record_to_file=0;
    int show_rt_map=0;
    int show_point_cloud=0;
    int rrt_one_step_max_iterations=0;
    double rrt_step_size=0;
    double rrt_too_close_size=0;
    double rrt_reach_goal_thresh=0;
    int minimumsnap_en=0;
    int no_move=0;
    int show_path_map=0;
    double rrt_route_refresh_thresh;
    string Path_Pursuit_Method;
    double map_grid_size=0;
};

#ifndef _UNIVERSAL_STATE
#define _UNIVERSAL_STATE
class UniversalState{
public:
    bool new_map_avaliable=false;
    bool new_cloud_avaliable_=false;
    bool first_data_get = false;
    int current_idx=0;
    bool flight_task_started = false;
    enum class CopterState{
        STATE_AUTO,
        STATE_LOITER,
        STATE_AVOIDING,
        STATE_RISING,
        STATE_IN_FLIGHT_REPLAN
    }copter_state;
};




#endif