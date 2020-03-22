#ifndef _BASE_TYPE_C
#define _BASE_TYPE_C
#include "cfg.h"
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
typedef pcl::PointXYZ PT;
typedef pcl::PointCloud<PT> PTC;



typedef class __WAYPOINT_T__{
public:
	double x=0;double y=0;double z=0;
    __WAYPOINT_T__(double _x_,double _y_,double _z_):x(_x_),y(_y_),z(_z_){}
    __WAYPOINT_T__(){}
}WP,HLWP;


typedef class __WAY_POINTS_T__{
	std::vector<WP> way_points;
public:
    __WAY_POINTS_T__(){};
    __WAY_POINTS_T__(WP wp){way_points.push_back(wp);}
    __WAY_POINTS_T__(const __WAY_POINTS_T__ &v):way_points(v.way_points){};
    int size(){return way_points.size();}
    void resize(uint64_t _size_){
        way_points.resize(_size_);
    }
    template<typename T>
    void push_back(T __wp){
        way_points.push_back(WP(__wp.x,__wp.y,__wp.z));
    }
    WP &operator[](const  int &__idx){
        return way_points[__idx];
    }
	std::vector<__WAYPOINT_T__>::iterator end()  {return way_points.end();} 
	std::vector<__WAYPOINT_T__>::iterator begin(){return way_points.begin();}
}WPS;

typedef class FLY_PLAN_C{
public:
	WPS wps;
	WP _target_Pos;
	FLY_PLAN_C(){}
	FLY_PLAN_C(WP target_Pos){
		_target_Pos=target_Pos;
		wps.push_back(target_Pos);
	}

	WP & operator [](const int &idx){
		return wps[idx];
	}
	int size(){
		return wps.size();
	}
	void push_back(WP wp){
		wps.push_back(wp);
	}

}FLY_PLAN_T;



#define rout ROS_INFO
class positon_Local_NED_t{
public:
	double x;
	double y;
	double z;
	double yaw;
	int type=0;
};
class positon_with_velocity_Local_NED_t{
public:
	double x;
	double y;
	double z;
	double vx,vy,vz;
	double yaw;
	int type=1;
};

class velocity_Local_NED_t{
public:
	double vx=0;
	double vy=0;
	double vz=0;
	double yaw_rate=0;
	int type=3;
};
class vel_acc_Local_NED_t{
public:
	double vx=0;
	double vy=0;
	double vz=0;
	double ax=0;
	double ay=0;
	double az=0;
	double yaw_rate=0;
	int type=3;
};
class positon_Local_Body_NED_t{
public:
	double x;
	double y;
	double z;
	double yaw;
};
class velocity_Local_Body_NED_t{
	public:
	double vx=0;
	double vy=0;
	double vz=0;
	double yaw_rate=0;

	velocity_Local_Body_NED_t  operator+ (const velocity_Local_Body_NED_t &p1){
		velocity_Local_Body_NED_t ret;
		ret.vx=p1.vx+this->vx;
		ret.vy=p1.vy+this->vx;
		ret.vz=p1.vz+this->vx;
		ret.yaw_rate=p1.yaw_rate+this->yaw_rate;
		return ret;
	}
	velocity_Local_Body_NED_t  operator- (const velocity_Local_Body_NED_t &p1){

		velocity_Local_Body_NED_t ret;

		ret.vx=this->vx-p1.vx;
		ret.vy=this->vx-p1.vx;
		ret.vz=this->vx-p1.vz;
		ret.yaw_rate=this->yaw_rate-p1.yaw_rate;

		return ret;
	}
};

class copter_local_pos_att_t{
public:
	double pos_x=0;
	double pos_y=0;
	double pos_z=0;
	double att_x=0;
	double att_y=0;
	double att_z=0;
	double att_w=0;
	double roll=0, pitch=0, yaw=0;
	double pos_vx=0;
	double pos_vy=0;
	double pos_vz=0;
	double ax=0;
	double ay=0;
	double az=0;
};

#endif