
#ifndef __LIDAR_DATA_PROCESS__
#define __LIDAR_DATA_PROCESS__

#include "ros/ros.h"
#include "base/cfg.h"
#include "base/baseMethod.h"
#include "base/baseType.h"
#include "mutex"
#include <thread>
#include <functional>
#include "map/occupied_map.h"
#include <random>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include "sensor_msgs/point_cloud_conversion.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
class LIDAR_DATA_PROCESS:public __BASE_METHOD__{
    PROBABILISTIC_MAP *_grid_map;
    Parameters *_p;
    UNIVERSAL_STATE *_unity;
    UAVCONTROL_INTERFACE *_mavlink;
    copter_local_pos_att_t att_pos_copy;
    std::random_device *rd;
    std::mutex *mtx;
public:
    LIDAR_DATA_PROCESS(std::random_device *_rd_,std::mutex *_mtx_,Parameters *_p_,UNIVERSAL_STATE *_unity_,UAVCONTROL_INTERFACE *_mavlink_p_,PROBABILISTIC_MAP *_grid_map_):
    rd(_rd_),_p(_p_),mtx(_mtx_),_mavlink(_mavlink_p_),_unity(_unity_),_grid_map(_grid_map_){
        ros::NodeHandle nh;
        cloud_sub= std::make_shared<ros::Subscriber>(nh.subscribe("cloud",1,&LIDAR_DATA_PROCESS::lidarCloudHandler,this));
    }
    std::shared_ptr<ros::Subscriber>  cloud_sub;

    void lidarCloudHandler(const sensor_msgs::PointCloudConstPtr &input);
    PTC cloud,edgecloud;        
    void Create_Thread();
};



#endif