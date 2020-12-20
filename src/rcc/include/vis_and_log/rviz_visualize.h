#ifndef __VIS__
#define __VIS__
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"

/**
 	@brief RVIZ可视化功能的封装,先创建一个<类>的智能指针，然后调用init()函数初始化。 
**/

typedef 
class __VISUALIZE_IMAGE__{

    std::shared_ptr<ros::Publisher> pub;
    sensor_msgs::Image image;
    __VISUALIZE_IMAGE__(std::string name);
public:
    std::shared_ptr<__VISUALIZE_IMAGE__> init(std::string name);
    void push_to_rviz(cv::Mat &In);
}visualize_image;

/**
 	@brief RVIZ可视化功能的封装,先创建一个<类>的智能指针，然后调用init()函数初始化。 
**/
typedef
class __VISUALIZE_MARKER__{

    std::shared_ptr<ros::Publisher> pub;
    visualization_msgs::Marker marker;
    std::string refresh_mode;
    __VISUALIZE_MARKER__(std::string name,std::string type,std::string marker_type,std::string refresh_method);
public:
    std::shared_ptr<__VISUALIZE_MARKER__> init(std::string name,std::string type,std::string marker_type,std::string refresh_method);
/**
   @brief 设置参数
   @param r 红
   @param g 绿
   @param b 蓝
   @param a 透明度
   @param scale_x x比例
   @param scale_y y比例
   @param scale_z z比例
**/
    void set_attribue(double r,double g,double b);
    void set_attribue(double r,double g,double b,double a);
    void set_attribue(double r,double g,double b,double a,double scale_x,double scale_y,double scale_z);
    void set_quat(double x,double y,double z,double w);
    template<typename T>
    void push_to_rviz(T PointList){
        if(refresh_mode=="refresh"){
            marker.points.clear();
        }
        for (int i=0;i<PointList.size();i++){
            geometry_msgs::Point p;
            p.x=PointList[i].x;
            p.y=PointList[i].y;
            p.z=PointList[i].z;  //maually set to uav level for now
            marker.points.push_back(p);
        }
        pub->publish(marker);
    }
    void clear(){

        marker.points.clear();

        pub->publish(marker);
    }
}visualizer_marker;



#endif