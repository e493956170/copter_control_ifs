#ifndef __VIS__
#define __VIS__
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
typedef
class __VISUALIZE_MARKER__{

    std::shared_ptr<ros::Publisher> pub;
    visualization_msgs::Marker marker;
    std::string refresh_mode;
    __VISUALIZE_MARKER__(std::string name,std::string type,std::string marker_type,std::string refresh_method);
public:
    std::shared_ptr<__VISUALIZE_MARKER__> init(std::string name,std::string type,std::string marker_type,std::string refresh_method);
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