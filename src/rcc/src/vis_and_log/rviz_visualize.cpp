#include "vis_and_log/rviz_visualize.h"

__VISUALIZE_MARKER__::__VISUALIZE_MARKER__(std::string name,std::string type,std::string marker_type,std::string refresh_method){
    ros::NodeHandle nh;
    if(type=="marker"){
        pub =  std::make_shared<ros::Publisher>(nh.advertise<visualization_msgs::Marker>(name, 1));
    }
    refresh_mode = refresh_method;
    marker.header.frame_id="map";
    marker.header.seq=0;
    marker.header.stamp=ros::Time::now();
    marker.ns=name;
    marker.id=0;
    marker.type=visualization_msgs::Marker::CUBE;
    if(marker_type=="cube")marker.type=visualization_msgs::Marker::CUBE;
    if(marker_type=="arrow")marker.type=visualization_msgs::Marker::ARROW;
    if(marker_type=="cylinder")marker.type=visualization_msgs::Marker::CYLINDER;
    if(marker_type=="line_strip")marker.type=visualization_msgs::Marker::LINE_STRIP;
    if(marker_type=="line_list")marker.type=visualization_msgs::Marker::LINE_LIST;
    if(marker_type=="point")marker.type=visualization_msgs::Marker::POINTS;

    marker.scale.x=0.2;
    marker.scale.y=0.2;
    marker.scale.z=0.2;
    if(marker.type==visualization_msgs::Marker::POINTS)
        marker.scale.z=0;
    marker.color.r=0;
    marker.color.b=1;
    marker.color.g=0;
    marker.color.a=1;
}
void __VISUALIZE_MARKER__::set_attribue(double r,double g,double b){
    marker.color.r=r;
    marker.color.g=g;
    marker.color.b=b;
}
void __VISUALIZE_MARKER__::set_attribue(double r,double g,double b,double a,double scale_x,double scale_y,double scale_z){
    marker.scale.x=scale_x;
    marker.scale.y=scale_y;
    marker.scale.z=scale_z;
    marker.color.a=a;
    marker.color.r=r;
    marker.color.g=g;
    marker.color.b=b;
}
std::shared_ptr<__VISUALIZE_MARKER__> __VISUALIZE_MARKER__::init(std::string name,std::string type,std::string marker_type,std::string refresh_method){
    return std::make_shared<__VISUALIZE_MARKER__>(__VISUALIZE_MARKER__(name,type,marker_type,refresh_method));
}

void __VISUALIZE_MARKER__::set_quat(double x,double y,double z,double w){

    marker.pose.orientation.x=x;
    marker.pose.orientation.y=y;
    marker.pose.orientation.z=z;
    marker.pose.orientation.w=w;

}
