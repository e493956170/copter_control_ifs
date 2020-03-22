#ifndef _OCCUPIED_MAP
#define _OCCUPIED_MAP

#include <ros/ros.h>
#include "baseMethod.h"
#include "octomap/octomap.h"
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include "baseType.h"
#include "Eigen/Dense"
#include "Eigen/Eigen"

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <mutex>
typedef pcl::PointXYZ PT;
typedef pcl::PointCloud<PT> PTC;

typedef class __OBSTACLEGRIDMAP_CONTAINER__{
public :
    typedef Eigen::MatrixXd __BASIC_MAP_CONTAINER__;
    __BASIC_MAP_CONTAINER__ map;
    int center_x;
    int center_y;
    double grid_size=0.2;
    void set_grid(int x ,int y,double value);//col first
    double operator() (const int &row,const int &col){
        if(col<0||row<0||col>=map.cols()||row>=map.rows()) return 0 ;
        return map(row,col);
    }
    void resize(int row ,int col){
        map.resize(row,col);
    }
    void setZero(){
        map.setZero();
    }
    int cols(){
        return map.cols();
    }
    int rows(){
        return map.rows();
    }
    int map_expand_size=0;

private:
//a higher value will reduce the count of expendting map;
//if index of col or row is out of bound,auto call to resize.
//@param:x->index of col
//@param:y->index of row
    void index_check(int &x,int &y);
}OBSTACLE_GRID_MAP;



class PROBABILISTIC_MAP:public CoordinateTransform{
    Parameters *_p;
    std::mutex *_mtx;
    UNIVERSAL_STATE *_unity;
    UAVCONTROL_INTERFACE *_mavlink_p;
    std::shared_ptr<ros::Publisher> pub_octomap;
    double map_griz_size = 0.2;
    ros::Time time;
public:

    std::shared_ptr<octomap::OcTree> tree;
    OBSTACLE_GRID_MAP Gridmap;
    Eigen::MatrixXi ShadowMap;
    double update_miss=0;
    double update_hit=0;
    PROBABILISTIC_MAP(std::mutex *_mtx_,Parameters *_p_,UNIVERSAL_STATE *_unity_,UAVCONTROL_INTERFACE *_mavlink_p_);
    enum class GRID_STATUS{
        HIT,
        MISS,
        
    };
    void update_grid(int x ,int y ,GRID_STATUS hit_or_miss);
    bool check_grid(int x,int y );
    double check_grid(int x,int y ,int z);

    bool isSpeckleNode(octomap::OcTree::iterator Node);
    void update_grid(WP _wp_ ,GRID_STATUS hit_or_miss);
    double check_grid(WP _wp_);

    void update_from_cloud(const PTC &cloud,const PTC &edge,copter_local_pos_att_t * attpos);

    void push_to_rviz();
    cv::Mat get_whole_map();
    void set_center(int x,int y);
    double l0=0;
    double occupied_p_of_hit =0.9;
    double occupied_p_of_miss=0.2;
    double occupied_thresh = 0.7;
};



#endif