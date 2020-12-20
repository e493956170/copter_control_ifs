#ifndef _BASE_METHOD_C
#define _BASE_METHOD_C
#include "math.h"
#define rout ROS_INFO

#include "baseType.h"
#include "uav_link_ifs/uavcontrol_interface.h"
#include "Eigen/Dense"
#include "Eigen/Eigen"
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

/**
 	@brief 提供了一些基本的数学计算方法。 
**/
class __BASE_METHOD__{
public :
    WPS bresenham(const WP & start,const WP & end);
    template <typename T1,typename T2>
    double calc_dist(const T1 &A,const T2 &B){
        double x_temp=(A.x-B.x);
        double y_temp=(A.y-B.y);
        return sqrt(pow(x_temp,2)+pow(y_temp,2));
    }

    template <typename T1,typename T2>
    double calc_dist_3(const T1 &A,const T2 &B){
        double x_temp=(A.x-B.x);
        double y_temp=(A.y-B.y);
        double z_temp=(A.z-B.z);
        return sqrt(pow(x_temp,2)+pow(y_temp,2)+pow(z_temp,2));
    }    
/**
 	@brief 计算两条边夹角
    @param angle1  边1角度
    @param angle2  边2角度
    @return 返回夹角
**/
    double include_angle_calc(const double & new_angle,const double & old_angle);

/** @brief 在首位相连的区间上，对某个朝向的夹角进行增减计算
**  @param op_angle  边1角度
**  @param change_val  要变动的范围
**  @return 返回变化后的角度
**/
    double angle_add(double op_angle,const double & change_val);

/**
 	@brief 直角坐标系转角度坐标系
**/
    double rect_coord_to_angle(const double & x,const double & y);

/**
 	@brief 点的坐标检查，如果坐标值小于零则返回假
    @return bool量
**/
    template <typename PointType,typename IMG>
    bool index_check(const PointType &p1,const PointType &p2,const IMG &img){
        if(p1.x<0||p1.y<0||p2.y<0||p2.y<0)  //不小于范围就不会报错
            return false;
        return true;
    }
/**
 	@brief 点的坐标检查，如果坐标值小于零则返回假
    @return bool量
**/    
    template <typename PointType,typename ObstacleGridMap_t>
    bool index_check(PointType p1,ObstacleGridMap_t obstaclesMap){
        if(p1.x<0||p1.y<0) 
            return false;
        return true;
    }

    /**
 	@brief 一些绘图指令的封装。
    **/
    template <typename PointType,typename ObstacleGridMap_t>
    void lineimg( cv::Mat &img, const PointType   &pt1  ,const PointType  &pt2 ,ObstacleGridMap_t obstaclesMap){
        cv::Point PT1(pt1.x/0.2+obstaclesMap.center_x,pt1.y/0.2+obstaclesMap.center_y);
        cv::Point PT2(pt2.x/0.2+obstaclesMap.center_x,pt2.y/0.2+obstaclesMap.center_y);
        if(!index_check(pt1,pt2,img)) return;
        line(img,PT1,PT2,cv::Scalar(0,255,0));
    }
    template <typename PointType,typename ObstacleGridMap_t>
    void lineimg( cv::Mat &img,const PointType   &pt1  ,const PointType  &pt2 ,ObstacleGridMap_t obstaclesMap,cv::Scalar Scalar_){
        cv::Point PT1(pt1.x/0.2+obstaclesMap.center_x,pt1.y/0.2+obstaclesMap.center_y);
        cv::Point PT2(pt2.x/0.2+obstaclesMap.center_x,pt2.y/0.2+obstaclesMap.center_y);
        if(!index_check(pt1,pt2,obstaclesMap)) return;  
        cv::line(img,PT1,PT2,Scalar_);
    }
    template <typename PointType,typename ObstacleGridMap_t>
    void circleimg(cv::Mat &img, const PointType   &pt1  ,ObstacleGridMap_t obstaclesMap,cv::Scalar Scalar_){
        cv::Point PT1(pt1.x/0.2+obstaclesMap.center_x,pt1.y/0.2+obstaclesMap.center_y);
        if(!index_check(pt1,obstaclesMap)) return;
        cv::circle(img,PT1,5,Scalar_);    
    }
    template <typename PointType,typename ObstacleGridMap_t>
    void circleimg(cv::Mat &img, const PointType  &pt1 ,ObstacleGridMap_t obstaclesMap,cv::Scalar Scalar_,int size){
        cv::Point PT1(pt1.x/0.2+obstaclesMap.center_x,pt1.y/0.2+obstaclesMap.center_y);
        if(!index_check(pt1,obstaclesMap)) return;
        cv::circle(img,PT1,size,Scalar_);    
    }
};


/**
 	@brief 与坐标系转化相关的一些指令 
**/
class CoordinateTransform:public __BASE_METHOD__{
public:
    FlyPlan Local_NED_Body_2_Local_NED(UAVLocalPositionAndAttitude attpos,FlyPlan fly_plan);
    FlyPlan Local_NED_2_Local_NED_Body(UAVLocalPositionAndAttitude attpos,FlyPlan fly_plan);
/**
 	@brief 坐标系平移+旋转yaw角度 
**/   
    template<typename P>
    WP tf(const double & x ,const double &  y ,const double &  yaw,const P &  pt){
        Eigen::MatrixXd tl;                
        tl.resize(4,4);
        tl.setZero();
        tl<<1,0,0,-x,
            0,1,0,-y,
            0,0,1,0,
            0,0,0,1;
        Eigen::MatrixXd tr;   tr.resize(4,4);    tr.setZero();
        double rad=yaw;
        tr<<cos(rad),-sin(rad),0,0,
            sin(rad),cos(rad),0,0,
            0,0,1,0,
            0,0,0,1;	
        Eigen::Vector4d wp;
        wp<<pt.x,pt.y,0,1;
        auto ret2 = tr*tl*wp;
        return (WP(ret2(0,0),ret2(1,0),0));
    }
/**
 	@brief 坐标系反向平移+反向旋转yaw角度 
**/   
    template<typename P>
    WP ft(const double &  x ,const double &  y ,const double & yaw,const P & pt){
        Eigen::MatrixXd tl;                
        tl.resize(4,4);
        tl.setZero();
        tl<<1,0,0,-x,
            0,1,0,-y,
            0,0,1,0,
            0,0,0,1;
        Eigen::MatrixXd tr;   tr.resize(4,4);    tr.setZero();
        double rad=yaw;
        tr<<cos(rad),-sin(rad),0,0,
            sin(rad),cos(rad),0,0,
            0,0,1,0,
            0,0,0,1;	
        Eigen::Vector4d wp;
        wp<<pt.x,pt.y,0,1;
        // auto ret = ;
        auto ret2 = tl*tr*wp;
        return (WP(ret2(0,0),ret2(1,0),0));
    }
    template<typename P>
    WPS ft_pts(const double & x ,const double & y ,const double & yaw,const P & pts){
        Eigen::MatrixXd tl;                
        tl.resize(4,4);
        tl.setZero();
        tl<<1,0,0,-x,0,1,0,-y,0,0,1,0,0,0,0,1;
        Eigen::MatrixXd tr;   tr.resize(4,4);    tr.setZero();
        double &rad=yaw;
        tr<<cos(rad),-sin(rad),0,0,sin(rad),cos(rad),0,0,0,0,1,0,0,0,0,1;	
        Eigen::MatrixXd wp;
        wp.resize(4,pts.size());
    #pragma omp parallel for
        for(int i =0 ; i<pts.size();i++){
            wp(0,i)=pts[i].x;
            wp(1,i)=pts[i].y;
        }
        wp.block(2,0,1,wp.cols()).setZero();
        wp.block(3,0,1,wp.cols()).setOnes();
        // auto ret = ;
        auto ret2 = tl*tr*wp;
        WPS tmp;
        tmp.resize(pts.size());
    #pragma omp parallel for
        for(int i = 0;i<pts.size();i++){
            tmp[i]=(WP(ret2(0,i),ret2(1,i),0));
        }
        return tmp;
    }
};

#endif