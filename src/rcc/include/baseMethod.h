#ifndef _BASE_METHOD_C
#define _BASE_METHOD_C
#include "math.h"
#define rout ROS_INFO

#include "baseType.h"
#include "uavcontrol_interface.h"
#include "Eigen/Dense"
#include "Eigen/Eigen"
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
using namespace cv;




class __BASE_METHOD__{
public :
    WPS bresenham( WP start, WP end);
    template <typename T1,typename T2>
    double calc_dist(T1 A,T2 B){
        double x_temp=(A.x-B.x);
        double y_temp=(A.y-B.y);
        return sqrt(pow(x_temp,2)+pow(y_temp,2));
    }

    template <typename T1,typename T2>
    double calc_dist_3(T1 A,T2 B){
        double x_temp=(A.x-B.x);
        double y_temp=(A.y-B.y);
        double z_temp=(A.z-B.z);
        return sqrt(pow(x_temp,2)+pow(y_temp,2)+pow(z_temp,2));
    }    
    double include_angle_calc(double new_angle,double old_angle);
    double angle_add(double op_angle,double change_val);
    double rect_coord_to_angle(double x,double y);
    template <typename PointType,typename IMG>
    bool index_check(PointType p1,PointType p2,IMG img){
        if(p1.x<0||p1.y<0||p2.y<0||p2.y<0)  //不小于范围就不会报错
            return false;
        return true;
    }
    template <typename PointType,typename ObstacleGridMap_t>
    bool index_check(PointType p1,ObstacleGridMap_t obstaclesMap){
        if(p1.x<0||p1.y<0) 
            return false;

        return true;
    }
    template <typename PointType,typename ObstacleGridMap_t>
    void lineimg( Mat &img, PointType  pt1 ,PointType pt2,ObstacleGridMap_t obstaclesMap){
        cv::Point PT1(pt1.x/0.2+obstaclesMap.center_x,pt1.y/0.2+obstaclesMap.center_y);
        cv::Point PT2(pt2.x/0.2+obstaclesMap.center_x,pt2.y/0.2+obstaclesMap.center_y);
        if(!index_check(pt1,pt2,img)) return;
        line(img,PT1,PT2,Scalar(0,255,0));
    }
    template <typename PointType,typename ObstacleGridMap_t>
    void lineimg( Mat &img, PointType  pt1 ,PointType pt2,ObstacleGridMap_t obstaclesMap,Scalar Scalar_){
        cv::Point PT1(pt1.x/0.2+obstaclesMap.center_x,pt1.y/0.2+obstaclesMap.center_y);
        cv::Point PT2(pt2.x/0.2+obstaclesMap.center_x,pt2.y/0.2+obstaclesMap.center_y);
        if(!index_check(pt1,pt2,obstaclesMap)) return;  
        cv::line(img,PT1,PT2,Scalar_);
    }
    template <typename PointType,typename ObstacleGridMap_t>
    void circleimg(Mat &img, PointType  pt1 ,ObstacleGridMap_t obstaclesMap,cv::Scalar Scalar_){
        cv::Point PT1(pt1.x/0.2+obstaclesMap.center_x,pt1.y/0.2+obstaclesMap.center_y);
        if(!index_check(pt1,obstaclesMap)) return;
        cv::circle(img,PT1,5,Scalar_);    
    }
    template <typename PointType,typename ObstacleGridMap_t>
    void circleimg(Mat &img, PointType  pt1 ,ObstacleGridMap_t obstaclesMap,cv::Scalar Scalar_,int size){
        cv::Point PT1(pt1.x/0.2+obstaclesMap.center_x,pt1.y/0.2+obstaclesMap.center_y);
        if(!index_check(pt1,obstaclesMap)) return;
        cv::circle(img,PT1,size,Scalar_);    
    }
};
class CoordinateTransform:public __BASE_METHOD__{
public:
    FLY_PLAN_T local_NED_Body_2_local_NED(copter_local_pos_att_t attpos,FLY_PLAN_T fly_plan);
    FLY_PLAN_T local_NED_2_local_NED_Body(copter_local_pos_att_t attpos,FLY_PLAN_T fly_plan);
    template<typename P>
    WP tf(double x ,double y ,double yaw,P pt){
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
    template<typename P>
    WP ft(double x ,double y ,double yaw,P pt){
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
    WPS ft_pts(double x ,double y ,double yaw,P pts){
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

// def Bresenham3D(p1, p2):

//     (x1, y1, z1) = p1
//     (x2, y2, z2) = p2

//     ListOfPoints = []
//     ListOfPoints.append((x1, y1, z1))

//     dx = abs(x2 - x1)
//     dy = abs(y2 - y1)
//     dz = abs(z2 - z1)

//     if (x2 > x1):
//         xs = 1
//     else:
//         xs = -1

//     if (y2 > y1):
//         ys = 1
//     else:
//         ys = -1

//     if (z2 > z1):
//         zs = 1
//     else:
//         zs = -1

//     # Driving axis is X-axis"
//     if (dx >= dy and dx >= dz):
//         p1 = 2 * dy - dx
//         p2 = 2 * dz - dx
//         while (x1 != x2):
//             x1 += xs
//             if (p1 >= 0):
//                 y1 += ys
//                 p1 -= 2 * dx
//             if (p2 >= 0):
//                 z1 += zs
//                 p2 -= 2 * dx
//             p1 += 2 * dy
//             p2 += 2 * dz
//             ListOfPoints.append((x1, y1, z1))

//     # Driving axis is Y-axis"
//     elif (dy >= dx and dy >= dz):
//         p1 = 2 * dx - dy
//         p2 = 2 * dz - dy
//         while (y1 != y2):
//             y1 += ys
//             if (p1 >= 0):
//                 x1 += xs
//                 p1 -= 2 * dy
//             if (p2 >= 0):
//                 z1 += zs
//                 p2 -= 2 * dy
//             p1 += 2 * dx
//             p2 += 2 * dz
//             ListOfPoints.append((x1, y1, z1))

//     # Driving axis is Z-axis"
//     else:
//         p1 = 2 * dy - dz
//         p2 = 2 * dx - dz
//         while (z1 != z2):
//             z1 += zs
//             if (p1 >= 0):
//                 y1 += ys
//                 p1 -= 2 * dz
//             if (p2 >= 0):
//                 x1 += xs
//                 p2 -= 2 * dz
//             p1 += 2 * dy
//             p2 += 2 * dx
//             ListOfPoints.append((x1, y1, z1))
//     return ListOfPoints

#endif