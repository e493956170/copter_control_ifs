#include "baseMethod.h"

FLY_PLAN_T CoordinateTransform::local_NED_2_local_NED_Body(copter_local_pos_att_t attpos,FLY_PLAN_T fly_plan)
{
	FLY_PLAN_T tmp;
    tmp.wps.resize(fly_plan.wps.size());
//平移旋转矩阵
// #pragma omp parallel for
    for (int i = 0 ;i<fly_plan.wps.size();i++){
        WP tmppt=tf(attpos.pos_x,attpos.pos_y,-angle_add(attpos.yaw,0),PT(fly_plan.wps[i].x,fly_plan.wps[i].y,0));
        tmp.wps[i]=tmppt;
        // rout("tx%f ty%f",ret2(0,0),ret2(1,0));
    }
	/*
		视为平面内旋转 暂时不考虑roll pitch
        / \   /                \  / \
		|s| = | cosyaw  -sinyaw|  |x|
		|t|   |sinyaw   cosyaw |  |y|
        \ /   \                /  \ /
	*/
	return tmp;
}

FLY_PLAN_T CoordinateTransform::local_NED_Body_2_local_NED(copter_local_pos_att_t attpos,FLY_PLAN_T fly_plan){
    FLY_PLAN_T tmp;
    tmp.wps.resize(fly_plan.wps.size());
// #pragma omp parallel for
    for (int i = 0 ;i<fly_plan.wps.size();i++){
        WP tmppt=ft(-attpos.pos_x,-attpos.pos_y,angle_add(attpos.yaw,0),PT(fly_plan.wps[i].x,fly_plan.wps[i].y,0));
        tmp.wps[i]=tmppt;
        // rout("tx%f ty%f",ret2(0,0),ret2(1,0));
    }
    /*
        视为平面内旋转 暂时不考虑roll pitch
        / \   /                \  / \
        |s| = | cosyaw  -sinyaw|  |x|
        |t|   |sinyaw   cosyaw |  |y|
        \ /   \                /  \ /
    */
    return tmp;
}

double __BASE_METHOD__::include_angle_calc(double new_angle,double old_angle){
    double temp=0;
    temp=old_angle-new_angle;
    if(temp>M_PIl) temp=temp-2*M_PIl;
    if(temp<-M_PIl) temp=temp+2*M_PIl;
    return temp;
}

double __BASE_METHOD__::angle_add(double op_angle,double change_val){
    op_angle+=change_val;
    while(op_angle>M_PIl) op_angle=op_angle-2*M_PIl;
    while(op_angle<-M_PIl) op_angle=op_angle+2*M_PIl;
    return op_angle;
}
double __BASE_METHOD__::rect_coord_to_angle(double x,double y){
    return atan2(y,x);
}

WPS __BASE_METHOD__::bresenham( WP start, WP end){

    int dx, dy;			//横纵坐标间距
    int incr1, incr2;	//P_m增量
    int d;					//P_m
    int x, y,xend, yend;//直线增长的首末端点坐标
    int xdirflag, ydirflag;//横纵坐标增长方向
    int cnt = 0;			//直线过点的点的序号

    dx = abs(end.x-start.x); 
    dy = abs(end.y-start.y);

    WPS points;
    bool reverse = false;

    if (dy <= dx) 
    {//斜率k的绝对值|k|<1时，在x方向进行单位步进
    d = dx - 2*dy; 	//初始点P_m0值
    incr1 = - 2 * dy;		//情况（1）
    incr2 = 2 * (dx - dy);//情况（2）
    if (start.x > end.x)
    {//起点横坐标比终点横坐标大，xdirflag = -1（负号可以理解为增长方向与直线始终点方向相反）
        x = end.x; y = end.y;	//设置增长起点，注意这里要选择小横坐标作为起点，用于增量时好理解
        ydirflag = (-1);				//此时默认(希望)纵坐标也是start.y > end.y的情况
        xend = start.x;		//设置增长终点横坐标
        reverse=true;
    } else {//xdirflag = 1 ,ydirflag = 1
        x = start.x; y = start.y;
        ydirflag = 1;
        xend = end.x;
        reverse=false;
    }
    points.push_back(WP(x,y,0));
    // cnt++;
    if (((end.y - start.y) * ydirflag) > 0)
        {//是预料到的情况，start.y > end.y
        while (x < xend)//开始进行增量递增
        {//x > xend，y > yend，处理向右上角爬升的直线群，下图1号所示直线
            x++;
            if (d > 0)
                d+=incr1;
            else {
                    y++; d+=incr2;		//纵坐标向正方向增长
            }
            // line->points[cnt].x=x;//添加新的点
            // line->points[cnt].y=y;
            points.push_back(WP(x,y,0));
            cnt++;
        }
    } else {//x > xend，y < yend，处理向右下角降落的直线群，下图2号所示直线
        while (x < xend) 
        {
            x++;
            if (d > 0) 
                    d+=incr1;
            else {
                y--; d+=incr2;	//纵坐标向负方向增长
            }
            points.push_back(WP(x,y,0));
            cnt++;
            }
        }		
    } else {//dy > dx，当斜率k的绝对值|k|>1时，在y方向进行单位步进
    d = dy - 2*dx;	//P_m0初值
    incr1 = -2*dx; 	//形同算法推导情况1
    incr2 = 2 * (dy - dx);
    if (start.y > end.y) 
    {
            y = end.y; x = end.x;	//取最小的纵坐标作为起点
            yend = start.y;
            xdirflag = (-1);		//期望start.x > end.x
            reverse=true;
        } else {//start.y < end.y
            y = start.y; x = start.x;
            yend = end.y;
            xdirflag = 1;
            reverse=false;
    }
    points.push_back(WP(x,y,0));
    cnt++;
    if (((end.x - start.x) * xdirflag) > 0) 
    {//x > xend ，y > yend，处理向右上角爬升的直线群，下图3号所示直线
        while (y < yend)
        {
            y++;
            if (d > 0) 
                d+=incr1;
            else {
                x++; d+=incr2;		//横坐标向正方向增长
            }
            points.push_back(WP(x,y,0));

            }
        } else 
        {//x < xend，y > yend，处理向左上角爬升的直线群，下图4号所示直线
        while (y < yend)
        {
        y++;
        if (d > 0) 
            d+=incr1;
        else {
            x--; d+=incr2;	////横坐标向负方向增长
        }
        points.push_back(WP(x,y,0));
        }
    }
    }
    if(reverse==true){
        std::reverse(points.begin(),points.end());
    }
    return points;
}