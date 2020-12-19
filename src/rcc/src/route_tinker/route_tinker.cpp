#include <route_tinker/route_tinker.h>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <time.h>
#include "route_tracker/route_tracker.h"
#include "pcl/filters/voxel_grid.h"
#include "Eigen/Dense"
#include "Eigen/Eigen"
#include "sensor_msgs/point_cloud_conversion.h"
#include <pcl/common/common.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <iostream>
#include <chrono>
#include <algorithm>  
#include <iomanip>
using namespace cv;

int RRT_Base::get_nearst_point_index(Tree_t trees,TreeNode_t new_point){
    double min_dist = calc_dist(trees[0],new_point);
    int min_dist_index = 0;
    // int area_x = new_point.x/5;
    // int area_y = new_point.y/5;
    for (int i=1;i<trees.size();i++){
        // if(trees[i].area_x_idx<(area_x-2)||trees[i].area_x_idx<(area_x+2)) continue;
        // if(trees[i].area_y_idx<(area_y-2)||trees[i].area_y_idx<(area_y+2)) continue;

        auto tmp_dist = calc_dist(trees[i],new_point);
        if(tmp_dist<min_dist){
            min_dist=tmp_dist;
            min_dist_index=i;
        }
    }
    return min_dist_index;
}

RRT_Base::TreeNode_t RRT_Base::get_nearst_point(Tree_t trees,TreeNode_t new_point){

    double min_dist = calc_dist(trees[0],new_point);
    int min_dist_index = 0;
    // int area_x = new_point.x/5;
    // int area_y = new_point.y/5;
    for (int i=0;i<trees.size();i++){
        // if(trees[i].area_x_idx<(area_x-2)||trees[i].area_x_idx<(area_x+2)) continue;
        // if(trees[i].area_y_idx<(area_y-2)||trees[i].area_y_idx<(area_y+2)) continue;
        auto tmp_dist = calc_dist(trees[i],new_point);
        if(tmp_dist<min_dist){
            min_dist=tmp_dist;
            min_dist_index=i;
        }
    }
    return trees[min_dist_index];
}

RRT_Base::TreeNode_t RRT_Base::get_nearst_point(Tree_t trees,TreeNode_t new_point,TreeNode_t target){
    double min_dist = calc_dist(trees[0],new_point);
    min_dist+=calc_dist(trees[0],target);
    int min_dist_index = 0;
    for (int i=1;i<trees.size();i++){
        auto tmp_dist = calc_dist(trees[i],new_point);
        double target_dist = calc_dist(trees[i],target)*0.1;
        tmp_dist+=target_dist;
        if(tmp_dist<min_dist){
            min_dist=tmp_dist;
            min_dist_index=i;
        }
    }
    return trees[min_dist_index];
}

double RRT_Base::calc_grow_theta(TreeNode_t cloest_point,TreeNode_t new_point){
    return atan2(new_point.x-cloest_point.x,new_point.y-cloest_point.y);
}
double RRT_Base::calc_grow_theta(WP cloest_point,WP new_point){
    return atan2(new_point.x-cloest_point.x,new_point.y-cloest_point.y);
}
RRT_Base::TreeNode_t RRT_Base::extend_a_step(TreeNode_t cloest_point,double step_size,double rad){
    cloest_point.x+=step_size*sin(rad);
    cloest_point.y+=step_size*cos(rad);
    cloest_point.z=_p->target_pos_z;
    TreeNode_t tmp(cloest_point);
    return tmp;
}


bool RRT_Base::single_point_check(WP newPoint ,OBSTACLE_GRID_MAP  &obstaclesMap){
    if(obstaclesMap((int)newPoint.y,(int)newPoint.x)>0) {
        return false;
        }
    return true;
}


bool RRT_Base::check_if_path_ok(TreeNode_t cloest_point,TreeNode_t new_point,OBSTACLE_GRID_MAP &obstaclesMap){

    auto grid_size = obstaclesMap.grid_size;
//转化为栅格坐标
    cloest_point.x = cloest_point.x/grid_size+obstaclesMap.center_x;
    cloest_point.y = cloest_point.y/grid_size+obstaclesMap.center_y;
    new_point.x = new_point.x/grid_size+obstaclesMap.center_x;
    new_point.y = new_point.y/grid_size+obstaclesMap.center_y;
    // rout("tmp index()%f  %f %f %f",cloest_point.p.x,cloest_point.p.y,new_point.p.x,new_point.p.y);
    
    WPS tmp = bresenham(cloest_point,new_point);
    // rout("tmp size()%d",tmp.size());
    std::stringstream ss;
    ss<<"bresenham index :";

    for(int i = 0 ;i <tmp.size();i++){
        ss<<"x:"<<tmp[i].x<<",y:"<<tmp[i].y<<"--->";
        if(!single_point_check(tmp[i],obstaclesMap)){
            // rout("check failed.");
            return false;
        }
    }
    // rout("%s ",ss.str().c_str());
    return true;
}



bool RRT_Base::check_if_path_ok(WP cloest_point,WP new_point,OBSTACLE_GRID_MAP &obstaclesMap){

    auto grid_size = obstaclesMap.grid_size;
//转化为栅格坐标
    cloest_point.x = cloest_point.x/grid_size+obstaclesMap.center_x;
    cloest_point.y = cloest_point.y/grid_size+obstaclesMap.center_y;
    new_point.x = new_point.x/grid_size+obstaclesMap.center_x;
    new_point.y = new_point.y/grid_size+obstaclesMap.center_y;
    
    WPS tmp = bresenham(cloest_point,new_point);
    // rout("tmp size()%d",tmp.size());
    std::stringstream ss;
    ss<<"bresenham index :";

    for(int i = 0 ;i <tmp.size();i++){
        ss<<"x:"<<tmp[i].x<<",y:"<<tmp[i].y<<"--->";
        if(!single_point_check(tmp[i],obstaclesMap)){
            // rout("check failed.");
            return false;
        }
    }
    // rout("%s ",ss.str().c_str());
    return true;
}

//step5
bool RRT_Base::check_if_reached_goal(TreeNode_t goal_point,TreeNode_t new_point,double threshold){
    if(calc_dist(goal_point,new_point)<threshold)
        return true;
    else
        return false;
}
//step6
bool RRT_Base::check_if_new_point_too_close_to_other_points(Tree_t trees,TreeNode_t new_point,double threshold,int skip_idx){
    for(int i=0;i<trees.size();i++){
        if(i==skip_idx)continue;
        if(calc_dist(trees[i],new_point)<threshold){
            return true;
        }
    }
    return false;
}



WPS RRT_Base::get_rrt_path(Tree_t &src_trees){
    
    WPS tmp;
    int father_index =(src_trees.end()-1)->father_index;
    tmp.push_back(*(src_trees.end()-1));
    std::stringstream ss;
    ss<<"->"<<(src_trees.end()-1)->index<<"("<<(src_trees.end()-1)->x<<")("<<(src_trees.end()-1)->y<<")";
    while(father_index!=-1){
        tmp.push_back(src_trees[father_index]);
        ss<<"->"<<src_trees[father_index].index<<"("<<src_trees[father_index].x<<")("<<src_trees[father_index].y<<")";
        father_index=src_trees[father_index].father_index;
        if(father_index == -1) break;
        
    }
    // rout("\r\n%s",ss.str().c_str());
    WPS tmp_path_reverse;
    for(int i=tmp.size()-1;i>=0;i--){
        tmp_path_reverse.push_back(tmp[i]);
    }
    return tmp_path_reverse;

}


WPS RRT_Base::get_rrt_path(Tree_t &first_tree,Tree_t &sec_tree){
    
    WPS tmp;
    int father_index =(first_tree.end()-1)->father_index;
    tmp.push_back(*(first_tree.end()-1));
    std::stringstream ss;
    ss<<"->"<<(first_tree.end()-1)->index<<"("<<(*(first_tree.end()-1)).x<<")("<<(*(first_tree.end()-1)).y<<")";
    while(father_index>-1){
        tmp.push_back(first_tree[father_index]);
        // ss<<"->"<<first_tree[father_index].index<<"("<<first_tree[father_index].p.x<<")("<<first_tree[father_index].p.y<<")";
        father_index=first_tree[father_index].father_index;
        if(father_index == -1) break;
    }

    WPS tmp_path_reverse;
    for(int i=tmp.size()-1;i>=0;i--){
        tmp_path_reverse.push_back(tmp[i]);
    }
    return tmp_path_reverse;

}

void RRT_Base::simplify_rrt_trees(WPS &path,OBSTACLE_GRID_MAP &obstaclesMap){
    WPS tmp_path;
    tmp_path.push_back(path[0]);
    
    for(int i=0;i<path.size();i++){
        bool found=false;
        for(int j=i+1;j<path.size();j++){
            if(check_if_path_ok(path[i],path[j],obstaclesMap)){
            }
            else{
                tmp_path.push_back(path[j-1]);
                i=j-1;
                found=true;
                break;
            }
        }
        if(!found){
            tmp_path.push_back(path[path.size()-1]);
            break;
        }
    }
    path=tmp_path;
}
bool CONNECT_RRT::Check_If_Target_OK(OBSTACLE_GRID_MAP &obstaclesMap,FLY_PLAN_T &current_route){

	WP WayPoint(_p->target_pos_x/obstaclesMap.grid_size+obstaclesMap.center_x,_p->target_pos_y/obstaclesMap.grid_size+obstaclesMap.center_y,0);
	double  length = 0.4;
	double new_x,new_y;
	bool point_found=false;
	if(obstaclesMap((int)WayPoint.x,(int)WayPoint.y)>0) {
		while(!point_found)
		for(double rad =-M_PI ;rad<=M_PI;rad+=M_PI_4){
			new_x = cos(rad)*length+WayPoint.x;
			new_y = sin(rad)*length+WayPoint.y;
			if(obstaclesMap((int)new_x,(int)new_y)<0){
				point_found=true;
				current_route._target_Pos.x=new_x;
				current_route._target_Pos.y=new_y;
				rout("-------------Warning---------------\r\n Target temporarily changed due to target collision.\r\nNew Target is  x:%f ,y%f",new_x,new_y);
				break;
			}
		}
		length+=0.4;
	}
	else{
		current_route._target_Pos.x=_p->target_pos_x;
		current_route._target_Pos.y=_p->target_pos_y;		
	}
	return !point_found;	//找到了新的点，返回 目标已变化，返回目标未变化	
}



CONNECT_RRT::RRT_STATE_C CONNECT_RRT::minimumsnap_calc(
  
                             copter_local_pos_att_t attpos
                            ,WPS &path_input
                            ,WPS &path_output
                            )
{

    WPS ret_wp;
    minimumsnap_srv->request.waypointsx.clear();
    minimumsnap_srv->request.waypointsy.clear();
    minimumsnap_srv->request.waypointsz.clear();
    minimumsnap_srv->request.waypointsid.clear();
    minimumsnap_srv->request.startpva.clear();
    minimumsnap_srv->request.endpva.clear();

    for(int i=0;i<path_input.size();i++)
    {
        minimumsnap_srv->request.waypointsx.push_back(path_input[i].x);
        minimumsnap_srv->request.waypointsy.push_back(path_input[i].y);
        minimumsnap_srv->request.waypointsz.push_back(path_input[i].z);
        minimumsnap_srv->request.waypointsid.push_back(0);
    }
    {
        minimumsnap_srv->request.startpva.push_back(0);//px
        minimumsnap_srv->request.startpva.push_back(0);//py
        minimumsnap_srv->request.startpva.push_back(0);//pz
        minimumsnap_srv->request.startpva.push_back(0);//vx
        minimumsnap_srv->request.startpva.push_back(0);//vy
        minimumsnap_srv->request.startpva.push_back(0);//vz
        minimumsnap_srv->request.startpva.push_back(0);//ax
        minimumsnap_srv->request.startpva.push_back(0);//ay
        minimumsnap_srv->request.startpva.push_back(0);//az
        minimumsnap_srv->request.endpva.push_back(0);//px
        minimumsnap_srv->request.endpva.push_back(0);//py
        minimumsnap_srv->request.endpva.push_back(0);//pz
        minimumsnap_srv->request.endpva.push_back(0);//vx
        minimumsnap_srv->request.endpva.push_back(0);//vy
        minimumsnap_srv->request.endpva.push_back(0);//vz
        minimumsnap_srv->request.endpva.push_back(0);//ax
        minimumsnap_srv->request.endpva.push_back(0);//ay
        minimumsnap_srv->request.endpva.push_back(0);//az
    }
    if(minimumsnap_client->call(*minimumsnap_srv))
    {
        if(minimumsnap_srv->response.success==true){
            minimumsnap_srv->response.ret_coefficients;
            minimumsnap_srv->response.ret_n_poly;
            minimumsnap_srv->response.ret_n_order;
            for(uint32_t i=0;i<minimumsnap_srv->response.ret_wpx.size();i++){
                ret_wp.push_back(
                    WP(
                        minimumsnap_srv->response.ret_wpx[i]
                        ,minimumsnap_srv->response.ret_wpy[i]
                        ,minimumsnap_srv->response.ret_wpz[i]
                    )   
                );
            }
            path_output=ret_wp;
            return RRT_STATE_C::MINIMUMSNAP_SUCCESS;
        }
        else{
            std::stringstream ss;
            ss<<minimumsnap_srv->response.wrong_code;
            rout("Calc service wrong.Details:%s.",ss.str().c_str());
            return RRT_STATE_C::MINIMUMSNAP_FAILED;
        }
    }
    else{
        rout("Calc service wrong. Details:Time out.");
        return RRT_STATE_C::MINIMUMSNAP_SRV_TIME_OUT_FAILED;
    }
}
class LOG{
    void to_log(){
        std::ofstream outputfile;
        outputfile.open(file_path.str().c_str(),ios::app);
		outputfile.setf(std::ios::fixed);
		outputfile.precision(10);
        outputfile<<data.str();
        outputfile.close();
    }

    std::stringstream file_path;
    virtual void title()=0;
    public:
    std::stringstream data;

    LOG(std::string file = "calc_time_log"){
        static std::stringstream init_time;
        std::time_t today_time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        init_time<< std::ctime(&today_time);
        file_path<<"/home/az/rcc/"<<"calc_time_log";
        mkdir(file_path.str().c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
        fstream _file;
        file_path<<"/calc_time_log.csv";  
		data.setf(std::ios::fixed);
		data.precision(10);
    }
    template<typename T>
    LOG &operator<< (T value){
		
        data<<value<<";";
        return *this;
    }
    void end(){
        data<<"\r\n";
        to_log();
    }
};
class CALC_TIME_LOG:public LOG{
    void title(){
        (*this)<<"起始位置x"<<"起始位置y"<<"结束位置x"<<"结束位置y";
        (*this)<<"RRT 开始时间"<<"RRT 结束时间"<<"总迭代次数";
        (*this)<<"是否找到RRT路径";
        (*this)<<"树1大小"<<"树2大小"<<"简化后尺寸"<<"路径长度";
		(*this) << "MS开始时间" << "MS结束时间";
		(this->data)<<"\r\n";
    }
public:
    CALC_TIME_LOG()
    {
        title();
    }

};

WPS CONNECT_RRT::get_RRT_Path(Tree_t RRTtree){
    WPS path;
    int last_idx = RRTtree.path_end_idx;
    while(last_idx>-1){
        path.push_back(RRTtree[last_idx]);
        last_idx=RRTtree[last_idx].father_index;
    }
    return path;
}

WPS CONNECT_RRT::combine_two_rrt_path(Tree_t RRTtree1,Tree_t RRTtree2){
    WPS path;

    WPS path1=get_RRT_Path(RRTtree1);

    WPS path2=get_RRT_Path(RRTtree2);

    for(int i = path1.size()-1;i>=0;i--){//树1 正向添加
        path.push_back(path1[i]);
    }
    for(int i = 0;i<path2.size();i++){//树2 反向添加
        path.push_back(path2[i]);
    } 
    return path;
}

CONNECT_RRT::RRT_STATE_C CONNECT_RRT::plan(
                                 copter_local_pos_att_t att_pos_copy      
                                ,OBSTACLE_GRID_MAP &obstaclesMap
                                ,FLY_PLAN_T &current_route
                                ,cv::Mat &Path
                                ,std::vector<Tree_t> &get_trees
                                ){
    auto attpos = att_pos_copy;
    cv::Mat tmp(obstaclesMap.map.rows(),obstaclesMap.map.cols(),CV_8UC1);
    CALC_TIME_LOG log;
    if(_p->show_path_map){
        cv::eigen2cv(obstaclesMap.map,tmp);
        tmp.convertTo(tmp,CV_8UC3);
    }
    Mat img =tmp;
    if(_p->show_path_map){
        cvtColor(tmp, img, CV_GRAY2BGR);
    }
	TreeNode_t start(WP(attpos.pos_x,attpos.pos_y,attpos.pos_z),0,-1,-1);
	TreeNode_t target(WP(current_route._target_Pos.x,current_route._target_Pos.y,current_route._target_Pos.z),0,-2,-2);

    if(_p->show_path_map){
        lineimg(img,start,target,obstaclesMap,Scalar(255, 90, 90));
        circleimg(img,start,obstaclesMap,Scalar(0,255,0));
        circleimg(img,WP(attpos.pos_x,attpos.pos_y,0),obstaclesMap,Scalar(0,255,255));
        circleimg(img,target,obstaclesMap,Scalar(0,0,255));
    }
    
    log<<start.x<<start.y<<target.x<<target.y;

	rout("\r\n RRT Mission Started.\r\nstart x:%f,y:%f, end x:%f,y:%f.",start.x,start.y,target.x,target.y);
	Tree_t RRTtree_1(start,"start"),RRTtree_2(target,"target");

	set_direct();
	bool find_path=false;	int iter=0;  
    std::stringstream ss;
    if(_p->show_path_map)
    {
        cv::Mat tmp;
        cv::flip(img,tmp,0);
        cv::imshow("Path_map",tmp);
    }
 
    uint64_t iter_cnt=0;
    std::stringstream start_time ;
    ros::WallTime start_time_mark = ros::WallTime::now();
    start_time << start_time_mark.toSec();
	while(iter<_p->rrt_one_step_max_iterations)
	{
        if(ros::WallTime::now()-start_time_mark>ros::WallDuration(1.2)){
            log.end();
            return RRT_STATE_C::RRT_FAILED;
            break;
        }
        iter_cnt++;
        // rout("dddd");
		TreeNode_t newPoint_Tree_1,newPoint_Tree_2;
		if(check_if_rrt_status()){
			get_sampling_points(newPoint_Tree_1,start,target);
			set_direct();
		}
		else{
            if(RRTtree_1(0,"father_index")==-1){
                newPoint_Tree_1 = TreeNode_t(target);
            }
            else{
                newPoint_Tree_1 = TreeNode_t(start); 
            }
		}

        bool RRT1_Extend_Success=true;

	    TreeNode_t cloest_point = get_nearst_point(RRTtree_1,newPoint_Tree_1,target);

		double rad = calc_grow_theta(cloest_point,newPoint_Tree_1);

		newPoint_Tree_1 = extend_a_step(cloest_point,_p->rrt_step_size,rad);
        
		if(!check_if_path_ok(cloest_point,newPoint_Tree_1,obstaclesMap)){
			iter++;
			set_rrt();
            RRT1_Extend_Success=false;
			continue;
		}else{
            RRT1_Extend_Success=true;
        }
        newPoint_Tree_2=newPoint_Tree_1;
        bool RRT2_Extend_Success=false;

        tapimg(img,target,obstaclesMap);

        if (RRT1_Extend_Success==true){//如果第一个点路径检查通过，那么尝试添加第二个点

            if(check_if_new_point_too_close_to_other_points(RRTtree_1,newPoint_Tree_1,_p->rrt_too_close_size,cloest_point.index)){
                iter++;
                continue; //如果第一个点就没有添加成功，那就重新采样
            }
            RRTtree_1(cloest_point,newPoint_Tree_1);//树1添加第一个点

            if(_p->show_path_map){
                tapimg(img,newPoint_Tree_1,obstaclesMap);
            }

            TreeNode_t cloest_point = get_nearst_point(RRTtree_2,newPoint_Tree_2,target);

            double rad = calc_grow_theta(cloest_point,newPoint_Tree_2);

            newPoint_Tree_2 = extend_a_step(cloest_point,_p->rrt_step_size,rad);

            if(!check_if_path_ok(cloest_point,newPoint_Tree_1,obstaclesMap)){
                iter++;RRT2_Extend_Success=false;//不直接跳出,用标志位是为了让第一个点能够正常添加
            }else{
                RRT2_Extend_Success=true;
            }
            if (RRT2_Extend_Success==true){ //如果第二树节点障碍检测通过，那么检测是否过近
                if(check_if_new_point_too_close_to_other_points(RRTtree_2,newPoint_Tree_2,_p->rrt_too_close_size,cloest_point.index)){
                    RRT2_Extend_Success=false;
                }else{//如果没有过近，那么添加新的树二点
                    RRTtree_2(cloest_point,newPoint_Tree_2);
                    if(_p->show_path_map){
                        tapimg(img,newPoint_Tree_2,obstaclesMap);
                    }   
                    RRT2_Extend_Success=true;
                }
            }
            if(RRT2_Extend_Success==true){
                TreeNode_t cloest_point_in_tree_1 = get_nearst_point(RRTtree_1,newPoint_Tree_2);
                //判断两棵树是否相连了
                if(check_if_reached_goal(cloest_point_in_tree_1,newPoint_Tree_2,_p->rrt_reach_goal_thresh)){
                    rout("%f %f",cloest_point_in_tree_1.x,cloest_point_in_tree_1.y);
                    RRTtree_1.set_path_end(cloest_point_in_tree_1);
                    RRTtree_2.set_path_end(RRTtree_2.size()-1);
                    find_path=true;
                    break;
                }
            }
        }       
        if(RRTtree_1.size()>=RRTtree_2.size()){
            RRTtree_1.swap(RRTtree_2);
        }
		iter=0;
	}
    std::stringstream end_time ;
	end_time.setf(std::ios::fixed);
	end_time.precision(10);
    end_time << ros::WallTime::now().toSec();

    rout("RRT start time %s,end time:%s.Iteration :%d",start_time.str().c_str(),end_time.str().c_str(),iter_cnt);

    log<<start_time.str()<<end_time.str()<<iter_cnt;

    log<<find_path;

    get_trees.push_back(RRTtree_1);
    get_trees.push_back(RRTtree_2);

	if(find_path==true){

        std::stringstream ss;
		ss.setf(std::ios::fixed);
		ss.precision(10);
        ss<<"finished size"<<RRTtree_1.size()+RRTtree_2.size()<<"\r\n";
        log<<RRTtree_1.size()<<RRTtree_2.size();

        WPS path=combine_two_rrt_path(RRTtree_1,RRTtree_2);

		auto simplified_path = path;
        {
            ss<<"total finished size"<<path.size()<<"\r\n";
            log<<path.size();
        }
		simplify_rrt_trees(simplified_path,obstaclesMap);

        {
            log<<simplified_path.size();
            ss<<"Path simplified to:"<<simplified_path.size();
		    rout("%s",ss.str().c_str());
        }
        WPS ret_wps=simplified_path;

        if(_p->minimumsnap_en){
            if(simplified_path.size()>2){
                double start_time = ros::WallTime::now().toSec();
                switch(minimumsnap_calc(attpos
                                            ,simplified_path
                                            ,ret_wps))
                {
                    case RRT_STATE_C::MINIMUMSNAP_SUCCESS:
                        break;
                    case RRT_STATE_C::MINIMUMSNAP_FAILED:
                        return RRT_STATE_C::MINIMUMSNAP_FAILED;
                    case RRT_STATE_C::MINIMUMSNAP_SRV_TIME_OUT_FAILED:
                        return RRT_STATE_C::MINIMUMSNAP_SRV_TIME_OUT_FAILED;
                }
                double end_time = ros::WallTime::now().toSec();
                rout("MinimumSnap start time %f,end time:%f.",start_time,end_time);
                log<<start_time<<end_time;
            }else{
                ret_wps.clear();
                ret_wps.push_back(start);
                for (int i = 0 ; i <40;i++){
                    ret_wps.push_back(WP(start.x+i*(target.x-start.x)/40.,start.y+i*(target.y-start.y)/40.,start.z+i*(target.z-start.z)/40.));
                }
                ret_wps.push_back(target);
            }
        }
        current_route.wps=ret_wps;
        if(_p->show_path_map){
            for(int i=0;i<ret_wps.size()-1;i++){
                lineimg(img,ret_wps[i],ret_wps[i+1],obstaclesMap);
                route_vis->push_to_rviz(ret_wps);
                rrt_vis->push_to_rviz(RRTtree_1);
                rrt_vis2->push_to_rviz(RRTtree_2);
            }
        }
        log.end();
	}
	else{
        rout("no found");
        log.end();
	    return RRT_STATE_C::RRT_FAILED;
	}
    if(_p->show_path_map){
        cv::flip(img,img,0);
        Path = img;
        cv::imshow("Path_map",img);
        cv::waitKey(1);
    }
	return RRT_STATE_C::FOUND_AVALIABLE_PATH;
}

CONNECT_RRT::Tree_t CONNECT_RRT::rrt_to_point(Tree_t RRTtree,TreeNode_t target,OBSTACLE_GRID_MAP &obstaclesMap){
    uint32_t iter_cnt=0;
    uint32_t iter=0;
    auto start = RRTtree.get_root_node();
    
    if(check_if_reached_goal(get_nearst_point(RRTtree,target),target,_p->rrt_reach_goal_thresh*2)){
        rout("SKIP RRT TO POINT");
        return RRTtree;
    }
	while(iter<_p->rrt_one_step_max_iterations)
	{
        iter_cnt++;
		TreeNode_t newPoint_Tree;
		if(check_if_rrt_status()){
			get_sampling_points(RRTtree[RRTtree.get_root_index()],start,target);
			set_direct();
		}
		else{
            newPoint_Tree = TreeNode_t(target); 
		}

	    TreeNode_t cloest_point = get_nearst_point(RRTtree,newPoint_Tree,target);

		double rad = calc_grow_theta(cloest_point,newPoint_Tree);

		newPoint_Tree = extend_a_step(cloest_point,_p->rrt_step_size,rad);
        
		if(!check_if_path_ok(cloest_point,newPoint_Tree,obstaclesMap)){
			iter++;
			set_rrt();
			continue;
		}

        if(check_if_new_point_too_close_to_other_points(RRTtree,newPoint_Tree,_p->rrt_too_close_size,cloest_point.index)){
            iter++;
            continue; //如果第一个点就没有添加成功，那就重新采样
        }
        RRTtree(cloest_point,newPoint_Tree);//树1添加一个点

        //判断是否添加成功
        if(check_if_reached_goal(newPoint_Tree,target,_p->rrt_reach_goal_thresh)){
		    if(check_if_path_ok(target,newPoint_Tree,obstaclesMap)){
                rout("RRTtree %d x:%f y:%f",RRTtree.get_root_index(),newPoint_Tree.x,newPoint_Tree.y);
                RRTtree.set_path_end(newPoint_Tree);
                break;
            }
        }
		iter=0;
	}
    return RRTtree;
}

std::vector<int> CONNECT_RRT::find_all_son_index(Tree_t RRTtree,int start_index,std::vector<int>list,bool *finded_mask){

    for(int i = 0;i<RRTtree.size();i++){ //遍历
        if(finded_mask[i]) continue;//如已查过则跳过，这是为了避免多次检查，可能存在重复的工作
        if(RRTtree[i].father_index==start_index){ //如果该树的父节点为起始节点
            list.push_back(i);    //往list中添加
            finded_mask[i]=true;  //标记为已经查过
            // rout("f.2%d   %d",i,RRTtree[i].father_index);
            list= find_all_son_index(RRTtree,i,list,finded_mask);//递归
        }
    }
    // rout("f.3");
    return list;
}


CONNECT_RRT::Tree_t CONNECT_RRT::trim_tree(Tree_t RRTtree,OBSTACLE_GRID_MAP &obstaclesMap){

    
    int *change_of_tree_index = new int[RRTtree.size()]();

    std::vector<int> output_list;
    // rout("trim 1");
    for (int i=0;i<RRTtree.size();i++){
        if(i!=RRTtree.get_root_index()&&RRTtree[i].father_index>-1){
            if(!check_if_path_ok(RRTtree[i],RRTtree[RRTtree[i].father_index],obstaclesMap)){
                std::vector<int> tmp_list;
                bool *finded_mask = new bool[RRTtree.size()]();
                tmp_list=find_all_son_index(RRTtree,i,tmp_list,finded_mask);
                //rout("tmp_list_size %d",tmp_list.size());
                delete [] finded_mask;
                if(!tmp_list.empty()) {
                    output_list.insert(output_list.end(),tmp_list.begin(),tmp_list.end());//列表合并
                    sort(output_list.begin(),output_list.end());//列表排序
                    output_list.erase(unique(output_list.begin(),output_list.end()),output_list.end());//列表去重，去重前必须排序
                }else{
                    // rout("empty List");
                }
            }
        }        
    }
    // rout("trim 2");
    int tmp_value=0;

    for (uint32_t i =0;i<output_list.size();i++){  //确定更新后的索引
        // rout("output_list:%d",output_list[i]);
        for(int j=0;j<RRTtree.size();j++){
            if(output_list[i]<RRTtree[j].father_index){
                change_of_tree_index[j]-=1;
                // rout("%d %d",j,change_of_tree_index[j]);
            }
        }
        if(output_list[i]<RRTtree.get_root_index()){
             tmp_value--;
        }
        //rout("%s tmp_value%d   %d  %d",RRTtree.name.c_str(),output_list[i],tmp_value,RRTtree.get_root_index());
    }

 
    //rout("current root index %d",RRTtree.get_root_index());
    RRTtree.set_root_index(RRTtree.get_root_index()+tmp_value);
    for (int i=0;i<RRTtree.size();i++){  //更新所有的索引到新的索引上
        if(change_of_tree_index[i]!=0){
            RRTtree[i].father_index+=change_of_tree_index[i];
        }
    }
    //rout("new root index %d",RRTtree.get_root_index());

    // rout("trim 4");
    
    for(uint32_t i = 0;i<output_list.size();i++){//删除指定位置元素;
        std::vector<TreeNode_t>::iterator it = RRTtree.begin()+output_list[i]-i; 
        RRTtree.erase(it);    
    }
    for (uint32_t i=0;i<RRTtree.size();i++){  //更新所有的索引到新的索引上
        RRTtree[i].index=i;
        //rout("RRTtree[i].index=%d",RRTtree[i].index);
    }
    // rout("trim 5");

    delete []change_of_tree_index; //释放临时变量
    return RRTtree; //返回
}

CONNECT_RRT::RRT_STATE_C CONNECT_RRT::plan_online(
                                 copter_local_pos_att_t att_pos_copy      
                                ,OBSTACLE_GRID_MAP &obstaclesMap
                                ,FLY_PLAN_T &current_route
                                ,cv::Mat &Path
                                ,std::vector<Tree_t> &get_trees
                                ,std::vector<WPS> &save_route
                                ){
    auto attpos = att_pos_copy;
    cv::Mat tmp(obstaclesMap.map.rows(),obstaclesMap.map.cols(),CV_8UC1);
    CALC_TIME_LOG log;
    if(_p->show_path_map){
        cv::eigen2cv(obstaclesMap.map,tmp);
        tmp.convertTo(tmp,CV_8UC3);
    }
    Mat img =tmp;
    if(_p->show_path_map){
        cvtColor(tmp, img, CV_GRAY2BGR);
    }
	TreeNode_t start(WP(attpos.pos_x,attpos.pos_y,attpos.pos_z),0,-1,-1);
	TreeNode_t target(WP(current_route._target_Pos.x,current_route._target_Pos.y,current_route._target_Pos.z),0,-1,-1);

    if(_p->show_path_map){
        lineimg(img,start,target,obstaclesMap,Scalar(255, 90, 90));
        circleimg(img,start,obstaclesMap,Scalar(0,255,0));
        circleimg(img,WP(attpos.pos_x,attpos.pos_y,0),obstaclesMap,Scalar(0,255,255));
        circleimg(img,target,obstaclesMap,Scalar(0,0,255));
    }
    
    log<<start.x<<start.y<<target.x<<target.y;

	rout("RRT Mission Started");
	rout("start x:%.6f,y:%.6f, end x:%.6f,y:%.6f.",start.x,start.y,target.x,target.y);

	Tree_t RRTtree_1(start,"start"),RRTtree_2(target,"target");

	set_direct();
	bool find_path=false;	int iter=0;    auto grid_size = obstaclesMap.grid_size;
    std::stringstream ss;
    if(_p->show_path_map)
    {
        cv::Mat tmp;
        cv::flip(img,tmp,0);
        cv::imshow("Path_map",tmp);
    }
 
    uint64_t iter_cnt=0;
    std::stringstream start_time;
	start_time.setf(std::ios::fixed);
	start_time.precision(10);
    static Tree_t RRTtree_1_Last,RRTtree_2_Last;
    ros::WallTime start_time_mark = ros::WallTime::now();
    start_time << start_time_mark.toSec();

    //计算树和起始树和终点树的距离
    if(RRTtree_1_Last.size()!=0&&RRTtree_2_Last.size()!=0) {
        double s_2_t_1 = calc_dist(get_nearst_point(RRTtree_1_Last,start),start);
        double s_2_t_2 = calc_dist(get_nearst_point(RRTtree_2_Last,start),start);
        if(s_2_t_1<s_2_t_2)
        {
            RRTtree_1=rrt_to_point(RRTtree_1_Last,start,obstaclesMap);
            int idx_1 = get_nearst_point_index(RRTtree_1,start);
            RRTtree_1.reorder(idx_1);//重排列
            rout("tree1 reorder to %d",idx_1);
            RRTtree_1=trim_tree(RRTtree_1,obstaclesMap);//修剪


            RRTtree_2=rrt_to_point(RRTtree_2_Last,target,obstaclesMap);
            int idx_2 = get_nearst_point_index(RRTtree_2,target);
            RRTtree_2.reorder(idx_2);//
            RRTtree_2=trim_tree(RRTtree_2,obstaclesMap);

            // rout("tree2 reorder to %d",idx_2);
            // rout("%d %d",RRTtree_1.size(),RRTtree_2.size());
        
        }
        //距离树一较远
        if(s_2_t_1>s_2_t_2){//ro
            // rout("far tree1 ");
            RRTtree_2=rrt_to_point(RRTtree_2_Last,target,obstaclesMap);
            int idx_2 = get_nearst_point_index(RRTtree_2,target);
            RRTtree_2.reorder(idx_2);//
            RRTtree_2=trim_tree(RRTtree_2,obstaclesMap);

        }
    }
    

	while(iter<_p->rrt_one_step_max_iterations)
	{
        find_path=false;
        if(ros::WallTime::now()-start_time_mark>ros::WallDuration(2)){
            log.end();
            return RRT_STATE_C::RRT_FAILED;
            break;
        }
        iter_cnt++;
		TreeNode_t newPoint_Tree_1,newPoint_Tree_2;
		if(check_if_rrt_status()){
			get_sampling_points(newPoint_Tree_1,start,target);
			set_direct();
		}
		else{
            if(RRTtree_1.name=="start"){
                newPoint_Tree_1 = TreeNode_t(target);
            }
            else{
                newPoint_Tree_1 = TreeNode_t(start); 
            }
		}

        bool RRT1_Extend_Success=true;
	    TreeNode_t cloest_point = get_nearst_point(RRTtree_1,newPoint_Tree_1,target);
		double rad = calc_grow_theta(cloest_point,newPoint_Tree_1);

		newPoint_Tree_1 = extend_a_step(cloest_point,_p->rrt_step_size,rad);
		if(!check_if_path_ok(cloest_point,newPoint_Tree_1,obstaclesMap)){
			iter++;
			set_rrt();
            RRT1_Extend_Success=false;
			continue;
		}else{
            RRT1_Extend_Success=true;
        }

        newPoint_Tree_2=newPoint_Tree_1;
        bool RRT2_Extend_Success=false;
        if(_p->show_path_map)
            tapimg(img,target,obstaclesMap);

        if (RRT1_Extend_Success==true){//如果第一个点路径检查通过，那么尝试添加第二个点

            if(check_if_new_point_too_close_to_other_points(RRTtree_1,newPoint_Tree_1,_p->rrt_too_close_size,cloest_point.index)){
                iter++;
                continue; //如果第一个点就没有添加成功，那就重新采样
            }

            RRTtree_1(cloest_point,newPoint_Tree_1);//树1添加第一个点
            TreeNode_t cloest_point_in_tree_2 = get_nearst_point(RRTtree_2,newPoint_Tree_1);//检测是否相连
            if(check_if_reached_goal(cloest_point_in_tree_2,newPoint_Tree_1,_p->rrt_reach_goal_thresh)){
        
                if(check_if_path_ok(cloest_point_in_tree_2,newPoint_Tree_1,obstaclesMap)){
                    RRTtree_1.set_path_end(RRTtree_1.size()-1);
                    RRTtree_2.set_path_end(cloest_point_in_tree_2);
                    find_path=true;
                    break;
                }
            }
            rrt_new_point_vis->push_to_rviz(WPS(newPoint_Tree_1));

            if(_p->show_path_map){
                tapimg(img,newPoint_Tree_1,obstaclesMap);
            }
            bool tree_2_keep_add = true;

            auto tree_2_target = newPoint_Tree_2;

            bool get_closest_point_one_time =false;
            TreeNode_t cloest_point ;
            while(tree_2_keep_add){
                if(!get_closest_point_one_time){
                    cloest_point = get_nearst_point(RRTtree_2,newPoint_Tree_2,target);
                    get_closest_point_one_time=true;
                }
                double rad = calc_grow_theta(cloest_point,tree_2_target);

                newPoint_Tree_2 = extend_a_step(cloest_point,_p->rrt_step_size,rad);

                if(!check_if_path_ok(cloest_point,newPoint_Tree_1,obstaclesMap)){
                    iter++;RRT2_Extend_Success=false;//不直接跳出,用标志位是为了让第一个点能够正常添加
                    tree_2_keep_add=false;
                }else{
                    RRT2_Extend_Success=true;
                }
                if (RRT2_Extend_Success==true){ //如果第二树节点障碍检测通过，那么检测是否过近
                    if(check_if_new_point_too_close_to_other_points(RRTtree_2,newPoint_Tree_2,_p->rrt_too_close_size,cloest_point.index)){
                        RRT2_Extend_Success=false;
                        tree_2_keep_add=false;
                    }else{//如果没有过近，那么添加新的树二点
                        RRTtree_2(cloest_point,newPoint_Tree_2);
                        rrt_new_point_vis->push_to_rviz(WPS(newPoint_Tree_1));
                        if(_p->show_path_map){
                            tapimg(img,newPoint_Tree_2,obstaclesMap);
                        }   
                        RRT2_Extend_Success=true;
                        tree_2_keep_add=false;
                        TreeNode_t cloest_point_in_tree_1 = get_nearst_point(RRTtree_1,newPoint_Tree_2);
                        //判断两棵树是否相连了
                        if(check_if_reached_goal(cloest_point_in_tree_1,newPoint_Tree_2,_p->rrt_reach_goal_thresh)){
                            // rout("%f %f",cloest_point_in_tree_1.x,cloest_point_in_tree_1.y);
                            if(check_if_path_ok(cloest_point_in_tree_1,newPoint_Tree_2,obstaclesMap)){
                                RRTtree_1.set_path_end(cloest_point_in_tree_1);
                                RRTtree_2.set_path_end(RRTtree_2.size()-1);
                                tree_2_keep_add=false;
                                find_path=true;
                                break;
                            }
                        }  
                        cloest_point= newPoint_Tree_2;
                    }

                }
            }
            if(find_path) break;
        }       
        if(RRTtree_1.size()>=RRTtree_2.size()){
            RRTtree_1.swap(RRTtree_2);
        }
		iter=0;
	}
    rrt_new_point_vis->clear();
    std::stringstream end_time ;
	end_time.setf(std::ios::fixed);
	end_time.precision(10);
    end_time << ros::WallTime::now().toSec();

    if(RRTtree_1.name!="start"){ //保证tree1是起始树 tree2 是终点树
        RRTtree_1.swap(RRTtree_2);
        // rout("%s %s",RRTtree_1.name.c_str(),RRTtree_2.name.c_str());
    }
    rrt_vis->push_to_rviz(RRTtree_1);
    rrt_vis2->push_to_rviz(RRTtree_2);
    rout("RRT start time %s,end time:%s.Iteration :%d",start_time.str().c_str(),end_time.str().c_str(),iter_cnt);
    rout("RRT1 root %d,end :%d,size %d",RRTtree_1.get_root_index(),RRTtree_1.path_end_idx,RRTtree_1.size());
    rout("RRT2 root %d,end :%d,size %d",RRTtree_2.get_root_index(),RRTtree_2.path_end_idx,RRTtree_2.size());


    RRTtree_1_Last = RRTtree_1;
    RRTtree_2_Last = RRTtree_2;
    log<<start_time.str()<<end_time.str()<<iter_cnt;

    log<<find_path;

    get_trees.push_back(RRTtree_1);
    get_trees.push_back(RRTtree_2);

	if(find_path==true){

        std::stringstream ss;

        ss<<"finished size"<<RRTtree_1.size()+RRTtree_2.size()<<"\r\n";
        log<<RRTtree_1.size()<<RRTtree_2.size();

        WPS path=combine_two_rrt_path(RRTtree_1,RRTtree_2);
        
        rrt_path_vis->push_to_rviz(path);

		auto simplified_path = path;
        {
            ss<<"total finished size"<<path.size()<<"\r\n";
            log<<path.size();
        }
		simplify_rrt_trees(simplified_path,obstaclesMap);
        simplified_vis->push_to_rviz(simplified_path);
        save_route.push_back(simplified_path);
        save_route.push_back(path);

        {
            log<<simplified_path.size();
            ss<<"Path simplified to:"<<simplified_path.size();
		    rout("%s",ss.str().c_str());
        }
        WPS ret_wps=simplified_path;
        
        if(_p->minimumsnap_en)
        if(simplified_path.size()>2){
            double start_time = ros::WallTime::now().toSec();
            switch(minimumsnap_calc(attpos
                                        ,simplified_path
                                        ,ret_wps))
            {
                case RRT_STATE_C::MINIMUMSNAP_SUCCESS:
                    break;
                case RRT_STATE_C::MINIMUMSNAP_FAILED:
                    return RRT_STATE_C::MINIMUMSNAP_FAILED;
                case RRT_STATE_C::MINIMUMSNAP_SRV_TIME_OUT_FAILED:
                    return RRT_STATE_C::MINIMUMSNAP_SRV_TIME_OUT_FAILED;
            }
            double end_time = ros::WallTime::now().toSec();
            rout("MinimumSnap start time: %.10f,end time:%.10f.",start_time,end_time);
            log<<start_time<<end_time;
        }else{
            ret_wps.clear();
            ret_wps.push_back(start);
            for (int i = 0 ; i <40;i++){
                ret_wps.push_back(WP(start.x+i*(target.x-start.x)/40.,start.y+i*(target.y-start.y)/40.,start.z+i*(target.z-start.z)/40.));
            }
            ret_wps.push_back(target);
        }
        current_route.wps=ret_wps;
        if(_p->show_path_map){
            for(int i=0;i<ret_wps.size()-1;i++){
                lineimg(img,ret_wps[i],ret_wps[i+1],obstaclesMap);
            }
        }
        route_vis->push_to_rviz(ret_wps);

        log.end();
	}
	else{
        rout("no found");
        log.end();
	    return RRT_STATE_C::RRT_FAILED;
	}
    if(_p->show_path_map){
        cv::flip(img,img,0);
        Path = img;
        cv::imshow("Path_map",img);
        cv::waitKey(1);
    }
	return RRT_STATE_C::FOUND_AVALIABLE_PATH;
}



void CONNECT_RRT::set_rrt(){
    grow_status=GROW_STATUS::rrt;
}

void CONNECT_RRT::set_direct(){
    grow_status=GROW_STATUS::direct;
}
bool CONNECT_RRT::check_if_rrt_status(){
    if(grow_status==GROW_STATUS::rrt){
        return true;
    }
    else{
        return false;
    }
}
void CONNECT_RRT::get_sampling_points(TreeNode_t &new_point,const TreeNode_t start ,const TreeNode_t end){

    double rand_num_x = (*rd)()%100000000/100000000.;
    double rand_num_y = (*rd)()%100000000/100000000.;

    double s =10;
    double xR=end.x>start.x?end.x:start.x;
    double xL=end.x>start.x?start.x:end.x;
    double yR=end.y>start.y?end.y:start.y;
    double yL=end.y>start.y?start.y:end.y;

    double x = rand_num_x*(xR-xL+2*s)+xL-s;
    double y = rand_num_y*(yR-yL+2*s)+yL-s;

    new_point.x=x;
    new_point.y=y;
}

void CONNECT_RRT::get_sampling_points(TreeNode_t &new_point){

    double rand_num_x = (*rd)()%100000000/100000000.;
    double rand_num_y = (*rd)()%100000000/100000000.;

    double rad = rand_num_x*(200)-100;
    double leng = rand_num_y*200-100;
    
    double x = rad;//leng*sin(rad);
    double y = leng;//leng*cos(rad);

    new_point.x=x;
    new_point.y=y;
}


CONNECT_RRT::CONNECT_RRT(std::random_device *_rd_,std::mutex *_mtx_,Parameters *_p_,UNIVERSAL_STATE *_unity_,UAVCONTROL_INTERFACE *_mavlink_p_,PROBABILISTIC_MAP *_grid_map_):RRT_Base(_rd_,_mtx_,_p_,_unity_,_mavlink_p_,_grid_map_){
    
    ros::NodeHandle nh;
    minimumsnap_client = std::make_shared<ros::ServiceClient>(nh.serviceClient<minimumsnap_route::service>("minimumsnap_route/minimumsnap_calc_request"));
    minimumsnap_srv = std::make_shared<minimumsnap_route::service>();
	minimumsnap_srv->request.request=true;
	minimumsnap_srv->request.flyspeed=1.;
	minimumsnap_srv->request.mode="normal";

    route_vis = route_vis->init("minimumsnap","marker","line_strip","refresh");
    route_vis->set_attribue(0.5,0,0.8);
    rrt_vis = rrt_vis->init("rrt","marker","point","refresh");
    rrt_vis2 = rrt_vis2->init("rrt2","marker","point","refresh");
    rrt_vis->set_attribue(1,0.8,0);
    rrt_vis2->set_attribue(139./255.,69./255.,19./255.);
    land_mark_vis = land_mark_vis->init("land_mark","marker","cylinder","add");
    land_mark_vis->set_attribue(1,1,1,1,0.2,0.2,5);
    check_radius_vis = check_radius_vis->init("check_radius_vis","marker","cylinder","refresh");
    check_radius_vis->set_attribue(0.5,0.5,0.5,0.2,8,8,3);

    simplified_vis=simplified_vis->init("simplified_path","marker","line_strip","refresh");
    rrt_path_vis=rrt_path_vis->init("rrt_path","marker","line_strip","refresh");
    simplified_vis->set_attribue(0,0,1,0.5);
    rrt_path_vis->set_attribue(255./255.,69./255.,19./255.,0.8);

    rrt_new_point_vis=rrt_new_point_vis->init("rrt_new_point_vis","marker","point","refresh");
    rrt_new_point_vis->set_attribue(1,1,1,1,3,3,3);
}


void CONNECT_RRT::Create_Thread(PROBABILISTIC_MAP &GridMap
                            ,FLY_PLAN_T &current_route)
{
    ros::Rate *loop_rate = new ros::Rate(20);
    RRT_STATE_C RRT_Calc_State = RRT_STATE_C::READY_TO_CALC;
    std::stringstream ss;ss<<std::this_thread::get_id();
    
    rout("rrt_thread started .Thead id: %s",ss.str().c_str());
    static std::stringstream init_time;
    std::time_t today_time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    init_time<< std::ctime(&today_time);

    OBSTACLE_GRID_MAP obstaclesMap;
    while(1){
        // rout("hellow world rRT.");
        // rout("%d",_unity->first_boot);
        if(_unity->new_map_avaliable ==false) {  //如果计算失败 那就停下来  继续计算
            if(RRT_Calc_State==RRT_STATE_C::TRY_AGAIN){ 
                loop_rate->sleep();
            }else{
                loop_rate->sleep();
                continue;
            }
        }

        std::stringstream time;
        std::time_t today_time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        time<< std::ctime(&today_time)<<"_"<<(*rd)()%100;
        auto map_copy = GridMap.Gridmap.map;
        if(_p->record_to_file)
        {
            std::ofstream outputfile;
            std::stringstream file_path;
            file_path<<"/home/az/rcc/"<<init_time.str();
            mkdir(file_path.str().c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
            file_path<<"/"<<time.str();
            mkdir(file_path.str().c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
            file_path<<"/originMat.csv";
            outputfile.open(file_path.str().c_str());
            outputfile<<GridMap.Gridmap.center_x<<";"<<GridMap.Gridmap.center_y<<";"<<GridMap.Gridmap.grid_size<<";"<<std::endl;
            for(int j =0;j<map_copy.cols();j++){
                for(int i=0;i<map_copy.rows();i++){
                    outputfile<<map_copy(i,j)<<";";
                }
                outputfile<<std::endl;
            }
            outputfile.close();
        }

        _unity->new_map_avaliable=false;

        auto img = GridMap.get_whole_map();           auto tar1 = GridMap.get_whole_map();
        
        cv::threshold(img,img,_p->map_occupied_thresh*255,255,CV_THRESH_BINARY);
        cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(_p->dilate_second_x,_p->dilate_second_y));
        cv::dilate(img, tar1, element);

        if(_p->show_rt_map){
            cv::imshow("Origin_map",tar1);
        }
        // rout("hellow ");
        if(!_unity->flight_task_started) continue;

        obstaclesMap=GridMap.Gridmap;

        cv::cv2eigen(tar1,obstaclesMap.map);

        if(_p->record_to_file)
        {
            std::ofstream outputfile;
            std::stringstream file_path;
            file_path<<"/home/az/rcc/"<<init_time.str()<<"/"<<time.str()<<"/obstacleMapMat.csv";
            outputfile.open(file_path.str().c_str());
            outputfile<<GridMap.Gridmap.center_x<<";"<<GridMap.Gridmap.center_y<<";"<<GridMap.Gridmap.grid_size<<";"<<std::endl;
            for(int j =0;j<obstaclesMap.map.cols();j++){
                for(int i=0;i<obstaclesMap.map.rows();i++){
                    outputfile<<obstaclesMap.map(i,j)<<";";
                }
                outputfile<<std::endl;
            }
            outputfile.close();
        }
        // 如果有新地图，那就把当前的路径放进去检查一下有没有碰了,如果有的话先停下来,如果已经是二次尝试状态，那就跳过这个过程
        bool current_route_ocllision = false;
        copter_local_pos_att_t att_pos_copy = _mavlink->get_pose();
        // check_radius_vis->push_to_rviz(WPS(WP(att_pos_copy.pos_x,att_pos_copy.pos_y,att_pos_copy.pos_z)));
        if(RRT_Calc_State != RRT_Base::RRT_STATE_C::TRY_AGAIN)
        {
            // rout("current_route.size()%d",current_route.size());
            if(current_route.size()>1)//当从未进行过规划时跳过
            {
                int tmp_idx = (_unity->current_idx-5);
                tmp_idx<0?0:tmp_idx;
                for(int i = 0;i<current_route.wps.size()-1;i++){
                    if(!check_if_path_ok(current_route.wps[i],current_route.wps[i+1],obstaclesMap)) //如果路径不可行 那么就查找
                    {
                        double tmp  = calc_dist(current_route.wps[i],WP(att_pos_copy.pos_x,att_pos_copy.pos_y,att_pos_copy.pos_z));
                        if(tmp<_p->rrt_route_refresh_thresh)
                        {
                            mtx->lock();
                            rout("Current route ocllision check failed in Danger zone.");
                            _unity->copter_state = UNIVERSAL_STATE::COPTER_STATE::STATE_LOITER;
                            mtx->unlock();
                            current_route_ocllision = true; 
                        }
                        break;
                    }
                }
                if(!current_route_ocllision) continue;
            }	
        }
        
        cv::Mat PathMat;
        std::vector<Tree_t> get_trees;
        std::vector<WPS> save_route;
        cv::Mat element2 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size((int)_p->dilate_first_x, (int)_p->dilate_first_y));
        dilate(img, img, element2);	 
        obstaclesMap.map.setZero();	
        cv::cv2eigen(img,obstaclesMap.map);

        Check_If_Target_OK(obstaclesMap,current_route);
        static int RRT_Faided_Cnt = 0;
        rout("Route Planning.");

        // land_mark_vis->push_to_rviz(WPS(WP(att_pos_copy.pos_x,att_pos_copy.pos_y,att_pos_copy.pos_z)));

        switch(plan_online(att_pos_copy,obstaclesMap,current_route,PathMat,get_trees,save_route))
        {
            case RRT_STATE_C::FOUND_AVALIABLE_PATH:
                mtx->lock();
                _unity->copter_state = UNIVERSAL_STATE::COPTER_STATE::STATE_AUTO;
                mtx->unlock();
                RRT_Calc_State=RRT_STATE_C::READY_TO_CALC;
                RRT_Faided_Cnt=0;
            break;
            case RRT_STATE_C::RRT_FAILED:
                if(++RRT_Faided_Cnt>5){
                    rout("Warning:RRT_Failed %d times",RRT_Faided_Cnt);
                    rout("Deduction: Map_Wrong.Operation:Clear Grid Map.");
                    // GridMap.tree->clear();
                    rout("Deduction: Map_Wrong.Operation: Grid Map Cleared.");
                    loop_rate->sleep();
                    loop_rate->sleep();
                    loop_rate->sleep();
                    // GridMap.Gridmap.map.setZero();
                    RRT_Calc_State=RRT_STATE_C::TRY_AGAIN;
                    RRT_Faided_Cnt=0;
                }else{
                    rout("Warning:RRT_Failed %d times",RRT_Faided_Cnt);
                    RRT_Calc_State=RRT_STATE_C::TRY_AGAIN;
                }
                mtx->lock();
                 _unity->copter_state = UNIVERSAL_STATE::COPTER_STATE::STATE_LOITER;
                mtx->unlock();
            break;
            case RRT_STATE_C::MINIMUMSNAP_FAILED:
            case RRT_STATE_C::MINIMUMSNAP_SRV_TIME_OUT_FAILED:
            case RRT_STATE_C::OTHER_FAILED:
                mtx->lock();
                _unity->copter_state = UNIVERSAL_STATE::COPTER_STATE::STATE_LOITER;
                mtx->unlock();
                rout("Find a avaliable Path Failed .Ready to Try Again.");
                RRT_Calc_State=RRT_STATE_C::TRY_AGAIN;
            break;
        }

        {
            std::vector<WPS> save_route_copy=save_route;
            if(_p->record_to_file)
            {
                if(PathMat.empty()){rout("empty .!");}
                std::stringstream file_path;
                file_path<<"/home/az/rcc/"<<init_time.str()<<"/"<<time.str()<<"/PathMapMat_Flipped.png";
                // cv::imwrite(file_path.str().c_str(),PathMat);
                file_path.str("");
                file_path<<"/home/az/rcc/"<<init_time.str()<<"/"<<time.str()<<"/Path.csv";
                std::ofstream outputfile;
                outputfile.open(file_path.str().c_str());
                for(int j =0;j<current_route.wps.size();j++){
                    outputfile<<current_route.wps[j].x<<";";
                    outputfile<<current_route.wps[j].y<<";";
                    outputfile<<current_route.wps[j].z<<";";
                    outputfile<<std::endl;
                }

                for(int tree_idx=0;tree_idx<get_trees.size();tree_idx++){
                    file_path.str("");
                    file_path<<"/home/az/rcc/"<<init_time.str()<<"/"<<time.str()<<"/RRT_"<<tree_idx<<"_Path.csv";
                    std::ofstream outputfile;
                    outputfile.open(file_path.str().c_str());
                    for(int j =0;j<get_trees[tree_idx].size();j++){
                        outputfile<<get_trees[tree_idx][j].index<<";";
                        outputfile<<get_trees[tree_idx][j].x<<";";
                        outputfile<<get_trees[tree_idx][j].y<<";";
                        outputfile<<get_trees[tree_idx][j].z<<";";
                        outputfile<<get_trees[tree_idx][j].father_index<<";";
                        outputfile<<std::endl;
                    }
                    rout("RRT_%d Recorded,size%d,file_path:%s",tree_idx,get_trees[tree_idx].size(),file_path.str().c_str());
                    outputfile.close();
                }

                for(int i = 0 ; i<save_route.size();i++){
                    if(i==0){
                        file_path.str("");
                        file_path<<"/home/az/rcc/"<<init_time.str()<<"/"<<time.str()<<"/simplified_Path.csv";
                        std::ofstream outputfile;
                        outputfile.open(file_path.str().c_str());
                        for(int j =0;j<save_route[0].size();j++){
                            outputfile<<save_route[0][j].x<<";";
                            outputfile<<save_route[0][j].y<<";";
                            outputfile<<save_route[0][j].z<<";";
                            outputfile<<std::endl;
                        }
                        rout("simplified_path Recorded,size%d,file_path:%s",save_route[0].size(),file_path.str().c_str());
                        outputfile.close();
                    }
                    if(i==1){
                        file_path.str("");
                        file_path<<"/home/az/rcc/"<<init_time.str()<<"/"<<time.str()<<"/rrt_Path.csv";
                        std::ofstream outputfile;
                        outputfile.open(file_path.str().c_str());
                        for(int j =0;j<save_route[1].size();j++){
                            outputfile<<save_route[1][j].x<<";";
                            outputfile<<save_route[1][j].y<<";"; 
                            outputfile<<save_route[1][j].z<<";";
                            outputfile<<std::endl;
                        }
                        rout("rrt_path Recorded,size%d,file_path:%s",save_route[1].size(),file_path.str().c_str());
                        outputfile.close();
                    }
                }
            }
        }
    }
}