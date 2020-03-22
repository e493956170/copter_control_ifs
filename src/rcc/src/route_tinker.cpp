#include <route_tinker.h>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <time.h>
#include "route_tracker.h"
#include "pcl/filters/voxel_grid.h"
#include "Eigen/Dense"
#include "Eigen/Eigen"
#include "sensor_msgs/point_cloud_conversion.h"
#include <pcl/common/common.h>
#include<opencv2/opencv.hpp>
#include<opencv2/core/eigen.hpp>

#include <chrono>
using namespace cv;



RRT_Base::TreeNode_t RRT_Base::get_nearst_point(Tree_t trees,TreeNode_t new_point){
    double min_dist = calc_dist(trees[0],new_point);
    int min_dist_index = 0;
    for (int i=1;i<trees.size();i++){
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
    TreeNode_t tmp = cloest_point;
    return tmp;
}


bool RRT_Base::single_point_check(WP newPoint ,OBSTACLE_GRID_MAP  &obstaclesMap){
    if(obstaclesMap((int)newPoint.x,(int)newPoint.y)>0) {
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

bool RRT_Base::push_new_point(Tree_t &trees,TreeNode_t cloest_point,TreeNode_t new_point){
    TreeNode_t tmp;
    tmp=new_point;
    tmp.index=trees.size();
    tmp.father_index=cloest_point.index;
    // rout("father index -> %d self index -> %d ",tmp.father_index,tmp.index);
    tmp.children_index=-1;
    trees[cloest_point.index].children_index=tmp.index;
    trees.push_back(tmp);
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
    int i =0;
    WPS tmp_path;
    tmp_path.push_back(path[0]);
    
    for(int i=0;i<path.size();i++){
        bool found=false;
        int farest_idx = i;
        for(int j=i+1;j<path.size();j++){
            if(check_if_path_ok(path[i],path[j],obstaclesMap)){
                // rout("path ok");
            }
            else{
                // if(farest_idx<j-1) farest_idx=j-1;
                tmp_path.push_back(path[j-1]);
                i=j-1;
                found=true;
                break;
                // rout("path wrong");
            }
        }
        if(!found){
            tmp_path.push_back(path[path.size()-1]);
            break;
        }
    }
    path=tmp_path;
}
bool Direct_RRT::Check_If_Target_OK(OBSTACLE_GRID_MAP &obstaclesMap,FLY_PLAN_T &current_route){

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



Direct_RRT::RRT_STATE_C Direct_RRT::minimumsnap_calc(
  
                             copter_local_pos_att_t attpos
                            ,WPS &path_input
                            ,WPS &path_output
                            )
{

    WPS ret_wp;
    bool minimum_route=false;
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
            for(int i=0;i<minimumsnap_srv->response.ret_wpx.size();i++){
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
Direct_RRT::RRT_STATE_C Direct_RRT::plan(
                                 copter_local_pos_att_t att_pos_copy      
                                ,OBSTACLE_GRID_MAP &obstaclesMap
                                ,FLY_PLAN_T &current_route
                                ,cv::Mat &Path
                                ,std::vector<Tree_t> &get_trees
                                ){
    auto attpos = att_pos_copy;
    cv::Mat tmp(obstaclesMap.map.rows(),obstaclesMap.map.cols(),CV_8UC1);
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

	rout("\r\n RRT Mission Started.\r\nstart x:%f,y:%f, end x:%f,y:%f.",start.x,start.y,target.x,target.y);
	Tree_t RRTtree_1(start),RRTtree_2(target);

	set_direct();
	bool find_path=false;	int iter=0;    auto grid_size = obstaclesMap.grid_size;
    std::stringstream ss;
    if(_p->show_path_map)
    {
        cv::Mat tmp;
        cv::flip(img,tmp,0);
        cv::imshow("Path_map",tmp);
    }

    // double start_time = ros::Time::now().toSec();    
    uint64_t iter_cnt=0;
    std::stringstream start_time ;
    start_time << ros::Time::now().toSec();
    auto start_time_mark= ros::Time::now();

	while(iter<_p->rrt_one_step_max_iterations)
	{
        if(ros::Time::now()-start_time_mark>ros::Duration(1.2)){
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
                newPoint_Tree_1 = target;
            }
            else{
                newPoint_Tree_1 = start; 
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
    end_time << ros::Time::now().toSec();

    rout("RRT start time %s,end time:%s.Iteration :%d",start_time.str().c_str(),end_time.str().c_str(),iter_cnt);

    get_trees.push_back(RRTtree_1);
    get_trees.push_back(RRTtree_2);

	if(find_path==true){

        std::stringstream ss;

        ss<<"finished size"<<RRTtree_1.size()<<"\r\n";
        WPS path1;
        int last_idx = RRTtree_1.path_end_idx;
        while(last_idx>-1){
            path1.push_back(RRTtree_1[last_idx]);
            last_idx=RRTtree_1(last_idx,"father_idx");
        }
        WPS path2;
        last_idx = RRTtree_2.path_end_idx;
        while(last_idx>-1){
            path2.push_back(RRTtree_2[last_idx]);
            last_idx=RRTtree_2(last_idx,"father_idx");
        }
        WPS path;
        if(RRTtree_1(0,"father_idx")==-1&&RRTtree_2(0,"father_idx")==-2){
            for(int i = path2.size()-1;i>=0;i--){
                path.push_back(path2[i]);
                // rout("dd");
            } 
 
            for(int i = 0;i<path1.size();i++){
                path.push_back(path1[i]);
            }
        }
        else{
            for(int i = path1.size()-1;i>=0;i--){
                path.push_back(path1[i]);
            }
            for(int i = 0;i<path2.size();i++){
                path.push_back(path2[i]);
            }
        }
        std::reverse(path.begin(),path.end());

        ss<<"   RRT Info\nstart x:"<<path[0].x<<",y:"<<path[0].y<<";\r\nend x:"<<(path.end()-1)->x<<",y:"<<(path.end()-1)->y<<"\r\n";

		auto simplified_path = path;
        ss<<"total finished size"<<path.size()<<"\r\n";
		simplify_rrt_trees(simplified_path,obstaclesMap);

        ss<<"Path simplified to:"<<simplified_path.size();

		rout("%s",ss.str().c_str());
        WPS ret_wps=simplified_path;

        if(_p->minimumsnap_en)
        if(simplified_path.size()>2){
            double start_time = ros::Time::now().toSec();
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
            double end_time = ros::Time::now().toSec();
            rout("MinimumSnap start time %f,end time:%f.",start_time,end_time);
        }
        current_route.wps=ret_wps;
        if(_p->show_path_map){
            for(int i=0;i<ret_wps.size()-1;i++){
                lineimg(img,ret_wps[i],ret_wps[i+1],obstaclesMap);
            }
            route_vis->push_to_rviz(ret_wps);
            rrt_vis->push_to_rviz(RRTtree_1);
            rrt_vis2->push_to_rviz(RRTtree_2);
        }
	}
	else{
        rout("no found");
	    return RRT_STATE_C::RRT_FAILED;
        // simplified_path.push_back(WP(1,0,0));
	}
    if(_p->show_path_map){
        cv::flip(img,img,0);
        Path = img;
        cv::imshow("Path_map",img);
        cv::waitKey(1);
    }
	return RRT_STATE_C::FOUND_AVALIABLE_PATH;
}

void Direct_RRT::set_rrt(){
    grow_status=GROW_STATUS::rrt;
}

void Direct_RRT::set_direct(){
    grow_status=GROW_STATUS::direct;
}
bool Direct_RRT::check_if_rrt_status(){
    if(grow_status==GROW_STATUS::rrt){
        return true;
    }
    else{
        return false;
    }
}
void Direct_RRT::get_sampling_points(TreeNode_t &new_point,const TreeNode_t start ,const TreeNode_t end){

    double rand_num_x = (*rd)()%100000000/100000000.;
    double rand_num_y = (*rd)()%100000000/100000000.;

    double s =20;
    double xR=end.x>start.x?end.x:start.x;
    double xL=end.x>start.x?start.x:end.x;
    double yR=end.y>start.y?end.y:start.y;
    double yL=end.y>start.y?start.y:end.y;

    double x = rand_num_x*(xR-xL+2*s)+xL-s;
    double y = rand_num_y*(yR-yL+2*s)+yL-s;

    new_point.x=x;
    new_point.y=y;
}

void Direct_RRT::get_sampling_points(TreeNode_t &new_point){

    double rand_num_x = (*rd)()%100000000/100000000.;
    double rand_num_y = (*rd)()%100000000/100000000.;

    double rad = rand_num_x*(200)-100;
    double leng = rand_num_y*200-100;
    
    double x = rad;//leng*sin(rad);
    double y = leng;//leng*cos(rad);

    new_point.x=x;
    new_point.y=y;
}


Direct_RRT::Direct_RRT(std::random_device *_rd_,std::mutex *_mtx_,Parameters *_p_,UNIVERSAL_STATE *_unity_,UAVCONTROL_INTERFACE *_mavlink_p_,PROBABILISTIC_MAP *_grid_map_):RRT_Base(_rd_,_mtx_,_p_,_unity_,_mavlink_p_,_grid_map_){
    
    ros::NodeHandle nh;
    minimumsnap_client = std::make_shared<ros::ServiceClient>(nh.serviceClient<minimumsnap_route::service>("minimumsnap_route/minimumsnap_calc_request"));
    minimumsnap_srv = std::make_shared<minimumsnap_route::service>();
	minimumsnap_srv->request.request=true;
	minimumsnap_srv->request.flyspeed=1.;
	minimumsnap_srv->request.mode="normal";

    route_vis = std::make_shared<visualizer_marker>(visualizer_marker("minimumsnap","marker","line_strip","refresh"));
    route_vis->set_attribue(0.5,0,0.8);
    rrt_vis = std::make_shared<visualizer_marker>(visualizer_marker("rrt","marker","point","refresh"));
    rrt_vis2 = std::make_shared<visualizer_marker>(visualizer_marker("rrt2","marker","point","refresh"));
    rrt_vis->set_attribue(0.8,1,1);
    rrt_vis2->set_attribue(0.2,1,1);
    land_mark_vis = std::make_shared<visualizer_marker>(visualizer_marker("land_mark","marker","cube","add"));
    land_mark_vis->set_attribue(1,1,1,0.5,0.2,0.2,5);

}


void Direct_RRT::Create_Thread(PROBABILISTIC_MAP &GridMap
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
        if(_p->record_to_file)
        {
            std::ofstream outputfile;
            std::stringstream file_path;
            file_path<<"/home/az/rcc/"<<init_time.str();
            mkdir(file_path.str().c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
            file_path<<"/"<<time.str();
            mkdir(file_path.str().c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
            file_path<<"/originMat.csv";
            outputfile<<GridMap.Gridmap.center_x<<";"<<GridMap.Gridmap.center_y<<";"<<GridMap.Gridmap.grid_size<<";"<<std::endl;
            outputfile.open(file_path.str().c_str());
            for(int j =0;j<GridMap.Gridmap.map.cols();j++){
                for(int i=0;i<GridMap.Gridmap.map.rows();i++){
                    outputfile<<GridMap.Gridmap.map(i,j)<<";";
                }
                outputfile<<std::endl;
            }
            outputfile.close();
        }

        _unity->new_map_avaliable=false;

        auto img = GridMap.get_whole_map();           auto tar1 = GridMap.get_whole_map();
        
        cv::threshold(img,img,_p->map_occupied_thresh*255,255,CV_THRESH_BINARY);
        cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(_p->dilate_second_x,_p->dilate_second_y));
        dilate(img, tar1, element);

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
        int ocllision_index =-1;
        copter_local_pos_att_t att_pos_copy = _mavlink->get_pose();

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
                            ocllision_index=i;
                        }
                        break;
                    }
                }
                if(!current_route_ocllision) continue;
            }	
        }

        cv::Mat PathMat;
        std::vector<Tree_t> get_trees;
        cv::Mat element2 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size((int)_p->dilate_first_x, (int)_p->dilate_first_y));
        dilate(img, img, element2);	
        obstaclesMap.map.setZero();	
        cv::cv2eigen(img,obstaclesMap.map);

        Check_If_Target_OK(obstaclesMap,current_route);
        static int RRT_Faided_Cnt = 0;
        rout("Route Planning.");

        land_mark_vis->push_to_rviz(WPS(WP(att_pos_copy.pos_x,att_pos_copy.pos_y,att_pos_copy.pos_z)));

        switch(plan(att_pos_copy,obstaclesMap,current_route,PathMat,get_trees))
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
                    GridMap.tree->clear();
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
                outputfile<<std::endl;}

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
        }
    }
}