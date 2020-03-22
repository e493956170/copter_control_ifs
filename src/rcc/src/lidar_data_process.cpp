#include "lidar_data_process.h"



/*
    雷达数据处理线程

*/
void LIDAR_DATA_PROCESS::Create_Thread(){

    std::stringstream ss;ss<<std::this_thread::get_id();

    // rout("point_process_thread started .Thead id: %s",ss.str().c_str());
        // rout("new Data %d",_unity->new_cloud_avaliable_);
    copter_local_pos_att_t att_pos_copy_copy = att_pos_copy;
    _unity->new_cloud_avaliable_=false;
    if(cloud.empty()&&edgecloud.empty())return ;
    PTC d;
    pcl::VoxelGrid<PT> sor;
    sor.setInputCloud(cloud.makeShared());
    sor.setLeafSize(0.40f, 0.40f, 0.40f);
    sor.filter(d);
    PTC e;
    sor.setInputCloud(edgecloud.makeShared());
    sor.setLeafSize(1.5f, 1.5f, 1.5f);
    sor.filter(e);
    _grid_map->update_from_cloud(d,e,&att_pos_copy_copy);
    _unity->new_map_avaliable=true;
}

void LIDAR_DATA_PROCESS::lidarCloudHandler(const sensor_msgs::PointCloudConstPtr &input){

    if(!_unity->first_data_get) return;

    sensor_msgs::PointCloud2 input_;

    sensor_msgs::convertPointCloudToPointCloud2(*input,input_);

    PTC cloud_TMP,cloud_TMP2_FILTER;

    pcl::fromROSMsg(input_,cloud_TMP);
    cloud.clear();
    edgecloud.clear();
    if(_mavlink!=nullptr){
        if(abs(_mavlink->get_pose().roll)>0.8||abs(_mavlink->get_pose().pitch)>0.8)
            return;
    }
    att_pos_copy = _mavlink->get_pose();
    if(_unity->new_cloud_avaliable_== true) return;
    for (int i =0;i<cloud_TMP.size();i++){
        double dist =pow(cloud_TMP.points[i].x,2)+pow(cloud_TMP.points[i].y,2);
        if(dist<pow(40,2)&&dist>pow(0.3,2)){
            cloud.push_back(cloud_TMP.points[i]);//cloud_TMP2_FILTER
        }
        else{
            edgecloud.push_back(cloud_TMP.points[i]);
        }
    }
    _unity->new_cloud_avaliable_=true;

    std::thread _point_process_thread(&LIDAR_DATA_PROCESS::Create_Thread,this);

    _point_process_thread.join();

}