#include "occupied_map.h"
#include "octomap_msgs/Octomap.h"
#include "octomap_msgs/conversions.h"
#include "baseType.h"
using namespace cv;

//声明advertise，octomap rviz plug in 默认接受topic为octomap_full的message
void PROBABILISTIC_MAP::push_to_rviz()
{
    //声明message
    static int cnt=0;
    octomap_msgs::Octomap map_msg;
    //设置header
    map_msg.header.frame_id = "map";
    map_msg.header.stamp = ros::Time::now();
    map_msg.header.seq=cnt++;
    //fullMapToMsg负责转换成message
    if (octomap_msgs::fullMapToMsg(*tree, map_msg))
        pub_octomap->publish(map_msg);
    else
        ROS_ERROR("Error serializing OctoMap");
}

PROBABILISTIC_MAP::PROBABILISTIC_MAP(std::mutex *_mtx_,Parameters *_p_,UNIVERSAL_STATE *_unity_,UAVCONTROL_INTERFACE *_mavlink_p_)
                    :_mtx(_mtx_),_p(_p_),_unity(_unity_),_mavlink_p(_mavlink_p_){
    int x = _p->map_size_x;
    int y = _p->map_size_y;
    map_griz_size = _p->map_grid_size;
    Gridmap.map_expand_size = _p->map_expand_size;
    Gridmap.grid_size=map_griz_size;
    Gridmap.map.resize(y/Gridmap.grid_size,x/Gridmap.grid_size);
    Gridmap.map.setZero();
    occupied_thresh=_p->map_occupied_thresh;
    ShadowMap.resize(y/Gridmap.grid_size,x/Gridmap.grid_size);
    ShadowMap.setZero();
    set_center(y/Gridmap.grid_size/2,x/Gridmap.grid_size/2);
    update_hit=log(occupied_p_of_hit/(1-occupied_p_of_hit));
    update_miss=log(occupied_p_of_miss/(1-occupied_p_of_miss));
    
    ros::NodeHandle nh;
    pub_octomap = std::make_shared<ros::Publisher>(nh.advertise<octomap_msgs::Octomap>("octomap_full", 1, true));
    tree = std::make_shared<octomap::OcTree>(map_griz_size);
    tree->setProbHit(0.8);
    tree->setProbMiss(0.4);
    tree->setClampingThresMin(0.005);
    tree->setClampingThresMax(0.999);
}

void PROBABILISTIC_MAP::update_grid(int x ,int y ,GRID_STATUS hit_or_miss){
    if(hit_or_miss==GRID_STATUS::HIT)
        Gridmap.map(x,y)+=+update_hit; 
    if(hit_or_miss==GRID_STATUS::MISS)
        Gridmap.map(x,y)+=+update_miss;
    Gridmap.map(x,y)=Gridmap.map(x,y)>150?150:Gridmap.map(x,y);
    Gridmap.map(x,y)=Gridmap.map(x,y)<-150?-150:Gridmap.map(x,y);
}
bool PROBABILISTIC_MAP::check_grid(int x,int y ){
    
    double p = 1-1/(1+exp(Gridmap.map(x,y)));
    
    return p>occupied_thresh?true:false;
}
double PROBABILISTIC_MAP::check_grid(int x,int y ,int z){
    
    double p = 1-1/(1+exp(Gridmap.map(x,y)));
    
    return p;
}
void PROBABILISTIC_MAP::update_grid(WP _wp_ ,GRID_STATUS hit_or_miss){
    bool _hit_or_miss =hit_or_miss==GRID_STATUS::HIT?true:false;
    tree->updateNode(_wp_.x,_wp_.y,_wp_.z,_hit_or_miss);
}

double PROBABILISTIC_MAP::check_grid(WP _wp_){
    octomap::OcTreeNode *point;
    if((point=tree->search(_wp_.x,_wp_.y,_wp_.z))!=nullptr);
    {
        return point->getOccupancy();
    }
    return .0;
}

void PROBABILISTIC_MAP::update_from_cloud(const PTC &cloud,const PTC &edge,copter_local_pos_att_t * attpos){
//     ShadowMap.setZero();
//     //对整个点云进行处理 转化为世界localned坐标
    // WPS pointCloudInworld = ft_pts(-attpos->pos_x,-attpos->pos_y,angle_add(attpos->yaw,0),cloud);
    // WPS EdgeCloudInworld = ft_pts(-attpos->pos_x,-attpos->pos_y,angle_add(attpos->yaw,0),edge);
    OBSTACLE_GRID_MAP obstaclesMapshow;
    Mat img;
    if(_p->show_point_cloud)
    {
        obstaclesMapshow=this->Gridmap;
        obstaclesMapshow.center_x/=2;
        img = Mat(obstaclesMapshow.map.rows(),obstaclesMapshow.map.cols(),CV_8UC3);
        img.setTo(0);
        for(int i=0;i<edge.size();i++){
            circleimg(img,edge[i],obstaclesMapshow,Scalar(255,20,0),2);
        }
        for(int i = 0;i<cloud.size();i++){
            circleimg(img,cloud[i],obstaclesMapshow,Scalar(0,255,0),2);
        }
        imshow("edgecloud",img);

    }

    octomap::KeyRay ray;
    octomap::Pointcloud cloud_ed;
    for(int i = 0 ; i<edge.size();i++){
        cloud_ed.push_back(edge[i].x,edge[i].y,edge[i].z);     
    }
    cloud_ed.transform(octomath::Pose6D(attpos->pos_x,attpos->pos_y,attpos->pos_z,attpos->roll,attpos->pitch,attpos->yaw));
    for(auto cloud_iter = cloud_ed.begin();cloud_iter!=cloud_ed.end();cloud_iter++){
        ray.reset();
        tree->computeRayKeys(octomap::point3d(attpos->pos_x,attpos->pos_y,attpos->pos_z),*cloud_iter,ray);
        for (auto iter=ray.begin();iter!=ray.end();iter++){
            tree->updateNode(*iter, false);
        }
    }

    octomap::Pointcloud cloud_oc;
    for(int i = 0 ; i<cloud.size();i++){
        cloud_oc.push_back(cloud[i].x,cloud[i].y,cloud[i].z);     
    }
    tree->insertPointCloud(cloud_oc,octomap::point3d(0,0,0)
                            ,octomath::Pose6D(attpos->pos_x,attpos->pos_y,attpos->pos_z,attpos->roll,attpos->pitch,attpos->yaw)
                            ,45,true,true);

    OBSTACLE_GRID_MAP tmpMap(Gridmap);
    tmpMap.map.resize(Gridmap.map.rows(),Gridmap.map.cols());
    tmpMap.map.setOnes();
    tmpMap.map=-150*tmpMap.map;
    double min =attpos->pos_z-0.5; min=min<0?0:min;
    double max = attpos->pos_z+0.5;
    for(octomap::OcTree::iterator it = tree->begin(16),
        end = tree->end(); it != end; ++it){
        if(tree->isNodeOccupied(*it)){
            double z = it.getZ();
            double half_size = it.getSize()/2;
            if (z + half_size > min && z - half_size < max ){
                double x = it.getX();
                double y = it.getY();
                double size = it.getSize();
                 if ((it.getDepth() ==16) && !isSpeckleNode(it)){
                    continue;
                }   
                tmpMap.set_grid(x/tmpMap.grid_size+tmpMap.center_x,y/tmpMap.grid_size+tmpMap.center_y,150);
                // rout("%f %f %d %d",x/tmpMap.grid_size+tmpMap.center_x,y/tmpMap.grid_size+tmpMap.center_y,tmpMap.map.rows(),tmpMap.map.cols());
            }
        }
    }
    _mtx->lock();
    Gridmap=tmpMap;
    _mtx->unlock();
    push_to_rviz();
}
bool PROBABILISTIC_MAP::isSpeckleNode(octomap::OcTree::iterator Node){

    auto nKey = Node.getKey();

    octomap::OcTreeKey key;
    bool neighborFound = false;
    for (key[2] = nKey[2] - 1; !neighborFound && key[2] <= nKey[2] + 1; ++key[2]){
        for (key[1] = nKey[1] - 1; !neighborFound && key[1] <= nKey[1] + 1; ++key[1]){
            for (key[0] = nKey[0] - 1; !neighborFound && key[0] <= nKey[0] + 1; ++key[0]){
                if (key != nKey){
                    octomap::OcTreeNode* node = tree->search(key);
                    if (node && tree->isNodeOccupied(node)){
                        // we have a neighbor => break!
                        neighborFound = true;
                        return neighborFound;
                    }
                }
            }
        }
    }
    return false;
}

void PROBABILISTIC_MAP::set_grid(int x,int y,int value){
    if(x<0||y<0||x>=Gridmap.map.cols()||y>=Gridmap.map.rows()) return;
    Gridmap.map(y,x)=value;    
}
void PROBABILISTIC_MAP::set_grid(OBSTACLE_GRID_MAP *map, int x,int y,int value){
   
    if(x<0||y<0||x>=map->map.cols()||y>=map->map.rows()) return;
    map->map(y,x)=value;
}

cv::Mat PROBABILISTIC_MAP::get_whole_map(){
    cv::Mat img(Gridmap.map.rows(),Gridmap.map.cols(),CV_8UC1);
// #pragma omp parallel for
    for(int i = 0;i<Gridmap.map.rows();i++)
        for(int j = 0;j<Gridmap.map.cols();j++){
                img.at<uchar>(i,j)=check_grid(i,j,0)*255;
    }


    return img;
}
void PROBABILISTIC_MAP::set_center(int x,int y ){
    Gridmap.center_x=x;
    Gridmap.center_y=y;
}

void OBSTACLE_GRID_MAP::index_check(int &x,int &y){ 
    int offset_more=map_expand_size;
    if(x<0){
        int cur_cols=map.cols();
        int cur_rows=map.rows();
        int d_x = -x+offset_more;
        Eigen::MatrixXd tmp;
        tmp.resize(cur_rows,cur_cols+d_x);
        tmp.setZero();
        tmp.block(0,d_x,cur_rows,cur_cols)=map;
        center_x +=d_x;//offset x
        map.resizeLike(tmp);
        map.setZero();
        map=tmp;
        x=offset_more;
        rout("x<  ,map resize %d %d to %d %d",cur_rows,cur_cols,map.rows(),map.cols());
    }
    if(y<0){
        int cur_cols=map.cols();
        int cur_rows=map.rows();
        int d_y = -y+offset_more;
        Eigen::MatrixXd tmp;
        tmp.resize(cur_cols+d_y,cur_rows);
        tmp.setZero();
        tmp.block(d_y,0,cur_rows,cur_cols)=map;
        center_y +=d_y;//offset x  
        map.resizeLike(tmp);
        map.setZero();
        map=tmp; 
        y=offset_more;   
        rout("y<  ,map resize %d %d to %d %d",cur_rows,cur_cols,map.rows(),map.cols());
    }
    if(x>=map.cols()){
        int cur_cols=map.cols();
        int cur_rows=map.rows();
        int d_x = x-map.cols()+1+offset_more;
        Eigen::MatrixXd tmp;
        tmp.setZero();
        tmp.resize(cur_rows,cur_cols+d_x);
        tmp.block(0,0,cur_rows,cur_cols)=map;
        map.resizeLike(tmp);
        map.setZero();
        map=tmp;
        // x=map.cols()-1-offset_more;
        rout("x>= ,map resize %d %d to %d %d",cur_rows,cur_cols,map.rows(),map.cols());

    }  
    if(y>=map.rows()){
        int cur_cols=map.cols();
        int cur_rows=map.rows();
        int d_y = y-map.rows()+1+offset_more;
        Eigen::MatrixXd tmp;
        tmp.setZero();
        tmp.resize(cur_rows+d_y,cur_cols);
        tmp.block(0,0,cur_rows,cur_cols)=map;
        map.resizeLike(tmp);
        map.setZero();
        map=tmp;
        // y=map.rows()-1-offset_more;
        rout("x>= ,map resize %d %d to %d %d",cur_rows,cur_cols,map.rows(),map.cols());
    }          
}
void OBSTACLE_GRID_MAP::set_grid(int x ,int y,int value){  //col first
        index_check(x,y);
        // if(x<0||y<0||x>=map.cols()||y>=map.rows()) return;
        map(y,x)=value;
}