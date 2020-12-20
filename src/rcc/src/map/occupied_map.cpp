#include "map/occupied_map.h"
#include "octomap_msgs/Octomap.h"
#include "octomap_msgs/conversions.h"
#include "base/baseType.h"
using namespace cv;

//声明advertise，octomap rviz plug in 默认接受topic为octomap_full的message
void ProbabilisticMap::push_to_rviz()
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

ProbabilisticMap::ProbabilisticMap(std::mutex *_mtx_,Parameters *_p_,UniversalState *_unity_,UAVControlInterface *_mavlink_p_)
                    :_mtx(_mtx_),_p(_p_),_unity(_unity_),_mavlink_p(_mavlink_p_){
    int x = _p->map_size_x;
    int y = _p->map_size_y;
    map_griz_size = _p->map_grid_size;
    Gridmap.map_expand_size = _p->map_expand_size;
    Gridmap.grid_size=map_griz_size;
    Gridmap.map.resize(y/Gridmap.grid_size,x/Gridmap.grid_size);
    Gridmap.map.setZero();
    occupied_thresh=_p->map_occupied_thresh;

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
    lidar_data_vis = lidar_data_vis->init("lidar_data_vis","marker","point","refresh");
    lidar_data_vis->set_attribue(0,1,0,0.7);
}

double ProbabilisticMap::check_grid(int row,int col ,int z){
    
    double p = Gridmap(row,col);  //row index   col index
    // if(p>0) rout("%f",p);
    return p;
}
void ProbabilisticMap::update_grid(WP _wp_ ,GRID_STATUS hit_or_miss){
    bool _hit_or_miss =hit_or_miss==GRID_STATUS::HIT?true:false;
    tree->updateNode(_wp_.x,_wp_.y,_wp_.z,_hit_or_miss);
}

double ProbabilisticMap::check_grid(WP _wp_){
    octomap::OcTreeNode *point;
    if((point=tree->search(_wp_.x,_wp_.y,_wp_.z))!=nullptr);
    {
        return point->getOccupancy();
    }
    return .0;
}
void ProbabilisticMap::update_from_cloud(const PTC &cloud,const PTC &edge,UAVLocalPositionAndAttitude * attpos){
//     ShadowMap.setZero();
//     //对整个点云进行处理 转化为世界localned坐标
    // WPS pointCloudInworld = ft_pts(-attpos->pos_x,-attpos->pos_y,angle_add(attpos->yaw,0),cloud);
    // WPS EdgeCloudInworld = ft_pts(-attpos->pos_x,-attpos->pos_y,angle_add(attpos->yaw,0),edge);
    OBSTACLE_GRID_MAP obstaclesMapshow;
    extern std::shared_ptr<visualize_image> edge_cloud_vis;

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
        // cv::imshow("edgecloud",img);
        edge_cloud_vis->push_to_rviz(img);

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
    tree->insertPointCloud(cloud_oc,octomap::point3d(0.1,0,0)
                            ,octomath::Pose6D(attpos->pos_x,attpos->pos_y,attpos->pos_z,attpos->roll,attpos->pitch-0.12,attpos->yaw)
                            ,45,true,true);
    // lidar_data_vis->push_to_rviz(cloud_oc);
    

    OBSTACLE_GRID_MAP tmpMap(Gridmap);
    tmpMap.map.resize(Gridmap.map.rows(),Gridmap.map.cols());
    tmpMap.map.setZero();
    double min =attpos->pos_z-1; min=min<0?0:min;
    double max = attpos->pos_z+1;
    for(octomap::OcTree::iterator it = tree->begin(16),
        end = tree->end(); it != end; ++it){
        if(tree->isNodeOccupied(*it)){
            double z = it.getZ();
            double half_size = it.getSize()/2;
            if (z + half_size > min && z - half_size < max ){  //slice
                double x = it.getX();
                double y = it.getY();
                double size = it.getSize();
                 if ((it.getDepth() ==16) && !isSpeckleNode(it)){
                    continue;
                }
                // rout("it->getOccupancy() %f",it->getOccupancy());
                tmpMap.set_grid(x/tmpMap.grid_size+tmpMap.center_x,y/tmpMap.grid_size+tmpMap.center_y,it->getOccupancy());
                // rout("tmpMap%f  get%f",tmpMap(y/tmpMap.grid_size+tmpMap.center_y,x/tmpMap.grid_size+tmpMap.center_x),it->getOccupancy());
            }
        }
    }
    _mtx->lock();
    Gridmap=tmpMap;
    _mtx->unlock();
    push_to_rviz();
}

bool ProbabilisticMap::isSpeckleNode(octomap::OcTree::iterator Node){

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
                    }
                }
            }
        }
    }
    return neighborFound;
}

cv::Mat ProbabilisticMap::get_whole_map(){
    cv::Mat img(Gridmap.map.rows(),Gridmap.map.cols(),CV_8UC1);
    for(int i = 0;i<Gridmap.map.rows();i++)
        for(int j = 0;j<Gridmap.map.cols();j++){
                img.at<uchar>(i,j)=check_grid(i,j,0)*255;
    }
    return img;
}

void ProbabilisticMap::set_center(int x,int y ){
    Gridmap.center_x=x;
    Gridmap.center_y=y;
}

void OBSTACLE_GRID_MAP::index_check(int &_row_,int &_col_){ 
    int offset_more=map_expand_size;
    if(_col_<0){
        int cur_cols=map.cols();
        int cur_rows=map.rows();
        int d_x = -_col_+offset_more;
        Eigen::MatrixXd tmp;
        
        tmp.resize(cur_rows,cur_cols+d_x);
        tmp.setZero();
        tmp.block(0,d_x,cur_rows,cur_cols)=map;
        center_x +=d_x;//offset x
        map.resizeLike(tmp);
        map.setZero();
        map=tmp;
        _col_=offset_more;
        rout("x<  ,map resize %d %d to %d %d",cur_rows,cur_cols,map.rows(),map.cols());
    }
    if(_col_>=map.cols()){
        int cur_cols=map.cols();
        int cur_rows=map.rows();
        int d_x = _col_-map.cols()+1+offset_more;

        Eigen::MatrixXd tmp;
        tmp.setZero();
        tmp.resize(cur_rows,cur_cols+d_x);
        tmp.block(0,0,cur_rows,cur_cols)=map;
        map.resizeLike(tmp);
        map.setZero();
        map=tmp;
        // x=map.cols()-1-offset_more;NO add
        rout("x>= ,map resize %d %d to %d %d",cur_rows,cur_cols,map.rows(),map.cols());
    }  
    if(_row_<0){
        int cur_cols=map.cols();
        int cur_rows=map.rows();
        int d_y = -_row_+offset_more;

        Eigen::MatrixXd tmp;
        tmp.resize(cur_rows+d_y,cur_cols);
        tmp.setZero();
        tmp.block(d_y,0,cur_rows,cur_cols)=map;
        center_y +=d_y;//offset x  
        map.resizeLike(tmp);
        map.setZero();
        map=tmp; 
        _row_=offset_more;   
        rout("y<  ,map resize %d %d to %d %d",cur_rows,cur_cols,map.rows(),map.cols());
    }

    if(_row_>=map.rows()){
        int cur_cols=map.cols();
        int cur_rows=map.rows();
        int d_y = _row_-map.rows()+1+offset_more;
        Eigen::MatrixXd tmp;
        tmp.setZero();
        tmp.resize(cur_rows+d_y,cur_cols);
        tmp.block(0,0,cur_rows,cur_cols)=map;
        map.resizeLike(tmp);
        map.setZero();
        map=tmp;
        // y=map.rows()-1-offset_more;;NO add
        rout("y>= ,map resize %d %d to %d %d",cur_rows,cur_cols,map.rows(),map.cols());
    }          
}

void OBSTACLE_GRID_MAP::set_grid(int x ,int y,double value){  //col first
        index_check(y,x);
        map(y,x)=value;
}