#ifndef ROUTE_FINDER
#define ROUTE_FINDER
#include "omp.h"
#include <random>
#include "route_tracker/route_tracker.h"
#include <minimumsnap_route/service.h>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include "mutex"
#include <thread>
#include <functional>
#include "base/cfg.h"
#include "base/baseMethod.h"
#include "base/baseType.h"
#include "map/occupied_map.h"
#include "vis_and_log/rviz_visualize.h"
/*                                                                                                                      
                                                                                                             G08             
                                                                                               ,f     Ltf0L0L0@@G0   C08     
                                                                                              0L1L;:18fG1i0L@@000G08G000LG   
                                                                                             8f0f8,tLi0GGG8G00880L0L00000G   
                                   8 1f                                                     0C: ;ttC0f0GGG0G8800iG0G00080G0  
                                 0L1t,:18t               8:GC8C:C.              G;:.ff0    @C   tt10CCGCCCCC0C8C0G08L0000G:  
                                0LtLtft1C8CL:         C:Ltii;;;iiGi,        C0C8G111Lt:L    08  0ttf00C8800CLG0L0008G8ttC0   
                               ,tf1t10:0   G0        ;f1;:,,::::::;tC.      G@  0C:81t:;0    80  0ttt00i1Ci1L0G0000008       
                               0Lftt;L10   @G      .:ti;,:::,,:::,:;:L     .fG  GtCtttt:;.    08  0GttCLL0GGG0G088880L       
                               ;LLt1LfL    8f   .  L1;:,,,.....,::,:1f.    0C    0tLtttt:0   L,f18 0GGGi1808880GG080i0       
                              tfLLLCtL,    iC0   ::;1;,,.       .,,,1t1    8C     Lt0LtttL  ;.,iCt:     .0CG   ,C00C         
                000G ;1G       0LGLL0      CCGGGLii;i:..        .:,;it1iLftGC     .0Lf:8G0 0::C1GGG0G00L.00                  
              ;8,tG0   00       GtG.           :f1iii:,.        ,:::;t1iG80:         8G0L 0GtGLf0C      t08                  
            CG:: ,1G80 tG       0t0             .tt11;::,.. .. ,,,,;;tLf             fG, C0 fC                               
          0:::C0tt008tt000:t8088L;CLi             .Lf,::,,,:,,::.:::tLG             00Gi80;0                                 
         tttt1::::f0LG010tC  fGC00G0                .ft11,,,,,,,i1tf.               0LGC0.                                   
        0LLfttt:8Gft111C:CCC;  .C0                   G1it.......fiiC                  0L0                                    
        0LLLLC81f1:;0Cf8Gft                          ffffttffttffffG                                                         
        :GLLLitf1L18,t1:ti..,.                        .L10     0tLC                                                          
         .tLL00f;ift1f11i;:,..1                        181iii0 ;L                                                            
           ;tGtf10tttttt1i,i:,:                      81fffi1fff10                                                            
             .80L0Gttt1:ffC8Lt:                    0CLffff11fffLC08G                                                         
              10G0GGGt0t80000t8                 0:C80GCLffLfffff0Cttt;L                                                      
                .,@GGG811000;0                 :tt1tf8ffff8ffff8LLtttt0L       0CGLiL                                        
                    ,0CtGGG0                  i001fLL10ffLCfL0t8000i00fG      0CGGGLi0                                       
                                             80ff;,,,.,G0f1LfC;,,,,.CtL8      0CLGGLif                                       
                                             CC0;;,,,,.:fL10Ci;:,,,,;800      800088Cf                                       
                                            01001;;;;;;ffGf0fti;;;;;1fGfiLf    0808tL ;1                                     
                                            0ftGCG111L8Gf;;18G80GL0G8fit;C    08t10fL0 8C                                    
                                         0GCf8ffG1800CG0G8fGGG:C;1tL0Cft     L08tt0it0 8:                                    
                                          ifCC080C0t0,t,1 t ;t.Lf0800GfC0L0;11088080010L                                     
                                          0LGGL88fGfGf80iGCG:i0fCf0L0fLLf8Cifff080tt010                                      
                                       .LGtG0000fCL00CC8CCLLfGf0CiL;008CL880f   00000                                        
                                        :80LfC.;;;;0088C0GC8G80CLCC0G0L         0CLL@                                        
                                      1CC8CCCL11111tGG01ftCi8LtGfff;   t                                                     
                                    .8CCC8t1iCttLLf8fLfttff10fCftL00                                                         
                                   C,8801:81it::.808GfGG00000C008CLC0                                                        
                                 :;1t0C1LttC0;,:G88CCCCCCCCCCCCC8808L,                                                       
                                  8L00 :000CL0,0C0L0ff0LfffLLffLG0G000                                                       
*/

class RRT_Base:public CoordinateTransform{
public:

    RRT_Base(std::random_device *_rd_,std::mutex *_mtx_,Parameters *_p_,UNIVERSAL_STATE *_unity_,UAVCONTROL_INTERFACE *_mavlink_p_,PROBABILISTIC_MAP *_grid_map_):
                    rd(_rd_),mtx(_mtx_),_p(_p_),_unity(_unity_),_mavlink(_mavlink_p_),_grid_map(_grid_map_) {}

    typedef class TREE_NODE_T:public __WAYPOINT_T__{
    public:
        TREE_NODE_T(){};
        template<typename T>
        TREE_NODE_T(T __p,int __index,int __father_index,int __children_index){
            x=__p.x;
            y=__p.y;
            z=__p.z;
            father_index=__father_index;
            children_index=__children_index;
            index=__index;
            // area_x_idx=x/5;
            // area_y_idx=y/5;
        }
        template<typename T>
        TREE_NODE_T(T __p){
            x=__p.x;
            y=__p.y;
            z=__p.z;
            // area_x_idx=x/5;
            // area_y_idx=y/5;
            // area_z_idx=y/5;
        }
        int father_index=-1; 
        int children_index=-1;
        int index=-1;
        int area_x_idx=0;
        int area_y_idx=0;
        int area_z_idx=0;
    }TreeNode_t;
    inline void tapimg( Mat &img,  TreeNode_t node ,OBSTACLE_GRID_MAP obstaclesMap){
        int x = node.x/0.2+obstaclesMap.center_x;
        int y = node.y/0.2+obstaclesMap.center_y;
        if(x<0||y<0||y>=obstaclesMap.rows()||x>=obstaclesMap.cols()) return ;

        img.at<cv::Vec3b>(y,x)[0]=255;
        img.at<cv::Vec3b>(y,x)[1]=0;
        img.at<cv::Vec3b>(y,x)[2]=255;
    }
    enum class RRT_STATE_C{
        ROUTE_NOT_IN_SCAN_ZONE,
        ROUTE_IS_SAFE,
        RRT_FAILED,
        MINIMUMSNAP_FAILED,
        MINIMUMSNAP_SRV_TIME_OUT_FAILED,
        FOUND_AVALIABLE_PATH,
        OTHER_FAILED,
        READY_TO_CALC,
        TRY_AGAIN,
        START_POS_OCLLISION,
        TARGET_POS_OCLLISION,
        MINIMUMSNAP_SUCCESS
    };

    typedef class TREE_T:public __BASE_METHOD__{
        int root_index=0;
        
    public:
        TREE_T(){};
        std::string name ; 
        TREE_T(TreeNode_t __init_node,std::string _name){name=_name;__init_node.name=name;tree.push_back(__init_node);}
        std::vector<TreeNode_t> tree;
        int size(){return tree.size();}
        void push_back(TreeNode_t __node){__node.name=name;tree.push_back(__node);}
        TreeNode_t &operator [](int __idx){
            return tree[__idx];
        }
        int &operator()(const int &__idx,const string &__cmd){
            if(__cmd=="father_idx"){
                return tree[__idx].father_index;
            }
            return tree[__idx].father_index;
        }
        void erase(std::vector<TreeNode_t>::iterator it){
            tree.erase(it);
        }
        std::vector<TreeNode_t>::iterator begin(){return tree.begin();}
        std::vector<TreeNode_t>::iterator end(){return tree.end();}

        void operator() (const TreeNode_t &father,TreeNode_t &new_node){
            new_node.index=size();
            new_node.father_index=father.index;
            // rout("father index -> %d self index -> %d ",tmp.father_index,tmp.index);
            new_node.children_index=-1;
            tree[father.index].children_index=new_node.index;
            tree.push_back(new_node);
            // new_node.index--;//dirty trick，更新new_node的索引
        }

        int get_root_index(){return root_index;}
        void set_root_index(int idx){root_index=idx;}

        std::vector<TreeNode_t> __reorder__(std::vector<TreeNode_t> tree_copy,int start_index,int &last_index){
            //由于是单链条，所以用此记录子节点
            if(tree[start_index].father_index>=0)//父节点存在
            {
                tree_copy[tree[start_index].father_index].father_index=start_index;
                last_index=start_index;
                return __reorder__(tree_copy,tree[start_index].father_index,last_index);
            }
            //父节点不存在，指向子节点
            if(last_index<0)
                tree_copy[tree[start_index].father_index].father_index=last_index;
            return tree_copy;
        }

        void reorder(int _root_index){
            int last_index=0;
            std::vector<TreeNode_t>  tree_copy=tree;
            tree=__reorder__(tree_copy,_root_index,last_index);
            this->root_index=_root_index;
            tree[root_index].father_index=-1;
            // rout("reorder_success . ");
        }
        void swap(TREE_T &__swap_obj){
            TREE_T tmp;
            tmp = *this;
            *this =__swap_obj;
            __swap_obj=tmp;
        }
        void set_path_end(TreeNode_t end_node){
            path_end_idx=end_node.index;
        }
        void set_path_end(int end_node_idx){
            path_end_idx=end_node_idx;
        }
        TreeNode_t &get_root_node(){
            return (*this)[root_index];
        }
        int path_end_idx=0;

    }Tree_t;
    int get_nearst_point_index(Tree_t trees,TreeNode_t new_point);
    TreeNode_t get_nearst_point(Tree_t trees,TreeNode_t new_point);//step2
    TreeNode_t get_nearst_point(Tree_t trees,TreeNode_t new_point,TreeNode_t target);//step2
    double calc_grow_theta(TreeNode_t cloest_point,TreeNode_t new_point);//step3.1
    double calc_grow_theta(WP cloest_point,WP new_point);
    TreeNode_t extend_a_step(TreeNode_t cloest_point,double step_size,double rad);//step3.2
    bool check_if_path_ok(WP cloest_point,WP new_point,OBSTACLE_GRID_MAP &obstaclesMap);
    bool check_if_path_ok(TreeNode_t cloest_point,TreeNode_t new_point,OBSTACLE_GRID_MAP &obstaclesMap);//step4 step0
    bool check_if_reached_goal(TreeNode_t goal_point,TreeNode_t new_point,double threshold);//step5
    bool check_if_new_point_too_close_to_other_points(Tree_t trees,TreeNode_t new_point,double threshold,int skip_idx);//step6
    WPS get_rrt_path(Tree_t &src_trees);
    WPS get_rrt_path(Tree_t &first_tree,Tree_t &sec_tree);
    void simplify_rrt_trees(WPS &path,OBSTACLE_GRID_MAP &obstaclesMap);

protected:
    bool single_point_check(WP newPoint ,OBSTACLE_GRID_MAP  &obstaclesMap);
    uint32_t max_iterations;
    PROBABILISTIC_MAP *_grid_map;
    std::random_device *rd;
    Parameters *_p;
    UNIVERSAL_STATE *_unity;
    UAVCONTROL_INTERFACE *_mavlink;
    std::mutex *mtx;
    bool find_route=false;
};


class CONNECT_RRT:public RRT_Base{

    enum class GROW_STATUS{
        direct,
        rrt
    }grow_status;

    bool Check_If_Target_OK(OBSTACLE_GRID_MAP &obstaclesMap,FLY_PLAN_T &current_route);
    std::shared_ptr<ros::ServiceClient> minimumsnap_client;
    std::shared_ptr<minimumsnap_route::service> minimumsnap_srv;
    std::shared_ptr<visualizer_marker> route_vis;
    std::shared_ptr<visualizer_marker> rrt_vis;
    std::shared_ptr<visualizer_marker> simplified_vis;
    std::shared_ptr<visualizer_marker> rrt_path_vis;
    std::shared_ptr<visualizer_marker> rrt_new_point_vis;
    std::shared_ptr<visualizer_marker> rrt_vis2;
    std::shared_ptr<visualizer_marker> land_mark_vis;
    std::shared_ptr<visualizer_marker> check_radius_vis;

    WPS get_RRT_Path(Tree_t RRTtree);
    WPS combine_two_rrt_path(Tree_t RRTtree1,Tree_t RRTtree2);

public :
    CONNECT_RRT(std::random_device *_rd_,std::mutex *_mtx_,Parameters *_p_,UNIVERSAL_STATE *_unity_,UAVCONTROL_INTERFACE *_mavlink_p_,PROBABILISTIC_MAP *_grid_map_);

    void Create_Thread(
								 PROBABILISTIC_MAP &GridMap
								,FLY_PLAN_T &current_route);
    RRT_STATE_C minimumsnap_calc(
                                copter_local_pos_att_t attpos
                                ,WPS &path_input
                                ,WPS &path_output
                                );
    
    void get_sampling_points(TreeNode_t &new_point);
    void get_sampling_points(TreeNode_t &new_point,const TreeNode_t start ,const TreeNode_t end);
    void set_sample_limit(double angle_start,double angle_end,double min_sample_dist,double max_sample_dist); //根据sick激光雷达特性设计
    void set_direct();    void set_rrt();    bool check_if_rrt_status();


    Tree_t rrt_to_point(Tree_t RRTtree,TreeNode_t target,OBSTACLE_GRID_MAP &obstaclesMap);

    Tree_t trim_tree(Tree_t RRTtree,OBSTACLE_GRID_MAP &obstaclesMap);

    std::vector<int> find_all_son_index(Tree_t RRTtree,int start_index,std::vector<int>list,bool *finded_mask);

    RRT_STATE_C plan(	         copter_local_pos_att_t att_pos_copy  
                                ,OBSTACLE_GRID_MAP &obstaclesMap
                                ,FLY_PLAN_T &current_route
                                ,cv::Mat &Path
                                ,std::vector<Tree_t> &get_trees
            );
    RRT_STATE_C plan_online(copter_local_pos_att_t att_pos_copy  
                            ,OBSTACLE_GRID_MAP &obstaclesMap
                            ,FLY_PLAN_T &current_route
                            ,cv::Mat &Path
                            ,std::vector<Tree_t> &get_trees
                            ,std::vector<WPS> &save_route 
                        );
};



#endif