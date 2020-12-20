#include "base/cfg.h"
#include <fstream>

//构造函数
Config::Config(string file_name)
{
    //定义一个配置文件
    this->file_name = file_name;
    //默认一共0个配置
    this->cfg_line = 0;
    printf("ok here");

    printf("ok here");
        //从文件读取全部配置 加入链表
    getCFG();
        //打印全部配置
        //printCfg();
}

//析构函数
Config::~Config()
{
}

//获取配置的总数
int Config::getLines()
{
    return this->cfg_line;
}

//设置某个配置项
int Config::setCFG(string key, string value)
{
    int rt = 0;
    //插入链表
    string dtype = "";
    joinHead(key, value, dtype);
    //同步到文件
    inputFile();
    return rt;
}

//内存配置同步到文件
int Config::inputFile()
{
    int rt = 1;

    return rt;
}

//获取某项特定配置
string Config::getCFG(string key)
{
    if(cfg_list.find(key)!=cfg_list.end()){
        return cfg_list[key].first;
    }else{
        return "未找到";
    }
    
}

//从文件获取全部的配置
int Config::getCFG()
{
    int rt = 0;
    //先清空链表

    //配置总数为0
    this->cfg_line = 0;
    //定义缓存字符串变量
    string st;
    string key, value, dtype;
    //定义文件变量
    ifstream infile;
    string::size_type idx,colon_idx;

    //打开文件
    infile.open(this->file_name);
    //遍历直到文件的最后
    while (getline(infile,st))
    {
        //初始化缓存
        //取得一行配置
        //找不到等号则继续

        //如果是注释那么跳过这一行
        if(st.find("--")!=string::npos) continue;

        //删去注释
        idx = st.find("#");         
        if (idx != string::npos)
        {
            st = st.substr(0, idx);
        }

        //#找到数据类型的分隔符索引
        colon_idx = st.find(":");
        if (colon_idx == string::npos)
        {
            printf("数据类型缺失。\r\n");
            continue;
        }
        //找到数据的分隔符索引
        idx = st.find("=");        
        if (idx == string::npos)
        {
            continue;
        } 
        //截断字符串得到key和value字符串
        dtype = st.substr(0,colon_idx);
        key = st.substr(colon_idx+1, idx-colon_idx-1);
        key.erase(colon_idx,key.find_first_not_of(" "));
        key.erase(key.find_last_not_of(" ")+1);
        value = st.substr(idx + 1,st.length()-idx);
        value.erase(0,value.find_first_not_of(" "));
        value.erase(value.find_last_not_of(" ")+1);
        cout<<dtype<<":"<<key<<":"<<value<<endl;
        //插入链表
        joinHead(key,value,dtype);
        st = "";
    }
    //关闭文件
    infile.close();
    return rt;
}

//将配置插入内存链表
int Config::joinHead(string key,string value,string dtype)
{
    int rt = 1;
    cfg_list[key]=std::pair<string,string>(value,dtype);
    cfg_line++;
    return rt;
}

//释放全部节点（除了头指针）
int Config::freeJoin()
{

}

//打印所有的配置
void Config::printCfg()
{

}

//创建配置链表头指针
int Config::createHead()
{

}

Parameters::Parameters(string file_path){
    cfg = std::make_shared<Config>(Config(file_path));
   
    printf("%s\r\n",cfg->getCFG("config_title").c_str());
    if(cfg->getCFG("config_title")=="UAV_TEST_PLATFORM_SOFTWARE"){
        if(cfg->getCFG("config_version")=="0.1"){
            if(cfg->getCFG("author")=="ZhengZi"){
                target_pos_x=atof(cfg->getCFG("target_pos_x").c_str());
                target_pos_y=atof(cfg->getCFG("target_pos_y").c_str());
                target_pos_z=atof(cfg->getCFG("target_pos_z").c_str());
                Vz_Kp=atof(cfg->getCFG("Vz_Kp").c_str());
                Vz_Ki=atof(cfg->getCFG("Vz_Ki").c_str());
                Vz_Kd=atof(cfg->getCFG("Vz_Kd").c_str());
                V_Kp=atof(cfg->getCFG("V_Kp").c_str());
                V_Ki=atof(cfg->getCFG("V_Ki").c_str());
                V_Kd=atof(cfg->getCFG("V_Kd").c_str());
                Yaw_Kp=atof(cfg->getCFG("Yaw_Kp").c_str());
                Yaw_Ki=atof(cfg->getCFG("Yaw_Ki").c_str());
                Yaw_Kd=atof(cfg->getCFG("Yaw_Kd").c_str());
                map_size_x=atoi(cfg->getCFG("map_size_x").c_str());
                map_size_y=atoi(cfg->getCFG("map_size_y").c_str());
                map_occupied_thresh=atof(cfg->getCFG("map_occupied_thresh").c_str());
                max_fly_speed=atof(cfg->getCFG("max_fly_speed").c_str());
                max_yaw_rad=atof(cfg->getCFG("max_yaw_rad").c_str());
                max_z_speed=atof(cfg->getCFG("max_z_speed").c_str());
                dilate_first_x=atoi(cfg->getCFG("dilate_first_x").c_str());
                dilate_first_y=atoi(cfg->getCFG("dilate_first_y").c_str());
                dilate_second_x=atoi(cfg->getCFG("dilate_second_x").c_str());
                dilate_second_y=atoi(cfg->getCFG("dilate_second_y").c_str());
                record_to_file=atoi(cfg->getCFG("record_to_file").c_str());
                show_rt_map=atoi(cfg->getCFG("show_rt_map").c_str());
                show_point_cloud=atoi(cfg->getCFG("show_point_cloud").c_str());
                rrt_one_step_max_iterations=atoi(cfg->getCFG("rrt_one_step_max_iterations").c_str());
                rrt_step_size=atof(cfg->getCFG("rrt_step_size").c_str());
                rrt_reach_goal_thresh=atof(cfg->getCFG("rrt_reach_goal_thresh").c_str());
                rrt_too_close_size=atof(cfg->getCFG("rrt_too_close_size").c_str());
                minimumsnap_en= atoi(cfg->getCFG("minimumsnap_en").c_str());
                show_path_map=atof(cfg->getCFG("show_path_map").c_str());
                rrt_route_refresh_thresh= atof(cfg->getCFG("rrt_route_refresh_thresh").c_str());      
                no_move=atof(cfg->getCFG("no_move").c_str());
                Pure_Pursuit_K=atof(cfg->getCFG("Pure_Pursuit_K").c_str());
                Pure_Pursuit_LFC=atof(cfg->getCFG("Pure_Pursuit_LFC").c_str());
                Vz_I_Max= atof(cfg->getCFG("Vz_I_Max").c_str());
                V_I_Max=atof(cfg->getCFG("V_I_Max").c_str());
                Yaw_I_Max=atof(cfg->getCFG("Yaw_I_Max").c_str());
                A_Kp=atof(cfg->getCFG("A_Kp").c_str());
                A_Ki=atof(cfg->getCFG("A_Ki").c_str());
                A_I_Max=atof(cfg->getCFG("A_I_Max").c_str());
                A_Kd=atof(cfg->getCFG("A_Kd").c_str());
                A_Max=atof(cfg->getCFG("A_Max").c_str());
                map_expand_size=atoi(cfg->getCFG("map_expand_size").c_str());
                Az_Kp=atof(cfg->getCFG("Az_Kp").c_str());
                Az_Ki=atof(cfg->getCFG("Az_Ki").c_str());
                Az_I_Max=atof(cfg->getCFG("Az_I_Max").c_str());
                Az_Kd=atof(cfg->getCFG("Az_Kd").c_str());
                acc_control=atoi(cfg->getCFG("acc_control").c_str());
                map_grid_size=atof(cfg->getCFG("map_grid_size").c_str());
                Path_Pursuit_Method=cfg->getCFG("Path_Pursuit_Method");
                cout<<("Config has been successfully loaded.");
                load_success=true;
            }
            else{
                cout<<"Author info is wrong .Exiting.";
            }
        }
        else{
            cout<<"Config_version Info"<<cfg->getCFG("config_version")<< "is wrong.Exiting.";
        }

    }else{
        printf("Config title is wrong.Exiting.");
    }
}