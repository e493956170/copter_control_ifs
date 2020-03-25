#include "base/cfg.h"
#include <fstream>

//构造函数
Config::Config(string file_name)
{
    //定义一个配置文件
    this->file_name = file_name;
    //默认一共0个配置
    this->cfg_line = 0;
    if (createHead() == 0)
    {
        //从文件读取全部配置 加入链表
        getCFG();
        //打印全部配置
        //printCfg();
    }
}

//析构函数
Config::~Config()
{
    //释放链表的各个节点
    freeJoin();
    //释放头节点
    if (this->head != NULL)
    {
        delete this->head;
    }
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
    joinHead(key, value);
    //同步到文件
    inputFile();
    return rt;
}

//内存配置同步到文件
int Config::inputFile()
{
    int rt = 1;
    if (this->head == NULL)
    {
        return rt;
    }
    //缓存字符串
    string st;
    //定义文件类型
    ofstream outfile;
    //打开文件方式
    outfile.open(this->file_name , ios::out | ios::trunc);
    // 遍历向文件写入用户输入的数据
    CFG_J * p = this->head->next;
    while (p != NULL)
    {
        //定义字符串
        st = p->key + "=" + p->value;
        //写入字符串
        outfile << st << endl;
        //移动指针
        p = p->next;
    }
    //关闭文件
    outfile.close();
    return rt;
}

//获取某项特定配置
string Config::getCFG(string key)
{
    //默认找不到
    string rt = "CANNOT FIND THIS CONFIG.";
    if (this->head == NULL)
    {
        return rt;
    }
    //遍历抓取配置项
    CFG_J *p = this->head->next;
    while (p != NULL)
    {
        if (p->key==(key))
        {
            //捕捉到则返回值
            rt = p->value;
            cout<<key<<":"<<p->value<<endl;
            break;
        }else{
        }
        p = p->next;
    }
    if(rt == "CANNOT FIND THIS CONFIG."){
        cout<<"Check for existence of key:\""<<key<<"\".";
        rt="-1";
    }
    return rt;
}

//从文件获取全部的配置
int Config::getCFG()
{
    int rt = 0;
    //先清空链表
    freeJoin();
    //配置总数为0
    this->cfg_line = 0;
    //定义缓存字符串变量
    string st;
    string key, value;
    //定义文件变量
    ifstream infile;
    string::size_type idx;
    char *p = NULL, *q = NULL;
    //打开文件
    infile.open(this->file_name);
    //遍历直到文件的最后
    while (getline(infile,st))
    {
        //初始化缓存
        //取得一行配置
        //找不到等号则继续
        idx = st.find("#");         
        if (idx != string::npos)
        {
            st = st.substr(0, idx);
        }
        idx = st.find("=");        
        if (idx == string::npos)
        {
            continue;
        }

        //截断字符串得到key和value字符串
        key = st.substr(0, idx);
        key.erase(0,key.find_first_not_of(" "));
        key.erase(key.find_last_not_of(" ")+1);
        value = st.substr(idx + 1,st.length()-idx);
        value.erase(0,value.find_first_not_of(" "));
        value.erase(value.find_last_not_of(" ")+1);
        // cout<<st<<"_________________-;"<<key<<":"<<value<<endl;
        //插入链表
        joinHead(key,value);
        st = "";

    }
    //关闭文件
    infile.close();
    return rt;
}

//将配置插入内存链表
int Config::joinHead(string key,string value)
{
    int rt = 1;
    if (this->head == NULL)
    {
        rt = 2;
        return rt;
    }
    //定义移动指针
    CFG_J * p = this->head;
    CFG_J * cur = p->next;
    while (cur != NULL)
    {
        //cout << cur->key << " " << key << " " << cur->key.compare(key) << endl;
        //找到值则直接改变
        if (cur->key.compare(key) == 0)
        {
            cur->value = value;
            rt = 0;
            break;
        }
        p = cur;
        cur = p->next;
    }
    //找不到值则再最后插入
    if (rt != 0)
    {
        CFG_J *q = new CFG_J();
        q->key = key;
        q->value = value;
        q->next = NULL;
        p->next = q;
        //配置数自增
        this->cfg_line ++;
    }
    return rt;
}

//释放全部节点（除了头指针）
int Config::freeJoin()
{
    int rt = 1;
    //定义移动指针
    CFG_J * p = this->head->next;
    CFG_J * cur = p;
    if (p == NULL)
    {
        return rt;
    }
    //遍历释放内存
    while (cur != NULL)
    {
        p = cur->next;
        delete cur;
        cur = p;
    }
    //初始化头指针
    this->head->next = NULL;
    rt = 0;
    return rt;
}

//打印所有的配置
void Config::printCfg()
{
    if (this->head == NULL)
    {
        return;
    }
    //定义移动指针
    CFG_J * p = this->head->next;
    while (p != NULL)
    {
        cout << p->key << "=" << p->value << endl;
        //移动指针
        p = p->next;
    }
}

//创建配置链表头指针
int Config::createHead()
{
    int rt = 1;
    CFG_J *p = new CFG_J();
    p->key = "headkey";
    p->value = "headvalue";
    p->next = NULL;
    this->head = p;
    rt = 0;//没有问题
    return rt;
}

Parameters::Parameters(string file_path){
    cfg=new Config(file_path);
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
        cout<<"Config title is wrong.Exiting.";
    }
}