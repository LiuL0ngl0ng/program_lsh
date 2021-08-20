#include "astardoublexml.h"
#include "ui_astardoublexml.h"
#include <QMessageBox>
#include <qdebug.h>
#include "MapSearchNode.h"
#include <QFile>
#include <QTextStream>

AStarDoubleXML::AStarDoubleXML(int argc, char** argv, QWidget *parent) :
    QMainWindow(parent)
    ,ros_pub(argc, argv) {//lsh//初始化话题的发布者和订阅者，初始化规划有关参数
/*AStarDoubleXML::AStarDoubleXML(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::AStarDoubleXML)
{
    ui->setupUi(this);  //Setupui function builds a user interface for "this" pointer*/
    this->base_dir =  QString::fromStdString(ros_pub.base_dir);//lsh//string向Qstring格式转换
    this->m_attach_xml_file.base_dir = this->base_dir;//lsh//m_attach_xml_file是一个全局规划器
    ui.setupUi(this);
    m_bRecvRndf=false;//lsh//是否读取路网的标志

    //build topological map in real time
    record_flag = true;
    end_task_node_id = -1;
    first_planning_finished = false;
    second_plan_trig_dis = 6;   //触发重规划第二次路径发送控制距离
    backup_flag = false;
    first_path_flag = false;
    first_plan_start_id = -1;
    first_planning_flag = false;
    wait_times = 0;
    wait_times_for_second_path = 0;
    backup_start_gps.longitude = 0;
    backup_start_gps.latitude = 0;
    repeat_tri_times = 0;
    ros_pub.transPathPlanner(&m_attach_xml_file);//lsh//把m_attach_xml_file的地址设为ros_pub的全局规划器指针path_planner_
    m_attach_xml_file.way_path = QString(QString::fromLocal8Bit(ros_pub.way_net_file_path.c_str()));
    //get_node_from_txt = false;       //从txt中读取任务点路网，若为false，则从xml文件读取路网
    //use_prior_map_replan = true;    //是否先验地图进行重规划
    get_node_from_txt = ros_pub.run_param.get_node_from_txt;
    use_prior_map_replan = ros_pub.run_param.use_prior_map_replan;
    open_fixed_line_patrol = ros_pub.run_param.open_fixed_line_patrol;
    m_attach_xml_file.set_patrol_times = ros_pub.run_param.set_patrol_times;
    m_attach_xml_file.open_dynamic_obs_det = ros_pub.run_param.open_dynamic_obs_det;
    m_attach_xml_file.open_foggy_det = ros_pub.run_param.open_foggy_det;
    m_attach_xml_file.if_vrep=ros_pub.run_param.vrep_simulate;
    m_attach_xml_file.m_cMapMatch.CMap_if_vrep=ros_pub.run_param.vrep_simulate;
    std::cout << "read param: " << std::endl;
    std::cout << "     get_node_from_txt:     " << std::boolalpha << get_node_from_txt << std::endl
              << "     use_prior_map_replan:  " << std::boolalpha << use_prior_map_replan << std::endl
              << "     open_dynamic_obs_det:  " << std::boolalpha << m_attach_xml_file.open_dynamic_obs_det << std::endl
              << "     open_foggy_det:        " << std::boolalpha << m_attach_xml_file.open_foggy_det << std::endl
              << "     open_fixed_line_patrol:" << std::boolalpha << open_fixed_line_patrol << std::endl
              << "     set_patrol_times: " << m_attach_xml_file.set_patrol_times << std::endl;
    //connect(this,SIGNAL(lastWindowClosed()),&ros_pub,SLOT(quit()));
}

AStarDoubleXML::~AStarDoubleXML() {
    //delete ui;
}

void AStarDoubleXML::on_ReadFile_clicked() {
//lsh//规划完成标志PathPlanFinishFlag置false，清空m_astarsearch.NodeList（xml里的点及其拓扑关系）、original_road_network_list（复制NodeList）、m_RoadList（以道路为单位存放路点）、m_path_map_list（规划结果的点序列）、v_mapping_list（匹配当前路段速度）
//lsh//将指定路径文件夹下所有xml文件读入m_astarsearch.NodeList，同时设置m_Pathid_vector、m_RoadList
//lsh//初始化显示
//lsh//将m_astarsearch.NodeList复制到original_road_network_list
    {//add pathplanfinishflag lock
        QMutexLocker locker(&m_attach_xml_file.mutex_PathPlanFinishFlag);
        //lsh//QMutexLocker类是一个方便类，简化了锁定和解锁互斥。
        //lsh//QMutex的目的是保护一个对象、数据结构或者代码段，所以同一时间只有一个线程可以访问它。
        //lsh//即当多个线程访问同一个上锁程序时，先由一个线程访问执行，执行完后另一个线程才执行，如果不加上锁解锁，两线程会随机轮流执行每一句程序
        m_attach_xml_file.PathPlanFinishFlag= false;
    }
    if(!m_attach_xml_file.original_road_network_list.isEmpty()){//lsh//original_road_network_list用于复制路网
        qDeleteAll(m_attach_xml_file.original_road_network_list);
        m_attach_xml_file.original_road_network_list.clear();
        //lsh//当T的类型为指针时，调用clear方法能置空，但并不能释放其内存。
        //lsh//qDeleteAll可以释放容器元素内存，但没有对容器的置空操作，也就是size没变。
        //lsh//所以qDeleteAll之后必须加上clear方法。
    }
    if(!m_attach_xml_file.m_astarsearch.NodeList.isEmpty()) {//lsh//xml里的点及其拓扑关系保存在m_astarsearch.NodeList
        m_attach_xml_file.m_Pathid_vector.clear();//lsh////用来存放道路ID，防止重复读入道路数据          //note
        //qDeleteAll(m_attach_xml_file.m_astarsearch.NodeList);
        m_attach_xml_file.m_astarsearch.NodeList.clear();
        if(!m_attach_xml_file.m_astarsearch.NodeList.isEmpty())
        ROS_FATAL("on_ReadFile_clicked,NodeList cannnot been cleared.");
    }
    if(!m_attach_xml_file.m_RoadList.isEmpty()) {//lsh////以道路为单位存放路点，用于道路匹配
        m_attach_xml_file.m_RoadList.clear();
    }
    if(!m_attach_xml_file.m_path_map_list.isEmpty()) {//lsh//规划结果的点序列，只包含xy等信息，还需要进行进一步分析才能用于导航
        //qDeleteAll(m_attach_xml_file.m_path_map_list);
        m_attach_xml_file.m_path_map_list.clear();
    }
    if(!m_attach_xml_file.v_mapping_list.isEmpty()){//lsh//用于匹配当前路段速度
        m_attach_xml_file.v_mapping_list.clear();
    }

    if(get_node_from_txt){
        std::cout<<"get node from txt file."<< std::endl;
        m_attach_xml_file.GetNodeFromTxtFile();
    }else{
        std::cout<<"get node from xml file."<< std::endl;
        m_attach_xml_file.GetNodeFromFile();	//读入地图xml文件
        //lsh//  将道路id存入m_Pathid_vector
        //lsh//  将包含拓扑关系和详细信息的路点录入m_astarsearch.NodeList
        //lsh//  以道路为单位储存路点的向量m_RoadList，其不包含拓扑关系
    };
    m_attach_xml_file.InitDisplay();	//初始化显示
    m_bRecvRndf = true;
    this->ros_pub.init();//lsh//什么也没有

    //copy NodeList to original_road_network_list
    if(!m_attach_xml_file.original_road_network_list.isEmpty())
        m_attach_xml_file.original_road_network_list.clear();
    QList<MapSearchNode*>::iterator road_iter = m_attach_xml_file.m_astarsearch.NodeList.begin();
    for(; road_iter != m_attach_xml_file.m_astarsearch.NodeList.end(); road_iter++){
        m_attach_xml_file.original_road_network_list.append(*road_iter);
    }

    //only for test! test the BuildTopologicalMap function
    /*QList<MapSearchNode*>::iterator intersec_iter = m_attach_xml_file.m_astarsearch.NodeList.begin();
    for(; intersec_iter != m_attach_xml_file.m_astarsearch.NodeList.end(); intersec_iter++){
        if((*intersec_iter)->intersection){
            extractroad_msg::extractroad temp_intersec_data;
            temp_intersec_data.roadcount = 2;
            temp_intersec_data.leadpoints.push_back(114.38947930);
            temp_intersec_data.leadpoints.push_back(37.83803321);
            temp_intersec_data.leadpoints.push_back(114.38950377);
            temp_intersec_data.leadpoints.push_back(37.83781871);
            temp_intersec_data.vehicle_point[0] = (*intersec_iter)->lon;
            temp_intersec_data.vehicle_point[1] = (*intersec_iter)->lat;

            original_intersec_list.append(temp_intersec_data);
        }
    }*/
}

void AStarDoubleXML::on_Pathplan_clicked() {
    ROS_INFO("plan path......");
    if(!m_attach_xml_file.m_path_map_list.isEmpty()) {
        m_attach_xml_file.m_path_map_list.clear();
    }
    if(!m_bRecvRndf) {
        on_ReadFile_clicked();
    }//lsh//若仍未读入路点文件则重新读入
    if(get_node_from_txt){//lsh//此处默认为false
        int start_id = -1;
        int end_id = -1;
        QList<MapSearchNode*>::iterator iter = m_attach_xml_file.m_astarsearch.NodeList.begin();
        for( ; iter != m_attach_xml_file.m_astarsearch.NodeList.end(); iter++) {
            if ((*iter)->type == 0)
                start_id = (*iter)->node_id;
            if ((*iter)->type == 1)
                end_id = (*iter)->node_id;
        }//lsh//找到起止点
        m_attach_xml_file.PathPlan(start_id, end_id);
    }else{
        if(!m_attach_xml_file.m_cMapMatch.TaskBeReaded) {
            on_ReadTaskFile_clicked();
        }//lsh//若仍未读入任务文件则重新读入
        ///Planning
        m_attach_xml_file.MatchTaskPoints(m_attach_xml_file.plan_task_list);  //匹配任务点
        //lsh//将任务点投影到路网中，建立连接关系并分别标注任务点是否在路网上
        m_attach_xml_file.PlanWithTaskPoint(m_attach_xml_file.plan_task_list); //在匹配了任务点的路网上规划出一条依次经过任务列表中的任务点的路径
        //lsh//利用A*分别规划出相邻两任务投影点之间的路径，将规划结果存储在m_path_map_list
    }
    //ros_pub.transPathPlanner(&m_attach_xml_file);

    QList<MapSearchNode*>::iterator path_iter = m_attach_xml_file.m_path_map_list.begin();
    std::cout << "result of path:  " << std::endl;
    for(;path_iter != m_attach_xml_file.m_path_map_list.end(); ++path_iter){
        std::cout << "   " << (*path_iter)->node_id << std::endl;
    }

    m_attach_xml_file.m_bPathPlaned = true;
    m_attach_xml_file.m_flag = 0;

    {//add pathplanfinishflag lock
        QMutexLocker locker(&m_attach_xml_file.mutex_PathPlanFinishFlag);
        m_attach_xml_file.PathPlanFinishFlag=true;
    }
    {QMutexLocker locker(&ros_pub.mutex_attach_success);
        ros_pub.attach_success_ = true;//lsh//是否已经完成了第一次规划的标志位
    }
    m_attach_xml_file.is_forward = true;

    //copy NodeList to original_road_network_list
    if(!m_attach_xml_file.original_road_network_list.isEmpty())
        m_attach_xml_file.original_road_network_list.clear();
    QList<MapSearchNode*>::iterator road_iter = m_attach_xml_file.m_astarsearch.NodeList.begin();
    for(; road_iter != m_attach_xml_file.m_astarsearch.NodeList.end(); road_iter++){
        m_attach_xml_file.original_road_network_list.append(*road_iter);
    }//lsh//original_road_network_list复制新的路网

    //更新发布路网
    sensor_msgs::NavSatFix current_vehicle_gps;
    {QMutexLocker locker(&ros_pub.mutex_is_rev_gps);
        if(ros_pub.is_rcv_gps){
            {QMutexLocker locker1(&ros_pub.mutex_vehicle_gps);
                current_vehicle_gps = ros_pub.vehicle_gps;}
        }
    }

    //生成了global_way_msgs、global_gps_way_msgs，找到match_id(车辆当前位置)，生成80m路，检测下一个任务点是否有特殊属性，计算到特殊属性点的距离
    //并将相应的tag置位,改变way_msgs的is_forward、task_area、wall_area、ditch_area……
    m_attach_xml_file.publishWay(current_vehicle_gps,m_attach_xml_file.m_path_map_list);//此处发布的是规划出的连接所有任务点的路，包含普通路点和任务点
    ROS_INFO("plan path done.");
}

void AStarDoubleXML::on_ReadTaskFile_clicked() {
//lsh//规划完成标志位PathPlanFinishFlag置false
//lsh//清空m_cMapMatch.TaskList、plan_task_list
//lsh//订阅并设置车辆当前位置
//lsh//读取任务文件对应信息存入TaskList
//lsh//设置任务起始点为强制起始点或车辆坐标与任务投影点距离最短的任务段的后一个任务点
//lsh//起始任务点当前车辆位置15m范围内则忽略，plan_task_list存起始点之后距离本车大于15m的任务点
//lsh//将当前车辆位置设为第一个任务点
    {//add pathplanfinishflag lock
        QMutexLocker locker(&m_attach_xml_file.mutex_PathPlanFinishFlag);
        m_attach_xml_file.PathPlanFinishFlag= false;
    }

    //Read task file
    if(!m_attach_xml_file.m_cMapMatch.TaskList.isEmpty()) {
        m_attach_xml_file.m_cMapMatch.TaskList.clear();
        //m_attach_xml_file.m_cMapMatch.TaskBeReaded=false;
    }//lsh//m_cMapMatch是CMapMatch类实例，包含一些计算工具和读入任务点
    //lsh//m_cMapMatch.TaskList保存任务点序列，每个任务点包括位置、任务点属性、相关道路信息
    if(!m_attach_xml_file.plan_task_list.isEmpty()){
        m_attach_xml_file.plan_task_list.clear();
    }//lsh//plan_task_list存放规划任务点
    sensor_msgs::NavSatFix cur_veh_gps;
    {QMutexLocker locker(&ros_pub.mutex_is_rev_gps);
        if(ros_pub.is_rcv_gps) {
            { QMutexLocker locker1(&ros_pub.mutex_vehicle_gps);
                /*m_attach_xml_file.m_cMapMatch.gps_updated = true;
                m_attach_xml_file.m_cMapMatch.currentlon =
                                                this->ros_pub.vehicle_gps.longitude;
                m_attach_xml_file.m_cMapMatch.currentlat =
                                                this->ros_pub.vehicle_gps.latitude;*/
                cur_veh_gps = this->ros_pub.vehicle_gps;
                //lsh//通过/gpsdata_sync订阅的车辆位置
            }
        } else {
            ROS_WARN("on_ReadTaskFile_clicked(): unable to recive the gps information.");
            return;
        }
    }
    //lsh//设置车辆当前位置

    int start_num = 0;
    int forced_start_task_id = -1;
    //m_attach_xml_file.m_cMapMatch.ReadTaskFile(this->base_dir);
    m_attach_xml_file.m_cMapMatch.ReadTaskFile(this->taskfile_dir);
    //lshadd//0715 读取任务文件高度为0的任务点对应信息存入TaskList
    m_attach_xml_file.CreatOutRoad();
    //lshadd//为搜索任务创建新道路
    QList<Task_Node>::iterator iter = m_attach_xml_file.m_cMapMatch.TaskList.begin();
    for(;iter != m_attach_xml_file.m_cMapMatch.TaskList.end(); iter++){
        if(iter->manu == 100){
            forced_start_task_id = iter->Task_num;
            //
            ROS_INFO("forced_start_task_id = %d",forced_start_task_id);
        }
    }//lsh//记录人为控制属性为100的任务点编号，也就是强制开始任务点
    int last_id,next_id;
    m_attach_xml_file.taskMapMatch(cur_veh_gps.latitude,cur_veh_gps.longitude,
                               &m_attach_xml_file.m_cMapMatch.TaskList,
                               &last_id,&next_id);
    //lsh//获取当前车辆坐标在每两个任务点坐标上的投影点Road_nodex（投影点在两任务点之间）
    //lsh//计算当前车辆坐标与任务投影点距离？？？为什么算投影点距离，会导致忽略任务点
    //lsh//找到投影点距离最近的任务点ID
    //lsh//相当于找到车辆当前所在的任务段，last_id返回车辆后方的任务点，next_id返回车辆前方的任务点
    //若车辆还未到第一个任务点，则last_id是第一个任务点
    if(forced_start_task_id != -1){
        start_num = forced_start_task_id;
    }else{
        start_num = next_id;//lsh//？？？会导致第一个任务点为到达时直接访问第二个任务点
    }
    std::cout << "on_ReadTaskFile_clicked(), start_num = " << start_num << std::endl;
    QList<Task_Node>::iterator task_iter = m_attach_xml_file.m_cMapMatch.TaskList.begin();
    for(; task_iter != m_attach_xml_file.m_cMapMatch.TaskList.end(); task_iter++){
        if(task_iter->Task_num == start_num){
            for(;task_iter != m_attach_xml_file.m_cMapMatch.TaskList.end(); task_iter++){
                if(task_iter->Task_num == start_num){   //当检测到车辆当前位置距离第一个任务点很近时，放弃第一个任务点
                    float current_x,current_y;
                    m_attach_xml_file.Position_Trans_From_ECEF_To_UTM(cur_veh_gps.latitude,
                            cur_veh_gps.longitude,0,0,&current_x,&current_y);
                    double car2startTask = m_attach_xml_file.distance(current_x,current_y,task_iter->x,task_iter->y);
                    //if(car2startTask < 15){
                    if(car2startTask < 0){
                        ROS_WARN("the car is near the start task point,abondon the start task point");
                        continue;
                    }
                }
                m_attach_xml_file.plan_task_list.append(*task_iter);
            }//lsh//起始任务点当前车辆位置15m范围内则忽略，plan_task_list存大于15m的任务点
            break;
        }
    }

    //将当前车辆位置放入任务链表头
    Task_Node temp_node;
    int last_task_type;
    int last_task_num;
    bool find_last_task_num;
    find_last_task_num = false;
    QList<Task_Node>::iterator task_iter3 = m_attach_xml_file.m_cMapMatch.TaskList.begin();
    for(; task_iter3 != m_attach_xml_file.m_cMapMatch.TaskList.end(); task_iter3++){
        if((task_iter3+1)->Task_num == start_num){
            last_task_num=task_iter3->Task_num;
            find_last_task_num = true;
            break;
        }
    }//lsh//找到开始任务的上一个任务点，主要是为了应对有强制起始任务点时需要忽略多处未做过的任务点时，找到强制任务点起点的前一个点，记录其属性和编号，用于任务段匹配
    if(!find_last_task_num){
        ROS_WARN("can not find the task before start_num, directly use last_id.");
        last_task_num=last_id;
    }//lsh//若找不到则直接使用上一个之前求出的last_id
    QList<Task_Node>::iterator task_iter2 = m_attach_xml_file.m_cMapMatch.TaskList.begin();
    for(; task_iter2 != m_attach_xml_file.m_cMapMatch.TaskList.end(); task_iter2++){
        if(task_iter2->Task_num == last_task_num){
            last_task_type = task_iter2->type;
            break;
        }
    }//lsh//记录上一个任务点的属性
    temp_node.lon = cur_veh_gps.longitude;
    temp_node.lat = cur_veh_gps.latitude;
    temp_node.Task_num = (m_attach_xml_file.restart_id++)*10000000+last_task_num;//lsh//车辆坐标形成的任务点的编号等于上一个点的任务点编号加上10000000的整数倍
    temp_node.on_road = true;
    temp_node.type = last_task_type;//lsh//车辆坐标形成的任务点的类型等于上一个任务点的类型
    m_attach_xml_file.Position_Trans_From_ECEF_To_UTM(temp_node.lat,temp_node.lon,0,0,&temp_node.x,&temp_node.y);
    m_attach_xml_file.plan_task_list.prepend(temp_node);
    //lsh//将当前车辆位置设为第一个任务点
    m_attach_xml_file.update_flag_for_task = 0;
    //lsh//在任务点链表上进行匹配和更新的标志位。0：重新进行匹配 其它：更新
    m_attach_xml_file.ShowPathWithPathList();  //更新显示

    //copy NodeList to original_road_network_list
    /*if(!m_attach_xml_file.original_road_network_list.isEmpty())
        m_attach_xml_file.original_road_network_list.clear();
    QList<MapSearchNode*>::iterator road_iter = m_attach_xml_file.m_astarsearch.NodeList.begin();
    for(; road_iter != m_attach_xml_file.m_astarsearch.NodeList.end(); road_iter++){
        m_attach_xml_file.original_road_network_list.append(*road_iter);
    }*/
}

void AStarDoubleXML::on_pushButton_clicked() {
    /// replanning
    QFile data(base_dir+"/taskfile_manual.txt");
    qDebug() << "output task list to " << base_dir + "/taskfile_manual.txt" << endl;
    if (data.open(QFile::WriteOnly | QIODevice::Truncate)) {
        QTextStream out(&data);
        for(int i = 0; i < m_attach_xml_file.plan_task_list.size(); ++i){
            std::cout << setprecision(10) << m_attach_xml_file.plan_task_list.at(i).lat << std::endl;
            out << qSetRealNumberPrecision(10) << i+1
            << " " << m_attach_xml_file.plan_task_list.at(i).lon
            << " " << m_attach_xml_file.plan_task_list.at(i).lat
            << " " << 0.0
            << " " << m_attach_xml_file.plan_task_list.at(i).type
            << "\n";
        }
    }
    std::cout << "output task list to file done." << std::endl;
    /*{
        QMutexLocker locker(&ros_pub.mutex_need_replanning);
        if(!ros_pub.need_replanning_) {
            ros_pub.need_replanning_ = true;
        }
    }*/
    /*float lat, lon;
    lat = ros_pub.vehicle_gps.latitude;
    lon = ros_pub.vehicle_gps.longitude;
    std::cout << "start replan!!!" << std::endl;

    m_attach_xml_file.RePlanning(lat,lon);

    m_attach_xml_file.m_bPathPlaned = true;
    m_attach_xml_file.m_flag = 0;*/
}

//自启动入口，目前程序主体都是从此处开始
void AStarDoubleXML::automaticStart() {
    ROS_INFO("AUTOMATIC START!");
    on_ReadFile_clicked();
    //lsh//读入路网,生成NodeList、m_RoadList、original_road_network_list、m_Pathid_vector
    //lsh//绘制道路
    ROS_INFO("automatic start: read road network done.");
    on_ReadTaskFile_clicked();
    //lsh//读入任务文件，并将任务点匹配至路网中,生成一个plan_task_list，其中第一个点为自车坐标
    //lsh//并将update_flag_for_task = 0;
    //lsh//生成TaskList
    //lsh//将当前车辆位置设为第一个任务点
    ROS_INFO("automatic start: read task file done.");
    on_Pathplan_clicked();
    //lsh//将任务点投影到路网中，建立连接关系并分别标注任务点是否在路网上
    //lsh//利用A*分别规划出相邻两任务投影点之间的路径，将规划结果存储在m_path_map_list，复制到v_path_map_list
    //lsh//复制新的路网到original_road_network_list
    ROS_INFO("automatic start: path plan done.");
}

void AStarDoubleXML::on_OutputTxt_clicked() {
    //m_attach_xml_file.OutputTxtFileWithPath(this->base_dir);
    m_attach_xml_file.task_id = 0;
    if(!m_attach_xml_file.plan_task_list.isEmpty()){
        m_attach_xml_file.plan_task_list.clear();
    }
}

void AStarDoubleXML::RealTime(){
    //lsh//根据检测到的新路口，和实时记录的GPS，实施构建路网，将其记录到road_network_list
    //lsh//局部规划道路小于8m并计次完成后，生成有当前车辆位置和从沿规划结果下一路段开始的剩余任务点的重规划任务列表m_Replan_Task_list
    //lsh//从走过的轨迹中规划出一条身后20m的道路
    //lsh//倒车5m后道路小于15m，need_replanning_real_time再次变为true，此时使first_planning_flag = true;backup_flag = false;first_path_flag = true;开始重规划
    //lsh//断点，在新的任务列表里进行重规划，重规划的结果存储在v_mapping_list
    //lsh//将规划结果分为两条道路，并分别为两条路补充一段额外的掉头路段，并将第一条道路放入m_path_map_list
    //lsh// 判断倒车到上一个岔道口位置的倒车路径是否大于300米，如果大于300米，则认为该路径不合适，或者规划失败时，继续使用之前的路径
    //lsh//如果倒车第一阶段规划道路长度小于一定距离或者车辆速度持续小于0.2m/s，则发送第二条道路
    BuildTopologicalMap();
    //lsh//根据检测到的新路口，和实时记录的GPS，实施构建路网，将其记录到road_network_list
    dataTransfer();
    //lsh//更新速度、档位和朝向

    //检查任务文件是否被更新，若更新，则重新规划
    m_attach_xml_file.m_cMapMatch.taskFile.refresh();
    //lsh//重新读取文件信息
    if(m_attach_xml_file.m_cMapMatch.taskFile.lastModified() != m_attach_xml_file.m_cMapMatch.lastModifiedTime){
        //lsh//LastModified()方法用一个长整型值来代表文件最后一次被修改的时间，其实返回的是文件修改时的时刻与00:00:00 GMT, January 1, 1970的差值(用毫秒计)
        qDebug() << "The task file was modified:" << m_attach_xml_file.m_cMapMatch.taskFile.lastModified() << endl;
        m_attach_xml_file.m_cMapMatch.lastModifiedTime = m_attach_xml_file.m_cMapMatch.taskFile.lastModified();
        automaticStart();
        return;
    }//lsh//若任务文件被修改则重新自启动全局规划

    sensor_msgs::NavSatFix current_vehicle_gps;
    float x = 0.0;
    float y = 0.0;
    {QMutexLocker locker(&ros_pub.mutex_is_rev_gps);
        if(ros_pub.is_rcv_gps){
            {
                QMutexLocker locker(&ros_pub.mutex_vehicle_gps);
                current_vehicle_gps = ros_pub.vehicle_gps;

                //显示当前车辆位置
                m_attach_xml_file.Position_Trans_From_ECEF_To_UTM(current_vehicle_gps.latitude,
                                                                  current_vehicle_gps.longitude,
                                                                  0,0,&x,&y);
                m_attach_xml_file.ShowPathWithPathList();  //更新显示
                m_attach_xml_file.ShowVelPos(x,y);//在地图内显示车辆位置
            }
        }
    }//lsh//显示车辆位置
    if(m_bRecvRndf){//lsh//是否读取路网的标志读取过路往后变为true
        if(!m_attach_xml_file.use_coherence_mapping)//lsh//决定是否使用连贯性进行路点匹配，默认为true
            m_attach_xml_file.findNodeType(current_vehicle_gps);//lsh//找到距离最小的投影点的前一个路点，记录其类型到cur_road_type
    }
    //发送当前任务点类型
    {QMutexLocker locker(&ros_pub.mutex_cur_road_type);
        ros_pub.cur_road_type = m_attach_xml_file.cur_road_type;}//lsh//记录当前路点向量中第一个路点的类型

    Backup(current_vehicle_gps);
    //lsh//生成有当前车辆位置和从沿规划结果下一路段开始的剩余任务点的重规划任务列表m_Replan_Task_list
    //lsh//从走过的轨迹中规划出一条身后20m的道路
    //lsh//倒车5m后道路小于15m，need_replanning_real_time再次变为true，此时使first_planning_flag = true;backup_flag = false;first_path_flag = true;开始重规划

    if(use_prior_map_replan){//lsh//是否采用先验地图进行重规划的控制位，默认为true

        //因为有些功能需要拦截重规划，但不能从源头拦截重规划，有些重规划还是有必要的
        //下面拦截重规划的目的是定线巡逻，只有在路线的两个端点处才需要重规划，认为在
        //巡逻过程中由于路线确定所以是不需要重规划的
        //为添加这个功能，重规划位置的记录功能被关闭了
        if(open_fixed_line_patrol){//lsh//是否开启定线巡逻，此处为false
            //m_Replan_Task_list链表里保存了重规划触发时还未通过的任务点，m_Replan_Task_list的size初始值为0
            //当m_Replan_Task_list中的任务点数为1且车辆当前位置距离原始任务点链表中的某个端点比较近时，这是认为车
            //辆已经到端点了
            if(first_planning_flag && !m_attach_xml_file.m_cMapMatch.TaskList.isEmpty()){
                first_planning_flag = false;        //清除重规划标志
                first_path_flag = false;            //清除在倒车第一段路的标志
                double temp_dis_to_last_task = m_attach_xml_file.distance(x,y,
                        m_attach_xml_file.m_cMapMatch.TaskList.last().x,
                        m_attach_xml_file.m_cMapMatch.TaskList.last().y);
                std::cout << "trigger replanning in fixed line patrol: " << std::endl
                << "  distance to end task point is     " <<  temp_dis_to_last_task << std::endl
                << "  the size of m_Replan_Task_list is " <<  m_attach_xml_file.m_Replan_Task_list.size() << std::endl;
                if(m_attach_xml_file.m_Replan_Task_list.size() == 1 ){  /*默认最后一个任务点与车辆在同一条路上，所以此时只有起点*/
                    std::cout << "end point, vehicle return" << std::endl;
                    m_attach_xml_file.numOfPatrols++;     //累加巡逻次数
                    if(m_attach_xml_file.numOfPatrols < m_attach_xml_file.set_patrol_times){
                        first_planning_flag = true;    //只有在端点处且巡逻次数没够的时候可以重规划
                        first_path_flag = true;        //在重规划的倒车阶段不接收重规划标志
                    }
                    //反转任务链表，因为任务链表的匹配是建立在TaskList上的，不反转的话在完成第一次巡逻匹配就停止不动了
                    QList<Task_Node> temp_task_list;
                    QList<Task_Node>::iterator temp_iter = m_attach_xml_file.m_cMapMatch.TaskList.begin();
                    for(; temp_iter != m_attach_xml_file.m_cMapMatch.TaskList.end(); temp_iter++){
                        temp_task_list.append(*temp_iter);
                    }
                    m_attach_xml_file.m_cMapMatch.TaskList.clear();
                    m_attach_xml_file.m_cMapMatch.TaskList.reserve(temp_task_list.size());
                    std::reverse_copy(temp_task_list.begin(), temp_task_list.end(),
                            std::back_inserter(m_attach_xml_file.m_cMapMatch.TaskList));
                    //更改任务链表属性
                    m_attach_xml_file.m_cMapMatch.TaskList.first().type = 0;
                    m_attach_xml_file.m_cMapMatch.TaskList.last().type = 1;
                    //从TaskList中提取余下任务点
                    QList<Task_Node>::iterator task_iter = m_attach_xml_file.m_cMapMatch.TaskList.begin();
                    for(; task_iter != m_attach_xml_file.m_cMapMatch.TaskList.end(); task_iter++){
                        if(task_iter == m_attach_xml_file.m_cMapMatch.TaskList.begin()) continue;
                        m_attach_xml_file.m_Replan_Task_list.append(*task_iter);
                    }
                    m_attach_xml_file.update_flag_for_task = 0;                 //重新匹配任务点
                }
            }
        }

        //重规划第一阶段
        if(first_planning_flag){
            first_planning_flag = false;
            ros_pub.semantic_replanning_flag = 0;///lshadd//语义识别重规划标志位清零
            ROS_INFO("replanning in prior map.");
            //add pathplanfinishflag lock
            {QMutexLocker locker(&m_attach_xml_file.mutex_PathPlanFinishFlag);
                m_attach_xml_file.PathPlanFinishFlag= false;}
            ROS_INFO("planning the first path ......");
            if(!m_attach_xml_file.m_astarsearch.NodeList.isEmpty()){
                m_attach_xml_file.m_astarsearch.NodeList.clear();
            }
            QList<MapSearchNode*>::iterator copy_iter = m_attach_xml_file.original_road_network_list.begin();
            for(; copy_iter != m_attach_xml_file.original_road_network_list.end(); copy_iter++){
                m_attach_xml_file.m_astarsearch.NodeList.append(*copy_iter);
            }//lsh/把original_road_network_list复制给NodeList

            //复制用于匹配的规划路径，防止规划失败，这里对需要temp_list做个说明：如果上一次重规划失败，那么v_mapping_list中指向的变量将是
            //上一次temp_mapmatch_list保存的内容，如果没有temp_list直接清空temp_list然后再讲v_mapping_list中的内容复制到team_mapmatch_list
            //中的话，就会出现错误，由于清除操作，在复制之前,v_mapping_list中全是野指针。
            QList<MapSearchNode> temp_list;
            QList<MapSearchNode*>::iterator temp_iter = m_attach_xml_file.v_mapping_list.begin();
            for(; temp_iter != m_attach_xml_file.v_mapping_list.end(); temp_iter++){
                temp_list.append(*(*temp_iter));
            }
            if(!temp_mapmatch_list.isEmpty()){
                temp_mapmatch_list.clear();
            }
            QList<MapSearchNode>::iterator path_iter = temp_list.begin();
            for(;path_iter != temp_list.end(); path_iter++){
                temp_mapmatch_list.append(*path_iter);
            }//lsh//此时temp_mapmatch_list存储上一次规划结果v_mapping_list

            int value = m_attach_xml_file.RePlanning(current_vehicle_gps.latitude,current_vehicle_gps.longitude);
            //lsh//断点，在新的任务列表里进行重规划，重规划的结果存储在v_mapping_list
            //lsh//将规划结果分为两条道路，并分别为两条路补充一段额外的掉头路段，并将第一条道路放入m_path_map_list
            ROS_INFO("the first plan done.");


            float temp_dis = 0.0;
            if(!m_attach_xml_file.m_path_map_list.isEmpty()){
                MapSearchNode* start_point = m_attach_xml_file.m_path_map_list.first();
                MapSearchNode* end_point = m_attach_xml_file.m_path_map_list.last();
                temp_dis = m_attach_xml_file.distance(start_point->x, start_point->y,end_point->x,end_point->y);
            }
            if(temp_dis > 300 || value == 0){
                if(temp_dis > 300){
                    ROS_WARN("replanning: the first planned reversing path is more than 300 meters! replan failed.");
                    std::cout << "size of the first path:" << m_attach_xml_file.m_path_map_list.size() << std::endl
                              << "temp_dis: " << temp_dis << std::endl
                              << "first path:  " << std::endl;
                }
                QList<MapSearchNode*>::iterator path_iter = m_attach_xml_file.m_path_map_list.begin();
                for(; path_iter != m_attach_xml_file.m_path_map_list.end(); path_iter++){
                    std::cout << (*path_iter)->node_id << std::endl;
                }

                if(!m_attach_xml_file.v_mapping_list.isEmpty()){
                    m_attach_xml_file.v_mapping_list.clear();
                }
                if(!m_attach_xml_file.m_path_map_list.isEmpty()){
                    m_attach_xml_file.m_path_map_list.clear();
                }
                QList<MapSearchNode>::iterator copy_iter1 = temp_path_list.begin();
                for(; copy_iter1 != temp_path_list.end() ; copy_iter1++){
                    m_attach_xml_file.m_path_map_list.append(&(*copy_iter1));
                }
                QList<MapSearchNode>::iterator copy_iter2 = temp_mapmatch_list.begin();
                for(; copy_iter2 != temp_mapmatch_list.end(); copy_iter2++){
                    m_attach_xml_file.v_mapping_list.append(&(*copy_iter2));
                }
                first_path_flag = false;
                m_attach_xml_file.m_flag = 0;
                m_attach_xml_file.is_forward = true;
                {QMutexLocker locker(&m_attach_xml_file.mutex_PathPlanFinishFlag);
                    m_attach_xml_file.PathPlanFinishFlag= true;}
                m_attach_xml_file.ShowPathWithPathList();  //更新显示
                m_attach_xml_file.publishWay(current_vehicle_gps,m_attach_xml_file.m_path_map_list);    //更新发布路网
                return;
            }//lsh// 判断倒车到上一个岔道口位置的倒车路径是否大于300米，如果大于300米，则认为该路径不合适，或者规划失败时，继续使用之前的路径

            m_attach_xml_file.is_forward = false;
            {QMutexLocker locker(&m_attach_xml_file.mutex_PathPlanFinishFlag);
                m_attach_xml_file.PathPlanFinishFlag= true;}
            first_planning_finished = true;     //第一次规划完成标志
            m_attach_xml_file.m_bPathPlaned = true;
            m_attach_xml_file.m_flag = 0;
        }
        //重规划第二阶段
        if(first_planning_finished) {
            MapSearchNode *fallback_node = NULL;
            fallback_node = m_attach_xml_file.first_path.last();
            float current_x, current_y;
            m_attach_xml_file.Position_Trans_From_ECEF_To_UTM(current_vehicle_gps.latitude,
                                                              current_vehicle_gps.longitude,
                                                              0, 0,
                                                              &current_x, &current_y);
            float temp_distance;
            if (fallback_node != NULL) {
                temp_distance = m_attach_xml_file.distance(fallback_node->x, fallback_node->y, current_x, current_y);
                {
                    QMutexLocker locker(&ros_pub.mutex_vehicle_vel);
                    if(fabs(ros_pub.vehicle_vel) < 0.2){
                        ROS_WARN("vel is %f",ros_pub.vehicle_vel);
                        ROS_INFO("vel < 0.2,wait_times_for_second_path %d",wait_times_for_second_path);
                        wait_times_for_second_path++;
                    }else{
                        wait_times_for_second_path = 0;
                    }
                }//lsh//如果速度小于0.2开始计次
                if (temp_distance < second_plan_trig_dis || wait_times_for_second_path > 600) {
                    {QMutexLocker locker(&m_attach_xml_file.mutex_PathPlanFinishFlag);
                        m_attach_xml_file.PathPlanFinishFlag = false;}
                    first_planning_finished = false;
                    first_path_flag = false;
                    ROS_INFO("the left length of the first path is %f.",temp_distance);
                    ROS_INFO("clear wait_times_for_second_path.");
                    wait_times_for_second_path = 0;
                    ROS_INFO("planning the second path ......");
                    if (!m_attach_xml_file.m_path_map_list.isEmpty()) {
                        m_attach_xml_file.m_path_map_list.clear();
                    }
                    QList<MapSearchNode *>::iterator copy_iter = m_attach_xml_file.second_path.begin();
                    for (; copy_iter != m_attach_xml_file.second_path.end(); copy_iter++) {
                        m_attach_xml_file.m_path_map_list.append(*copy_iter);
                    }
                    ROS_INFO("the second plan done.");
                    m_attach_xml_file.is_forward = true;
                    {QMutexLocker locker(&m_attach_xml_file.mutex_PathPlanFinishFlag);
                        m_attach_xml_file.PathPlanFinishFlag = true;}
                    m_attach_xml_file.m_bPathPlaned = true;
                    //m_attach_xml_file.m_flag = 0;
                    m_attach_xml_file.ShowPathWithPathList();  //更新显示
                }//lsh//如果倒车第一阶段规划道路长度小于一定距离或者车辆速度持续小于0.2m/s，则发送第二条道路
            } else {
                ROS_FATAL("In RealTime(): can't get fallback node\n");
            }
        }
    }else{
        if(first_planning_flag){
            ROS_INFO("replanning in real-time map.");
            {QMutexLocker locker(&m_attach_xml_file.mutex_PathPlanFinishFlag);
                m_attach_xml_file.PathPlanFinishFlag= false;}
            first_planning_flag = false;
            ROS_INFO("planning the first path ......");
            PlanTheFirstPath();
            ROS_INFO("the first plan done.");

            //判断倒车到上一个岔道口位置的倒车路径是否大于200米，如果大于200米，则认为该路径不合适，继续使用之前的路径
            MapSearchNode* start_point = m_attach_xml_file.m_path_map_list.first();
            MapSearchNode* end_point = m_attach_xml_file.m_path_map_list.last();
            float temp_dis = m_attach_xml_file.distance(start_point->x, start_point->y,end_point->x,end_point->y);
            if(temp_dis > 300){
                ROS_WARN("replanning: the first planned reversing path is more than 300 meters! replan failed.");
                if(!m_attach_xml_file.m_path_map_list.isEmpty()){
                    m_attach_xml_file.m_path_map_list.clear();
                }
                if(!m_attach_xml_file.v_mapping_list.isEmpty()){
                    m_attach_xml_file.v_mapping_list.clear();
                }
                QList<MapSearchNode>::iterator copy_iter = temp_path_list.begin();
                for(; copy_iter != temp_path_list.end() ; copy_iter++){
                    m_attach_xml_file.m_path_map_list.append(&(*copy_iter));
                    m_attach_xml_file.v_mapping_list.append(&(*copy_iter));
                }
                first_path_flag = false;
                m_attach_xml_file.is_forward = true;
                {QMutexLocker locker(&m_attach_xml_file.mutex_PathPlanFinishFlag);
                    m_attach_xml_file.PathPlanFinishFlag= true;}
                m_attach_xml_file.ShowPathWithPathList();  //更新显示
                m_attach_xml_file.publishWay(current_vehicle_gps,m_attach_xml_file.m_path_map_list);    //更新发布路网
                return;
            }

            {QMutexLocker locker(&m_attach_xml_file.mutex_PathPlanFinishFlag);
                m_attach_xml_file.PathPlanFinishFlag= true;}
        }

        if(first_planning_finished) {
            MapSearchNode *fallback_node = NULL;
            QList<MapSearchNode *>::iterator road_iter = m_attach_xml_file.road_network_list.begin();
            for (; road_iter != m_attach_xml_file.road_network_list.end(); road_iter++) {
                if ((*road_iter)->node_id == m_attach_xml_file.fallback_node_id) {
                    fallback_node = *road_iter;
                    break;
                }
            }
            float current_x,current_y;
            m_attach_xml_file.Position_Trans_From_ECEF_To_UTM(current_vehicle_gps.latitude,
                                                              current_vehicle_gps.longitude,
                                                              0,0,
                                                              &current_x,&current_y);
            float temp_distance;
            if (fallback_node != NULL) {
                temp_distance = m_attach_xml_file.distance(fallback_node->x, fallback_node->y, current_x, current_y);
                /*if(temp_distance < 10){
                    std::cout<<"distance to the fallback node:"<<temp_distance<<std::endl;
                }*/
                if (temp_distance < second_plan_trig_dis) {
                    {QMutexLocker locker(&m_attach_xml_file.mutex_PathPlanFinishFlag);
                        m_attach_xml_file.PathPlanFinishFlag = false;
                    }
                    first_planning_finished = false;
                    ROS_INFO("planning the second path ......");
                    PlanTheSecondPath();
                    ROS_INFO("the second plan done.");
                    {QMutexLocker locker(&m_attach_xml_file.mutex_PathPlanFinishFlag);
                        m_attach_xml_file.PathPlanFinishFlag = true;
                    }
                }
            }else{
                ROS_FATAL("In RealTime(): can't get fallback node");
            }
        }
    }
    m_attach_xml_file.publishWay(current_vehicle_gps,m_attach_xml_file.m_path_map_list);    //更新发布路网
}

void AStarDoubleXML::BuildTopologicalMap() {
    //lsh//根据检测到的新路口，和实时记录的GPS，实施构建路网，将其记录到road_network_list
    if(record_flag) {//lsh//实施构建拓扑地图的开关，当其关闭时，不构建拓扑地图
        QMutexLocker locker(&ros_pub.mutex_is_rcv_intersec);
        if(ros_pub.is_rcv_intersec){//lsh//如果检测到其他路口
            ros_pub.is_rcv_intersec = false;
            QMutexLocker locker1(&ros_pub.mutex_intersec_data);
            m_attach_xml_file.AddIntersectionNode(ros_pub.intersec_data);
            //lsh//（当检查到新的路口或者行驶到无路网的地区时，）通过其他模块检测路口
            //lsh//找到所有检测到的路口中距离最近的路口，如果距离小于一定阈值，则将其和上一个新添加的路口建立连接关系
            //lsh//若距离大于阈值，则记录当前位置作为第一个新增路口，当距离最近的检测到的路口的引导点设为当前路口的子节点，将其和上一个新添加的路口建立连接关系
            /*{
                QMutexLocker locker(&ros_pub.mutex_vehicle_gps);
                m_attach_xml_file.BuildNewRoadNetwork(ros_pub.vehicle_gps);
            }

            PathPlanwithTaskPoint(end_point);*/
        } else{//lsh//如果没检测到路口
            {QMutexLocker locker(&ros_pub.mutex_is_rev_gps);
                if(ros_pub.is_rcv_gps){
                    QMutexLocker locker2(&ros_pub.mutex_vehicle_gps);
                    m_attach_xml_file.AddNormalNode(ros_pub.vehicle_gps);
                }
            }
        }//lsh//记录车辆当前GPS，并以一定的间距将其加入到创建的路网列表中，并建立连接关系
        //lsh//如果位于重规划阶段，不需要大于一定距离，直接将GPS位置记入路网
    }else{
        m_attach_xml_file.topology_parent_id = m_attach_xml_file.last_intersec_id;

        float lat,lon,x,y;
        {QMutexLocker locker(&ros_pub.mutex_vehicle_gps);
            lat = ros_pub.vehicle_gps.latitude;
            lon = ros_pub.vehicle_gps.longitude;
        }
        m_attach_xml_file.Position_Trans_From_ECEF_To_UTM(lat, lon,0,0,&x,&y);

        MapSearchNode* last_intersec_node = NULL;
        QList<MapSearchNode*>::iterator iter = m_attach_xml_file.road_network_list.begin();
        for(; iter != m_attach_xml_file.road_network_list.end(); iter++){
            if((*iter)->node_id == m_attach_xml_file.last_intersec_id){
                last_intersec_node = (*iter);
                break;
            }
        }
        if(last_intersec_node == NULL)
            ROS_FATAL("In BuildTopologicalMap(): fail to find last_intersec_node.");

        float distance_to_last_intersec = m_attach_xml_file.distance(x,y,last_intersec_node->x,last_intersec_node->y);

        if(distance_to_last_intersec < 5){      //5m
            record_flag = true;
        }
    }
}

void AStarDoubleXML::Backup(sensor_msgs::NavSatFix current_vehicle_gps) {
//lsh//进入倒车确认
//lsh//生成有当前车辆位置和从沿规划结果下一路段开始的剩余任务点的重规划任务列表m_Replan_Task_list
//lsh//判断是否在同一个位置反复触发倒车，如果在同一个地方触发倒车累加超过2次，则直接进行重规划
//lsh//从走过的轨迹中规划出一条身后20m的道路
//lsh//倒车5m后重新发送之前的规划结果，并开始计时3秒，期间车辆停车
//lsh//倒车5m后道路小于15m，need_replanning_real_time再次变为true，此时使first_planning_flag = true;backup_flag = false;first_path_flag = true;开始重规划
//lsh//倒车5m后道路大于15m，不进行重规划，或者即使小于15m，但在某些特殊任务关闭重规划
    {
        QMutexLocker locker(&ros_pub.mutex_need_replanning);
        if (ros_pub.need_replanning_real_time && !backup_flag && !first_path_flag) {
            //lsh//计次够之后need_replanning_变为true，然后把need_replanning_real_time设为true，然后把need_replanning_变为false
            //lsh//backup_flag为倒车的标志，开始为false
            //lsh//first_path_flag为车辆是否处于重规划第一阶段的标志位，开始为false。当处于倒车阶段的时候，不接收重规划信息
            //lsh//此处表示车辆检查局部规划持续小于8m，准备倒车确认
            if(ros_pub.need_replanning_real_time){
                ROS_INFO("tri back up, clean re-planning msg");
                ros_pub.need_replanning_real_time = false;
            }
            if(m_attach_xml_file.road_network_list.size() <= 2){
                //lsh//road_network_list为走过的路和订阅到的路口形成的路网
                ROS_WARN("abondon back up, the size of road_network_list is less than 2!");
                return;
            }
            {
                QMutexLocker locker(&m_attach_xml_file.mutex_PathPlanFinishFlag);
                m_attach_xml_file.PathPlanFinishFlag = false;//lsh//规划完成标志
            }
            backup_flag = true;//lsh//倒车确认阶段标志
            first_plan_vehicle_gps = current_vehicle_gps;
            m_attach_xml_file.replan_flag = true;//lsh//重规划标志位，用于无条件向road_network_list插入当前位置
            m_attach_xml_file.AddNormalNode(first_plan_vehicle_gps);//lsh//将当前位置加入自己构造的路网
            first_plan_start_id = m_attach_xml_file.topology_parent_id;//lsh//刚加入路点的id
            m_attach_xml_file.first_plan_tri_intersec_id = m_attach_xml_file.last_intersec_id;//lsh//first_plan_tri_intersec_id为重规划第一次规划触发时的上一个岔道口
            m_attach_xml_file.ReCreateTaskList(current_vehicle_gps.latitude,current_vehicle_gps.longitude);
            //lsh//生成有当前车辆位置和从沿规划结果下一路段开始的剩余任务点的重规划任务列表m_Replan_Task_list

            ///lshadd//语义识别重规划
            if (ros_pub.semantic_replanning_flag == 1)
            {
                first_planning_flag = true;
                backup_flag = false;//lsh//是否处于倒车确认标志位
                first_path_flag = true;//lsh//是否处于重规划第一阶段标志位
                return;
            }

            //判断是否在同一个位置反复触发倒车，如果在同一个地方触发倒车累加超过2次，则直接进行重规划
            if(backup_start_gps.latitude != 0 && backup_start_gps.longitude != 0){
                float backup_start_x,backup_start_y,current_x,current_y;
                m_attach_xml_file.Position_Trans_From_ECEF_To_UTM(backup_start_gps.latitude,
                                                                  backup_start_gps.longitude,
                                                                  0, 0,
                                                                  &backup_start_x, &backup_start_y);
                m_attach_xml_file.Position_Trans_From_ECEF_To_UTM(current_vehicle_gps.latitude,
                                                                  current_vehicle_gps.longitude,
                                                                  0, 0,
                                                                  &current_x, &current_y);
                float temp_dis = m_attach_xml_file.distance(current_x,current_y,backup_start_x,backup_start_y);
                if(temp_dis < 10 ){
                    //lsh//当距离上一次触发重规划位置小于10米时，认为是在同一个位置触发的重规划
                    ++repeat_tri_times;
                    ROS_WARN("it's less than 10 meters from the last trigger re-planning position, the rep-trig-times is %d",repeat_tri_times);
                } else{
                    repeat_tri_times = 0;
                    ROS_INFO("it's more than 10 meters from the last trigger re-planning position, clean rep-trig-times. ");
                }
                if(repeat_tri_times >= 1){//lsh//如果第二次到这个重规划点
                    ROS_WARN("repeat the trigger in the same  place, re-planning !");
                    repeat_tri_times = 0;
                    first_planning_flag = true;//lsh//进入重规划第一阶段标志位
                    backup_flag = false;//lsh//是否处于倒车确认标志位
                    first_path_flag = true;//lsh//是否处于重规划第一阶段标志位
                    return;
                }
            }
            backup_start_gps = current_vehicle_gps;     //更新倒车触发位置点

            QList<MapSearchNode> temp_list;
            QList<MapSearchNode*>::iterator temp_iter = m_attach_xml_file.m_path_map_list.begin();
            for(;temp_iter != m_attach_xml_file.m_path_map_list.end(); temp_iter++){
                temp_list.append(*(*temp_iter));
            }
            //如果上一次重规划失败，那么m_path_map_list和v_mapping_list中指针指向的上一次重规划前的temp_path_list
            //所以这里在清除temp_path_list之前需要先用一个暂时链表把m_path_map_list保存起来
            if(!temp_path_list.isEmpty()){
                temp_path_list.clear();
            }
            QList<MapSearchNode >::iterator copy_iter = temp_list.begin();
            for (; copy_iter != temp_list.end(); copy_iter++) {
                temp_path_list.append(*copy_iter);
            }//lsh//复制上次规划的路网m_path_map_list到temp_path_list
            if (!m_attach_xml_file.m_path_map_list.isEmpty()) {
                m_attach_xml_file.m_path_map_list.clear();
            }//lsh//清空规划结果m_path_map_list

            //找到倒车引导点（5米）
            float backup_start_x, backup_start_y;
            m_attach_xml_file.Position_Trans_From_ECEF_To_UTM(backup_start_gps.latitude,
                                                              backup_start_gps.longitude,
                                                              0, 0,
                                                              &backup_start_x, &backup_start_y);
            MapSearchNode* backup_ptr = NULL;
            /*if(!m_attach_xml_file.road_network_list.isEmpty()){
                backup_ptr = m_attach_xml_file.road_network_list.last();
            }else{
                ROS_WARN("the road_network_list is empty, can't back up.");
                first_path_flag = false;
                backup_flag = false;
                return;
            }
            float temp_dis;
            do{
                temp_dis = m_attach_xml_file.distance(backup_ptr->x, backup_ptr->y,
                                                      backup_start_x, backup_start_y);
                if (temp_dis <= 10 || m_attach_xml_file.m_path_map_list.isEmpty()) {
                    std::cout << "backup node is: " << backup_ptr->node_id << std::endl
                              << "dis to this backup node: " << temp_dis << std::endl;
                    m_attach_xml_file.m_path_map_list.append(backup_ptr);
                }
                if(!backup_ptr->searchParent.isEmpty()) {
                    backup_ptr = backup_ptr->searchParent.last();
                } else {
                    break;
                }
            }while(temp_dis < 5);*/
            int i = m_attach_xml_file.road_network_list.size() - 1;
            for (; i >= 0; i--) {
                double temp_dis = m_attach_xml_file.distance(m_attach_xml_file.road_network_list.at(i)->x,
                                                            m_attach_xml_file.road_network_list.at(i)->y,
                                                            backup_start_x, backup_start_y);
                //lsh//倒着索引road_network_list的每一个点，记录其离倒车点的距离
                double last2start = std::numeric_limits<double>::infinity();
                if(!m_attach_xml_file.m_path_map_list.isEmpty()){
                    MapSearchNode* last_ptr = m_attach_xml_file.m_path_map_list.last();
                    last2start = m_attach_xml_file.distance(last_ptr->x,last_ptr->y,backup_start_x,backup_start_y);
                    if(temp_dis < last2start){
                        ROS_WARN("backup: filter %d node.",m_attach_xml_file.road_network_list.at(i)->node_id);
                    }
                }//lsh//计算上次记入m_path_map_list里的点和倒车点的距离，若大于road_network_list当前索引的点距离倒车点的距离则报错
                if ((temp_dis <= 20.0 && temp_dis >= last2start) || m_attach_xml_file.m_path_map_list.isEmpty()) {
                    backup_ptr = m_attach_xml_file.road_network_list.at(i);
                    if (backup_ptr == NULL) {
                        ROS_WARN("Backup(): backup_ptr = NULL");
                        break;
                    } else {
                        std::cout << "   backup node is: " << backup_ptr->node_id << std::endl
                                  << "   dis to this backup node: " << temp_dis << std::endl;
                        m_attach_xml_file.m_path_map_list.append(backup_ptr);
                    }//lsh//如果temp_dis小于20m且倒车道路正常，或者m_path_map_list为空，则把当前的点数据存入m_path_map_list
                }
                if(temp_dis > 20.0 || i == 0){
                    break;
                }
            }//lsh//规划出一条身后20m的道路
            m_attach_xml_file.is_forward = false;//lsh//控制车辆前进或者后退
            {
                QMutexLocker locker(&m_attach_xml_file.mutex_PathPlanFinishFlag);
                m_attach_xml_file.PathPlanFinishFlag = true;
            }
            m_attach_xml_file.ShowPathWithPathList();  //更新显示
        }
    }

    if(first_path_flag){    //lsh//车辆是否处于重规划第一阶段的标志位。
        QMutexLocker locker(&ros_pub.mutex_need_replanning);
        if (ros_pub.need_replanning_real_time) {
            ROS_INFO("in first path, clean the re-planning msg!");
            ros_pub.need_replanning_real_time = false;
        }//lsh//need_replanning_real_time变为false，此时车辆不进入上述生成后方20m道路的逻辑，即不进入倒车确认
        {
            QMutexLocker locker1(&ros_pub.mutex_first_path_flag);
            if(!ros_pub.first_path_flag){
                ros_pub.first_path_flag = true;//lsh//当前车辆是否处于重规划的第一条路上
            }
        }
    }else{
        QMutexLocker locker1(&ros_pub.mutex_first_path_flag);
        if(ros_pub.first_path_flag){
            ros_pub.first_path_flag = false;
        }
    }

    if(backup_flag){//lsh//如果处于倒车确认阶段
        float current_x,current_y;
        float backup_start_x,backup_start_y;
        m_attach_xml_file.Position_Trans_From_ECEF_To_UTM(current_vehicle_gps.latitude,
                                                          current_vehicle_gps.longitude,
                                                          0,0,
                                                          &current_x,&current_y);
        m_attach_xml_file.Position_Trans_From_ECEF_To_UTM(backup_start_gps.latitude,
                                                          backup_start_gps.longitude,
                                                          0,0,
                                                          &backup_start_x,&backup_start_y);
        float temp_dis = m_attach_xml_file.distance(current_x,current_y, backup_start_x,backup_start_y);
        if(temp_dis < 5){       //在未到达5米之前清除重规划触发信号
            {
                QMutexLocker locker(&ros_pub.mutex_need_replanning);
                if (ros_pub.need_replanning_real_time) {
                    ros_pub.need_replanning_real_time = false;
                }
            }
        }//lsh//在没倒车5m时不接收重规划信息
        if(temp_dis > 5 || wait_times > 1){
            //清空路线，让车停在那
            if(wait_times < 1){
                {QMutexLocker locker(&ros_pub.mutex_backup_confirm);
                ros_pub.backup_confirm_flag = true;}//lsh//开始检查局部规划路径是否大于15m
                std::cout << "dis to back up start position: " << temp_dis << std::endl
                          << "wait ......" << std::endl;
                wait_times++;
                wait_start_time = time(NULL);
                ros_pub.wait_judge_repanning.data = 1;
                //确认前先发送前进的路
                {QMutexLocker locker(&m_attach_xml_file.mutex_PathPlanFinishFlag);
                    m_attach_xml_file.PathPlanFinishFlag = false;}
                if(!m_attach_xml_file.m_path_map_list.isEmpty()) {
                    m_attach_xml_file.m_path_map_list.clear();
                }
                QList<MapSearchNode>::iterator copy_iter = temp_path_list.begin();
                for(; copy_iter != temp_path_list.end() ; copy_iter++){
                    m_attach_xml_file.m_path_map_list.append(&(*copy_iter));
                }//lsh//将以前规划的道路重新赋值给m_path_map_list
                m_attach_xml_file.is_forward = true;
                {QMutexLocker locker(&m_attach_xml_file.mutex_PathPlanFinishFlag);
                    m_attach_xml_file.PathPlanFinishFlag = true;}
                m_attach_xml_file.ShowPathWithPathList();  //更新显示
            }
            time_t record_time = time(NULL);
            if(wait_times < 30 && (double)(record_time - wait_start_time) <= 3){ //等待4S
                ROS_INFO("wait_times:%d",wait_times);
                ROS_INFO("wait time:%d",(int)(record_time - wait_start_time));
                wait_times++;
                return;
            }else{
                {QMutexLocker locker(&ros_pub.mutex_backup_confirm);
                    ros_pub.backup_confirm_flag = false;}
                ros_pub.wait_judge_repanning.data = 0;
                ROS_INFO("wait done.");
                wait_times = 0;
            }//lsh//倒车5m后重新发送之前的规划结果，并开始计时4秒，在4秒后恢复ackup_confirm_flag，保证下次重规划可用

            {QMutexLocker locker(&ros_pub.mutex_need_replanning);       //确认是否还有重规划的信息
                /*if (ros_pub.need_replanning_real_time && m_attach_xml_file.m_pnexttask.type != 3
                    && m_attach_xml_file.m_pcurtask.type != 3 && m_attach_xml_file.m_pcurtask.type != 4){*///lsh//这是什么意思？如果是在特殊区域关闭重规划，为什么是在倒车后才关闭？
                if (ros_pub.need_replanning_real_time && m_attach_xml_file.m_pnexttask.type != 1
                    && m_attach_xml_file.m_pcurtask.type != 1 /*&& m_attach_xml_file.m_pcurtask.type != 7*/){
                    ros_pub.need_replanning_real_time = false;
                    first_planning_flag = true;
                    backup_flag = false;
                    first_path_flag = true;//lsh//倒车5m后道路小于15m，need_replanning_real_time再次变为true，开始重规划
                }else{
                    if(ros_pub.need_replanning_real_time){
                        /*ROS_WARN("abandon replanning,because the car is at park or patrol area!");*///lsh//此时不是停车区？？？
                        ROS_WARN("abandon replanning,because the car is at park or search area!");
                    }
                    ros_pub.need_replanning_real_time = false;
                    backup_flag = false;
                    first_path_flag = false;
                }//lsh//倒车5m后道路大于15m，不进行重规划，或者即使小于15m，但在某些特殊任务关闭重规划
            }
        }
    }
}

void AStarDoubleXML::PlanTheFirstPath(){
    int value1 = m_attach_xml_file.findFbIntersec(first_plan_vehicle_gps);       //找到回退岔道口和普通路点
    if(!m_attach_xml_file.m_astarsearch.NodeList.isEmpty()){
        m_attach_xml_file.m_astarsearch.NodeList.clear();
    }
    QList<MapSearchNode*>::iterator copy_iter = m_attach_xml_file.road_network_list.begin();
    for(; copy_iter != m_attach_xml_file.road_network_list.end(); copy_iter++){
        m_attach_xml_file.m_astarsearch.NodeList.append(*copy_iter);
    }
    if(!m_attach_xml_file.m_path_map_list.isEmpty()) {
        m_attach_xml_file.m_path_map_list.clear();
    }
    std::cout<<"first pathplan start&stop id: "<< first_plan_start_id
             << "," <<m_attach_xml_file.fallback_node_id << std::endl;
    std::cout<<"first plan fb_intersec_id :"<<m_attach_xml_file.fb_intersec_id<< std::endl;
    int value2 = m_attach_xml_file.PathPlan(first_plan_start_id,
                                   m_attach_xml_file.fallback_node_id);

    if(value1 == 0 || value2 == 0){
        ROS_FATAL("Replanning: the first plan failed! value1= %d , value2=%d",value1,value2);
        if(!m_attach_xml_file.m_path_map_list.isEmpty()){
            m_attach_xml_file.m_path_map_list.clear();
        }
        if(!m_attach_xml_file.v_mapping_list.isEmpty()){
            m_attach_xml_file.v_mapping_list.clear();
        }
        QList<MapSearchNode>::iterator copy_iter = temp_path_list.begin();
        for(; copy_iter != temp_path_list.end() ; copy_iter++){
            m_attach_xml_file.m_path_map_list.append(&(*copy_iter));
            m_attach_xml_file.v_mapping_list.append(&(*copy_iter));
        }
        first_path_flag = false;
        m_attach_xml_file.is_forward = true;
        m_attach_xml_file.ShowPathWithPathList();  //更新显示
        return;
    }
    m_attach_xml_file.is_forward = false;
    m_attach_xml_file.first_plan_intersec_id = m_attach_xml_file.fb_intersec_id;
    first_planning_finished = true;     //第一次规划完成标志
    m_attach_xml_file.m_bPathPlaned = true;
    //m_attach_xml_file.m_flag = 0;
    m_attach_xml_file.ShowPathWithPathList();  //更新显示
}

void AStarDoubleXML::PlanTheSecondPath() {
    first_path_flag = false;
    if(!m_attach_xml_file.m_astarsearch.NodeList.isEmpty()){
        m_attach_xml_file.m_astarsearch.NodeList.clear();
    }
    QList<MapSearchNode*>::iterator copy_iter = m_attach_xml_file.original_road_network_list.begin();
    for(; copy_iter != m_attach_xml_file.original_road_network_list.end(); copy_iter++){
        m_attach_xml_file.m_astarsearch.NodeList.append(*copy_iter);
    }

    if(!m_attach_xml_file.m_path_map_list.isEmpty()) {
        m_attach_xml_file.m_path_map_list.clear();
    }

    m_attach_xml_file.FindConnectTaskID(first_plan_vehicle_gps);

    int node_num = m_attach_xml_file.original_road_network_list.size();
    int value1 = 1;
    end_task_node_id = m_attach_xml_file.original_road_network_list.at(node_num-1)->node_id;
    if(end_task_node_id != m_attach_xml_file.connect_task_id){
        std::cout<<"second pathplan start&stop id: "<< m_attach_xml_file.connect_task_id
                 << "," << end_task_node_id << std::endl;
        value1 = m_attach_xml_file.PathPlan(m_attach_xml_file.connect_task_id,
                                   end_task_node_id);
    }else{
        QList<MapSearchNode*>::iterator iter = m_attach_xml_file.original_road_network_list.begin();
        for(; iter != m_attach_xml_file.original_road_network_list.end(); iter++){
            if((*iter)->node_id == m_attach_xml_file.connect_task_id){
                m_attach_xml_file.m_path_map_list.append(*iter);
                break;
            }
        }
    }

    //add last node/last intersec/lead point to the m_path_map_list
    int value2 = m_attach_xml_file.AddPassableFork();

    if(value1 == 0 || value2 == 0){
        ROS_FATAL("Replanning: the second plan failed! value1= %d , value2= %d",value1,value2);
        if(!m_attach_xml_file.m_path_map_list.isEmpty()){
            m_attach_xml_file.m_path_map_list.clear();
        }
        QList<MapSearchNode>::iterator copy_iter = temp_path_list.begin();
        for(; copy_iter != temp_path_list.end() ; copy_iter++){
            m_attach_xml_file.m_path_map_list.append(&(*copy_iter));
        }
        m_attach_xml_file.is_forward = true;
        m_attach_xml_file.ShowPathWithPathList();  //更新显示
        return;
    }
    m_attach_xml_file.is_forward = true;
    m_attach_xml_file.m_bPathPlaned = true;
    m_attach_xml_file.m_flag = 0;
    m_attach_xml_file.ShowPathWithPathList();  //更新显示
}

void AStarDoubleXML::dataTransfer() {
    m_attach_xml_file.vehicle_vel = ros_pub.vehicle_vel;
    m_attach_xml_file.gear_position = ros_pub.gear_position;
    m_attach_xml_file.vehicle_theta = ros_pub.vehicle_pose_.z;
}
