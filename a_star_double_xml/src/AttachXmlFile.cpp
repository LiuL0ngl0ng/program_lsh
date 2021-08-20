#include "AttachXmlFile.h"
#include <QDir>
#include <stdio.h>
#include <QMessageBox>
#include <QDomDocument>
#include <QDebug>
#include <qmath.h>
#include <QCoreApplication>
#include <astardoublexml.h>

AttachXmlFile::AttachXmlFile() {
    m_EnvImage_Global_diaplay=cvCreateImage(cvSize(MAP_WIDTH_CELL,MAP_HEIGHT_CELL),8,3);
    //cvZero(m_EnvImage_Global_diaplay);
    cvSet(m_EnvImage_Global_diaplay,cvScalar(255,255,255));
    cvNamedWindow("global env",1);
    //cvMoveWindow("global env",300,0);
    cvShowImage("global env",m_EnvImage_Global_diaplay);
    cvSetMouseCallback("global env",on_mouse_globalenv,this);

    /*if(!m_path_map_list.IsEmpty())
    {
        m_path_map_list.RemoveAll();
    }*/
    if(!m_path_map_list.isEmpty())
    {
        //qDeleteAll(m_path_map_list);
        m_path_map_list.clear();
    }

    m_Pathid_vector.clear();

    vehicle_state.m_prevstate=NULL;
    vehicle_state.pos=NULL;
    vehicle_state.m_nextstate=NULL;
    vehicle_state.distonextstate=0;
    vehicle_state.s=0;
    vehicle_state.dis_to_nextintersection=0;

    m_nStartLastNode = 0;
    m_nstart_world.x =0;
    m_nstart_world.y =0;
    m_fStartNodex = -71927;
    m_fStartNodey = 4398767;

    m_pVelNode     = new Vel_Node;
    m_pTaskNode    = new Task_Node;
    m_pRoadLine1   = new Road_Line;//分配2条道路的内存空间，并对结构体中的数据进行初始化
    m_pRoadLine2   = new Road_Line;
    m_pRoadLine1->m_Probability = 0;
    m_pRoadLine1->Angle_Probability=0;
    m_pRoadLine1->Dist = 0;
    m_pRoadLine1->Distance_Probability =0;
    m_pRoadLine1->R_Intercept = 0;
    m_pRoadLine1->R_Slope = 0;
    m_pRoadLine1->Road_nodex =0;
    m_pRoadLine1->Road_nodey=0;
    m_pRoadLine2->m_Probability = 0;
    m_pRoadLine2->Angle_Probability=0;
    m_pRoadLine2->Dist = 0;
    m_pRoadLine2->Distance_Probability =0;
    m_pRoadLine2->R_Intercept = 0;
    m_pRoadLine2->R_Slope = 0;
    m_pRoadLine2->Road_nodex =0;
    m_pRoadLine2->Road_nodey=0;

    //Replanning
    ReStartNextID = -1;
    if(!obs_node_id.isEmpty()){
        obs_node_id.clear();
    }

    replanning_add_node = 0;

    espx = 0;
    espy = 0;

    //update the vehicle location in real time
    PathPlanFinishFlag=false;

    //build topological map in real time
    intersec_id = 100;
    ordinary_id = 100000;
    leadpoints_id = 1000000;
    add_intersec_ref_dis = 10; //添加或更新岔道口点的控制距离
    eli_task_node_ref_dis = 30;  //忽略触发重规划触发车辆位置周围n米的任务点
    add_ordinary_node_ref_dis = 5;  //添加普通路点的控制距离
    fallback_ref_dis = 10;  // 倒车控制距离，m
    last_intersec_id = -1;
    topology_parent_id = -1;
    connect_task_id = -1;
    replan_flag = false;
    first_plan_tri_intersec_id = -1;
    first_plan_intersec_id = -1;
    fallback_node_id = -1;
    fb_intersec_id = -1;
    last_replan_tri_rec = -1;
    restart_id = 1;
    pub_path_size = 0;

    //路网匹配与更新
    update_flag_for_task = 0;
    m_bPathPlaned = false;
    m_flag = 0;
    m_pStartNode = NULL;
    m_pEndNode = NULL;
    m_ptempnode = NULL;
    m_pnextnode = NULL;
    m_pthirdnode = NULL;
    use_coherence_mapping = true;
    cur_road_type = -1;
    cur_road_vlimit = 10;   //m/s
    vehicle_vel = 0.0;
    gear_position = -1;
    last_match_num = -1;
    switch_point_node = -1;

    m_pcurtask.Task_num = -1;
    m_pnexttask.Task_num = -1;
    m_pthirdtask.Task_num = -1;
    wall_node.Task_num = -1;
    ditch_node.Task_num = -1;

    get_node_from_txt = false;

    //发布路网
    is_forward = true;
    open_dynamic_obs_det = true;
    open_foggy_det = true;
    set_patrol_times = 0;
    numOfPatrols = 0;
    reversing_flag = false;

    //通过界面选择任务点
    task_id = 0;

    windowCenterUpdateF = 0;
    vehicle_theta = 0;
}

AttachXmlFile::~AttachXmlFile() {
    if(NULL != m_EnvImage_Global_diaplay)
    {
        cvReleaseImage(&m_EnvImage_Global_diaplay);
        cvDestroyWindow("global env");
    }

    /*if(!m_astarsearch.NodeList.IsEmpty())
      {
        POSITION d_search_pos;
        d_search_pos = m_astarsearch.NodeList.GetHeadPosition();
        while(d_search_pos)
        {
            //MapSearchNode *dp_search_node = (MapSearchNode *)m_astarsearch.NodeList.GetNext(d_search_pos);
            if(NULL != dp_search_node)
            {
                delete dp_search_node;
                dp_search_node = NULL;
            }
        }
        //QList<MapSearchNode*>::iterator d_search_pos= m_astarsearch.NodeList.begin();
        //for(;i!= m_astarsearch.NodeList.end();++i)
      }*/
    /*if(!m_astarsearch.NodeList.isEmpty()){
        QList<MapSearchNode*>::iterator star_iter = m_astarsearch.NodeList.begin();
        for(; star_iter != m_astarsearch.NodeList.end(); ){
            int erase_flag = 0;
            QList<MapSearchNode*>::iterator road_iter = road_network_list.begin();
            for(; road_iter != road_network_list.end(); road_iter++){
                if((*road_iter)->node_id == (*star_iter)->node_id){
                    star_iter = m_astarsearch.NodeList.erase(star_iter);
                    erase_flag =1;
                    break;
                }
            }
            QList<MapSearchNode*>::iterator intersec_iter = intersec_list.begin();
            for(; intersec_iter != intersec_list.end(); intersec_iter++){
                if((*intersec_iter))
            }
            if(erase_flag == 0){
                star_iter++;
            }
        }
    }*/
    if(!m_astarsearch.NodeList.isEmpty())       //revise
    {
        //qDeleteAll(m_astarsearch.NodeList);
        m_astarsearch.NodeList.clear();
    }

    if(!m_path_map_list.isEmpty())
    {
        //qDeleteAll(m_path_map_list);
        m_path_map_list.clear();
    }

    if(!road_network_list.isEmpty()){
        qDeleteAll(road_network_list);
        road_network_list.clear();
    }
    if(!intersec_list.isEmpty()){
        qDeleteAll(intersec_list);
        intersec_list.clear();
    }
    if(!original_road_network_list.isEmpty()){
        qDeleteAll(original_road_network_list);
        original_road_network_list.clear();
    }

    if( NULL != m_pVelNode)
        delete m_pVelNode;
    if(NULL!=m_pTaskNode)       //notice:keyizheyang release?
        delete m_pTaskNode;
    if(NULL != m_pRoadLine2)
        delete m_pRoadLine2;
    if(NULL != m_pRoadLine1)
        delete m_pRoadLine1;
}

void AttachXmlFile::GetNodeFromFile() {
    //lsh//1.设置道路文件读取路径、验证文件路径是否存在、读取路径下所有文件、排序、遍历每个文件，读取后缀名为.xml的文件
    //lsh//2.处理xml文件
    //lsh//  将道路id存入m_Pathid_vector
    //lsh//  将包含拓扑关系和详细信息的路点录入m_astarsearch.NodeList
    //lsh//  以道路为单位储存路点的向量m_RoadList，其不包含拓扑关系
    /*CFileFind tempFind;
    char tempFileFind[100];
    sprintf_s(tempFileFind,"xml_director\\*.*");

    BOOL bfind = (BOOL)tempFind.FindFile(tempFileFind);
    //查找到的文件里面包含“.”和 ".."两个文件，这两个文件是不想被采用的所以要排除
    //返回值: 如果成功,则返回非零值,否则为0。
    while(bfind)
    {
        bfind = tempFind.FindNextFile();

        if(tempFind.IsDots())
            continue;

        if(tempFind.IsDirectory())
        {
            CString str = tempFind.GetFilePath();

            CFileFind tempFind1;
            char tempFileFind1[200];
            sprintf_s(tempFileFind1,"%s\\*.xml",str);

            BOOL bfind1 = (BOOL)tempFind1.FindFile(tempFileFind1);
            while(bfind1)
            {
                bfind1 = tempFind1.FindNextFile();

                if(tempFind1.IsDots() || tempFind1.IsDirectory())
                    continue;

                char file_name[200];
                sprintf(file_name,"%s",tempFind1.GetFilePath());
                char pure_file_name[200];
                sprintf(pure_file_name,"%s",tempFind1.GetFileName());

                GetXmlInfoWithSearchList(file_name,pure_file_name);
            }
        }
    }
    tempFind.Close();*/

    //should be started with </catkin_ws>
    //lsh//读取路径点的路径
    QString xml_file_path(base_dir + way_path);
//    QString xml_file_path("/home/tb/Desktop/ros/src/a_star_double_xml/xml_director/GPS_test_July31_out");
    qDebug() <<  xml_file_path;

    QDir dir(xml_file_path);//lsh//定义一个QDir类，对象名为dir，使用xml_file_path来初始化
    if(!dir.isRelative())//lsh//可以使用QDir类的方法isRelative()或者是isAbsolute()来判断QDir指向的路径是相对路径还是绝对路径
        //lsh//如果是相对路径，可使用方法makeAbsolute()将相对路径转换为绝对路径。
    {
        qWarning("Started with <~/catkin_ws>");
    }
    if(!dir.exists()) /*判断文件夹是否存在*/
    {
        qWarning("Cannot find the example directory");
    }
    dir.setFilter(QDir::Files); /*设置dir的过滤模式,表示只遍历本文件夹内的文件*/
    dir.setSorting(QDir::Name|QDir::Time|QDir::Reversed);//lsh//将文件排序
    //lsh// QDir::Name - 按名称排序、QDir::Time - 按时间排序（修改时间）、QDir::Reversed - 相反的排序顺序（由从小到大->从大到小）。
    QFileInfoList fileList = dir.entryInfoList();  /*获取本文件夹内所有文件的信息*/
    int fileCount = fileList.count(); /*获取本文件夹内的文件个数*/
    if(fileCount==0)
    {
        qWarning("the fileCount is 0!");
        // return;
    }
    for(int i=0;i<fileCount;i++)  /*遍历每个文件*/
    {
        QFileInfo fileInfo = fileList[i]; /*获取每个文件信息*/
        QString suffix = fileInfo.suffix(); /*获取文件后缀名*/
        /*筛选出所有xml文件(如果要筛选其他格式的文件则根据需要修改"xml"中的字符串即可)*/
        if(QString::compare(suffix, QString("xml"), Qt::CaseInsensitive) == 0)
            //lsh//QString::compare负责比较两字符串是否相等
            //lsh//Qt::CaseSensitivity 为枚举类型, 可取值Qt::CaseSensitive 和 Qt::CaseInsensitive, 表示匹配（对大小写）的灵敏度。
        {
            QString filePath = fileInfo.absoluteFilePath();/*获取文件绝对路径即全路径*/
            QString fileName = fileInfo.baseName();/*获取文件后名(不带后缀的文件名)*/
            GetXmlInfoWithSearchList(filePath,fileName);
            //lsh//将道路id存入m_Pathid_vector
            //lsh//将包含拓扑关系和详细信息的路点录入m_astarsearch.NodeList
            //lsh//以道路为单位储存路点的向量m_RoadList，其不包含拓扑关系
        }
    }
}

void AttachXmlFile::GetNodeFromTxtFile() {
    QString home = QDir::homePath();
    QFile file(home + "/taskfile/WRXLC2019.txt");
//    QFile file(base_dir + "/2018_7_14_1.txt");
    if(!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        qDebug()<<"GetNodeFromTxtFile(): Can't open the file!"<<endl;
    }

    int all_node_null = 10000;
    Road tempRoad;
    tempRoad.RoadID = 10000;
    tempRoad.way_version = "doubleway";
    int parent_node_id = -1;
    int current_node_id = -1;

    file.seek(0);       //go back to the begin of the file
    while(!file.atEnd())
    {
        QByteArray line = file.readLine();
        QString str_line(line);
        QString str_GPS_Lat = str_line.section(QRegExp("[  ]"),2,2);
        QString str_GPS_Lon = str_line.section(QRegExp("[  ]"),1,1);
        QString str_type = str_line.section(QRegExp("[  ]"),4,4);
        //qDebug()<<"str_gps_lat"<<str_GPS_Lat<<endl;
        float f_GPS_Lat = str_GPS_Lat.toFloat();    //note precision
        float f_GPS_Lon = str_GPS_Lon.toFloat();
        int node_type = str_type.toInt();

        all_node_null++;
        MapSearchNode* temp_node = new MapSearchNode;
        temp_node->lon = f_GPS_Lon;
        temp_node->lat = f_GPS_Lat;
        temp_node->node_id = all_node_null;
        temp_node->road_id = 10000;
        temp_node->intersection = false;
        Position_Trans_From_ECEF_To_UTM(temp_node->lat,temp_node->lon,0,0,&temp_node->x,&temp_node->y);
        temp_node->type = node_type;
        temp_node->vlimit = 5.0;
        temp_node->concave_obs_det = true;
        temp_node->dynamic_obs_det = true;
        temp_node->foogy_det = true;
        temp_node->water_det = true;
        temp_node->wall_area = true;
        temp_node->ditch_area = true;
        m_astarsearch.NodeList.append(temp_node);

        std::cout << "TXT node id :" << temp_node->node_id << std::endl;
        parent_node_id = current_node_id;
        current_node_id = temp_node->node_id;

        //下面开始建立路点之间的拓扑关系
        if(parent_node_id != -1)
        {
            MapSearchNode *cur_node = NULL, *parent_node = NULL;
            QList<MapSearchNode*>::iterator iter = m_astarsearch.NodeList.begin();        //note
            for( ; iter != m_astarsearch.NodeList.end(); iter++)
            {
                if((*iter)->node_id == current_node_id)
                    cur_node=(*iter);
                if((*iter)->node_id == parent_node_id)
                    parent_node=(*iter);
            }
            BuildTopologicalRelation(cur_node, parent_node);
        }

        //没有岔道口以及道路ID属性
        RoadNode tempRoadNode;
        tempRoadNode.lat = f_GPS_Lat;
        tempRoadNode.lon = f_GPS_Lon;
        tempRoadNode.NodeID = all_node_null;
        tempRoadNode.type = node_type;
        Position_Trans_From_ECEF_To_UTM(tempRoadNode.lat,tempRoadNode.lon,0,0, &tempRoadNode.x,&tempRoadNode.y);
        tempRoadNode.vlimit = 5.0;
        tempRoadNode.dynamic_obs_det = true;
        tempRoadNode.concave_obs_det = true;
        tempRoadNode.water_det = true;
        tempRoadNode.foogy_det = true;
        tempRoadNode.wall_area = true;
        tempRoadNode.ditch_area = true;
        tempRoad.RoadNodeList.append(tempRoadNode);
    }
    m_RoadList.append(tempRoad);
    file.close();
}

MapSearchNode* AttachXmlFile::GetNodeFormList(int node_id) {
    //lsh//得到NodeList中对应id的路点

    /*POSITION get_search_pos;
    if(m_astarsearch.NodeList.IsEmpty())
    {
        TRACE("ERROR: can't get search node because nodelist is empty \n");
        return get_search_node;
    }
    else
    {
        get_search_pos = m_astarsearch.NodeList.GetHeadPosition();

        while(get_search_pos)
        {
            get_search_node = (MapSearchNode*)m_astarsearch.NodeList.GetNext(get_search_pos);

            if(get_search_node->node_id == node_id)
            {
                break;
            }
        }
    }

    if(get_search_node->node_id != node_id)
    {
        TRACE("ERROR:can't get search node \n");
        return get_search_node;
    }
    else
    {
        return get_search_node;
    }*/

    MapSearchNode *get_search_node = NULL;
    if(m_astarsearch.NodeList.isEmpty()) {
        ROS_FATAL("In GetNodeFormList:can't get search node, because nodelist is empty");
        return get_search_node;
    } else {
        QList<MapSearchNode *>::iterator get_search_pos=m_astarsearch.NodeList.begin();
        for( ; get_search_pos!=m_astarsearch.NodeList.end() ; get_search_pos++) {
            if((*get_search_pos)->node_id == node_id) {
                get_search_node=(*get_search_pos);
                break;
            }
        }
    }

    if(get_search_node == NULL || get_search_node->node_id != node_id) {
        ROS_FATAL("In GetNodeFormList:can't get search node.");
        return get_search_node;
    } else {
        return get_search_node;
    }

}
//--------------------------上面是交互函数-------------------------------------------------------
//--------------------------下面是私有函数-------------------------------------------------------

// 路网的加载有7种情况
//　	                                              子节点为路点
//                               _________________________|___________________________
//　　　　　　　　　　　　　　　|						  |                           |
//   子节点为路口点         3.子节点　　　　　　  父节点为路口点            　　 父节点为路点
//	 ______|_______　　　　　 没有父　　　　　  　________|________　　　　    _______|________
// 	|              |　　　　　节点               |                 |          |                |
//1.子父节点      2.子父节点                  4.子父节点       5.子父节点    6.子父节点      7.子父节点
//为同一xml　　　　为不同xml　　　　　　　　  为同一xml　　　　为不同xm　  　为同一xml　　　　为不同xml
//
//链表中包含有该子节点，则这个节点为路口点，否则为路点。子节点的父节点的road_count不等于1，则变为父节点为路口点。对于不同xml，则没有关系，不作处理
//
// 通过道路id判断是否为同一xml
//
// 通过version判断是否为路口点

int AttachXmlFile::InsertNodeIntoList(XmlNodeInfo *xml_parent_node_info,
                                      XmlNodeInfo *xml_node_info) {
    /*
     * //lsh//检查该路点是否在别的道路中出现过，记录出现次数
     * //lsh//未出现过，录入并建立拓扑关系
     * */

    //本路段的第一个节点
    bool node_existed=false;
    QList<MapSearchNode*>::iterator iter=m_astarsearch.NodeList.begin();
    for( ; iter!=m_astarsearch.NodeList.end(); iter++) {
        if((*iter)->node_id==xml_node_info->nodeID) {
            node_existed=true;
            (*iter)->road_count++;//lsh//该节点属于几条道路
            break;
        }
    }
    //lsh//通过id检查m_astarsearch.NodeList中是否包含要录入的路点
    //lsh//检查该路点是否在别的道路中出现过
    if(!node_existed)
    {
        //一个新的节点，获得当前node的位置和id信息，并加入NodeList；
        MapSearchNode *search_node;
        search_node = new MapSearchNode;
        Position_Trans_From_ECEF_To_UTM(xml_node_info->flat,xml_node_info->flon,0,0, &search_node->x,&search_node->y);
        search_node->lat=xml_node_info->flat;
        search_node->lon=xml_node_info->flon;
        search_node->road_id=xml_node_info->roadID;
        search_node->node_id = xml_node_info->nodeID;
        search_node->type = xml_node_info->type;
        search_node->vlimit = xml_node_info->vlimit;
        search_node->intersection=xml_node_info->intersection;
        search_node->concave_obs_det = xml_node_info->concave_obs_det;
        search_node->dynamic_obs_det = xml_node_info->dynamic_obs_det;
        search_node->foogy_det = xml_node_info->foogy_det;
        search_node->water_det = xml_node_info->water_det;
        search_node->wall_area = xml_node_info->wall_area;
        search_node->ditch_area = xml_node_info->ditch_area;
        //lsh//将XmlNodeInfo类的xml_node_info转化为MapSearchNode类的*search_node
        m_astarsearch.NodeList.append(search_node);             //向NodeList中插入节点
    }//lsh//若该路点未在别的道路中出现过，将该路点记录在m_astarsearch.NodeList

    //lsh//建立路点之间的拓扑关系（包括单双行）
    //lsh//每条道路的第一个节点是没有母节点的
    //lsh//若其存在母节点，会有其他路径的最后一个节点正好和他重合，进而在另外一条路径中完成了对该节点母节点的录入
    if(xml_parent_node_info->nodeID!=0) {
        MapSearchNode *cur_node=NULL, *parent_node=NULL;
        QList<MapSearchNode*>::iterator iter=m_astarsearch.NodeList.begin();
        for( ; iter!=m_astarsearch.NodeList.end(); iter++) {
            if((*iter)->node_id==xml_node_info->nodeID)
                cur_node=(*iter);
            if((*iter)->node_id==xml_parent_node_info->nodeID)
                parent_node=(*iter);
        }
        if(cur_node && parent_node && cur_node!=parent_node)
        {
            cur_node->parentList.append(parent_node);
            cur_node->parentNum++;
            parent_node->successorList.append(cur_node);
            parent_node->successorNum++;
            //if(QString::compare(xml_node_info->way_version, QString("oneway"), Qt::CaseInsensitive) == 0)
            if(QString::compare(xml_node_info->way_version, QString("oneway"), Qt::CaseInsensitive)) {
                //lsh//如果不是单行道
                parent_node->parentList.append(cur_node);
                parent_node->parentNum++;
                cur_node->successorList.append(parent_node);
                cur_node->successorNum++;
            }
        }else{
            if(cur_node == NULL){
                ROS_WARN("cur_node = null");
            } else if(parent_node == NULL){
                ROS_WARN("parent_noe = null");
            } else if (cur_node == parent_node){
                ROS_WARN("cur_node = parent_node, road_id %d , cur_node id %d , parent_node id %d"
                ,xml_node_info->roadID,cur_node->node_id,parent_node->node_id);
            }
            ROS_WARN("InsertNodeIntoList: build node %d's topological relation failed!",cur_node->node_id);
        }
    }
    return 0;

}

int AttachXmlFile::GetXmlInfoWithSearchList(QString& filePath, QString& fileName) {
//lsh//使用只读打开文件
//lsh//读取<way>类型节点，记录其ID属性，也就是记录道路编号
//lsh//读取<way></way>中每个子节点，寻找名字是<tag>的子节点，读取其k和v属性用于判断单双行
//lsh//在filename与nroad_id相等时才能添加节点
//lsh//检查现有的读取过的id中是否有和即将读入的道路nroad_id相同的
//lsh//把即将读入的道路id存入m_Pathid_vector
//lsh//读取<node>类型节点，读取每个节点每个<node>属性的值
//lsh//读取每个<node>的每个子节点，寻找名字是<tag>的子节点，读取其v属性用于判断是否为路口
//lsh//检查该路点是否在别的道路中出现过，记录出现次数，未出现过，录入并建立拓扑关系m_astarsearch.NodeList
//lsh//把道路录入以道路为单位储存路点的向量m_RoadList，其用于道路匹配
    /*//static XmlNodeInfo xml_parent_node_info;
    XmlNodeInfo xml_parent_node_info;//本xml文档中的
    xml_parent_node_info.nodeID = 0;

    //xerces的一些初始化
     try {
        XMLPlatformUtils::Initialize();
    }
    catch (const XMLException& toCatch) {
        char* message = XMLString::transcode(toCatch.getMessage());
        cout << "Error during initialization! :\n"
             << message << "\n";
        XMLString::release(&message);
        return 1;
    }
    XercesDOMParser* parser = new XercesDOMParser();
    parser->setValidationScheme(XercesDOMParser::Val_Always);
    parser->setDoNamespaces(true);    // optional

    ErrorHandler* errHandler = (ErrorHandler*) new HandlerBase();
    parser->setErrorHandler(errHandler);

    char* xmlFile = file_name;

    try {
        parser->parse(xmlFile);
    }
    catch (const XMLException& toCatch) {
        char* message = XMLString::transcode(toCatch.getMessage());
        cout << "Exception message is: \n"
             << message << "\n";
        XMLString::release(&message);
        return -1;
    }
    catch (const DOMException& toCatch) {
        char* message = XMLString::transcode(toCatch.msg);
        cout << "Exception message is: \n"
             << message << "\n";
        XMLString::release(&message);
        return -1;
    }
    catch (...) {
        cout << "Unexpected Exception \n" ;
        return -1;
    }

    xercesc_3_1::DOMDocument *doc = parser->getDocument();
    DOMElement *Ele = doc->getDocumentElement();

    //得到道路属性
    char cway_version[20]="";//单向或双向由way里的tag中的highway来表明
    int nroad_id;//道路id
    DOMNodeList *road_nodelist = Ele->getElementsByTagName(X("way"));
    DOMNode *road_node = road_nodelist->item(0);
    //定位到“0”号位置

    if(road_node->hasAttributes())
    {	//hasAttributes() 方法在当前元素节点拥有属性时返回 TRUE，否则返回 FALSE。
        DOMNamedNodeMap *road_map =  road_node->getAttributes();
        DOMNode *road_id_node = road_map->getNamedItem(X("id"));
        char* road_id_str = XMLString::transcode(road_id_node->getNodeValue());
        nroad_id = atoi(road_id_str);//本条道路的id
        //获取本条路的ID
        DOMNodeList *tag_elelist = road_node->getChildNodes();//获取“0”号位置的子节点列表

        //循环访问WAY标签下面的子节点
        for(int i=0;i<tag_elelist->getLength();i++)
        {
            DOMNode *tag_node=tag_elelist->item(i);
            char* tag_name=XMLString::transcode(tag_node->getNodeName());
            if(!strcmp(tag_name,"tag"))
            {	//（1）字符串1=字符串2，返回0
                //（2）字符串1>字符串2，返回一个正整数
                //（3）字符串1<字符串2，返回一个负整数。
                if(tag_node->hasAttributes())
                {
                    DOMNamedNodeMap *tag_maps =  tag_node->getAttributes();
                    DOMNode *tag_k_node = tag_maps->getNamedItem(X("k"));
                    char* k_str = XMLString::transcode(tag_k_node->getNodeValue());
                    if(!strcmp(k_str,"oneway"))
                    {//如果是oneway则返回0，前面加一个非，说明匹配oneway即执行
                        DOMNode *tag_v_node = tag_maps->getNamedItem(X("v"));
                        char* v_str = XMLString::transcode(tag_v_node->getNodeValue());
                        if(!strcmp(v_str,"yes"))
                        {//为什么要判断是否为yes?
                            sprintf(cway_version,"oneway");
                        }
                    }
                }
            }
        }
    }

    char road_id_from_filename_char[20];
    memset(road_id_from_filename_char,0,20);
    for(int i=0;i<20;i++)
    {//为什么是20，pure_file_name是一个200长度的字符数组，如果20个字符就差不多了，那为什么要将pure_file_name声明为200呢？
        if(pure_file_name[i]=='.')
            break;
        road_id_from_filename_char[i]=pure_file_name[i];
        //这段程序是用来提取*.xml前的*吧
    }
    int road_id_from_filename=atoi(road_id_from_filename_char);

    if(road_id_from_filename==nroad_id)//只有道路id与文件名相同才能写入node_list
    {
        bool roadexisted=false;
        for(int t=0;t<m_Pathid_vector.size();t++)
        {//又称向量，跟数组结构差不多。它的内存是连续的，拥有与数组一样的特点，但它又更加灵活，
         //可以在首尾两端插入删除数据更加方便。支持[]操作符，在中间插入删除元素效率很低，而且支持扩容。
            int tmpid=m_Pathid_vector.at(t);
            if(tmpid==nroad_id)
            {
                roadexisted=true;
                break;
            }
        }
            //上几句程序的作用是判断这个road是否在之前被插入到m_Pathid_vector容器中过，如果没有，那么就执行下面的程序
        if(!roadexisted)//同一个id的道路只能插入一次
        {
            m_Pathid_vector.push_back(nroad_id);   //push_back在数组的最后添加一个数据**

            //得到节点属性
            DOMNodeList *node_nodelist = Ele->getElementsByTagName(X("node"));//获取当前道路的node列表
            int node_lenght = (int)node_nodelist ->getLength();
            for(int i=0;i < node_lenght;i++)
            {//对当前道路的node列表依次遍历
                DOMNode *node_node = node_nodelist->item(i);
                if(node_node->hasAttributes())//得到xml文件的特性
                {
                    DOMNamedNodeMap *node_map =  node_node->getAttributes();
                    DOMNode *lon_node = node_map->getNamedItem(X("lon"));
                    char* lon_str = XMLString::transcode(lon_node->getNodeValue());
                    DOMNode *lat_node = node_map->getNamedItem(X("lat"));
                    char* lat_str = XMLString::transcode(lat_node->getNodeValue());
                    DOMNode *nodeID_node = node_map->getNamedItem(X("id"));
                    char* nodeID_str = XMLString::transcode(nodeID_node->getNodeValue());

                    bool intersection=false;
                    DOMNodeList *tag_nodes = node_node->getChildNodes();
                    for(int j=0;j<tag_nodes->getLength();j++)
                    {//node_node不一定有子节点“tag”
                        DOMNode *tag_node=tag_nodes->item(j);
                        char* tag_name=XMLString::transcode(tag_node->getNodeName());
                        if(!strcmp(tag_name,"tag"))
                        {
                            if(tag_node->hasAttributes())
                            {
                                DOMNamedNodeMap *tag_maps =  tag_node->getAttributes();
                                DOMNode *tag_v_node = tag_maps->getNamedItem(X("v"));
                                char* v_str = XMLString::transcode(tag_v_node->getNodeValue());
                                if(!strcmp(v_str,"traffic_signals"))
                                {
                                    intersection=true;
                                }
                            }
                        }
                    }

                    //创建一个xml_node_info结构体变量，记录从xml中提取出来的道路节点信息
                    XmlNodeInfo xml_node_info;
                    xml_node_info.flat = (float)atof(lat_str);
                    xml_node_info.flon = (float)atof(lon_str);
                    xml_node_info.nodeID = atoi(nodeID_str);
                    xml_node_info.intersection=intersection;
                    sprintf(xml_node_info.way_version,cway_version);


                    InsertNodeIntoList(&xml_parent_node_info,&xml_node_info);

                    xml_parent_node_info = xml_node_info;
                    //xml_parent_node_info节点为xml_node_info的父节点
                }
            }
        }
    }
    delete parser;
    delete errHandler;
    return 0;*/

    //static XmlNodeInfo xml_parent_node_info;
    XmlNodeInfo xml_parent_node_info;//本xml文档中的
    xml_parent_node_info.nodeID = 0;

    //Open a xml file
    QDomDocument doc;
    QFile file(filePath);
    QString error = "";
    int row = 0, column = 0;
    if (!file.open(QIODevice::ReadOnly)) return -2;
    //lsh//使用QIODevice::WriteOnly或QIODevice::ReadOnly标志，可以指定打开特定文件的模式。

    if(!doc.setContent(&file, false, &error, &row, &column)) {
        qDebug() << "parse file failed:" << row << "---" << column <<":" <<error;
        file.close();
        return -1;
    }
//lsh//貌似是将doc与file建立连接，再判断文件特定位置是否可解析
    file.close();

    //Parsing XML
    QString cway_version;//单向或双向由way里的tag中的highway来表明
    int nroad_id;//道路id

    QDomNodeList road_nodelist = doc.elementsByTagName("way");    /*< 读取类型节点集合 */
    //lsh//返回的是一个<way></way>包围的childNodes()
    //lsh//但自己的理解是返回所有<way></way>本身的节点
    QDomNode road_node=road_nodelist.item(0);

    if(road_node.hasAttributes()) {
        //lsh//判断其是否拥有属性
        QString road_id_node=road_node.toElement().attribute("id");
        nroad_id=road_id_node.toInt();
        //lsh//读取道路ID
        //qDebug()<<"road id is:"<<nroad_id<<endl;

        //Visiting the child node of the WAY tag
        QDomNodeList tag_elelist=road_node.childNodes();
        //lsh//读取<way></way>中每个子节点
        for(int i=0; i<tag_elelist.count(); i++) {
            QDomNode node =tag_elelist.at(i);
            QString node_name=node.nodeName();
            if(QString::compare(node_name, QString("tag"), Qt::CaseInsensitive) == 0) {
                if(node.hasAttributes()) {//if you'd like a function of double way ,you could add code into the code below.
                    QString k_value=node.toElement().attribute("k");
                    if(QString::compare(k_value, QString("oneway"), Qt::CaseInsensitive) == 0) {
                        QString v_value=node.toElement().attribute("v");
                        if(QString::compare(v_value, QString("yes"), Qt::CaseInsensitive) == 0) {
                            cway_version="oneway";
                            //qDebug()<<"cway_version:"<<cway_version<<endl;
                        }
                    }else{
                        cway_version="doubleway";
                    }
                }
            }
        }

    }

    if(fileName.toInt()==nroad_id)  //只有在filename与nroad_id相等时才能添加节点
    {
        bool roadexisted=false;
        for(unsigned int t=0;t<m_Pathid_vector.size();t++)
        {//又称向量，它的内存是连续的，拥有与数组一样的特点，但它又更加灵活，
            //可以在首尾两端插入删除数据更加方便。支持[]操作符，在中间插入删除元素效率很低，而且支持扩容。
            int tmpid=m_Pathid_vector.at(t);
            if(tmpid==nroad_id)
            {
                roadexisted=true;
                break;
            }
        }
        //lsh//检查现有的读取过的id中是否有和即将读入的道路nroad_id相同的
        if(!roadexisted)//同一个id的道路只能插入一次
        {
            m_Pathid_vector.push_back(nroad_id);
            //lsh//把即将读入的道路id存入m_Pathid_vector
            Road tempRoad;
            tempRoad.RoadID = nroad_id;
            tempRoad.way_version = cway_version;

            //得到节点属性
            QDomNodeList node_nodelist=doc.elementsByTagName("node");
            //lsh//但自己的理解是返回所有<node></node>本身的节点
            for(int i=0; i<node_nodelist.count(); i++) {
                QDomNode node = node_nodelist.at(i);

                float lon,lat;
                int nodeID;
                bool intersection = false;
                int type;
                double vlimit;
                int concave_ob;
                int dyn_ob;
                int smoke;
                int water;
                int wall;
                int ditch;

                if(node.hasAttributes()) {
                    QString lon_str=node.toElement().attribute("lon");
                    QString lat_str=node.toElement().attribute("lat");
                    QString nodeID_str=node.toElement().attribute("id");
                    QString node_type_str = node.toElement().attribute("type");
                    QString node_vlimit_str = node.toElement().attribute("vel");
                    QString node_concave_ob = node.toElement().attribute("concave_ob");
                    QString node_dyn_ob = node.toElement().attribute("dyn_ob");
                    QString node_smoke = node.toElement().attribute("smoke");
                    QString node_water = node.toElement().attribute("water");
                    QString node_wall = node.toElement().attribute("wall");
                    QString node_ditch = node.toElement().attribute("ditch");
                    //lsh//读取文件中每个node节点下每个属性的值

                    lon=lon_str.toFloat();
                    lat=lat_str.toFloat();
                    nodeID=nodeID_str.toInt();
                    type = node_type_str.toInt();
                    vlimit = node_vlimit_str.toDouble();
                    concave_ob = node_concave_ob.toInt();
                    dyn_ob = node_dyn_ob.toInt();
                    smoke = node_smoke.toInt();
                    water = node_water.toInt();
                    wall = node_wall.toInt();
                    ditch = node_ditch.toInt();

                    QDomNodeList tag_nodes=node.childNodes();
                    //lsh//读取<node></node>中每个子节点
                    for(int j=0; j<tag_nodes.count(); j++) {
                        QDomNode tag_node=tag_nodes.item(0);
                        //lsh//自己的理解是返回每个子节点<subnode></subnode>中的<subnode>
                        QString tag_name=tag_node.nodeName();
                        if(QString::compare(tag_name, QString("tag"), Qt::CaseInsensitive) == 0) {
                            if(tag_node.hasAttributes()) {
                                QString tag_v_value=tag_node.toElement().attribute("v");
                                if(QString::compare(tag_v_value, QString("traffic_signals"), Qt::CaseInsensitive) == 0){
                                    intersection=true;
                                }
                            }
                        }
                    }

                    XmlNodeInfo xml_node_info;
                    xml_node_info.flat =lat;
                    xml_node_info.flon =lon;
                    xml_node_info.roadID=nroad_id;
                    xml_node_info.nodeID =nodeID;
                    xml_node_info.intersection=intersection;
                    xml_node_info.way_version=cway_version;
                    xml_node_info.type = type;
                    xml_node_info.vlimit = vlimit;
                    if(concave_ob == 1){
                        xml_node_info.concave_obs_det = true;
                    }else{ xml_node_info.concave_obs_det = false;}
                    if(dyn_ob == 1){
                        xml_node_info.dynamic_obs_det = true;
                    }else{xml_node_info.dynamic_obs_det = false;}
                    if(smoke == 1){
                        xml_node_info.foogy_det = true;
                    }else{xml_node_info.foogy_det = false;}
                    if(water == 1){
                        xml_node_info.water_det = true;
                    }else{xml_node_info.water_det = false;}
                    if(wall == 1){
                        xml_node_info.wall_area = true;
                    }else{xml_node_info.wall_area = false;}
                    if(ditch == 1){
                        xml_node_info.ditch_area = true;
                    }else{xml_node_info.ditch_area = false;}
                    InsertNodeIntoList(&xml_parent_node_info,&xml_node_info);       //build the NodeList
                    //lsh//xml_parent_node_info是整个读取程序开始时创建的，xml_node_info是某个节点自己的信息
                    //lsh//检查该路点是否在别的道路中出现过，记录出现次数
                    //lsh//未出现过，录入并建立拓扑关系
                    xml_parent_node_info = xml_node_info;//lsh//将该节点设为下一个节点的父节点


                    RoadNode tempRoadNode;
                    tempRoadNode.lat=lat;
                    tempRoadNode.lon=lon;
                    tempRoadNode.NodeID=nodeID;
                    Position_Trans_From_ECEF_To_UTM(tempRoadNode.lat,tempRoadNode.lon,0,0, &tempRoadNode.x,&tempRoadNode.y);
                    tempRoadNode.type = type;
                    tempRoadNode.vlimit = vlimit;
                    tempRoadNode.concave_obs_det = xml_node_info.concave_obs_det;
                    tempRoadNode.dynamic_obs_det = xml_node_info.dynamic_obs_det;
                    tempRoadNode.foogy_det = xml_node_info.foogy_det;
                    tempRoadNode.water_det = xml_node_info.water_det;
                    tempRoadNode.wall_area = xml_node_info.wall_area;
                    tempRoadNode.ditch_area = xml_node_info.ditch_area;
                    tempRoad.RoadNodeList.append(tempRoadNode);
                }
            }

            m_RoadList.append(tempRoad);
        }
    }

}

void AttachXmlFile::Position_Trans_From_ECEF_To_UTM(float latitude, float longitude,
                                                    float e0, float n0, float *e, float *n) {
    //lsh//gps to UTM
    cartographer::transform::TransformTools tft;
    double x, y;
    tft.geographic_to_utm(longitude, latitude, &x, &y);
    if(if_vrep == true){
        *e = x-500000;
        *n = y;
    }
    else{
        *e = x;
        *n = y;
    }
}

void AttachXmlFile::InitDisplay() {
    //lsh//找到整个地图左下角的点作为绘制原点
    //lsh//

    //初始化显示
    global_env_zoom=0.5;
    delta_origin_pos.x=0;
    delta_origin_pos.y=0;//初始位置偏移量为0

    double miny_ECEF=10000000000;
    double minx_ECEF=10000000000;


    /*for(POSITION i=m_astarsearch.NodeList.GetHeadPosition();i!=NULL;)
    {
        MapSearchNode *node_ptr=(MapSearchNode*)m_astarsearch.NodeList.GetNext(i);*/
    QList<MapSearchNode *>::iterator node_ptr=m_astarsearch.NodeList.begin();
    for(; node_ptr!=m_astarsearch.NodeList.end(); node_ptr++)
    {
        if((*node_ptr)->y<miny_ECEF) miny_ECEF=(*node_ptr)->y;
        if((*node_ptr)->x<minx_ECEF) minx_ECEF=(*node_ptr)->x;
    }
    //lsh//找到整个地图左下角的点
    delta_origin_pos.x=minx_ECEF;
    delta_origin_pos.y=miny_ECEF;//lsh//绘图原点的经纬度坐标

    ShowPathWithPathList();
}

void AttachXmlFile::ShowRoadWithNodeList() {
    //qDebug()<<"display"<<endl;
    //cvZero(m_EnvImage_Global_diaplay);
    cvSet(m_EnvImage_Global_diaplay,cvScalar(255,255,255));


    //给v_mapping_list画线
    QList<MapSearchNode*>::iterator first_ptr = v_mapping_list.begin();
    if(first_ptr != v_mapping_list.end()){
        QList<MapSearchNode*>::iterator sec_ptr = first_ptr+1;
        for(; sec_ptr != v_mapping_list.end(); sec_ptr++) {

            state_struct tmp_node_state_first;
            pos_int tmp_node_pos_first;
            tmp_node_state_first.position.x=(*first_ptr)->x;
            tmp_node_state_first.position.y=(*first_ptr)->y;
            tmp_node_pos_first.x=int((tmp_node_state_first.position.x-delta_origin_pos.x)*global_env_zoom);
            tmp_node_pos_first.y=int(MAP_HEIGHT_CELL-(tmp_node_state_first.position.y-delta_origin_pos.y)*global_env_zoom);

            state_struct tmp_node_state_sec;
            pos_int tmp_node_pos_sec;
            tmp_node_state_sec.position.x=(*sec_ptr)->x;
            tmp_node_state_sec.position.y=(*sec_ptr)->y;
            tmp_node_pos_sec.x=int((tmp_node_state_sec.position.x-delta_origin_pos.x)*global_env_zoom);
            tmp_node_pos_sec.y=int(MAP_HEIGHT_CELL-(tmp_node_state_sec.position.y-delta_origin_pos.y)*global_env_zoom);
            cvLine(m_EnvImage_Global_diaplay,
                   cvPoint(tmp_node_pos_first.x,tmp_node_pos_first.y),
                   cvPoint(tmp_node_pos_sec.x,tmp_node_pos_sec.y),COLORGREY,3);

            first_ptr = sec_ptr;
        }
    }

    //画连接子父节点的线段 m_astarsearch.NodeList
    QList<MapSearchNode *>::iterator node_ptr1=original_road_network_list.begin();
    for(; node_ptr1!=original_road_network_list.end() ;node_ptr1++) {
        state_struct tmp_state_path_map;
        tmp_state_path_map.position.x=(*node_ptr1)->x;
        tmp_state_path_map.position.y=(*node_ptr1)->y;
        pos_int tmp_pos_path_map;
        tmp_pos_path_map.x=int((tmp_state_path_map.position.x-delta_origin_pos.x)*global_env_zoom);
        tmp_pos_path_map.y=int(MAP_HEIGHT_CELL-(tmp_state_path_map.position.y-delta_origin_pos.y)*global_env_zoom);

        /*for(POSITION j=node_ptr->successorList.GetHeadPosition();j!=NULL;)
        {
            MapSearchNode *succnode_ptr=(MapSearchNode*)node_ptr->successorList.GetNext(j);*/
        QList<MapSearchNode *>::iterator succnode_ptr =(*node_ptr1)->successorList.begin();
        for(; succnode_ptr!=(*node_ptr1)->successorList.end(); succnode_ptr++)
        {
            state_struct tmp_state_path_map1;
            tmp_state_path_map1.position.x=(*succnode_ptr)->x;
            tmp_state_path_map1.position.y=(*succnode_ptr)->y;
            pos_int tmp_pos_path_map1;
            tmp_pos_path_map1.x=int((tmp_state_path_map1.position.x-delta_origin_pos.x)*global_env_zoom);
            tmp_pos_path_map1.y=int(MAP_HEIGHT_CELL-(tmp_state_path_map1.position.y-delta_origin_pos.y)*global_env_zoom);
            cvLine(m_EnvImage_Global_diaplay,cvPoint(tmp_pos_path_map.x,tmp_pos_path_map.y),cvPoint(tmp_pos_path_map1.x,tmp_pos_path_map1.y),COLORBLACK,1);
        }
    }

    //画点 road_network_list
    QList<MapSearchNode *>::iterator node_ptr3 = road_network_list.begin();
    for(; node_ptr3!=road_network_list.end() ;node_ptr3++) {
        state_struct tmp_state_path_map;
        tmp_state_path_map.position.x=(*node_ptr3)->x;
        tmp_state_path_map.position.y=(*node_ptr3)->y;

        pos_int tmp_pos_path_map;
        tmp_pos_path_map.x=int((tmp_state_path_map.position.x-delta_origin_pos.x)*global_env_zoom);
        tmp_pos_path_map.y=int(MAP_HEIGHT_CELL-(tmp_state_path_map.position.y-delta_origin_pos.y)*global_env_zoom);

        //if(!(*node_ptr)->intersection)
        //    cvCircle(m_EnvImage_Global_diaplay,cvPoint(tmp_pos_path_map.x,tmp_pos_path_map.y),2,COLORWHITE,-1);//道路
        //else
        //    cvCircle(m_EnvImage_Global_diaplay,cvPoint(tmp_pos_path_map.x,tmp_pos_path_map.y),4,COLORGREEN,-1);//路口

        if(!(*node_ptr3)->intersection)
        {
            if((*node_ptr3)->node_id<100)       //if task nodes
                cvCircle(m_EnvImage_Global_diaplay,cvPoint(tmp_pos_path_map.x,tmp_pos_path_map.y),3,COLORRED,-1);//task nodes
            else if((*node_ptr3)->node_id == fallback_node_id){
                cvCircle(m_EnvImage_Global_diaplay,cvPoint(tmp_pos_path_map.x,tmp_pos_path_map.y),3,COLORVIOLET,-1);
//            }else if((*node_ptr3)->node_id == last_last_to_intersec_id){
//                cvCircle(m_EnvImage_Global_diaplay,cvPoint(tmp_pos_path_map.x,tmp_pos_path_map.y),3,COLORMEAT,-1);
//            }else if((*node_ptr3)->node_id == last_node_id_to_last_intersection){
//                cvCircle(m_EnvImage_Global_diaplay,cvPoint(tmp_pos_path_map.x,tmp_pos_path_map.y),3,COLORMEAT,-1);
            }else{
                cvCircle(m_EnvImage_Global_diaplay,cvPoint(tmp_pos_path_map.x,tmp_pos_path_map.y),2,COLORBLACK,-1);//道路
            }
        }
        else
            cvCircle(m_EnvImage_Global_diaplay,cvPoint(tmp_pos_path_map.x,tmp_pos_path_map.y),3,COLORGREEN,-1);//路口
        /*char szChar[256];//测试路网文件完整性用
        CvFont font;
        cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.4f, 0.4f, 0, 1, 8);
        sprintf_s(szChar, "%d", node_ptr->node_id);
        cvPutText(m_EnvImage_Global_diaplay, szChar, cvPoint(tmp_pos_path_map.x,tmp_pos_path_map.y), &font, COLORRED);*/
    }

    //画连接子父节点的线段 road_network_list
    QList<MapSearchNode *>::iterator node_ptr4=road_network_list.begin();
    for(; node_ptr4!=road_network_list.end() ;node_ptr4++) {
        state_struct tmp_state_path_map;
        tmp_state_path_map.position.x=(*node_ptr4)->x;
        tmp_state_path_map.position.y=(*node_ptr4)->y;
        pos_int tmp_pos_path_map;
        tmp_pos_path_map.x=int((tmp_state_path_map.position.x-delta_origin_pos.x)*global_env_zoom);
        tmp_pos_path_map.y=int(MAP_HEIGHT_CELL-(tmp_state_path_map.position.y-delta_origin_pos.y)*global_env_zoom);

        /*for(POSITION j=node_ptr->successorList.GetHeadPosition();j!=NULL;)
        {
            MapSearchNode *succnode_ptr=(MapSearchNode*)node_ptr->successorList.GetNext(j);*/
        QList<MapSearchNode *>::iterator succnode_ptr =(*node_ptr4)->successorList.begin();
        for(; succnode_ptr!=(*node_ptr4)->successorList.end(); succnode_ptr++)
        {
            state_struct tmp_state_path_map1;
            tmp_state_path_map1.position.x=(*succnode_ptr)->x;
            tmp_state_path_map1.position.y=(*succnode_ptr)->y;
            pos_int tmp_pos_path_map1;
            tmp_pos_path_map1.x=int((tmp_state_path_map1.position.x-delta_origin_pos.x)*global_env_zoom);
            tmp_pos_path_map1.y=int(MAP_HEIGHT_CELL-(tmp_state_path_map1.position.y-delta_origin_pos.y)*global_env_zoom);
            cvLine(m_EnvImage_Global_diaplay,cvPoint(tmp_pos_path_map.x,tmp_pos_path_map.y),
                   cvPoint(tmp_pos_path_map1.x,tmp_pos_path_map1.y),COLORYELLOW2,1);
        }
    }

    //画点 intersec_list
    QList<MapSearchNode *>::iterator node_ptr5 = intersec_list.begin();       //I add
    for(; node_ptr5!=intersec_list.end() ;node_ptr5++) {
        if((*node_ptr5)->node_id == last_intersec_id){
            QList<MapSearchNode*>::iterator intersec_suc_iter = (*node_ptr5)->successorList.begin();
            for(; intersec_suc_iter != (*node_ptr5)->successorList.end(); intersec_suc_iter++){
                state_struct tmp_state_path_map;
                tmp_state_path_map.position.x=(*intersec_suc_iter)->x;       //notice: from float to double
                tmp_state_path_map.position.y=(*intersec_suc_iter)->y;

                pos_int tmp_pos_path_map;
                tmp_pos_path_map.x=int((tmp_state_path_map.position.x-delta_origin_pos.x)*global_env_zoom);
                tmp_pos_path_map.y=int(MAP_HEIGHT_CELL-(tmp_state_path_map.position.y-delta_origin_pos.y)*global_env_zoom);
                if(!((*intersec_suc_iter)->bConnection)){
                    cvCircle(m_EnvImage_Global_diaplay,cvPoint(tmp_pos_path_map.x,tmp_pos_path_map.y),2,COLORYELLOW,-1);
                }else{
                    cvCircle(m_EnvImage_Global_diaplay,cvPoint(tmp_pos_path_map.x,tmp_pos_path_map.y),2,COLORGREY,-1);
                }
            }
        }
        if((*node_ptr5)->node_id == first_plan_intersec_id){
            QList<MapSearchNode*>::iterator intersec_suc_iter = (*node_ptr5)->successorList.begin();
            for(; intersec_suc_iter != (*node_ptr5)->successorList.end(); intersec_suc_iter++){
                state_struct tmp_state_path_map;
                tmp_state_path_map.position.x=(*intersec_suc_iter)->x;       //notice: from float to double
                tmp_state_path_map.position.y=(*intersec_suc_iter)->y;

                pos_int tmp_pos_path_map;
                tmp_pos_path_map.x=int((tmp_state_path_map.position.x-delta_origin_pos.x)*global_env_zoom);
                tmp_pos_path_map.y=int(MAP_HEIGHT_CELL-(tmp_state_path_map.position.y-delta_origin_pos.y)*global_env_zoom);
                if(!((*intersec_suc_iter)->bConnection)){
                    cvCircle(m_EnvImage_Global_diaplay,cvPoint(tmp_pos_path_map.x,tmp_pos_path_map.y),3,COLORYELLOW,-1);
                }else{
                    cvCircle(m_EnvImage_Global_diaplay,cvPoint(tmp_pos_path_map.x,tmp_pos_path_map.y),3,COLORGREY,-1);
                }
            }
        }
    }

    //给plan_task_list画点
    QList<Task_Node>::iterator node_ptr6 = plan_task_list.begin();
    for(; node_ptr6 != plan_task_list.end(); node_ptr6++) {
        state_struct tmp_state_path_map;
        tmp_state_path_map.position.x = node_ptr6->x;
        tmp_state_path_map.position.y = node_ptr6->y;

        pos_int tmp_pos_path_map;
        tmp_pos_path_map.x=int((tmp_state_path_map.position.x-delta_origin_pos.x)*global_env_zoom);
        tmp_pos_path_map.y=int(MAP_HEIGHT_CELL-(tmp_state_path_map.position.y-delta_origin_pos.y)*global_env_zoom);
        cvCircle(m_EnvImage_Global_diaplay,cvPoint(tmp_pos_path_map.x,tmp_pos_path_map.y),4,COLORRED,-1);
        CvFont font;
        char szChar[100];
        sprintf(szChar, "%d", node_ptr6->Task_num);
        cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.4f, 0.4f, 0, 1, 8);
        cvPutText(m_EnvImage_Global_diaplay, szChar, cvPoint(tmp_pos_path_map.x,tmp_pos_path_map.y), &font, COLORYELLOW);
    }

    CvFont font;
    cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.4f, 0.4f, 0, 1, 8);
    //cvCircle(m_EnvImage_Global_diaplay,cvPoint(15,13),3,COLORPINK,-1);
    //cvCircle(m_EnvImage_Global_diaplay,cvPoint(15,28),3,COLORVIOLET,-1);
    //cvCircle(m_EnvImage_Global_diaplay,cvPoint(15,43),3,COLORYELLOW,-1);
    cvCircle(m_EnvImage_Global_diaplay,cvPoint(15,58),3,COLORGREEN,-1);
    cvCircle(m_EnvImage_Global_diaplay,cvPoint(15,73),3,COLORRED,-1);

    cvCircle(m_EnvImage_Global_diaplay,cvPoint(5,88),2,COLORBLACK,-1);
    cvCircle(m_EnvImage_Global_diaplay,cvPoint(15,88),2,COLORBLACK,-1);
    cvLine(m_EnvImage_Global_diaplay,cvPoint(5,88),cvPoint(15,88),COLORYELLOW2,1);
    cvCircle(m_EnvImage_Global_diaplay,cvPoint(5,103),2,COLORWHITE,-1);
    cvCircle(m_EnvImage_Global_diaplay,cvPoint(15,103),2,COLORWHITE,-1);
    cvLine(m_EnvImage_Global_diaplay,cvPoint(5,103),cvPoint(15,103),COLORBLUE,1);
    cvCircle(m_EnvImage_Global_diaplay,cvPoint(5,118),2,COLORBLACK,-1);
    cvCircle(m_EnvImage_Global_diaplay,cvPoint(15,118),2,COLORBLACK,-1);
    cvLine(m_EnvImage_Global_diaplay,cvPoint(5,118),cvPoint(15,118),COLORWHITE,1);
    //sprintf(szChar1, "the second planning's connect_node");
    char vel[100];
    char gear[100];
    sprintf(vel, "velocity(m/s): %f", vehicle_vel);
    sprintf(gear, "gear: %d", gear_position);
    cvPutText(m_EnvImage_Global_diaplay, vel, cvPoint(20,15), &font, COLORBLACK);
    cvPutText(m_EnvImage_Global_diaplay, gear, cvPoint(20,30), &font, COLORBLACK);
    //cvPutText(m_EnvImage_Global_diaplay, "the lead points",
    //         cvPoint(20,45), &font, COLORWHITE);
    cvPutText(m_EnvImage_Global_diaplay, "intersection",
              cvPoint(20,60), &font, COLORBLACK);
    cvPutText(m_EnvImage_Global_diaplay, "task point",
              cvPoint(20,75), &font, COLORBLACK);
    cvPutText(m_EnvImage_Global_diaplay, "real-time map",
              cvPoint(20,90), &font, COLORBLACK);
    cvPutText(m_EnvImage_Global_diaplay, "the result of path-planning",
              cvPoint(20,105), &font, COLORBLACK);
    cvPutText(m_EnvImage_Global_diaplay, "prior map",
              cvPoint(20,120), &font, COLORBLACK);
    cvShowImage("global env",m_EnvImage_Global_diaplay);
}

void AttachXmlFile::ShowPathWithPathList() {
    //ROS_INFO("DISPLAY ......");
    ShowRoadWithNodeList();
    //copy the path before drawing the line, Because after executing this step,
    //the planning result path will be cleared,and the BUG has not been found yet.
    /*QList<MapSearchNode*>::iterator CopyIter=m_path_map_list.begin();
    for(; CopyIter!=m_path_map_list.end(); CopyIter++)
    {
        MapSearchNode tmpMapNode;
        tmpMapNode.intersection=(*CopyIter)->intersection;
        tmpMapNode.lat=(*CopyIter)->lat;
        tmpMapNode.lon=(*CopyIter)->lon;
        tmpMapNode.x=(*CopyIter)->x;
        tmpMapNode.y=(*CopyIter)->y;
        tmpMapNode.road_id=(*CopyIter)->road_id;

        qDebug()<<"tmpmapnode:"<<tmpMapNode.road_id<<endl;
        qDebug()<<"pathnode:"<<(*CopyIter)->road_id;
        m_RePlan_path_list.append(tmpMapNode);
    }*/

    //画点 m_astarsearch.NodeList
    QList<MapSearchNode *>::iterator node_ptr=original_road_network_list.begin();
    for(; node_ptr!=original_road_network_list.end() ;node_ptr++)
    {
        state_struct tmp_state_path_map;
        tmp_state_path_map.position.x=(*node_ptr)->x;       //notice: from float to double
        tmp_state_path_map.position.y=(*node_ptr)->y;

        pos_int tmp_pos_path_map;
        tmp_pos_path_map.x=int((tmp_state_path_map.position.x-delta_origin_pos.x)*global_env_zoom);
        tmp_pos_path_map.y=int(MAP_HEIGHT_CELL-(tmp_state_path_map.position.y-delta_origin_pos.y)*global_env_zoom);
        if((*node_ptr)->intersection) {
            cvCircle(m_EnvImage_Global_diaplay,cvPoint(tmp_pos_path_map.x,tmp_pos_path_map.y),3,COLORGREEN,-1);//路口
        } else {
            if((*node_ptr)->type >= 0)       // task nodes
                cvCircle(m_EnvImage_Global_diaplay,cvPoint(tmp_pos_path_map.x,tmp_pos_path_map.y),3,COLORBLACK,-1);//task nodes
            else
                cvCircle(m_EnvImage_Global_diaplay,cvPoint(tmp_pos_path_map.x,tmp_pos_path_map.y),1,COLORBLACK,-1);//道路
        }
        /*switch((*node_ptr)->type){
            case 0:
                cvCircle(m_EnvImage_Global_diaplay,cvPoint(tmp_pos_path_map.x,tmp_pos_path_map.y),3,COLORRED,-1);
                break;
            case 1:
                cvCircle(m_EnvImage_Global_diaplay,cvPoint(tmp_pos_path_map.x,tmp_pos_path_map.y),3,COLORRED,-1);
                break;
            case 3:
                cvCircle(m_EnvImage_Global_diaplay,cvPoint(tmp_pos_path_map.x,tmp_pos_path_map.y),3,COLORGREEN,-1);
                break;
            case 4:
                cvCircle(m_EnvImage_Global_diaplay,cvPoint(tmp_pos_path_map.x,tmp_pos_path_map.y),4,COLORWHITE2,-1);
                break;
            default:
                cvCircle(m_EnvImage_Global_diaplay,cvPoint(tmp_pos_path_map.x,tmp_pos_path_map.y),2,COLORWHITE,-1);
                break;
        }*/
        if((*node_ptr)->node_id == connect_task_id){
            cvCircle(m_EnvImage_Global_diaplay,cvPoint(tmp_pos_path_map.x,tmp_pos_path_map.y),3,COLORPINK,-1);
        }
        /*if( m_ptempnode != NULL && (*node_ptr)->node_id == m_ptempnode->node_id){
            cvCircle(m_EnvImage_Global_diaplay,cvPoint(tmp_pos_path_map.x,tmp_pos_path_map.y),5,COLORGREEN,-1);
            cvCircle(m_EnvImage_Global_diaplay,cvPoint(tmp_pos_path_map.x,tmp_pos_path_map.y),3,COLORYELLOW,-1);
        }
        if( m_pnextnode != NULL && (*node_ptr)->node_id == m_pnextnode->node_id){
            cvCircle(m_EnvImage_Global_diaplay,cvPoint(tmp_pos_path_map.x,tmp_pos_path_map.y),5,COLORGREEN,-1);
            cvCircle(m_EnvImage_Global_diaplay,cvPoint(tmp_pos_path_map.x,tmp_pos_path_map.y),3,COLORYELLOW,-1);
        }
        if( m_pthirdnode != NULL && (*node_ptr)->node_id == m_pthirdnode->node_id){
            cvCircle(m_EnvImage_Global_diaplay,cvPoint(tmp_pos_path_map.x,tmp_pos_path_map.y),5,COLORGREEN,-1);
            cvCircle(m_EnvImage_Global_diaplay,cvPoint(tmp_pos_path_map.x,tmp_pos_path_map.y),3,COLORRED,-1);
        }*/
    }

    //给规划结果画点
    if(m_path_map_list.size() < 3){
        QList<MapSearchNode *>::iterator node_ptr = m_path_map_list.begin();
        for(; node_ptr != m_path_map_list.end(); node_ptr++)
        {
            state_struct tmp_state_path_map;
            tmp_state_path_map.position.x=(*node_ptr)->x;
            tmp_state_path_map.position.y=(*node_ptr)->y;

            pos_int tmp_pos_path_map;
            tmp_pos_path_map.x=int((tmp_state_path_map.position.x-delta_origin_pos.x)*global_env_zoom);
            tmp_pos_path_map.y=int(MAP_HEIGHT_CELL-(tmp_state_path_map.position.y-delta_origin_pos.y)*global_env_zoom);

            cvCircle(m_EnvImage_Global_diaplay,cvPoint(tmp_pos_path_map.x,tmp_pos_path_map.y),3,COLORBLUE,-1);
        }
    }

    //给路径规划结果画线
    QList<MapSearchNode*>::iterator first_node_ptr = m_path_map_list.begin();
    if(first_node_ptr != m_path_map_list.end()){
        QList<MapSearchNode*>::iterator sec_node_ptr = first_node_ptr+1;
        for(; sec_node_ptr != m_path_map_list.end(); sec_node_ptr++) {
            state_struct tmp_node_state_first;
            pos_int tmp_node_pos_first;
            tmp_node_state_first.position.x=(*first_node_ptr)->x;
            tmp_node_state_first.position.y=(*first_node_ptr)->y;
            tmp_node_pos_first.x=int((tmp_node_state_first.position.x-delta_origin_pos.x)*global_env_zoom);
            tmp_node_pos_first.y=int(MAP_HEIGHT_CELL-(tmp_node_state_first.position.y-delta_origin_pos.y)*global_env_zoom);

            state_struct tmp_node_state_sec;
            pos_int tmp_node_pos_sec;
            tmp_node_state_sec.position.x=(*sec_node_ptr)->x;
            tmp_node_state_sec.position.y=(*sec_node_ptr)->y;
            tmp_node_pos_sec.x=int((tmp_node_state_sec.position.x-delta_origin_pos.x)*global_env_zoom);
            tmp_node_pos_sec.y=int(MAP_HEIGHT_CELL-(tmp_node_state_sec.position.y-delta_origin_pos.y)*global_env_zoom);
            cvLine(m_EnvImage_Global_diaplay,
                   cvPoint(tmp_node_pos_first.x,tmp_node_pos_first.y),
                   cvPoint(tmp_node_pos_sec.x,tmp_node_pos_sec.y),COLORBLUE,3);//规划路径

            first_node_ptr = sec_node_ptr;
        }
    }

    //给发布道路划线
    QList<MapSearchNode*>::iterator pub_first_ptr = pub_way_display.begin();
    if(pub_first_ptr != pub_way_display.end()){
        QList<MapSearchNode*>::iterator pub_second_ptr = pub_first_ptr+1;
        for(; pub_second_ptr != pub_way_display.end(); pub_second_ptr++)
        {
            state_struct tmp_node_state_first;
            pos_int tmp_node_pos_first;
            tmp_node_state_first.position.x=(*pub_first_ptr)->x;
            tmp_node_state_first.position.y=(*pub_first_ptr)->y;
            tmp_node_pos_first.x=int((tmp_node_state_first.position.x-delta_origin_pos.x)*global_env_zoom);
            tmp_node_pos_first.y=int(MAP_HEIGHT_CELL-(tmp_node_state_first.position.y-delta_origin_pos.y)*global_env_zoom);

            state_struct tmp_node_state_sec;
            pos_int tmp_node_pos_sec;
            tmp_node_state_sec.position.x=(*pub_second_ptr)->x;
            tmp_node_state_sec.position.y=(*pub_second_ptr)->y;
            tmp_node_pos_sec.x=int((tmp_node_state_sec.position.x-delta_origin_pos.x)*global_env_zoom);
            tmp_node_pos_sec.y=int(MAP_HEIGHT_CELL-(tmp_node_state_sec.position.y-delta_origin_pos.y)*global_env_zoom);
            cvLine(m_EnvImage_Global_diaplay,
                   cvPoint(tmp_node_pos_first.x,tmp_node_pos_first.y),
                   cvPoint(tmp_node_pos_sec.x,tmp_node_pos_sec.y),COLORGREEN,3);//规划路径

            pub_first_ptr = pub_second_ptr;
        }
    }

    //给插值后的全局路径画点
    if(!interpolation_way.points.empty()){
        std::size_t control_point_num = interpolation_way.points.size();
        //ROS_WARN("interpolation_way size: %d",(int)control_point_num);
        double sum = 0.0;
        for (std::size_t i = 0; i < control_point_num; i++) {
            if(i >= 1){
                sum += distance(interpolation_way.points[i-1].point.x,
                                interpolation_way.points[i-1].point.y,
                                interpolation_way.points[i].point.x,
                                interpolation_way.points[i].point.y);
            }
            pos_int tmp_pos_path_map;
            tmp_pos_path_map.x=int((interpolation_way.points.at(i).point.x-delta_origin_pos.x)*global_env_zoom);
            tmp_pos_path_map.y=int(MAP_HEIGHT_CELL-(interpolation_way.points.at(i).point.y-delta_origin_pos.y)*global_env_zoom);
            if(sum < 100.0){
                cvCircle(m_EnvImage_Global_diaplay,cvPoint(tmp_pos_path_map.x,tmp_pos_path_map.y),5,COLORBLUE,-1);
            }else{
                cvCircle(m_EnvImage_Global_diaplay,cvPoint(tmp_pos_path_map.x,tmp_pos_path_map.y),3,COLORBLUE,-1);
            }
        }
    }
    //给插值后的发布路径画点
    if(!way_msgs.points.empty()){
        std::size_t control_point_num = way_msgs.points.size();
        for (std::size_t i = 0; i < control_point_num; i++) {
            pos_int tmp_pos_path_map;
            tmp_pos_path_map.x=int((way_msgs.points.at(i).point.x-delta_origin_pos.x)*global_env_zoom);
            tmp_pos_path_map.y=int(MAP_HEIGHT_CELL-(way_msgs.points.at(i).point.y-delta_origin_pos.y)*global_env_zoom);
            cvCircle(m_EnvImage_Global_diaplay,cvPoint(tmp_pos_path_map.x,tmp_pos_path_map.y),4,COLORDARKGREEN,-1);
        }
    }
    //画转折点
    QList<MapSearchNode *>::iterator node_ptr2=original_road_network_list.begin();
    for(; node_ptr2!=original_road_network_list.end() ;node_ptr2++) {
        state_struct tmp_state_path_map;
        tmp_state_path_map.position.x = (*node_ptr2)->x;       //notice: from float to double
        tmp_state_path_map.position.y = (*node_ptr2)->y;

        pos_int tmp_pos_path_map;
        tmp_pos_path_map.x = int((tmp_state_path_map.position.x - delta_origin_pos.x) * global_env_zoom);
        tmp_pos_path_map.y = int(
                MAP_HEIGHT_CELL - (tmp_state_path_map.position.y - delta_origin_pos.y) * global_env_zoom);
        if((*node_ptr2)->node_id == switch_point_node){
            cvCircle(m_EnvImage_Global_diaplay,cvPoint(tmp_pos_path_map.x,tmp_pos_path_map.y),8,COLORPINK,-1);
        }
    }

    //画当前匹配路段与候选匹配路段
    QList<MapSearchNode*>::iterator pNode = original_road_network_list.begin();
    for(; pNode != original_road_network_list.end() ;pNode++) {
        state_struct tmp_state_path_map;
        tmp_state_path_map.position.x=(*pNode)->x;
        tmp_state_path_map.position.y=(*pNode)->y;
        pos_int tmp_pos_path_map;
        tmp_pos_path_map.x=int((tmp_state_path_map.position.x-delta_origin_pos.x)*global_env_zoom);
        tmp_pos_path_map.y=int(MAP_HEIGHT_CELL-(tmp_state_path_map.position.y-delta_origin_pos.y)*global_env_zoom);
        if( m_ptempnode != NULL && (*pNode)->node_id == m_ptempnode->node_id){
            cvCircle(m_EnvImage_Global_diaplay,cvPoint(tmp_pos_path_map.x,tmp_pos_path_map.y),5,COLORGREEN,-1);
            cvCircle(m_EnvImage_Global_diaplay,cvPoint(tmp_pos_path_map.x,tmp_pos_path_map.y),3,COLORYELLOW,-1);
        }
        if( m_pnextnode != NULL && (*pNode)->node_id == m_pnextnode->node_id){
            cvCircle(m_EnvImage_Global_diaplay,cvPoint(tmp_pos_path_map.x,tmp_pos_path_map.y),5,COLORGREEN,-1);
            cvCircle(m_EnvImage_Global_diaplay,cvPoint(tmp_pos_path_map.x,tmp_pos_path_map.y),3,COLORYELLOW,-1);
        }
        if( m_pthirdnode != NULL && (*pNode)->node_id == m_pthirdnode->node_id){
            cvCircle(m_EnvImage_Global_diaplay,cvPoint(tmp_pos_path_map.x,tmp_pos_path_map.y),5,COLORGREEN,-1);
            cvCircle(m_EnvImage_Global_diaplay,cvPoint(tmp_pos_path_map.x,tmp_pos_path_map.y),3,COLORRED,-1);
        }
    }

    //画任务点
    QList<Task_Node>::iterator task_ptr = m_cMapMatch.TaskList.begin();
    for(; task_ptr != m_cMapMatch.TaskList.end(); task_ptr++) {
        state_struct tmp_state_path_map;
        tmp_state_path_map.position.x = task_ptr->x;
        tmp_state_path_map.position.y = task_ptr->y;

        pos_int tmp_pos_path_map;
        tmp_pos_path_map.x=int((tmp_state_path_map.position.x-delta_origin_pos.x)*global_env_zoom);
        tmp_pos_path_map.y=int(MAP_HEIGHT_CELL-(tmp_state_path_map.position.y-delta_origin_pos.y)*global_env_zoom);

        if(m_pcurtask.Task_num != -1 && task_ptr->Task_num == m_pcurtask.Task_num){
            cvCircle(m_EnvImage_Global_diaplay,cvPoint(tmp_pos_path_map.x,tmp_pos_path_map.y),5,COLORYELLOW,-1);
            cvCircle(m_EnvImage_Global_diaplay,cvPoint(tmp_pos_path_map.x,tmp_pos_path_map.y),3,COLORGREEN,-1);
        } else {
            if(task_ptr->type == 3){
                cvCircle(m_EnvImage_Global_diaplay,cvPoint(tmp_pos_path_map.x,tmp_pos_path_map.y),5,COLORYELLOW,-1);
            }else if(task_ptr->type != 2){
                cvCircle(m_EnvImage_Global_diaplay,cvPoint(tmp_pos_path_map.x,tmp_pos_path_map.y),4,COLORYELLOW,-1);
            } else {
                cvCircle(m_EnvImage_Global_diaplay,cvPoint(tmp_pos_path_map.x,tmp_pos_path_map.y),4,COLORRED,-1);
            }
        }
        CvFont font;
        char szChar[100];
        if(m_pcurtask.Task_num != -1 && task_ptr->Task_num == m_pcurtask.Task_num){
            sprintf(szChar, "%d(%d)Match", task_ptr->Task_num,task_ptr->type);
        }else if (task_ptr->type != 2 ){
            sprintf(szChar, "%d(%d)", task_ptr->Task_num,task_ptr->type);
        }else{
            sprintf(szChar, "%d", task_ptr->Task_num);
        }
        cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.4f, 0.4f, 0, 1, 8);
        if(m_pcurtask.Task_num != -1 && task_ptr->Task_num == m_pcurtask.Task_num){
            cvPutText(m_EnvImage_Global_diaplay, szChar, cvPoint(tmp_pos_path_map.x,tmp_pos_path_map.y), &font, COLORRED);
        }else{
            cvPutText(m_EnvImage_Global_diaplay, szChar, cvPoint(tmp_pos_path_map.x,tmp_pos_path_map.y), &font, COLORRED);
        }
    }

    if(vehicle_state.m_prevstate!=NULL)
    {
        state_struct tmp_node_state_first;      //notice：以下vehicle_state中的参数进行赋值
        pos_int tmp_node_pos_first;
        state_struct tmp_node_state_sec;
        pos_int tmp_node_pos_sec;

        //在上一状态与前方状态之间画线
        tmp_node_state_first.position.x=vehicle_state.m_prevstate->x;
        tmp_node_state_first.position.y=vehicle_state.m_prevstate->y;
        tmp_node_pos_first.x=int((tmp_node_state_first.position.x-delta_origin_pos.x)*global_env_zoom);
        tmp_node_pos_first.y=int(MAP_HEIGHT_CELL-(tmp_node_state_first.position.y-delta_origin_pos.y)*global_env_zoom);

        tmp_node_state_sec.position.x=vehicle_state.m_nextstate->x;
        tmp_node_state_sec.position.y=vehicle_state.m_nextstate->y;
        tmp_node_pos_sec.x=int((tmp_node_state_sec.position.x-delta_origin_pos.x)*global_env_zoom);
        tmp_node_pos_sec.y=int(MAP_HEIGHT_CELL-(tmp_node_state_sec.position.y-delta_origin_pos.y)*global_env_zoom);

        cvLine(m_EnvImage_Global_diaplay,cvPoint(tmp_node_pos_first.x,tmp_node_pos_first.y),cvPoint(tmp_node_pos_sec.x,tmp_node_pos_sec.y),COLORCYAN,4);//当前所属路段

        //显示dis to intersection，汽车当前状态到前方交叉路口的距离
        char szChar[256];
        CvFont font;
        cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.4f, 0.4f, 0, 1, 8);
        sprintf(szChar, "dis to intersection %.1f", vehicle_state.dis_to_nextintersection);   //notice，sprintf_s
        cvPutText(m_EnvImage_Global_diaplay, szChar, cvPoint(10,10), &font, COLORRED);

        //显示当前车辆位置
        tmp_node_state_first.position.x=vehicle_state.x;
        tmp_node_state_first.position.y=vehicle_state.y;
        tmp_node_pos_first.x=int((tmp_node_state_first.position.x-delta_origin_pos.x)*global_env_zoom);
        tmp_node_pos_first.y=int(MAP_HEIGHT_CELL-(tmp_node_state_first.position.y-delta_origin_pos.y)*global_env_zoom);
        cvCircle(m_EnvImage_Global_diaplay,cvPoint(tmp_node_pos_first.x,tmp_node_pos_first.y),2,COLORRED,-1);
        sprintf(szChar, "%.1f %.1f", vehicle_state.x,vehicle_state.y);        //notice:sprintf_s
        cvPutText(m_EnvImage_Global_diaplay, szChar, cvPoint(tmp_node_pos_first.x,tmp_node_pos_first.y), &font, COLORRED);

        //这个好像也是显示当前车辆的位置，这个和上面的区别是输入源不一样
        tmp_node_state_first.position.x=vehicle_state_on_roadmap.position.x;
        tmp_node_state_first.position.y=vehicle_state_on_roadmap.position.y;
        tmp_node_pos_first.x=int((tmp_node_state_first.position.x-delta_origin_pos.x)*global_env_zoom);
        tmp_node_pos_first.y=int(MAP_HEIGHT_CELL-(tmp_node_state_first.position.y-delta_origin_pos.y)*global_env_zoom);
        cvCircle(m_EnvImage_Global_diaplay,cvPoint(tmp_node_pos_first.x,tmp_node_pos_first.y),2,COLORRED,-1);
        sprintf(szChar, "%.1f %.1f", vehicle_state_on_roadmap.position.x,vehicle_state_on_roadmap.position.y);  //notice:sprintf_s
        cvPutText(m_EnvImage_Global_diaplay, szChar, cvPoint(tmp_node_pos_first.x,tmp_node_pos_first.y), &font, COLORRED);
    }
    cvShowImage("global env",m_EnvImage_Global_diaplay);
    //ROS_INFO("DISPLAY DONE");
}

void AttachXmlFile::on_mouse_globalenv(int event, int x, int y, int flags, void* param) {
    AttachXmlFile* m_Parent=(AttachXmlFile*) param;
    switch (event) {
        case CV_EVENT_MOUSEMOVE:
        {
            if((flags & CV_EVENT_FLAG_CTRLKEY)==0)
            {
                //if(flags & CV_EVENT_FLAG_LBUTTON && (flags & CV_EVENT_FLAG_CTRLKEY) ==0  && (flags & CV_EVENT_FLAG_SHIFTKEY) ==0)
                if(flags & CV_EVENT_FLAG_MBUTTON && (flags & CV_EVENT_FLAG_CTRLKEY) ==0  && (flags & CV_EVENT_FLAG_SHIFTKEY) ==0)
                {
                    if(m_Parent->prev_mouse_pos.x!=0 && m_Parent->prev_mouse_pos.y!=0)
                    {
                        //移动地图
                        pos_int tmp_delta_pos;
                        tmp_delta_pos.x=x-m_Parent->prev_mouse_pos.x;
                        tmp_delta_pos.y=y-m_Parent->prev_mouse_pos.y;

                        m_Parent->delta_origin_pos.x=m_Parent->delta_origin_pos.x-tmp_delta_pos.x/m_Parent->global_env_zoom;
                        m_Parent->delta_origin_pos.y=m_Parent->delta_origin_pos.y+tmp_delta_pos.y/m_Parent->global_env_zoom;
                        //重新绘制地图
                        m_Parent->ShowPathWithPathList();
                    }
                    m_Parent->prev_mouse_pos.x=x;
                    m_Parent->prev_mouse_pos.y=y;
                }
                else
                {
                    m_Parent->prev_mouse_pos.x=0;
                    m_Parent->prev_mouse_pos.y=0;
                }
            }
        }
            break;

            //case CV_EVENT_LBUTTONUP:
        case CV_EVENT_RBUTTONUP:
        {
            if(flags & CV_EVENT_FLAG_SHIFTKEY)
            {
                //规划起点
                m_Parent->m_nstart_image.x = x;
                m_Parent->m_nstart_image.y = y;

                if(x>= 0 || x<MAP_WIDTH_CELL || y>=0 || y<MAP_HEIGHT_CELL)
                {
                    m_Parent->m_nstart_world.x=x/m_Parent->global_env_zoom+m_Parent->delta_origin_pos.x;
                    m_Parent->m_nstart_world.y=(MAP_HEIGHT_CELL-y)/m_Parent->global_env_zoom+m_Parent->delta_origin_pos.y;
                }
            } else if(flags & CV_EVENT_FLAG_CTRLKEY) {
                //测试
                /*SVehicleStateToNet m_sVehicleStateToNet;            //notice:这个定义在tssocket.h中进行定义，但其中包含大量win的库文件
                m_sVehicleStateToNet.fPos_east=x/m_Parent->global_env_zoom+m_Parent->delta_origin_pos.x;
                m_sVehicleStateToNet.fPos_north=(MAP_HEIGHT_CELL-y)/m_Parent->global_env_zoom+m_Parent->delta_origin_pos.y;
                //m_Parent->PositioningInRoadMap(m_sVehicleStateToNet);*/       //notice:add those code later
                if(!m_Parent->plan_task_list.isEmpty()){
                    m_Parent->plan_task_list.removeLast();
                    ROS_INFO("remove the last point of plan_task_list.");
                }
                m_Parent->ShowPathWithPathList();
            }
        }
            break;
            //case CV_EVENT_RBUTTONDOWN:
        case CV_EVENT_LBUTTONDOWN:
        {
            //缩放
            if((flags & CV_EVENT_FLAG_CTRLKEY)==0)
            {
                m_Parent->zoom_mouse_pos.x=x;
                m_Parent->zoom_mouse_pos.y=y;
            }
        }
            break;
            //case CV_EVENT_RBUTTONUP:
        case CV_EVENT_LBUTTONUP:
        {
            if(flags & CV_EVENT_FLAG_CTRLKEY && (flags & CV_EVENT_FLAG_SHIFTKEY) ==0)
            {
                /*//2018/8/12  tangbo
                //规划终点
                m_Parent->m_ngoal_image.x = x;
                m_Parent->m_ngoal_image.y = y;
                if(x>= 0 || x<MAP_WIDTH_CELL || y>=0 || y<MAP_HEIGHT_CELL) {
                    m_Parent->m_ngoal_world.x=x/m_Parent->global_env_zoom+m_Parent->delta_origin_pos.x;
                    m_Parent->m_ngoal_world.y=(MAP_HEIGHT_CELL-y)/m_Parent->global_env_zoom+m_Parent->delta_origin_pos.y;
                }
                m_Parent->m_nmiddle_world.x = 0;
                m_Parent->m_nmiddle_world.y = 0;
                m_Parent->RoadMapPathPlan();*/
                pos_int ntask_world;
                MapSearchNode* nearest_node = NULL;
                if(x>= 0 || x<MAP_WIDTH_CELL || y>=0 || y<MAP_HEIGHT_CELL) {
                    ntask_world.x=x/m_Parent->global_env_zoom+m_Parent->delta_origin_pos.x;
                    ntask_world.y=(MAP_HEIGHT_CELL-y)/m_Parent->global_env_zoom+m_Parent->delta_origin_pos.y;
                }
                int nearest_id = m_Parent->GetNodeidFromXY(&m_Parent->m_astarsearch.NodeList,ntask_world.x,ntask_world.y);
                QList<MapSearchNode*>::iterator iter = m_Parent->m_astarsearch.NodeList.begin();
                for(; iter != m_Parent->m_astarsearch.NodeList.end(); iter++){
                    if((*iter)->node_id == nearest_id){
                        nearest_node = *iter;
                    }
                }
                if(nearest_node != NULL){
                    Task_Node temp_task;
                    temp_task.x = nearest_node->x;
                    temp_task.y = nearest_node->y;
                    temp_task.lon = nearest_node->lon;
                    temp_task.lat = nearest_node->lat;
                    temp_task.type = 2;
                    temp_task.on_road= false;
                    temp_task.Task_num = m_Parent->task_id++;
                    temp_task.vlimit = nearest_node->vlimit;
                    temp_task.concave_obs_det = nearest_node->concave_obs_det;
                    temp_task.dynamic_obs_det = nearest_node->dynamic_obs_det;
                    temp_task.foogy_det = nearest_node->foogy_det;
                    temp_task.water_det = nearest_node->water_det;
                    temp_task.wall_area = nearest_node->wall_area;
                    temp_task.ditch_area = nearest_node->ditch_area;
                    m_Parent->plan_task_list.append(temp_task);
                }else{
                    ROS_WARN("on_mouse_globalenv: nearest is NULL");
                }
                m_Parent->ShowPathWithPathList();
            } else if ((flags & CV_EVENT_FLAG_CTRLKEY) && (flags & CV_EVENT_FLAG_SHIFTKEY)) {    //加入搜索任务点
                pos_int ntask_world;
                if(x>= 0 || x<MAP_WIDTH_CELL || y>=0 || y<MAP_HEIGHT_CELL) {
                    ntask_world.x=x/m_Parent->global_env_zoom+m_Parent->delta_origin_pos.x;
                    ntask_world.y=(MAP_HEIGHT_CELL-y)/m_Parent->global_env_zoom+m_Parent->delta_origin_pos.y;
                }
                double min_dis = std::numeric_limits<double>::infinity();
                int min_id = -1;
                QList<Task_Node>::iterator task_iter = m_Parent->m_cMapMatch.TaskList.begin();
                for(; task_iter != m_Parent->m_cMapMatch.TaskList.end(); ++task_iter){
                    double temp_dis = m_Parent->distance(ntask_world.x,ntask_world.y,task_iter->x,task_iter->y);
                    if(temp_dis < min_dis){
                        min_dis = temp_dis;
                        min_id = task_iter->Task_num;
                        ROS_INFO("mid_id: %d , mid_dis: %lf" , task_iter->Task_num , min_dis);
                    }
                }
                Task_Node nearest_task;
                task_iter = m_Parent->m_cMapMatch.TaskList.begin();
                for(; task_iter != m_Parent->m_cMapMatch.TaskList.end(); ++task_iter){
                    if(task_iter->Task_num == min_id){
                        nearest_task = *task_iter;
                    }
                }
                if(min_id != -1){
                    Task_Node temp_task;
                    temp_task.x = nearest_task.x;
                    temp_task.y = nearest_task.y;
                    temp_task.lon = nearest_task.lon;
                    temp_task.lat = nearest_task.lat;
                    temp_task.type = 3;
                    temp_task.on_road= false;
                    temp_task.Task_num = m_Parent->task_id++;
                    temp_task.vlimit = 4.0;
                    temp_task.concave_obs_det = true;
                    temp_task.dynamic_obs_det = true;
                    temp_task.foogy_det = true;
                    temp_task.water_det = true;
                    temp_task.wall_area = true;
                    temp_task.ditch_area = true;
                    m_Parent->plan_task_list.append(temp_task);
                }else{
                    ROS_WARN("on_mouse_globalenv: nearest task is NULL");
                }
                m_Parent->ShowPathWithPathList();
            } else {
                //缩放
                pos tmppos;//车身坐标系内的鼠标位置
                tmppos.x=m_Parent->zoom_mouse_pos.x/m_Parent->global_env_zoom+m_Parent->delta_origin_pos.x;
                tmppos.y=(MAP_HEIGHT_CELL-m_Parent->zoom_mouse_pos.y)/m_Parent->global_env_zoom+m_Parent->delta_origin_pos.y;

                if(y>m_Parent->zoom_mouse_pos.y) {  //放大
                    m_Parent->global_env_zoom=m_Parent->global_env_zoom*0.5;
                } else {    //缩小
                    m_Parent->global_env_zoom=m_Parent->global_env_zoom*2;
                }
                m_Parent->delta_origin_pos.x=int(tmppos.x-MAP_WIDTH_CELL/2/m_Parent->global_env_zoom);
                m_Parent->delta_origin_pos.y=int(tmppos.y-MAP_HEIGHT_CELL/2/m_Parent->global_env_zoom);
                m_Parent->ShowPathWithPathList();
            }
        }
            break;
        case CV_EVENT_MBUTTONUP:
        {
            if(flags & CV_EVENT_FLAG_CTRLKEY)
            {
                //规划终点
                m_Parent->m_nmiddle_image.x = x;
                m_Parent->m_nmiddle_image.y = y;

                if(x>= 0 || x<MAP_WIDTH_CELL || y>=0 || y<MAP_HEIGHT_CELL)
                {
                    m_Parent->m_nmiddle_world.x=x/m_Parent->global_env_zoom+m_Parent->delta_origin_pos.x;
                    m_Parent->m_nmiddle_world.y=(MAP_HEIGHT_CELL-y)/m_Parent->global_env_zoom+m_Parent->delta_origin_pos.y;
                }

                m_Parent->RoadMapPathPlan();
            }
        }
    }
}

void AttachXmlFile::RoadMapPathPlan() {

    //重新得到规划结果
    if(!m_path_map_list.isEmpty()) {
        m_path_map_list.clear();
    }

    if(!m_astarsearch.NodeList.isEmpty()) {
        DeleteAStarSearchNodeList();
    }

    GetNodeFromFile();

    InitDisplay();


    //int nstart_id = 276094637;//三环路
    //int nstart_id = 1554171696;//中关村南大街
    //int nstart_id = 1362305662;//bei_san_huan_xi_dong
    //int nstart_id = 1760770190;
    //int nstart_id =981019372 ;//西三环北路

    //tmp_state_path_map.position.x=node_ptr->x;
    //	tmp_state_path_map.position.y=node_ptr->y;
    //	tmp_pos_path_map.x=int((tmp_state_path_map.position.x-delta_origin_pos.x)*global_env_zoom);
    //	tmp_pos_path_map.y=int(MAP_HEIGHT_CELL-(tmp_state_path_map.position.y-delta_origin_pos.y)*global_env_zoom);

    int nstart_id=-1;
    int ngoal_id=-1;
    int nmiddle_id=-1;

    nstart_id = GetNodeidFromXY(&m_astarsearch.NodeList,m_fStartNodex,m_fStartNodey);
    ngoal_id = GetNodeidFromXY(&m_astarsearch.NodeList,m_ngoal_world.x,m_ngoal_world.y);
    nmiddle_id = GetNodeidFromXY(&m_astarsearch.NodeList,m_nmiddle_world.x,m_nmiddle_world.y);

    if(nmiddle_id!=-1 && nstart_id!=-1 && ngoal_id!=-1) {
        PathPlan(nstart_id,nmiddle_id,ngoal_id);
    }
    else if(nstart_id!=-1 && ngoal_id!=-1) {
        PathPlan(nstart_id,ngoal_id);
    }
    else {
        ROS_FATAL("In RoadMapPathPlan: path-planning fail!");
    }

    m_bPathPlaned = true;
    m_flag = 0;
}

int AttachXmlFile::GetNodeidFromXY(QList<MapSearchNode *> *NodeList, int x, int y){     //x和y暂定为单位米
    int bestid = -1;        //-1为无效id
    double mindisthreshold = 1000;//米

    MapSearchNode* bestnode = NULL;

    double mindis = 100000000;
    /*
    for(POSITION pos=NodeList->GetHeadPosition();pos!=NULL;)
    {
        MapSearchNode* node= (MapSearchNode*)NodeList->GetNext(pos);
        double dis=sqrt(pow((x-node->x),2) + pow((y-node->y),2));
        if(dis<mindis)
        {
            bestnode=node;
            mindis=dis;
        }
    }*/
    QList<MapSearchNode*>::iterator pos = NodeList->begin();
    for(; pos != NodeList->end() ; pos++)
    {
        MapSearchNode* node = (*pos);
        double dis = sqrt(pow((x-node->x),2) + pow((y-node->y),2));
        if(dis < mindis)
        {
            bestnode = node;
            mindis = dis;
        }
    }
    if(mindis < mindisthreshold && bestnode != NULL)
    {
        bestid = bestnode->node_id;
    }
    return bestid;
}

float AttachXmlFile::PointToTheta(float x1,float y1, float x2,float y2) {
    //lsh//通过两点坐标计算第一个点朝向（0~2*pi）
    float theta;
    if(x2 - x1==0)
    {
        if(y2 - y1>0)
            theta=CV_PI/2;
        else
            theta=CV_PI*3/2;
    }
    else if(x2-x1>0)
        theta=(atan((y2-y1)/(x2-x1)));//lsh//atan算出来的值在pi/2到-pi/2
    else
        theta=((CV_PI+atan((y2-y1)/(x2-x1))));

    if(theta<0) theta=CV_PI*2+theta;
    return theta;
}

int AttachXmlFile::GeneratePathInfo() {
    /*if(!m_path_map_list.isEmpty())
    {
        //先计算m_path_map_list中各点的THETA和弧长S
        POSITION pos = m_path_map_list.GetHeadPosition();
        MapSearchNode *first_node_ptr=(MapSearchNode*)m_path_map_list.GetNext(pos);
        first_node_ptr->s=0;
        while(pos)
        {
            MapSearchNode *second_node_ptr=(MapSearchNode*)m_path_map_list.GetNext(pos);
            first_node_ptr->theta=PointToTheta(first_node_ptr->x,first_node_ptr->y,second_node_ptr->x,second_node_ptr->y);
            second_node_ptr->s=first_node_ptr->s+sqrt(pow(first_node_ptr->y-second_node_ptr->y,2)+pow(first_node_ptr->x-second_node_ptr->x,2));
            second_node_ptr->theta=first_node_ptr->theta;
            first_node_ptr=second_node_ptr;
        }
        return 1;
    }
    else
    {
        return 0;
    }*/
    if(!m_path_map_list.isEmpty()) {
        //lsh//计算每个点的朝向theta和行驶距离s
        QList<MapSearchNode *>::iterator first_node_ptr,second_node_ptr;
        first_node_ptr = m_path_map_list.begin();
        second_node_ptr = first_node_ptr+1;
        (*first_node_ptr)->s = 0;
        for(; second_node_ptr != m_path_map_list.end(); second_node_ptr++)
        {
            (*first_node_ptr)->theta = PointToTheta((*first_node_ptr)->x,(*first_node_ptr)->y,(*second_node_ptr)->x,(*second_node_ptr)->y);
            (*second_node_ptr)->s = (*first_node_ptr)->s+sqrt(pow((*first_node_ptr)->y-(*second_node_ptr)->y,2)+pow((*first_node_ptr)->x-(*second_node_ptr)->x,2));
            (*second_node_ptr)->theta = (*first_node_ptr)->theta;//lll:这一句有什么用？
            first_node_ptr = second_node_ptr;
        }
        return 1;
    } else {
        return 0;
    }
}

void AttachXmlFile::Cal_Point_Dis_to_Line(float pointx, float pointy,
                                          float linex1, float liney1,
                                          float linex2, float liney2,
                                          float *dis, float *u) {
    float length=sqrt(pow(liney1-liney2,2)+pow(linex1-linex2,2));
    *dis=abs((linex2-linex1)*(liney1-pointy)-(linex1-pointx)*(liney2-liney1))/length;

    *u=((pointx-linex1)*(linex2-linex1)+(pointy-liney1)*(liney2-liney1))/(length*length);
}

void AttachXmlFile::DeleteAStarSearchNodeList() {

    /*m_Pathid_vector.clear();
    if(!m_astarsearch.NodeList.isEmpty())
    {
        MapSearchNode *deleteNode;
        //POSITION deletepos;
        while(!m_astarsearch.NodeList.isEmpty())
        {
            deleteNode = (MapSearchNode*)m_astarsearch.NodeList.RemoveTail();
            if(NULL != deleteNode )
            {
                delete deleteNode;
                deleteNode = NULL;
            }
        }
    }
    else
    {
        AfxMessageBox(_T("删除链表失败!"));
    }*/
    //再次加载xml文件，因此要对这个vector进行清空
    m_Pathid_vector.clear();
    if(!m_astarsearch.NodeList.isEmpty()) {
        qDeleteAll(m_astarsearch.NodeList);
        m_astarsearch.NodeList.clear();
    } else {
        ROS_FATAL("DeleteAStarSearchNodeList: fail to delete List!");
    }
}

void AttachXmlFile::DeleteParentNode(MapSearchNode *tempnode,int NodeID) {
    /*POSITION pos,deletepos,pos1;
    pos = tempnode ->parentList.GetHeadPosition();
    while(pos)
    {
        deletepos = pos;
        deleteparentnode = (MapSearchNode*)tempnode ->parentList.GetNext(pos);
        if(deleteparentnode ->node_id != NodeID)
        {
            pos1 = deleteparentnode ->successorList.GetHeadPosition();
            if(NULL != pos1)
            {
                deleteparentnode ->successorList.RemoveAll();
                pos1 = NULL;
            }
            pos1 = deleteparentnode->parentList.GetHeadPosition();
            if(NULL != pos1)
            {
                deleteparentnode->parentList.RemoveAll();
                pos1 = NULL;
            }
        }
        tempnode ->parentList.RemoveAt(deletepos);
    }*/
    MapSearchNode *deleteparentnode;
    QList<MapSearchNode *>::iterator pos,deletepos;
    pos = tempnode ->parentList.begin();
    //ROS_WARN("tempnode ->parentList.size111 is %d",tempnode ->parentList.size());
    for( ; pos != tempnode ->parentList.end() ; )
    {
        deletepos = pos;//lsh//父节点的地址
        deleteparentnode = *pos;//lsh//父节点
        if(deleteparentnode ->node_id != NodeID)
        {
            if(!deleteparentnode ->successorList.isEmpty()){
                deleteparentnode ->successorList.clear();
            }
            if(!deleteparentnode ->parentList.isEmpty()){
                deleteparentnode ->parentList.clear();
            }
        }
        pos = tempnode ->parentList.erase(deletepos);       //erase操作后，指针会自动指向下一个
    }
    //ROS_WARN("tempnode ->parentList.size222 is %d",tempnode ->parentList.size());
    //ROS_INFO("Delete parent relation between ( %d , %d )", tempnode->node_id , NodeID);
}

void AttachXmlFile::DeleteChildNode(MapSearchNode *tempnode,int NodeID) {
    /*
    POSITION pos,deletepos,pos1;
    pos = tempnode ->successorList.GetHeadPosition();
    while(pos)
    {
        deletepos = pos;
        deletechildnode = (MapSearchNode*)tempnode ->successorList.GetNext(pos);
        if(deletechildnode ->node_id != NodeID)
        {
            pos1 = deletechildnode ->successorList.GetHeadPosition();
            if(NULL != pos1)
            {
                deletechildnode ->successorList.RemoveAll();
                pos1 = NULL;
            }
            pos1 = deletechildnode->parentList.GetHeadPosition();
            if(NULL != pos1)
            {
                deletechildnode->parentList.RemoveAll();
                pos1 = NULL;
            }
        }
        tempnode ->successorList.RemoveAt(deletepos);
    }*/
    MapSearchNode *deletechildnode;
    QList<MapSearchNode *>::iterator pos,deletepos;
    pos = tempnode ->successorList.begin();
    //ROS_WARN("tempnode ->successorList.size111 is %d",tempnode ->successorList.size());
    for( ; pos != tempnode ->successorList.end() ; ) {
        deletepos = pos;//lsh//tempnode子节点的地址
        deletechildnode = *pos;//lsh//tempnode的子节点
        if(deletechildnode -> node_id != NodeID)
        {
            if(!deletechildnode ->successorList.isEmpty()){
                deletechildnode ->successorList.clear();
            }
            if(!deletechildnode ->parentList.isEmpty()){
                deletechildnode ->parentList.clear();
            }
        }//lsh//删除除重规划触发节点外其他子节点的所有父子节点
        pos = tempnode ->successorList.erase(deletepos);
        //lsh//删除tempnode每个子节点
        //erase操作后，指针会自动指向下一个
    }
    //ROS_WARN("tempnode ->successorList.size222 is %d",tempnode ->successorList.size());
    //ROS_INFO("Delete child relation between ( %d , %d )", tempnode->node_id , NodeID);
}

void AttachXmlFile::AddSolutionToAllPlanNodeList() {
    //lsh//将路径存入m_path_map_list
    MapSearchNode *tempnode;
    tempnode = m_astarsearch.GetSolutionStart();
    while(tempnode)
    {
        m_path_map_list.append(tempnode);
        tempnode = m_astarsearch.GetSolutionNext();
        if(NULL == tempnode)
        {
            break;
        }
    }

}

int AttachXmlFile::PathPlan(int nstartid,int nmiddleid,int ngoalid) {
    unsigned int nsearch_state_first;
    unsigned int nsearch_state_second;
    if(m_nStartLastNode != 0)//如果规划过，则有此点，删除节点间联系
    {
        /*POSITION pos2;
        MapSearchNode *tempnode4;
        pos2 = m_astarsearch.NodeList.GetHeadPosition();
        tempnode4 = (MapSearchNode*) m_astarsearch.NodeList.GetNext(pos2);
        while(pos2)//pos2是指向m_ptempnode的位置，删除它前面相邻一个节点的所有联系，除了m_ptempnode
        {
            if(tempnode4 ->node_id == m_nStartLastNode)
            {
                break;
            }
            if(!pos2)
            {
                AfxMessageBox(_T("没有找到相同节点"));
            }
            tempnode4 = (MapSearchNode*) m_astarsearch.NodeList.GetNext(pos2);
        }*/
        QList<MapSearchNode *>::iterator tempnode4= m_astarsearch.NodeList.begin();
        for(; tempnode4!= m_astarsearch.NodeList.end(); tempnode4++) {
            if((*tempnode4)->node_id == m_nStartLastNode) {
                break;
            }
            if((tempnode4+1) == m_astarsearch.NodeList.end()) {
                ROS_FATAL("In function PathPlan(int nstartid,int nmiddleid,int ngoalid):\n Don't find the same node.");
            }
        }
        DeleteChildNode((*tempnode4),nstartid);                 //删除此节点的子节点的所有联系
        DeleteParentNode((*tempnode4),nstartid);
    }

    MapSearchNode *pstart_node = GetNodeFormList(nstartid);      //这三行代码其实与上一级调用程序重复了
    MapSearchNode *pmiddle_node = GetNodeFormList(nmiddleid);
    MapSearchNode *pgoal_node = GetNodeFormList(ngoalid);
    if(NULL == pstart_node || NULL ==pmiddle_node|| NULL ==pgoal_node ) {
        ROS_FATAL("In function PathPlan(int nstartid,int nmiddleid,int ngoalid): No start, middle or target point found.");
        return 0;
    }

    //第一次规划
    m_astarsearch.SetStartAndGoalStates(pstart_node,pmiddle_node);
    do
    {
        nsearch_state_first = m_astarsearch.SearchStep();
    }
    while(nsearch_state_first == AStarSearch<MapSearchNode>::SEARCH_STATE_SEARCHING);
    if(nsearch_state_first == AStarSearch<MapSearchNode>::SEARCH_STATE_SUCCEEDED) {
        AddSolutionToAllPlanNodeList();
    } else if(nsearch_state_first == AStarSearch<MapSearchNode>::SEARCH_STATE_FAILED) {
        ROS_FATAL("In function PathPlan(int nstartid,int nmiddleid,int ngoalid): Intermediate point path planning failed.");
        return 0;
    }
    /*MapSearchNode *delete_node1;
    m_astarsearch.GetSolutionEnd();//
    delete_node1 = m_astarsearch.GetSolutionPrev();//得到上规划出来的倒数第二个节点

    DeleteChildNode(delete_node1,pmiddle_node ->node_id);//删除此节点的子节点的所有联系
    DeleteParentNode(delete_node1,pmiddle_node ->node_id);//删除此节点的父节点的所有联系*/

    //第二次规划
    m_astarsearch.SetStartAndGoalStates(pmiddle_node,pgoal_node);
    do {
        nsearch_state_second = m_astarsearch.SearchStep();
    }
    while(nsearch_state_second == AStarSearch<MapSearchNode>::SEARCH_STATE_SEARCHING);
    if(nsearch_state_second == AStarSearch<MapSearchNode>::SEARCH_STATE_SUCCEEDED) {
        MapSearchNode *tempnode7;
        m_astarsearch.GetSolutionStart();//第一个节点不加入到m_AllPlanNodeList中
        tempnode7 = m_astarsearch.GetSolutionNext();
        while(tempnode7) {
            m_path_map_list.append(tempnode7);
            tempnode7 = m_astarsearch.GetSolutionNext();
            if(NULL == tempnode7) {
                break;
            }
        }
    } else if(nsearch_state_second == AStarSearch<MapSearchNode>::SEARCH_STATE_FAILED) {
        ROS_FATAL("In function PathPlan(int nstartid,int nmiddleid,int ngoalid):\nThe last planning failed!");
        return 0;
    }

    if(!m_path_map_list.isEmpty()) {
        GeneratePathInfo();
    }
    ShowPathWithPathList();
    return 1;
}

int AttachXmlFile::PathPlan(int nstartid,int ngoalid) {
//    if(m_nStartLastNode != 0)//如果规划过，则有此点，删除节点间联系
//    {
//        /*POSITION pos2;
//        MapSearchNode *tempnode4;
//        pos2 = m_astarsearch.NodeList.GetHeadPosition();
//        tempnode4 = (MapSearchNode*) m_astarsearch.NodeList.GetNext(pos2);
//        while(pos2)
//        {
//        //pos2是指向m_ptempnode的位置，删除它前面相邻一个节点的所有联系，除了m_ptempnode
//
//            if(tempnode4 ->node_id == m_nStartLastNode)
//            {
//                break;
//            }
//            if(!pos2)
//            {
//                AfxMessageBox(_T("没有找到相同节点"));
//            }
//            tempnode4 = (MapSearchNode*) m_astarsearch.NodeList.GetNext(pos2);
//        }*/
//        QList<MapSearchNode *>::iterator tempnode4= m_astarsearch.NodeList.begin();
//        for(; tempnode4!= m_astarsearch.NodeList.end(); tempnode4++)
//        {
//            if((*tempnode4)->node_id == m_nStartLastNode)
//            {
//                break;
//            }
//            if(!(*tempnode4))
//            {
//                QMessageBox::information(NULL,"infomation","In function PathPlan:\n Don't find the same node.");
//            }
//        }//2018/4/12
//        DeleteChildNode((*tempnode4),nstartid);//删除此节点的子节点的所有联系
//        DeleteParentNode((*tempnode4),nstartid);
//    }
    get_node_from_txt = true;
    unsigned int nsearch_state;
    MapSearchNode *pstart_node = GetNodeFormList(nstartid);
    MapSearchNode *pgoal_node = GetNodeFormList(ngoalid);
    std::cout<<"debug_vmappinglist"<<std::endl;
    if(pstart_node != NULL && pgoal_node != NULL) {
        m_astarsearch.SetStartAndGoalStates(pstart_node,pgoal_node);
        do {
            nsearch_state = m_astarsearch.SearchStep();
        } while(nsearch_state == AStarSearch<MapSearchNode>::SEARCH_STATE_SEARCHING);

        if(nsearch_state == AStarSearch<MapSearchNode>::SEARCH_STATE_SUCCEEDED) {
            m_astarsearch.GetSolutionNodeList(&m_path_map_list);
            if(!m_path_map_list.isEmpty()) {
                GeneratePathInfo();
            }
            ShowPathWithPathList();
            return 1;
        } else if(nsearch_state == AStarSearch<MapSearchNode>::SEARCH_STATE_FAILED ) {
            ROS_FATAL("In function PathPlan, SEARCH_STATE_FAILED");
            return 0;
        }
    } else {
        ROS_FATAL("In function PathPlan. pstart_node!=NULL || pgoal_node!=NULL");
        return 0;
    }
    //复制搜索结果路径用于匹配当前路段的速度
    if(!v_mapping_list.isEmpty()){
        v_mapping_list.clear();
    }
    QList<MapSearchNode*>::iterator copy_iter = m_path_map_list.begin();
    for(; copy_iter != m_path_map_list.end(); copy_iter++){
        v_mapping_list.append(*copy_iter);
    }
    return 1;
}

void AttachXmlFile::MatchTaskPoints(QList<Task_Node> &m_TaskList) {
    //lsh//计算每个任务点位置在m_RoadList中每条道路上每两个路点上的投影点和距离投影点的距离，存在RoadList
    //lsh//找到距离最小的投影点，通过距离判断其是否在道路上
    //lsh//在向量路点间插入任务投影点，重新建立连接关系加入NodeList
    //lsh//在这对应道路上RoadNodeList插入该投影点
    if(m_TaskList.isEmpty()) {
        ROS_FATAL("MatchTaskPoints: The TaskList is empty.");
        return;
    }

    if(m_astarsearch.NodeList.isEmpty()) {
        ROS_FATAL("MatchTaskPoints: The Nodelist have not been loead.");
    }

    /*tempVelNode.x = (float)((x + DISTX)*ZOOM);
    tempVelNode.y = (float)((y + DISTY)*ZOOM);
    m_pVelNode->x = (float)((x + DISTX)*ZOOM);
    m_pVelNode->y = (float)((y + DISTY)*ZOOM);

    POSITION pos2 = astarsearch.NodeList.GetHeadPosition();
    MapSearchNode *tempnode1 = (MapSearchNode*)astarsearch.NodeList.GetNext(pos2);//µÃ³öµ½ËùÓÐµÀÂ·µÄ¾àÀë
    while(pos2 )
    {
        MapSearchNode *nextnode1 = (MapSearchNode*)astarsearch.NodeList.GetNext(pos2);
        Road_Line *tempRoadLine1 = new Road_Line;
        m_cMapMatch.LineAndLinePoint(tempnode1->x,tempnode1->y,nextnode1->x,nextnode1->y,m_pVelNode->x,m_pVelNode->y,tempRoadLine1->Road_nodex,tempRoadLine1->Road_nodey);
        m_cMapMatch.PointToLineDistance(m_pVelNode->x,m_pVelNode->y,tempRoadLine1->Road_nodex,tempRoadLine1->Road_nodey,tempRoadLine1->Dist);
        m_pVelNode ->RoadList.AddTail(tempRoadLine1);
        tempnode1 = nextnode1;
    }*/

    QList<Task_Node>::iterator taskIter;
    taskIter = m_TaskList.begin();
    for(; taskIter != m_TaskList.end(); taskIter++ ) {
        if(replanning_add_node == 2){
            replanning_add_node = 0;
            m_Replan_AddNode_list.clear();
            taskIter = m_TaskList.begin();
            ROS_WARN("stop add new way point, start add task point.");
        }
        if(replanning_add_node == 1){
            replanning_add_node = 2;
            taskIter = m_Replan_AddNode_list.begin();
            ROS_WARN("add goback point as a new way point");
            ROS_WARN("the size of m_Replan_AddNode_list is %d",m_Replan_AddNode_list.size());
        }
        //qDebug()<<"m_cMapMatch.TaskList.count"<<m_cMapMatch.TaskList.count()<<endl;

        if(!m_pTaskNode->RoadList.isEmpty())//lsh//m_pTaskNode辅助计算，保存计算结果   //notice
            m_pTaskNode->RoadList.clear();
        float x,y;
        Position_Trans_From_ECEF_To_UTM(taskIter->lat,taskIter->lon,0,0,&x,&y);
        m_pTaskNode->x = x;
        m_pTaskNode->y = y;

        QList<Road>::iterator roadIter;
        roadIter=m_RoadList.begin();//m_RoadList是以xml为单位储存路点的
        for(; roadIter!=m_RoadList.end(); roadIter++)
        {
            QList<RoadNode>::iterator tempnode1,nextnode1;
            tempnode1=(roadIter->RoadNodeList).begin();     //notice
            nextnode1=tempnode1+1;
            for(; nextnode1!=(roadIter->RoadNodeList).end(); nextnode1++)
            {
                Road_Line tempRoadLine1;
                m_cMapMatch.LineAndLinePoint(tempnode1->x,tempnode1->y,nextnode1->x,nextnode1->y,m_pTaskNode->x,m_pTaskNode->y,tempRoadLine1.Road_nodex,tempRoadLine1.Road_nodey);
                m_cMapMatch.PointToLineDistance(m_pTaskNode->x,m_pTaskNode->y,tempRoadLine1.Road_nodex,tempRoadLine1.Road_nodey,tempRoadLine1.Dist);//notice:shibushi dizhi
                m_pTaskNode ->RoadList.append(tempRoadLine1);
                tempnode1 = nextnode1;
            }
        }//lsh//计算当前车辆位置在m_RoadList中每条道路上每两个路点上的投影点和距离投影点的距离，存在RoadList

        QList<Road_Line>::iterator tempRoadLine2,nextRoadLine2;
        tempRoadLine2=m_pTaskNode->RoadList.begin();
        nextRoadLine2=tempRoadLine2+1;
        for(; nextRoadLine2!=m_pTaskNode->RoadList.end(); nextRoadLine2++)
        {
            if( tempRoadLine2 ->Dist > nextRoadLine2 ->Dist)
            {
                *tempRoadLine2 = *nextRoadLine2;
            }
        }//lsh//找到距离最小的投影点

        if(tempRoadLine2->Dist < 15.0){
            taskIter->on_road = true;
        }else{
            taskIter->on_road = false;
            ROS_WARN("task node %d isn't at road net, dis = %lf!", taskIter->Task_num,tempRoadLine2->Dist);
        }//lsh//判断任务点是否在道路上

        int flag=0;
        QList<Road>::iterator roadIter1;
        roadIter1=m_RoadList.begin();
        for(; roadIter1!=m_RoadList.end(); roadIter1++) {
            QList<RoadNode>::iterator tempnode2,nextnode2;
            tempnode2=(roadIter1->RoadNodeList).begin();     //notice
            nextnode2=tempnode2+1;
            for(; nextnode2!=(roadIter1->RoadNodeList).end(); nextnode2++) {
                Road_Line tempRoadLine3;
                m_cMapMatch.LineAndLinePoint(tempnode2->x,tempnode2->y,nextnode2->x,nextnode2->y,
                                             m_pTaskNode->x,m_pTaskNode->y,
                                             tempRoadLine3.Road_nodex,tempRoadLine3.Road_nodey);
                if( (tempRoadLine3 .Road_nodex == tempRoadLine2->Road_nodex )//tempRoadLine2就是该任务点到路网上最近那个的投影点
                    && (tempRoadLine3 .Road_nodey == tempRoadLine2->Road_nodey ))
                {//lsh//找到最近的投影点坐标
                    tempRoadLine.Road_nodex = tempRoadLine2 ->Road_nodex;
                    tempRoadLine.Road_nodey = tempRoadLine2 ->Road_nodey;

                    Task_Node tmpTaskNode = *taskIter;
                    tmpTaskNode.vlimit = tempnode2->vlimit;//lsh//tempnode2是离投影点向量的前一个点
                    tmpTaskNode.concave_obs_det = tempnode2->concave_obs_det;
                    tmpTaskNode.dynamic_obs_det = tempnode2->dynamic_obs_det;
                    tmpTaskNode.foogy_det = tempnode2->foogy_det;
                    tmpTaskNode.water_det = tempnode2->water_det;
                    tmpTaskNode.wall_area = tempnode2->wall_area;
                    tmpTaskNode.ditch_area = tempnode2->ditch_area;
                    //insert this task node to the m_astarsearch.NodeList!
                    InsertNodeIntoList_WorldXY(tempnode2->NodeID,nextnode2->NodeID,
                                               tmpTaskNode,
                                               tempRoadLine.Road_nodex,tempRoadLine.Road_nodey,
                                               roadIter1->way_version);
                    //lsh//输入为某任务点在道路上最近投影点两路点的id、经前一个路点修改限速和控制信息的任务点、投影点坐标、双向路或单向路
                    //lsh//在向量路点间插入任务投影点，重新建立连接关系
                    //update the RoadNodeList in this Road!
                    RoadNode tempRoadNode;
                    tempRoadNode.lat = taskIter->lat;//lsh//该任务点经纬度
                    tempRoadNode.lon = taskIter->lon;
                    tempRoadNode.type = taskIter->type;
                    tempRoadNode.x = tempRoadLine.Road_nodex;//lsh//该任务点投影点坐标
                    tempRoadNode.y = tempRoadLine.Road_nodey;
                    tempRoadNode.NodeID = taskIter->Task_num;
                    tempRoadNode.vlimit = tempnode2->vlimit;
                    tempRoadNode.concave_obs_det = tempnode2->concave_obs_det;
                    tempRoadNode.dynamic_obs_det = tempnode2->dynamic_obs_det;
                    tempRoadNode.foogy_det = tempnode2->foogy_det;
                    tempRoadNode.water_det = tempnode2->water_det;
                    tempRoadNode.wall_area = tempnode2->wall_area;
                    tempRoadNode.ditch_area = tempnode2->ditch_area;
                    (roadIter1->RoadNodeList).insert(nextnode2,tempRoadNode);       //insert this task node to the Node RoadNodeList
                    //lsh//在这条道路上RoadNodeList插入该投影点
                    //only for test
                    qDebug()<<"RoadNodeList: ";
                    QList<RoadNode>::iterator output=(roadIter1->RoadNodeList).begin();
                    for(; output!=(roadIter1->RoadNodeList).end();output++)
                    {
                        qDebug()<<" "<<output->NodeID;
                    }

                    flag=1;
                    break;
                }
                tempnode2 = nextnode2;
            }
            if(flag==1)
                break;
        }

        //INSERT NODE
        /*for(int j=0; j<m_RoadList.count(); j++)
        {
            for(int i=0; i<m_RoadList.at(j).RoadNodeList.count(); i++)
            {
                if((m_RoadList.at(j).RoadNodeList.at(i).NodeID==insertnodeID))
                {
                    RoadNode tempRoadNode;
                    tempRoadNode.lat=0;
                    tempRoadNode.lon=0;
                    tempRoadNode.x=tempRoadLine.Road_nodex;
                    tempRoadNode.y=tempRoadLine.Road_nodey;
                    tempRoadNode.NodeID=taskIter->Task_num;

                    m_RoadList.at(j).RoadNodeList.insert(i+1,tempRoadNode);     //update

                    //only for test
                    qDebug()<<"RoadNodeList: ";
                    for(int i=0; i<m_RoadList.at(j).RoadNodeList.count(); i++)
                    {
                        qDebug()<<" "<<m_RoadList.at(j).RoadNodeList.at(i).NodeID;
                    }

                    break;

                }


            }

        }*/
        /*POSITION pos4 = astarsearch.NodeList.GetHeadPosition();
        MapSearchNode *tempnode2 = (MapSearchNode*)astarsearch.NodeList.GetNext(pos4);
        MapSearchNode *nextnode2;
        while(pos4)
        {
            nextnode2 = (MapSearchNode*)astarsearch.NodeList.GetNext(pos4);
            Road_Line *tempRoadLine3 = new Road_Line;
            m_cMapMatch.LineAndLinePoint(tempnode2->x,tempnode2->y,nextnode2->x,nextnode2->y,m_pVelNode->x,m_pVelNode->y,tempRoadLine3->Road_nodex,tempRoadLine3->Road_nodey);
            if( (tempRoadLine3 ->Road_nodex == tempRoadLine2 ->Road_nodex ) && (tempRoadLine3 ->Road_nodey == tempRoadLine2 ->Road_nodey ))
            {
                tempRoadLine.Road_nodex = tempRoadLine2 ->Road_nodex;
                tempRoadLine.Road_nodey = tempRoadLine2 ->Road_nodey;
                //cvCircle(image,cvPoint(cvRound(tempRoadLine.Road_nodex),cvRound(tempRoadLine.Road_nodey)),2,cvScalar(0,0,255),-1);

                delete tempRoadLine3;
                break;
            }
            tempnode2 = nextnode2;
            delete tempRoadLine3;
        }*/
        /*while(!m_pTaskNode->RoadList.isEmpty())
        {/*
            Road_Line *tempRoadLine4 = (Road_Line*)m_pVelNode->RoadList.RemoveTail();
            if(NULL != tempRoadLine4)
            {
                delete tempRoadLine4;
                tempRoadLine4 = NULL;
            }
            qDeleteAll(m_pTaskNode->RoadList);
            (m_pTaskNode->RoadList).clear();
        }*/
    }
    ShowPathWithPathList();
}

void AttachXmlFile::InsertNodeIntoList_WorldXY(int FirstID, int NextID,
                                               Task_Node tmpTaskNode,
                                               float wx, float wy,
                                               QString way_version) {
    //lsh//输入为某任务点在道路上最近投影点两路点的id、经前一个路点修改限速和控制信息的任务点、投影点坐标、双向路或单向路
    //lsh//以任务点投影点为坐标，其他值取路点向量的第一个路点，构造一个新的路点添加到NodeList
    //lsh//得到两个向量点的路点信息
    //lsh//检查两向量路点间是否有连接关系
    //lsh//打断两向量路点的连接关系
    //lsh//在其中插入任务投影点，重新建立连接关系
    //向NodeList中插入节点
    MapSearchNode *Task_node;
    Task_node = new MapSearchNode;

    //using thr world coordinate to add node.
    Task_node->x=wx;        //note: x&y is not the real x&y corresponding to the real latitude and longitude!
    Task_node->y=wy;
    Task_node->lat=tmpTaskNode.lat;     //note:the lat&lon should be correspond to the wx,xy.
    Task_node->lon=tmpTaskNode.lon;
    Task_node->node_id=tmpTaskNode.Task_num;
    Task_node->type = tmpTaskNode.type;
    Task_node->intersection = false;
    Task_node->vlimit = tmpTaskNode.vlimit;
    Task_node->concave_obs_det = tmpTaskNode.concave_obs_det;
    Task_node->dynamic_obs_det = tmpTaskNode.dynamic_obs_det;
    Task_node->foogy_det = tmpTaskNode.foogy_det;
    Task_node->water_det = tmpTaskNode.water_det;
    Task_node->wall_area = tmpTaskNode.wall_area;
    Task_node->ditch_area = tmpTaskNode.ditch_area;
    m_astarsearch.NodeList.append(Task_node);
    //lsh//以任务点投影点为坐标，其他值取路点向量的第一个路点，构造一个新的路点添加到NodeList

    MapSearchNode *pFirst_node = GetNodeFormList(FirstID);
    MapSearchNode *pNext_node = GetNodeFormList(NextID);
    //lsh//得到两个向量点的路点信息
    bool Berelative=false;
    QList<MapSearchNode *>::iterator pos=pFirst_node->successorList.begin();
    for(; pos!=pFirst_node->successorList.end(); pos++)
    {
        if((*pos)->node_id==NextID)
            Berelative=true;
    }
    QList<MapSearchNode *>::iterator pos1=pNext_node->parentList.begin();
    for(; pos1!=pNext_node->parentList.end(); pos1++)
    {
        if((*pos1)->node_id==FirstID)
            Berelative=true;
    }
    //lsh//检查两向量路点间是否有连接关系
    if(Berelative)
    {
        //erase the relationship
        for(int i=0; i<pFirst_node->successorList.size(); i++)
        {
            //qDebug()<<"successorList.size():"<<pFirst_node->successorList.size()<<endl;
            if(pFirst_node->successorList.at(i)->node_id==NextID)
            {
                pFirst_node->successorList.removeAt(i);
                pFirst_node->successorNum--;
            }
        }
        for(int j=0; j<pNext_node->parentList.size(); j++)
        {
            if(pNext_node->parentList.at(j)->node_id==FirstID)
            {
                pNext_node->parentList.removeAt(j);
                pNext_node->parentNum--;
            }
        }
        if(QString::compare(way_version, QString("oneway"), Qt::CaseInsensitive)!=0)       //if doubleway
        {
            for(int i=0; i<pFirst_node->parentList.size(); i++)
            {
                if(pFirst_node->parentList.at(i)->node_id==NextID)
                {
                    pFirst_node->parentList.removeAt(i);
                    pFirst_node->parentNum--;
                }
            }
            for(int j=0; j<pNext_node->successorList.size(); j++)
            {
                if(pNext_node->successorList.at(j)->node_id==FirstID)
                {
                    pNext_node->successorList.removeAt(j);
                    pNext_node->successorNum--;
                }
            }
        }//lsh//切断两路点的连接关系

        //build the relationship
        if(pFirst_node && Task_node && pFirst_node->node_id != Task_node->node_id)
        {
            pFirst_node->successorList.append(Task_node);
            pFirst_node->successorNum++;
            Task_node->parentList.append(pFirst_node);
            Task_node->parentNum++;
            if(QString::compare(way_version, QString("oneway"), Qt::CaseInsensitive)!=0)
            {
                pFirst_node->parentList.append(Task_node);
                pFirst_node->parentNum++;
                Task_node->successorList.append(pFirst_node);
                Task_node->successorNum++;
            }//lsh//建立任务投影点和路点向量的连接关系
        }else{
            ROS_FATAL("InsertNodeIntoList_WorldXY: build topological relation failed.");
        }
        if(pNext_node && Task_node && pNext_node->node_id != Task_node->node_id)
        {
            Task_node->successorList.append(pNext_node);
            Task_node->successorNum++;
            pNext_node->parentList.append(Task_node);
            pNext_node->parentNum++;
            if(QString::compare(way_version, QString("oneway"), Qt::CaseInsensitive)!=0)
            {
                Task_node->parentList.append(pNext_node);
                Task_node->parentNum++;
                pNext_node->successorList.append(Task_node);
                pNext_node->successorNum++;
            }
        }else{
            ROS_FATAL("InsertNodeIntoList_WorldXY: build topological relation failed.");
        }
    } else {    //Berealtive==false
        ROS_FATAL("InsertNodeIntoList_WorldXY: berelative is false.");
    }
}

int AttachXmlFile::PlanWithTaskPoint(QList<Task_Node> &MyTaskList)  {
//lsh//获得已加入到Nodelist中相邻任务点的投影点形成的路点
//lsh//若搜索的是第一个任务点到第二个任务点的路径
//lsh//利用A*规划量投影点之间的路径
//lsh//将路径存入m_path_map_list
//lsh//若搜索的不是第一个任务点到第二个任务点的路径，而是其他任务点间的路径
//lsh//利用A*规划量投影点之间的路径
//lsh//忽略规划处的第一个点，将路径存入m_path_map_list
//lsh//复制m_path_map_list到v_mapping_list
//lsh//计算m_path_map_list每个点的朝向theta和行驶距离s

    /*if(m_nStartLastNode != 0)//如果规划过，则有此点，删除节点间联系
    {
        /*POSITION pos2;
        MapSearchNode *tempnode4;
        pos2 = m_astarsearch.NodeList.GetHeadPosition();
        tempnode4 = (MapSearchNode*) m_astarsearch.NodeList.GetNext(pos2);
        while(pos2)//pos2是指向m_ptempnode的位置，删除它前面相邻一个节点的所有联系，除了m_ptempnode
        {
            if(tempnode4 ->node_id == m_nStartLastNode)
            {
                break;
            }
            if(!pos2)
            {
                AfxMessageBox(_T("没有找到相同节点"));
            }
            tempnode4 = (MapSearchNode*) m_astarsearch.NodeList.GetNext(pos2);
        }
        QList<MapSearchNode *>::iterator tempnode4= m_astarsearch.NodeList.begin();
        for(; tempnode4!= m_astarsearch.NodeList.end(); tempnode4++)
        {
            if((*tempnode4)->node_id == m_nStartLastNode)
            {
                break;
            }
            if(!(*tempnode4))
            {
                QMessageBox::information(NULL,"infomation","In function PathPlan(int nstartid,int nmiddleid,int ngoalid):\n Don't find the same node.");
            }
        }
        DeleteChildNode((*tempnode4),nstartid);//删除此节点的子节点的所有联系
        DeleteParentNode((*tempnode4),nstartid);
    }*/
    QList<Task_Node>::iterator Task1,Task2;
    Task1 = MyTaskList.begin();
    Task2 = Task1+1;
    if(Task2 == MyTaskList.end()){
        ROS_FATAL("PlanWithTaskPoint: the number of task list is less than 1!!!");
        //lsh//因为此处输入的任务点中包括自车点
        return 0;
    }
    for(; Task2 != MyTaskList.end(); Task2++ )
    {
        unsigned int nsearch_state_first;
        unsigned int nsearch_state_second;
        MapSearchNode *pnodeF=GetNodeFormList(Task1->Task_num);
        MapSearchNode *pnodeB=GetNodeFormList(Task2->Task_num);
        //lsh//获得已加入到Nodelist中相邻任务点的投影点形成的路点
        //lsh//该路点经纬度为任务点实际位置，编号为任务点编号，x、y为投影点坐标
        if(NULL == pnodeF ||NULL == pnodeB) {
            ROS_FATAL("PlanWithTaskPoint: cannot find task point!!!");
            return 0;
        }

        //path-planning
        if(Task1 == MyTaskList.begin()) {//lsh//若搜索的是第一个任务点到第二个任务点的路径   //if it's the first-time plan
            if(/*Task2->type == 3 && !Task2->on_road*/false){
                //将当前点加到路网中
                search_last_point.lat = Task1->lat;
                search_last_point.lon = Task1->lon;
                //lsh//与搜索引导点对接的点
                Position_Trans_From_ECEF_To_UTM(search_last_point.lat,search_last_point.lon,
                                                0,0, &search_last_point.x,&search_last_point.y);
                search_last_point.type = 2;
                search_last_point.node_id = Task1->Task_num;
                search_last_point.vlimit = 3.0;     //搜索任务区限速
                search_last_point.intersection = false;
                search_last_point.concave_obs_det = true;
                search_last_point.dynamic_obs_det = true;
                search_last_point.foogy_det = true;
                search_last_point.water_det = true;
                search_last_point.wall_area = true;
                search_last_point.ditch_area = true;
                m_path_map_list.append(&search_last_point);
                //m_path_map_list.append(pnodeF);
                //将搜索引导点加入到路网中
                search_lead_point.lat = Task2->lat;
                search_lead_point.lon = Task2->lon;
                Position_Trans_From_ECEF_To_UTM(search_lead_point.lat,search_lead_point.lon,
                                                0,0, &search_lead_point.x,&search_lead_point.y);
                search_lead_point.type = 3;
                search_lead_point.node_id = Task2->Task_num;
                search_lead_point.vlimit = 3.0;     //搜索任务区限速
                search_lead_point.intersection = false;
                search_lead_point.concave_obs_det = true;
                search_lead_point.dynamic_obs_det = true;
                search_lead_point.foogy_det = true;
                search_lead_point.water_det = true;
                search_lead_point.wall_area = true;
                search_lead_point.ditch_area = true;
                m_path_map_list.append(&search_lead_point);
                ROS_INFO("add search lead point to path.");
            }else{
                //Astar规划部分
                m_astarsearch.SetStartAndGoalStates(pnodeF,pnodeB);//设置规划起点与终点
                ROS_INFO("plan path form %d to %d ....", pnodeF->node_id, pnodeB->node_id);
                do {
                    nsearch_state_first = m_astarsearch.SearchStep();//具体的Astar算法规划过程
                } while(nsearch_state_first == AStarSearch<MapSearchNode>::SEARCH_STATE_SEARCHING);
                //lsh//利用A*规划量投影点之间的路径，结果存在m_astarsearch.m_Start到m_astarsearch.m_Goal的父子关系中
                if(nsearch_state_first == AStarSearch<MapSearchNode>::SEARCH_STATE_SUCCEEDED) {//规划成功
                    AddSolutionToAllPlanNodeList();
                    //lsh//将路径存入m_path_map_list
                    for(int i=0;i<m_path_map_list.size()-1;i++){
                        m_path_map_list.at(i)->cur_task_num=Task1->Task_num;
                    }
                    m_path_map_list.at(m_path_map_list.size()-1)->cur_task_num=Task2->Task_num;
                    //lsh//给规划的路径上除最后一个点外的所有路点附上这段路第一个任务点的属性，最后一个点附上第二个任务点属性
                    ROS_INFO("plan path form %d to %d done.", pnodeF->node_id, pnodeB->node_id);
                } else if(nsearch_state_first == AStarSearch<MapSearchNode>::SEARCH_STATE_FAILED) {//规划失败
                    ROS_FATAL("PlanWithTaskPoint: plan path form %d to %d failed!", pnodeF->node_id, pnodeB->node_id);
                    return 0;
                }

                //打断节点关系，防止往回搜索路径
                /*MapSearchNode *delete_node1;
                m_astarsearch.GetSolutionEnd();
                delete_node1 = m_astarsearch.GetSolutionPrev();//得到上规划出来的倒数第二个节点
                if(delete_node1 != NULL){
                    DeleteChildNode(delete_node1,pnodeB ->node_id);//删除此节点的子节点的所有联系
                    DeleteParentNode(delete_node1,pnodeB ->node_id);//删除此节点的父节点的所有联系
                }*/
            }
        } else {//lsh//若搜索的不是第一个任务点到第二个任务点的路径，而是其他任务点间的路径    //it is not the first time
            if(/*Task2->type == 3 && !Task2->on_road*/false){
                //将搜索引导点加入到路网中
                search_lead_point.lat = Task2->lat;
                search_lead_point.lon = Task2->lon;
                Position_Trans_From_ECEF_To_UTM(search_lead_point.lat,search_lead_point.lon,
                                                0,0, &search_lead_point.x,&search_lead_point.y);
                search_lead_point.type = 3;
                search_lead_point.node_id = Task2->Task_num;
                search_lead_point.vlimit = 3.0;     //搜索任务区限速
                search_lead_point.intersection = false;
                search_lead_point.concave_obs_det = true;
                search_lead_point.dynamic_obs_det = true;
                search_lead_point.foogy_det = true;
                search_lead_point.water_det = true;
                search_lead_point.wall_area = true;
                search_lead_point.ditch_area = true;
                m_path_map_list.append(&search_lead_point);
                ROS_INFO("add search lead point to path.");
            }else if (/*Task1->type == 3 && !Task1->on_road*/false) {
                m_path_map_list.append(pnodeB);
                ROS_INFO("add search area exit point to path.");
            }else{
                m_astarsearch.SetStartAndGoalStates(pnodeF,pnodeB);
                ROS_INFO("plan path form %d to %d ....", pnodeF->node_id, pnodeB->node_id);
                do {
                    nsearch_state_second = m_astarsearch.SearchStep();
                } while(nsearch_state_second == AStarSearch<MapSearchNode>::SEARCH_STATE_SEARCHING);

                if(nsearch_state_second == AStarSearch<MapSearchNode>::SEARCH_STATE_SUCCEEDED) {
                    MapSearchNode *tempnode7;
                    m_astarsearch.GetSolutionStart();
                    //lsh//第一个节点不加入到m_path_map_list中
                    tempnode7 = m_astarsearch.GetSolutionNext();
                    while(tempnode7) {
                        tempnode7->cur_task_num=Task1->Task_num;
                        m_path_map_list.append(tempnode7);
                        tempnode7 = m_astarsearch.GetSolutionNext();
                        if(NULL == tempnode7) {
                            break;
                        }
                    }
                    m_path_map_list.back()->cur_task_num=Task2->type;
                    //lsh//给规划的路径上除最后一个点外的所有路点附上这段路第一个任务点的属性，最后一个点附上第二个任务点属性
                    ROS_INFO("plan path form %d to %d done.", pnodeF->node_id, pnodeB->node_id);
                } else if (nsearch_state_second == AStarSearch<MapSearchNode>::SEARCH_STATE_FAILED) {
                    ROS_FATAL("PlanWithTaskPoint: plan path form %d to %d failed!", pnodeF->node_id, pnodeB->node_id);
                    return 0;
                }

                //打断节点关系，防止往回搜索路径
                /*MapSearchNode *delete_node1;
                m_astarsearch.GetSolutionEnd();
                delete_node1 = m_astarsearch.GetSolutionPrev();//得到上规划出来的倒数第二个节点
                if(delete_node1 != NULL){
                    DeleteChildNode(delete_node1,pnodeB ->node_id);//删除此节点的子节点的所有联系
                    DeleteParentNode(delete_node1,pnodeB ->node_id);//删除此节点的父节点的所有联系
                }*/
            }
        }
        Task1=Task2;
    }

    //复制搜索结果路径用于匹配当前路段的速度
    if(!v_mapping_list.isEmpty()){
        v_mapping_list.clear();
    }
    QList<MapSearchNode*>::iterator copy_iter = m_path_map_list.begin();
    for(; copy_iter != m_path_map_list.end(); copy_iter++){
        v_mapping_list.append(*copy_iter);
    }
    if(!m_path_map_list.isEmpty()) {
        GeneratePathInfo();
    }
    ShowPathWithPathList();
    ROS_INFO("the path-planning done!");
    return 1;
}

void AttachXmlFile::Receiveinfo(float lat,float lon) {
//lsh//m_flag=0，开始匹配位
//lsh//找到包围车辆在规划道路上投影点的路点向量中的第一个路点、第二个路点和第三个路点
//lsh//根据剩余路点数设置m_flag
//lsh//m_flag=2，距离终点还有一段距离
//lsh//分别计算车辆坐标到当前路点向量和下一段路点向量的投影点和距离，并分别存储到m_pRoadLine1和m_pRoadLine2
//lsh//若此时在最后一条道路上（剩余路点的道路id一直不变），计算车辆投影点到终点的距离
//lsh//若车辆不在最后一条道路上（剩余路点的道路id会发生变化），计算车辆投影点到当前道路的最后一个路点的距离
//lsh//m_flag=3，第三个路点为终点
//lsh//返回车辆投影点距离终点的距离
//lsh/若此时车辆投影点距离第二个路点过近，则认为已经到达下一个路点，第一个路点、第二个路点都往后延后一个
//lsh//m_flag=4，第二个路点为终点
//lsh/若此时车辆投影点距离第二个路点过近，则认为已经到达终点，把m_flag置为5，表示结束
//lsh//m_flag=5，到达终点前一路点
//lsh//什么也不做
//lsh//对于所有m_flag情况
//lsh//记录当前位置到在当前路点向量上的投影点，和距离投影点的距离
//lsh//计算车辆当前位置到所有规划路点的最小距离
//lsh//若m_flag=2且当前位置到规划点距离小但到投影点距离大时重新向前匹配
    float x,y;
    Position_Trans_From_ECEF_To_UTM(lat,lon,0,0, &x,&y);
    //ShowVelPos(x,y);//在地图内显示车辆位置
    float fVelToRoadDist;

    if( 0 == m_flag) {  //第一步找出在哪一段路上
        ///m_flag在规划结果链表上进行匹配和更新的标志位。0：重新进行匹配 其它：更新
        ROS_WARN("match flag 0, rematch");
        if(v_mapping_list.size() < 2) {
            ROS_WARN("Receiveinfo:\n The size of v_mapping_list is less than 2!");
            return ;
        }
        m_pVelNode->x = x;
        m_pVelNode->y = y;
        //lsh//当前坐标
        QList<MapSearchNode *>::iterator tempnode1,nextnode1;
        tempnode1 = v_mapping_list.begin();
        nextnode1 = tempnode1+1;
        for(; nextnode1 != v_mapping_list.end(); nextnode1++) {
            Road_Line *tmp_RoadLine = new Road_Line;
            m_cMapMatch.LineAndLinePoint((*tempnode1)->x,(*tempnode1)->y,
                                         (*nextnode1)->x,(*nextnode1)->y,
                                         m_pVelNode->x,m_pVelNode->y,
                                         tmp_RoadLine->Road_nodex,tmp_RoadLine->Road_nodey);
            //lsh//计算车辆坐标在相邻路点的投影点
            m_cMapMatch.PointToLineDistance(m_pVelNode->x,m_pVelNode->y,
                                            tmp_RoadLine->Road_nodex,tmp_RoadLine->Road_nodey,
                                            tmp_RoadLine->Dist);
            //lsh//计算车辆坐标到投影点距离
            m_pVelNode ->RoadList.append(tmp_RoadLine);
            tempnode1 = nextnode1;
            //delete tmp_RoadLine;
        }
        Road_Line * temp;
        QList<Road_Line*>::iterator tmp_RoadLine1,tmp_RoadLine2;
        tmp_RoadLine1=m_pVelNode->RoadList.begin();
        tmp_RoadLine2=tmp_RoadLine1+1;
        for(; tmp_RoadLine2!=m_pVelNode->RoadList.end(); tmp_RoadLine2++) {
            if((*tmp_RoadLine1)->Dist  >  (*tmp_RoadLine2) ->Dist){
                temp=*tmp_RoadLine1;
                *tmp_RoadLine1 = *tmp_RoadLine2;//lsh//较小的一个
                *tmp_RoadLine2=temp;//lsh//较大的一个
            }
        }//lsh//找到最近的投影点
        QList<MapSearchNode *>::iterator tempnode2,nextnode2;
        tempnode2 = v_mapping_list.begin();
        nextnode2 = tempnode2 + 1;
        for(; nextnode2 != v_mapping_list.end(); nextnode2++)
        {
            Road_Line *tmp_RoadLine3 = new Road_Line;
            m_cMapMatch.LineAndLinePoint((*tempnode2)->x,(*tempnode2)->y,
                                         (*nextnode2)->x,(*nextnode2)->y,
                                         m_pVelNode->x,m_pVelNode->y,
                                         tmp_RoadLine3->Road_nodex,tmp_RoadLine3->Road_nodey);
            m_cMapMatch.PointToLineDistance(m_pVelNode->x,m_pVelNode->y,
                                            tmp_RoadLine3->Road_nodex,tmp_RoadLine3->Road_nodey,
                                            tmp_RoadLine3->Dist);
            /*if((tmp_RoadLine3 ->Road_nodex == (*tmp_RoadLine1) ->Road_nodex)
               &&(tmp_RoadLine3 ->Road_nodey == (*tmp_RoadLine1) ->Road_nodey)){*/
            if(std::abs(tmp_RoadLine3->Dist - (*tmp_RoadLine1)->Dist) < 0.1){
                tempRoadLine.Road_nodex = (*tmp_RoadLine1) ->Road_nodex;
                tempRoadLine.Road_nodey = (*tmp_RoadLine1) ->Road_nodey;
                delete tmp_RoadLine3;
                break;
            }//lsh//？？？为什么这么做，之前就可以记录
            tempnode2 = nextnode2;
            delete tmp_RoadLine3;
        }//lsh//记录最近的投影点到tempRoadLine
        if(!m_pVelNode->RoadList.isEmpty())     //释放内存
        {
            qDeleteAll(m_pVelNode->RoadList);
            m_pVelNode->RoadList.clear();
        }//lsh//原本记录的是投影点和投影点距离
        m_ptempnode = *tempnode2;//lsh//包围最近投影点的路点向量中的第一个路点
        m_nStartLastNode = m_ptempnode ->node_id;//lsh//用来存放道路ID，防止重复读入道路数据
        m_pnextnode = *nextnode2;//lsh//包围最近投影点的路点向量中的第二个路点
        if((nextnode2+1) != v_mapping_list.end()){
            m_pthirdnode = *(nextnode2+1);//lsh//包围最近投影点的路点向量中的第三个路点
        }else{
            m_flag = 4;//lsh//此时第二个路点为终点
            return;
        }
        m_pos = nextnode2+2;
        if(m_pos != v_mapping_list.end())
            m_flag = 2;//lsh//此时离终点还有距离
        else
            m_flag = 3;//lsh//此时第三个路点为终点
        last_match_num = -1;
        update_flag_for_task = 0;
    }
//lsh//m_flag=0
//lsh//找到包围车辆在规划道路上投影点的路点向量中的第一个路点、第二个路点和第三个路点
//lsh//根据剩余路点数设置m_flag

    //lsh//此时离终点还有距离
    if( 2 == m_flag) {
        m_pVelNode ->x = (float)x ;
        m_pVelNode ->y = (float)y ;

        m_cMapMatch.LineAndLinePoint(m_ptempnode->x,m_ptempnode->y,
                                     m_pnextnode->x,m_pnextnode->y,
                                     m_pVelNode->x,m_pVelNode->y,
                                     m_pRoadLine1->Road_nodex,m_pRoadLine1->Road_nodey);//点与道路1的交点
        m_cMapMatch.LineAndLinePoint(m_pnextnode->x,m_pnextnode->y,
                                     m_pthirdnode->x,m_pthirdnode->y,
                                     m_pVelNode->x,m_pVelNode->y,
                                     m_pRoadLine2->Road_nodex,m_pRoadLine2->Road_nodey);//点与道路2的交点
        m_cMapMatch.PointToLineDistance(m_pVelNode->x,m_pVelNode->y,
                                        m_pRoadLine1->Road_nodex,m_pRoadLine1->Road_nodey,
                                        m_pRoadLine1->Dist);//点到道路1的距离
        m_cMapMatch.PointToLineDistance(m_pVelNode->x,m_pVelNode->y,
                                        m_pRoadLine2->Road_nodex, m_pRoadLine2->Road_nodey,
                                        m_pRoadLine2->Dist);//点到道路2的距离
        //lsh//分别计算车辆坐标到当前路点向量和下一段路点向量的投影点和距离，并分别存储到m_pRoadLine1和m_pRoadLine2
        if(m_pRoadLine1->Dist < m_pRoadLine2->Dist) {//lsh//？？？一定
            //ShowVelPos(m_pVelNode->x,m_pVelNode->y);//在地图内显示车辆位置
            /*if( m_flag == 2)
            {
                MapSearchNode *tempnode6,*nextnode6,*thirdnode6;//这一段给出到路口节点的距离

                POSITION pos4;
                tempnode6 = m_ptempnode;
                pos4 = m_path_map_list.GetHeadPosition();
                nextnode6 = (MapSearchNode *)m_path_map_list.GetNext(pos4);
                while(pos4 )
                {
                    thirdnode6 = (MapSearchNode *)m_path_map_list.GetNext(pos4);
                    if(tempnode6 == nextnode6)
                    {
                        thirdnode6 = (MapSearchNode *)m_path_map_list.GetPrev(pos4);
                        while(pos4)
                        {
                            thirdnode6 = (MapSearchNode *)m_path_map_list.GetNext(pos4);
                            if(pos4 == NULL)
                            {
                                fVelToRoadDist = sqrt(pow((m_pRoadLine1->Road_nodex - thirdnode6->x),2) + pow((m_pRoadLine1->Road_nodey - thirdnode6->y),2));
                                //ShowDisplay(fVelToRoadDist,m_ptempnode,m_pnextnode,m_pthirdnode,m_pEndNode);
                                break;

                            }
                            else if( tempnode6 ->road_id != thirdnode6 ->road_id)
                            {
                                fVelToRoadDist = sqrt(pow((m_pRoadLine1->Road_nodex - nextnode6->x),2) + pow((m_pRoadLine1->Road_nodey - nextnode6->y),2));
                                //ShowDisplay(fVelToRoadDist,m_ptempnode,m_pnextnode,m_pthirdnode,m_pEndNode);

                                break;
                            }
                            nextnode6 = thirdnode6;
                            if( NULL == pos4)
                            {
                                break;
                            }
                        }
                        break;
                    }
                    nextnode6 = thirdnode6;
                    if(NULL == pos4)
                    {
                        break;
                    }
                }
            }*/
            if( m_flag == 2)//lsh//？？？？一定        //这一段给出到路口节点的距离
            {
                //MapSearchNode *tempnode6,*nextnode6,*thirdnode6;
                MapSearchNode *tempnode6;
                tempnode6 = m_ptempnode;
                /*POSITION pos4;
                pos4 = m_path_map_list.GetHeadPosition();
                nextnode6 = (MapSearchNode *)m_path_map_list.GetNext(pos4);
                while(pos4 )
                {
                    thirdnode6 = (MapSearchNode *)m_path_map_list.GetNext(pos4);*/
                QList<MapSearchNode*>::iterator nextnode6,thirdnode6;
                nextnode6 = v_mapping_list.begin();
                thirdnode6 = nextnode6+1;
                for(; thirdnode6 != v_mapping_list.end(); thirdnode6++) {
                    if(tempnode6 == *nextnode6) {
                        /*thirdnode6 = (MapSearchNode *)m_path_map_list.GetPrev(pos4);
                        while(pos4)
                        {
                            thirdnode6 = (MapSearchNode *)m_path_map_list.GetNext(pos4);*/
                        for(; thirdnode6 != v_mapping_list.end(); thirdnode6++) {
                            //if(pos4 == NULL)
                            if((thirdnode6+1) == v_mapping_list.end()) {//lsh//？？？？一定不会
                                fVelToRoadDist = sqrt(pow((m_pRoadLine1->Road_nodex - (*thirdnode6)->x),2)
                                                      + pow((m_pRoadLine1->Road_nodey - (*thirdnode6)->y),2));
                                //ShowDisplay(fVelToRoadDist,m_ptempnode,m_pnextnode,m_pthirdnode,m_pEndNode);
                                break;

                            }//lsh//若此时在最后一条道路上（剩余路点的道路id一直不变），计算车辆投影点到终点点的距离
                            else if( tempnode6->road_id != (*thirdnode6)->road_id) {//lsh//？？？？一定
                                fVelToRoadDist = sqrt(pow((m_pRoadLine1->Road_nodex - (*nextnode6)->x),2)
                                                      + pow((m_pRoadLine1->Road_nodey - (*nextnode6)->y),2));
                                //ShowDisplay(fVelToRoadDist,m_ptempnode,m_pnextnode,m_pthirdnode,m_pEndNode);

                                break;
                            }//lsh//若车辆不在最后一条道路上（剩余路点的道路id会发生变化），计算车辆投影点到当前道路的最后一个路点的距离
                            nextnode6 = thirdnode6;
                            /*if( NULL == pos4)
                            {
                                break;
                            }*/
                        }
                        break;
                    }
                    nextnode6 = thirdnode6;
                    /*if(NULL == pos4)
                    {
                        break;
                    }*/
                }
            }
            tempRoadLine.Road_nodex = m_pRoadLine1->Road_nodex;//lsh//投影点
            tempRoadLine.Road_nodey = m_pRoadLine1->Road_nodey;
            tempVelNode.x = m_pVelNode ->x;//lsh//当前车辆位置
            tempVelNode.y = m_pVelNode ->y;
        } else {    //到一下道路的距离小于本条道路的距离时，证明换道了
            //ShowVelPos(m_pVelNode->x,m_pVelNode->y);//在地图内显示车辆位置
            m_flag = 2;
            m_nStartLastNode = m_ptempnode->node_id;
            m_nStartNode = m_pnextnode->node_id;
            m_ptempnode = m_pnextnode;
            m_pnextnode = m_pthirdnode;
            //m_pthirdnode = (MapSearchNode*)m_path_map_list.GetNext(m_pos);
//            if(m_pos != m_path_map_list.end())

            if(m_pos != v_mapping_list.end())
                m_pthirdnode = *m_pos;
            m_pos++;
            if(m_pos == v_mapping_list.end())
            {
                m_flag = 3;
//                espx = (m_pnextnode->x - m_ptempnode->x)/10;        //notice
//                espy = (m_pnextnode->y - m_ptempnode->y)/10;
                m_cMapMatch.PointToSlope(m_ptempnode->x,m_ptempnode->y,
                                         m_pnextnode->x,m_pnextnode->y,
                                         m_pRoadLine1->R_Slope);
                //lsh//输入为当前路点向量的第一个路点和第二个路点，节点间的斜率
            }
            else
            {
                /*m_cMapMatch.PointToSlope(m_ptempnode->x,m_ptempnode->y,
                                         m_pnextnode->x,m_pnextnode->y,
                                         m_pRoadLine1->R_Slope);//求道路1的斜率
                m_cMapMatch.PointToSlope(m_pnextnode->x,m_pnextnode->y,
                                         m_pthirdnode->x,m_pthirdnode->y,
                                         m_pRoadLine2->R_Slope);//求道路2的斜率*/
//            espx = (m_pnextnode->x - m_ptempnode->x)/10;        //notice
//            espy = (m_pnextnode->y - m_ptempnode->y)/10;
            }
        }
    }//lsh//分别计算车辆坐标到当前路点向量和下一段路点向量的投影点和距离，并分别存储到m_pRoadLine1和m_pRoadLine2
    //lsh//若此时在最后一条道路上（剩余路点的道路id一直不变），计算车辆投影点到终点前一个点的距离
    //lsh//若车辆不在最后一条道路上（剩余路点的道路id会发生变化），计算车辆投影点到当前道路的最后一个路点的距离

    //if( (3 == m_flag) || ( m_flag == 4))//最后三个节点的前两个节点
    if( 3 == m_flag ) { //lsh//第三个路点为终点
        m_pVelNode ->x = (float)x ;
        m_pVelNode ->y = (float)y ;
        //ShowVelPos(m_pVelNode->x,m_pVelNode->y);//在地图内显示车辆位置
        m_cMapMatch.LineAndLinePoint(m_ptempnode->x,m_ptempnode->y,
                                     m_pnextnode->x,m_pnextnode->y,
                                     m_pVelNode->x,m_pVelNode->y,
                                     m_pRoadLine1->Road_nodex,m_pRoadLine1->Road_nodey);
        fVelToRoadDist = sqrt(pow((m_pRoadLine1->Road_nodex - m_pthirdnode->x),2)
                              + pow((m_pRoadLine1->Road_nodey - m_pthirdnode->y),2));
        //lsh//返回车辆投影点距离终点前一个点的距离
        tempRoadLine.Road_nodex = m_pRoadLine1->Road_nodex;
        tempRoadLine.Road_nodey = m_pRoadLine1->Road_nodey;
        tempVelNode.x = m_pVelNode ->x;
        tempVelNode.y = m_pVelNode ->y;
        if( (std::fabs(m_pRoadLine1->Road_nodex - m_pnextnode->x) < 1)
            && (std::fabs(m_pRoadLine1->Road_nodey - m_pnextnode ->y) < 1) ) {
            m_flag = 4;
            m_nStartLastNode = m_ptempnode->node_id;
            m_nStartNode = m_pnextnode ->node_id;
            m_ptempnode = m_pnextnode;
            m_pnextnode = m_pthirdnode;
            espx = (m_pnextnode->x - m_ptempnode->x)/10;
            espy = (m_pnextnode->y - m_ptempnode->y)/10;//lsh//？？？有什么用
        }//lsh/若此时车辆投影点距离第二个路点过近，则认为已经到达下一个路点，第一个路点、第二个路点都往后延后一个
    }//lsh//返回车辆投影点距离终点的距离
    //lsh/若此时车辆投影点距离第二个路点过近，则认为已经到达下一个路点，第一个路点、第二个路点都往后延后一个

    if(4 == m_flag){//lsh//第二个路点为终点
        m_pVelNode ->x = (float)x ;
        m_pVelNode ->y = (float)y ;
        //ShowVelPos(m_pVelNode->x,m_pVelNode->y);//在地图内显示车辆位置
        m_cMapMatch.LineAndLinePoint(m_ptempnode->x,m_ptempnode->y,
                                     m_pnextnode->x,m_pnextnode->y,
                                     m_pVelNode->x,m_pVelNode->y,
                                     m_pRoadLine1->Road_nodex,m_pRoadLine1->Road_nodey);
        if( (std::fabs(m_pRoadLine1->Road_nodex - m_pnextnode->x) < 1)
            && (std::fabs(m_pRoadLine1->Road_nodey - m_pnextnode ->y) < 1) ) {
            m_flag = 5; //结束
        }
    }//lsh/若此时车辆投影点距离第二个路点过近，则认为已经到达终点，把m_flag置为5，表示结束

    if(5 == m_flag){
        //KillTimer(1);
        //m_bTimer1Flag = false;
    }
    m_fStartNodex = x;
    m_fStartNodey = y;
    //lsh//记录当前位置

    //当匹配点距离当前点超过一定值且当前点距离规划路经最近点小于一定值，则重新匹配
    /*double dis_to_match_point = distance(x,y,m_ptempnode->x,m_ptempnode->y);
    double min_dis_to_path = std::numeric_limits<double>::infinity();
    for(int i = 0; i < v_mapping_list.size(); ++i) {
        double dist = this->distance(x, y, v_mapping_list.at(i)->x, v_mapping_list.at(i)->y);
        if(dist < min_dis_to_path) {
            min_dis_to_path = dist;
        }
    }
    if(min_dis_to_path < 5 && (dis_to_match_point - min_dis_to_path) > 30 && m_pnextnode->type != 3){
        ROS_WARN("too far from match point, rematch !!!");
        m_flag = 0;
    }*/
    float dis_to_match_road = 0.0;
    m_cMapMatch.LineAndLinePoint(m_ptempnode->x,m_ptempnode->y,
                                 m_pnextnode->x,m_pnextnode->y,
                                 m_pVelNode->x,m_pVelNode->y,
                                 m_pRoadLine1->Road_nodex,m_pRoadLine1->Road_nodey);//点与道路1的交点
    m_cMapMatch.PointToLineDistance(m_pVelNode->x,m_pVelNode->y,
                                    m_pRoadLine1->Road_nodex,m_pRoadLine1->Road_nodey,
                                    dis_to_match_road);
    //lsh//记录当前位置到在当前路点向量上的投影点，和距离投影点的距离
    double min_dis_to_path = std::numeric_limits<double>::infinity();
    for(int i = 0; i < v_mapping_list.size(); ++i) {
        double dist = this->distance(x, y, v_mapping_list.at(i)->x, v_mapping_list.at(i)->y);
        if(dist < min_dis_to_path) {
            min_dis_to_path = dist;
        }
    }//lsh//计算车辆当前位置到所有规划路点的最小距离
    if(min_dis_to_path < 5 && dis_to_match_road > 20 /*&& m_pnextnode->type != 3*/){    //当下一任务区为搜索区域时不进行匹配更新
        float tempD2MatchRoad = 0.0;
        if(m_flag == 2){    //当flag = 3or4时，停止重新匹配
            std::cout << "min_dis_to_path" << min_dis_to_path << std::endl;
            std::cout << "dis_to_match_road" << dis_to_match_road << std::endl;
            ROS_WARN("too far from match point, rematch !!!");
            m_flag=0;
            //lshadd0818//
            /*MapSearchNode *m_ptempnode_cp,*m_pnextnode_cp,*m_pthirdnode_cp;
            QList<MapSearchNode *>::iterator m_pos_cp;
            m_ptempnode_cp=m_ptempnode;
            m_pnextnode_cp=m_pnextnode;
            m_pthirdnode_cp=m_pthirdnode;
            m_pos_cp=m_pos;//lshadd0818//
            do{
                m_nStartLastNode = m_ptempnode->node_id;
                m_nStartNode = m_pnextnode->node_id;
                m_ptempnode = m_pnextnode;
                m_pnextnode = m_pthirdnode;
                if(m_pos != v_mapping_list.end()) m_pthirdnode = *m_pos;
                m_pos++;
                if(m_pos == v_mapping_list.end()){
                    m_flag = 3;
                }
                m_cMapMatch.LineAndLinePoint(m_ptempnode->x,m_ptempnode->y,
                                             m_pnextnode->x,m_pnextnode->y,
                                             m_pVelNode->x,m_pVelNode->y,
                                             m_pRoadLine1->Road_nodex,m_pRoadLine1->Road_nodey);//点与道路1的交点
                m_cMapMatch.PointToLineDistance(m_pVelNode->x,m_pVelNode->y,
                                                m_pRoadLine1->Road_nodex,m_pRoadLine1->Road_nodey,
                                                tempD2MatchRoad);//点到道路1的距离
            *//*}while(tempD2MatchRoad < 10);*//*
            //lshadd0818//
            }while(tempD2MatchRoad > 10);
            if(tempD2MatchRoad > 10){
                m_ptempnode=m_ptempnode_cp;
                m_pnextnode=m_pnextnode_cp;
                m_pthirdnode=m_pthirdnode_cp;
                m_pos=m_pos_cp;
                m_nStartLastNode = m_ptempnode->node_id;
                m_nStartNode = m_pnextnode->node_id;
                m_flag=2;
                ROS_WARN("too far from front road, can not rematch, keep last m_ptempnode.");
            }*///lshadd0818//
        }
    }//lsh//？？？若mflag=2且当前位置到规划点距离小但到投影点距离大时重新向前匹配
    //非重规划时，在前进后退转换点进行切换时，当需要向前更新匹配点时，则进行下面的操作
    /*if(match_update && m_ptempnode != NULL && m_flag == 2){
        ROS_INFO("match update...");
        match_update = false;
        QList<MapSearchNode *>::iterator pos_iter = m_pos - 2;
        int curMatchID = m_ptempnode->node_id;
        for(;pos_iter != v_mapping_list.end(); pos_iter++){
            if((*pos_iter)->node_id == curMatchID){
                do{
                    m_nStartLastNode = m_ptempnode->node_id;
                    m_nStartNode = m_pnextnode->node_id;
                    m_ptempnode = m_pnextnode;
                    m_pnextnode = m_pthirdnode;
                    if(m_pos != v_mapping_list.end()) m_pthirdnode = *m_pos;
                    m_pos++;
                    if(m_pos == v_mapping_list.end()){
                        m_flag = 3;
                        break;
                    }
                }while(m_ptempnode->node_id != curMatchID);
                break;
            }
        }
        ROS_INFO("match update done");
    }*/
}

void AttachXmlFile::ShowVelPos(float x1,float y1) {
    //为车辆画点
    windowCenterUpdateF++;
    if(windowCenterUpdateF > 30){   //3s更新一次，将车辆放置在窗口中心
        windowCenterUpdateF = 0;
        delta_origin_pos.x = (int)(x1 - MAP_WIDTH_CELL/(2*global_env_zoom));
        delta_origin_pos.y = (int)(y1 - MAP_HEIGHT_CELL/(2*global_env_zoom));
    }
    int x=int((x1-delta_origin_pos.x)*global_env_zoom);
    int y=int(MAP_HEIGHT_CELL-(y1-delta_origin_pos.y)*global_env_zoom);
    cvCircle(m_EnvImage_Global_diaplay,cvPoint(x,y),3,COLORRED,-1);
    CvFont font;
    char szChar[100];
    sprintf(szChar, "Car");
    cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.4f, 0.4f, 0, 1, 8);
    cvPutText(m_EnvImage_Global_diaplay, szChar, cvPoint(x,y), &font, COLORRED);
    /*pos_int left_below,left_top;
    pos_int right_below,right_top;
    pos_int right;
    double sita = vehicle_theta;*/
    /*if(m_ptempnode != NULL && m_pnextnode != NULL && m_ptempnode->x != m_pnextnode->x)
    {
        sita = atan((m_pnextnode->y - m_ptempnode->y)/(m_pnextnode->x - m_ptempnode->x));
        if((m_pnextnode->y - m_ptempnode->y > 0) && (sita < 0))
        {
            sita += CV_PI;
        }
        else if( (m_pnextnode->y -m_ptempnode->y <0) && (sita >0))
        {
            sita -= CV_PI;
        }
        //note:else?
    }
    else
    {
        sita = CV_PI/2;
    }*/
    /*right_top.x = x + VEL_SIZE_LENGHT*cos(sita) - VEL_SIZE_WIDTH*sin(sita);
    right_top.y = y +VEL_SIZE_LENGHT*sin(sita) + VEL_SIZE_WIDTH*cos(sita);
    left_top.x = x -VEL_SIZE_LENGHT*cos(sita) - VEL_SIZE_WIDTH*sin(sita);
    left_top.y = y -VEL_SIZE_LENGHT*sin(sita) + VEL_SIZE_WIDTH*cos(sita);
    left_below.x = x -VEL_SIZE_LENGHT*cos(sita) + VEL_SIZE_WIDTH*sin(sita);
    left_below.y = y -VEL_SIZE_LENGHT*sin(sita) - VEL_SIZE_WIDTH*cos(sita);
    right_below.x = x + VEL_SIZE_LENGHT*cos(sita) + VEL_SIZE_WIDTH*sin(sita);
    right_below.y = y +VEL_SIZE_LENGHT*sin(sita) - VEL_SIZE_WIDTH*cos(sita);
    right.x = x + (VEL_SIZE_LENGHT+VEL_SIZE_WIDTH)*cos(sita);
    right.y = y +(VEL_SIZE_LENGHT+VEL_SIZE_WIDTH)*sin(sita);
    cvLine(m_EnvImage_Global_diaplay,cvPoint(left_below.x,left_below.y),cvPoint(left_top.x,left_top.y),COLORBLACK,1);
    cvLine(m_EnvImage_Global_diaplay,cvPoint(left_top.x,left_top.y),cvPoint(right_top.x,right_top.y),COLORBLACK,1);
    cvLine(m_EnvImage_Global_diaplay,cvPoint(right_top.x,right_top.y),cvPoint(right.x,right.y),COLORBLACK,1);
    cvLine(m_EnvImage_Global_diaplay,cvPoint(right.x,right.y),cvPoint(right_below.x,right_below.y),COLORBLACK,1);
    cvLine(m_EnvImage_Global_diaplay,cvPoint(right_below.x,right_below.y),cvPoint(left_below.x,left_below.y),COLORBLACK,1);*/
    //为车辆画框
    vector<cv::Point2i> Points;
    cv::Point2i left_below(x-7, y+7);
    cv::Point2i left_top(x-7, y-7);
    cv::Point2i middle_top(x, y-16);
    cv::Point2i right_top(x+7, y-7);
    cv::Point2i right_below(x+7, y+7);
    Points.push_back(left_below);
    Points.push_back(left_top);
    Points.push_back(middle_top);
    Points.push_back(right_top);
    Points.push_back(right_below);
    cv::Point rotate_center(x,y);
    double rotate_angle = (vehicle_theta - M_PI_2)*180.0/M_PI;
    vector<cv::Point2i> dstPoints = getRotatePoint(Points,rotate_center, rotate_angle);
    cvLine(m_EnvImage_Global_diaplay,cvPoint(dstPoints[0].x,dstPoints[0].y),cvPoint(dstPoints[1].x,dstPoints[1].y),COLORRED,2);
    cvLine(m_EnvImage_Global_diaplay,cvPoint(dstPoints[1].x,dstPoints[1].y),cvPoint(dstPoints[2].x,dstPoints[2].y),COLORRED,2);
    cvLine(m_EnvImage_Global_diaplay,cvPoint(dstPoints[2].x,dstPoints[2].y),cvPoint(dstPoints[3].x,dstPoints[3].y),COLORRED,2);
    cvLine(m_EnvImage_Global_diaplay,cvPoint(dstPoints[3].x,dstPoints[3].y),cvPoint(dstPoints[4].x,dstPoints[4].y),COLORRED,2);
    cvLine(m_EnvImage_Global_diaplay,cvPoint(dstPoints[4].x,dstPoints[4].y),cvPoint(dstPoints[0].x,dstPoints[0].y),COLORRED,2);
    cvShowImage("global env",m_EnvImage_Global_diaplay);
}

vector<cv::Point2i> AttachXmlFile::getRotatePoint(vector<cv::Point2i> Points, const cv::Point rotate_center, const double angle){
    vector<cv::Point2i> dstPoints;
    int x1 = 0, y1 = 0;
    int row = MAP_HEIGHT_CELL;
    for(size_t i = 0; i < Points.size(); i++){
        x1 = Points.at(i).x;
        y1 = row - Points.at(i).y;
        int x2 = rotate_center.x;
        int y2 = row - rotate_center.y;
        int x = cvRound((x1 - x2)*cos(M_PI / 180.0 * angle) - (y1 - y2)*sin(M_PI / 180.0 * angle) + x2);
        int y = cvRound((x1 - x2)*sin(M_PI / 180.0 * angle) + (y1 - y2)*cos(M_PI / 180.0 * angle) + y2);
        y = row - y;
        dstPoints.push_back(cv::Point2i(x, y));
    }
    return dstPoints;
}

int AttachXmlFile::RePlanning(float flat, float flon) {
    //lsh//断点，在新的任务列表里进行重规划，重规划的结果存储在v_mapping_list
    //lsh//将规划结果分为两条道路，并分别为两条路补充一段额外的掉头路段，并将第一条道路放入m_path_map_list
    //更新任务链表
    //ReCreateTaskList(flat,flon);

    //重新加载和匹配任务点
    /*if(!m_astarsearch.NodeList.isEmpty()) {
        DeleteAStarSearchNodeList();
    }
    if(!m_path_map_list.isEmpty()) {
        m_path_map_list.clear();
    }
    if(!m_RoadList.isEmpty()) {
        m_RoadList.clear();
    }*/
    if(!original_road_network_list.isEmpty()){
        qDeleteAll(original_road_network_list);
        original_road_network_list.clear();
    }
    if(!m_astarsearch.NodeList.isEmpty()) {
        m_Pathid_vector.clear();
        //qDeleteAll(m_attach_xml_file.m_astarsearch.NodeList);
        m_astarsearch.NodeList.clear();
        if(!m_astarsearch.NodeList.isEmpty())
            ROS_FATAL("m_astarsearch.NodeList: clear error!");
    }
    if(!m_RoadList.isEmpty()) {
        m_RoadList.clear();
    }
    if(!m_path_map_list.isEmpty()) {
        m_path_map_list.clear();
    }
    if(!v_mapping_list.isEmpty()){
        v_mapping_list.clear();
    }//lsh//清空original_road_network_list、m_Pathid_vector、NodeList、m_RoadList、m_path_map_list、v_mapping_list

    GetNodeFromFile();
    //lsh//重新读取道路
    //InitDisplay();

    QList<Task_Node>::iterator iter;
    iter = m_Replan_Task_list.begin();
    std::cout << "replanning task file list:" << std::endl << "     ID     type" << std::endl;
    for(; iter != m_Replan_Task_list.end(); iter++) {
        std::cout << "     " <<  iter->Task_num << "     " << iter->type << std::endl;
    }//lsh//打印打算重规划的任务点

    //在没有更新路网的情况下，仅插入车辆当前位置点作为规划起点
    /*QList<Task_Node> temp_list;
    temp_list.append(m_Replan_Task_list.first());*/
    MatchTaskPoints(m_Replan_Task_list);
    //lsh//在路网中插入任务点的投影点

    //打断堵塞位置节点关系
    /*ROS_INFO("break the previous blocking location node relation ......");
    for(int i = 0; i < obs_node_id.size() ; i++){
        QList<MapSearchNode *>::iterator node_iter = m_astarsearch.NodeList.begin();
        for(; node_iter != m_astarsearch.NodeList.end(); node_iter++)
        {
            if((*node_iter)->node_id == obs_node_id.at(i)) {
                break;
            }
            if((node_iter + 1) == m_astarsearch.NodeList.end()) {
                ROS_FATAL("In RePlanning: Don't find the obs node.");
            }
        }
        DeleteChildNode(*(node_iter+1),obs_node_id.at(i));
        DeleteParentNode(*(node_iter+1),obs_node_id.at(i));
    }
    ROS_INFO("break the previous blocking location node relation done.");*/
    QList<MapSearchNode *>::iterator tempnode= m_astarsearch.NodeList.begin();
    for(; tempnode != m_astarsearch.NodeList.end(); tempnode++) {
        if((*tempnode)->node_id == ReStartNextID) {//lsh//找到触发重规划时车辆当前位置的下一个路点ID
            break;
        }
        if((tempnode +1) == m_astarsearch.NodeList.end()) {
            std::cout << "m_astarsearch.NodeList size: " << m_astarsearch.NodeList.size();
            std::cout << "RestartnextID: " << ReStartNextID << std::endl;
            ROS_FATAL("In RePlanning: Don't find the ReStartNextID node.");
            return 0;
        }
    }
    DeleteChildNode((*tempnode),m_StartNode.ID);//lsh//每次重规划前，将车当前位置作为第一个任务点，并对其分配ID，m_StartNode.ID从10000000开始记
    //lsh//删除(*tempnode)的所有子节点，删除除m_StartNode.ID对应节点外其他子节点的的所有父子节点
    DeleteParentNode((*tempnode),m_StartNode.ID);

    int value = PlanWithTaskPoint(m_Replan_Task_list);//lsh//规划结果存储在m_path_map_list与v_mapping_list
    if(value == 0){//lsh//此时规划失败
        ROS_FATAL("PlanWithTaskPoint: plan failed!!!");
        //copy new NodeList to original_road_network_list
        if(!original_road_network_list.isEmpty())
            original_road_network_list.clear();
        QList<MapSearchNode*>::iterator road_iter = m_astarsearch.NodeList.begin();
        for(; road_iter != m_astarsearch.NodeList.end(); road_iter++){
            original_road_network_list.append(*road_iter);
        }
        return 0;
    }//lsh//规划失败时original_road_network_list与m_astarsearch.NodeList存放内容相同
    //在规划成功后，记录该阻塞位置
    /*obs_node_id.append((*tempnode)->node_id);
    ROS_INFO("record obs id: %d" , (*tempnode)->node_id);*/
    //obs_node_id.append((*(tempnode-1))->node_id);

    //复制搜索结果路径用于匹配当前路段的速度
    if(!v_mapping_list.isEmpty()){
        v_mapping_list.clear();
    }
    QList<MapSearchNode*>::iterator copy_iter = m_path_map_list.begin();
    for(; copy_iter != m_path_map_list.end(); copy_iter++){
        v_mapping_list.append(*copy_iter);
    }

    //copy new NodeList to original_road_network_list
    if(!original_road_network_list.isEmpty())
        original_road_network_list.clear();
    QList<MapSearchNode*>::iterator road_iter = m_astarsearch.NodeList.begin();
    for(; road_iter != m_astarsearch.NodeList.end(); road_iter++){
        original_road_network_list.append(*road_iter);
    }
    splitPath(&m_path_map_list);
    //lsh//将规划结果从路口点分为两段，分别放入first_path和second_path
    //lsh//找到一条18米左右的可通行倒车路径
    //lsh//将该倒车路径添加到first_path与second_path中，并将first_path放入m_path_map_list
    ShowPathWithPathList();  //更新显示
    return 1;
}

void AttachXmlFile::splitPath(QList<MapSearchNode *> *result_path) {
    //lsh//将规划结果从路口点分为两段，分别放入first_path和second_path
    //lsh//找到路口点的所有子节点中不与当前规划结果路点相同的子节点
    //lsh//找到一条18米左右的可通行倒车路径
    //lsh//将该倒车路径添加到first_path与second_path中，并将first_path放入m_path_map_list
    std::cout << "split path ......." << std::endl;
    if(!first_path.isEmpty()){
        first_path.clear();
    }
    if(!second_path.isEmpty()){
        second_path.clear();
    }

    //找到最近的岔道口以及通往最近岔道口的一条路径
    MapSearchNode* nearest_intersec = NULL;
    QList<MapSearchNode*> recent_path;
    QList<MapSearchNode*>::iterator path_iter = result_path->begin();
    for(; path_iter != result_path->end(); path_iter++){
        first_path.append(*path_iter);
        recent_path.append(*path_iter);
        if((*path_iter)->intersection){
            nearest_intersec = *path_iter;//lsh//nearest_intersec记住路口点位置
            if(path_iter + 1 != result_path->end()){
                recent_path.append(*(path_iter + 1));
            }
            for(; path_iter != result_path->end(); path_iter++){
                second_path.append(*path_iter);
            }
            break;
        }
    }//lsh//将路口前且包含路口的点存入first_path，将路口后的点且包括路口存入second_path
    //lsh//nearest_intersec记录路口点，recent_path存放比一段路多一个再往后延伸的路点

    //找到一个不在规划路径上的分岔口用于倒车
    MapSearchNode* available_node = NULL;
    QList<MapSearchNode*>::iterator fork_iter = nearest_intersec->successorList.begin();
    for(; fork_iter != nearest_intersec->successorList.end(); fork_iter++){
        bool find_flag = false;
        QList<MapSearchNode*>::iterator path_iter = recent_path.begin();
        for(; path_iter != recent_path.end(); path_iter++){
            if((*fork_iter)->node_id == (*path_iter)->node_id){ break; }
            if((path_iter + 1) == recent_path.end()){
                find_flag = true;
                available_node = *fork_iter;
                //std::cout << "available_node id: " << available_node->node_id << std::endl;
                break;
            }
        }
        if(find_flag){ break; }
    }//lsh//找到路口点的所有子节点中不与当前规划结果路点相同的子节点
    /*QList<MapSearchNode*>::iterator fork_iter1 = nearest_intersec->successorList.begin();
    //std::cout << "the nearest_intersec successorlist :" << std::endl;
    for(; fork_iter1 != nearest_intersec->successorList.end(); fork_iter1++){
        std::cout << "    " << (*fork_iter1)->node_id << std::endl;
    }*/

    QList<MapSearchNode*> temp_avai_fork_list;
    if(available_node == NULL){
        ROS_WARN("splitPath(): cannot find an available fork.");
    }else{
        temp_avai_fork_list.append(available_node);
        int is_chosen_id = nearest_intersec->node_id;
        float dis_to_intersec = distance(available_node->x,available_node->y,nearest_intersec->x,nearest_intersec->y);
        std::cout << "dis_to_intersec:" << dis_to_intersec << std::endl;
        while(dis_to_intersec <= 18){
            QList<MapSearchNode*>::iterator net_iter = available_node->successorList.begin();
            for(;net_iter != available_node->successorList.end(); net_iter++){
                if((*net_iter)->node_id != is_chosen_id){
                    is_chosen_id = available_node->node_id;
                    available_node = *net_iter;
                    /*std::cout << "is_chosen_id: " << std::endl
                              << "available_node:" << available_node->node_id << std::endl;*/
                    temp_avai_fork_list.append(*net_iter);
                    dis_to_intersec = distance((*net_iter)->x,(*net_iter)->y,nearest_intersec->x,nearest_intersec->y);
                    std::cout << "dis_to_intersec:" << dis_to_intersec << std::endl;
                    break;
                }
            }
        }
    }//lsh//找到一条18米左右的可通行倒车路径

    //将该倒车路径添加到两段重规划路径中
    std::cout << "temp_avai_fork_list: " << std::endl;
    QList<MapSearchNode*>::iterator iter = temp_avai_fork_list.begin();
    for(;iter != temp_avai_fork_list.end(); iter++) {
        std::cout << "   " << (*iter)->node_id << std::endl;
        first_path.append(*iter);
        second_path.prepend(*iter);
    }

    if(!m_path_map_list.isEmpty()){
        m_path_map_list.clear();
    }
    QList<MapSearchNode*>::iterator copy_iter = first_path.begin();
    for(; copy_iter != first_path.end(); copy_iter++){
        m_path_map_list.append(*copy_iter);
    }//lsh//m_path_map_list存储第一段路

    std::cout << "split path done." << std::endl;
}

void AttachXmlFile::ReCreateTaskList(float flat,float flon) {
    //lsh//将距离最近的车辆投影点作为重规划任务列表m_Replan_Task_list的第一个任务点
    //lsh//记录触发重规划时下一个路点的ID，ReStartNextID
    //lsh//将从下一段道路上的任务点开始的剩余任务点加入重规划任务列表
    if(!m_Replan_Task_list.isEmpty())
        m_Replan_Task_list.clear();
    if(!m_StartNode.RoadList.isEmpty())
        m_StartNode.RoadList.clear();
    m_StartNode.lat = flat;
    m_StartNode.lon = flon;//lsh//重规划的起始点
    Position_Trans_From_ECEF_To_UTM(m_StartNode.lat, m_StartNode.lon,0,0,&m_StartNode.x, &m_StartNode.y);

    QList<MapSearchNode*>::iterator Iter_cp;
    int first_task_num_cp = -1;
    Iter_cp = m_pos;//lsh//表示规划结果v_mapping_list中车辆第四个路点
    bool on_same_road_flag_cp = true;
    for(; Iter_cp != v_mapping_list.end(); Iter_cp++) {//lsh//v_mapping_list为m_path_map_list的复制，为规划结果
        if(on_same_road_flag_cp && ((*Iter_cp)->intersection)) {
            on_same_road_flag_cp = false;
        }//lsh//相当于从下条道路开始*Iter_cp++
        //找到未遍历任务点的第一个任务点
        if((*Iter_cp)->type > 0//lsh//不是起点
        && std::abs((*Iter_cp)->node_id - m_pnexttask.Task_num) <=2//lsh//找到下条道路上的第一个任务投影点，其需要在下一个任务点序号2个左右
        && !on_same_road_flag_cp){
            first_task_num_cp = (*Iter_cp)->node_id;
            if(first_task_num_cp < 2){
                ROS_WARN("first_task_num_cp < 2, can not find the last task num when replanning.");
                return;
            }
            break;
        }//lsh//实质上是找到下一段路第一个任务点的编号
    }

    m_StartNode.ID = (restart_id++)*10000000+(first_task_num_cp-1);//lsh//每次重规划前，将车当前位置作为第一个任务点，并对其分配ID，restart_id其为10000000的整数倍加上下一段路第一个任务点的上一个任务点的编号

    QList<MapSearchNode*>::iterator PreIter,NextIter;
    PreIter = m_path_map_list.begin();//lsh//规划结果的点序列，只包含xy等信息
    NextIter = PreIter+1;
    for(; NextIter != m_path_map_list.end(); NextIter++) {
        Road_Line tempRoadLine;
        m_cMapMatch.LineAndLinePoint((*PreIter)->x,(*PreIter)->y,(*NextIter)->x,(*NextIter)->y,m_StartNode.x,m_StartNode.y,tempRoadLine.Road_nodex,tempRoadLine.Road_nodey);
        m_cMapMatch.PointToLineDistance(m_StartNode.x,m_StartNode.y,tempRoadLine.Road_nodex,tempRoadLine.Road_nodey,tempRoadLine.Dist);//notice:shibushi dizhi
        m_StartNode.RoadList.append(tempRoadLine);
        PreIter = NextIter;
    }

    QList<Road_Line>::iterator tempRoadLine2,nextRoadLine2;
    tempRoadLine2=m_StartNode.RoadList.begin();
    nextRoadLine2=tempRoadLine2+1;
    for(; nextRoadLine2!=m_StartNode.RoadList.end(); nextRoadLine2++) {
        if( tempRoadLine2 ->Dist > nextRoadLine2 ->Dist) {
            *tempRoadLine2 = *nextRoadLine2;
        }
    }//lsh//找到距离之前规划好道路最近的车辆位置投影点

    QList<MapSearchNode*>::iterator PreIter1,NextIter1,Iter;
    PreIter1=m_path_map_list.begin();
    NextIter1=PreIter1+1;
    for(; NextIter1!=m_path_map_list.end(); NextIter1++) {
        Road_Line tempRoadLine3;
        m_cMapMatch.LineAndLinePoint((*PreIter1)->x,(*PreIter1)->y,(*NextIter1)->x,(*NextIter1)->y,m_StartNode.x,m_StartNode.y,tempRoadLine3.Road_nodex,tempRoadLine3.Road_nodey);
        if( (tempRoadLine3 .Road_nodex == tempRoadLine2->Road_nodex ) && (tempRoadLine3 .Road_nodey == tempRoadLine2->Road_nodey )) {
            tempRoadLine.Road_nodex = tempRoadLine2 ->Road_nodex;
            tempRoadLine.Road_nodey = tempRoadLine2 ->Road_nodey;

            int last_task_type;
            QList<Task_Node>::iterator task_iter2 = m_cMapMatch.TaskList.begin();
            for(; task_iter2 != m_cMapMatch.TaskList.end(); task_iter2++){
                if(task_iter2->Task_num == (*PreIter1)->cur_task_num){
                    last_task_type = task_iter2->type;
                    break;
                }
            }

            //update the task list.
            Task_Node tmptaskNode;
            tmptaskNode.lat=m_StartNode.lat;//lsh//使用车辆当前经纬度
            tmptaskNode.lon=m_StartNode.lon;
            tmptaskNode.x=tempRoadLine.Road_nodex;//lsh//使用车辆投影坐标
            tmptaskNode.y=tempRoadLine.Road_nodey;
            tmptaskNode.Task_num=m_StartNode.ID;
            tmptaskNode.type = last_task_type;//lsh//将当前任务点的属性记录为上一次规划时最近路点的任务属性
            tmptaskNode.on_road = true;                //默认起点位置在路上
            tmptaskNode.vlimit = (*NextIter1)->vlimit;
            tmptaskNode.concave_obs_det = (*NextIter1)->concave_obs_det;
            tmptaskNode.dynamic_obs_det = (*NextIter1)->dynamic_obs_det;
            tmptaskNode.foogy_det = (*NextIter1)->foogy_det;
            tmptaskNode.water_det = (*NextIter1)->water_det;
            tmptaskNode.wall_area = (*NextIter1)->wall_area;
            tmptaskNode.ditch_area = (*NextIter1)->ditch_area;//lsh//使用下一个路点的属性
            m_Replan_Task_list.append(tmptaskNode);
            //lsh//构造重规划任务列表的起点
            //the next node of the Restart node.
            /*while((*NextIter1)->node_id < 10000){//lsh//根据xml文件定义，node_id不可能小于10000，除非为后插入的任务点的投影点，这里表示如果是任务点投影点则向前推进一个路点
                NextIter1++;
                if(NextIter1 == m_path_map_list.end()) break;
            }*/
            if((*NextIter1)->node_id < 10000){//lsh//根据xml文件定义，node_id不可能小于10000，除非为后插入的任务点的投影点，这里表示如果是任务点投影点则向前推进一个路点
                Task_Node tmpaddnode;
                tmpaddnode.lon = (*NextIter1)->lon;
                tmpaddnode.lat = (*NextIter1)->lat;
                tmpaddnode.type = 0;
                tmpaddnode.Task_num = add_goback_id++;
                if(!m_Replan_AddNode_list.isEmpty()){
                    m_Replan_AddNode_list.clear();
                }
                m_Replan_AddNode_list.push_back(tmpaddnode);
                replanning_add_node = 1;
                ReStartNextID = tmpaddnode.Task_num;//lsh//用于把当前重规划时的路点加入到道路连接关系中
                ROS_WARN("next point is goback point,record it as a new way point");
            }else{
                ReStartNextID = (*NextIter1)->node_id;//lsh//触发重规划时车辆当前位置的下一个路点ID
                ROS_WARN("next point is a way point, directly record ReStartNextID");
            }
            //ReStartNextID = (*NextIter1)->node_id;//lsh//触发重规划时车辆当前位置的下一个路点ID

//            bool on_same_road_flag = true;
//            for(int i = last_match_num; i < m_path_map_list.size(); i++){
//                if(on_same_road_flag && (m_path_map_list[i]->intersection /*|| m_path_map_list[i]->type == 3*/)) {
//                    on_same_road_flag = false;
//                }
//                if(m_path_map_list[i]->type > 0 && !on_same_road_flag){    //选择不在同一条路上的余下所有未通过任务点
//                    Task_Node tmptaskNode;
//                    tmptaskNode.lat = m_path_map_list[i]->lat;
//                    tmptaskNode.lon = m_path_map_list[i]->lon;
//                    tmptaskNode.x = m_path_map_list[i]->x;
//                    tmptaskNode.y = m_path_map_list[i]->y;
//                    tmptaskNode.Task_num = m_path_map_list[i]->node_id;
//                    tmptaskNode.type = m_path_map_list[i]->type;
//                    tmptaskNode.on_road = false;
//                    tmptaskNode.vlimit = m_path_map_list[i]->vlimit;
//                    tmptaskNode.concave_obs_det = m_path_map_list[i]->concave_obs_det;
//                    tmptaskNode.dynamic_obs_det = m_path_map_list[i]->dynamic_obs_det;
//                    tmptaskNode.foogy_det = m_path_map_list[i]->foogy_det;
//                    tmptaskNode.water_det = m_path_map_list[i]->water_det;
//                    m_Replan_Task_list.append(tmptaskNode);
//                }
//            }
//            break;
            Iter = m_pos;//lsh//表示规划结果v_mapping_list中车辆第四个路点
            bool on_same_road_flag = true;
            for(; Iter != v_mapping_list.end(); Iter++) {//lsh//v_mapping_list为m_path_map_list的复制，为规划结果
                if(on_same_road_flag && ((*Iter)->intersection)) {
                    on_same_road_flag = false;
                }//lsh//相当于从下条道路开始*Iter++
                //找到未遍历任务点的第一个任务点
                int first_task_num = -1;
                if((*Iter)->type > 0//lsh//不是起点
                   && std::abs((*Iter)->node_id - m_pnexttask.Task_num) <=2//lsh//找到下条道路上的第一个任务投影点，其需要在下一个任务点序号2个左右
                   && !on_same_road_flag){
                    first_task_num = (*Iter)->node_id;
                }//lsh//实质上是找到下一条道路的第一个任务投影点
                if(first_task_num != -1){
                    QList<Task_Node>::iterator task_iter1,task_iter2;
                    task_iter1 = m_cMapMatch.TaskList.begin();
                    for(;task_iter1 != m_cMapMatch.TaskList.end();task_iter1++){
                        if(task_iter1->Task_num == first_task_num){
                            task_iter2 = task_iter1;
                            for(;task_iter2 != m_cMapMatch.TaskList.end();task_iter2++){
                                Task_Node tmptaskNode;
                                tmptaskNode.lat = task_iter2->lat;
                                tmptaskNode.lon = task_iter2->lon;
                                tmptaskNode.x = task_iter2->x;
                                tmptaskNode.y = task_iter2->y;
                                tmptaskNode.Task_num = task_iter2->Task_num;
                                tmptaskNode.type = task_iter2->type;
                                tmptaskNode.on_road = false;
                                tmptaskNode.vlimit = task_iter2->vlimit;
                                tmptaskNode.concave_obs_det = task_iter2->concave_obs_det;
                                tmptaskNode.dynamic_obs_det = task_iter2->dynamic_obs_det;
                                tmptaskNode.foogy_det = task_iter2->foogy_det;
                                tmptaskNode.water_det = task_iter2->water_det;
                                tmptaskNode.wall_area = task_iter2->wall_area;
                                tmptaskNode.ditch_area = task_iter2->ditch_area;
                                m_Replan_Task_list.append(tmptaskNode);
                            }//lsh//把从下一路段上开始的任务点都加入m_Replan_Task_list
                            break;
                        }
                    }
                    break;
                }
            }
            break;
            /*Iter=NextIter1;
            bool on_same_road_flag = true;
            for(; Iter != m_path_map_list.end(); Iter++) {
                if(on_same_road_flag && ((*Iter)->intersection || (*Iter)->type == 3)) {
                    on_same_road_flag = false;
                }
                if((*Iter)->type > 0 && !on_same_road_flag){    //选择不在同一条路上的余下所有未通过任务点
                        Task_Node tmptaskNode;
                        tmptaskNode.lat=(*Iter)->lat;
                        tmptaskNode.lon=(*Iter)->lon;
                        tmptaskNode.x=(*Iter)->x;
                        tmptaskNode.y=(*Iter)->y;
                        tmptaskNode.Task_num=(*Iter)->node_id;
                        tmptaskNode.type = (*Iter)->type;
                        tmptaskNode.on_road = false;
                        tmptaskNode.vlimit = (*Iter)->vlimit;
                        tmptaskNode.concave_obs_det = (*Iter)->concave_obs_det;
                        tmptaskNode.dynamic_obs_det = (*Iter)->dynamic_obs_det;
                        tmptaskNode.foogy_det = (*Iter)->foogy_det;
                        tmptaskNode.water_det = (*Iter)->water_det;
                        m_Replan_Task_list.append(tmptaskNode);
                }
            }
            break;*/
        }
        //break;
        PreIter1 = NextIter1;
    }
}

void AttachXmlFile::OutputTxtFileWithPath(QString base_dir) {
    //QString path = QCoreApplication::applicationDirPath();
    QList<MapSearchNode*>::iterator iter1;
    iter1=m_path_map_list.begin();
    for(; iter1!=m_path_map_list.end(); iter1++)
    {
        qDebug("m_path_map_list ID:%d\n",(*iter1)->node_id);
        qDebug("m_path_map_list lat:%12f\n",(*iter1)->lat);
        qDebug("m_path_map_list lon:%12f\n",(*iter1)->lon);
    }

    QFile file(base_dir + "/path.txt");
    QTextStream out(&file);

    out.setFieldWidth(2);
    out.setFieldAlignment(QTextStream::AlignLeft);
    if (!file.open(QFile::WriteOnly | QIODevice::Truncate)) {
        ROS_FATAL("Output Txt File: fail to open file!");
    }

    int m_path_num=1;

    QList<MapSearchNode*>::iterator PreIter,NextIter;
    PreIter=m_path_map_list.begin();
    NextIter=PreIter+1;

    QString strx=QString::number((*PreIter)->x,'f',1);
    QString stry=QString::number((*PreIter)->y,'f',1);
    qDebug()<<strx<<endl;
    qDebug()<<stry<<endl;
    out <<m_path_num++<<"  "<<(*PreIter)->node_id<<"  "<<strx<<"  "<<stry<<"\n";

    //m_publish_list.append(*(*PreIter));

    float dis_path=0;
    for(; NextIter!=m_path_map_list.end(); NextIter++)
    {
        float x1=(*PreIter)->x;
        float y1=(*PreIter)->y;
        float x2=(*NextIter)->x;
        float y2=(*NextIter)->y;
        float dis_xy;
        float theta;
        float LineSegment,SumSeg;

        theta=PointToTheta(x1,y1,x2,y2);

        //Determine how many nodes need to be inserted,  1 node/10m
        m_cMapMatch.PointToLineDistance(x1,y1,x2,y2,dis_xy);    //计算两点之间的距离
        int n=dis_xy/1;
        qDebug()<<"dis_xy"<<dis_xy<<",n="<<n<<endl;

        LineSegment=dis_xy/n;
        SumSeg=LineSegment;
        qDebug()<<"LineSegment"<<LineSegment<<endl;

        for(int i=0; i<n-1; i++)
        {
            float tempx=x1+SumSeg*cos(theta);
            float tempy=y1+SumSeg*sin(theta);

            QString strx=QString::number(tempx,'f',1);
            QString stry=QString::number(tempy,'f',1);

            out <<m_path_num++<<"  "<<10000<<"  "<<strx<<"  "<<stry<<"\n";

            /*MapSearchNode tempnode;
            tempnode.node_id=m_path_num;
            tempnode.x=tempx;
            tempnode.y=tempy;
            m_publish_list.append(tempnode);*/

            SumSeg=SumSeg+LineSegment;
        }

        dis_path+=SumSeg;
        // only publish 100m
        if (dis_path>=100){
            cout<<"dis_path: "<<dis_path<<endl;
            break;
        }

        QString strx=QString::number(x2,'f',1);
        QString stry=QString::number(y2,'f',1);
        out <<m_path_num++<<" "<<(*NextIter)->node_id<<"  "<<strx<<"  "<<stry<<"\n";

        //m_publish_list.append(*(*NextIter));

        PreIter=NextIter;
    }
    file.close();

}

double AttachXmlFile::distance(double x1, double y1, double x2, double y2) {
    return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
}

void AttachXmlFile::AddIntersectionNode(extractroad_msg::extractroad inter_data){
//lsh//（当检查到新的路口或者行驶到无路网的地区时，）通过其他模块检测路口
//lsh//找到所有检测到的路口中距离最近的路口，如果距离小于一定阈值，则将其和上一个新添加的路口建立连接关系
//lsh//若距离大于阈值，则记录当前位置作为第一个新增路口，当距离最近的检测到的路口的引导点设为当前路口的子节点，将其和上一个新添加的路口建立连接关系
    float lat,lon,x,y;
    lon = inter_data.vehicle_point[0];
    lat = inter_data.vehicle_point[1];
    Position_Trans_From_ECEF_To_UTM(lat,lon,0,0, &x,&y);

    //determine whether this node already exists
    float min_distance = 1000000;       //任意取一个极大值
    int min_distance_id = 0;
    QList<MapSearchNode*>::iterator iter = intersec_list.begin();
    //遍历新检测到的岔道口与路网中距离最近的岔道口
    for(; iter != intersec_list.end(); iter++ ) {
        float temp_distance = distance(x,y, (*iter)->x, (*iter)->y);
        if(min_distance > temp_distance){
            min_distance = temp_distance;
            min_distance_id = (*iter)->node_id;
        }
    }//lsh//找到所有检测到的路口中距离最近的路口
    if(min_distance < add_intersec_ref_dis) {
        //build topological relation
        MapSearchNode *cur_node = NULL, *parent_node = NULL;
        QList<MapSearchNode*>::iterator iter = road_network_list.begin();
        for( ; iter != road_network_list.end(); iter++)
        {
            if((*iter)->node_id == min_distance_id)
                cur_node = (*iter);
            if((*iter)->node_id == topology_parent_id)  //default: the parent node of this intersection is last ordinary node.
                parent_node = (*iter);
        }
        BuildTopologicalRelation(cur_node, parent_node);

        //update the last_node_id_to_last_intersection/last_intersec_id/topology_parent_id
        double distance_to_last_intersec = 0;
        QList<MapSearchNode*>::iterator intersec_iter = intersec_list.begin();
        for( ; intersec_iter != intersec_list.end() ; intersec_iter++){
            if((*intersec_iter)->node_id == last_intersec_id){
                distance_to_last_intersec = distance((*intersec_iter)->x,(*intersec_iter)->y, cur_node->x , cur_node->y);
                break;
            }
        }
        if(distance_to_last_intersec >= add_intersec_ref_dis){
//            last_last_to_intersec_id = last_topology_parent_id;
//            last_node_id_to_last_intersection = topology_parent_id;
//            last_topology_parent_id = topology_parent_id;

//            if(min_distance_id != record_intersec.last().intersec_id){
//                intersec_info tem_intersec;
//                tem_intersec.intersec_id = min_distance_id;
//                tem_intersec.last_ord_id = topology_parent_id;
//                record_intersec.append(tem_intersec);
//            }

            //update the lead_points
            /*QList<MapSearchNode*>::iterator intersection_iter = intersec_list.begin();
            for(; intersection_iter != intersec_list.end(); intersection_iter++){
                if((*intersection_iter)->node_id == min_distance_id){
                    if(!(*intersection_iter)->successorList.isEmpty()){
                        qDeleteAll((*intersection_iter)->successorList);        //delete and clear leadpoints
                        (*intersection_iter)->successorList.clear();
                    }
                    for(int i=0; i< (inter_data.leadpoints.size())/2; i++) {        //add leadpoints
                        MapSearchNode* temp_node1;
                        temp_node1 = new MapSearchNode;
                        temp_node1->lon = inter_data.leadpoints.at(2*i);
                        temp_node1->lat = inter_data.leadpoints.at(2*i+1);
                        Position_Trans_From_ECEF_To_UTM(temp_node1->lat,temp_node1->lon,0,0, &temp_node1->x,&temp_node1->y);
                        temp_node1->node_id = leadpoints_id++;
                        temp_node1->intersection = false;
                        temp_node1->bConnection = false;

                        (*intersection_iter)->successorList.append(temp_node1);
                    }
                    qDebug()<<"updata "<<min_distance_id<<" intersection's leadpoints"<<endl;
                    break;
                }
            }*/
        }
        last_intersec_id = min_distance_id;         //the node existed
        topology_parent_id = min_distance_id;
        //lsh//找到所有检测到的路口中距离最近的路口，如果距离小于一定阈值，则将其和上一个新添加的路口或者车辆当前位置建立连接关系
    } else{
        MapSearchNode* temp_node;
        temp_node = new MapSearchNode;
        temp_node->lon = inter_data.vehicle_point[0];
        temp_node->lat = inter_data.vehicle_point[1];
        Position_Trans_From_ECEF_To_UTM(temp_node->lat,temp_node->lon,0,0, &temp_node->x,&temp_node->y);
        temp_node->node_id = intersec_id++;
        temp_node->type = -1;
        temp_node->vlimit = 8;
        temp_node->intersection = true;
        temp_node->successorNum = inter_data.roadcount;

        //add leadpoints
        for(int i=0; i< (inter_data.leadpoints.size())/2; i++) {
            MapSearchNode* temp_node1;
            temp_node1 = new MapSearchNode;
            temp_node1->lon = inter_data.leadpoints.at(2*i);
            temp_node1->lat = inter_data.leadpoints.at(2*i+1);
            Position_Trans_From_ECEF_To_UTM(temp_node1->lat,temp_node1->lon,0,0, &temp_node1->x,&temp_node1->y);
            temp_node1->node_id = leadpoints_id++;
            temp_node1->intersection = false;
            temp_node1->bConnection = false;
            temp_node1->type = -1;
            temp_node1->vlimit = 8;
            temp_node->successorList.append(temp_node1);
        }
        intersec_list.append(temp_node);            //add this intersection to intersec_list
        std::cout << "add intersection node,id:" << temp_node->node_id << std::endl;
        InsertIntersecToList(temp_node);            //insert to road_network_list
        //用来保存新插入岔道口的上一个普通节点，在重规划第一阶段倒车时方便遍历倒车终点
        intersec_info tem_intersec;
        tem_intersec.intersec_id = temp_node->node_id;
        tem_intersec.last_ord_id = topology_parent_id;
        record_intersec.append(tem_intersec);
        last_intersec_id = temp_node->node_id;
        topology_parent_id = temp_node->node_id;
    }//lsh//若距离大于阈值，则记录当前位置作为第一个新增路口，当距离最近的检测到的路口的引导点设为当前路口的子节点，将其和上一个新添加的路口建立连接关系
    ShowPathWithPathList();         //update display

}

void AttachXmlFile::InsertIntersecToList(MapSearchNode *intersec_node) {

    MapSearchNode* temp_intersec_node;
    temp_intersec_node = new MapSearchNode;

    temp_intersec_node->lat = intersec_node->lat;
    temp_intersec_node->lon = intersec_node->lon;
    temp_intersec_node->x = intersec_node->x;
    temp_intersec_node->y = intersec_node->y;
    temp_intersec_node->node_id = intersec_node->node_id;
    temp_intersec_node->intersection = intersec_node->intersection;
    temp_intersec_node->type = intersec_node->type;
    temp_intersec_node->vlimit = intersec_node->vlimit;

    road_network_list.append(temp_intersec_node);

    //build topological relationship
    if(road_network_list.isEmpty()){
        //QMessageBox::information(NULL,"infomation","In InsertIntersecToList:\n ERROR: road_network_list is empty \n");
        return;
    }else{
        MapSearchNode *cur_node=NULL, *parent_node=NULL;

        QList<MapSearchNode*>::iterator iter = road_network_list.begin();
        for( ; iter!=road_network_list.end(); iter++)
        {
            if((*iter)->node_id == intersec_node->node_id)
                cur_node=(*iter);
            if((*iter)->node_id == topology_parent_id)            //default: the parent node of this intersection is last ordinary node.
                parent_node=(*iter);
        }

        BuildTopologicalRelation(cur_node, parent_node);
    }
}

void AttachXmlFile::BuildTopologicalRelation(MapSearchNode* cur_node, MapSearchNode* parent_node){
    if(cur_node && parent_node && cur_node!=parent_node) {
        cur_node->parentList.append(parent_node);
        cur_node->parentNum++;
        parent_node->successorList.append(cur_node);
        parent_node->successorNum++;
        //if(QString::compare(xml_node_info->way_version, QString("oneway"), Qt::CaseInsensitive))
        //{
        parent_node->parentList.append(cur_node);
        parent_node->parentNum++;
        cur_node->successorList.append(parent_node);
        cur_node->successorNum++;
        //}
    }else{
        ROS_WARN("In BuildTopologicalRelation:\n ERROR: fail to child-parent relation.\n");
        return;
    }
}

void AttachXmlFile::AddNormalNode(sensor_msgs::NavSatFix vehicle_gps){
    //lsh//（如果没检测到路口）记录车辆当前GPS，并以一定的间距将其加入到创建的路网列表中，并建立连接关系
    //lsh//如果位于重规划阶段，采样点间距不需要大于add_ordinary_node_ref_dis，只要大于2，或者上一个点是新加入的路口，则直接将GPS位置记入路网
    if(road_network_list.isEmpty()){
        MapSearchNode* temp_node;
        temp_node = new MapSearchNode;
        temp_node->lon = vehicle_gps.longitude;
        temp_node->lat = vehicle_gps.latitude;
        Position_Trans_From_ECEF_To_UTM(temp_node->lat,temp_node->lon,0,0, &temp_node->x,&temp_node->y);
        temp_node->node_id = ordinary_id++;
        temp_node->type = -1;
        temp_node->intersection = false;
        temp_node->vlimit = 8;
        road_network_list.append(temp_node);

        topology_parent_id = temp_node->node_id;        //update parent node

        std::cout<<"add ordinary node,id:"<<temp_node->node_id<<std::endl;
        //初始化显示
        global_env_zoom=0.5;
        delta_origin_pos.x=temp_node->x;
        delta_origin_pos.y=temp_node->y;//初始位置偏移量为0
        //lsh//如果road_network_list为空的，记录当前位置作为第一个点
    }else{
        float lat,lon,x,y;
        lat = vehicle_gps.latitude;
        lon = vehicle_gps.longitude;
        Position_Trans_From_ECEF_To_UTM(lat,lon,0,0,&x,&y);

        MapSearchNode* parent_iter=NULL;
        QList<MapSearchNode*>::iterator iter = road_network_list.begin();
        for(; iter != road_network_list.end(); iter++){
            if((*iter)->node_id == topology_parent_id)
                parent_iter = (*iter);
        }
        if(parent_iter == NULL){
            ROS_WARN("In AddNormalNode:\n ERROR: Fail to find parent_node.\n");
            return;
        }

        float temp_distance = distance(x,y,parent_iter->x,parent_iter->y);
        if(replan_flag){    //当重规划时，无条件将车辆当前位置插入到路网中
            replan_flag = false;
            if((*parent_iter).node_id < 100000 || temp_distance > 2)   //在实时构建的拓扑地图中，ID号小于100000的都是岔道口
                temp_distance = add_ordinary_node_ref_dis +1;
        }//lsh//如果位于重规划阶段，采样点间距不需要大于add_ordinary_node_ref_dis，只要大于2，或者上一个点是新加入的路口，则直接将GPS位置记入路网

        if(temp_distance > add_ordinary_node_ref_dis){
            MapSearchNode* temp_node;
            temp_node = new MapSearchNode;
            temp_node->lon = vehicle_gps.longitude;
            temp_node->lat = vehicle_gps.latitude;
            Position_Trans_From_ECEF_To_UTM(temp_node->lat,temp_node->lon,0,0, &temp_node->x,&temp_node->y);
            temp_node->node_id = ordinary_id++;
            temp_node->type = -1;
            temp_node->intersection = false;
            temp_node->vlimit = 8;
            road_network_list.append(temp_node);

            //std::cout<<"add ordinary node,id:"<<temp_node->node_id<<std::endl;
            BuildTopologicalRelation(temp_node, parent_iter);
            ShowPathWithPathList();         //display
            topology_parent_id = temp_node->node_id;        //update parent node
        }else{
            return;
        }
    }
}

void AttachXmlFile::FindConnectTaskID(sensor_msgs::NavSatFix current_vehicle_gps){
    /******find the position of last intersection in m_astarsearch.NodeList******/
    if(!m_pTaskNode->RoadList.isEmpty())       //notice
        m_pTaskNode->RoadList.clear();

    /*float last_intersec_x,last_intersec_y;
    QList<MapSearchNode*>::iterator road_iter = road_network_list.begin();
    for(; road_iter != road_network_list.end(); road_iter++){
        if((*road_iter)->node_id == last_intersec_id){
            last_intersec_x = (*road_iter)->x;
            last_intersec_y = (*road_iter)->y;
            break;
        }
    }
    m_pTaskNode->x = last_intersec_x;
    m_pTaskNode->y = last_intersec_y;*/
    float current_x,current_y;
    Position_Trans_From_ECEF_To_UTM(current_vehicle_gps.latitude,current_vehicle_gps.longitude,0,0,&current_x,&current_y);
    m_pTaskNode->x = current_x;
    m_pTaskNode->y = current_y;
    QList<Road>::iterator roadIter;
    roadIter = m_RoadList.begin();
    for(; roadIter != m_RoadList.end(); roadIter++)
    {
        QList<RoadNode>::iterator tempnode1,nextnode1;
        tempnode1 = (roadIter->RoadNodeList).begin();
        nextnode1 = tempnode1+1;
        for(; nextnode1 != (roadIter->RoadNodeList).end(); nextnode1++)
        {
            Road_Line tempRoadLine1;
            m_cMapMatch.LineAndLinePoint(tempnode1->x,tempnode1->y,nextnode1->x,nextnode1->y,m_pTaskNode->x,m_pTaskNode->y,tempRoadLine1.Road_nodex,tempRoadLine1.Road_nodey);
            m_cMapMatch.PointToLineDistance(m_pTaskNode->x,m_pTaskNode->y,tempRoadLine1.Road_nodex,tempRoadLine1.Road_nodey,tempRoadLine1.Dist);//notice:shibushi dizhi
            m_pTaskNode ->RoadList.append(tempRoadLine1);
            tempnode1 = nextnode1;
        }
    }

    QList<Road_Line>::iterator tempRoadLine2,nextRoadLine2;
    tempRoadLine2 = m_pTaskNode->RoadList.begin();
    nextRoadLine2 = tempRoadLine2+1;
    for(; nextRoadLine2 != m_pTaskNode->RoadList.end(); nextRoadLine2++)
    {
        if( tempRoadLine2->Dist > nextRoadLine2->Dist)
        {
            *tempRoadLine2 = *nextRoadLine2;
        }
    }

    int flag=0;
    QList<Road>::iterator roadIter1;
    roadIter1 = m_RoadList.begin();
    for(; roadIter1 != m_RoadList.end(); roadIter1++) {
        QList<RoadNode>::iterator tempnode2, nextnode2;
        tempnode2 = (roadIter1->RoadNodeList).begin();
        nextnode2 = tempnode2 + 1;
        for (; nextnode2 != (roadIter1->RoadNodeList).end(); nextnode2++) {
            Road_Line tempRoadLine3;
            m_cMapMatch.LineAndLinePoint(tempnode2->x, tempnode2->y, nextnode2->x, nextnode2->y, m_pTaskNode->x,
                                         m_pTaskNode->y, tempRoadLine3.Road_nodex, tempRoadLine3.Road_nodey);
            if ((tempRoadLine3.Road_nodex == tempRoadLine2->Road_nodex) &&
                (tempRoadLine3.Road_nodey == tempRoadLine2->Road_nodey)) {
                //qDebug()<<"connect_task_id:"<<connect_task_id<<endl;
                for(;nextnode2 != (roadIter1->RoadNodeList).end();){
                    float temp_dis = distance(current_x,current_y,nextnode2->x,nextnode2->y);
                    if(temp_dis > eli_task_node_ref_dis
                       || (!m_cMapMatch.TaskList.isEmpty() && nextnode2->NodeID == m_cMapMatch.TaskList.last().Task_num)
                       || (nextnode2 + 1) == (roadIter1->RoadNodeList).end()){
                        connect_task_id = nextnode2->NodeID;       //the next task node relative to current position
                        break;
                    }else{
                        nextnode2++;
                    }
                }
                flag = 1;
                break;
            }
            tempnode2 = nextnode2;
        }
        if(flag == 1)
            break;
    }
}

int AttachXmlFile::AddPassableFork() {

    //Select a passable fork
    MapSearchNode* connect_task_node = NULL;
    MapSearchNode* fb_intersec_node = NULL;
    MapSearchNode* fallback_node = NULL;
    QList<MapSearchNode*>::iterator iter = original_road_network_list.begin();
    for(;iter != original_road_network_list.end(); iter++) {
        if ((*iter)->node_id == connect_task_id) {
            connect_task_node = *iter;
        }
    }

    QList<MapSearchNode*>::iterator road_iter = road_network_list.begin();
    for(;road_iter != road_network_list.end(); road_iter++){
        if((*road_iter)->node_id == first_plan_intersec_id){
            fb_intersec_node = *road_iter;
        }
        if((*road_iter)->node_id == fallback_node_id){
            fallback_node = *road_iter;
        }
    }

    if(connect_task_node == NULL || fb_intersec_node == NULL || fallback_node == NULL){
        ROS_WARN("In AddPassableFork():\n ERROR:can't get relactive node \n");
        return 0;
    }

    float min_angle = 180;
    MapSearchNode* select_leadpoint_node = NULL;
    MapSearchNode* min_angle_leadpoint_node = NULL;
    QList<MapSearchNode*>::iterator intersec_iter = intersec_list.begin();
    for(; intersec_iter != intersec_list.end(); intersec_iter++){
        if((*intersec_iter)->node_id == first_plan_intersec_id){
            QList<MapSearchNode*>::iterator intersec_suc_iter = (*intersec_iter)->successorList.begin();
            for(;intersec_suc_iter != (*intersec_iter)->successorList.end(); intersec_suc_iter++){
                if((*intersec_suc_iter)->bConnection == false){
                    float temp_angle = getAngelOfTwoVector(fb_intersec_node,connect_task_node,*intersec_suc_iter);
                    //qDebug()<<"temp_angle"<<temp_angle<<endl;
                    if(temp_angle <= min_angle){
                        min_angle = temp_angle;
                        min_angle_leadpoint_node = *intersec_suc_iter;
                    }
                }else{
                    continue;
                }
            }
            select_leadpoint_node = min_angle_leadpoint_node;       //select a min_angle leadpoint
            if(select_leadpoint_node == NULL){
                ROS_FATAL("In AddPassableFork():\n ERROR:can't get min_angle_leadpoint_node \n");
                return 0;
            }
            break;
        }
    }
    ////add last node/last intersec/lead point to the beginning of m_path_map_list
    m_path_map_list.prepend(select_leadpoint_node);
    //m_path_map_list.prepend(fb_intersec_node);
    m_path_map_list.prepend(fallback_node);

    if(!m_path_map_list.isEmpty()) {
        GeneratePathInfo();
    }
    ShowPathWithPathList();  //display
    return 1;
}

void AttachXmlFile::JudgeWhichForkHasPassed() {
    MapSearchNode* pass_road_node = NULL;
    MapSearchNode* fb_intersec_node = NULL;
    QList<MapSearchNode*>::iterator fb_iter = road_network_list.begin();
    for(; fb_iter != road_network_list.end(); fb_iter++){
        if((*fb_iter)->node_id == fb_intersec_id){
            fb_intersec_node = *fb_iter;
            break;
        }
    }
//    for(;i >= 0; i--){
//        if(road_network_list.at(i)->node_id == fb_intersec_id){
//            fb_intersec_node = road_network_list.at(i);
//            /*if((i+2) <= (road_network_list.size() - 1)){
//                pass_road_node = road_network_list.at(i+2);
//            }else{
//                pass_road_node = road_network_list.at(i+1);
//            }*/
//            if(!fb_intersec_node->searchChild.last()->searchChild.isEmpty()){
//                pass_road_node = fb_intersec_node->searchChild.last()->searchChild.last();
//            }else if(!fb_intersec_node->searchChild.isEmpty()){
//                pass_road_node = fb_intersec_node->searchChild.last();
//            }
//
//            break;
//        }
//    }
    int i = road_network_list.size() - 1;
    for(; i >= 0 ; i--) {
        float temp_dis = distance(road_network_list.at(i)->x,
                                  road_network_list.at(i)->y,
                                  fb_intersec_node->x,
                                  fb_intersec_node->y);
        if(temp_dis <= 10){
            pass_road_node = road_network_list.at(i);
            break;
        }
    }
    if(pass_road_node == NULL){
        ROS_WARN("In JudgeWhichForkHasPassed():\n ERROR:can't get pass_road_node node \n");
        return;
    }

    float min_angle = 180;
    MapSearchNode* min_angle_leadpoint_node = NULL;
    QList<MapSearchNode*>::iterator intersec_iter = intersec_list.begin();
    for(; intersec_iter != intersec_list.end(); intersec_iter++){
        if((*intersec_iter)->node_id == fb_intersec_id){
            QList<MapSearchNode*>::iterator intersec_suc_iter = (*intersec_iter)->successorList.begin();
            for(;intersec_suc_iter != (*intersec_iter)->successorList.end(); intersec_suc_iter++){
//                if((*intersec_suc_iter)->bConnection == false){
                float temp_angle = getAngelOfTwoVector(fb_intersec_node,pass_road_node,*intersec_suc_iter);
                if(temp_angle <= min_angle){
                    min_angle = temp_angle;
                    min_angle_leadpoint_node = *intersec_suc_iter;
                }
//                }else{
//                    continue;
//                }
            }
            if(min_angle_leadpoint_node != NULL){
                if(min_angle_leadpoint_node->bConnection == false){
                    min_angle_leadpoint_node->bConnection = true;
                    (*intersec_iter)->successorNum--;
                }else{
                    ROS_WARN("repeat the same path!");
                }
            }else{
                ROS_WARN("In JudgeWhichForkHasPassed():\n ERROR:can't get min_angle_leadpoint_node \n");
                return;
            }
            break;
        }
    }
}

double AttachXmlFile::getAngelOfTwoVector(MapSearchNode* base_node,
                                          MapSearchNode* node1,
                                          MapSearchNode* node2) {
    float theta;
    double a[2]={node1->x - base_node->x, node1->y - base_node->y};
    double b[2]={node2->x - base_node->x, node2->y - base_node->y};
    double ab,a1,b1,cosr;
    ab=a[0]*b[0]+a[1]*b[1];
    a1=std::sqrt(std::pow(a[0], 2)+std::pow(a[1], 2));
    b1=std::sqrt(std::pow(b[0], 2)+std::pow(b[1], 2));
    cosr=ab/a1/b1;
    theta = std::acos(cosr)*180/M_PI;
    return theta;
}

void AttachXmlFile::findNodeType(sensor_msgs::NavSatFix vehicle_gps) {
//lsh//记录当前车辆位置在每一条道路的每两个路点间的投影点，和间距
//lsh//找到车辆位置距离投影点最小的投影点
//lsh//找到距离最小的投影点的前一个路点，记录其类型到cur_road_type
    if(!m_pTaskNode->RoadList.isEmpty())       //notice
        m_pTaskNode->RoadList.clear();
    float current_x,current_y;
    Position_Trans_From_ECEF_To_UTM(vehicle_gps.latitude,vehicle_gps.longitude,0,0,&current_x,&current_y);
    m_pTaskNode->x = current_x;
    m_pTaskNode->y = current_y;
    QList<Road>::iterator roadIter;
    roadIter=m_RoadList.begin();
    for(; roadIter != m_RoadList.end(); roadIter++)
    {
        QList<RoadNode>::iterator tempnode1,nextnode1;
        tempnode1=(roadIter->RoadNodeList).begin();     //notice
        nextnode1=tempnode1+1;
        for(; nextnode1!=(roadIter->RoadNodeList).end(); nextnode1++)
        {
            Road_Line tempRoadLine1;
            m_cMapMatch.LineAndLinePoint(tempnode1->x,tempnode1->y,nextnode1->x,nextnode1->y,m_pTaskNode->x,m_pTaskNode->y,tempRoadLine1.Road_nodex,tempRoadLine1.Road_nodey);
            m_cMapMatch.PointToLineDistance(m_pTaskNode->x,m_pTaskNode->y,tempRoadLine1.Road_nodex,tempRoadLine1.Road_nodey,tempRoadLine1.Dist);//notice:shibushi dizhi
            m_pTaskNode ->RoadList.append(tempRoadLine1);
            tempnode1 = nextnode1;
        }
    }//lsh//记录当前车辆位置在每一条道路的每两个路点间的投影点，和间距

    QList<Road_Line>::iterator tempRoadLine2,nextRoadLine2;
    tempRoadLine2=m_pTaskNode->RoadList.begin();
    nextRoadLine2=tempRoadLine2+1;
    for(; nextRoadLine2!=m_pTaskNode->RoadList.end(); nextRoadLine2++)
    {
        if( tempRoadLine2->Dist > nextRoadLine2->Dist)
        {
            *tempRoadLine2 = *nextRoadLine2;        //notice:zheli yaobuyao *
        }
    }//lsh//找到车辆位置距离投影点最小的投影点

    int flag=0;
    QList<Road>::iterator roadIter1;
    roadIter1=m_RoadList.begin();
    for(; roadIter1!=m_RoadList.end(); roadIter1++) {
        QList<RoadNode>::iterator tempnode2, nextnode2;
        tempnode2 = (roadIter1->RoadNodeList).begin();     //notice
        nextnode2 = tempnode2 + 1;
        for (; nextnode2 != (roadIter1->RoadNodeList).end(); nextnode2++) {
            Road_Line tempRoadLine3;
            m_cMapMatch.LineAndLinePoint(tempnode2->x, tempnode2->y, nextnode2->x, nextnode2->y, m_pTaskNode->x,
                                         m_pTaskNode->y, tempRoadLine3.Road_nodex, tempRoadLine3.Road_nodey);
            if ((tempRoadLine3.Road_nodex == tempRoadLine2->Road_nodex) &&
                (tempRoadLine3.Road_nodey == tempRoadLine2->Road_nodey)) {

                cur_road_type = tempnode2 -> type;
                // qDebug("the current road type: %d",cur_road_type);
                flag = 1;
                break;
            }
            tempnode2 = nextnode2;
        }
        if(flag == 1)
            break;
    }
}

int AttachXmlFile::findFbIntersec(sensor_msgs::NavSatFix vehicle_gps){
    //找到回退岔道口
    if(last_replan_tri_rec != -1){
        MapSearchNode* last_replan_intersec = NULL;
        QList<MapSearchNode*>::iterator intersec_iter = intersec_list.begin();
        for(; intersec_iter != intersec_list.end(); intersec_iter++){
            if((*intersec_iter)->node_id == last_replan_tri_rec){
                last_replan_intersec = *intersec_iter;
                break;
            }
        }
        float first_replan_x,first_replan_y;
        Position_Trans_From_ECEF_To_UTM(vehicle_gps.latitude,
                                        vehicle_gps.longitude,
                                        0,0,
                                        &first_replan_x,
                                        &first_replan_y);
        float temp_dis = distance(first_replan_x,first_replan_y,
                                  last_replan_intersec->x,last_replan_intersec->y);
        if(temp_dis < 50) {
            fb_intersec_id = last_replan_tri_rec;
        }else{
            fb_intersec_id = first_plan_tri_intersec_id;
            last_replan_tri_rec = first_plan_tri_intersec_id;
        }
    }else{
        fb_intersec_id = first_plan_tri_intersec_id;
        last_replan_tri_rec = first_plan_tri_intersec_id;
    }
    if(fb_intersec_id == -1){
        ROS_WARN("In findfbintersec(): no fb_intersec_id found.");
        return 0;
    }

    JudgeWhichForkHasPassed();          //判断哪条路走过
    //判断是否回退岔道口可通行，如果不可以，则向上找到可以通行的岔道口
    QList<MapSearchNode*>::iterator iter = intersec_list.begin();
    for(; iter != intersec_list.end(); iter++){
        if((*iter)->node_id == fb_intersec_id){
            if((*iter)->successorNum <=0){
                for(;iter >= intersec_list.begin();iter--){
                    if((*iter)->successorNum > 1){      //至少需要2个及2个以上可通行岔道口才会被选中
                        fb_intersec_id = (*iter)->node_id;
                        last_replan_tri_rec = fb_intersec_id;
                        JudgeWhichForkHasPassed();          //判断哪条路走过
                        break;
                    }
                }
            }
            break;
        }
    }

    QList<MapSearchNode*>::iterator iter1 = intersec_list.begin();
    for(; iter1 != intersec_list.end(); iter1++){
        if((*iter1)->node_id == fb_intersec_id){
            if((*iter1)->successorNum <=0){
                ROS_WARN("the available leadpoint is empty!");
                return 0;
            }
        }
    }

    //找到回退普通路点
    QList<intersec_info>::iterator info_iter = record_intersec.begin();
    for(;info_iter != record_intersec.end(); info_iter++){
        if(info_iter->intersec_id == fb_intersec_id){
            fallback_node_id = info_iter->last_ord_id;
            break;
        }
    }
    if(fallback_node_id == -1){
        ROS_WARN("In PlanTheFirstPath():\n ERROR: can't get fallback node\n");
        return 0;
    }
    MapSearchNode* fb_intersec_node = NULL;
    MapSearchNode* fallback_node = NULL;
    QList<MapSearchNode*>::iterator road_iter = road_network_list.begin();
    for(;road_iter != road_network_list.end(); road_iter++){
        if((*road_iter)->node_id == fb_intersec_id){
            fb_intersec_node = *road_iter;
        }
    }
    int i = road_network_list.size()-1;
    for(;i >= 0; i--){
        if(road_network_list.at(i)->node_id == fallback_node_id){
            /*fallback_node = road_network_list.at(i);
            temp_dis = distance(fb_intersec_node->x,fb_intersec_node->y,
                                fallback_node->x, fallback_node->y);
            if(temp_dis > fallback_ref_dis) {
                fallback_node_id = fallback_node->node_id;
            }else{
                do{
                    if(!fallback_node->searchParent.isEmpty()){
                        fallback_node = fallback_node->searchParent.last();
                    }else{
                        break;
                    }
                    temp_dis = distance(fb_intersec_node->x,fb_intersec_node->y,
                                        fallback_node->x, fallback_node->y);
                }while(temp_dis <= fallback_ref_dis);
                fallback_node_id = fallback_node->node_id;
            }
            break;*/
            for(;i >= 0;){
                float temp_dis = distance(fb_intersec_node->x, fb_intersec_node->y,
                                          road_network_list.at(i)->x,
                                          road_network_list.at(i)->y);
                if(temp_dis > fallback_ref_dis || i == 0){
                    fallback_node_id = road_network_list.at(i)->node_id;
                    break;
                } else{
                    i--;
                }
            }
            break;
        }
    }
    return 1;
}

void AttachXmlFile::taskMapMatch(float flat, float flon, QList<Task_Node> *task_list,
                                 int *last_id , int *next_id){
//lsh//清空RoadList
//lsh//记录当前坐标
//lsh//获取当前车辆坐标在每两个任务点坐标上的投影点Road_nodex（投影点在两任务点之间）
//lsh//计算当前车辆坐标与任务投影点距离
//lsh//找到投影点距离最近的任务点ID
    if(!m_pTaskNode->RoadList.isEmpty())
        m_pTaskNode->RoadList.clear();
    //lsh//m_pTaskNode辅助计算，保存计算结果
    float current_x,current_y;
    Position_Trans_From_ECEF_To_UTM(flat,flon,0,0,&current_x,&current_y);
    m_pTaskNode->x = current_x;     //m_pTaskNode是车的位置
    m_pTaskNode->y = current_y;
    //lsh//当前坐标

    QList<Task_Node>::iterator tempnode1,nextnode1;
    tempnode1 = task_list->begin();
    nextnode1 = tempnode1+1;
    //lsh//tempnode1指向TaskList第一个
    //lsh//nextnode1指向tempnode1下一个
    for(; nextnode1 != task_list->end(); nextnode1++)
    {
        Road_Line tempRoadLine1;
        m_cMapMatch.LineAndLinePoint(tempnode1->x,tempnode1->y,nextnode1->x,nextnode1->y,
                                     m_pTaskNode->x,m_pTaskNode->y,
                                     tempRoadLine1.Road_nodex,tempRoadLine1.Road_nodey);
        //lsh//输入为第一个任务点坐标、下一个任务点坐标、当前车辆坐标、点和直线的交点
        //lsh//获取当前车辆坐标在向量第一个任务点坐标->下一个任务点坐标上的投影点Road_nodex
        m_cMapMatch.PointToLineDistance(m_pTaskNode->x,m_pTaskNode->y,
                                        tempRoadLine1.Road_nodex,tempRoadLine1.Road_nodey,
                                        tempRoadLine1.Dist);
        //lsh//输入为当前坐标、任务投影点、距离
        //lsh//当前坐标与任务投影点间距
        m_pTaskNode ->RoadList.append(tempRoadLine1);
        tempnode1 = nextnode1;
    }//lsh//获取当前车辆坐标在每两个任务点坐标上的投影点Road_nodex（投影点在两任务点之间）
    //lsh//计算当前车辆坐标与任务投影点距离

    QList<Road_Line>::iterator tempRoadLine2,nextRoadLine2;
    tempRoadLine2 = m_pTaskNode->RoadList.begin();
    nextRoadLine2=tempRoadLine2+1;
    for(; nextRoadLine2 != m_pTaskNode->RoadList.end(); nextRoadLine2++)
    {
        if( tempRoadLine2->Dist > nextRoadLine2->Dist)
        {
            *tempRoadLine2 = *nextRoadLine2;
        }
    }//lsh//找到RoadList中Dist最小的

    QList<Task_Node>::iterator tempnode2, nextnode2;
    tempnode2 = task_list->begin();
    nextnode2 = tempnode2 + 1;
    for (; nextnode2 != task_list->end(); nextnode2++) {
        Road_Line tempRoadLine3;
        m_cMapMatch.LineAndLinePoint(tempnode2->x, tempnode2->y, nextnode2->x, nextnode2->y,
                                     m_pTaskNode->x, m_pTaskNode->y,
                                     tempRoadLine3.Road_nodex, tempRoadLine3.Road_nodey);
        //lsh//获取当前车辆坐标在向量第一个任务点坐标->下一个任务点坐标上的投影点Road_nodex
        if ((tempRoadLine3.Road_nodex == tempRoadLine2->Road_nodex) &&
            (tempRoadLine3.Road_nodey == tempRoadLine2->Road_nodey)) {

            *last_id = tempnode2->Task_num;
            *next_id = nextnode2->Task_num;

            break;
        }
        tempnode2 = nextnode2;
    }
}

void AttachXmlFile::mapMatchAndUpdatefortask(float flat, float flon){
//lsh//清空m_pTaskNode->RoadList
//lsh//update_flag_for_task=0，开始匹配任务点
//lsh//查看TaskList中是否包含强制起点，若为强制起点则find_start_task=true，表示找到了强制任务起点
//lsh//找到车辆当前位置在哪两个任务点之间
//lsh//若有强制任务起点，找到强制起始任务点设置为当前任务向量的第一个任务点，下一个任务点设为第二个任务点
//lsh//若无强制任务起点则设当前距离最近任务点为当前任务向量的第一个任务点、下一个任务点为第二个任务点
//lsh//根据后续任务点距离最后一个任务点的距离设置update_flag_for_task
//lsh//update_flag_for_task=2，当前任务点距离任务点结尾还远
//lsh//若车辆位置到当前任务点向量距离比到下一段任务点向量距离远，或者车辆任务投影点到下一个任务点距离小于5，则向后平移任务点
//lsh//update_flag_for_task=3，第三个任务点是任务点列表的结尾
//lsh//车辆到下一个任务点距离小于5时，update_flag_for_task = 4，任务点向前平移
//lsh//update_flag_for_task=4，第二个任务点是任务点列表的结尾
//lsh//车辆当前位置到最后一个第二个任务点的距离小于8时，update_flag_for_task = 5，表示进入终点停车区域
    float x,y;
    Position_Trans_From_ECEF_To_UTM(flat,flon,0,0, &x,&y);
    if(!m_pTaskNode->RoadList.isEmpty())
        m_pTaskNode->RoadList.clear();//lsh//清空m_pTaskNode->RoadList
    if(update_flag_for_task == 0) { //第一步找出在任务点的哪一段上
        if(m_cMapMatch.TaskList.size() < 2) {
            ROS_WARN("Receiveinfo:\n The size of m_cMapMatch.TaskList is less than 2!");
            return ;
        }
        bool find_start_task = false;
        QList<Task_Node>::iterator temp_iter = m_cMapMatch.TaskList.begin();
        for(; temp_iter != m_cMapMatch.TaskList.end(); temp_iter++){
            if(temp_iter->manu == 100){
                find_start_task = true;
                break;
            }
        }//lsh//查看TaskList中是否包含强制起点，若为强制起点则find_start_task=true，表示找到了起点
        m_pVelNode->x = x;
        m_pVelNode->y = y;
        int last_id,next_id;
        taskMapMatch(flat,flon, &m_cMapMatch.TaskList, &last_id,&next_id);
        QList<Task_Node>::iterator task_iter = m_cMapMatch.TaskList.begin();
        for(; task_iter != m_cMapMatch.TaskList.end(); task_iter++){
            if(false/*find_start_task*/){       //lll:这仍然这样吗？
                if(task_iter->manu == 100){
                    m_pcurtask = *task_iter;
                    task_iter++;
                    if(task_iter!= m_cMapMatch.TaskList.end()){
                        m_pnexttask = *(task_iter+1);
                    }
                    break;
                }
            }//lsh//若find_start_task=true，找到强制起始任务点设置为当前任务向量的第一个任务点，下一个任务点设为第二个任务点
            /*else{
                if(task_iter->Task_num == last_id){
                    m_pcurtask = *task_iter;
                }
                if(task_iter->Task_num == next_id){
                    m_pnexttask = *task_iter;
                    break;
                }
            }//lsh//若无强制任务起点则设当前距离最近任务点为当前任务向量的第一个任务点、下一个任务点为第二个任务点*/
            else{
                int cur_task;
                if (m_ptempnode->cur_task_num > 10000000){
                    cur_task=m_ptempnode->cur_task_num % 10000000;       //lll是不是应该 % 10 ?
                }else{
                    cur_task=m_ptempnode->cur_task_num;
                }
                if(cur_task > m_cMapMatch.TaskList.size()){
                    ROS_ERROR("(cur_task_num is bigger than m_cMapMatch.TaskList.size");
                    return;
                }
                if(task_iter->Task_num == cur_task){
                    m_pcurtask = *task_iter;
                    task_iter++;
                    m_pnexttask = *task_iter;
                    QList<MapSearchNode*>::iterator Iter_shadow=v_mapping_list.begin();
                    for(;Iter_shadow != v_mapping_list.end();Iter_shadow++){
                        if((*Iter_shadow)->node_id==m_pnexttask.Task_num){
                            nexttask_shadow=*Iter_shadow;
                            break;
                        }
                    }
                    if(Iter_shadow==v_mapping_list.end()){
                        nexttask_shadow->x=m_pnexttask.x;
                        nexttask_shadow->y=m_pnexttask.y;
                        ROS_ERROR("cannot find the shadow do the m_pnexttask, directly use m_pnexttask.");
                    }//lsh//寻找m_pnextnode在道路上的投影点
                    break;
                }
            }
        }
        if((task_iter+1) != m_cMapMatch.TaskList.end()){
            m_pthirdtask = *(task_iter+1);
        }else{
            update_flag_for_task = 4;//lsh//只剩下一个任务点
            return;
        }
        m_task_pos = task_iter+2;
        if(m_task_pos != m_cMapMatch.TaskList.end())
            update_flag_for_task = 2;//lsh//当前任务点距离最后一个任务点还远
        else
            update_flag_for_task = 3;//lsh//只剩下两个任务点
    }

    if(update_flag_for_task == 2) {
        m_pVelNode ->x = (float)x ;
        m_pVelNode ->y = (float)y ;

        /*m_cMapMatch.LineAndLinePoint(m_pcurtask.x,m_pcurtask.y, m_pnexttask.x,m_pnexttask.y,
                                     m_pVelNode->x,m_pVelNode->y,
                                     m_pRoadLine1->Road_nodex,m_pRoadLine1->Road_nodey);
        //lsh//当前车辆位置与当前任务点向量的交点
        m_cMapMatch.LineAndLinePoint(m_pnexttask.x,m_pnexttask.y, m_pthirdtask.x,m_pthirdtask.y,
                                     m_pVelNode->x,m_pVelNode->y,
                                     m_pRoadLine2->Road_nodex,m_pRoadLine2->Road_nodey);
        //lsh//当前车辆位置与下一段任务点向量的交点
        m_cMapMatch.PointToLineDistance(m_pVelNode->x,m_pVelNode->y,
                                        m_pRoadLine1->Road_nodex,m_pRoadLine1->Road_nodey,
                                        m_pRoadLine1->Dist);
        //lsh//当前坐标到第一段任务段投影点的距离
        m_cMapMatch.PointToLineDistance(m_pVelNode->x,m_pVelNode->y,
                                        m_pRoadLine2->Road_nodex, m_pRoadLine2->Road_nodey,
                                        m_pRoadLine2->Dist);
        //lsh//当前坐标到下一段任务段投影点的距离
        double dis2next = distance(m_pRoadLine1->Road_nodex,m_pRoadLine1->Road_nodey,m_pnexttask.x,m_pnexttask.y);
        //lsh//当前投影点到下一个任务点的距离
        if(m_pRoadLine1->Dist > m_pRoadLine2->Dist || dis2next < 5) {       //到一下道路的距离小于本条道路的距离时，证明换道了*/
        m_cMapMatch.LineAndLinePoint(m_ptempnode->x,m_ptempnode->y, m_pnextnode->x,m_pnextnode->y,
                                     m_pVelNode->x,m_pVelNode->y,
                                     m_pRoadLine1->Road_nodex,m_pRoadLine1->Road_nodey);
        //lsh//当前车辆位置与当前路点向量的交点
        /*double dis2next = distance(m_pRoadLine1->Road_nodex,m_pRoadLine1->Road_nodey,m_pnexttask.x,m_pnexttask.y);*/
        double dis2next = distance(m_pRoadLine1->Road_nodex,m_pRoadLine1->Road_nodey,nexttask_shadow->x,nexttask_shadow->y);
        //lsh//当前投影点到下一个任务点的距离
        if(dis2next < 5) {       //到一下道路的距离小于本条道路的距离时，证明换道了
            if(dis2next < 5){
                ROS_INFO("dis2next < 5 , toggle current path.");
            }else{
                ROS_INFO("dis to next path is less than dis to cur path , toggle current path");
            }
            update_flag_for_task = 2;
            m_pcurtask = m_pnexttask;
            m_pnexttask = m_pthirdtask;

            QList<MapSearchNode*>::iterator Iter_shadow=v_mapping_list.begin();
            for(;Iter_shadow != v_mapping_list.end();Iter_shadow++){
                if((*Iter_shadow)->node_id==m_pnexttask.Task_num){
                    nexttask_shadow=*Iter_shadow;
                    break;
                }
            }
            if(Iter_shadow==v_mapping_list.end()){
                nexttask_shadow->x=m_pnexttask.x;
                nexttask_shadow->y=m_pnexttask.y;
                ROS_ERROR("cannot find the shadow do the m_pnexttask, directly use m_pnexttask.");
            }//lsh//寻找m_pnextnode在道路上的投影点

            if(m_task_pos != m_cMapMatch.TaskList.end())
                m_pthirdtask = *m_task_pos;
            m_task_pos++;
            if(m_task_pos == m_cMapMatch.TaskList.end()) {
                update_flag_for_task = 3;
            }
            ROS_INFO("the car is at %d task point.",m_pcurtask.Task_num);
        }//lsh//若车辆位置到当前任务点向量距离比到下一段任务点向量距离远，或者车辆任务投影点到下一个任务点距离小于5，则向后平移任务点
    }

    if(update_flag_for_task == 3 )  //lsh//只剩下两个任务点
    {
        m_pVelNode ->x = (float)x ;
        m_pVelNode ->y = (float)y ;
        /*m_cMapMatch.LineAndLinePoint(m_pcurtask.x,m_pcurtask.y,m_pnexttask.x,m_pnexttask.y,
                                     m_pVelNode->x,m_pVelNode->y,
                                     m_pRoadLine1->Road_nodex,m_pRoadLine1->Road_nodey);
        //lsh//记录车辆当前位置到任务点上的投影点*/
        m_cMapMatch.LineAndLinePoint(m_ptempnode->x,m_ptempnode->y, m_pnextnode->x,m_pnextnode->y,
                                     m_pVelNode->x,m_pVelNode->y,
                                     m_pRoadLine1->Road_nodex,m_pRoadLine1->Road_nodey);
        //lsh//当前车辆位置与当前路点向量的交点
        /*double dis2next = distance(m_pRoadLine1->Road_nodex,m_pRoadLine1->Road_nodey,m_pnexttask.x,m_pnexttask.y);*/
        double dis2next = distance(m_pRoadLine1->Road_nodex,m_pRoadLine1->Road_nodey,nexttask_shadow->x,nexttask_shadow->y);
        //lsh//记录当前投影点到下一个任务点的距离
        if( dis2next < 5 ) {
            ROS_INFO("dis2next < 5 , toggle current path.");
            update_flag_for_task = 4;
            m_pcurtask = m_pnexttask;
            m_pnexttask = m_pthirdtask;

            QList<MapSearchNode*>::iterator Iter_shadow=v_mapping_list.begin();
            for(;Iter_shadow != v_mapping_list.end();Iter_shadow++){
                if((*Iter_shadow)->node_id==m_pnexttask.Task_num){
                    nexttask_shadow=*Iter_shadow;
                    break;
                }
            }
            if(Iter_shadow==v_mapping_list.end()){
                nexttask_shadow->x=m_pnexttask.x;
                nexttask_shadow->y=m_pnexttask.y;
                ROS_ERROR("cannot find the shadow do the m_pnexttask, directly use m_pnexttask.");
            }//lsh//寻找m_pnextnode在道路上的投影点

            ROS_INFO("the car is at %d task point.",m_pcurtask.Task_num);
        }//lsh//车辆到下一个任务点距离小于5时，update_flag_for_task = 4，任务点向前平移
    }

    if(update_flag_for_task == 4){//lsh//只剩下一个任务点
        m_pVelNode ->x = (float)x ;
        m_pVelNode ->y = (float)y ;
        /*m_cMapMatch.LineAndLinePoint(m_pcurtask.x,m_pcurtask.y,m_pnexttask.x,m_pnexttask.y,
                                     m_pVelNode->x,m_pVelNode->y,
                                     m_pRoadLine1->Road_nodex,m_pRoadLine1->Road_nodey);
        //lsh//记录车辆当前位置到任务点上的投影点*/
        m_cMapMatch.LineAndLinePoint(m_ptempnode->x,m_ptempnode->y, m_pnextnode->x,m_pnextnode->y,
                                     m_pVelNode->x,m_pVelNode->y,
                                     m_pRoadLine1->Road_nodex,m_pRoadLine1->Road_nodey);
        //lsh//当前车辆位置与当前路点向量的交点
        /*double dis2park = distance(m_pRoadLine1->Road_nodex,m_pRoadLine1->Road_nodey,m_pnexttask.x,m_pnexttask.y);*/
        double dis2park = distance(m_pRoadLine1->Road_nodex,m_pRoadLine1->Road_nodey,nexttask_shadow->x,nexttask_shadow->y);
        //lsh//记录当前投影点到下一个任务点的距离
        if(dis2park < 10.0) {
            //ROS_INFO("the car is at park area!");
            update_flag_for_task = 5;
            //lsh//车辆当前位置到第二个任务点的距离小于8时，update_flag_for_task = 5，表示进入终点停车区域
        }
    }
}

void AttachXmlFile::mapMatchAndUpdate(float flat, float flon)   {
//lsh//匹配车辆在道路的位置，记录第一、二、三个路点，并更新m_flag的值
//lsh//匹配车辆在任务点的位置，记录第一、二、三个任务点，并更新update_flag_for_task的值
//lsh//若当前道路的原始道路属性和任务点当前任务属性不相同，并且不是即将结束任务，显示当前任务信息
//lsh//若到达终点停车区域，当前道路属性cur_road_type = 1，否则但前道路属性等于任务属性
//lsh//设置当前路段限速cur_road_vlimit为当前路点的限速
    //qDebug()<<"match thread: "<<QThread::currentThreadId();
    Receiveinfo(flat,flon);
    //lsh//匹配车辆在道路的位置，记录第一、二、三个路点，并更新m_flag的值
    mapMatchAndUpdatefortask(flat,flon);
    //lsh//匹配车辆在任务点的位置，记录第一、二、三个任务点，并更新update_flag_for_task的值
    if(use_coherence_mapping){//lsh//决定是否使用连贯性进行路点匹配，默认为true
        if(cur_road_type != m_pcurtask.type && update_flag_for_task != 5){
            //lsh//若当前道路的原始道路属性和任务点当前任务属性不相同，并且不是即将结束任务
            /*switch (m_pcurtask.type){
                case 0:
                    ROS_INFO("the car is at start point");
                    break;
                case 1:
                    ROS_INFO("the car is at end point");
                    break;
                case 2:
                    ROS_INFO("the car is at normal point");
                    break;
                case 3:
                    ROS_INFO("the car is at weapon area");
                    break;
                case 4:
                    ROS_INFO("the car is at patrol start");
                    break;
                case 5:
                    ROS_INFO("the car is at patrol end");
                    break;
                case 6:
                    ROS_INFO("the car is at wall area");
                    break;
                case 7:
                    ROS_INFO("the car is at ditch area");
                default:
                    ROS_WARN("cur task type error!!!");
                    break;
            }*/
            switch (m_pcurtask.type){
                case 0:
                    ROS_INFO("the car is at start point");
                    break;
                case 1:
                    ROS_INFO("the car is at end point");
                    break;
                case 2:
                    ROS_INFO("the car is at normal point");
                    break;
                case 3:
                    ROS_INFO("the car is at semantic scene area");
                    break;
                case 4:
                    ROS_INFO("the car has left semantic scene area");
                    break;
                case 5:
                    ROS_INFO("the car is at obstacle detection area");
                    break;
                case 6:
                    ROS_INFO("the car has left obstacle detection area");
                    break;
                case 7:
                    ROS_INFO("the car is at search area");
                    break;
                default:
                    ROS_WARN("cur task type error!!!");
                    break;
            }
        }//lsh//若当前道路的原始道路属性和任务点当前任务属性不相同，并且不是即将结束任务，显示当前任务信息
        if(update_flag_for_task == 5){
            cur_road_type = 1;                  //1:park_area
        }else{
            cur_road_type = m_pcurtask.type;        //当前路段属性
        }//lsh//若到达终点停车区域，当前道路属性cur_road_type = 1，否则但前道路属性等于任务属性
    }
    if(m_ptempnode != NULL){
        if(cur_road_vlimit != m_ptempnode->vlimit) {
            ROS_INFO("vel limit: %lf", m_ptempnode->vlimit);
        }
        cur_road_vlimit = m_ptempnode->vlimit;      //当前路段最大限速
    }//lsh//设置当前路段限速cur_road_vlimit为当前路点的限速
}

//生成了global_way_msgs、global_gps_way_msgs，找到match_id(车辆当前位置)，生成80m路，检测下一个任务点是否有特殊属性，计算到特殊属性点的距离
//并将相应的tag置位,改变way_msgs的is_forward、task_area、wall_area、ditch_area……
void AttachXmlFile::publishWay(sensor_msgs::NavSatFix cur_gps,QList<MapSearchNode*> planning_result) {
//lsh//用规划结果在global_way_msgs填充Node中的id和point，其中id为当前路段类型，point为规划结果坐标
//lsh//匹配车辆在规划结果中所处的任务段：
//lsh//如果还没匹配过则认为当前匹配的的点为与当前第一个路点相等planning_result中的路点，设当前匹配id为planning_result中的第i个对应路点
//lsh//如果当前匹配的规划结果索引id在旧id左右10个范围内则更新match_id
//lsh//下一任务点type=6，将wall_node设为下一任务点，当距离小于30m时，wall_tag=1否则为0
//lsh//下一任务点type=7，将ditch_node设为下一任务点，当距离小于30m时，ditch_tag=1否则为0
//lsh//记录下一个折返点在planning_result中的id，并记录折返点
//lsh//记录折返点之前的所有路点到temp_way_msgs
//lsh//若剩余路点大于5个点，则依次进行线性插值和B样条插值到interpolation_way
//lsh//检查interpolation_way编号是或否合理，后一个的编号与前一个编号相减插值为1或0
//lsh//若剩余路点小于5个点，则只进行线性插值到interpolation_way
//lsh//以之前的匹配结果为指引重新匹配车辆的在interpolation_way的位置，并记录其在interpolation_way中的新编号作为当前新的匹配编号
//lsh//找到匹配match_id的之前的第5组interpolation_way点作为开始，以当前新的匹配点之后的80m作为结束，截取要发送的路段放入way_msgs.points
//lsh//设置要发送道路的相关参数
    QMutexLocker locker(&mutex_pub_way);
    if(!way_msgs.points.empty()){
        way_msgs.points.clear();
    }
    if(!pub_way_display.isEmpty()){
        pub_way_display.clear();
    }//lsh//用于在界面显示发布的路
    if(!global_way_msgs.points.empty()){
        global_way_msgs.points.clear();
    }//lsh//用于存放要发布的全局规划路径

    // 发送全局路径
    QList<MapSearchNode*>::iterator global_iter = planning_result.begin();
    for(;global_iter != planning_result.end(); global_iter++){
        lanelet_map_msgs::Node global_node;//lsh//Node类型包括id、type、vlimit、point
        global_node.id = cur_road_type;
        global_node.point.x = (*global_iter)->x;
        global_node.point.y = (*global_iter)->y;
        global_way_msgs.points.push_back(global_node);
    }//lsh//用规划结果在global_way_msgs填充Node中的id和point

    lanelet_map_msgs::Way temp_way_msgs;
    // 在路网中找到车辆当前的位置
    float cur_x,cur_y;
    Position_Trans_From_ECEF_To_UTM(cur_gps.latitude,cur_gps.longitude,0,0,&cur_x,&cur_y);
    //find m_ptempnpde
    int match_id = -1;
    for(int i = 0; i < planning_result.size(); ++i) {
        if(m_ptempnode != NULL && planning_result.at(i)->node_id == m_ptempnode->node_id
           && m_pnextnode != NULL && (i+1) < planning_result.size()
           && planning_result.at(i+1)->node_id == m_pnextnode->node_id) {
            //lsh//匹配车辆在规划结果中所处的任务段
            /*std::cout << "match road:" << i << "/" << i+1 << std::endl;
            std::cout << "strt id:" << m_ptempnode->node_id << "/" << m_pnextnode->node_id << "/" << m_pthirdnode->node_id << std::endl;
            std::cout << "planning at i/i+1:" << planning_result[i]->node_id << "/" << planning_result[i+1]->node_id <<
            "/" << planning_result[i+2]->node_id<< std::endl;
            std::cout << "begin id:" << planning_result[0]->node_id << "/" << planning_result[1]->node_id<< "/"
            << planning_result[2]->node_id<< "/"
            << planning_result[3]->node_id<< "/"
            <<planning_result[4]->node_id<< "/"
            << planning_result[2]->node_id<<std::endl;*/
            if(last_match_num == -1){
                match_id = i;
                last_match_num = i;
                break;//lsh//如果还没匹配过则认为当前匹配的id为planning_result的第i个
            }else if(i >= last_match_num - 10 && i <= last_match_num + 10){
                match_id = i;
                last_match_num = i;
                break;
            }//lsh//如果还当前匹配的规划结果的索引id在旧id左右10个范围内则更新match_id
        }
    }//lsh//设置matchid与lastmatchid
    //std::cout << "match_id:" << match_id << std::endl;

    //垂直墙区域设置
    static int wall_tag=0;
    static float wall_task_dis=-1;
//    static int pass=-1;
    if(m_pnexttask.Task_num!=-1){//lsh//m_pcurtask表示车辆当前所在的任务段
        if(m_pnexttask.type==16){
            wall_node=m_pnexttask;
        }
    }
    if(wall_node.Task_num!=-1){
        wall_task_dis=distance(cur_x,cur_y,wall_node.x,wall_node.y);
//        std::cout<<"wall_task_dis"<<wall_task_dis<<std::endl;
        if(wall_task_dis<30){
//            std::cout<<"true"<<std::endl;
            wall_tag=1;
        } else{
            wall_tag=0;
        }

//        if(wall_task_dis<10)
//            pass=1;
//        if(wall_task_dis>10&&pass==1)
//            wall_tag=0;
    }//lsh//若下一个任务段m_pnexttask.type==6，则其为垂直墙任务点，判断是否距离小于30m，小于则wall_tag=1
//    std::cout<<"wall_task_dis/wall_tag: "<<wall_task_dis<<"/"<<wall_tag<<std::endl;

    //壕沟区域设置
    static int ditch_tag=0;
    static float ditch_task_dis=-1;
    if(m_pnexttask.Task_num!=-1){
        if(m_pnexttask.type==17){
            ditch_node=m_pnexttask;
        }
    }
    if(ditch_node.Task_num!=-1){
        ditch_task_dis=distance(cur_x,cur_y,ditch_node.x,ditch_node.y);
//        std::cout<<"ditch_task_dis"<<ditch_task_dis<<std::endl;
        if(ditch_task_dis<30){
//            std::cout<<"true"<<std::endl;
            ditch_tag=1;
        } else{
            ditch_tag=0;
        }
    }//lsh//若下一任务点type=7，将ditch_node设为下一任务点，当距离小于30m时，ditch_tag=1否则为0
//    std::cout<<"ditch_task_dis/ditch_tag: "<<ditch_task_dis<<"/"<<ditch_tag<<std::endl;

    //找到距离匹配点最近的原路往返的转换点
    int switch_point_id = -1;
    if(match_id != -1 && is_forward){
        for(int i = match_id; i < planning_result.size() - 4; ++i){
            if(planning_result[i]->node_id == planning_result[i+2]->node_id
               || planning_result[i]->node_id == planning_result[i+3]->node_id){
                switch_point_id = i+1;
                switch_point_node = planning_result[i+1]->node_id;
                break;
            }
        }
    }//lsh//记录下一个折返点在planning_result中的id，并记录折返点
    lanelet_map_msgs::Node node;
    for(int i = 0; i < planning_result.size(); ++i) {
        //基于现有的导航策略，由于每次碰到转折点都会重规划，所以可以保证当前点前方不会有转折点
        node.id = i;
        node.point.x = planning_result[i]->x;
        node.point.y = planning_result[i]->y;
        /*switch(planning_result[i]->type){//给每个节点的类型赋值
            case 0:
                node.type = "start_point";
                break;
            case 1:
                node.type = "end_point";
                break;
            case 2:
                node.type = "way_point";
                break;
            case 3:
                node.type = "search_task";
                break;
            case 4:
                node.type = "patrol_task";
                break;
            default:
                node.type = "-1";
                break;
        }
        if(planning_result[i]->node_id >= 1000000){//default: the ID of leadpoint is beyond 1000000
            node.type = "cross_guidance";
        }*/
        node.vlimit = planning_result[i]->vlimit;
        //pub_way_display.append(planning_result[i]);      //用于显示
        temp_way_msgs.points.push_back(node);        //用于插值
        //在未经过转换点之前，不发转换点之后的路点，在经过之后，则没有这个限制
        if(switch_point_id != -1 && switch_point_id == i /*&& !reversing_flag*/)break;
    }//lsh//记录折返点之前的所有路点到temp_way_msgs
    //实时检测路径发送链表temp_way_msgs是否更新
    static int interp_match_id = -1;
    if(temp_way_msgs.points.size() > 1 ){
        if(temp_way_msgs.points.size() != pub_path_size){
            interp_match_id = -1;
            pub_path_size = (int)temp_way_msgs.points.size();
            ROS_INFO("pub way has been updated, reinterpolate");
            //lsh//若发送的路点数发生改变说明发送的路段发生了变化，显示信息
            if(temp_way_msgs.points.size() >= 5){
                lanelet_map_msgs::Way temp_linear_iterp_way;
                this->linearInterpolation(temp_way_msgs,&temp_linear_iterp_way);
                this->bsplineInterpolate(temp_linear_iterp_way,&interpolation_way);
                //lsh//若剩余路点大于5个点，则依次进行线性插值和B样条插值到interpolation_way
//                std::cout<<"插值debug"<<std::endl;
                //检查插值后的ID分布是否合理
                size_t  interp_size  = interpolation_way.points.size();
                for(size_t i = 1; i < interp_size ; i++){
                    if(interpolation_way.points[i].id != interpolation_way.points[i-1].id){
                        std::cout << "interp id" << interpolation_way.points[i].id << std::endl;
                    }
                    if(interpolation_way.points[i].id - interpolation_way.points[i-1].id > 1
                       || interpolation_way.points[i].id - interpolation_way.points[i-1].id < 0){
                        ROS_WARN("ID assign error! %d,%d",interpolation_way.points[i-1].id,interpolation_way.points[i].id);
                    }
                }//lsh//interpolation_way中在原路点向量间形成的新路点id为原路点向量中前一个点的id
                //lsh//检查interpolation_way编号是或否合理，后一个的编号与前一个编号相减插值为1或0
            }else{
                this->linearInterpolation(temp_way_msgs,&interpolation_way);     //线性插值
                ROS_WARN("The size of way points is less than 5, only linearInterpolation");
            }//lsh//若剩余路点小于5个点，则只进行线性插值到interpolation_way
        }
    }else{
        ROS_WARN("size of pub way is less than 1");
    }
    //this->bsplineInterpolate(temp_way_msgs,&interpolation_way);     //样条插值
    //找到插值后路径中的当车辆当前匹配位置ID
    double dis2interpl= std::numeric_limits<double>::infinity();
    //static int interp_match_id = -1;
    if(match_id != -1){
        for(int i = 0; i < interpolation_way.points.size(); ++i) {
            if(interpolation_way.points[i].id >= match_id - 2
               && interpolation_way.points[i].id <= match_id + 2){
                double temp_dis = distance(cur_x,cur_y,
                                           interpolation_way.points[i].point.x,
                                           interpolation_way.points[i].point.y);
                if(temp_dis < dis2interpl){
                    dis2interpl = temp_dis;
                    interp_match_id = i;
                }
            }
        }
    }//lsh//以之前的匹配结果为指引重新匹配车辆的在interpolation_way的位置，并记录其在interpolation_way中的新编号作为当前新的匹配编号

    //找出一条80米的路
    if(/*get_node_from_txt*/true){
        double sum = 0;
        double back_sum = 0;
        int start_id = 0;
        for(int i = interp_match_id; i >= 0; i--)
        {
            double dist = this->distance(interpolation_way.points[i].point.x,
                                         interpolation_way.points[i].point.y,
                                         interpolation_way.points[i + 1].point.x,
                                         interpolation_way.points[i + 1].point.y);
            //cout<<"test"<<endl;
            /*ROS_INFO("1x: %f   ,1y:%f",interpolation_way.points[1].point.x,interpolation_way.points[1].point.y);
            ROS_WARN("199x: %f   ,199y:%f",interpolation_way.points[199].point.x,interpolation_way.points[199].point.y);
            ROS_WARN("200x: %f   ,200y:%f",interpolation_way.points[200].point.x,interpolation_way.points[200].point.y);
            ROS_WARN("201x: %f   ,201y:%f",interpolation_way.points[201].point.x,interpolation_way.points[201].point.y);
            ROS_INFO("2x: %f   ,2y:%f",interpolation_way.points[2].point.x,interpolation_way.points[2].point.y);
            ROS_WARN("202x: %f   ,202y:%f",interpolation_way.points[202].point.x,interpolation_way.points[201].point.y);
            ROS_INFO("size of interpolation_way.points is %d",interpolation_way.points.size());*/
            /*ROS_WARN("interp_match_id is %d",interp_match_id);
            ROS_WARN("i is %d",i);*/
            back_sum += dist;
            /*ROS_INFO("back_sum is %f",back_sum);*/
            if (back_sum < 20.0){
                /*ROS_WARN("back_sum<20,the start id is %d",start_id);*/
                start_id = i;
            }
            else{
                break;
            }
        }//lsh//记录身后20m的路点id
        for(int i = start_id; i < interpolation_way.points.size(); ++i) {
            //ROS_ERROR("start publish way, the start id is %d",start_id);
            if ((i - 1) >= 0 && i > start_id) {
                double dist = this->distance(interpolation_way.points[i - 1].point.x,
                                             interpolation_way.points[i - 1].point.y,
                                             interpolation_way.points[i].point.x,
                                             interpolation_way.points[i].point.y);
                sum += dist;
            }
            //if (sum < 60.0 || interpolation_way.points[i].id <=  match_id + 1) {   //控制路径长度为60米
            if (sum < 60.0) {
                way_msgs.points.push_back(interpolation_way.points[i]);
            }else{
                break;
            }
        }
    }//lsh//发送当前位置前60m，后20m的道路

    //添加属性
    if(is_forward /*&& !reversing_flag*/){
        way_msgs.is_forward = 1;
    }else{
        way_msgs.is_forward = 0;
    }

    /*switch (cur_road_type){
        case 0:
            way_msgs.task_area = "start_area";
            break;
        case 1:
            //定线巡航在的终点在这里控制
            if(numOfPatrols < set_patrol_times){
                way_msgs.task_area = "normal_area";
            }else{
                way_msgs.task_area = "park_area";          //1:终点停车区域
                ROS_INFO("the car is at park area!");
                ROS_WARN("at park area");
            }
            break;
        case 2:
            way_msgs.task_area = "normal_area";
            break;
        case 3:
            way_msgs.task_area = "weapon";        //3：武器站
            break;
        case 4:
            way_msgs.task_area = "patrol_area";        //4:巡线区域
            break;
        case 5:
            way_msgs.task_area = "patrol_stop";
            break;
        case 6:
            way_msgs.task_area = "wall_area";
            break;
        case 7:
            way_msgs.task_area = "ditch_area";
            break;
        default:
            ROS_WARN("task type error!");
            break;
    }//lsh//根据任务向量中第一个任务点的type位设定*/


    switch (cur_road_type){
        case 0:
            way_msgs.task_area = "start_area";          //lsh//0:起点
            break;
        case 1:
            //定线巡航在的终点在这里控制
            if(numOfPatrols < set_patrol_times){
                way_msgs.task_area = "normal_area";
            }else{
                way_msgs.task_area = "park_area";          //lsh//1:终点停车
                ROS_INFO("the car is at park area!");
                ROS_WARN("at park area");
            }
            break;
        case 2:
            way_msgs.task_area = "normal_area";        //lsh//2:普通区域
            break;
        case 3:
            way_msgs.task_area = "semantic_scene_area";     //lsh//3:场景语义勘测
            break;
        case 4:
            way_msgs.task_area = "normal_area";        //lsh//4:出场景语义勘测
            break;
        case 5:
            way_msgs.task_area = "obstacle_detection";    //lsh//5:正负障碍检测
            break;
        case 6:
            way_msgs.task_area = "normal_area";     //lsh//6:出正负障碍检测
            break;
        case 7:
            way_msgs.task_area = "search_area";//lsh//若cur_road_type为7则说明执行过搜索任务，属于普通区域
            break;
        default:
            ROS_WARN("task type error!, the cur_road _type is %d",cur_road_type);
            break;
    }//lsh//根据任务向量中第一个任务点的type位初步设定任务类型*/

    static float task_dis=-1;
    if(m_pnexttask.Task_num!=-1){
        if(m_pnexttask.type==3){
            task_dis=distance(cur_x,cur_y,m_pnexttask.x,m_pnexttask.y);
            if(task_dis<10){
                way_msgs.task_area = "semantic_scene_area";
            }
        }
        else if(m_pnexttask.type==5){
            task_dis=distance(cur_x,cur_y,m_pnexttask.x,m_pnexttask.y);
            if(task_dis<10){
                way_msgs.task_area = "obstacle_detection";
            }
        }else if(m_pnexttask.type==7){
            task_dis=distance(cur_x,cur_y,m_pnexttask.x,m_pnexttask.y);
            if(task_dis<10){
                way_msgs.task_area = "search_area";
            }
        }
    }//lsh//若下一任务点type=7，将ditch_node设为下一任务点，当距离小于30m时，ditch_tag=1否则为0


    if(cur_road_vlimit <= 1.0){
        way_msgs.vel_limit = 1.0;
    }else{
        way_msgs.vel_limit = cur_road_vlimit;          //当前路段的最大限速
    }//lsh//根据原路点向量中第一个路点的限速设置
    //路网开关
    if(m_ptempnode && m_ptempnode->concave_obs_det){
        way_msgs.open_concave_obs_det = 1;
    }else{
        way_msgs.open_concave_obs_det = 0;
    }//lsh//根据路网文件的concave_ob位设定
    if((m_ptempnode &&m_ptempnode->dynamic_obs_det) || open_dynamic_obs_det){
        way_msgs.open_dynamic_obs_det = 1;
    }else{
        way_msgs.open_dynamic_obs_det = 0;
    }//lsh//dynamic_obs_det根据路网文件的dyn_ob位设定，open_dynamic_obs_det根据yaml文件的open_dynamic_obs_det设定
    if((m_ptempnode &&m_ptempnode->foogy_det) || open_foggy_det){
        way_msgs.open_foggy_det = 1;
    }else{
        way_msgs.open_foggy_det = 0;
    }//lsh//dynamic_obs_det根据路网文件的smoke位设定，open_foggy_det根据yaml文件的open_dynamic_obs_det设定
    if(m_ptempnode &&m_ptempnode->water_det){
        way_msgs.open_water_det = 1;
    }else{
        way_msgs.open_water_det = 0;
    }//lsh//根据路网文件的water位设定
    if(wall_tag==1){
        way_msgs.wall_area = 1;
    }else{
        way_msgs.wall_area = 0;
    }//lsh//只与下一个任务点的type和当前车辆与其距离有关
    //lsh//？？？？与路网文件无关
    if(ditch_tag==1){
        way_msgs.ditch_area = 1;
    }else{
        way_msgs.ditch_area = 0;
    }//lsh//只与下一个任务点的type和当前车辆与其距离有关
    if(wall_tag==1||ditch_tag==1){
        way_msgs.vel_limit = 1.5;
    }
//    std::cout<<"wall_area/ditch_area:"<<way_msgs.wall_area+0<<"/"<<way_msgs.ditch_area+0<<std::endl;
    //人为开关
//    way_msgs.open_concave_obs_det = 1;
//    way_msgs.open_water_det = 1;
//    if(open_dynamic_obs_det){
//        /*if(way_msgs.open_dynamic_obs_det != 1){
//            ROS_INFO("open dynamic obs det !!!");
//        }*/
//        way_msgs.open_dynamic_obs_det = 1;
//    }else{
//        /*if(way_msgs.open_dynamic_obs_det != 0){
//            ROS_INFO("close dynamic obs det !!!");
//        }*/
//        way_msgs.open_dynamic_obs_det = 0;
//    }
//    if(open_foggy_det){
//        /*if(way_msgs.open_foggy_det != 1){
//            ROS_INFO("open foggy det !!!");
//        }*/
//        way_msgs.open_foggy_det = 1;
//    }else{
//        /*if(way_msgs.open_foggy_det != 0){
//            ROS_INFO("close foggy det !!!");
//        }*/
//        way_msgs.open_foggy_det = 0;
//    }
}

//void AttachXmlFile::publishWay(sensor_msgs::NavSatFix cur_gps,QList<MapSearchNode*> planning_result) {
//    QMutexLocker locker(&mutex_pub_way);
//    if(!way_msgs.points.empty()){
//        way_msgs.points.clear();
//    }
//    if(!pub_way_display.isEmpty()){
//        pub_way_display.clear();
//    }
//    if(!global_way_msgs.points.empty()){
//        global_way_msgs.points.clear();
//    }
//    // 发送全局路径
//    QList<MapSearchNode*>::iterator global_iter = planning_result.begin();
//    for(;global_iter != planning_result.end(); global_iter++){
//        lanelet_map_msgs::Node global_node;
//        global_node.id = 1;
//        global_node.point.x = (*global_iter)->x;
//        global_node.point.y = (*global_iter)->y;
//        global_way_msgs.points.push_back(global_node);
//    }
//
//    // find the nearest point to vehicle
//    float cur_x,cur_y;
//    Position_Trans_From_ECEF_To_UTM(cur_gps.latitude,cur_gps.longitude,0,0,&cur_x,&cur_y);
//    double min_dis = std::numeric_limits<double>::infinity();
//    int min_id = -1;
//    for(int i = 0; i < planning_result.size(); ++i) {
//        double dist = this->distance(cur_x, cur_y, planning_result[i]->x, planning_result[i]->y);
//        if(dist < min_dis) {
//            min_dis = dist;
//            min_id = i;
//        }
//    }
//    //find m_ptempnpde
//    int match_id = -1;
//    double dis_to_match_point = std::numeric_limits<double>::infinity();
//    for(int i = 0; i < planning_result.size(); ++i) {
//        if(m_ptempnode != NULL && planning_result.at(i)->node_id == m_ptempnode->node_id
//        && m_pnextnode != NULL && (i+1) < planning_result.size()
//        && planning_result.at(i+1)->node_id == m_pnextnode->node_id) {
//            dis_to_match_point = distance(cur_x,cur_y,m_ptempnode->x,m_ptempnode->y);
//            match_id = i;
//        }
//    }
//
//    if(match_id != -1 /*&& dis_to_match_point < 20基于任务点规划不能要这个条件*/){
//        this->nearest_point_id = match_id;
//    } else {
//        this->nearest_point_id = min_id;
//    }
//    int start_id = std::max(0, this->nearest_point_id - 5);
//    lanelet_map_msgs::Node node;
//
//    //找到距离匹配点最近的原路往返的转换点
//    MapSearchNode* switch_point = NULL;
//    if(match_id != -1 && is_forward){
//        for(int i = match_id; i < planning_result.size() - 4; ++i){
//            if(planning_result[i]->node_id == planning_result[i+2]->node_id
//            || planning_result[i]->node_id == planning_result[i+3]->node_id){
//                switch_point = planning_result[i+1];
//                //ROS_INFO("find a switch point at %d",switchPointID);
//                break;
//            }
//        }
//    }
//    /*if(switch_point != NULL && !reversing_flag){
//        double dis2switch = this->distance(cur_x,cur_y,switch_point->x,switch_point->y);
//        if(dis2switch < 0){
//            ROS_INFO("reversing_flag = true");
//            reversing_flag = true;
//            match_update = true;
//        }
//    }
//    if(reversing_flag){
//        if(match_id != -1){
//            for(int i = match_id; i < planning_result.size(); ++i){
//                if(planning_result[i]->intersection){
//                    double dis2nearstItersection = this->distance(cur_x,cur_y
//                            ,planning_result[i]->x,planning_result[i]->y);
//                    if(dis2nearstItersection < 5.0){
//                        reversing_flag = false;
//                        ROS_INFO("reversing_flag = false");
//                    }
//                    break;
//                }
//            }
//        }
//    }*/
//
//    double sum = 0;
//    for(int i = start_id; i < planning_result.size(); ++i) {
//        if(i >= nearest_point_id && (i-1) >= 0){
//            double dist = this->distance(planning_result[i-1]->x, planning_result[i-1]->y,
//                                         planning_result[i]->x, planning_result[i]->y);
//            sum += dist;
//        }
//        if(i < nearest_point_id || sum < 80.0 || way_msgs.points.size() <= 7) {
//            node.id = i;
//            node.point.x = planning_result[i]->x;
//            node.point.y = planning_result[i]->y;
//            switch(planning_result[i]->type){           //给每个节点的类型赋值
//                case 0:
//                    node.type = "start_point";
//                    break;
//                case 1:
//                    node.type = "end_point";
//                    break;
//                case 2:
//                    node.type = "way_point";
//                    break;
//                case 3:
//                    node.type = "search_task";
//                    break;
//                case 4:
//                    node.type = "patrol_task";
//                    break;
//                default:
//                    node.type = "-1";
//                    break;
//            }
//            if(planning_result[i]->node_id >= 1000000){     //default: the ID of leadpoint is beyond 1000000
//                node.type = "cross_guidance";
//            }
//            node.vlimit = planning_result[i]->vlimit;
//            pub_way_display.append(planning_result[i]);      //用于显示
//            way_msgs.points.push_back(node);        //用于发布
//            //在未经过转换点之前，不发转换点之后的路点，在经过之后，则没有这个限制
//            if(switch_point != NULL && planning_result[i]->node_id == switch_point->node_id /*&& !reversing_flag*/) break;
//        } else {
//            break;
//        }
//    }
//
//    if(is_forward /*&& !reversing_flag*/){  //车辆前进后退控制
//        way_msgs.is_forward = 1;
//    }else{
//        way_msgs.is_forward = 0;
//    }
//
//    switch (cur_road_type){
//        case 1:
//            //定线巡航在的终点在这里控制
//            if(numOfPatrols < set_patrol_times){
//                way_msgs.task_area = "normal_area";
//            }else{
//                way_msgs.task_area = "park_area";          //1:终点停车区域
//                ROS_INFO("the car is at park area!");
//                ROS_WARN("at park area");
//            }
//            break;
//        case 3:
//            way_msgs.task_area = "hidden_area";        //3：当前道路在搜索区域
//            break;
//        case 4:
//            way_msgs.task_area = "patrol_area";        //4:巡线区域
//            break;
//        default:
//            way_msgs.task_area = "normal_area";
//            break;
//    }
//
//    if(cur_road_vlimit <= 0){
//        way_msgs.vel_limit = 5.0;
//    }else{
//        way_msgs.vel_limit = cur_road_vlimit;          //当前路段的最大限速
//    }
//
//    //路网开关
//    if(m_ptempnode && m_ptempnode->concave_obs_det){
//        way_msgs.open_concave_obs_det = 1;
//    }else{
//        way_msgs.open_concave_obs_det = 0;
//    }
//    if(m_ptempnode &&m_ptempnode->dynamic_obs_det){
//        way_msgs.open_dynamic_obs_det = 1;
//    }else{
//        way_msgs.open_dynamic_obs_det = 0;
//    }
//    if(m_ptempnode &&m_ptempnode->foogy_det){
//        way_msgs.open_foggy_det = 1;
//    }else{
//        way_msgs.open_foggy_det = 0;
//    }
//    if(m_ptempnode &&m_ptempnode->water_det){
//        way_msgs.open_water_det = 1;
//    }else{
//        way_msgs.open_water_det = 0;
//    }
//
//    //人为开关
//    way_msgs.open_concave_obs_det = 1;
//    way_msgs.open_water_det = 1;
//    if(open_dynamic_obs_det){
//        /*if(way_msgs.open_dynamic_obs_det != 1){
//            ROS_INFO("open dynamic obs det !!!");
//        }*/
//        way_msgs.open_dynamic_obs_det = 1;
//    }else{
//        /*if(way_msgs.open_dynamic_obs_det != 0){
//            ROS_INFO("close dynamic obs det !!!");
//        }*/
//        way_msgs.open_dynamic_obs_det = 0;
//    }
//    if(open_foggy_det){
//        /*if(way_msgs.open_foggy_det != 1){
//            ROS_INFO("open foggy det !!!");
//        }*/
//        way_msgs.open_foggy_det = 1;
//    }else{
//        /*if(way_msgs.open_foggy_det != 0){
//            ROS_INFO("close foggy det !!!");
//        }*/
//        way_msgs.open_foggy_det = 0;
//    }
//
//    //用于显示
//    if(way_msgs.points.size() >= 4){
//        this->bsplineInterpolate(way_msgs,&interpolation_way);     //样条插值
//    }else{
//        this->linearInterpolation(way_msgs,&interpolation_way);    //线性插值
//    }
//}

void AttachXmlFile::bsplineInterpolate(const lanelet_map_msgs::Way &origin_way,
                                       lanelet_map_msgs::Way *result) {
    *result = origin_way;
    const size_t ctrlpt_num = origin_way.points.size();
    if (ctrlpt_num < 5) {
        ROS_ERROR("input origin way size is less than 5!");
        return;
    }
    result->points.clear();
    std::vector<double> ctrlp;
    for (const auto &node:origin_way.points) {
        ctrlp.push_back(node.point.x);
        ctrlp.push_back(node.point.y);
    }
    size_t degree = ctrlpt_num > 9 ? 8 : ctrlpt_num - 1;
    tinyspline::BSpline clamped_spline(ctrlpt_num, 2, degree);
    clamped_spline.setControlPoints(ctrlp);
    double origin_length = 0.0;
    for(int i = 0; i< origin_way.points.size() - 1; i++){
        origin_length += distance(origin_way.points[i].point.x,
                                  origin_way.points[i].point.y,
                                  origin_way.points[i+1].point.x,
                                  origin_way.points[i+1].point.y);
    }
    auto length_size = static_cast<size_t>(origin_length * 3.0);
    printf("Debug: sample num %d\n", length_size);
    const std::size_t sample_num = std::max((std::size_t) 200, length_size);
    size_t count = 0;
    double size_f = static_cast<double>(sample_num - 1);
    double interpolation_way_length = 0.0;
    double x = clamped_spline.eval(0).result().at(0);
    double y = clamped_spline.eval(0).result().at(1);
    for(std::size_t i = 1; i < sample_num; i++){
        double knot_percent = static_cast<double>(i) / size_f;
        double x_plus = clamped_spline.eval(knot_percent).result().at(0);
        double y_plus = clamped_spline.eval(knot_percent).result().at(1);
        interpolation_way_length += distance(x,y,x_plus,y_plus);
        x = x_plus;
        y = y_plus;
    }
    double new_length = 0.0;
    for (std::size_t j = 0; j < sample_num; j++) {
        double knot_percent = static_cast<double>(j) / size_f;
        lanelet_map_msgs::Node node;
        node.point.x = clamped_spline.eval(knot_percent).result().at(0);
        node.point.y = clamped_spline.eval(knot_percent).result().at(1);
        //node.id = static_cast<int>(j);
        /*double dis2origin = std::numeric_limits<double>::infinity();
        size_t temp_count = 0;
        for(size_t i = 0; i < origin_way.points.size(); i++){
            double temp_dis = distance(node.point.x,
                                       node.point.y,
                                       origin_way.points[i].point.x,
                                       origin_way.points[i].point.y);
            if(temp_dis < dis2origin && count <= i){
                dis2origin = temp_dis;
                temp_count = i;
            }
        }
        count = temp_count;*/
        if (j > 0) {
            new_length += distance(result->points.back().point.x,
                                   result->points.back().point.y,
                                   node.point.x,
                                   node.point.y);
            /*while (new_length > origin_length) {
                count++;
                if (count >= origin_way.points.size()) {
                    count = origin_way.points.size() - 1;
                    break;
                }
                origin_length +=
                    distance(origin_way.points.at(count - 1).point.x,
                             origin_way.points.at(count - 1).point.y,
                             origin_way.points.at(count).point.x,
                             origin_way.points.at(count).point.y);
            }*/
        }
        double origin_should_be = (new_length/interpolation_way_length)*origin_length;
        double accumulated_distance = 0.0;
        for(size_t i = 0; i < origin_way.points.size() - 1; i++){
            accumulated_distance += distance(origin_way.points[i].point.x,
                                             origin_way.points[i].point.y,
                                             origin_way.points[i+1].point.x,
                                             origin_way.points[i+1].point.y);
            if(accumulated_distance >= origin_should_be){
                if(count <= i){
                    count = i;
                    break;
                }
            }
        }
        if(count > 0){
            node.id = origin_way.points.at(count-1).id;
            node.type = origin_way.points.at(count-1).type;
            node.vlimit = origin_way.points.at(count-1).vlimit;
        }else{
            node.id = origin_way.points.at(count).id;
            node.type = origin_way.points.at(count).type;
            node.vlimit = origin_way.points.at(count).vlimit;
        }
        result->points.push_back(node);
    }
}

void AttachXmlFile::linearInterpolation(const lanelet_map_msgs::Way &origin_way,
                                        lanelet_map_msgs::Way *result) {
//lsh//放入第一个规划路点
//lsh//取原规划路径中相邻两点，计算朝向0~2*pi和间距
//lsh//每4m一个点进行线性插值，结果中插值路点的id等于原两路点中的前一个路点的id
    result->points.clear();
    if(origin_way.points.size() < 2){
        return;
    }
    result->points.push_back(origin_way.points[0]);
    int control_point_num = origin_way.points.size();
    for (int i = 0; i < control_point_num - 1; i++) {
        float x1 = origin_way.points[i].point.x;
        float y1 = origin_way.points[i].point.y;
        float x2 = origin_way.points[i+1].point.x;
        float y2 = origin_way.points[i+1].point.y;
        float theta = PointToTheta(x1,y1,x2,y2);
        double dis_xy = distance(x1,y1,x2,y2);    //计算两点之间的距离
        int n = static_cast<int>(dis_xy / 4.0);       //4米一个点
        n = std::max(n, 1);
        double line_segment = dis_xy / n;
        double sum_seg = line_segment;
        lanelet_map_msgs::Node node;
        for(int j = 0; j < n-1; j++) {
            node.id = origin_way.points[i].id;
            node.point.x = x1 + sum_seg * std::cos(theta);
            node.point.y = y1 + sum_seg * std::sin(theta);
            result->points.push_back(node);
            sum_seg = sum_seg + line_segment;
        }
        result->points.push_back(origin_way.points[i+1]);
    }//lsh//每4m一个点进行线性插值，结果中插值路点的id等于原两路点中的前一个路点的id
}

void AttachXmlFile::CreatOutRoad(){
    if(m_astarsearch.NodeList.isEmpty()) {
        ROS_FATAL("CreatOutRoad: The Nodelist have not been loead.");
    }

    QList<Task_Node>::iterator taskIter;
    taskIter = m_cMapMatch.TaskList.begin();
    //得到包括首尾两个on_road任务点的out_road列表
    for(; taskIter != m_cMapMatch.TaskList.end(); taskIter++)
    {
        if(taskIter->type == 7){
            if(!m_pTaskNode->RoadList.isEmpty())       //notice
                m_pTaskNode->RoadList.clear();
            float x,y;
            Position_Trans_From_ECEF_To_UTM(taskIter->lat,taskIter->lon,0,0,&x,&y);
            m_pTaskNode->x = x;
            m_pTaskNode->y = y;
            //lsh//每个任务点的坐标
            QList<Road>::iterator roadIter;
            roadIter=m_RoadList.begin();//m_RoadList是以xml为单位储存路点的
            for(; roadIter!=m_RoadList.end(); roadIter++)
            {
                QList<RoadNode>::iterator tempnode1,nextnode1;
                tempnode1=(roadIter->RoadNodeList).begin();     //notice
                nextnode1=tempnode1+1;
                for(; nextnode1!=(roadIter->RoadNodeList).end(); nextnode1++)
                {
                    Road_Line tempRoadLine1;
                    m_cMapMatch.LineAndLinePoint(tempnode1->x,tempnode1->y,nextnode1->x,nextnode1->y,m_pTaskNode->x,m_pTaskNode->y,tempRoadLine1.Road_nodex,tempRoadLine1.Road_nodey);
                    m_cMapMatch.PointToLineDistance(m_pTaskNode->x,m_pTaskNode->y,tempRoadLine1.Road_nodex,tempRoadLine1.Road_nodey,tempRoadLine1.Dist);//notice:shibushi dizhi
                    m_pTaskNode ->RoadList.append(tempRoadLine1);
                    tempnode1 = nextnode1;
                }
            }//lsh//计算任务点到每条道路上每两点的投影点和投影距离
            QList<Road_Line>::iterator tempRoadLine2,nextRoadLine2;
            tempRoadLine2=m_pTaskNode->RoadList.begin();
            nextRoadLine2=tempRoadLine2+1;
            for(; nextRoadLine2!=m_pTaskNode->RoadList.end(); nextRoadLine2++)
            {
                if( tempRoadLine2 ->Dist > nextRoadLine2 ->Dist)
                {
                    *tempRoadLine2 = *nextRoadLine2;
                }
            }//lsh//找到最小投影距离
            //cout<<taskIter->Task_num<<"-------"<<"距离： "<<tempRoadLine2->Dist<<endl;
            if(tempRoadLine2->Dist > 7.0)
            {
                MapSearchNode *parent_node = new MapSearchNode;
                double dist = std::numeric_limits<double>::infinity();;
                QList<MapSearchNode*>::iterator way_node = m_astarsearch.NodeList.begin();
                for( ; way_node!=m_astarsearch.NodeList.end(); way_node++)
                {
                    double dist2 = distance((*way_node)->x,(*way_node)->y,taskIter->x,taskIter->y);
                    if(dist2 < dist)
                    {
                        dist = dist2;
                        parent_node = *way_node;
                    }
                }
                parent_node -> intersection = true;
                parent_node -> road_count++;
                //lsh//找到最近的母节点
                MapSearchNode* new_node = new MapSearchNode;
                new_node->lon = taskIter->lon;
                new_node->lat = taskIter->lat;
                new_node->node_id = taskIter->Task_num*100000+70000000+1;
                new_node->road_id = taskIter->Task_num*100000+70000000;
                new_node->intersection = false;
                Position_Trans_From_ECEF_To_UTM(taskIter->lat,taskIter->lon,0,0,&new_node->x,&new_node->y);
                new_node->type = 2;
                new_node->vlimit = 5.0;
                new_node->concave_obs_det = true;
                new_node->dynamic_obs_det = true;
                new_node->foogy_det = true;
                new_node->water_det = true;
                new_node->wall_area = true;
                new_node->ditch_area = true;
                new_node->road_count = 1;
                //lsh//找到要扩充的节点
                InsertOutNodeIntoList(new_node,parent_node);
                //lsh//连接子父节点
                m_astarsearch.NodeList.append(new_node);
                //lsh//把其加入NodeList

                Road tempRoad;
                RoadNode tempRoadNode1;
                RoadNode tempRoadNode2;
                //先把最靠近out_road_list[0]的路网上的点插入这个RoadNodeList列表
                tempRoadNode1.lat=new_node->lat;
                tempRoadNode1.lon=new_node->lon;
                tempRoadNode1.NodeID=new_node->node_id;
                tempRoadNode1.x=new_node->x;
                tempRoadNode1.y=new_node->y;
                tempRoadNode1.type = 0;
                tempRoadNode1.vlimit = 5;
                tempRoad.RoadNodeList.append(tempRoadNode1);
                //lsh//为创建的新道路加入路点
                tempRoadNode2.lat=parent_node->lat;
                tempRoadNode2.lon=parent_node->lon;
                tempRoadNode2.NodeID=parent_node->node_id;
                tempRoadNode2.x=parent_node->x;
                tempRoadNode2.y=parent_node->y;
                tempRoadNode2.type = 0;
                tempRoadNode2.vlimit = 5;
                tempRoad.RoadNodeList.append(tempRoadNode2);
                //lsh//为创建的新道路加入路点
                tempRoad.RoadID = taskIter->Task_num*100000+70000000;
                tempRoad.way_version = "doubleway";
                m_RoadList.append(tempRoad);
                //lsh//加入创建的新道路
            }
        }
    }
    if(!original_road_network_list.isEmpty())
        original_road_network_list.clear();
    QList<MapSearchNode*>::iterator road_iter = m_astarsearch.NodeList.begin();
    for(; road_iter != m_astarsearch.NodeList.end(); road_iter++)
    {
        original_road_network_list.append(*road_iter);
    }//lsh//复制路网
}

void AttachXmlFile::InsertOutNodeIntoList(MapSearchNode *cur,
                                          MapSearchNode *parent)
{
    if(cur && parent && cur!=parent)
    {
        cur->parentList.append(parent);
        cur->parentNum++;
        cur->successorList.append(parent);
        cur->successorNum++;
        parent->successorList.append(cur);
        parent->successorNum++;
        parent->parentList.append(cur);
        parent->parentNum++;
    }else{
        if(cur == NULL){
            ROS_WARN("cur_node = null");
        } else if(parent == NULL){
            ROS_WARN("parent_noe = null");
        }
        ROS_WARN("InsertNodeIntoList: build node %d's topological relation failed!",cur->node_id);
    }
}
