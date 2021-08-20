#ifndef ATTACHXMLFILE_H
#define ATTACHXMLFILE_H

#include "qmath.h"
#include "cv.h"
#include "highgui.h"
#include <iostream>
#include <vector>
#include "MapMatch.h"
#include "MapSearchNode.h"
#include <QList>
#include <QString>
#include <QThread>

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <lanelet_map_msgs/Way.h>
#include <extractroad_msg/extractroad.h>
#include <sensor_driver_msgs/GpswithHeading.h>
#include <tinyspline_ros/tinysplinecpp.h>
#endif

/***********全局地图常量*************/
#define MAP_WIDTH_CELL 800
#define	MAP_HEIGHT_CELL	600

#define COLORRED	cvScalar(0,0,255)
#define COLORBLUE	cvScalar(255,0,0)
#define COLORDARKGREEN cvScalar(0,100,0)
#define COLORGREEN	cvScalar(0,255,0)
#define	COLORWHITE	cvScalar(255,255,255)
#define COLORWHITE2 cvScalar(255,235,205)
#define COLORCYAN	cvScalar(255,255,0)
#define COLORYELLOW	cvScalar(0,255,255)
#define COLORPINK	cvScalar(255,0,255)
#define	COLORBLACK	cvScalar(0,0,0)
#define	COLORGREY	cvScalar(122,122,122)
#define COLORVIOLET cvScalar(138,42,226)
#define COLORBULECOLOR cvScalar(64,224,208)
#define COLORMEAT   cvScalar(255,125,64)
#define COLORORANGE cvScalar(255,128,0)
#define COLORStrawberry cvScalar(135,38,87)
#define COLORYELLOW2 cvScalar(238,238,0)

#define VEL_SIZE_WIDTH 6
#define VEL_SIZE_LENGHT 8

//路口信息目前完全由xml里来指定。最终目的是在规划结果中的任意一点都能够通过其属性表明是否需要特殊处理（路口、匝道等）。
//地图自己生成，要求提供道路信息，目前在xml地图分析中只保留了点的概念，可能下一步还需要增加道路信息。

using namespace std;

struct pos_int {
    int x;
    int y;
};

struct pos {
    double x;
    double y;
};

/// \brief 用于匹配的路点描述
struct RoadNode
{
    int NodeID;             /*节点ID*/
    int type;               /*节点类型*/
    float x,y;              /*节点大地坐标*/
    float lon,lat;          /*节点经纬度*/
    double vlimit;          /*记录当前路点的最大速度限制*/
    bool concave_obs_det;   /*凹碍物检测，true：开启 false:关闭*/
    bool dynamic_obs_det;   /*动态障碍物检测， true:开启 false:关闭*/
    bool foogy_det;         /*烟雾区检测，true:开启 false:关闭*/
    bool water_det;         /*水检测，true:开启 false:关闭*/
    bool wall_area;         /*垂直墙区域，true:开启 false:关闭*/
    bool ditch_area;        /*壕沟区域，true:开启 false:关闭*/
};

/// \brief 道路结构单元
struct Road {
    int RoadID;                     /*道路ID*/
    QString way_version;            /*双向路或单向路*/
    QList<RoadNode> RoadNodeList;   /*每段路中包含的节点*/
};

struct vehicle_state_inroadmap {
    double x,y,theta;
    double s;
    double prev_distonextstate,distonextstate;
    double dis_to_nextintersection;
    //POSITION pos;//上一个状态对应的pos
    vehicle_state_inroadmap *pos;
    MapSearchNode *m_prevstate, *m_prevstate_inlaststep;//上一状态
    MapSearchNode *m_nextstate;//前方状态
};

struct state_struct {
    int index;
    pos position;           //pos结构体包含x,y
    double s;               //弧长
    //double heading;       //弧度，车头与正北方向夹角，是惯导测出的，单位弧度，在局部坐标系下，heading是与正上方（y）的夹角
    double theta;           //弧度，车头与正东方向夹角，heading=pi/2-theta; heading_g=heading_l+heading_v
    double steering_angle;
    char turning;           //0 未知，1 直行，2 右转， 3 左转 4 掉头
    char type;              //0未知，1路点，2，入路口，3，出路口
    vector<pos_int> footprint;
    double heading_incoming;//道路出口航向
};

/// \brief 记录岔道口信息
struct intersec_info            /*记录岔道口信息*/
{
    int intersec_id;            /*本岔道口的ID*/
    int last_ord_id;            /*上一个普通路点的ID*/
};

    /// \brief xml节点
typedef struct XML_NODE_INFO {
    float flon;
    float flat;
    int   nodeID;                   /*节点ID*/
    int   roadID;                   /*所处路段ID*/
    int   type;                     /*节点属性*/
    bool  intersection;             /*是否岔道口*/
    QString  way_version;           /*单双向路属性*/
    double vlimit;                  /*记录当前路段的最大速度限制*/
    bool concave_obs_det;           /*凹碍物检测，true：开启 false:关闭*/
    bool dynamic_obs_det;           /*动态障碍物检测， truw:开启 false:关闭*/
    bool foogy_det;                 /*烟雾区检测，true:开启 false:关闭*/
    bool water_det;                 /*水检测，true:开启 false:关闭*/
    bool wall_area;                 /*垂直墙区域，true:开启 false:关闭*/
    bool ditch_area;                /*壕沟区域，true:开启 false:关闭*/
}XmlNodeInfo;//lsh//路点存储格式，不包含父子节点信息

/// \brief 规划工具包类
class AttachXmlFile {
public:
    ///lsh//用于构建任务点到全局路径直连的道路
    void CreatOutRoad();
    ///lsh//用于链接创建道路的子父节点
    void InsertOutNodeIntoList(MapSearchNode *parent_node,MapSearchNode *node);
    ///规划用链表，xml里的点及其拓扑关系保存在m_astarsearch.NodeList里
    AStarSearch<MapSearchNode> m_astarsearch;
    //CPtrList m_path_map_list;
    ///规划结果的点序列，只包含xy等信息，还需要进行进一步分析才能用于导航
    QList<MapSearchNode*> m_path_map_list;
    ///用于记录m_path_map_list是否被更新
    int pub_path_size;
    ///定位后的vehicle_state
    vehicle_state_inroadmap vehicle_state;
    state_struct vehicle_state_on_roadmap;
    ///lshadd//
    int add_goback_id = 100000000;

public:
    AttachXmlFile();
    ~AttachXmlFile();

    /**
     * 从xml文件中读入路网
     */
    void GetNodeFromFile();

    /**
     * 初始化显示
     */
    void InitDisplay();

    /**
     * 显示规划道路，由节点连接而成
     */
    void ShowPathWithPathList();

    /**
     * 显示所有道路，参数是节点链表
     */
    void ShowRoadWithNodeList();

    /**
     * 在规划的路径上匹配当前车辆所在路段
     * @param lat  当前车辆经度
     * @param lon  当前车辆纬度
     */
    void Receiveinfo(float lat,float lon);

    /**********显示和规划***********/
    ///显示控制变量
    IplImage *m_EnvImage_Global_diaplay;
    ///绘图原点的经纬度坐标
    pos_int delta_origin_pos;
    pos_int prev_mouse_pos;
    pos_int zoom_mouse_pos;
    double global_env_zoom;
    pos_int	m_ngoal_image;
    pos_int m_nmiddle_image;
    pos_int m_nstart_image;
    pos_int	m_ngoal_world;
    pos_int m_nmiddle_world;
    pos_int m_nstart_world;
    ///控制原点更新频率
    int windowCenterUpdateF;
    ///车辆航向
    double vehicle_theta;
    ///用来存放道路ID，防止重复读入道路数据
    vector<int> m_Pathid_vector;
    float espx;
    float espy;

    int   m_nStartLastNode;
    int   m_nEndNode;
    int   m_nStartNode;
    int   m_nNeedNode;
    float     m_fNeedNodex;
    float     m_fNeedNodey;
    float     m_fEndNodex;
    float     m_fEndNodey;
    float     m_fStartNodex;
    float     m_fStartNodey;

public: //2018.6.2
    /**
     * 在界面上选择起点、终点以及中间点进行路径规划
     */
    void RoadMapPathPlan();

    /**
     * 鼠标操作回调函数
     */
    static void on_mouse_globalenv(int event, int x, int y, int flags, void* param);

    /**
     * 求两条线段之间的夹角
     * @return theta 范围 0 到 CV_PI*2
     */
    float PointToTheta(float x1,float y1, float x2,float y2);

    /**
     * 对规划结果路径进行处理
     * @return 1成功，0失败
     */
    int GeneratePathInfo();

    /**
     * 将节点加入到链表，并建立节点间的关系，第一个参数是第二个参数的上一个节点
     * @return 0成功
     */
    int InsertNodeIntoList(XmlNodeInfo *xml_parent_node_info,XmlNodeInfo *xml_node_info);

    /**
     * 用于加载xml信息函数
     * @param filePath xml文件路径
     * @param fileName xml文件名
     */
    int GetXmlInfoWithSearchList(QString& filePath, QString& fileName);

    /**
     * 用于转换坐标关系，讲经纬度坐标转换为大地坐标。第一第二个参数表标经度和纬度，第三第四表示相对位置，第五第六表示输出值
     */
    void Position_Trans_From_ECEF_To_UTM(float latitude,float longitude,float e0, float n0, float *e, float *n);

    /**
     * 获取参考链表上距离坐标点最近的节点ID(距离最小)
     * @param NodeList 参考链表
     * @param x 大地坐标x
     * @param y 大地坐标y
     */
    int GetNodeidFromXY(QList<MapSearchNode*> *NodeList,int x,int y);

    /**
     * 从搜索链表中找到对应ID的节点
     */
    MapSearchNode* GetNodeFormList(int node_id);

public:
    //新添加一些处理函数
    /**
     * 计算点到直线的距离
     */
    void Cal_Point_Dis_to_Line(float pointx, float pointy, float linex1, float liney1, float linex2, float liney2, float *dis, float *u);

    /**
     * 用于完成一次规划后删除副本
     */
    void DeleteAStarSearchNodeList();

    /**
     * 删除节点父节点的所有联系，防止往回规划
     */
    void DeleteParentNode(MapSearchNode *tempnode,int NodeID);

    /**
     * 删除节点子节点的所有联系，防止往回规划，第一个删除父节点的节点，第二个参数为不应删除的关系的节点
     */
    void DeleteChildNode(MapSearchNode *tempnode,int NodeID);

    /**
     * 实现路径的拼接
     */
    void AddSolutionToAllPlanNodeList();

    /**
     * 三点规划
     */
    int PathPlan(int nstartid,int nmiddleid,int ngoalid);

    /**
     * 两点规划
     */
    int PathPlan(int nstartid,int ngoalid);

public:
    /**
     * 将任务点匹配并且插入到路网中
     */
    void MatchTaskPoints(QList<Task_Node> &m_TaskList);

    ///以道路为单位存放路点，用于道路匹配
    QList<Road> m_RoadList;

    /**
     * 将任务点插入到路网中
     */
    void InsertNodeIntoList_WorldXY(int FirstID, int NextID, Task_Node tmpTaskNode, float wx, float wy, QString way_version);

public:
    ///存放规划任务点
    QList<Task_Node> plan_task_list;

    /**
     * 根据任务链表在路网中规划路径
     * @param MyTaskList 任务链表
     * @return 1规划成功 0规划失败
     */
    int PlanWithTaskPoint(QList<Task_Node> &MyTaskList);

    /**
     * 确认重规划后，执行重规划职能
     */
    int RePlanning(float flat,float flon);

    /**
     * 在重规划前重新创建用于重规划的任务链表
     */
    void ReCreateTaskList(float flat,float flon);

    ///重规划的起始点
    ReStartNode m_StartNode;
    ///保存用于重规划的任务链表
    QList<Task_Node> m_Replan_Task_list;
    ///触发重规划时车辆当前位置的下一个路点ID
    int ReStartNextID;
    ///lshadd//折返点重规划时存储由任务点投影点形成的折返点，用于后续重规划断点时将其加入路点列表中，可避免断点断错的情况
    QList<Task_Node> m_Replan_AddNode_list;
    ///lshadd//判断在重规划过程中在路网上加入上述折返点的标志位
    int replanning_add_node;

public:
    //output txt file
    void OutputTxtFileWithPath(QString base_dir);
    void SendToRosMsg();

    ///规划完成标志 完成true 未完成false
    bool PathPlanFinishFlag;
    ///规划完成标志锁
    QMutex mutex_PathPlanFinishFlag;
    ///可执行程序所在目录
    QString base_dir;
    ///是否使用Vrep仿真
    bool if_vrep;

public:

    /**
     * 当接收到一个岔道口消息时，判断是否插入该岔道口数据
     * @param inter_data 是岔道口数据
     */
    void AddIntersectionNode(extractroad_msg::extractroad inter_data);

    /**
     * 接收车辆当前位置，定距向实时构建的路网中插入普通路点
     */
    void AddNormalNode(sensor_msgs::NavSatFix vehicle_gps);

    /**
     * 计算两点之前的距离
     */
    double distance(double x1, double y1, double x2, double y2);

    /**
     * 将岔道口数据插入到实时构建的拓扑地图road_network_list中
     * @param intersec_node 是岔道口数据
     */
    void InsertIntersecToList(MapSearchNode* intersec_node);

    /**
     * 采用实时构建的拓扑地图进行重归规划时，该函数用于找到余下未完成任务点中的第一个任务点
     */
    void FindConnectTaskID(sensor_msgs::NavSatFix current_vehicle_gps);

    /**
     * 在岔道口处找到一个合适的道路分支用于引导车辆通行
     * @return 1成功 0失败
     */
    int AddPassableFork();

    /**
     * 构建两个路点之间的拓扑链接关系，由于是越野路况，所以默认为双向连接
     * @param cur_node 是待插入点
     * @param parent_node 是路网中已有的路点
     */
    void BuildTopologicalRelation(MapSearchNode* cur_node, MapSearchNode* parent_node);

    /**
     * 每个岔道口都有若干个分支，该函数用于判断哪个分支是车辆已经通过了的分支
     */
    void JudgeWhichForkHasPassed();

    /**
     * 计算两个点相对于某个岔道口的夹角
     */
    double getAngelOfTwoVector(MapSearchNode* intersec,MapSearchNode* pass_node,MapSearchNode* lead_point);

public:
    ///路网文件路径
    QString way_path;
    ///记录实时构建拓扑地图中的岔道口
    QList<MapSearchNode*> intersec_list;
    ///记录经过的岔道口ID以及其上一个普通路点的ID
    QList<intersec_info> record_intersec;
    ///保存实时构建的拓扑路网
    QList<MapSearchNode*> road_network_list;
    ///用于复制路网
    QList<MapSearchNode*> original_road_network_list;
    ///给新插入到路网中的岔道口分配ID
    int intersec_id;
    ///给插入到路网中的普通路点分配ID
    int ordinary_id;
    ///给岔道口引导节点分配ID
    int leadpoints_id;
    ///用于判断新接收到的岔道口点是否之前已经被插入到路网中了
    float add_intersec_ref_dis;
    ///用于过滤重规划触发车辆位置周围n米的任务点
    float eli_task_node_ref_dis;
    ///控制实时构建的拓扑地图的普通路点之间的间隔
    float add_ordinary_node_ref_dis;
    ///上一次插入到路网中的岔道口点ID
    int last_intersec_id;
    ///每次插入新节点时，与新节点构建逻辑双向关系的父节点ID
    int topology_parent_id;
    ///重规划第二阶段的连接路点的ID
    int connect_task_id;
    ///重规划标志位，用于无条件向路网中插入普通一个普通路点
    bool replan_flag;
    ///重规划第一次规划触发时的上一个岔道口
    int first_plan_tri_intersec_id;
    ///重规划第一次规划的回退岔道口ID
    int first_plan_intersec_id;
    ///重规划的回退ID
    int fallback_node_id;
    ///重规划回退岔道口ID
    int fb_intersec_id;
    ///记录上一次触发重规划时的岔道口ID
    int last_replan_tri_rec;
    ///找到回退路点的控制距离
    float fallback_ref_dis;

public:
    /**
     * 直接读入任务文件用作搜索的路网
     */
    void GetNodeFromTxtFile();

    /**
     * 找到车当前位置的道路属性
     * @param vehicle_gps 车辆当前位置
     */
    void findNodeType(sensor_msgs::NavSatFix vehicle_gps);

    /**
     * 在采用实时构建拓扑地图进行重归重规划时，在重规划第一阶段，该函数用于找到倒车回退的目标岔道口和最终回退路点
     * @param vehicle_gps 第一次触发重规划时（也就是在倒车确认前）车辆所处位置，并不是执行重规划操作时车辆所处位置
     * @return 1成功 0失败
     */
    int findFbIntersec(sensor_msgs::NavSatFix vehicle_gps);

    //使用先验地图进行重规划
public:
    /**
     * 用于重规划时，找到距离车辆当前位置最近的岔道口，并依此为分界线将一条路径拆分为两条路径，分别是倒车路径和前进路径
     * @param result_path
     */
    void splitPath(QList<MapSearchNode*> *result_path);

    ///保存路径拆分结果的第一条路径
    QList<MapSearchNode*> first_path;
    ///保存路径拆分结果的第二条路径
    QList<MapSearchNode*> second_path;
    ///每次重规划前，将车当前位置作为第一个任务点，并对其分配ID
    int restart_id;
    ///记录各重归划的堵塞位置节点ID，在更新路网后用此链表打断各处节点关系，使路径不再通过此处
    QList<int> obs_node_id;

public:
    /**
     * 在界面中显示车辆当前位置
     */
    void ShowVelPos(float x1,float y1);

    /**
     * 旋转坐标
     */
    vector<cv::Point2i> getRotatePoint(vector<cv::Point2i> Points, const cv::Point rotate_center, const double angle);

    /**
     * 在任务点链表上进行匹配当前车辆所在的任务点区间
     */
    void taskMapMatch(float flat, float flon, QList<Task_Node> *task_list, int *last_id , int *next_id);

    /**
     * 在任务链表上进行匹配和更新
     */
    void mapMatchAndUpdatefortask(float flat, float flon);

    /**
     * 匹配和更新
     */
    void mapMatchAndUpdate(float flat, float flon);

    /// CMapMatch类实例，包含一些计算工具和读入任务点
    CMapMatch m_cMapMatch;
    ///判断是否已经规划
    bool m_bPathPlaned;
    ///在规划结果链表上进行匹配和更新的标志位。0：重新进行匹配 其它：更新
    int  m_flag;
    ///在任务点链表上进行匹配和更新的标志位。0：重新进行匹配 其它：更新
    int update_flag_for_task;
    QList<MapSearchNode *>::iterator m_pos;
    QList<Task_Node>::iterator m_task_pos;
    Vel_Node  tempVelNode;
    ///辅助计算，保存计算结果
    Vel_Node  *m_pVelNode;
    Task_Node tempTaskNode;        //tangbo
    ///辅助计算，保存计算结果
    Task_Node *m_pTaskNode;
    ///定义结构体的指针，接下来加入到链表中
    Road_Line *m_pRoadLine1, *m_pRoadLine2;
    Road_Line tempRoadLine;
    ///m_ptempnode指向当前车辆所处位置路点，m_pnextnode和m_pthirdnode分别指向当前所处位置的下一个和下下个路点,nextnode_shadow表示//lsh//下一个任务点的投影点
    MapSearchNode *m_pStartNode ,*m_pEndNode,*m_ptempnode,*m_pnextnode,*m_pthirdnode,*nexttask_shadow;
    ///m_pcurtask表示车辆当前所在的任务段，m_pnexttask和m_pthirdtask表示下一个任务和下下个任务
    Task_Node m_pcurtask,m_pnexttask,m_pthirdtask;
    ///垂直墙与壕沟任务点
    Task_Node wall_node,ditch_node;
    ///用于匹配当前路段速度
    QList<MapSearchNode*>  v_mapping_list;
    ///当前所在路段的属性
    int cur_road_type;
    ///当前所在路段的最大限速，m/s;
    double cur_road_vlimit;
    ///决定是否使用连贯性进行路点匹配
    bool use_coherence_mapping;
    ///当前车速
    double vehicle_vel;
    ///当前档位
    int gear_position;
    ///上一匹配点编号
    int last_match_num;
    ///折返点
    int switch_point_node;

    bool get_node_from_txt;
    /**
     * 发布路网
     * @param cur_gps 车辆当前位置
     * @param planning_result 路径规划结果
     */
    void publishWay(sensor_msgs::NavSatFix cur_gps, QList<MapSearchNode*> planning_result);
    /**
    * B样条插值
    */
    void bsplineInterpolate(const lanelet_map_msgs::Way &origin_way,
                            lanelet_map_msgs::Way *result);

    /**
     * 线性插值
     */
    void linearInterpolation(const lanelet_map_msgs::Way &origin_way,
                             lanelet_map_msgs::Way *result);

    ///发布路网锁
    QMutex mutex_pub_way;
    ///控制车辆前进或者后退，true前进 false后退
    bool is_forward;
    ///用于在界面显示发布的路
    QList<MapSearchNode*> pub_way_display;
    ///存放插值(样条插值或线性插值)后的路径
    lanelet_map_msgs::Way interpolation_way;
    ///用于存放待发布的路
    lanelet_map_msgs::Way way_msgs;
    ///用于存放全局规划路径
    lanelet_map_msgs::Way global_way_msgs;
    ///人为控制动态障碍物检测开关
    bool open_dynamic_obs_det;
    ///人为控制烟雾检测开关
    bool open_foggy_det;
    ///设定巡逻的次数
    int set_patrol_times;
    ///已经定线巡逻次数
    int numOfPatrols;
    ///距离车辆最近的路点ID或匹配点ID
    int nearest_point_id;
    ///非重规划时，行进过程中的前进后退标志
    bool reversing_flag;

    ///与搜索引导点对接的点
    MapSearchNode search_last_point;
    ///通向搜索点的引导点
    MapSearchNode search_lead_point;

    ///给任务点分配ID（通过界面选择任务点）
    int task_id;
};

#endif // ATTACHXMLFILE_H
