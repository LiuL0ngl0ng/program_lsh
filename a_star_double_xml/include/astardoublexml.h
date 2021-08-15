#ifndef ASTARDOUBLEXML_H
#define ASTARDOUBLEXML_H

#include <QMainWindow>
#include "AttachXmlFile.h"
#include "cv.h"
#include "highgui.h"
#include "cxcore.h"
#include "stdlib.h"
#include "math.h"
#include "ui_astardoublexml.h"
#include "PubMsg.h"

//namespace Ui {
//class AStarDoubleXML;
//}

/// \brief 界面以及规划控制类
class AStarDoubleXML : public QMainWindow
{
    Q_OBJECT

public:
    explicit AStarDoubleXML(int argc, char** argv, QWidget *parent = 0);
    //lsh//C++中的explicit关键字只能用于修饰只有一个参数的类构造函数
    //lsh//它的作用是表明该构造函数是显示的, 而非隐式的
    //lsh//跟它相对应的另一个关键字是implicit, 意思是隐藏的,类构造函数默认情况下即声明为implicit(隐式).
    //lsh//隐式构造函数可以用=初始化，显式构造函数只能用（）初始化
    ~AStarDoubleXML();

private:
    //Ui::AStarDoubleXML *ui;
    ///界面控制端口
    Ui_AStarDoubleXML ui;

private Q_SLOTS:
    /**
     * 读取路网文件界面回调函数
     */
    void on_ReadFile_clicked();

    /**
     * 读取任务点文件界面回调函数
     */
    void on_ReadTaskFile_clicked();

    /**
     * 规划路径界面回调函数
     */
    void on_Pathplan_clicked();

    /**
     * 人为重规划触发界面回调函数
     */
    void on_pushButton_clicked();

    /**
     * 回调函数，清除人工选择的任务点链表
     */
    void on_OutputTxt_clicked();

public:
    /**
     * 自动远程启动程序
     */
    void automaticStart();

    /**
     * 接收车辆当前位置信息以及岔道口信息实时建立已通过路段的拓扑地图
     */
    void BuildTopologicalMap();

    /**
     * 当采用实时构建的拓扑地图进行重规划时，该步骤产生一条回退到上一个岔道口的回退路径
     */
    void PlanTheFirstPath();

    /**
     * 当采用实时构建的拓扑地图进行重规划时，当车辆在第一条路径上倒退到指定位置时，则触发条件规划发送第二条路径
     */
    void PlanTheSecondPath();

    /**
     * 实时构建拓扑地图、倒车确认以及是否进行重规划
     */
    void RealTime();

    /**
     * 1.实时检测当前车辆状态，当车辆前方道路被阻断时，该函数接收其他模块传输的过来的重规划信号，首先进行倒车确认
     * （倒车5米），以此来消除由感知误检导致前方路段暂时被阻断的情况。在倒车确认后，如果还能收到由于前方路段被阻断
     * 而引发的重规划信息，那么就进行重规划
     * 2.如何检测到在同一位置连续触发两次重规划，则不进行倒车确认，直接进行重规划
     * @param current_vehicle_gps 是当前的车辆位置
     */
    void Backup(sensor_msgs::NavSatFix current_vehicle_gps);

    /**
     * 将从话题接收过来的数据向下层对象转移传输
     */
    void dataTransfer();

public:
    ///路网节点类的实例，规划在其中进行。
    AttachXmlFile m_attach_xml_file;
    ///是否读取路网的标志
    bool m_bRecvRndf;
    ///获取程序本地目录
    QString base_dir;
    ///lshadd//0715
    QString taskfile_dir;
    ///接收和发布实例类
    PubMsg ros_pub;
    ///接收gps信号的话题名 :lll
    QString gps_topic;

public:

    ///实施构建拓扑地图的开关，当其关闭时，不构建拓扑地图
    bool record_flag;
    ///重规划第一阶段的倒车路径规划完成标志
    bool first_planning_finished;
    ///当直接读取任务点构建稀疏路网给局部规划进行引导时（此时路网中的路点全是任务点）。在依据实时构建拓扑地图进行重规划的前提下
    ///，第二阶段规划的终点应当为路网的最后一个节点，即最后一个任务点
    int end_task_node_id;
    ///触发重规划时车辆所处的位置
    sensor_msgs::NavSatFix first_plan_vehicle_gps;
    ///倒车的标志
    bool backup_flag;
    ///车辆是否处于重规划第一阶段的标志位。当处于倒车阶段的时候，不接收重规划信息
    bool first_path_flag;
    ///记录倒车的起始位置
    sensor_msgs::NavSatFix backup_start_gps;
    ///重规划第一阶段的起始点ID
    int first_plan_start_id;
    ///临时存放路径,这里不能用指针的原因是每次重规划之前都会重新加载路网
    QList<MapSearchNode> temp_path_list;
    ///临时存放用于匹配的规划路径
    QList<MapSearchNode> temp_mapmatch_list;
    ///重规划第一次规划触发标志
    bool first_planning_flag;
    ///记录在同一个位置重复触发倒车的次数，如果超过2次，则直接进行重规划
    int repeat_tri_times;
    ///是否采用先验地图进行重规划的控制位
    bool use_prior_map_replan;
    ///是否开启定线巡逻
    bool open_fixed_line_patrol;
    ///计数计时
    int wait_times;
    ///计时（为多圈功能补充的技术方法）
    time_t wait_start_time;
    ///等待发第二条路的等待时间
    int wait_times_for_second_path;

    ///第二次规划触发控制距离
    float second_plan_trig_dis;

    ///是否从txt中直接读取任务点用做路网
    bool get_node_from_txt;

    //only for test! test the BuildTopologicalMap function
    QList<extractroad_msg::extractroad> original_intersec_list;
};

#endif // ASTARDOUBLEXML_H
