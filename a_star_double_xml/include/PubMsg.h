//
// Created by mengze on 6/19/18.
//

#ifndef A_STAR_DOUBLE_XML_PUBMSG_H
#define A_STAR_DOUBLE_XML_PUBMSG_H

#include <QList>
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <sensor_driver_msgs/GpswithHeading.h>
#include <anm_msgs/VehicleState.h>
#include <lanelet_map_msgs/Way.h>
#include <extractroad_msg/extractroad.h>
#include <stiff_msgs/stiffwater.h>
#include "plan2control_msgs/Trajectory.h"
#include "ros/package.h"
#include <tinyspline_ros/tinysplinecpp.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include "three_one_msgs/Report.h"
#include "three_one_msgs/ControlWeapon.h"
#include "vertical_wall_grid/vertical_wall_grid.h"
#include "control_msgs/GetECUReport.h"
#endif
#include <string>
#include <QThread>
#include <QStringListModel>
#include <sstream>
#include "QMutexLocker"
#include "QMutex"
#include <QDebug>
#include <time.h>
#include "AttachXmlFile.h"

/// \brief 从配置文件读入参数
struct runParam{
    bool automatic_start;
    bool get_node_from_txt;
    bool use_prior_map_replan;
    bool open_dynamic_obs_det;
    bool open_foggy_det;
    bool open_fixed_line_patrol;
    int set_patrol_times;
    bool remote_control_mode;
    ///lsh///是否使用vrep仿真标志位
    bool vrep_simulate;
};

using namespace std;

/// \brief 接收发布消息类
class PubMsg : public QThread {
Q_OBJECT

public:
    PubMsg(int argc, char** argv);
    ~PubMsg(void);

    /**
     * 初始化
     */
    bool init();

    /**
     * 重载函数，所有需要在该分线程实现的功能都需要写在该函数内
     */
    void run();

    /**
     * 回调函数，接收车辆当前状态
     */
    void vehicleStateCb(const sensor_driver_msgs::GpswithHeading &gps);
    /**
     * ///lsh///回调函数，接收Vrep发送来的车辆当前状态
     */
    void vehicleVrepStateCb(const anm_msgs::VehicleState &vehicle_data_vrep);
    /**
     * ///lsh///回调函数，接收Vrep发送来的车辆当前状态
     */
    void ReplanningStateCb(const std_msgs::UInt8 &semantic_replanning_signal);
    /**
     * 回调函数，接收局部规划轨迹，用于判断是否倒车和重规划
     */
    void trajectoryCb(const plan2control_msgs::Trajectory &traj);

    /**
     * 回调函数，接收岔道口消息
     */
    void IsIntersection(const extractroad_msg::extractroad &inter_data);

    /**
     * 回调函数，接收烟雾消息
     */
    void isFoogy(const std_msgs::String &isfoogy);

    /**
     * 回调函数，接收草消息
     */
    void isGrass(const std_msgs::String &isgrass);

    /**
     * 回调函数，接收水消息
     */
    void isWater(const stiff_msgs::stiffwater &water_msg);

    /**
     * 回调函数，接收搜索停止消息
     */
    void searchStop(const std_msgs::UInt8 &isSearchStop);

    /**
     * 回调函数，接收当前车辆速度
     */
    void callbackEcuDataReport(const control_msgs::GetECUReport &msg);

    /**
     * 回调函数，接收远程终端发送过来的启动/停止/暂停命令，在停止期间有可能重新发送任务点
     */
    void callbackTaskCommand(const std_msgs::String &commandData);

    /**
     * gps状态回调函数
     */
    void gpsStateCb(const sensor_driver_msgs::GpswithHeading &gps);

    /**
     * 垂直墙回调函数
     */
    void vertical_wall_cb(const vertical_wall_grid::vertical_wall_grid &detectData);

    /**
     * 壕沟回调函数
     */
    void ditch_cb(const vertical_wall_grid::vertical_wall_grid &detectData);

    ///保存车辆位置信息
    sensor_msgs::NavSatFix vehicle_gps;
    ///车辆位姿
    geometry_msgs::Point vehicle_pose_;
    ///保存接收到的岔道口数据
    extractroad_msg::extractroad intersec_data;
    ///lsh//重规划停车标志位
    std_msgs::UInt8 wait_judge_repanning;
    ///lsh//保证语义识别重规划一次标志位
    int semantic_once_flag;
    ///lsh//语义识别重规划标志位
    int semantic_replanning_flag;
    ///标记
    int index;
    ///是否接收到了岔道口数据
    bool is_rcv_intersec;
    ///是否接收到了位置数据
    bool is_rcv_gps;
    ///gps卫星信号状态
    bool gps_state;
    ///检查更新
    bool check_gps_update;
    ///垂直墙检测
    bool vertical_wall_state;
    ///壕沟检测
    bool ditch_state;
    ///重规划标志位
    bool need_replanning_;
    ///是否已经完成了第一次规划的标志位
    bool attach_success_;
    ///是否为烟雾区
    bool is_foogy;
    ///前方是否有草
    bool is_grass;
    ///前方是否有水
    bool is_water;
    ///等待搜索控制位
    bool wait_for_search;
    ///远程终端发送过来的任务开始(1)/暂停（２）/停止(3)等任务命令
    std::string task_command;

    ///等待巡线控制位
    bool wait_for_xunxian;
    ///搜索停止标志
    bool search_stop;
    ///巡线停止标志
    bool xunxian_stop;
    ///可执行程序的本地目录
    std::string base_dir;
    ///倒车确认标志
    bool backup_confirm_flag;
    ///当前汽车车速
    double vehicle_vel;
    ///当前档位，0：N 1:D 2:R
    int gear_position;
    ///当前车辆是否处于重规划的第一条路上
    bool first_path_flag;

public:
    //变量锁
    QMutex mutex_need_replanning,mutex_attach_success;
    QMutex mutex_is_rev_gps,mutex_vehicle_gps;
    QMutex mutex_is_rcv_intersec,mutex_intersec_data;
    QMutex mutex_is_forward;
    QMutex mutex_cur_road_type;
    QMutex mutex_backup_confirm;
    QMutex mutex_vehicle_vel;
    QMutex mutex_first_path_flag;

Q_SIGNALS:
    void rosShutdown();

private:
    ///接收定位信息
    ros::Subscriber vehicle_state_sub;
    ///接收Vrep车辆位资定位信息
    ros::Subscriber vehicle_vrep_state_sub;
    ///接收重规划信息
    ros::Subscriber semantic_sub;
    ///接收局部规划轨迹，用于判断是否倒车和重规划
    ros::Subscriber traj_sub_;
    ///接收岔道口信息
    ros::Subscriber intersection_sub;
    ///接收烟雾区信息
    ros::Subscriber foggy_sub_;
    ///接收草信息
    ros::Subscriber grass_sub_;
    ///接收前方是否有水消息
    ros::Subscriber water_sub_;
    ///发布路网
    ros::Publisher path_pub;
    ///发布全局路径规划结果
    ros::Publisher global_way_pub;
    ///重规划倒车确认时停车
    ros::Publisher not_move_pub;
    ///武器站话题
    ros::Publisher weapon_pub;
    ///垂直墙话题
    ros::Subscriber vertical_wall_sub_;
    ///壕沟话题
    ros::Subscriber ditch_sub_;
    ///gps接收话题
    ros::Subscriber gps_sub_;
    ///接收搜索停止信息
    ros::Subscriber search_stop_sub_;
    ///接收车速
    ros::Subscriber vel_sub_;
    ///接收任务命令
    ros::Subscriber task_command_sub_;
    ///车辆状态更新标志
    bool vehicle_state_updated_;
    ///最近的节点ID
    int nearest_point_id_;
    ///指向m_attach_xml_file的指针
    AttachXmlFile *path_planner_;
    ///阻断路径计数
    int empty_traj_count_;
    ///武器站任务开始指令
    bool weapon_contorl;
    ///武器占任务结束标志
    bool weapon_task_finished;
    ///武器站任务执行开始执行时间；
    time_t weapon_start_time;

Q_SIGNALS:
    /**
     * 信号函数，触发重规划
     */
    void ReplanMsg(float lat,float lon) const;

public Q_SLOTS:
    /**
     * 槽函数，重规划
     */
    void Replanning(float lat,float lon) const;

// transport object address
public:
    /**
     * 传输指针
     */
    void transPathPlanner(AttachXmlFile *planner);

    /**
     * 计算两点之间的距离
     */
    double distance(double x1, double y1, double x2, double y2);

    /**
     * B样条插值
     */
    void interpolateBspline(const lanelet_map_msgs::Way &origin_way,
                            lanelet_map_msgs::Way *result);

    /**
     * 线性插值
     */
    void linearInterpolation(const lanelet_map_msgs::Way &origin_way,
                             lanelet_map_msgs::Way *result);

    /**
     * 求两条线段之间的夹角
     */
    float PointToTheta(float x1,float y1, float x2,float y2);

    std::vector<geometry_msgs::Point> way_point;
    void transParameter(const std::vector<geometry_msgs::Point> &object);
    ///线性插值距离,决定线性插值间隔
    float linear_interpo_dis;
    ///是否使用样条插值
    bool use_bspline;
    ///是否需要插值
    bool need_interpolation;
    ///接收到岔道口数据
    bool rcv_cross_road;
    ///是否重规划
    bool need_replanning_real_time;
    ///当前路的属性
    int cur_road_type;
    //读取全局规划运行参数
    runParam run_param;
    ///路网文件读取路径
    string way_net_file_path;
    ///任务文件读取路径
    string input_task_file_path;
    ///接收gps信号的话题名：lll
    string input_gps_topic;

};
#endif //A_STAR_DOUBLE_XML_PUBMSG_H
