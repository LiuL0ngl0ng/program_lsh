//
// Created by tb on 18-6-14.
//

#ifndef A_STAR_DOUBLE_XML_THREAD_H
#define A_STAR_DOUBLE_XML_THREAD_H

#include <QThread>
#include "astardoublexml.h"
#include <QTimer>
#include <QDebug>
#include "MapSearchNode.h"
#include <extractroad_msg/extractroad.h>
#include <QtCore/QFile>

/// \brief 循环检测控制分线层类
class Thread : public QThread
{
Q_OBJECT
public:
    /**
     * 构造函数
     */
    Thread();

    /**
     * 重载函数，所有需要在线程执行的功能都需要在写在该函数内
     */
    void run();

public Q_SLOTS:
    /**
     * 槽函数
     */
    void onTimeout();

public:
    ///指向最高层的指针
    AStarDoubleXML* AStarThread;
    ///车辆当前位置经纬度
    float currentLat,currentLon;
    ///自动远程启动开关
    bool automatic_start;
    ///自启动成功标志
    bool auto_start_suc;
    ///保存程序可执行文件目录
    std::string base_dir;
    ///任务文件所在目录
    QString task_file_dir;
    ///本地路径
    QString home;
    ///接收到任务文件标志
    bool rcv_task_file;
    ///接收gps信号的话题名
    QString gps_topic;
};

#endif //A_STAR_DOUBLE_XML_THREAD_H
