#include "../include/astardoublexml.h"
#include <QApplication>
#include "thread.h"
#include <qdebug.h>
int main(int argc, char *argv[])
{
    QApplication app(argc, argv);//lsh//QApplication管理GUI程序的控制流和主要设置
    AStarDoubleXML w(argc, argv);//lsh//修改读取地图路径在AttachXmlFile::GetNodeFromFile()
    //lsh//调用PubMsg建立ros_pub初始化话题的发布者和订阅者，初始化规划有关参数
    //lsh//完成相关参数的读取
    //lsh//获取包的位置
    //lsh//利用AttachXmlFile建立全局规划器m_attach_xml_file，其构造函数完成相关界面设置和参数设定
    //lsh//把m_attach_xml_file的地址设为ros_pub的全局规划器指针path_planner_
    //lsh//其他参数设定
    w.show();
    qDebug()<<"From main thread: "<<QThread::currentThreadId();//lsh//qDebug用于在控制台输出调试信息
    Thread w_thred;//lsh//修改任务点路径在Thread::Thread() 及MapMatch中CMapMatch::ReadTaskFile     //new thread for replanning in real time and updata postion
    //lsh//与任务相关的参数设置
    //lsh//任务文件读取位置设定
    w_thred.AStarThread = &w;//lsh//指向最高层的指针，使w_thred可调用w

    //lsh//读取任务点路径
    w_thred.task_file_dir = w_thred.home + QString(QString::fromLocal8Bit(w_thred.AStarThread->ros_pub.input_task_file_path.c_str()));;
    qDebug() << "task file dir: " << w_thred.task_file_dir << endl;
    w_thred.AStarThread->taskfile_dir = w_thred.task_file_dir;//lshadd//0715

    if(w_thred.isRunning()){
        ROS_INFO("the thread t is running.");
    }else{
        w_thred.start();//从此处进入thread.cpp中的run()函数
        //lsh//读取自启动设置，每100ms执行一次Thread::onTimeout()
        //lsh//Thread::onTimeout()是程序主循环
    }
    //w_thred.connect(&w_thred, SIGNAL(finished()), &w_thred, SLOT(deleteLater()));
    w_thred.connect(&app,SIGNAL(lastWindowClosed()), &w_thred,  SLOT(quit()));
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    int result = app.exec();
    return result;
    //return a.exec();
}
