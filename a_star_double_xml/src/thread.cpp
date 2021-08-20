//
// Created by tb on 18-6-14.
//

#include "../include/thread.h"
#include <QDir>

Thread::Thread() {
    std::cout << "constructe thread class" << std::endl;
    currentLat=0;
    currentLon=0;
    //automatic_start = true;     //自动启动开关
    rcv_task_file = false;
    auto_start_suc = false;
//    this->base_dir = ros::package::getPath("a_star_double_xml");
//    task_file_dir = QString::fromStdString(base_dir + "/task_file_dir/KYXZ2018A.txt");
    home = QDir::homePath();
    //AStarThread->taskfile_dir = home + QString(QString::fromLocal8Bit(AStarThread->ros_pub.input_task_file_path.c_str()));;
    //qDebug() << "task file dir: " << task_file_dir << endl;
}

void Thread::run()
//lsh//读取自启动设置，每100ms执行一次Thread::onTimeout()
{
    //初始化参数
    automatic_start = AStarThread->ros_pub.run_param.automatic_start;
    std::cout << "automatic start: " << std::boolalpha << automatic_start << std::endl;
    //lsh//boolalpha的作用是使bool型变量按照false、true的格式输出。
    std::cout <<"From thread run function: "<<currentThreadId();
    QTimer* m_pTimer = new QTimer();//建立定时器
    m_pTimer->setInterval(100);         //10hz
    //connect(m_pTimer, SIGNAL(timeout()), this, SLOT(onTimeout()) , Qt::DirectConnection);
    connect(m_pTimer, SIGNAL(timeout()), this, SLOT(onTimeout()));
    m_pTimer->start();//以一定频率（10hz）调用onTimeout()函数
    //lsh//创建一个QTimer对象，将信号timeout()与相应的槽函数相连，然后调用start()函数。
    //lsh//接下来，每隔一段时间，定时器便会发出一次timeout()信号。
    //lsh//每触发一次SIGNAL，便执行一次SLOT
    this->exec();//lsh//信号槽需要消息机制支持，QThread::exec()就是启动该线程的消息机制。
    //lsh//从qt4.4开始，run默认调用QThread::exec()
    return;
}

//程序主循环
void Thread::onTimeout() {
    //if(AStarThread->ros_pub.if_search_end)
    if(true)
    {
        //qDebug()<<"From ontimeout function thread: "<<currentThreadId();
        //****update the current location of vehicle*****//
        {
            QMutexLocker locker(&AStarThread->ros_pub.mutex_is_rev_gps);
            if(AStarThread->ros_pub.is_rcv_gps){
                //lsh//通过话题/gpsdata_sync接收到了GPS信号
                {
                   QMutexLocker locker(&AStarThread->ros_pub.mutex_vehicle_gps);
                  currentLon=AStarThread->ros_pub.vehicle_gps.longitude;
                  currentLat=AStarThread->ros_pub.vehicle_gps.latitude;
                 //lsh//设置车辆当前的经纬度坐标
                }
            }else{
                ROS_WARN("waiting for gps msg ...");
                return;
            }
        }//lsh//设置车辆当前经纬度

        //启动程序/实时构建地图/实时检测是否重规划
        if(automatic_start){//lsh////自动远程启动开关
            if(!rcv_task_file){
                if (QFile::exists(task_file_dir)){
                    //lsh//判断文件或文件夹是否存在
                    ROS_INFO("task file received.");
                    rcv_task_file = true;
                }else{
                    ROS_WARN("Waiting to recv task file");
                    return;
                }
        }
            if(rcv_task_file && !auto_start_suc){//lsh//auto_start_suc自启动成功标志
                auto_start_suc = true;
                AStarThread->automaticStart();//自启动程序
            }//lsh//读取路网文件、任务文件、规划全局路径
            if(auto_start_suc){
                AStarThread->RealTime();//实施构建地图、检测是否重规划，该函数在全局路径规划完成后会持续被调用
            }
        } else {
            AStarThread->RealTime();
        }

        //匹配
        {
            //add pathplanFinishFlag lock
            QMutexLocker locker(&(AStarThread->m_attach_xml_file.mutex_PathPlanFinishFlag));
            if(AStarThread->m_attach_xml_file.PathPlanFinishFlag) {
                AStarThread->m_attach_xml_file.mapMatchAndUpdate(currentLat,currentLon);
            }
        }

        //only for test! test the BuildTopologicalMap function
        /*float current_x,current_y;
        float intersec_x,intersec_y;
        AStarThread->m_attach_xml_file.Position_Trans_From_ECEF_To_UTM(currentLat,currentLon,0,0,&current_x,&current_y);
        float distance_to_last_intersec =100;
        QList<extractroad_msg::extractroad>::iterator intersec_iter = AStarThread->original_intersec_list.begin();
        for(; intersec_iter != AStarThread->original_intersec_list.end(); intersec_iter++) {
            AStarThread->m_attach_xml_file.Position_Trans_From_ECEF_To_UTM(
                    (*intersec_iter).vehicle_point[1], (*intersec_iter).vehicle_point[0], 0, 0, &intersec_x, &intersec_y);
            distance_to_last_intersec =
                    AStarThread->m_attach_xml_file.distance(current_x, current_y, intersec_x, intersec_y);
            if (distance_to_last_intersec < 15) {
                {
                    QMutexLocker locker(&AStarThread->ros_pub.mutex_is_rcv_intersec);
                    AStarThread->ros_pub.is_rcv_intersec = true;
                }
                {
                    QMutexLocker locker(&AStarThread->ros_pub.mutex_intersec_data);
                    AStarThread->ros_pub.intersec_data = (*intersec_iter);
                }
            }
        }*/
    }else{
        ROS_WARN("Search task is being done, global plan is sleeping.");
    }
}
