#include "../include/MapMatch.h"
#include "qmath.h"
#include <QString>
#include "cv.h"
#include "highgui.h"
#include <QFile>
#include <QDebug>
#include <QRegExp>
#include <QtCore/QDir>

CMapMatch::CMapMatch() {
    TaskBeReaded=false;
    gps_updated=false;
    //match task point
    currentlat=0;
    currentlon=0;
}
CMapMatch::~CMapMatch() {
}
void CMapMatch::PointToLineDistance(float Pointx1,float Pointy1,float Pointx2,float Pointy2,float &Dist) {
    //if((Pointx1 == Pointx2) ||(Pointy1 == Pointy2))//相等的情况
    if((Pointx1 == Pointx2) && (Pointy1 == Pointy2))//TangBo修改
        Dist = 0;
    else
        Dist = sqrt(pow((Pointx1-Pointx2),2) + pow((Pointy1-Pointy2),2));//不相等的情况
}

void CMapMatch::LineAndLinePoint(float nodex1,float nodey1,float nodex2,float nodey2,float Pointx1,float Pointy1,float &Pointx2,float &Pointy2)//分三种情况，水平，垂直，斜的
{
    //lsh//获取Point1在向量node1->node2上的投影点Point2，且Point2在向量node1->node2之间
    if( (nodey1 == nodey2) && (nodex1 != nodex2))//水平
    {
        Pointx2 = Pointx1;
        Pointy2 = nodey1;
        if(nodex1 < nodex2)//当前一节点的x值小于后一节点的x值
        {
            if((Pointx2 >= nodex1)&&(Pointx2 <= nodex2))//在两者中间时取原来的值，当小于nodex1时取nodex1,当大于nodex2时，取nodex2
                Pointx2 = Pointx2;
            else if(Pointx2 < nodex1)
                Pointx2 = nodex1;
            else
                Pointx2 = nodex2;
        }
        else//当前一节点x值大于后一节点的x值
        {
            if((Pointx2 >= nodex2)&&(Pointx2 <= nodex1))//在两者中间时取原来的值，当大于nodex1时取nodex1,当小于nodex2时，取nodex2
                Pointx2 = Pointx2;
            else if(Pointx2 > nodex1)
                Pointx2 = nodex1;
            else
                Pointx2 = nodex2;
        }
    }
    else if( (nodex1 == nodex2)&&(nodey1 != nodey2) )//垂直
    {
        Pointx2 = nodex1;
        Pointy2 = Pointy1;
        if(nodey1 < nodey2)//当前一节点的x值小于后一节点的x值
        {
            if((Pointy2 >= nodey1) && (Pointy2 <= nodey2))//在直线中间
                Pointy2 = Pointy2;
            else if(Pointy2 < nodey1)//在直线下面时投影到下面的节点上
                Pointy2 = nodey1;
            else
                Pointy2 = nodey2;//在直线上面时投影到上面的节点上
        /*	if(Pointy2<nodey1)
                Pointy2 = nodey1;*/
        }
        else//当前一节点的x值大于后一节点的x值
        {
            if((Pointy2 >= nodey2) && (Pointy2 <= nodey1))
                Pointy2 = Pointy2;
            else if(Pointy2 < nodey2)
                Pointy2 = nodey2;
            else
                Pointy2 = nodey1;
        /*	if(Pointy2 > nodey1)*/
        }
    }
    else//两个点的横坐标和纵坐标都不相同的情况
    {
        float k1 ,k2 ,b1 ,b2;//k1为两节点间的斜率，k2是点到直线垂线的斜率
        k1 = (nodey2 - nodey1)/(nodex2 - nodex1);
        b1 = nodey1 - k1*nodex1;
        k2 = -1/k1;
        b2 = Pointy1 - k2*Pointx1;
        Pointx2 = (b1 - b2)/(k2 - k1);
        Pointy2 = k1*Pointx2 + b1;
        if(nodex1 < nodex2)//当nodex1小于nodex2的时候
        {
            if( (Pointx2 <= nodex2)&&(Pointx2 >= nodex1))//在直线中间
            {
                Pointx2 = Pointx2;
                Pointy2 = Pointy2;
            }
            else if(Pointx2 < nodex1)//在线线外面时投影到靠近的节点上
            {
                Pointx2 = nodex1;
                Pointy2 = nodey1;
            }
            else //在nodex2后面投影到nodex2的节点上
            {
                Pointx2 = nodex2;
                Pointy2 = nodey2;
            }
        }
        else//当nodex1大于nodex2的时候
        {
            if((Pointx2 >= nodex2)&&(Pointx2 <= nodex1))//在两者中间时取原来的值，当大于nodex1时取nodex1,当小于nodex2时，取nodex2
            {
                Pointx2 = Pointx2;
                Pointy2 = Pointy2;
            }
            else if(Pointx2 > nodex1)//当其值在nodex1外面时，投影到nodex1上
            {
                Pointx2 = nodex1;
                Pointy2 = nodey1;
            }
            else//当其值在nodex2外面时，投影到nodex2上
            {
                Pointx2 = nodex2;
                Pointy2 = nodey2;
            }

        }
    }
}

void CMapMatch::PointToSlope(float node_x1, float node_y1,float node_x2,float node_y2,float &node_Slope) {
    //lsh//求斜率
    if((node_x1 != node_x2) && (node_y1 != node_y2))//三种情况，垂直，水平，倾斜的
        node_Slope = (node_y2 - node_y1)/(node_x2 - node_x1);
    else if((node_x1 != node_x2) && (node_y1 == node_y2))
        node_Slope = 0;
    else
        node_Slope = VERTICAL;
}

void CMapMatch::DistanceToProbability(Vel_Node *pVelNode) {
    QList<Road_Line*>::iterator pos,pos1;
    //POSITION pos,pos1;
    Road_Line *tempRoad;
    float Dist_Count = 0;
    //int j = pVelNode->RoadList.GetSize();     //notice:getsize与count的区别
    //getsize(),调用此方法以检索列表中的元素数。
    //pos = pVelNode->RoadList.GetHeadPosition();
    int j= pVelNode->RoadList.size();
    if(j > 0)
    {
        /*for(int i = 0; i < j ;i++)//求各距离的总和
        {
            tempRoad = ( Road_Line *)pVelNode->RoadList.GetNext(pos);
            if(tempRoad->Dist == 0)
            {
                Dist_Count += VERTICAL;
            }
            else
            {
                Dist_Count = Dist_Count + 1/(tempRoad ->Dist);
            }
        }*/

        pos = pVelNode->RoadList.begin();
        for(; pos!=pVelNode->RoadList.end() ; pos++)
        {
            tempRoad=(*pos);
            if(tempRoad->Dist == 0)
            {
                Dist_Count += VERTICAL;
            }
            else
            {
                Dist_Count = Dist_Count + 1/(tempRoad ->Dist);
            }
        }


        /*pos1 = pVelNode->RoadList.GetHeadPosition();
        for(int k = 0; k<j ;k++)//将距离改变成概率，但是用指针完成，链表中的元素没有变
        {
            tempRoad = ( Road_Line *)pVelNode->RoadList.GetNext(pos1);
            if(tempRoad->Dist == 0)
            {
                tempRoad ->Distance_Probability = VERTICAL/Dist_Count;
            }
            else
            {
                tempRoad ->Distance_Probability = 1/((tempRoad ->Dist)*Dist_Count);
            }
        }*/
        pos1=pVelNode->RoadList.begin();
        for(; pos1!=pVelNode->RoadList.end(); pos1++)
        {
            tempRoad= (*pos1);
            if(tempRoad->Dist == 0)
            {
                tempRoad ->Distance_Probability = VERTICAL/Dist_Count;
            }
            else
            {
                tempRoad ->Distance_Probability = 1/((tempRoad ->Dist)*Dist_Count);
            }
        }
    }
}
void CMapMatch::IncludeToProbability(Vel_Node *pVelNode) {

    QList<Road_Line*>::iterator pos,pos1,pos2,pos3;
    Road_Line *tempRoad,*tempRoad1,*tempRoad2;
    float Angle_Count = 0,Prob_Count = 0;
    int j = pVelNode->RoadList.size();
    if(j > 0)
    {
        //pos = pVelNode->RoadList.GetHeadPosition();
        //for(int i = 0;i < j;i++)
        pos=pVelNode->RoadList.begin();
        for(; pos!=pVelNode->RoadList.end() ; pos++)
        {
            tempRoad=(*pos);
            //tempRoad = (Road_Line *)pVelNode->RoadList.GetNext(pos);
            if(tempRoad->IncludeAngle == 0)
            {
                Angle_Count += VERTICAL;
            }
            else
            Angle_Count += fabs(1/(tempRoad->IncludeAngle)) ;
        }

        /*pos1 = pVelNode->RoadList.GetHeadPosition();
        for(int k = 0; k<j ;k++)//将角度改变成概率，但是用指针完成，链表中的元素没有变
        {
            tempRoad = ( Road_Line *)pVelNode->RoadList.GetNext(pos1);*/
        pos1=pVelNode->RoadList.begin();
        for(; pos1!=pVelNode->RoadList.end(); pos1++)
        {
            tempRoad=(*pos1);
            if(tempRoad ->IncludeAngle == 0)
            {
                tempRoad ->Angle_Probability = VERTICAL/Angle_Count;
            }
            else
            tempRoad ->Angle_Probability = 1/((tempRoad ->IncludeAngle)*Angle_Count);
        }


        /*pos2 = pVelNode->RoadList.GetHeadPosition();
        for(int i = 0;i < j; i++)
        {
            tempRoad1 = ( Road_Line *)pVelNode->RoadList.GetNext(pos2);*/
        pos2=pVelNode->RoadList.begin();
        for(; pos2!=pVelNode->RoadList.end(); pos2++)
        {
            tempRoad1=(*pos2);
            Prob_Count += tempRoad1 ->Angle_Probability;
        }

        /*pos3 = pVelNode->RoadList.GetHeadPosition();
        for(int i = 0;i < j; i++)
        {
            tempRoad2 = ( Road_Line *)pVelNode->RoadList.GetNext(pos3);*/
        pos3=pVelNode->RoadList.begin();
        for(; pos3!=pVelNode->RoadList.end(); pos3++)
        {
            tempRoad2=(*pos3);
            tempRoad2 ->Angle_Probability = tempRoad2 ->Angle_Probability/Prob_Count;
        }
    }
}
void CMapMatch::SlopeToIncludeAngle(float k1, float k2, float &IncludeAngle)//所有的角度表示都是用弧度来表示的
{
    IncludeAngle = fabs(atan((k1 - k2)/(1 + k1*k2)));
}
void CMapMatch::OrthonormalToProbability(Vel_Node *pVelNode) {
    Road_Line *tempRoad;
    float K = 0;//证据间的符合程序相当于D-S理论中的M1⊕M2中的K值
    //POSITION pos,pos1;
    QList<Road_Line*>::iterator pos,pos1;
    int j = pVelNode->RoadList.size();
    if( j > 0)
    {
        /*pos = pVelNode->RoadList.GetHeadPosition();
        for(int i = 0;i < j; i++)
        {
            tempRoad = (Road_Line *)pVelNode->RoadList.GetNext(pos);*/
        pos=pVelNode->RoadList.begin();
        for(; pos!=pVelNode->RoadList.end(); pos++)
        {
            tempRoad=(*pos);
            K += tempRoad ->Angle_Probability * tempRoad ->Distance_Probability;
        }


        /*pos1 = pVelNode->RoadList.GetHeadPosition();
        for(int k = 0;k < j; k++)//融合好后的概率分布
        {
            tempRoad = (Road_Line *)pVelNode->RoadList.GetNext(pos1);*/
        pos1=pVelNode->RoadList.begin();
        for(; pos1!=pVelNode->RoadList.end() ; pos1++)
        {
            tempRoad=(*pos1);
            tempRoad ->m_Probability = (tempRoad ->Angle_Probability * tempRoad ->Distance_Probability)/K;
        }
    }
}
//void CMapMatch::PointAndPointToProbability(float node_x1, float node_y1, float node_x2, float node_y2, float Vel_Nodex1, float Vel_Nodey1, Vel_Node *pVel_Node)
//{
//	float Vel_Nodex2 = 0,Vel_Nodey2 = 0;
//	LineAndLinePoint(node_x1,nody_y1,node_x2,node_y2,Vel_Nodex1,Vel_nodey1,Vel_Nodex2,Vel_Nodey2);
//
//}
void CMapMatch::Position_Trans_From_ECEF_To_UTM(float latitude, float longitude, float e0, float n0, float *e, float *n) {
    cartographer::transform::TransformTools tft;
    double x, y;
    tft.geographic_to_utm(longitude, latitude, &x, &y);
    if(CMap_if_vrep == true){
        *e = x-500000;
        *n = y;
    }
    else{
        *e = x;
        *n = y;
    }
}

void CMapMatch::ReadTaskFile(QString taskfile_dir) {
//lsh//清空TaskList
//lsh//设置任务文件读取位置
//lsh//记录文件信息taskFile，保存上一次任务文件修改时间lastModifiedTime
//lsh//读取任务文件任务开始点对应信息存入TaskList
    if(!TaskList.isEmpty()) {
        TaskList.clear();
    }
    /*//add current position to TaskList
    if(gps_updated) {
        Task_Node temp_node;
        temp_node.lon =currentlon;
        temp_node.lat =currentlat;
        temp_node.Task_num = 0;
        temp_node.type = 0; //起点
        Position_Trans_From_ECEF_To_UTM(temp_node.lat,temp_node.lon,0,0,&temp_node.x,&temp_node.y);
        TaskList.append(temp_node);
    } else {
        qDebug()<<"CMapMatch::ReadTaskFile(),The GPS is not updated!"<<endl;
        return;
    }*/
    //add task point to Tasklist
//    QString home = QDir::homePath();
//    QFile file(home + "/taskfile/WRXLC2020.txt");//lsh//？？？又设置一遍相同路径
//    qDebug() <<  "read task file from: " + home + "/taskfile/WRXLC2020.txt" << endl;
// //    QFile file(base_dir + "/task_file_dir/KYXZ2018A.txt");
// //    qDebug() << "read task file from: " << base_dir + "/task_file_dir/KYXZ2018A.txt" << endl;
//    if(!file.open(QIODevice::ReadOnly | QIODevice::Text)) {//lsh//QIODevice::Text使end-of-line结束符被转译为'\n'；
//        qDebug()<<"ReadTaskFile:Can't open the file!"<<endl;
//    }
//    taskFile.setFile(home + "/taskfile/WRXLC2020.txt");
//    //lsh// 根据相对路径或绝对路径访问文件信息

//lshadd//0715/*
    QFile file(taskfile_dir);
    qDebug() <<  "read task file from: " + taskfile_dir << endl;
    if(!file.open(QIODevice::ReadOnly | QIODevice::Text)) {//lsh//QIODevice::Text使end-of-line结束符被转译为'\n'；
        qDebug()<<"ReadTaskFile:Can't open the file!"<<endl;
    }
    taskFile.setFile(taskfile_dir);
    //lsh// 根据相对路径或绝对路径访问文件信息
    lastModifiedTime = taskFile.lastModified();
    //lsh////保存上一次任务文件修改时间
//lshadd//0715*/
    Task_Node temp_node;
    //int all_node_null =0;
    bool find_start_task = false;
    file.seek(0);       //go back to the begin of the file
    while(!file.atEnd()) {
        QByteArray line = file.readLine();
        //lsh//readline() 方法用于从文件读取整行，包括 "\n" 字符
        //lsh//每读一行指针位置会发生改变，下一次读取时自动读取下一行
        QString str_line(line);//lsh//建立str_line存储当前行
        QString str_num = str_line.section(QRegExp("[  ]"),0,0);
        //lsh//这个函数把字符串看成是几个块，这些块由 sep 分隔，start 和 end 指定块号，返回[start，end]之间的内容
        QString str_GPS_Lon = str_line.section(QRegExp("[  ]"),1,1);
        QString str_GPS_Lat = str_line.section(QRegExp("[  ]"),2,2);
        QString str_attr = str_line.section(QRegExp("[  ]"), 4, 4);
        QString str_manu = str_line.section(QRegExp("[  ]"), 5, 5);
        int i_num = str_num.toInt();
        float f_GPS_Lat=str_GPS_Lat.toFloat();    //notice precision
        float f_GPS_Lon=str_GPS_Lon.toFloat();
        int i_attr = str_attr.toInt();
        int i_manu = str_manu.toInt();
        if(i_attr == 0){ find_start_task = true;}
        //lsh//找到任务开始点
        if(find_start_task){
            //all_node_null++;
            temp_node.lon =f_GPS_Lon;
            temp_node.lat =f_GPS_Lat;
            temp_node.type = i_attr;
            temp_node.on_road= false;
            temp_node.Task_num = i_num;
            temp_node.manu = i_manu;
            temp_node.concave_obs_det = false;
            temp_node.dynamic_obs_det = false;
            temp_node.foogy_det = false;
            temp_node.water_det = false;
            temp_node.wall_area = false;
            temp_node.ditch_area = false;
            Position_Trans_From_ECEF_To_UTM(temp_node.lat,temp_node.lon,0,0,&temp_node.x,&temp_node.y);
            TaskList.append(temp_node);
            std::cout << "  taskId:" << temp_node.Task_num << std::endl;
            std::cout << "    attr:" << temp_node.type << std::endl;
            std::cout << "    on_road:" << temp_node.on_road << std::endl;
            std::cout << "    manu:" << temp_node.manu << std::endl;
            //lsh//读取任务文件中高度为0对应信息存入TaskList
        }
    }
    if(TaskList.isEmpty()){
        ROS_WARN("Cannot find task point with attribute 0");
    }
    file.close();
    TaskBeReaded=true;
}
