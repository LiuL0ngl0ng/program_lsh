#ifndef MAPMATCH_H
#define MAPMATCH_H

#include <iostream>
#include <assert.h>
#include <stdio.h>
#include <algorithm>
#include <QList>
#include <QDateTime>
#include <QFileInfo>

#ifndef Q_MOC_RUN
#include "transform_tools/transform_interface.hpp"
#endif

#define VERTICAL 1000

/**
 * R_Probability,Angle_Probability,m_Probability分别为距离概率，夹角的概率和合成的概率。
 * R_Slope为节点与节点间的斜率，R_Intercept为道路的截距
 * Vel_Slope为当前点的斜率相当于此点的轨迹的导数，IncludeAngle为此点的斜率与投影直线间的夹角,
 * Road_nodex,Road_nodey分别是点与直线间的交点坐标
 */
struct Road_Line {
    float Distance_Probability,R_Slope,R_Intercept,Angle_Probability,m_Probability,Dist,IncludeAngle,Road_nodex,Road_nodey;
};

/// \brief 描述任务点
struct Task_Node {
    float lat,lon,x,y;
    int Task_num;
    int type;       //任务点属性
    bool on_road;    //当前任务点是否能够匹配到拓扑地图上
    double vlimit; //当前任务点的最大速度限制
    bool concave_obs_det;   //在当前任务点是否开启凹碍物检测，true：开启 false:关闭
    bool dynamic_obs_det;   //在当前任务点是否开启动态障碍物检测， truw:开启 false:关闭
    bool foogy_det;     //在当前任务点是否开启烟雾区检测，true:开启 false:关闭
    bool water_det;     //在当前任务点是否开启水检测，true:开启 false:关闭
    bool wall_area;         /*垂直墙区域，true:开启 false:关闭*/
    bool ditch_area;        /*壕沟区域，true:开启 false:关闭*/
    int manu;       //人为控制属性
    QList<Road_Line> RoadList;
};

/// \brief 描述重规划起始点
struct ReStartNode {
    float lat,lon,x,y;
    int ID;
    QList<Road_Line> RoadList;
};

/// \brief 辅助计算
struct Vel_Node {
    float x,y,Vel_Slope;//x,y分别是车的位置点，RoadList用于存放每个车节点的候选道路
    //CPtrList RoadList;
    QList<Road_Line*> RoadList;      //tangbo
};

/// \brief 计算工具包以及读入任务点类
class CMapMatch {
public:
    CMapMatch();
    ~CMapMatch(void);
public:
    /**
     * 两点变成斜率，相当于某点求导，当为垂直时其斜率取为VERTICAL
     */
    void PointToSlope(float node_x1,float node_y1,float node_x2,float node_y2,float &node_Slope);

    /**
     * 点的垂线与路网直线的交点
     */
    void LineAndLinePoint(float nodex1,float nodey1,float nodex2,float ndoey2,float Pointx1,float Pointy1,float &Pointx2,float &Pointy2);

    /**
     * 点与直线间交点之间的距离(两种情况，在直线上)
     */
    void PointToLineDistance(float Pointx1,float Pointy1,float Pointx2,float Pointy2,float &Dist);

    /**
     * 求两直线的夹角
     */
    void SlopeToIncludeAngle(float k1,float k2,float &IncludeAngle);

    /**
     * 从点到直线的距离转化成概率
     */
    void DistanceToProbability(Vel_Node *pVelNode);

    /**
     * 从夹角转化成概率
     */
    void IncludeToProbability(Vel_Node *pVelNode);

    /**
     * 正交融合后的概率
     */
    void OrthonormalToProbability(Vel_Node *pVelNode);
    //void PointAndPointToProbability(float node_x1,float node_y1,float node_x2,float node_y2,float Vel_Nodex1,float Vel_Nodey1,Vel_Node *pVel_Node);

    /**
     * 从目标目录中读入并解析任务点序列
     */
    void ReadTaskFile(QString taskfile_dir);

    /**
     * 坐标转换
     */
    void Position_Trans_From_ECEF_To_UTM(float latitude, float longitude, float e0, float n0, float *e, float *n);

public:
    ///当前车辆位置经纬度
    float currentlon,currentlat;
    ///gps更新标志
    bool gps_updated;
    ///读入任务点完成标志
    bool TaskBeReaded;
    ///保存任务点序列
    QList<Task_Node> TaskList;
    ///保存任务文件信息
    QFileInfo taskFile;
    ///保存上一次任务文件修改时间
    QDateTime lastModifiedTime;
    ///lsh//vrep仿真标志
    bool CMap_if_vrep;
};


#endif // MAPMATCH_H
