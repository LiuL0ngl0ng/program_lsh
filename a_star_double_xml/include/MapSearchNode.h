#ifndef MAPSEARCHNODE_H
#define MAPSEARCHNODE_H

#include "stlastar.h"
#include <iostream>
#include <qmath.h>
using namespace std;

#include <QList>
#include "QMutexLocker"
#include "QMutex"

/// \brief 路网节点类
class MapSearchNode {//lsh//路点存储格式，包含父子节点和相关函数
public:
    ~MapSearchNode(void);

public:
    ///x,y是当前点的坐标，theta是当前点下一段路的方向，s是当前点到起点的距离
    float x,y,theta,s;
    ///经纬度
    float lat,lon;
    ///保留
    float dis,u;
    ///该节点属于几条道路
    unsigned int road_count;
    ///所属道路ID
    int road_id;
    ///节点ID
    int node_id;
    ///是否是路口点，以后该属性还需要丰富，比如是否是匝道
    bool intersection;
    ///保留
    bool bConnection;
    ///子节点数
    int successorNum;
    ///父节点数
    int parentNum;
    ///记录路点的属性， 0：起点 1：终点 2：必经点 3：岔道口 4：搜索区域
    int type;
    ///当前节点的最大限速，m/s
    double vlimit;
    ///凹碍物检测，true：开启 false:关闭
    bool concave_obs_det;
    ///动态障碍物检测， truw:开启 false:关闭
    bool dynamic_obs_det;
    ///烟雾区检测，true:开启 false:关闭
    bool foogy_det;
    ///水检测，true:开启 false:关闭
    bool water_det;
    ///垂直墙区域，true:开启 false:关闭
    bool wall_area;
    ///壕沟区域，true:开启 false:关闭
    bool ditch_area;
    ///子节点链表，即是MapSearchNode性质的点中从此链表找出此子节点
    QList<MapSearchNode*> successorList;
    ///父节点踩链表，从此链表中找到父节点
    QList<MapSearchNode*> parentList;

    /**
     * 构造函数
     */
    MapSearchNode()
    {
        x =0;
        y = 0;
        lat=0;
        lon=0;
        road_id = 0;
        node_id = 0;
        type = 2;
        vlimit = 10;
        concave_obs_det = false;
        dynamic_obs_det = false;
        foogy_det = false;
        water_det = false;
        wall_area = false;
        ditch_area = false;
        intersection=false;
        road_count = 0;
        successorNum = 0;
        successorList.clear();
        parentNum = 0;
        parentList.clear();
        bConnection = false;
        type = -1;
    }

    /**
     * 评估当前节点至目标节点的距离
     */
    float GoalDistanceEstimate( MapSearchNode *nodeGoal);

    /**
     * 判断当前节点是否是目标节点
     */
    bool IsGoal(MapSearchNode *nodeGoal);

    //AStarSearch is a templete.
    /**
     * 是否同一个节点
     */
    bool IsSameState(MapSearchNode *rhs);

    /**
     * 与相邻节点的消耗
     */
    float GetCost(MapSearchNode *rhs);
};



#endif // MAPSEARCHNODE_H
