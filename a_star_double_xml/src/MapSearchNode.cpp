#include "qmath.h"
//#include "MapSearchNode.h"
#include "../include/MapSearchNode.h"

MapSearchNode::~MapSearchNode() {}

float MapSearchNode::GoalDistanceEstimate(MapSearchNode *nodeGoal) {
    float xd = fabs(float((float)x - (float)nodeGoal->x));
    float yd = fabs(float((float)y - (float)nodeGoal->y));
    return sqrt(xd*xd + yd*yd);
}

bool MapSearchNode::IsGoal(MapSearchNode *nodeGoal) {
    if(( x == nodeGoal ->x) && (y == nodeGoal->y))
        return true;
    else
        return false;
}

float MapSearchNode::GetCost( MapSearchNode *successor) {
    float xd = fabs((float)(x - successor ->x));
    float yd = fabs((float)(y - successor ->y));
    return (xd + yd);
}

bool MapSearchNode::IsSameState(MapSearchNode *rhs) {
    if((x == rhs ->x) && ( y == rhs->y))
        return true;
    else
        return false;
}