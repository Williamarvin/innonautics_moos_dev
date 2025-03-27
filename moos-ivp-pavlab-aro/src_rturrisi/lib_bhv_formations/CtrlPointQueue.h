#pragma once

#include <string>
#include "XYPoint.h"
#include <list>
#include "AgentStateInfo.h"
#include <iterator>
#include <cmath>
#include "XYSegList.h"

class CtrlPoint
{
public:
    double x;
    double y;
    XYPoint XYPoint_state;
    AgentStateInfo agent_state;

    CtrlPoint();
    CtrlPoint(double x, double y);
    CtrlPoint(XYPoint p);
    CtrlPoint(const AgentStateInfo &a);
    CtrlPoint(const CtrlPoint &p);
    ~CtrlPoint();

    double dist(CtrlPoint p);

    double dist(double x, double y);

    std::string repr();
};

/*
    The objective for this class and file is to create a PointQueue class, which handles a
    collection of intermediate points, while also linking the distance between agents/termination points.
    This distance along the arbitrary trajectory becomes R

    Current Agent ----- * ----- * ----- * ----- * ----- (Leader)
    We'll always consider the distance from the current agent to the first point
    Circumstantially, if we are not in a convoy, there is not necessarily a leader point - there is
    a live point which is projected from the leader which it tracks in real time
*/

class CtrlPointQueue
{
public:
    CtrlPointQueue();
    CtrlPointQueue(double min_point_sep, double max_queue_length);
    ~CtrlPointQueue();

    void addPoint(CtrlPoint point);

    CtrlPoint peekPoint();

    CtrlPoint popPoint();

    // Ownship, target ship - used for convoying
    double getTrajectoryOdom(double os_x, double os_y, double ts_x, double ts_y);

    // Ownship - used for parallel formation
    double getTrajectoryOdom(double os_x, double os_y);

    void clear();

    uint32_t size();

    std::string repr();
    XYSegList to_seglist();


    std::list<CtrlPoint> m_point_queue;
    double m_min_point_sep = 0.5;   // meters
    double m_max_queue_length = 10; // meters
    double m_current_queue_length = 0;
};