#include <string>
#include "XYPoint.h"
#include <list>
#include "AgentStateInfo.h"
#include <iterator>
#include <cmath>
#include "CtrlPointQueue.h"

CtrlPoint::CtrlPoint(){

};
CtrlPoint::CtrlPoint(double x, double y) : x(x), y(y), XYPoint_state(x, y), agent_state() {}

CtrlPoint::CtrlPoint(const CtrlPoint &p) : x(p.x), y(p.y), XYPoint_state(p.XYPoint_state), agent_state(p.agent_state) {}

CtrlPoint::CtrlPoint(XYPoint p) : x(p.get_vx()), y(p.get_vy()), XYPoint_state(p)
{
    agent_state.q(0) = p.get_vx();
    agent_state.q(1) = p.get_vy();
}
CtrlPoint::CtrlPoint(const AgentStateInfo &a) : x(a.q(0)), y(a.q(1)), XYPoint_state(a.q[0], a.q[1]), agent_state(a)
{
    XYPoint_state.set_label(agent_state.repr());
    XYPoint_state.set_msg(agent_state.repr());
}
CtrlPoint::~CtrlPoint(){

};

double CtrlPoint::dist(CtrlPoint p)
{
    return hypot(x - p.x, y - p.y);
}

double CtrlPoint::dist(double x, double y)
{
    return hypot(this->x - x, this->y - y);
}

std::string CtrlPoint::repr()
{
    char buffer[256];
    snprintf(buffer, sizeof(buffer), "(%.2f,%.2f)", x, y);
    return std::string(buffer);
}

/*
    The objective for this class and file is to create a PointQueue class, which handles a
    collection of intermediate points, while also linking the distance between agents/termination points.
    This distance along the arbitrary trajectory becomes R

    Current Agent ----- * ----- * ----- * ----- * ----- (Leader)
    We'll always consider the distance from the current agent to the first point
    Circumstantially, if we are not in a convoy, there is not necessarily a leader point - there is
    a live point which is projected from the leader which it tracks in real time
*/

CtrlPointQueue::CtrlPointQueue()
{
    m_min_point_sep = 0;
    m_max_queue_length = 0;
    m_current_queue_length = 0;
};
CtrlPointQueue::CtrlPointQueue(double min_point_sep, double max_queue_length) : m_min_point_sep(min_point_sep), m_max_queue_length(max_queue_length) {
    m_current_queue_length = 0;
}
CtrlPointQueue::~CtrlPointQueue(){};

void CtrlPointQueue::addPoint(CtrlPoint point)
{
    // Check to see if the new point is further the min point separation distance, if it is, add it, otherwise, we skip this point
    // If we add a new point, and it makes the queue too long, we remove the oldest point from the queue, while updating
    // the queues new distance - we do this by computing the distance between the two oldest points, and subtracting this from the running length

    // If the new proposed point is not greater than the minimum point separation, we skip this point, otherwise, we update our CtrlPointQueue

    if (m_point_queue.size() == 0)
    {
        m_point_queue.push_back(point);
    }
    else if (m_point_queue.back().dist(point) > m_min_point_sep)
    {
        m_point_queue.push_back(point);
        m_current_queue_length = getTrajectoryOdom(m_point_queue.front().x, m_point_queue.front().y);
        // If adding this new point exceed the max queue length, we remove the oldest point and update our maintained distance
        while (m_current_queue_length > m_max_queue_length && m_point_queue.size() > 1)
        {
            // If the queue length is greater than the allowable length, we remove the oldest point, and update the distance of the point queue
            // If however we only have one element in the point queue and the queue length still some larger value, we messed up
            m_point_queue.pop_front();
        }
    }

    // If we are here, we have an issue - we should never have a queue length greater than the max queue length with only one element
}

CtrlPoint CtrlPointQueue::peekPoint()
{
    // Pop the next element in the
    return m_point_queue.front();
}

CtrlPoint CtrlPointQueue::popPoint()
{
    // Remove a point and update the queue length, and return the popped point
    CtrlPoint p = m_point_queue.front();
    if(m_point_queue.size() > 1)
        m_point_queue.pop_front();
    m_current_queue_length = getTrajectoryOdom(m_point_queue.front().x, m_point_queue.front().y);
    return p;
}

// Ownship, target ship - used for convoying
double CtrlPointQueue::getTrajectoryOdom(double os_x, double os_y, double ts_x, double ts_y)
{
    // From an agents current point, and the position of their target, find the distance from
    // ownship -> first point in queue + length along queue + last point in queue -> target agent
    double odom = 0;
    if (m_point_queue.size() > 0)
    {
        odom += m_point_queue.front().dist(os_x, os_y);
        for (auto it = m_point_queue.begin(); it != m_point_queue.end(); ++it)
        {
            if (std::next(it) != m_point_queue.end())
            {
                odom += it->dist(*std::next(it));
            }
        }
        return odom + m_point_queue.back().dist(ts_x, ts_y);
    }
    else
    {
        return hypot(os_x - ts_x, os_y - ts_y);
    }
}

// Ownship - used for parallel formation
double CtrlPointQueue::getTrajectoryOdom(double os_x, double os_y)
{
    // From an agents current point, and the position of their target, find the distance from
    // ownship -> first point in queue + length along queue + last point in queue -> target agent
    double odom = 0;
    if (m_point_queue.size() > 0)
    {
        odom += m_point_queue.front().dist(os_x, os_y);
        for (auto it = m_point_queue.begin(); it != m_point_queue.end(); ++it)
        {
            if (std::next(it) != m_point_queue.end())
            {
                odom += it->dist(*std::next(it));
            }
        }
        return odom;
    }
    else
    {
        return 0;
    }
}

void CtrlPointQueue::clear()
{
    m_point_queue.clear();
    m_current_queue_length = 0;
}

uint32_t CtrlPointQueue::size()
{
    return m_point_queue.size();
}

std::string CtrlPointQueue::repr()
{
    std::string result = "";
    for (auto it = m_point_queue.begin(); it != m_point_queue.end(); ++it)
    {
        result += it->repr() + " ";
    }
    return result;
}

XYSegList CtrlPointQueue::to_seglist()
{
    XYSegList result;
    if (m_point_queue.size() > 0)
    {
        result.add_vertex(m_point_queue.front().x, m_point_queue.front().y);
        for (auto it = m_point_queue.begin(); it != m_point_queue.end(); ++it)
        {
            result.add_vertex(XYPoint(it->x, it->y));
        }
    }
    return result;
}