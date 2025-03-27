
/* **************************************************************
  NAME: Raymond Turrisi 
  ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA
  FILE: ConvoyPointQueue.cpp
  CIRC: November 2023

  LICENSE:
    This is unreleased BETA code. No permission is granted or
    implied to use, copy, modify, and distribute this software 
    except by the author(s), or those designated by the author.
************************************************************** */
#include "ConvoyPointQueue.h"
#include "MBUtils.h"
#include <cmath>


ConvoyPoint::ConvoyPoint()
{
    p = XYPoint();
    seed_time = 0;
    leader_heading = 0;
    leader_heading_rate = 0;
    leader_speed = 0;
}
ConvoyPoint::ConvoyPoint(XYPoint xyp)
{
    p = xyp;
    seed_time = 0;
    leader_heading = 0;
    leader_heading_rate = 0;
    leader_speed = 0;
}
ConvoyPoint::ConvoyPoint(std::string strrep)
{
    unpack(strrep);
}
void ConvoyPoint::set_st(double t)
{
    seed_time = t;
    add_meta("seed_time", to_string(t));
}

void ConvoyPoint::set_spd(double spd)
{
    leader_speed = spd;
    add_meta("leader_speed", to_string(spd));
}

void ConvoyPoint::set_lh(double h)
{
    leader_heading = h;
    add_meta("leader_heading", to_string(h));
}
void ConvoyPoint::set_lhr(double hdot)
{
    leader_heading_rate = hdot;
    add_meta("leader_heading_rate", to_string(hdot));
}
void ConvoyPoint::add_meta(std::string k, std::string v)
{
    meta[k] = v;
}

double ConvoyPoint::dist(ConvoyPoint trg)
{
    return dist(trg.p);
}

double ConvoyPoint::dist(XYPoint trg)
{
    double dy = trg.get_vy() - p.get_vy();
    double dx = trg.get_vx() - p.get_vx();
    return sqrt(dy * dy + dx * dx);
}

XYPoint p;
std::map<std::string, std::string> meta;
double seed_time = 0;

// These meta data are used for smoothing the control inputs
double leader_heading = 0;
double leader_heading_rate = 0;
double leader_speed = 0;

std::string ConvoyPoint::repr()
{
    std::string meta_str = "{";
    std::map<std::string, std::string>::iterator it = meta.begin();

    for (; it != meta.end(); ++it)
    {
        meta_str += it->first + ":" + it->second;
        if (std::next(it) != meta.end())
            meta_str += ",";
    }
    meta_str += "}";
    return "{" + p.get_spec() + "}|" + meta_str;
}

void ConvoyPoint::unpack(std::string cp_str)
{
    std::vector<string> cp_partitioned = parseString(cp_str, "|");
    std::string xyp_spec = cp_partitioned[0] + ",";
    std::string cp_meta = cp_partitioned[1];

    std::string chunk, val;

    // Unpack the XYPoint specs
    while ((chunk = biteString(xyp_spec, ',')) != "")
    {
        size_t idx = 0;
        if ((idx = chunk.find("x=")) != std::string::npos)
        {
            val = chunk.substr(idx + 2);
            p.set_vx(stod(val));
        }
        else if ((idx = chunk.find("y=")) != std::string::npos)
        {
            val = chunk.substr(idx + 2);
            p.set_vy(stod(val));
        }
        else if ((idx = chunk.find("label=")) != std::string::npos)
        {
            val = chunk.substr(idx + 6);
            p.set_label(val);
        }
        else if ((idx = chunk.find("vertex_color=")) != std::string::npos)
        {
            val = chunk.substr(idx + 13);
            p.set_vertex_color(val);
        }
        else if ((idx = chunk.find("label_color=")) != std::string::npos)
        {
            val = chunk.substr(idx + 12);
            p.set_label_color(val);
        }
        else if ((idx = chunk.find("vertex_size=")) != std::string::npos)
        {
            val = chunk.substr(idx + 12);
            p.set_vertex_size(stod(val));
        }
        else if ((idx = chunk.find("id=")) != std::string::npos)
        {
            val = chunk.substr(idx + 3);
            p.set_id(val);
        }
    }

    // Unpack the attached meta data
    cp_meta.erase(cp_meta.find("{"), 1);
    cp_meta.erase(cp_meta.find("}"), 1);

    std::string k, v, kv_pair;
    std::vector<std::string> kv_pairs;
    kv_pairs = parseString(cp_meta, ',');
    for (int i = 0; i < kv_pairs.size(); i++)
    {
        k = biteString(kv_pairs[i], ':');
        v = kv_pairs[i];
        // We populate both the dictionary and the core variables - we may want to propagate variables later for analysis which aren't used in smoothing
        meta[k] = v;
        if (k == "seed_time")
        {
            seed_time = stod(v);
        }
        else if (k == "leader_heading")
        {
            leader_heading = stod(v);
        }
        else if (k == "leader_heading_rate")
        {
            leader_heading_rate = stod(v);
        }
        else if (k == "leader_speed")
        {
            leader_speed = stod(v);
        }
    }
}

ConvoyPointQueue::ConvoyPointQueue()
{
}

void ConvoyPointQueue::add_point(ConvoyPoint cp)
{
    m_points.push_back(cp);
}

ConvoyPoint ConvoyPointQueue::dequeue()
{
    ConvoyPoint cp = m_points.front();
    m_points.pop_front();
    return cp;
}

ConvoyPoint ConvoyPointQueue::front()
{
    ConvoyPoint cp = m_points.front();
    return cp;
}

ConvoyPoint ConvoyPointQueue::back()
{
    ConvoyPoint cp = m_points.back();
    return cp;
}

size_t ConvoyPointQueue::size() {
    return m_points.size();
}


double ConvoyPointQueue::get_dist_to_target()
{
    // Check to see how many points are in the queue
    size_t samples = m_points.size();
    double dx = 0, dy = 0;
    double total_convoy_dist = 0;
    // (os)  x x x x x x x  (cs)
    // If there are no points in the queue, we consider the straight distance to the target
    if (!samples)
    {
        total_convoy_dist = 0;
    }
    else
    {
        // If there are  points in the queue, we consider the distance from the ownship to the first point, the distance between all the points, and the distance from the end of the list of points to the target
        std::list<ConvoyPoint>::iterator it = m_points.begin();
        ConvoyPoint prev_point(*it);
        ++it;
        for (; it != m_points.end(); ++it)
        {
            total_convoy_dist += prev_point.dist(*it);
            prev_point = *it;
        }
    }
    return total_convoy_dist;
}

std::string ConvoyPointQueue::repr()
{
    return repr("^");
}

std::string ConvoyPointQueue::repr(std::string delim)
{
    std::string result = "[";
    for (ConvoyPoint cp : m_points)
    {
        result += cp.repr();
        if (cp.dist(m_points.back()) > 0.01)
        {
            result += delim;
        }
    }
    result += "]";
    return result;
}
