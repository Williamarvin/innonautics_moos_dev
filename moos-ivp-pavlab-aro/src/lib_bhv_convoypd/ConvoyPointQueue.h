
/* **************************************************************
  NAME: Raymond Turrisi 
  ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA
  FILE: ConvoyPointQueue.h
  CIRC: November 2023
  DESC:
    This file contains two datastructures/classes: ConvoyPoints, and 
    ConvoyPointQueue. A ConvoyPoint is an enhanced XYPoint with information
    and encoded leader states at the time it was seeded. A ConvoyPointQueue
    wraps a linkedlist to manage a queue of ConvoyPoints with some utility
    functions, such as computing the distance between all the XYPoints
    in the queue. 

  LICENSE:
    This is unreleased BETA code. No permission is granted or
    implied to use, copy, modify, and distribute this software 
    except by the author(s), or those designated by the author.
************************************************************** */

#pragma once

#include <list> 
#include <map> 
#include <string>
#include "IvPBehavior.h"
#include "MBUtils.h"
#include <cmath>
#include "GeomUtils.h"
#include "XYPoint.h"

/**
 * @brief A convoy point is an enhanced XYPoint with additional meta data to support convoying, with convenience functions
 * 
 */
class ConvoyPoint {
  public: 

  XYPoint p;
  std::map<std::string,std::string> meta;
  double seed_time;

  double leader_heading;
  double leader_heading_rate;
  double leader_speed;

  ConvoyPoint();

  ConvoyPoint(XYPoint xyp);

  ConvoyPoint(std::string strrep);

  /**
   * @brief Set the convoy point seed time
   * 
   * @param t time
   */
  void set_st(double t);
  
  /**
   * @brief Set leader speed
   * 
   * @param spd  speed
   */
  void set_spd(double spd);

  void set_lh(double h);

  void set_lhr(double hdot);

  void add_meta(std::string k, std::string v);

  double dist(ConvoyPoint trg);

  double dist(XYPoint trg);

  public:
  std::string repr();

  void unpack(std::string cp_str);

};


/**
 * @brief A wrapped queue of ConvoyPoints which contains convenience functions
 * 
 */
class ConvoyPointQueue {
  public:
  
    std::list<ConvoyPoint> m_points;

    ConvoyPointQueue();

    void add_point(ConvoyPoint cp);

    ConvoyPoint dequeue();

    size_t size();

    ConvoyPoint front();

    ConvoyPoint back();

    /**
     * @brief Get the total approximated distance between all the points in the point queue
     * 
     * @return double 
     */
    double get_dist_to_target();

    /**
     * @brief Seriaize the ConvoyPoint queue's representation into a string data structure with a default delimeter
     * 
     * @return std::string 
     */
    std::string repr();

    /**
     * @brief Seriaize the ConvoyPoint queue's representation into a string data structure with a chosen delimeter
     * 
     * @return std::string 
     */
    std::string repr(std::string delim);

};