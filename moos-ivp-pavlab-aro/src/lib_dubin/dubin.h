/*  ----------------------------------------------------------------------------
    Author: Filip Stromstad
    Circa: March 2024
    Origin: MIT, Cambridge MA
    File: dubin.h
    Desc: 
    Status: 
    ---------------------------------------------------------------------------- 
*/  
#pragma once

#include <iostream>
#include <cmath>
#include <vector>
#include <string>

class Point {
    public:
        Point();
        Point(double x_in, double y_in);

        double x;
        double y;
        double length();

        //Operator overloading
        Point operator+(const Point& p);
        Point operator-(const Point& p);
        Point operator*(const double& s);
        Point operator/(const double& s);
};

class Pose {
    public:
        Pose();
        Pose(Point p_in, double h_in);

        Point p;
        double h;
};

class DubinsPath {
    public:
        DubinsPath();

    public:
        void findOptimalPath(Point ps, double hs, Point pg, double hg, double r1, double r2, double r3, std::vector<std::string> illegal_paths={});
        std::string findOptimalWaypoints(Point ps, double hs, Point pg, double hg, double r1, double r2, double r3, double precision, std::vector<std::string> illegal_paths={});
        void updateOptimalVariables();
        
    private:
        double RSR();
        double LSL();
        double RSL();
        double LSR();
        double LRL();
        double RLR();

    public:
        double m_length;        //Length of the optimal path
        std::string m_type;     //Type of the optimal path

    private: //Input variables
        Point m_ps;             //Starting point
        double m_hs;            //Starting heading
        Point m_pg;             //Goal point
        double m_hg;            //Goal heading

        double m_r1;            //Radius of the first turn
        double m_r2;            //Radius of the second turn
        double m_r3;            //Radius of the third turn (for LRL and RLR paths)

    private: //Temp variables
        double m_l1_temp;
        double m_l2_temp;
        double m_l3_temp;
        Point m_c1_temp;
        Point m_c2_temp;
        Point m_c3_temp;
        Point m_t1_temp;
        Point m_t2_temp;

    private: //Optimal variables
        double m_l1_opt;
        double m_l2_opt;
        double m_l3_opt;
        Point m_c1_opt;
        Point m_c2_opt;
        Point m_c3_opt;
        Point m_t1_opt;
        Point m_t2_opt;

    public: //Extra variables
        std::string m_waypoint_extra;

};

// Utility functions for geometry
double arcLength(Point pc, Point ps, Point pe, std::string dir, double r);
Point rotatePoint(Point p, Point c, double angle, std::string dir);