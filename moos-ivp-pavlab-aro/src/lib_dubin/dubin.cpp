/*  ----------------------------------------------------------------------------
    Author: Filip Stromstad
    Circa: March 2024
    Origin: MIT, Cambridge MA
    File: dubin.cpp
    ---------------------------------------------------------------------------- 
*/  

#include "dubin.h"
#include <algorithm>

using namespace std;

DubinsPath::DubinsPath() {
    m_ps = Point();
    m_hs = 0;
    m_pg = Point();
    m_hg = 0;

    m_r1 = 0;
    m_r2 = 0;
    m_r3 = 0;

    m_length = -1;
    m_type = "None";

    m_waypoint_extra = "";
};  

//---------------------------------------------------------------
// Procedure: updateOptimalVariables()
//   Purpose: Update the optimal vars with the temporary vars

void DubinsPath::updateOptimalVariables() {
    m_l1_opt = m_l1_temp;
    m_l2_opt = m_l2_temp;
    m_l3_opt = m_l3_temp;
    m_c1_opt = m_c1_temp;
    m_c2_opt = m_c2_temp;
    m_c3_opt = m_c3_temp;
    m_t1_opt = m_t1_temp;
    m_t2_opt = m_t2_temp;
    return;
};

//---------------------------------------------------------------
// Procedure: findOptimalPath()
//   Purpose: Calculate the optimal dubinspath and store the
//            optimal variables

void DubinsPath::findOptimalPath(Point ps, double hs, Point pg, double hg, double r1, double r2, double r3, std::vector<std::string> illegal_paths) {
    m_ps = ps;
    m_hs = hs;
    m_pg = pg;
    m_hg = hg;
    m_r1 = r1;
    m_r2 = r2;
    m_r3 = r3;

    double min_length = -1; //Replace with -1??
    string min_type = "None";

    //Check if each path has feasible solution (length > 0) AND if it is the shortest path OR the first feasible path
    if (find(illegal_paths.begin(), illegal_paths.end(), "RSR") == illegal_paths.end()){
        double length_RSR = RSR();
        if (length_RSR > 0 && (length_RSR < min_length || min_length < 0)) {
            min_length = length_RSR;
            min_type = "RSR";
            updateOptimalVariables();
        }
    }

    if (find(illegal_paths.begin(), illegal_paths.end(), "LSL") == illegal_paths.end()){
        double length_LSL = LSL();
        if (length_LSL > 0 && (length_LSL < min_length || min_length < 0)) {
            min_length = length_LSL;
            min_type = "LSL";
            updateOptimalVariables();
        }
    }

    if (find(illegal_paths.begin(), illegal_paths.end(), "RSL") == illegal_paths.end()){
        double length_RSL = RSL();
        if (length_RSL > 0 && (length_RSL < min_length || min_length < 0)) {
            min_length = length_RSL;
            min_type = "RSL";
            updateOptimalVariables();
        }
    }

    if (find(illegal_paths.begin(), illegal_paths.end(), "LSR") == illegal_paths.end()){
        double length_LSR = LSR();
        if (length_LSR > 0 && (length_LSR < min_length || min_length < 0)) {
            min_length = length_LSR;
            min_type = "LSR";
            updateOptimalVariables();
        }
    }

    if (find(illegal_paths.begin(), illegal_paths.end(), "LRL") == illegal_paths.end()){
        double length_LRL = LRL();
        if (length_LRL > 0 && (length_LRL < min_length || min_length < 0)) {
            min_length = length_LRL;
            min_type = "LRL";
            updateOptimalVariables();
        }
    }
    
    if (find(illegal_paths.begin(), illegal_paths.end(), "RLR") == illegal_paths.end()){
        double length_RLR = RLR();
        if (length_RLR > 0 && (length_RLR < min_length || min_length < 0)) {
            min_length = length_RLR;
            min_type = "RLR";
            updateOptimalVariables();
        }
    }

    m_length = min_length;
    m_type = min_type;
    //Note: Segment lengths, circle centers and tangent points are updated in the functions RSR, LSL, etc.
    //      and saved in the optimal variables in the updateOptimalVariables() function
    return;
};

//---------------------------------------------------------------
// Procedure: findOptimalWaypoints()
//   Purpose: Calculate the optimal dubinspath and return 
//            waypoints on the format of XYSeglList for MOOS-IvP

string DubinsPath::findOptimalWaypoints(Point ps, double hs, Point pg, double hg, double r1, double r2, double r3, double precision, std::vector<std::string> illegal_paths){
    findOptimalPath(ps, hs, pg, hg, r1, r2, r3, illegal_paths); //Calulates optimal path and stores the optimal variables

    if (m_length < 0){
        return "No feasible path";
    }

    string waypoints = "";
    //Format of waypoints: "x1,y1:x2,y2 ... :xn,yn"
    
    //Add start point (NOTE: might not want first point as waypoint, since it is OS position)
    waypoints += to_string(m_ps.x) + "," + to_string(m_ps.y) + ":";

    //-------First segment, either L or R
    int number_of_points = floor(m_l1_opt/precision);
    if (number_of_points > 0){
        double precision_real = m_l1_opt/number_of_points;
        double angle_i = precision_real/m_r1; //Angle between each point
        string direction = m_type.substr(0, 1); //L or R
        for (int i = 1; i <= number_of_points; i++){
            Point p = rotatePoint(m_ps, m_c1_opt, angle_i*i, direction);
            waypoints += to_string(p.x) + "," + to_string(p.y) + ":";
        }
    }

    //-------Second segment, eiter L, R or S
    number_of_points = floor(m_l2_opt/precision);
    if (number_of_points > 0){
        double precision_real = m_l2_opt/number_of_points;
        string direction = m_type.substr(1, 1); //L, R or S
        if (direction == "S"){
            Point V = m_t2_opt - m_t1_opt;
            for (int i = 1; i <= number_of_points; i++){
                Point p = m_t1_opt + V*(precision_real*i/V.length());
                waypoints += to_string(p.x) + "," + to_string(p.y) + ":";
            }
        } else { //L or R
            double angle_i = precision_real/m_r3; //Angle between each point
            for (int i = 1; i <= number_of_points; i++){
                Point p = rotatePoint(m_t1_opt, m_c3_opt, angle_i*i, direction);
                waypoints += to_string(p.x) + "," + to_string(p.y) + ":";
            }
        }
    }

    //-------Third segment, either L or R
    number_of_points = floor(m_l3_opt/precision);
    if (number_of_points > 0){
        double precision_real = m_l3_opt/number_of_points;
        double angle_i = precision_real/m_r2; //Angle between each point
        string direction = m_type.substr(2, 1); //L or R
        for (int i = 1; i <= number_of_points; i++){
            Point p = rotatePoint(m_t2_opt, m_c2_opt, angle_i*i, direction);
            waypoints += to_string(p.x) + "," + to_string(p.y) + ":";
        }
        //Create one extra point after goal point
        Point p_extra = rotatePoint(m_pg, m_c2_opt, angle_i, direction);
        m_waypoint_extra = "x=" + to_string(p_extra.x) + ", y=" + to_string(p_extra.y);
    } else {
        m_waypoint_extra = "x=" + to_string(m_pg.x) + ", y=" + to_string(m_pg.y);
    }

    //Add goal point (NOTE: same as last point of third segment)
    waypoints += to_string(m_pg.x) + "," + to_string(m_pg.y);
    
    //Return points on standard XYSeglist format for MOOS-IvP
    return "pts={" + waypoints + "}";
}

//---------------------------------------------------------------
// Procedures: RSR(), LSL(), RSL(), LSR(), LRL(), RLR()
//   Purpose: Calculate the length of the optimal path for each
//            of the six possible Dubins path types

double DubinsPath::RSR() {
    //Calculate R and R circle centers
    m_c1_temp = m_ps + Point(cos(m_hs - M_PI/2), sin(m_hs - M_PI/2))*m_r1;
    m_c2_temp = m_pg + Point(cos(m_hg - M_PI/2), sin(m_hg - M_PI/2))*m_r1;

    //Calculate vector V1 between centers
    Point V1 = m_c2_temp - m_c1_temp;
    double D = V1.length();
    Point V1_hat = V1/D;

    //Check if RSR has feasible solution
    double c = (m_r1 - m_r2)/D;
    if (abs(c) > 1) {
        // cout << "RSR not possible" << endl;
        return -1;
    }

    //Calculate normal vector to V1, n_hat
    //Can be shown that c is the cosine of the angle between V1 and n_hat, simply rotate V1_hat by this angle:
    double n_x  = V1_hat.x*c - V1_hat.y*sqrt(1-pow(c, 2));
    double n_y  = V1_hat.y*c + V1_hat.x*sqrt(1-pow(c, 2));
    Point n_hat = Point(n_x, n_y);

    //Calculate vector V2, this will make up the "S" segment of "RSR"
    Point V2 = V1 - n_hat*(m_r1 - m_r2);

    //Calcaulte tangent points
    m_t1_temp = m_c1_temp + n_hat*m_r1;
    m_t2_temp = m_t1_temp + V2;

    //Calculate Dubins path length
    m_l1_temp = arcLength(m_c1_temp, m_ps, m_t1_temp, "R", m_r1);
    m_l2_temp = V2.length();
    m_l3_temp = arcLength(m_c2_temp, m_t2_temp, m_pg, "R", m_r2);

    return m_l1_temp + m_l2_temp + m_l3_temp;
};

double DubinsPath::LSL() {
    //Calculate L and L circle centers
    m_c1_temp = m_ps - Point(cos(m_hs - M_PI/2), sin(m_hs - M_PI/2))*m_r1;
    m_c2_temp = m_pg - Point(cos(m_hg - M_PI/2), sin(m_hg - M_PI/2))*m_r1;

    //Calculate vector V1 between centers
    Point V1 = m_c2_temp - m_c1_temp;
    double D = V1.length();
    Point V1_hat = V1/D;

    //Check if LSL has feasible solution
    double c = (m_r2 - m_r1)/D;
    if (abs(c) > 1) {
        // cout << "LSL not possible" << endl;
        return -1;
    }

    //Calculate normal vector to V1, n_hat
    //Can be shown that c is the cosine of the angle between V1 and n_hat, simply rotate V1_hat by this angle:
    double n_x  = V1_hat.x*c - V1_hat.y*sqrt(1-pow(c, 2));
    double n_y  = V1_hat.y*c + V1_hat.x*sqrt(1-pow(c, 2));
    Point n_hat = Point(n_x, n_y)*-1;

    //Calculate vector V2, this will make up the "S" segment of "LSL"
    Point V2 = V1 - n_hat*(m_r1 - m_r2);

    //Calcaulte tangent points
    m_t1_temp = m_c1_temp + n_hat*m_r1;
    m_t2_temp = m_t1_temp + V2;

    //Calculate Dubins path length
    m_l1_temp = arcLength(m_c1_temp, m_ps, m_t1_temp, "L", m_r1);
    m_l2_temp = V2.length();
    m_l3_temp = arcLength(m_c2_temp, m_t2_temp, m_pg, "L", m_r2);

    return m_l1_temp + m_l2_temp + m_l3_temp;
};

double DubinsPath::RSL() {
    //Calculate R and L circle centers
    m_c1_temp = m_ps + Point(cos(m_hs - M_PI/2), sin(m_hs - M_PI/2))*m_r1;
    m_c2_temp = m_pg - Point(cos(m_hg - M_PI/2), sin(m_hg - M_PI/2))*m_r1;

    //Calculate vector V1 between centers
    Point V1 = m_c2_temp - m_c1_temp;
    double D = V1.length();
    Point V1_hat = V1/D;

    //Check if RSL has feasible solution
    double c = (m_r1 + m_r2)/D;
    if (abs(c) > 1) {
        // cout << "RSL not possible" << endl;
        return -1;
    }

    //Calculate normal vector to V1, n_hat
    //Can be shown that c is the cosine of the angle between V1 and n_hat, simply rotate V1_hat by this angle:
    double n_x  = V1_hat.x*c - V1_hat.y*sqrt(1-pow(c, 2));
    double n_y  = V1_hat.y*c + V1_hat.x*sqrt(1-pow(c, 2));
    Point n_hat = Point(n_x, n_y);

    //Calculate vector V2, this will make up the "S" segment of "RSL"
    Point V2 = V1 - n_hat*(m_r1 + m_r2);

    //Calcaulte tangent points
    m_t1_temp = m_c1_temp + n_hat*m_r1;
    m_t2_temp = m_t1_temp + V2;

    //Calculate Dubins path length
    m_l1_temp = arcLength(m_c1_temp, m_ps, m_t1_temp, "R", m_r1);
    m_l2_temp = V2.length();
    m_l3_temp = arcLength(m_c2_temp, m_t2_temp, m_pg, "L", m_r2);

    return m_l1_temp + m_l2_temp + m_l3_temp;
};

double DubinsPath::LSR() {
    //Calculate L and R circle centers
    m_c1_temp = m_ps - Point(cos(m_hs - M_PI/2), sin(m_hs - M_PI/2))*m_r1;
    m_c2_temp = m_pg + Point(cos(m_hg - M_PI/2), sin(m_hg - M_PI/2))*m_r1;

    //Calculate vector V1 between centers
    Point V1 = m_c2_temp - m_c1_temp;
    double D = V1.length();
    Point V1_hat = V1/D;

    //Check if LSR has feasible solution
    double c = -(m_r1 + m_r2)/D;
    if (abs(c) > 1) {
        // cout << "LSR not possible" << endl;
        return -1;
    }

    //Calculate normal vector to V1, n_hat
    //Can be shown that c is the cosine of the angle between V1 and n_hat, simply rotate V1_hat by this angle:
    double n_x  = V1_hat.x*c - V1_hat.y*sqrt(1-pow(c, 2));
    double n_y  = V1_hat.y*c + V1_hat.x*sqrt(1-pow(c, 2));
    Point n_hat = Point(n_x, n_y)*-1;

    //Calculate vector V2, this will make up the "S" segment of "LSR"
    Point V2 = V1 - n_hat*(m_r1 + m_r2);

    //Calcaulte tangent points
    m_t1_temp = m_c1_temp + n_hat*m_r1;
    m_t2_temp = m_t1_temp + V2;

    //Calculate Dubins path length
    m_l1_temp = arcLength(m_c1_temp, m_ps, m_t1_temp, "L", m_r1);
    m_l2_temp = V2.length();
    m_l3_temp = arcLength(m_c2_temp, m_t2_temp, m_pg, "R", m_r2);

    return m_l1_temp + m_l2_temp + m_l3_temp;
};

double DubinsPath::LRL() {
    //Calculate L and L start and goal circle centers
    m_c1_temp = m_ps - Point(cos(m_hs - M_PI/2), sin(m_hs - M_PI/2))*m_r1;
    m_c2_temp = m_pg - Point(cos(m_hg - M_PI/2), sin(m_hg - M_PI/2))*m_r1;

    //Calculate vector V1 between centers
    Point V1 = m_c2_temp - m_c1_temp;

    //Check if LRL has feasible solution
    double D = V1.length();
    if (abs(D) > (m_r1 + m_r2 + 2*m_r3)) {
        // cout << "LRL not possible" << endl;
        return -1;
    }

    //Calculate theta and circle center 3
    double theta_temp = acos((pow(D,2) + pow(m_r1,2) - pow(m_r2,2) + 2*m_r3*(m_r1 - m_r2))/(2*D*(m_r1 + m_r3)));
    double theta = atan2(V1.y, V1.x) + theta_temp;
    m_c3_temp = m_c1_temp + Point(cos(theta), sin(theta))*(m_r1 + m_r3);

    //Calculate vector V2 and V3
    Point V2 = m_c3_temp - m_c1_temp;
    Point V3 = m_c3_temp - m_c2_temp;

    //Calcaulte tangent points
    m_t1_temp = m_c1_temp + V2/V2.length()*m_r1;
    m_t2_temp = m_c2_temp + V3/V3.length()*m_r2;

    //Calculate Dubins path length
    m_l1_temp = arcLength(m_c1_temp, m_ps, m_t1_temp, "L", m_r1);
    m_l2_temp = arcLength(m_c3_temp, m_t1_temp, m_t2_temp, "R", m_r3);
    m_l3_temp = arcLength(m_c2_temp, m_t2_temp, m_pg, "L", m_r2);

    return m_l1_temp + m_l2_temp + m_l3_temp;
};

double DubinsPath::RLR() {
    //Calculate R and R start and goal circle centers
    m_c1_temp = m_ps + Point(cos(m_hs - M_PI/2), sin(m_hs - M_PI/2))*m_r1;
    m_c2_temp = m_pg + Point(cos(m_hg - M_PI/2), sin(m_hg - M_PI/2))*m_r1;

    //Calculate vector V1 between centers
    Point V1 = m_c2_temp - m_c1_temp;

    //Check if RLR has feasible solution
    double D = V1.length();
    if (abs(D) > (m_r1 + m_r2 + 2*m_r3)) {
        // cout << "RLR not possible" << endl;
        return -1;
    }

    //Calculate theta and circle center 3
    double theta_temp = M_PI - acos((pow(D,2) + pow(m_r1,2) - pow(m_r2,2) + 2*m_r3*(m_r1 - m_r2))/(2*D*(m_r1 + m_r3)));
    double theta = atan2(V1.y, V1.x) + theta_temp;
    m_c3_temp = m_c1_temp - Point(cos(theta), sin(theta))*(m_r1 + m_r3);

    //Calculate vector V2 and V3
    Point V2 = m_c3_temp - m_c1_temp;
    Point V3 = m_c3_temp - m_c2_temp;

    //Calcaulte tangent points
    m_t1_temp = m_c1_temp + V2/V2.length()*m_r1;
    m_t2_temp = m_c2_temp + V3/V3.length()*m_r2;

    //Calculate Dubins path length
    m_l1_temp = arcLength(m_c1_temp, m_ps, m_t1_temp, "R", m_r1);
    m_l2_temp = arcLength(m_c3_temp, m_t1_temp, m_t2_temp, "L", m_r3);
    m_l3_temp = arcLength(m_c2_temp, m_t2_temp, m_pg, "R", m_r2);

    return m_l1_temp + m_l2_temp + m_l3_temp;
};

//---------------------------------------------------------------
// Procedure: arcLength()
//   Purpose: Calculate the length of an arc from point ps to pe,
//            with center point pc, in the direction "dir" (L or R)

double arcLength(Point pc, Point ps, Point pe, std::string dir, double r) {
    //Calculate vector from center to start and end point
    Point Vs = ps - pc;
    Point Ve = pe - pc;

    double theta = atan2(Ve.y, Ve.x) - atan2(Vs.y, Vs.x);
    if (theta < 0 && dir == "L") {
        theta += 2*M_PI;
    } else if (theta > 0 && dir == "R") {
        theta -= 2*M_PI;
    }

    return r*abs(theta);
};

//---------------------------------------------------------------
// Procedure: rotatePoint()
//   Purpose: Rotate the point p about the point c with angle 
//            "angle" in radians along the direction "dir"

Point rotatePoint(Point p, Point c, double angle, std::string dir) {
    if (dir == "R") {
        angle = -angle;
    }
    double x = c.x + (p.x - c.x)*cos(angle) - (p.y - c.y)*sin(angle);
    double y = c.y + (p.x - c.x)*sin(angle) + (p.y - c.y)*cos(angle);
    return Point(x, y);
}


Point::Point() {
    x = 0;
    y = 0;
};

Point::Point(double x_in, double y_in) {
    x = x_in;
    y = y_in;
};

double Point::length() {
    return sqrt(pow(x, 2) + pow(y, 2));
};

Point Point::operator+(const Point& p) {
    return Point(x + p.x, y + p.y);
};

Point Point::operator-(const Point& p) {
    return Point(x - p.x, y - p.y);
};

Point Point::operator*(const double& s) {
    return Point(x * s, y * s);
};

Point Point::operator/(const double& s) {
    return Point(x / s, y / s);
};

Pose::Pose() {
    p = Point();
    h = 0;
};

Pose::Pose(Point p_in, double h_in) {
    p = p_in;
    h = h_in;
};
