/*  ----------------------------------------------------------------------------
    Author: Filip Stromstad
    Circa: March 2024
    Origin: MIT, Cambridge MA
    File: demuster_geometry.h
    Desc: 
    Status: WIP
    ---------------------------------------------------------------------------- 
*/  

#pragma once

#include <vector>
#include <string>
#include <dubin.h>

std::vector<Pose> calculateFormation(std::vector<Point> points, double distance, double heading, double margin, double circle_radius, double arrow_angle, std::string type, Point CG = Point(0, 0), Point anchor = Point(0, 0));


double heading2Radians(double heading);
double radians2Heading(double radians);