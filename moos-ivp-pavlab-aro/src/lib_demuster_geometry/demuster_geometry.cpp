/*  ----------------------------------------------------------------------------
    Author: Filip Stromstad
    Circa: March 2024
    Origin: MIT, Cambridge MA
    File: demuster_geometry.cpp
    ---------------------------------------------------------------------------- 
*/  

#include "demuster_geometry.h"

using namespace std; 

// TODO: Function to calculate a specific point in a formation based on index + everything else

vector<Pose> calculateFormation(vector<Point> points, double distance, double heading, double margin, double circle_radius, double arrow_angle, string type, Point CG, Point anchor) {
    srand(time(NULL)); //Seed for random formation

    int N = points.size(); //Number of points

    if (CG.x != 0 || CG.y != 0) {
        //CG given, do nothing
    } else {
        //Calculate CG
        for (int i = 0; i < N; i++) {
            CG = CG + points[i];
        }
        CG = CG / N;
    }

    Point anchor_point;
    if (type == "return"){
        //Return to the pavlab dock
        anchor_point = Point(15, -10);
        heading = 335;
        type = "line";
    } else if (anchor.x != 0 || anchor.y != 0) {
        //Anchor given, calculate heading and set anchor point
        heading = radians2Heading(atan2(anchor.y - CG.y, anchor.x - CG.x)); //NB: overwrites heading
        anchor_point = anchor;
    } else {
        //Not given, calculate anchor point
        Point formation_direction_temp = Point(cos(heading2Radians(heading)), sin(heading2Radians(heading))); //Direction of the formation
        anchor_point = CG + formation_direction_temp * distance; //Anchor point of the formation 
    }

    Point formation_direction = Point(cos(heading2Radians(heading)), sin(heading2Radians(heading))); //Direction of the formation
    
    //---- Calculate formation ----
    vector<Pose> formation;
    if (type == "line"){
        Point line_direction = Point(formation_direction.y, -formation_direction.x); //Direction of the line

        if (N % 2 == 1) {
            //Odd number of points
            formation.push_back(Pose(anchor_point, heading));
            for (int i = 1; i <= (N-1)/2; i++) {
                Pose right_pose = Pose(anchor_point + line_direction * (i * margin), heading);
                Pose left_pose = Pose(anchor_point - line_direction * (i * margin), heading);
                formation.push_back(right_pose); //Right side
                formation.push_back(left_pose); //Left side
            }
        } else {
            //Even number of points
            for (int i = 1; i <= N/2; i++) {
                Pose right_pose = Pose(anchor_point + line_direction * ((i-0.5) * margin), heading);
                Pose left_pose = Pose(anchor_point - line_direction * ((i-0.5) * margin), heading);
                formation.push_back(right_pose); //Right side
                formation.push_back(left_pose); //Left side
            }
        }

    } else if (type == "circle"){
        Point center = anchor_point - formation_direction * circle_radius; //Center of the formation circle
        double delta_angle = margin/circle_radius; //Angle between points

        if (N % 2 == 1) {
            //Odd number of points
            formation.push_back(Pose(anchor_point, heading));
            for (int i = 1; i <= (N-1)/2; i++) {
                double angel_deg = delta_angle * i * 180 / M_PI;
                Pose right_pose = Pose(rotatePoint(anchor_point, center, delta_angle * i, "R"), heading + angel_deg);
                Pose left_pose = Pose(rotatePoint(anchor_point, center, delta_angle * i, "L"), heading - angel_deg);
                formation.push_back(right_pose); //Right side
                formation.push_back(left_pose); //Left side
            }
        } else {
            //Even number of points
            for (int i = 1; i <= N/2; i++) {
                double angel_deg = delta_angle * (i-0.5) * 180 / M_PI;
                Pose right_pose = Pose(rotatePoint(anchor_point, center, delta_angle * (i-0.5), "R"), heading + angel_deg);
                Pose left_pose = Pose(rotatePoint(anchor_point, center, delta_angle * (i-0.5), "L"), heading - angel_deg);
                formation.push_back(right_pose); //Right side
                formation.push_back(left_pose); //Left side
            }
        }

    } else if (type == "arrow"){
        double arrow_heading = heading;
        Point arrow_direction = Point(cos(arrow_heading), sin(arrow_heading)); //Direction of the arrow
        Point arrow_perpendicular = Point(arrow_direction.y, -arrow_direction.x); //Perpendicular direction of the arrow

        double arrow_angle_rad = M_PI/180*(arrow_angle/2); //Angle of the arrow (each side)
        double dir_factor = cos(arrow_angle_rad); //Factor for the direction
        double perp_factor = sin(arrow_angle_rad); //Factor for the perpendicular

        //Add the anchor point
        formation.push_back(Pose(anchor_point, heading));

        //Add the arrow legs
        for (int i = 2; i <= N; i++) {
            //Alernate between right and left side
            Point arrow_point;
            if (i % 2 == 1) { //Left side
                arrow_point = anchor_point - (arrow_direction*dir_factor + arrow_perpendicular*perp_factor) * ((i-1)/2 * margin);
            } else { //Right side
                arrow_point = anchor_point - (arrow_direction*dir_factor - arrow_perpendicular*perp_factor) * (i/2 * margin);
            }
            Pose arrow_pose = Pose(arrow_point, arrow_heading);
            formation.push_back(arrow_pose);
        }

    } else if (type == "convoy_line"){
        double convoy_heading = heading;
        for (int i = 0; i < N; i++) {
            Point convoy_point = anchor_point + formation_direction * (i * margin);
            Pose convoy_pose = Pose(convoy_point, convoy_heading);
            formation.push_back(convoy_pose);
        }
    } else if (type == "convoy" || type == "herringbone"){
        double convoy_heading = heading;
        Point convoy_point = anchor_point;
        Pose convoy_pose = Pose(convoy_point, convoy_heading);
        //Add N times the same convoy point
        for (int i = 0; i < N; i++) {
            formation.push_back(convoy_pose);
        }
    } else if (type == "random"){
        //Random formation

        double rand_formation_radius = 1; //Radius of the formation, will dynamically increase
        for (int i = 0; i < N; i++) {
            double random_x = anchor_point.x;
            double random_y = anchor_point.y;
            bool valid = false;
            int attempts = 0;
            while (!valid) {
                attempts++;
                double random_point_angle = ((rand() / (double)RAND_MAX) * 2 * M_PI);
                double random_point_radius = rand_formation_radius * sqrt(rand() / (double)RAND_MAX);

                random_x = anchor_point.x + random_point_radius * cos(random_point_angle);
                random_y = anchor_point.y + random_point_radius * sin(random_point_angle);

                valid = true;
                for (auto pose : formation) {
                    if (sqrt(pow(random_x - pose.p.x, 2) + pow(random_y - pose.p.y, 2)) < margin) {
                        valid = false;
                        break;
                    }
                }
                
                if (attempts > 15) {
                    rand_formation_radius += 1;
                    attempts = 0;
                }
            }

            // Valid point found - Generate random heading and add to formation
            double random_heading = rand() % 360;
            Pose random_pose = Pose(Point(random_x, random_y), random_heading);
            formation.push_back(random_pose);
        }


    } else {
        //Return empty formation
    }

    return formation;
}

double heading2Radians(double heading) {
    return (90 - heading) * M_PI / 180;
}

double radians2Heading(double radians) {
    return 90 - radians * 180 / M_PI;
}

