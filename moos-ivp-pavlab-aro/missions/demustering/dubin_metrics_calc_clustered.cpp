#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include "CPA_Utils.h"


double calculate_clustered_metric(const double x1, const double y1, const double x2, const double y2, const double s1, const double s2, const double h1, const double h2, const double dist_safe, double collision_thresh);
double clustered_metric_total(const std::vector<double>& x, const std::vector<double>& y, const std::vector<double>& s, const std::vector<double>& h, const double dist_safe, double collision_thresh);


int main(int argc, char* argv[]) {
    std::ifstream infile("vehicle_data_temp.txt");
    std::string line;

    std::vector<double> x, y, h;
    double dist_safe = 15.0, collision_thresh = 2.0; // Example values

    while (std::getline(infile, line)) {
        std::istringstream iss(line);
        double px, py, ps, ph;
        if (iss >> px >> py >> ph) {
            x.push_back(px);
            y.push_back(py);
            h.push_back(ph);
        }
    }
    
    std::vector<double> s(x.size(), 1.0); // Default speed

    double result = clustered_metric_total(x, y, s, h, dist_safe, collision_thresh);
    std::cout << result << std::endl;
    return 0;
}


double calculate_clustered_metric(const double x1, const double y1, const double x2, const double y2, const double s1, const double s2, const double h1, const double h2, const double dist_safe, double collision_thresh) {
    double distance = std::sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
    double distance_to_collision = std::max(distance - collision_thresh, 0.0);
    double closing_speed = closingSpeed(x1, y1, s1, h1, x2, y2, s2, h2);
    double closing_speed_max = s1 + s2;

    if (distance_to_collision == 0) { // Already in collision
        return 1.0;
    }

    if (closing_speed > 0 && distance_to_collision < dist_safe) { // Moving towards each other and within safe distance --> calculate metric (risk of collision)
        double distance_factor = 1 - (distance_to_collision/dist_safe);
        double closing_factor = std::min(closing_speed, closing_speed_max) / closing_speed_max;
        double metric = distance_factor * closing_factor;
        return metric;
    } 

    // Moving away from each other or not within safe distance
    return 0.0;
}

double clustered_metric_total(const std::vector<double>& x, const std::vector<double>& y, const std::vector<double>& s, const std::vector<double>& h, const double dist_safe, double collision_thresh) {
    double metric_total = 0.0;
    int num_vehicles = x.size();

    for (size_t veh_i = 0; veh_i < num_vehicles; ++veh_i) {
        double metric_i = 1.0;
        
        for (size_t veh_j = 0; veh_j < num_vehicles; ++veh_j) {
            if (veh_i == veh_j) {
                continue;
            }

            double M_ij = calculate_clustered_metric(x[veh_i], y[veh_i], x[veh_j], y[veh_j], s[veh_i], s[veh_j], h[veh_i], h[veh_j], dist_safe, collision_thresh);
            metric_i *= (1 - M_ij);
        }
        double P_ij = 1 - metric_i;
        metric_total += P_ij;
    }

    return metric_total / num_vehicles;
}