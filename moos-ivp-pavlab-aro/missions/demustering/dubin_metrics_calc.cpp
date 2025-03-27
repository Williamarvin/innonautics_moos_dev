#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <map>
#include <algorithm>
#include <numeric>
#include "CPA_Utils.h"
#include <cmath>

// Metrics functions
double calculate_average(const std::vector<double>& values);
double calculate_run_time(const std::vector<double>& time_series);
double calculate_odometry(const std::vector<double>& x, const std::vector<double>& y);
double calculate_idle_time(const double dt, const std::vector<double>& desired_speed);
void calculate_encounters(const std::vector<double>& x1, const std::vector<double>& y1, const std::vector<double>& x2, const std::vector<double>& y2, int& num_near_misses, int& num_collisions, double near_miss_thresh, double collision_thresh);
void calculate_potential_encounters(const std::vector<double>& x1, const std::vector<double>& y1, const std::vector<double>& x2, const std::vector<double>& y2, int& num_potential_collisions, double potential_collision_thresh);
double calculate_clustered_metric(const double x1, const double y1, const double x2, const double y2, const double s1, const double s2, const double h1, const double h2, const double dist_safe, double collision_thresh);
double closingSpeed_alt(double x1, double y1, double s1, double h1, double x2, double y2, double s2, double h2);

// JSON functions
void trim(std::string& s);
std::string extract_key(const std::string& line);
std::vector<double> parse_array(const std::string& line);
void load_from_json(const std::string& filepath, std::vector<std::vector<double>>& time_series_all, std::vector<std::map<std::string, std::map<std::string, std::vector<double>>>>& data_all, std::vector<std::string>& mission_names_all);

int main(int argc, char *argv[]) {
    std::string mission_name;
    std::string folder;
    bool multiple_configs = false;
    int potential_collision_thresh = 4;
    int near_miss_thresh = 4;
    int collision_thresh = 2;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--mission-name" && i + 1 < argc) {
            mission_name = argv[++i];
        } else if (arg == "--folder" && i + 1 < argc) {
            folder = argv[++i];
        }
    }

    if (mission_name.find("batch") != std::string::npos) {
        multiple_configs = true;
    }

    std::vector<std::string> mission_names_all;
    std::vector<std::vector<double>> time_series_all;
    std::vector<std::map<std::string, std::map<std::string, std::vector<double>>>> data_all;
    std::string filepath = folder + "/" + mission_name+".json";
    load_from_json(filepath, time_series_all, data_all, mission_names_all);
    int numRuns = time_series_all.size();

    /*****************************************************************/
    /* x. Calculate the metrics                                      */
    /*****************************************************************/

    std::map<std::string, std::vector<double>> metrics;
    std::map<std::string, std::vector<std::vector<double>>> dynamic_metrics;
    for (int run = 0; run < data_all.size(); ++run) {
        std::cout << "Run: " << (run + 1) << std::endl;
        std::cout << "Mission name: " << mission_names_all[run] << std::endl;
        // if (run == 35){
        //     continue; //TEMP DEBUG
        // }
        // 1. Load the run data
        std::map<std::string, double> run_metrics;
        std::map<std::string, std::vector<double>> run_dynamic_metrics;
        const std::vector<double>& time_series = time_series_all[run];
        const double dt = time_series[1] - time_series[0];
        const std::map<std::string, std::map<std::string, std::vector<double>>>& data = data_all[run];
        int num_vehicles = data.size();
        int T = time_series.size();
        
        // 2. Calculate non-vehicle specific metrics
        run_metrics["time_run_total"] = calculate_run_time(time_series);
        run_dynamic_metrics["time_series"] = time_series;

        // 3. Calculate vehicle specific metrics
        double odometry = 0.0;
        double sum_avg_speed = 0.0;
        double sum_idle_time = 0.0;
        double paths_calculated = 0.0;
        double deadlocked_time = 0.0;
        for (const auto& node : data) {
            // 3.1 Load the node data
            // std::cout << "Node: " << node.first << std::endl;
            const std::map<std::string, std::vector<double>>& node_data = node.second;
            const std::vector<double>& nav_x = node_data.at("NAV_X");
            const std::vector<double>& nav_y = node_data.at("NAV_Y");
            const std::vector<double>& nav_speed = node_data.at("NAV_SPEED");
            const std::vector<double>& desired_speed = node_data.at("DESIRED_SPEED");
            const std::vector<double>& nav_heading = node_data.at("NAV_HEADING");
            // std::cout << "Node: " << node.first << std::endl;
            

            // 3.2 Calculate the metrics for this vehicle
            odometry += calculate_odometry(nav_x, nav_y);
            sum_avg_speed += calculate_average(nav_speed);
            sum_idle_time += calculate_idle_time(dt, desired_speed);
            // paths_calculated += node_data.at("DUBIN_UPDATE_PATH_REGEN").size();
            if (node_data.find("DUBIN_UPDATE_PATH_REGEN") != node_data.end()) {
                paths_calculated += node_data.at("DUBIN_UPDATE_PATH_REGEN").size();
            }

            if (node_data.find("DEADLOCKED") != node_data.end()) {
                double deadlocked_time_node = 0.0;
                std::vector<double> deadlocked = node_data.at("DEADLOCKED");

                // Calculate time in deadlock:
                if (!deadlocked.empty()) {
                    double start_time = deadlocked[0];  // Start of the first deadlock period
                    double prev_time = deadlocked[0];   // Track the previous time

                    for (size_t i = 1; i < deadlocked.size(); ++i) {
                        if (deadlocked[i] - prev_time > 1.0) {
                            // New deadlock period detected, add duration of previous period
                            deadlocked_time_node += prev_time - start_time;
                            start_time = deadlocked[i];  // Reset start of new deadlock period
                        }
                        prev_time = deadlocked[i];  // Update previous timestamp
                    }

                    // Add the last deadlock period
                    deadlocked_time_node += prev_time - start_time;
                }
                deadlocked_time += deadlocked_time_node;
            }
        }
        run_metrics["odometry_avg"] = odometry / num_vehicles;
        run_metrics["speed_avg"] = sum_avg_speed / num_vehicles;
        run_metrics["time_idle_avg"] = sum_idle_time / num_vehicles;
        run_metrics["paths_calculated_avg"] = paths_calculated / num_vehicles;
        run_metrics["deadlocked_time"] = deadlocked_time / num_vehicles;

        // 4. Calculate vehicle to vehicle metrics
        int num_pot_encounters = 0;
        int num_near_misses = 0;
        int num_collisions = 0;
        // std::vector<double> clustered_metric_dynamic = std::vector<double>(T, 0.0);
        // std::vector<double> clustered_metric_dynamic_alt = std::vector<double>(T, 0.0);
        // std::vector<double> risk_of_collision_dynamic = std::vector<double>(T, 1.0);
        std::vector<double> risk_of_collision_dynamic_avg = std::vector<double>(T, 0.0);
        const double distance_safe = 15.0;

        // Double for loop iterating over EACH UNIQUE vehicle to vehicle pair:
        for (auto it1 = data.begin(); it1 != data.end(); ++it1) {
            for (auto it2 = std::next(it1); it2 != data.end(); ++it2) {
                const std::vector<double>& nav_x1 = it1->second.at("NAV_X");
                const std::vector<double>& nav_y1 = it1->second.at("NAV_Y");
                const std::vector<double>& nav_x2 = it2->second.at("NAV_X");
                const std::vector<double>& nav_y2 = it2->second.at("NAV_Y");
                const std::vector<double>& nav_h1 = it1->second.at("NAV_HEADING");
                const std::vector<double>& nav_h2 = it2->second.at("NAV_HEADING");

                const double default_speed = 1.0;

                calculate_potential_encounters(nav_x1, nav_y1, nav_x2, nav_y2, num_pot_encounters, potential_collision_thresh);
                calculate_encounters(nav_x1, nav_y1, nav_x2, nav_y2, num_near_misses, num_collisions, near_miss_thresh, collision_thresh);
                // std::cout << "Vehicle 1: " << it1->first << " Vehicle 2: " << it2->first << std::endl;

                
                // // Calculate the dynamic metric
                // for (int t = 0; t < T; ++t) {
                //     clustered_metric_dynamic[t] += calculate_clustered_metric(nav_x1[t], nav_y1[t], nav_x2[t], nav_y2[t], default_speed, default_speed, nav_h1[t], nav_h2[t], distance_safe, collision_thresh);
                //     clustered_metric_dynamic_alt[t] += calculate_clustered_metric(nav_x1[t], nav_y1[t], nav_x2[t], nav_y2[t], default_speed, default_speed, nav_h1[t], nav_h2[t], 12.0, collision_thresh);
                //     risk_of_collision_dynamic[t] *= (1 - calculate_clustered_metric(nav_x1[t], nav_y1[t], nav_x2[t], nav_y2[t], default_speed, default_speed, nav_h1[t], nav_h2[t], distance_safe, collision_thresh));
                // }
            }
        }

        // Double for loop iterating over ALL vehicle to vehicle pairs:
        for (auto it1 = data.begin(); it1 != data.end(); ++it1) {
            std::vector<double> risk_of_collision_i = std::vector<double>(T, 1.0);

            for (auto it2 = data.begin(); it2 != data.end(); ++it2) {
                if (it1 == it2) { // Skip same vehicle
                    continue; 
                }

                const std::vector<double>& nav_x1 = it1->second.at("NAV_X");
                const std::vector<double>& nav_y1 = it1->second.at("NAV_Y");
                const std::vector<double>& nav_x2 = it2->second.at("NAV_X");
                const std::vector<double>& nav_y2 = it2->second.at("NAV_Y");
                const std::vector<double>& nav_h1 = it1->second.at("NAV_HEADING");
                const std::vector<double>& nav_h2 = it2->second.at("NAV_HEADING");

                const double default_speed = 1.0;
                
                // Calculate the dynamic metric
                for (int t = 0; t < T; ++t) {
                    risk_of_collision_i[t] *= (1 - calculate_clustered_metric(nav_x1[t], nav_y1[t], nav_x2[t], nav_y2[t], default_speed, default_speed, nav_h1[t], nav_h2[t], distance_safe, collision_thresh));
                }
            }

            for (int t = 0; t < T; ++t) {
                risk_of_collision_dynamic_avg[t] += (1 - risk_of_collision_i[t]);
            }
        }

        for (int t = 0; t < T; ++t) {
            // clustered_metric_dynamic[t] = clustered_metric_dynamic[t] / (num_vehicles * (num_vehicles - 1) / 2);
            // clustered_metric_dynamic_alt[t] = clustered_metric_dynamic_alt[t] / (num_vehicles * (num_vehicles - 1) / 2);
            // risk_of_collision_dynamic[t] = 1 - risk_of_collision_dynamic[t];
            risk_of_collision_dynamic_avg[t] = risk_of_collision_dynamic_avg[t] / num_vehicles;
        }

        run_metrics["num_pot_encounters"] = num_pot_encounters;
        run_metrics["num_near_misses"] = num_near_misses;
        run_metrics["num_collisions"] = num_collisions;
        run_metrics["clustered"] = risk_of_collision_dynamic_avg[0];

        // run_dynamic_metrics["clustered_metric"] = clustered_metric_dynamic;
        // run_dynamic_metrics["clustered_metric_alt"] = clustered_metric_dynamic_alt;
        // run_dynamic_metrics["risk_of_collision"] = risk_of_collision_dynamic;
        run_dynamic_metrics["risk_of_collision_avg"] = risk_of_collision_dynamic_avg;
        run_dynamic_metrics["deadlocked_time"] = std::vector<double>(1, deadlocked_time / num_vehicles);
        run_dynamic_metrics["num_collisions"] = std::vector<double>(1, num_collisions);
        run_dynamic_metrics["num_near_misses"] = std::vector<double>(1, num_near_misses);
        run_dynamic_metrics["num_pot_encounters"] = std::vector<double>(1, num_pot_encounters);



        // 5. Add the metrics to the metrics map
        for (const auto& metric : run_metrics) {
            metrics[metric.first].push_back(metric.second);
        }

        for (const auto& metric : run_dynamic_metrics) {
            dynamic_metrics[metric.first].push_back(metric.second);
        }
    }

    /*****************************************************************/
    /* x. Calculate the avg metrics per config                       */
    /*****************************************************************/

    std::map<std::string, std::map<std::string, double>> config_metrics;
    std::map<std::string, std::map<std::string, std::vector<std::vector<double>>>> config_metrics_dynamic;

    if (multiple_configs){
        for (int i = 0; i < numRuns; ++i) {
            std::string mission_name = mission_names_all[i];
            std::string config_name = mission_name.substr(0, mission_name.find("_", mission_name.find("_") + 1));
            // std::cout << "Mission name: " << mission_name << " Config name: " << config_name << std::endl;

            // 1. Static metrics
            if (config_metrics.find(config_name) == config_metrics.end()) {
                config_metrics[config_name] = std::map<std::string, double>();
                config_metrics[config_name]["trials"] = 0;
                for (const auto& metric : metrics) {
                    std::string metric_name = metric.first;
                    config_metrics[config_name][metric_name] = 0.0;
                }
            }

            config_metrics[config_name]["trials"] += 1;
            for (const auto& metric : metrics) {
                std::string metric_name = metric.first;
                config_metrics[config_name][metric_name] += metric.second[i];
            }

            // 2. Dynamic metrics
            if(config_metrics_dynamic.find(config_name) == config_metrics_dynamic.end()){
                config_metrics_dynamic[config_name] = std::map<std::string, std::vector<std::vector<double>>>();
                for (const auto& metric : dynamic_metrics) {
                    std::string metric_name = metric.first;
                    config_metrics_dynamic[config_name][metric_name] = std::vector<std::vector<double>>();
                }
            }

            // std::map<std::string, std::vector<std::vector<double>>> dynamic_metrics;
            for (const auto& metric : dynamic_metrics) {
                std::string metric_name = metric.first;
                std::vector<std::vector<double>> metric_values = metric.second;
                std::vector<double> metric_values_run = metric_values[i];
                config_metrics_dynamic[config_name][metric_name].push_back(metric_values_run);
            }
            
        }

        for (auto& config : config_metrics) {
            int trials = config.second["trials"];
            for (auto& metric : config.second) {
                if (metric.first != "trials") {
                    metric.second /= trials;
                }
            }
        }

        /*****************************************************************/
        /* x. Write the config metrics to a JSON file                    */
        /*****************************************************************/    

        std::string metrics_filepath = folder + "/" + mission_name + "_metrics.json";
        std::ofstream metrics_file(metrics_filepath);
        std::map<std::string, std::map<std::string, std::vector<double>>> config_metrics_output;

        // First, build the required data structure
        for (const auto& config_metric : config_metrics) {
            std::string config = config_metric.first;
            std::map<std::string, double> metrics = config_metric.second;

            std::string config_var = config.substr(0, config.find("_"));
            std::string config_val = config.substr(config.find("_") + 1);

            try{
                double value = std::stod(config_val);
                // If fine, do nothing
            }
            catch (const std::invalid_argument&){ // Not a range, use full config
                config_var = config;
                config_val = "0";
            }

            // Initialize if not already present
            if (config_metrics_output.find(config_var) == config_metrics_output.end()) {
                config_metrics_output[config_var]["config_values"] = std::vector<double>();
            }

            // Add the config_val to "config_values"
            config_metrics_output[config_var]["config_values"].push_back(std::stod(config_val));
            // config_metrics_output[config_var]["config_values"].push_back(bool(config_val == "true"));

            // Add each metric to its respective vector
            for (const auto& metric : metrics) {
                std::string metric_name = metric.first;
                double metric_value = metric.second;

                config_metrics_output[config_var][metric_name].push_back(metric_value);
            }
        }

        // Now, write the JSON output
        metrics_file << "{\n";

        for (auto config_it = config_metrics_output.begin(); config_it != config_metrics_output.end(); ++config_it) {
            std::string config_var = config_it->first;
            metrics_file << "  \"" << config_var << "\": {\n";

            const auto& metrics_map = config_it->second;
            
            // Write config_values array
            metrics_file << "    \"config_values\": [";
            for (size_t i = 0; i < metrics_map.at("config_values").size(); ++i) {
                metrics_file << metrics_map.at("config_values")[i];
                if (i < metrics_map.at("config_values").size() - 1) {
                    metrics_file << ", ";
                }
            }
            metrics_file << "],\n";

            // Write other metrics
            for (auto metric_it = metrics_map.begin(); metric_it != metrics_map.end(); ++metric_it) {
                if (metric_it->first == "config_values") {
                    continue;  // Skip config_values since we've already printed them
                }

                metrics_file << "    \"" << metric_it->first << "\": [";
                for (size_t i = 0; i < metric_it->second.size(); ++i) {
                    metrics_file << metric_it->second[i];
                    if (i < metric_it->second.size() - 1) {
                        metrics_file << ", ";
                    }
                }
                metrics_file << "]";
                if (std::next(metric_it) != metrics_map.end()) {
                    metrics_file << ",";
                }
                metrics_file << "\n";
            }

            metrics_file << "  }";
            if (std::next(config_it) != config_metrics_output.end()) {
                metrics_file << ",";
            }
            metrics_file << "\n";
        }

        metrics_file << "}\n";
    } else {
        std::string metrics_filepath = folder + "/" + mission_name + "_metrics.json";
        std::ofstream metrics_file(metrics_filepath);

        // Write the JSON output
        metrics_file << "{\n";
        metrics_file << "\"metrics\": {\n";

        for (auto metric_it = metrics.begin(); metric_it != metrics.end(); ++metric_it) {
            std::string metric_name = metric_it->first;
            std::vector<double> metric_values = metric_it->second;

            metrics_file << "  \"" << metric_name << "\": [";
            for (size_t i = 0; i < metric_values.size(); ++i) {
                metrics_file << metric_values[i];
                if (i < metric_values.size() - 1) {
                    metrics_file << ", ";
                }
            }
            metrics_file << "]";
            if (std::next(metric_it) != metrics.end()) {
                metrics_file << ",";
            }
            metrics_file << "\n";
        }
        metrics_file << "  }\n";
        metrics_file << "}\n";
    }



    /*****************************************************************/
    /* x. Print the metrics                                      */
    /*****************************************************************/

    size_t maxMetricNameWidth = 0;
    for (const auto& pair : metrics) {
        maxMetricNameWidth = std::max(maxMetricNameWidth, pair.first.size());
    }

    // Print header: print space for the "Run" labels
    std::cout << std::setw(maxMetricNameWidth) << "Run";
    for (const auto& pair : metrics) {
        std::cout << "  " << std::setw(maxMetricNameWidth) << pair.first;
    }
    std::cout << std::endl;

    // Print each run and its metric values
    for (size_t i = 0; i < numRuns; ++i) {
        if (multiple_configs){
            std::cout << std::setw(maxMetricNameWidth) << mission_names_all[i];
        } else {
            std::cout << std::setw(maxMetricNameWidth) << ("Run " + std::to_string(i + 1));
        }
        // std::cout << std::setw(maxMetricNameWidth) << mission_names_all[i];
        for (const auto& pair : metrics) {
            std::cout << "  " << std::setw(maxMetricNameWidth) << round(pair.second[i] * 100) / 100;
        }
        std::cout << std::endl;
    }

    // Print header: print space for the "Run" labels
    std::cout << std::setw(maxMetricNameWidth) << "Run";
    for (const auto& pair : metrics) {
        std::cout << "  " << std::setw(maxMetricNameWidth) << pair.first;
    }
    std::cout << std::endl;

    // Print delimiter line
    std::cout << std::setw(maxMetricNameWidth) << " " << std::setfill('-');
    for (size_t i = 0; i < metrics.size(); ++i) {
        std::cout << "  " << std::setw(maxMetricNameWidth) << "--------";
    }
    std::cout << std::endl;
    std::cout << std::setfill(' ');  // Reset fill character

    // Print averages, max, min, and standard deviation
    std::cout << std::setw(maxMetricNameWidth) << "Average";
    for (const auto& pair : metrics) {
        double sum = 0.0;
        double min_val = pair.second[0];
        double max_val = pair.second[0];
        double sum_squared_diff = 0.0;

        for (size_t i = 0; i < numRuns; ++i) {
            double value = pair.second[i];
            sum += value;
            if (value < min_val) min_val = value;
            if (value > max_val) max_val = value;
        }
        double average = sum / numRuns;

        for (size_t i = 0; i < numRuns; ++i) {
            double diff = pair.second[i] - average;
            sum_squared_diff += diff * diff;
        }
        double stddev = std::sqrt(sum_squared_diff / numRuns);

        std::cout << "  " << std::setw(maxMetricNameWidth) << round(average * 100) / 100;
    }
    std::cout << std::endl;

    std::cout << std::setw(maxMetricNameWidth) << "Max";
    for (const auto& pair : metrics) {
        double max_val = pair.second[0];
        for (size_t i = 0; i < numRuns; ++i) {
            if (pair.second[i] > max_val) max_val = pair.second[i];
        }
        std::cout << "  " << std::setw(maxMetricNameWidth) << round(max_val * 100) / 100;
    }
    std::cout << std::endl;

    std::cout << std::setw(maxMetricNameWidth) << "Min";
    for (const auto& pair : metrics) {
        double min_val = pair.second[0];
        for (size_t i = 0; i < numRuns; ++i) {
            if (pair.second[i] < min_val) min_val = pair.second[i];
        }
        std::cout << "  " << std::setw(maxMetricNameWidth) << round(min_val * 100) / 100;
    }
    std::cout << std::endl;

    std::cout << std::setw(maxMetricNameWidth) << "Std Dev";
    for (const auto& pair : metrics) {
        double sum = 0.0;
        double sum_squared_diff = 0.0;
        for (size_t i = 0; i < numRuns; ++i) {
            sum += pair.second[i];
        }
        double average = sum / numRuns;

        for (size_t i = 0; i < numRuns; ++i) {
            double diff = pair.second[i] - average;
            sum_squared_diff += diff * diff;
        }
        double stddev = std::sqrt(sum_squared_diff / numRuns);

        std::cout << "  " << std::setw(maxMetricNameWidth) << round(stddev * 100) / 100;
    }
    std::cout << std::endl;


    if (multiple_configs){
        // Print the metrics per config
        std::cout << std::endl;
        std::cout << std::setw(maxMetricNameWidth) << "Config";
        for (const auto& pair : metrics) {
            std::cout << "  " << std::setw(maxMetricNameWidth) << pair.first;
        }
        std::cout << std::endl;

        for (const auto& pair : config_metrics) {
            std::cout << std::setw(maxMetricNameWidth) << pair.first;
            for (const auto& metric : pair.second) {
                // if (metric.first != "trials") {
                //     std::cout << "  " << std::setw(maxMetricNameWidth) << round(metric.second * 100) / 100;
                // }
                std::cout << "  " << std::setw(maxMetricNameWidth) << round(metric.second * 100) / 100;
            }
            std::cout << std::endl;
        }
    }


    /*****************************************************************/
    /* x. Write the metrics to a JSON file                           */
    /*****************************************************************/

    // Only do dynamic metrics for now
    std::string metrics_filepath = folder + "/" + mission_name + "_metrics_dynamic.json";
    std::ofstream metrics_file(metrics_filepath);
    
    // Write the JSON output
    metrics_file << "{\n";
    metrics_file << "  \"dynamic_metrics\": {\n";
    
    if(multiple_configs){
        for (auto config_it = config_metrics_dynamic.begin(); config_it != config_metrics_dynamic.end(); ++config_it) {
            std::string config_var = config_it->first;
            metrics_file << "    \"" << config_var << "\": {\n";

            const auto& config_metrics = config_it->second;
            for (auto metric_it = config_metrics.begin(); metric_it != config_metrics.end(); ++metric_it) {
                std::string metric_name = metric_it->first;
                metrics_file << "      \"" << metric_name << "\": [\n";

                const auto& metric_values = metric_it->second;
                for (size_t i = 0; i < metric_values.size(); ++i) {
                    metrics_file << "        [";
                    for (size_t t = 0; t < metric_values[i].size(); ++t) {
                        metrics_file << std::fixed << std::setprecision(6) << metric_values[i][t];
                        if (t < metric_values[i].size() - 1) {
                            metrics_file << ", ";
                        }
                    }
                    metrics_file << "]";
                    if (i < metric_values.size() - 1) {
                        metrics_file << ",";
                    }
                    metrics_file << "\n";
                }

                metrics_file << "      ]";
                if (std::next(metric_it) != config_metrics.end()) {
                    metrics_file << ",";
                }
                metrics_file << "\n";
            }

            metrics_file << "    }";
            if (std::next(config_it) != config_metrics_dynamic.end()) {
                metrics_file << ",";
            }
            metrics_file << "\n";
        }

        metrics_file << "  }\n";
        metrics_file << "}\n";   
    } else {
        std::string config_var = "metric_avg";
        metrics_file << "    \"" << config_var << "\": {\n";
        for (auto metric_it = dynamic_metrics.begin(); metric_it != dynamic_metrics.end(); ++metric_it) {
            std::string metric_name = metric_it->first;
            metrics_file << "      \"" << metric_name << "\": [\n";

            const auto& metric_values = metric_it->second;
            for (size_t i = 0; i < metric_values.size(); ++i) {
                metrics_file << "        [";
                for (size_t t = 0; t < metric_values[i].size(); ++t) {
                    metrics_file << std::fixed << std::setprecision(6) << metric_values[i][t];
                    if (t < metric_values[i].size() - 1) {
                        metrics_file << ", ";
                    }
                }
                metrics_file << "]";
                if (i < metric_values.size() - 1) {
                    metrics_file << ",";
                }
                metrics_file << "\n";
            }

            metrics_file << "      ]";
            if (std::next(metric_it) != dynamic_metrics.end()) {
                metrics_file << ",";
            }
            metrics_file << "\n";
        }

        metrics_file << "    }\n";
        metrics_file << "  }\n";    
        metrics_file << "}\n";
    }





    return 0;
}


/*****************************************************************/
/* x. Metric functions                                           */
/*****************************************************************/

double calculate_average(const std::vector<double>& values) {
    if (values.empty()) {
        return 0.0;
    }

    double const count = static_cast<double>(values.size());
    return std::accumulate(values.begin(), values.end(), 0.0) / count;
}

double calculate_run_time(const std::vector<double>& time_series) {
    return time_series.back() - time_series.front();
}

double calculate_odometry(const std::vector<double>& x, const std::vector<double>& y) {
    double distance = 0.0;
    for (int i = 1; i < x.size(); i++) {
        double dx = x[i] - x[i - 1];
        double dy = y[i] - y[i - 1];
        distance += std::sqrt(dx * dx + dy * dy);
    }
    return distance;
}

double calculate_idle_time(const double dt, const std::vector<double>& speed){
    double idle_time = 0.0;
    for (int i = 0; i < speed.size(); i++) {
        if (speed[i] == 0.0) {
            idle_time += dt;
        }
    }
    return idle_time;
}


// void calculate_encounters(const std::vector<double>& x1, const std::vector<double>& y1, 
//                           const std::vector<double>& x2, const std::vector<double>& y2, 
//                           int& num_near_misses, int& num_collisions, 
//                           double near_miss_thresh, double collision_thresh) {
    
//     if (x1.size() != y1.size() || x2.size() != y2.size()) {
//         throw std::invalid_argument("Coordinate vectors for each vehicle must be of the same length");
//     }

//     if (x1.size() != x2.size()) {
//         throw std::invalid_argument("Coordinate vectors for both vehicles must be of the same length");
//     }

//     double initial_delta_x = x1[0] - x2[0];
//     double initial_delta_y = y1[0] - y2[0];
//     double previous_distance = std::sqrt(initial_delta_x * initial_delta_x + initial_delta_y * initial_delta_y);
//     // double cpa = -1;
//     double cpa = previous_distance;

//     for (size_t i = 1; i < x1.size(); ++i) {
//         double delta_x = x1[i] - x2[i];
//         double delta_y = y1[i] - y2[i];
//         double distance = std::sqrt(delta_x * delta_x + delta_y * delta_y);

//         double delta_distance = distance - previous_distance;
//         double eps = 0.5;

//         if (delta_distance < -eps) { //approaching
//             cpa = distance;
//         } else if (delta_distance > eps) { //moving away
//             if (cpa < 0){ //encounter already counted
//                 continue;
//                 std::cout << "Collision already counted: " << i << std::endl;
//                 std::cout << "Distance: " << distance << std::endl;
//             }
//             if (cpa <= collision_thresh) {
//                 num_collisions++;
//                 std::cout << "Collision at time: " << i << " -  Distance: " << distance << " - delta_distance: " << delta_distance << std::endl;
//                 // std::cout << "Distance: " << distance << std::endl;
//             } else if (cpa <= near_miss_thresh) {
//                 num_near_misses++;
//             }
//             cpa = -1;
//         } else {
//             //no change in distance
//         }

//         if (abs(delta_distance) >= eps) {
//             previous_distance = distance;
//         }
//     }
//     return;
// }


void calculate_encounters(const std::vector<double>& x1, const std::vector<double>& y1, 
                          const std::vector<double>& x2, const std::vector<double>& y2, 
                          int& num_near_misses, int& num_collisions, 
                          double near_miss_thresh, double collision_thresh) {
    
    if (x1.size() != y1.size() || x2.size() != y2.size()) {
        throw std::invalid_argument("Coordinate vectors for each vehicle must be of the same length");
    }

    if (x1.size() != x2.size()) {
        throw std::invalid_argument("Coordinate vectors for both vehicles must be of the same length");
    }

    
    bool in_near_miss = false;
    bool in_collision = false;

    double safe_distance = 5.0;
    for (size_t i = 1; i < x1.size(); ++i) {
        double delta_x = x1[i] - x2[i];
        double delta_y = y1[i] - y2[i];
        double distance = std::sqrt(delta_x * delta_x + delta_y * delta_y);

        if (distance <= collision_thresh) {
            in_collision = true;
        } else if (distance <= near_miss_thresh) {
            in_near_miss = true;
        } else if (distance > safe_distance){
            if (in_collision) {
                num_collisions++;
                in_collision = false;
                in_near_miss = false;
            } else if (in_near_miss) {
                num_near_misses++;
                in_near_miss = false;
            }
        }
    }
    return;
}



void calculate_potential_encounters(const std::vector<double>& x1, const std::vector<double>& y1, 
                                    const std::vector<double>& x2, const std::vector<double>& y2, 
                                    int& num_potential_collisions, double potential_collision_thresh) {
    
    if (x1.size() != y1.size() || x2.size() != y2.size()) {
        throw std::invalid_argument("Coordinate vectors for each vehicle must be of the same length");
    }

    bool in_potential_collision = false;
    // bool in_potential_collision = true;
    for (size_t i = 0; i < x1.size(); ++i) {
        double min_distance = std::numeric_limits<double>::max();
        for (size_t j = 0; j < x2.size(); ++j) {
            double delta_x = x1[i] - x2[j];
            double delta_y = y1[i] - y2[j];
            double distance = std::sqrt(delta_x * delta_x + delta_y * delta_y);

            if (distance < min_distance) {
                min_distance = distance;
            }
        }


        if (min_distance <= potential_collision_thresh) {
            if (!in_potential_collision) { //Only count once per potential collision
                num_potential_collisions++;
                in_potential_collision = true;
            }
        } else { // Not within potential collision range
            in_potential_collision = false;
        }
    }
    return;
}


double calculate_clustered_metric(const double x1, const double y1, const double x2, const double y2, const double s1, const double s2, const double h1, const double h2, const double dist_safe, double collision_thresh) {
    double distance = std::sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
    double distance_to_collision = std::max(distance - collision_thresh, 0.0);
    double closing_speed = closingSpeed(x1, y1, s1, h1, x2, y2, s2, h2);
    double closing_speed_alt = closingSpeed_alt(x1, y1, s1, h1, x2, y2, s2, h2);
    // double closing_speed_alt = closingSpeed_alt(0, 0, 1, 0, 0, 10, 1, 180);
    double diff = closing_speed - closing_speed_alt;
    // std::cout << "Closing speed: " << closing_speed << " - Closing speed alt: " << closing_speed_alt << " - Diff: " << diff << std::endl;
    // std::cout << diff << std::endl;

    double eps = 0.0000001;
    if (diff > eps) {
        std::cout << "Closing speed: " << closing_speed << " - Closing speed alt: " << closing_speed_alt << " - Diff: " << diff << std::endl;
    }

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


/*****************************************************************/
/* x. JSON functions                                             */
/*****************************************************************/

void trim(std::string& s) {
    s.erase(0, s.find_first_not_of(" \t\n\r\f\v"));
    s.erase(s.find_last_not_of(" \t\n\r\f\v") + 1);
}

std::string extract_key(const std::string& line) {
    size_t start = line.find("\"") + 1;
    size_t end = line.find("\"", start);
    return line.substr(start, end - start);
}

std::vector<double> parse_array(const std::string& line) {
    std::vector<double> values;
    std::string trimmed = line;
    trim(trimmed);
    trimmed = trimmed.substr(trimmed.find('[') + 1);  // Remove the starting '['
    trimmed.pop_back();  // Remove the ending ']'

    std::stringstream ss(trimmed);
    std::string item;
    while (std::getline(ss, item, ',')) {
        values.push_back(std::stod(item));
    }
    return values;
}

void load_from_json(const std::string& filepath, std::vector<std::vector<double>>& time_series_all, std::vector<std::map<std::string, std::map<std::string, std::vector<double>>>>& data_all, std::vector<std::string>& mission_names_all) {
    std::ifstream file(filepath);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open file " + filepath);
    }

    std::string line;
    bool in_mission_names = false;
    bool in_time_series = false;
    bool in_data_all = false;
    std::map<std::string, std::vector<double>> current_variable_map;
    std::map<std::string, std::map<std::string, std::vector<double>>> current_node_map;

    while (std::getline(file, line)) {
        trim(line);

        if (line == "\"mission_names\": [") {
            in_mission_names = true;
            continue;
        }

        if (line == "\"time_series\": [") {
            in_time_series = true;
            continue;
        }

        if (line == "\"data_all\": [") {
            in_time_series = false;
            in_data_all = true;
            continue;
        }

        if (in_mission_names) {
            if (line == "],") {
                in_mission_names = false;
            } else {
                // Only one line on the format: "mission_name1", "mission_name2", ... , "mission_nameN"
                std::stringstream ss(line);
                std::string mission_name;
                while (std::getline(ss, mission_name, ',')) {
                    // Remove first part of mission name, up to and including the first "_"
                    mission_name = mission_name.substr(mission_name.find("_") + 1);
                    mission_names_all.push_back(mission_name);
                }
            }
        }

        if (in_time_series) {
            if (line == "],") {
                in_time_series = false;
            } else if (line.front() == '[') {
                time_series_all.push_back(parse_array(line));
            }
        }

        if (in_data_all) {
            if (line.find("},{") != std::string::npos) {
                data_all.push_back(current_node_map);
                current_node_map.clear();
            } else if (line.find(':') != std::string::npos) {
                std::string key = extract_key(line);

                // Check if this line is the start of a node (i.e., it doesn't contain a '[')
                if (line.find('{') != std::string::npos) {
                    std::string node_name = key;
                    while (std::getline(file, line)) {
                        trim(line);
                        if (line.find('}') != std::string::npos) {
                            break;
                        }

                        std::string variable_name = extract_key(line);
                        std::vector<double> values = parse_array(line);
                        current_variable_map[variable_name] = values;
                    }
                    current_node_map[key] = current_variable_map;
                    current_variable_map.clear();
                }
            }
        }
    }

    if (!current_node_map.empty()) { //Add the last run
        data_all.push_back(current_node_map);
    }

    file.close();
    return;
}





double closingSpeed_alt(double x1, double y1, double s1, double h1, double x2, double y2, double s2, double h2){
    double h1_rad = (90 - h1) * M_PI / 180;
    double h2_rad = (90 - h2) * M_PI / 180;

    double dx = x2 - x1;
    double dy = y2 - y1;
    double theta = std::atan2(dy, dx);

    double v1x = s1 * std::cos(h1_rad);
    double v1y = s1 * std::sin(h1_rad);
    double v2x = s2 * std::cos(h2_rad);
    double v2y = s2 * std::sin(h2_rad);

    double v_12_x = v1x - v2x;
    double v_12_y = v1y - v2y;

    double closing_speed = v_12_x * std::cos(theta) + v_12_y * std::sin(theta);

    return closing_speed;
}