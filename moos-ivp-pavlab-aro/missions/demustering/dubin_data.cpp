#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <map>
#include <algorithm>
#include <typeinfo>


double findClosest(const std::vector<std::pair<double, double>>& irregular_series, double regular_time);
void save_to_json(const std::string& filepath, std::vector<std::vector<double>>& time_series_all, std::vector<std::map<std::string, std::map<std::string, std::vector<double>>>>& data_all, std::vector<std::string>& mission_names_all);

int main(int argc, char *argv[]) {
    std::vector<double> start_times;
    std::vector<double> end_times;
    std::vector<std::string> names;
    std::vector<std::string> mission_names;
    std::vector<std::string> periodic_variables;
    std::vector<std::string> non_periodic_variables;
    std::string json_name;
    std::string folder;
    double shore_start_time = 0.0;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--start-times") {
            while (i + 1 < argc && argv[i + 1][0] != '-') {
                start_times.push_back(std::stod(argv[++i]));
            }
        } else if (arg == "--end-times") {
            while (i + 1 < argc && argv[i + 1][0] != '-') {
                end_times.push_back(std::stod(argv[++i]));
            }
        } else if (arg == "--names") {
            while (i + 1 < argc && argv[i + 1][0] != '-') {
                names.push_back(argv[++i]);
            }
        } else if (arg == "--mission_names") {
            while (i + 1 < argc && argv[i + 1][0] != '-') {
                mission_names.push_back(argv[++i]);
            }
        } else if (arg == "--periodic-variables") {
            while (i + 1 < argc && argv[i + 1][0] != '-') {
                periodic_variables.push_back(argv[++i]);
            }
        } else if (arg == "--non-periodic-variables") {
            while (i + 1 < argc && argv[i + 1][0] != '-') {
                non_periodic_variables.push_back(argv[++i]);
            }
        } else if (arg == "--json-name" && i + 1 < argc) {
            json_name = argv[++i];
        } else if (arg == "--folder" && i + 1 < argc) {
            folder = argv[++i];
        } else if (arg == "--shore-start-time" && i + 1 < argc) {
            shore_start_time = std::stod(argv[++i]);
        }
        
    }

    double delta_t = 0.25;


/*****************************************************************/
/* 1. Synchronize and store all the data                         */
/*****************************************************************/

    std::vector<std::vector<double>> time_series_all;
    std::vector<std::map<std::string, std::map<std::string, std::vector<double>>>> data_all;
    //A vector of all the runs. A map of all nodes and their data. A map of all periodic_variables and their data series.
    //Runs.Nodes.Variables.Data

    //For each run (one run per start time)
    std::cout << "Synching data..." << std::endl;
    for (int i = 0; i < start_times.size(); ++i) {
        // std::cout << "Run " << i+1 << std::endl;
        std::cout << "\t" << (i+1) * 100 / start_times.size() << "%" << " synched"<< std::endl;
        std::vector<double> time_series_run_i;
        std::map<std::string, std::map<std::string, std::vector<double>>> data_run_i;

        //Generate a time series between start (rounded down to nearest delta_t) and end (rounded up to nearest delta_t)
        double start_time = delta_t * std::floor(start_times[i] / delta_t);
        double end_time = delta_t * std::ceil(end_times[i] / delta_t);
        for (double t = start_time; t <= end_time; t += delta_t) {
            time_series_run_i.push_back(t);
        }

        // std::cout << std::endl;


        //For each node...
        for (const auto& name : names) {
            // std::cout << "Node " << name << std::endl;

            std::map<std::string, std::vector<std::pair<double, double>>> node_data_raw;
            //A map of all variables and their data series (time, value)

            std::string filename = "processed_logs/" + name + "/" + name + "_" + std::to_string(i+1) + ".alog";
            // std::cout << "Reading data from " << filename << std::endl;

            std::ifstream file(filename);
            if (file.is_open()) {
                std::string line;

                double start_time;
                //First 4 lines are headers, extract start_time from the 4th line
                for (int i = 0; i < 4; ++i) {
                    std::getline(file, line);
                    //%% LOGSTART           1724342159.847351
                    if (i == 3) {
                        std::istringstream iss(line);
                        std::string temp;
                        iss >> temp >> temp >> start_time;
                    }
                }
                double offset = start_time - shore_start_time;

                //Read the rest of the lines
                while (std::getline(file, line)) {
                    //Line on format: "Time        Variable_name                Origion   Value"
                    //For example: "50.33681        NAV_X                uSimMarineV22   22.18486"
                    std::istringstream iss(line);
                    double time;
                    std::string variable_name;
                    std::string origin;
                    double d_value;
                    std::string s_value;

                    // Parse the line
                    if (iss >> time >> variable_name >> origin >> d_value) {
                        time += offset;
                        node_data_raw[variable_name].push_back({time, d_value});
                    } 
                    else {
                        // Clear the stream before reading the string value
                        iss.clear();
                        iss.str(line);
                        if (iss >> time >> variable_name >> origin >> s_value) {
                            if (variable_name == "DUBIN_UPDATE"){
                                if (s_value == "regenerate_path=true"){
                                    time += offset;
                                    double snap_time = delta_t * std::round(time / delta_t);
                                    data_run_i[name]["DUBIN_UPDATE_PATH_REGEN"].push_back(snap_time);
                                }
                                else if (s_value.find("goal_point=") != std::string::npos) {
                                    double x = 0.0, y = 0.0;
                                    sscanf(s_value.c_str(), "goal_point=x=%lf,y=%lf", &x, &y);
                                    data_run_i[name]["DUBIN_UPDATE_GOAL_X"].push_back(x);
                                    data_run_i[name]["DUBIN_UPDATE_GOAL_Y"].push_back(y);
                                }
                                else if (s_value.find("goal_heading=") != std::string::npos) {
                                    double heading = 0.0;
                                    sscanf(s_value.c_str(), "goal_heading=%lf", &heading);
                                    data_run_i[name]["DUBIN_UPDATE_GOAL_HEADING"].push_back(heading);
                                }
                            } else if (variable_name == "DEADLOCKED"){
                                time += offset;
                                double snap_time = delta_t * std::round(time / delta_t);
                                data_run_i[name]["DEADLOCKED"].push_back(snap_time);
                            }
                        } else {
                            std::cerr << "Error parsing line: " << line << std::endl;
                        }
                    }
                }
                file.close();
            } else {
                // std::cerr << "Unable to open file" << std::endl;
            }

            //Interpolate the data to the time series
            for (const auto& variable : node_data_raw) {
                std::string variable_name = variable.first;
                // std::cout << "Variable: " << variable_name << std::endl;
                for (const auto& time : time_series_run_i) {
                    double value = findClosest(variable.second, time);
                    data_run_i[name][variable_name].push_back(value);
                }
            }
        }

        time_series_all.push_back(time_series_run_i);
        data_all.push_back(data_run_i);
    }


/*****************************************************************/
/* 2. Output data as JSON for further processing in Matlab       */
/*****************************************************************/

    std::string filepath = folder + "/" + json_name + ".json";
    save_to_json(filepath, time_series_all, data_all, mission_names);

    std::ifstream src(filepath, std::ios::binary);
    std::ofstream dst("/Users/filipts/Documents/Thesis/Demustering-MATLAB/Data/Synched_data/" + json_name + ".json", std::ios::binary);
    dst << src.rdbuf();

    return 0;
}

// Function to find the closest value using binary search
double findClosest(const std::vector<std::pair<double, double>>& irregular_series, double regular_time) {
    auto lower = std::lower_bound(irregular_series.begin(), irregular_series.end(), std::make_pair(regular_time, 0.0));

    if (lower == irregular_series.end()) {
        // If all timestamps are less than regular_time, return the last value
        return irregular_series.back().second;
    }

    if (lower == irregular_series.begin()) {
        // If all timestamps are greater than regular_time, return the first value
        return lower->second;
    }

    auto prev = std::prev(lower);

    // Compare which timestamp is closer to the regular_time
    if (regular_time - prev->first <= lower->first - regular_time) {
        return prev->second;
    } else {
        return lower->second;
    }
}

void save_to_json(const std::string& filepath, std::vector<std::vector<double>>& time_series_all, std::vector<std::map<std::string, std::map<std::string, std::vector<double>>>>& data_all, std::vector<std::string>& mission_names_all) {
    std::ofstream file(filepath);

    //Start the JSON object
    file << "{\n";

    //Add the mission names
    file << "\"mission_names\": [\n    ";
    for (int i = 0; i < mission_names_all.size(); ++i) {
        file << "\"" << mission_names_all[i] << "\"";
        if (i < mission_names_all.size() - 1) file << ",";
        // file << "\n";
    }
    file << "\n  ],\n";

    //Add the time series
    file << "\"time_series\": [\n";
    for (int i = 0; i < time_series_all.size(); ++i) {
        file << "    [";
        for (int j = 0; j < time_series_all[i].size(); ++j) {
            file << std::fixed << std::setprecision(2) << time_series_all[i][j];
            if (j < time_series_all[i].size() - 1) file << ", ";
        }
        file << "]";
        if (i < time_series_all.size() - 1) file << ",";
        file << "\n";
    }
    file << "  ],\n";

    //Add the data
    file << "  \"data_all\": [\n    ";
    for (int i = 0; i < data_all.size(); ++i) {
        //For each run...
        // file << "    {\n";
        file << "{\n";
        for (const auto& node : data_all[i]) {
            //...For each node..
            std::string node_name = node.first;
            file << "      \"" << node_name << "\": {\n";
            for (const auto& variable : node.second) {
                //...For each variable...
                std::string variable_name = variable.first;
                std::vector<double> values = variable.second;
                file << "        \"" << variable_name << "\": [";
                for (int j = 0; j < values.size(); ++j) {
                    //...For each value:
                    file << values[j];
                    if (j < values.size() - 1) file << ", ";
                }
                file << "]";
                if (variable_name != node.second.rbegin()->first) file << ",";
                file << "\n";
            }
            file << "      }";
            if (node_name != data_all[i].rbegin()->first) file << ",";
            file << "\n";
        }
        file << "    }";
        if (i < data_all.size() - 1) file << ",";
        // file << "\n";
    }
    file << "  ]\n";

    // End the JSON object
    file << "}\n";
    file.close();
    return;
}
