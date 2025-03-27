#include "FormationGeometry.hpp"
#include <iostream>
#include <fstream>
#include <vector>
#include <map>

void writeToFile(const std::string &filename, const std::vector<std::string> &data) {
    std::ofstream file(filename);
    if (file.is_open()) {
        for (const auto &line : data) {
            file << line << "\n";
        }
        file.close();
    } else {
        std::cerr << "Unable to open file " << filename << std::endl;
    }
}

int main() {
    std::vector<uint16_t> num_agents = {1, 2, 3, 4, 5, 8, 9, 12, 15, 20, 50};
    std::vector<std::string> formation_types = {"vee", "echelon", "ncolumn", "wedge", "circle"};
    
    std::map<std::string, std::map<std::string, std::string>> formation_arguments = {
        {"vee", {{"vee_angle", "45"}, {"min_spacing", "5"}}},
        {"echelon", {{"echelon_angle", "45"}, {"min_spacing", "5"}, {"echelon_side", "left"}, {"fingered", "true"}}},
        {"ncolumn", {{"num_cols", "5"}, {"col_spacing", "5"}, {"row_spacing", "5"}}},
        {"wedge", {{"horiz_separation", "5"}, {"vert_separation", "5"}}},
        {"circle", {{"min_spacing", "5"}, {"leader_centered", "true"}}}
    };

    FormationGeometry formationGeometry;

    for (const auto &formation_type : formation_types) {
        std::vector<std::string> results;
        for (const auto &num_vehicle : num_agents) {
            formationGeometry.generateFormationGeometry(formation_type, num_vehicle, formation_arguments[formation_type]);
            std::string line = std::to_string(num_vehicle) + " - " + formationGeometry.repr() + " - " + formationGeometry.getCentroid().repr();
            results.push_back(line);
        }
        std::string filename = formation_type + ".txt";
        writeToFile(filename, results);
    }

    return 0;
}