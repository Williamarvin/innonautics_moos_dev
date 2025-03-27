#pragma once

/*
    Herein this file, we define a library for managing geometry of parallel formation structures for autonomous vehicles.
    There is an emphasis on leader-centric formation control, since it is the most general for heterogeneous groups of vehicles,
    but we shall consider other means of defining a formation as a whole with additional structures.

    TODO:
     - [ ] Implement the arrowhead formation
     - [ ] Handle error checking
     - [ ] Consider different default center types
     - [ ] Consider adding a swirl option for the circle formation
     - [ ] Standardize the formation argument names
*/
#include <string>
#include <vector>
#include <map>
#include <set>
#include "MBUtils.h"
#include <cmath>
#include "CtrlPointQueue.h"
#include <cstdarg>
#include <cstdio>

#define RAD_TO_DEG 57.2957795f

class FormationGeometry
{

public:
    FormationGeometry(){
        m_debug = false;
    };

    ~FormationGeometry(){

    };

    double cart2comp(double angle_deg)
    {
        return fmod(angle_deg - 450.0, 360.0);
    }

    double comp2cart(double angle_deg)
    {
        return fmod(450.0 + angle_deg, 360.0);
    }

    void setDebugFileName(const std::string& file_name) {
        m_debug_file_name = file_name;
        m_debug = true;
    }

    bool dbg_print(const char *format, ...) {
        if (m_debug) {
            va_list args;
            va_start(args, format);
            FILE *cfile = fopen(m_debug_file_name.c_str(), "a");
            if (cfile != nullptr) {
                vfprintf(cfile, format, args);
                fclose(cfile);
                va_end(args);
                return true;
            }
            va_end(args);
        }
        return false;
    }

    void resetFormation()
    {
        if (m_formation_set == true)
        {
            m_centroid = CtrlPoint(-1, -1);
            m_idx_to_point_mapping.clear();
            m_formation_set = false;
        }
    }

    bool isSupportedFormation(std::string formation)
    {
        return m_supported_formations.count(formation) > 0 ? true : false;
    }

    CtrlPoint getCentroid()
    {
        double xx = 0;
        double yy = 0;
        double entries = m_idx_to_point_mapping.size();
        for (const auto &entry : m_idx_to_point_mapping)
        {
            int idx = entry.first;
            CtrlPoint p = entry.second;
            xx += p.x;
            yy += p.y;
        }
        return CtrlPoint(xx / entries, yy / entries);
    }

    std::map<std::string, std::string> getDefaultArguments(std::string formation_type)
    {
        return m_formation_specific_default_arguments[formation_type];
    }

    std::string repr()
    {
        //[{idx:(x1,y1)},{idx+1:(x2,y2)}]
        std::string result = "[";
        char buffer[32];
        for (const auto &entry : m_idx_to_point_mapping)
        {
            int idx = entry.first;
            CtrlPoint p = entry.second;
            snprintf(buffer, sizeof(buffer), "{%d: %s},", idx, p.repr().c_str());
            result += std::string(buffer);
        }
        result.pop_back();
        result += "]";
        return result;
    }

    /*
        Given a formation type, the center type, and the center option, we populate the idx to point mapping table
    */
    void generateFormationGeometry(std::string formation_type, uint16_t num_vehicles, std::map<std::string, std::string> formation_arguments)
    {
        std::string formation_choice = tolower(formation_type);
        if (m_supported_formations.count(formation_choice) == 0)
        {
            // Handle Error
            return;
        }

        if (formation_choice == "vee")
        {
            // todo: check to make sure the formation arguments for this formation is relevant
            setVee(num_vehicles, formation_arguments);
        }
        else if (formation_choice == "echelon")
        {
            setEchelon(num_vehicles, formation_arguments);
        }
        else if (formation_choice == "ncolumn")
        {
            setNcolumn(num_vehicles, formation_arguments);
        }
        else if (formation_choice == "wedge")
        {
            setWedge(num_vehicles, formation_arguments);
        }
        else if (formation_choice == "circle")
        {
            setCircle(num_vehicles, formation_arguments);
        }
        else if (formation_choice == "arrowhead")
        {
        }
        else if (formation_choice == "diamond")
        {
            setDiamond(num_vehicles, formation_arguments);
        }
        else
        {
            // Bypassed error checking and illegal argument was provided
        }
    };

    void setVee(uint16_t num_vehicles, std::map<std::string, std::string> formation_arguments)
    {
        /*
            The Vee formation is a simple inverted V in the direction of the fleet,
            where the leader is at the tip. Here, for some number of vehicles, we
            define the ordering relative to the leader, which is at (0,0) in the
            formation frame.
        */
        // Clear the existing formation structure
        resetFormation();

        double vee_angle = stod(formation_arguments["vee_angle"])*M_PI/180.0f;
        double separation_dist = stod(formation_arguments["min_spacing"]);
        bool even_structure = false;

        if (num_vehicles % 2 == 0)
        {
            even_structure = true;
        }

        int row = 1;
        int agent = 1;
        int placed_vehicles = even_structure ? num_vehicles - 1 : num_vehicles;

        m_idx_to_point_mapping[0] = CtrlPoint(0, 0);
        // For an odd number of vehicles, place them along the two strips. If we have an even structure, we place the odd agent behind the lead agent in the first row
        for (int i = 1; i < placed_vehicles; ++i)
        {
            // 0 is the leader, all even idcs are on the right, all odd on the left. If an even Vee, then the last index is the agent behind the leader in the third 'row'
            row = ceil(static_cast<double>(i) / 2.0f);
            if (i % 2 == 0)
            {
                // To the right
                m_idx_to_point_mapping[i] = CtrlPoint(separation_dist * row * cos(vee_angle), -separation_dist * row * sin(vee_angle));
            }
            else
            {
                // To the left
                m_idx_to_point_mapping[i] = CtrlPoint(-separation_dist * row * cos(vee_angle), -separation_dist * row * sin(vee_angle));
            }
        }

        if (even_structure)
        {
            // places the last vehicle in the second row, but behind the leader
            m_idx_to_point_mapping[num_vehicles - 1] = CtrlPoint(0, -2.0 * separation_dist * sin(vee_angle));
        }
        m_formation_set = true;
        return;
    }

    void setEchelon(uint16_t num_vehicles, std::map<std::string, std::string> formation_arguments)
    {
        /*
            The Echelon formation is basically just a diagonal line of vehicles, defined by a formation angle,
            a separation distance, the side the vehicle lines up on, and then whether or not the formation
            should be "fingered".
        */
        // Clear the existing formation structure
        resetFormation();

        // Begin to populate the idx to point mapping (i.e. build this structure on this vehicle)
        // 0) leader - for the echelon, the leader is always in the front
        m_idx_to_point_mapping[0] = CtrlPoint(0, 0);
        double echelon_angle = stod(formation_arguments["echelon_angle"]) / RAD_TO_DEG;
        double separation_dist = stod(formation_arguments["min_spacing"]);
        bool echelon_on_right = tolower(formation_arguments["echelon_side"]) == "left" ? false : true; // could include better error checking - default is on the right hand side
        bool fingered = tolower(formation_arguments["fingered"]) == "true" ? true : false;
        int row = 1;
        int n_agents_downstream = num_vehicles;
        double side = echelon_on_right ? 1 : -1; // If to the right, places them to the right in x, else to the left in x
        if (fingered)
        {
            // Places the last agent next to the lead vehicle on the opposite side of the main echelon
            n_agents_downstream--;
            m_idx_to_point_mapping[num_vehicles - 1] = CtrlPoint(-side * separation_dist * cos(echelon_angle), -separation_dist * sin(echelon_angle));
        }
        for (int i = 1; i < n_agents_downstream; ++i)
        {
            m_idx_to_point_mapping[i] = CtrlPoint(side * separation_dist * i * cos(echelon_angle), -separation_dist * i * sin(echelon_angle));
        }
        m_formation_set = true;
        return;
    }

    void setNcolumn(uint16_t num_vehicles, std::map<std::string, std::string> formation_arguments)
    {
        // Clear the existing formation structure
        resetFormation();

        int num_cols = stoi(formation_arguments["num_cols"]);
        double col_spacing = stod(formation_arguments["col_spacing"]);
        double row_spacing = stod(formation_arguments["row_spacing"]);
        double row = 0;
        uint16_t entry = 0;

        m_idx_to_point_mapping[0] = CtrlPoint(0, 0);
        for (int i = 1; i < num_vehicles; i++)
        {
            // Every row is defined by categorized entries, where odd numbers for an entry in a column are on the left, even numbers are on the right
            row = i / num_cols;
            entry = (i % num_cols);
            double side_sep = ceil(static_cast<double>(entry) / ceil(static_cast<double>(num_cols) / 2.0));
            if (entry % 2 == 0)
            {
                m_idx_to_point_mapping[i] = CtrlPoint(col_spacing * side_sep, -row_spacing * row);
            }
            else
            {
                m_idx_to_point_mapping[i] = CtrlPoint(-col_spacing * side_sep, -row_spacing * row);
            }
        }
        m_formation_set = true;
        return;
    }

    void setWedge(uint16_t num_vehicles, std::map<std::string, std::string> formation_arguments)
    {
        /*
            Here we allocate a staggered wedge formation. i.e. starting from row 1, the number of agents in each row corresponds to the row number, and their alignment is such that it is always symmetric
        */
        // Clear the existing formation structure
        resetFormation();
        uint16_t row = 1;
        uint16_t agent = 0;

        double l_x = stod(formation_arguments["horiz_separation"]);
        double l_y = stod(formation_arguments["vert_separation"]);

        double side = 1;

        // Row zero is the first row, with a single agent
        uint16_t agents_in_row = 0;
        while (agent < num_vehicles)
        {
            if (agents_in_row >= row)
            {
                row++;
                agents_in_row = 0;
            }

            if (row % 2 == 0)
            {
                // For an even row, we want an offset symmetry, where the first and second agent both have the same half-length offset in opposite directions (i.e. entry 0 and 1 have the same offset)
                // i.e. 0 -> 0, 1 -> 0, 2 -> 1, 3 -> 1, ...
                double offset_idx = floor(static_cast<double>(agents_in_row) / 2.0);
                double x = (l_x * 0.5 + l_x * offset_idx) * side;
                double y = -l_y * (row - 1);
                m_idx_to_point_mapping[agent] = CtrlPoint(x, y);
            }
            else
            {
                // For an odd row, we want the first placed agent in the center, and then all the others to switch on both sides, (i.e. 0 with no offset, 1 and 2 with the same offset, etc..)
                // i.e. 0 -> 0, 1 -> 1, 2 -> 1
                double offset_idx = floor(static_cast<double>(agents_in_row + 1.0) / 2.0);
                double x = l_x * offset_idx * side;
                double y = -l_y * (row - 1);
                m_idx_to_point_mapping[agent] = CtrlPoint(x, y);
            }
            agent++;
            agents_in_row++;
            if (agents_in_row > 0)
                side *= -1.0;
        }
        m_formation_set = true;
        return;
    }

    void setCircle(uint16_t num_vehicles, std::map<std::string, std::string> formation_arguments)
    {
        /*
            Here we define the diamond formation. It is essentially a rotated set of columns
        */
        // Clear the existing formation structure
        resetFormation();
        bool leader_center = tolower(formation_arguments["leader_centered"]) == "true" ? true : false;

        if (num_vehicles == 1)
        {
            dbg_print("Single Agent Circle\n");
            m_idx_to_point_mapping[0] = CtrlPoint(0, 0);
        }
        else if (leader_center)
        {
            dbg_print("Leader Centered Circle\n");
            double sep_distance = stod(formation_arguments["min_spacing"]);
            double alpha = 2 * M_PI / ((num_vehicles - 1) * 1.0f);
            double radius;
            dbg_print("Sep: %f, Alpha: %f\n", sep_distance, alpha);

            bool even = (num_vehicles - 1) % 2 == 0 ? true : false;

            if (num_vehicles == 2)
            {
                radius = sep_distance;
            }
            else
            {
                radius = abs(sep_distance / (2 * sin(alpha / 2.0)));
            }
            dbg_print("Radius: %f\n", radius);

            // Add the lead agent, and the front most agent
            m_idx_to_point_mapping[0] = CtrlPoint(0, 0);

            m_idx_to_point_mapping[1] = CtrlPoint(0, radius);

            // Add mirrored sides simultaneously
            int idx = 1;
            for (int i = 2; i < (num_vehicles - 1) && !((num_vehicles - 1) <= 2); i += 2)
            {
                double x = radius * cos(alpha * idx + M_PI_2);
                double y = radius * sin(alpha * idx + M_PI_2);
                // left
                m_idx_to_point_mapping[i] = CtrlPoint(x, y);
                // right
                m_idx_to_point_mapping[i + 1] = CtrlPoint(-x, y);
                idx++;
            }

            if (even)
            {
                m_idx_to_point_mapping[num_vehicles - 1] = CtrlPoint(0, -radius);
            }
        }
        else
        {
            dbg_print("Leader Front Circle\n");
            double sep_distance = stod(formation_arguments["min_spacing"]);

            double alpha = 2 * M_PI / (num_vehicles * 1.0f);
            double radius;
            if (num_vehicles == 2)
            {
                radius = sep_distance;
            }
            else
            {
                radius = abs(sep_distance / (2 * sin(alpha / 2.0)));
            }
            // Add the lead agent
            m_idx_to_point_mapping[0] = CtrlPoint(0, 0);

            // If we have an even number of vehicles, place the last at the bottom in advance since we force symmetry
            bool even = num_vehicles % 2 == 0 ? true : false;
            if (even)
            {
                m_idx_to_point_mapping[num_vehicles - 1] = CtrlPoint(0, -2 * radius);
            }

            // Add mirrored sides simultaneously
            int idx = 1;
            for (int i = 1; i < num_vehicles && !(num_vehicles <= 2); i += 2)
            {
                double x = radius * cos(alpha * idx + M_PI_2);
                double y = -radius + radius * sin(alpha * idx + M_PI_2);
                // left
                m_idx_to_point_mapping[i] = CtrlPoint(x, y);
                // right
                m_idx_to_point_mapping[i + 1] = CtrlPoint(-x, y);
                idx++;
            }
        }

        m_formation_set = true;
        return;
    }

    void setDiamond(uint16_t num_vehicles, std::map<std::string, std::string> formation_arguments)
    {
        /*
            Here we define the diamond formation. It is essentially a rotated set of columns
            //TODO: this
        */
        // Clear the existing formation structure
        resetFormation();

        m_formation_set = true;
        return;
    }

    /*
        Given the leader point and its heading, we get the desired projected point for a current vehicle in a formation
    */
    CtrlPoint getPoint(CtrlPoint leader_point, double leader_heading, uint16_t idx)
    {
        CtrlPoint relative_point = m_idx_to_point_mapping[idx];
        double leader_angle_cart_rad = -(comp2cart(leader_heading) * M_PI / 180.0f - M_PI_2); //Visually, formations are constructed top down
        double x_rel_rot = relative_point.x * cos(leader_angle_cart_rad) - relative_point.y * sin(leader_angle_cart_rad);
        double y_rel_rot = relative_point.x * sin(leader_angle_cart_rad) + relative_point.y * cos(leader_angle_cart_rad);

        // Create a projected point which preserves the leader's other state information
        CtrlPoint projected_point = CtrlPoint(leader_point);
        projected_point.x+=x_rel_rot;
        projected_point.y+=y_rel_rot;
        return projected_point;
    };

public:
    std::string m_formation_type;
    std::map<uint16_t, CtrlPoint> m_idx_to_point_mapping;
    bool m_formation_set = false;
    CtrlPoint m_centroid = CtrlPoint(0, 0);
    bool m_debug;
    std::string m_debug_file_name;

    std::set<std::string> m_supported_formations =
        {
            // Formation Specific Arguments
            "vee", 
            "echelon", 
            "ncolumn", 
            "wedge", 
            "circle", 
    };

    std::map<std::string, std::set<std::string>> m_formation_specific_allowable_arguments = {
        {"vee", {"vee_angle", "min_spacing"}},
        {"echelon", {"echelon_side", "echelon_angle", "fingered", "min_spacing"}},
        {"ncolumn", {"col_spacing", "row_spacing", "num_cols", "leader_position"}},
        {"wedge", {"row_spacing", "min_spacing", "leader_position"}},
        {"arrowhead", {"angle", "min_spacing"}},
        {"diamond", {"angle", "min_spacing"}},
        {"circle", {"min_spacing", "leader_centered"}}};

    /*
        TODO: Allow inverted vee and echelon
    */
    std::map<std::string, std::map<std::string, std::string>> m_formation_specific_default_arguments = {
        {"vee", {{"vee_angle", "45"}, {"min_spacing", "5"}}},
        {"echelon", {{"echelon_side", "right"}, {"echelon_angle", "45"}, {"fingered", "false"}, {"min_spacing", "5"}}},
        {"ncolumn", {{"col_spacing", "5"}, {"row_spacing", "5"}, {"num_cols", "3"}, {"leader_position", "front"}}},
        {"wedge", {{"horiz_separation", "5"}, {"vert_separation", "5"}, {"leader_position", "front"}}},
        {"arrowhead", {{"angle", "45"}, {"min_spacing", "5"}}},
        {"diamond", {{"angle", "45"}, {"min_spacing", "5"}}},
        {"circle", {{"min_spacing", "5"}, {"leader_centered", "false"}}}};
};
