
#pragma once
#include "EvalEngine.h"
//#include <cstdio>
//#include <string.h>
#include <string>
#include <vector>

class xSurfaceOdometryKernel : public EvalEngineKernel
{
private:

    // State variables and default values
    std::string m_identity_key;
    bool m_first_reading;
    double m_x;
    double m_y;
    double m_x_prv;
    double m_y_prv;
    double m_total_distance;
    uint64_t m_num_sample_points;
    std::vector<std::string> m_sample_points;

    // Parameter defaults
    double m_conversion_factor;
    std::string m_units;

    // Subscriptions
    std::string m_nav_x_var_key;
    std::string m_nav_y_var_key;

    std::string m_nav_x_key;
    std::string m_nav_y_key;

    // Parameters
    std::string m_display_sample_points_key;
    std::string m_display_sample_points;
    std::string m_conversion_factor_key;
    std::string m_sample_points_output;
    std::string m_units_key;

    // Publications
    std::string m_eval_odometry_var_key;
    std::string m_eval_odometry_key;
    std::string m_sample_points_output_key;
    std::string m_sample_points_label_key;
    std::string m_sample_points_label;

    //Local utility functions
    bool hasMoved();
    
public:
    //Sets the subscriptions and the parameters that it will consume from the configuration file.
    //The EvalEngine core will reference these variables
    xSurfaceOdometryKernel(std::string identity = "xSurfaceOdometry", std::string suffix = "");

    ~xSurfaceOdometryKernel();

    //Sets the parameters that has been received from a file
    bool SetParams(const std::map<std::string, std::string> &params);

    //From the message buffer which is 
    bool UpdateVariable(const EEMessage &message);

    //On a pass over, it can post updates to an object maintained by the EvalEngine
    bool Evaluate(std::vector<std::pair<std::string, std::string> > &updates, std::vector<std::string> &kernel_level_warnings);

    //A kernel can make a contribution to the EvalEngine's AppCasting table
    bool AppCastingNotice(ACTable &actab);
};