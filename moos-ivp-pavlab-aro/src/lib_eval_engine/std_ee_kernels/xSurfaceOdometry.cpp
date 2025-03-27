
#include "xSurfaceOdometry.h"
#include <cmath>
using namespace std;

//---------------------------------------------------------
// EvalEngine Standard Kernel: xSurfaceOdometryKernel
/*
    Tracks the euclidian distance traveled between posted updates
*/

// TODO: Have it draw a seglist between all the sample points in pMarineViewer
xSurfaceOdometryKernel::xSurfaceOdometryKernel(std::string identity, std::string suffix)
{
    // Must provide an identity for the kernel
    m_identity = identity;

    // state variables and initial values
    m_first_reading = false;
    m_x = 0;
    m_y = 0;
    m_x_prv = 0;
    m_y_prv = 0;
    m_total_distance = 0;
    m_num_sample_points = 0;
    m_display_sample_points = "true";

    // Parameter defaults
    m_conversion_factor = 1.0;
    m_units = "meters";

    // Subscriptions - can be re-mapped as parameters
    m_nav_x_var_key = "input_x_var"+suffix;
    m_nav_y_var_key = "input_y_var"+suffix;

    m_nav_x_key = "NAV_X";
    m_nav_y_key = "NAV_Y";

    // Parameter names
    m_identity_key = "identity"+suffix;
    m_display_sample_points_key = "display_sample_points"+suffix;
    m_conversion_factor_key = "conversion_factor"+suffix;
    m_units_key = "units"+suffix;

    // Publications - can be remapped as a parameter
    m_eval_odometry_var_key = "output_var"+suffix;
    m_eval_odometry_key = "ODOMETRY";

    m_sample_points_output_key = "seglist_output"+suffix;
    m_sample_points_output = "VIEW_SEGLIST";

    m_sample_points_label_key = "sample_points_seglist_label"+suffix;
    m_sample_points_label = "odometry_sample_points";

    // Include the subscriptions which describe the needs of this kernel to the EvalEngine
    m_subscriptions.insert(m_nav_x_key);
    m_subscriptions.insert(m_nav_y_key);

    // Include the publications which describe the needs of this kernel to the EvalEngine
    m_publications.insert(m_sample_points_label);
    m_publications.insert(m_nav_y_key);

    // Include the parameters which describe the needs of this kernel to the EvalEngine, and their default values

    // The identity can be renamed, include it as parameter options with default values
    m_parameters[m_identity_key] = m_identity;

    // The variables which we listen to can be remapped, include these as parameter options with default values
    m_parameters[m_nav_x_var_key] = m_nav_x_key;
    m_parameters[m_nav_y_var_key] = m_nav_y_key;

    m_parameters[m_conversion_factor_key] = to_string(m_conversion_factor);
    m_parameters[m_units_key] = m_units;

    m_parameters[m_eval_odometry_var_key] = m_eval_odometry_key;

    m_parameters[m_sample_points_label_key] = m_sample_points_label;

    m_parameters[m_sample_points_output_key] = m_sample_points_output;

    m_parameters[m_display_sample_points_key] = m_display_sample_points;
};

bool xSurfaceOdometryKernel::hasMoved() {
    //TODO: This threshold should be a parameter if we are only working in long/lat instead of x/y. This is temporary for now. 
    return ((abs(m_x - m_x_prv) > 0.00001) || (abs(m_y - m_y_prv) > 0.00001));
}

// Sets the parameters that has been received from a file
bool xSurfaceOdometryKernel::SetParams(const map<string, string> &params)
{
    // TODO: Provide an example on error checking here
    // Renaming identity
    if (params.count(m_identity_key))
        m_identity = params.at(m_identity_key);

    // Remapping inputs
    if (params.count(m_nav_x_var_key))
    {
        m_subscriptions.erase(m_nav_x_key);
        m_nav_x_key = params.at(m_nav_x_var_key);
        m_subscriptions.insert(m_nav_x_key);
    }

    if (params.count(m_nav_y_var_key))
    {
        m_subscriptions.erase(m_nav_y_key);
        m_nav_y_key = params.at(m_nav_y_var_key);
        m_subscriptions.insert(m_nav_y_key);
    }

    // Assigning values to parameters
    if (params.count(m_conversion_factor_key))
        m_conversion_factor = stod((params.at(m_conversion_factor_key)));

    if (params.count(m_units_key))
        m_units = params.at(m_units_key);

    if (params.count(m_display_sample_points_key))
    {
        m_display_sample_points = params.at(m_display_sample_points_key);
    }

    if (params.count(m_sample_points_label_key))
        m_sample_points_label = params.at(m_sample_points_label_key);

    // Remapping outputs
    if (params.count(m_eval_odometry_var_key))
        m_eval_odometry_key = params.at(m_eval_odometry_var_key);

    return true;
}

// From the message buffer which is
bool xSurfaceOdometryKernel::UpdateVariable(const EEMessage &message)
{
    //Handle Mail
    bool captured_update = false;
    if (message.getVarName() == m_nav_x_key)
    {
        double time_written = message.time_stamp();
        m_x = stod(message.msg());
        captured_update = true;
    } else if (message.getVarName() == m_nav_y_key)
    {
        m_y = stod(message.msg());
        captured_update = true;
    } else {
        //Unhandled mail which we subscribed to
        return captured_update;
    }

    //Update states unique to handling mail
    if (m_first_reading == false &&
        ((m_x != m_x_prv) || (m_y != m_y_prv)))
    {
        //The actual first reading just means that it is seeded with an intial condition - we should not
        //consider the distance between the arbitrary initial *_prv variables and the location we just received
        m_x_prv = m_x;
        m_y_prv = m_y;
        m_sample_points.push_back(to_string(m_x) + "," + to_string(m_y));
        m_first_reading = true;
    }
    return captured_update;
}

// On a pass over, it can post updates to an object maintained by the EvalEngine
bool xSurfaceOdometryKernel::Evaluate(std::vector<std::pair<std::string, std::string> > &updates, std::vector<std::string> &kernel_level_warnings)
{
    if (m_first_reading && hasMoved())
    {
        double delta_x = m_x - m_x_prv;
        double delta_y = m_y - m_y_prv;
        m_total_distance += sqrt(pow(delta_x, 2) + pow(delta_y, 2)) * m_conversion_factor;
        m_num_sample_points++;
        updates.push_back(std::make_pair(m_eval_odometry_key,to_string(m_total_distance)));
        //updates[m_eval_odometry_key] = to_string(m_total_distance);
        if (m_display_sample_points == "true")
        {
            m_sample_points.push_back(to_string(m_x) + "," + to_string(m_y));
            string seg_list_msg = "pts={";
            for (int i = 0; i < m_sample_points.size(); i++)
            {
                seg_list_msg += m_sample_points[i];
                if (i != m_sample_points.size() - 1)
                    seg_list_msg += ":";
            }
            seg_list_msg += ("},label=" + m_sample_points_label);
            updates.push_back(std::make_pair(m_sample_points_output,seg_list_msg));
            //updates[m_sample_points_output] = seg_list_msg;
        }

        m_x_prv = m_x;
        m_y_prv = m_y;
    }
    return true;
}

// A kernel can make a contribution to the EvalEngine's AppCasting table
bool xSurfaceOdometryKernel::AppCastingNotice(ACTable &actab)
{
    const uint32_t msg_buff_len = 128;
    char message[msg_buff_len];
    memset(message, msg_buff_len, '\0');
    snprintf(message, msg_buff_len, "Total Distance: %0.2f %s\n", m_total_distance, m_units.c_str());
    actab << m_identity << message;
    return true;
}