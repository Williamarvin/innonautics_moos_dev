#pragma once

#include <cmath>
#include <vector>
#include <string>
#include <cstring>

class Lemniscate
{
public:
    double m_width = 0;
    double m_height = 0;
    uint32_t m_n_points = 100;
    double m_center_x = 0;
    double m_center_y = 0;
    double m_alpha_deg = 0;
    double m_alpha_rad = 0;
    const double m_vscale = 1.0 / 0.3535527625463974;
    double m_direction = 1;

    Lemniscate()
    {
    }

    bool getPoint(double &x, double &y, double center_x, double center_y, double width, double height, double alpha_deg, int direction, double percent)
    {
        // ts = np.linspace(0,2*np.pi,npoints)+np.pi;
        double t = (M_PI * 2.0 * percent) * direction + direction * M_PI / 2.0;
        double alpha_rad = alpha_deg * M_PI / 180.0;
        double x_0 = (width * cos(t) / (1 + pow(sin(t), 2.0)));
        double y_0 = -(m_vscale * height * cos(t) * sin(t)) / (1 + pow(sin(t), 2.0));
        x = center_x + x_0 * cos(alpha_rad) - y_0 * sin(alpha_rad);
        y = center_y + x_0 * sin(alpha_rad) + y_0 * cos(alpha_rad);
    }
};